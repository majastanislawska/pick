# klippy/extras/camera_v4l.py
# Native Klipper V4L2 Camera Streamer using Reactor FD Polling
#
# Copyright (C) 2026 Maja Stanislawska <maja@makershop.ie>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import subprocess
import signal
import fcntl
import mmap
import queue
import threading
import v4l2
import cv2
import re
import numpy
from http.server import BaseHTTPRequestHandler, HTTPServer
import socketserver
import datetime
import logging

# V4L2 (Video4Linux2) constants
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1

# Helper functions for camera lighting to make it independent from actual klipper driver running a light
# Camera light: LIGHT= is either brightness 0..1, hex RGB(W), or both.
# Hex in G-code/config must not use a bare '#" — that is a comment.
_HEX_COLOR_RE = re.compile(r'(?i)^#?(?:0x)?([0-9a-f]{3}|[0-9a-f]{4}|[0-9a-f]{6}|[0-9a-f]{8})$')
_FLOAT_RE = re.compile(r'^[+-]?(?:\d+(?:\.\d*)?|\.\d+)$')
def parse_hex_color(tok):
    """Parse RGB / RGBW hex → 3- or 4-tuple of 0..1 floats. None if not hex."""
    if tok is None:
        return None
    m = _HEX_COLOR_RE.match(str(tok).strip())
    if not m:
        return None
    h = m.group(1)
    if len(h) in (3, 4):
        return tuple(int(c * 2, 16) / 255. for c in h)
    return tuple(int(h[i:i+2], 16) / 255. for i in range(0, len(h), 2))

def parse_light(value):
    """Parse LIGHT= token(s) → (s_or_None, color_tuple_or_None).
    A float in 0..1 is brightness. A 3/4/6/8-digit hex string is RGB(W).
    if string starts with K rest is white temperature
    Both may appear, separated by comma/space: '0.8,F80' or 'FFF 0.5'.
    """
    if value is None or value == '':
        return None, None
    if isinstance(value, bool):
        raise ValueError("LIGHT: unexpected boolean")
    if isinstance(value, (int, float)):
        s = float(value)
        if s < 0. or s > 1.:
            raise ValueError("LIGHT brightness must be in 0..1 (got %s)" % (value,))
        return s, None
    s = color = None
    for tok in re.split(r'[\s,;]+', str(value).strip()):
        if not tok:
            continue
        if _FLOAT_RE.match(tok):
            val = float(tok)
            if val < 0. or val > 1.:
                raise ValueError("LIGHT brightness must be in 0..1 (got %s)" % (tok,))
            s = val
            continue
        if tok[0] in ['k','K']:
            c=kelvin_to_rgb(float(tok[1:]))
            if c is not None:
                color = c
                continue
        c = parse_hex_color(tok)
        if c is not None:
            color = c
            continue
        raise ValueError(
            "cannot parse LIGHT token %r (want 0..1 or hex RGB/RGBW or K<temp>)" % (tok,))
    return s, color

def format_hex_color(color):
    if not color:
        return None
    return ''.join('%02X' % int(c * 255. + .5) for c in color)

def channels_str(ch):
    return ''.join(c for i, c in enumerate('RGBW') if i in ch) or '-'

def probe_led_channels(led):
    """Which RGBW indices the LED object actually drives."""
    pins = getattr(led, 'pins', None)
    if pins:
        return frozenset(i for i, _pin in pins)
    cm = getattr(led, 'color_map', None)
    if not cm:
        return frozenset()
    ch = set()
    for item in cm:
        if isinstance(item, int):
            ch.add(item)          # pca9632: [R,G,B,W] indices
        else:
            ch.add(item[1][1])    # neopixel: (cdidx, (lidx, cidx))
    return frozenset(ch)

def map_rgb_to_led(color, s, ch):
    """Scale color by s and fold onto the channels the hardware has.
    RGB hex (3-tuple): on RGBW extract common white onto W.
    RGBW hex (4-tuple): respect the given W; if hardware has no W, fold
    it back into RGB. White-only PWM uses max(R,G,B,W)*s on W.
    """
    s = max(0., min(1., float(s)))
    r = g = b = 0.
    w_in = None
    if color:
        r, g, b = (max(0., min(1., float(c))) for c in color[:3])
        if len(color) > 3:
            w_in = max(0., min(1., float(color[3])))
    r, g, b = r * s, g * s, b * s
    w = 0. if w_in is None else w_in * s
    has_r, has_g, has_b, has_w = (i in ch for i in range(4))
    if has_w and not (has_r or has_g or has_b):
        return (0., 0., 0., max(r, g, b, w))
    if not has_w:
        return (
            min(1., r + w) if has_r else 0.,
            min(1., g + w) if has_g else 0.,
            min(1., b + w) if has_b else 0.,
            0.)
    if w_in is None:
        extra = min(c for c, h in ((r, has_r), (g, has_g), (b, has_b)) if h)
        r, g, b, w = (
            (r - extra) if has_r else 0.,
            (g - extra) if has_g else 0.,
            (b - extra) if has_b else 0.,
            extra)
    return (
        r if has_r else 0.,
        g if has_g else 0.,
        b if has_b else 0.,
        w if has_w else 0.)

def kelvin_to_rgb(kelvin: float):
    """
    Convert White light temperature (1000K - 6600K) to RGB HEX.
    based on http://www.tannerhelland.com/4435/convert-temperature-rgb-algorithm-code/
    """
    #can do to 40000K, fic clip and uncomment if/else
    #no point though as cameras can do whitebalance to 6500 max
    temp = numpy.clip(kelvin,1000.0,6600.0) / 100.0
    tt=temp - 60.0
    #if temp<=66.:
    r = 1.
    g = numpy.clip(0.390081578769 * numpy.log(temp) - 0.631841443788,0.,1.)
    b = 0. if temp <= 19.0 else numpy.clip(0.54320678911 * numpy.log(temp - 10.0) - 1.19625408914 , 0., 1.)
    # else:
        # r = nunpy.clip(1.292936186062 * ((temp - 60.0) ** -0.1332047592), 0., 1.)
        # g = nunpy.clip(1.129890860895 * ((temp - 60.0) ** -0.0755148492), 0., 1.)
        # b = 1
    return (r,g,b)

class ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, server_address, RequestHandlerClass):
        self.latest_jpeg = None
        self.condition = threading.Condition()
        super().__init__(server_address, RequestHandlerClass)

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != '/stream':
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                with self.server.condition:
                    self.server.condition.wait()
                    jpg = self.server.latest_jpeg
                if jpg:
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n'
                        b'Content-Length: %d\r\n\r\n' % len(jpg))
                    self.wfile.write(jpg)
                    self.wfile.write(b'\r\n')
        except Exception:
            pass  # client disconnect

    def log_message(self, format, *args):
        pass

class VisionWorker(threading.Thread):
    def __init__(self, parent):
        super().__init__(daemon=True)
        self.parent=parent
        self.reactor=self.parent.reactor
        self.frame_w = None
        self.frame_h = None
        self.fps=0.
        self.frame_counter=0
        self.frame_queue = queue.Queue(maxsize=2)
        self.snapshot_requests = queue.Queue(maxsize=10)
        self.overlay_lock = self.reactor.mutex()
        self.latest_snapshot = None
        self.frameoverlay = None
        self.overlayalpha=0.5
        self.cx = None
        self.cy = None
        self.map1, self.map2 = None,None

    def generate_maps(self, camera_matrix, dist_coeffs, rectification_matrix, virtual_camera_matrix):
        if (camera_matrix is None or dist_coeffs  is None or
            rectification_matrix is None or virtual_camera_matrix is None):
                self.map1, self.map2 = None,None
                raise Exception('clearing_maps')
        self.cx = virtual_camera_matrix[0, 2]
        self.cy = virtual_camera_matrix[1, 2]
        self.frame_w=int(virtual_camera_matrix[0, 0])
        self.frame_h=int(virtual_camera_matrix[1, 1])
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            camera_matrix, dist_coeffs, rectification_matrix, virtual_camera_matrix,
            (self.frame_w, self.frame_h), cv2.CV_16SC2 )

    def push_frame(self, msg):
        try: self.frame_queue.put_nowait(msg)
        except queue.Full: pass

    def get_snapshot(self):
        complete=self.reactor.completion()
        self.snapshot_requests.put(complete)
        return complete.wait()

    def set_overlay(self, overlay):
        with self.overlay_lock:
            self.frameoverlay=overlay

    def _publish_jpeg(self, jpg_bytes):
        server = self.parent.server
        if server is None:
            return
        with server.condition:
            server.latest_jpeg = jpg_bytes
            server.condition.notify_all()

    def hud(self,img,w,h,cx,cy):
        color=(0, 255, 255);thickness=2;size=50
        cv2.line(img, (cx - size, cy), (cx + size, cy), color, thickness)
        cv2.line(img, (cx, cy - size), (cx, cy + size), color, thickness)
        cv2.circle(img, (cx, cy), 25, color, 2)
        cv2.rectangle(img, (1, 1), (w-1, h-1), color, thickness)
        return img

    def run(self):
      try:
        prev_frame_time=0
        while True:
            (eventtime,raw_jpeg)= self.frame_queue.get()
            self.frame_counter += 1
            try: req = self.snapshot_requests.get_nowait()
            except queue.Empty: req = None
            if req is not None or (self.frame_counter % self.parent.fps_divider == 0):
                img = cv2.imdecode(numpy.frombuffer(raw_jpeg, numpy.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    continue
                if self.map1 is not None and self.map2 is not None:
                    img = cv2.remap(img, self.map1, self.map2, cv2.INTER_LINEAR)
                if req is not None:
                    req.complete((img.copy(),eventtime))
                    continue
                self.frame_h, self.frame_w = img.shape[:2]
                if self.parent.overlay_on and self.frameoverlay is not None:
                    with self.overlay_lock:
                        ov = self.frameoverlay
                        if ov is not None and ov.shape[:2] == img.shape[:2]:
                            if ov.ndim != img.ndim:
                                ov = (cv2.cvtColor(ov, cv2.COLOR_GRAY2BGR)
                                      if ov.ndim == 2 else ov)
                            if ov.shape == img.shape:
                                img=cv2.addWeighted(img, 1-self.overlayalpha, ov, self.overlayalpha, 0)
                        else:
                            self.frameoverlay = None
                if self.parent.hud_on:
                    self.hud(img, self.frame_w,self.frame_h,int(self.cx),int(self.cy))
                if self.parent.fps_on:
                    fps = "FPS:%.3f"%(1.0/(eventtime-prev_frame_time),)
                    cv2.putText(img, fps, (2, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (100, 255, 0), 1, cv2.LINE_AA)
                _, encoded_jpg = cv2.imencode('.jpg', img)
                self._publish_jpeg(encoded_jpg.tobytes())
                prev_frame_time = eventtime
      except Exception as e:
        logging.info(f"Error in VisionWorker.run {e}")

class V4L2Camera:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.device = config.get('device', '/dev/video0')
        self.resolution = config.get('resolution', '640x480')
        # looking: up = fixed bottom camera to look at nozzle; down = head-mounted topcam
        # flips image-Y sign in pixels_to_mm_offset (see there).
        self.looking_up = config.getchoice(
            'looking', {'up': True, 'down': False}, default='down')
        if self.looking_up:
            #Fixed physical on the frame coordinates of up-facing camera
            self.camera_x = config.getfloat('camera_x', None)
            self.camera_y = config.getfloat('camera_y', None)
            self.camera_z = config.getfloat('camera_z', None)
        self.httpaddr = config.get('http_addr', '0.0.0.0')
        self.httpport = config.getint('http_port', 8081)
        self.light_name = config.get('light', None)
        self.light = None
        self.light_channels = frozenset()
        self.light_s_default = config.getfloat('light_s', 1., minval=0., maxval=1.)
        self.light_idle = config.getfloat('light_idle', 0., minval=0., maxval=1.)
        self.light_settle = config.getfloat('light_settle', 0.1, minval=0.)
        self.light_hold = config.getfloat('light_hold', 5., minval=0.)
        color_cfg = config.get('light_color', 'FFF')
        try:
            _s, color = parse_light(color_cfg)
        except ValueError as e:
            raise config.error("[camera_v4l %s] light_color: %s" % (self.name, e))
        if color is None:
            raise config.error(
                "[camera_v4l %s] light_color must be hex RGB(W), got %r"
                % (self.name, color_cfg))
        self._light_color = color
        self._light_s = self.light_s_default
        self._output_s = 0.
        self._last_rgbw = None
        self._light_hold_timer = None
        self.settings={}
        try: self.controls = self._get_supported_v4l2_controls()
        except subprocess.CalledProcessError as e:
            raise config.error(f"[camera_v4l {self.name}] Error: {e.returncode}: {str(e.stderr)}")
        # logging.info(f"[camera_v4l {self.name}]: Detected V4L2 controls: {self.controls}")
        for ctrl_name, (value, ctrl_type_info, details) in self.controls.items():
            # logging.info(f"[camera_v4l {self.name}]: Detected V4L2 control: {ctrl_name} = {value}  #({ctrl_type_info}) {details}")
            val = config.get(ctrl_name, None)
            if val is not None:
                self.settings[ctrl_name] = val
        self.fd = None
        self.fd_handle=None
        self.server=None #placeholder for HTTP server
        self.vision_worker_thread=None #placeholder for VisionWorker thread
        self.buffers = []
        self.fps_divider = 3 #pass only Nth frame to stream
        self.lastevt=0
        self.heatmap_on=True
        self.overlay_on =True
        self.hud_on=True
        self.fps_on=True
        width, height = map(int, self.resolution.split('x'))
        self.fps_divider = config.getint('fps_divider',3) #pass only Nth frame to stream
        cm_list = config.getasteval('camera_matrix', [[float(width), 0.0, width/2], [0.0, float(height), height/2], [0.0, 0.0, 1.0]])
        self.camera_matrix = numpy.array(cm_list, dtype=numpy.float32).reshape((3, 3))
        dist_list = config.getasteval('dist_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.dist_coeffs = numpy.array(dist_list, dtype=numpy.float32)
        virtual_camera_matrix_list = config.getasteval('virtual_camera_matrix', [[width, 0.0, width//2], [0.0,height, height//2], [0.0, 0.0, 1.0]])
        self.virt_matrix = numpy.array(virtual_camera_matrix_list, dtype=numpy.float32).reshape((3, 3))
        rectification_matrix_list = config.getasteval('rectification_matrix', [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self.rectif_matrix = numpy.array(rectification_matrix_list, dtype=numpy.float32).reshape((3, 3))
        self.mm_per_px=config.getasteval('mm_per_px', None)
        self.def_z_plane=config.getfloat('def_z', -10.)

        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:disconnect",  self._handle_disconnect)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('CAM_SNAP', "CAM", self.name,
                        self.cmd_CAM_SNAP, desc=self.cmd_CAM_SNAP_help)
        gcode.register_mux_command('CAM_SET', 'CAM', self.name,
                        self.cmd_CAM_SET, desc=self.cmd_CAM_SET_help)
        gcode.register_mux_command('CAM_GET', 'CAM', self.name,
                        self.cmd_CAM_GET, desc=self.cmd_CAM_GET_help)
        gcode.register_mux_command('CAM_HUD', 'CAM', self.name,
                        self.cmd_CAM_HUD, desc=self.cmd_CAM_HUD_help)
        gcode.register_mux_command('CAM_LIGHT', 'CAM', self.name, 
                        self.cmd_CAM_LIGHT, desc=self.cmd_CAM_LIGHT_help)
        gcode.register_mux_command('CAM_CALIB', 'CAM', self.name,
                        self.cmd_CAM_CALIB, desc=self.cmd_CAM_CALIB_help)

    def _handle_connect(self):
        if self.light_name:
            self.light = self.printer.lookup_object(self.light_name)
            if not hasattr(self.light, 'led_helper'):
                raise self.printer.config_error(
                    "[camera_v4l %s] light '%s' has no led_helper"
                    % (self.name, self.light_name))
            self.light_channels = probe_led_channels(self.light)
            logging.info(
                "[camera_v4l %s]: light=%s channels=%s color=%s s=%s"
                % (self.name, self.light_name,
                   channels_str(self.light_channels),
                   format_hex_color(self._light_color), self._light_s))
        (self.frame_width, self.frame_height) = map(int, self.resolution.split('x'))
        try:
            self.fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK, 0)
            fmt = v4l2.v4l2_format()
            fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            fmt.fmt.pix.width = self.frame_width
            fmt.fmt.pix.height = self.frame_height
            fmt.fmt.pix.pixelformat = v4l2.V4L2_PIX_FMT_MJPEG
            fcntl.ioctl(self.fd, v4l2.VIDIOC_S_FMT, fmt)
            # fcntl.ioctl(self.fd, v4l2.VIDIOC_S_PRIORITY,v4l2.V4L2_PRIORITY_BACKGROUND)
            req = v4l2.v4l2_requestbuffers()
            req.count = 2  # Double-buffering
            req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            req.memory = V4L2_MEMORY_MMAP
            fcntl.ioctl(self.fd, v4l2.VIDIOC_REQBUFS, req)
            self.buffers = []
            for i in range(req.count):
                buf = v4l2.v4l2_buffer()
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
                buf.memory = V4L2_MEMORY_MMAP
                buf.index = i
                fcntl.ioctl(self.fd, v4l2.VIDIOC_QUERYBUF, buf)
                mm = mmap.mmap(self.fd, buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=buf.m.offset)
                self.buffers.append(mm)
                fcntl.ioctl(self.fd, v4l2.VIDIOC_QBUF, buf)
            type_buf = v4l2.v4l2_buf_type(V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.fd, v4l2.VIDIOC_STREAMON, type_buf)
            if self.settings:
                reactor = self.printer.get_reactor()
                reactor.pause(reactor.monotonic() + .250) # Wait 250ms for device to initialize
                self._apply_v4l2_settings(self.settings)
        except Exception as e:
            raise self.printer.config_error("cam %s: v4l error: %s"% (self.name, e))
        # Start HTTP micro-serwer
        self.server = ThreadedHTTPServer((self.httpaddr, self.httpport), MJPEGHandler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.vision_worker_thread = VisionWorker(self) #self as parent
        threading.Thread(target=self.vision_worker_thread.run, daemon=True).start()
        self.vision_worker_thread.generate_maps(self.camera_matrix,self.dist_coeffs,
            self.rectif_matrix,self.virt_matrix)
        self.fd_handle=self.reactor.register_fd(self.fd, self._handle_camera_fd)

    def _handle_camera_fd(self, eventtime):
        buf = v4l2.v4l2_buffer()
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        buf.memory = V4L2_MEMORY_MMAP
        # self.frame_counter+=1
        try:
            fcntl.ioctl(self.fd, v4l2.VIDIOC_DQBUF, buf)
            raw_jpeg = self.buffers[buf.index][:buf.bytesused]
            # if raw_jpeg is not None or (self.frame_counter % self.fps_divider == 0):
            self.vision_worker_thread.push_frame((eventtime,raw_jpeg))
            fcntl.ioctl(self.fd, v4l2.VIDIOC_QBUF, buf)
        except Exception as e:
            logging.error(f"_handle_camera_fd {e}")

    def _handle_disconnect(self):
        logging.info(f"Vision [{self.name}]:_handle_disconnect {self.httpaddr}:{self.httpport}")
        return self._handle_shutdown()
    def _handle_shutdown(self):
        logging.info(f"Vision [{self.name}]:_handle_shutdown {self.httpaddr}:{self.httpport}")
        self._cancel_light_hold()
        if self.server is not None:
            try:
                self.server.shutdown()
                self.server.server_close()
                logging.info(f"Vision [{self.name}]: MJPEG Server {self.httpaddr}:{self.httpport} shutdown.")
            except Exception as e:
                logging.info(f"Vision [{self.name}]: Error on shutdown: {str(e)}")
            self.server = None
        if self.fd_handle is not None:
            try: self.reactor.unregister_fd(self.fd_handle)
            except KeyError: pass
        if self.fd is not None:
            try:
                type_buf = v4l2.v4l2_buf_type(V4L2_BUF_TYPE_VIDEO_CAPTURE)
                fcntl.ioctl(self.fd, v4l2.VIDIOC_STREAMOFF, type_buf)
            except Exception as e:
                logging.info(f"Vision [{self.name}]: Błąd podczas zamykania fd: {str(e)}")
            os.close(self.fd)
            self.fd = None
        self.buffers = []

    def _get_supported_v4l2_controls(self):
        output = subprocess.check_output(["v4l2-ctl", "-d", self.device, "-l"], stderr=subprocess.PIPE, text=True)
        pattern = re.compile(r'^\s*([a-zA-Z0-9_]+)\s+(.+)\s+\((.+)\)\s+:\s+(.*)$')
        controls = {}
        for line in output.splitlines():
            # logging.info(f"[camera_v4l {self.name}]: Parsing line: {line}")
            match = pattern.match(line)
            if match:
                ctrl_name = match.group(1)
                ctrl_id_type = match.group(2).strip()
                ctrl_type_info = match.group(3).strip()
                ctrl_details = match.group(4).strip()
                val_match = re.search(r'\bvalue=(-?\d+)\b', ctrl_details) #remove value=.* from ctrl_details
                # logging.info(f"[camera_v4l {self.name}] match {ctrl_name} ")
                if val_match:
                    value = val_match.group(1)
                    details = re.sub(r'\bvalue=-?\d+\b', '', ctrl_details).strip()
                    details = re.sub(r'\s+', ' ', details) # ładne pojedyncze spacje
                    controls[ctrl_name] = (value, ctrl_type_info, details)
        return controls

    def _apply_v4l2_settings(self,params):
        args = ["v4l2-ctl", "-d", self.device]
        for ctrl_name, val in params.items():
            args.extend(["-c", f"{ctrl_name}={val}"])
        logging.info(f"Setting V4L2 parameters for [{self.name}]: {' '.join(args)}")
        return subprocess.check_output(args, stderr=subprocess.PIPE)

    cmd_CAM_GET_help = "Retrieves current V4L2 camera configuration"
    def cmd_CAM_GET(self, gcmd):
        gcmd.respond_info(f"=== Konfiguracja V4L2 wygenerowana dla [{self.name}] ===")
        #take it from system again
        try: self.controls = self._get_supported_v4l2_controls()
        except subprocess.CalledProcessError as e:
            raise gcmd.error(f"Failed to get V4L2 parameters: {e.returncode}: {str(e.stderr)}")
        gcmd.respond_info("\nyou can copy these values to your camera section in printer.cfg:\n\n")
        for ctrl_name, (value, ctrl_type_info, clean_details) in self.controls.items():
            gcmd.respond_info(f"{ctrl_name}: \t{value}  \t#{ctrl_type_info}, {clean_details}")

    cmd_CAM_SET_help = "Sets V4L2 camera parameters. Usage: CAM_SET CAM=... PARAM=VALUE"
    def cmd_CAM_SET(self, gcmd):
        params = {k.lower(): gcmd.get(k).lower() for k in gcmd.get_command_parameters() if k != 'CAM'}
        if params:
            try:
                return self._apply_v4l2_settings(params)
            except subprocess.CalledProcessError as e:
                raise gcmd.error(f"Failed to set V4L2 parameters: {e.returncode}: {str(e.stderr)}")
        raise gcmd.error("Missing parameters")

    def get_snapshot(self):
        return self.vision_worker_thread.get_snapshot()

    def set_overlay(self, overlay):
        return self.vision_worker_thread.set_overlay(overlay)

    cmd_CAM_SNAP_help = "Takes a snapshot"
    def cmd_CAM_SNAP(self, gcmd):
        tag=gcmd.get("TAG", None)  # optional tag for filename
        gcmd.respond_info(f"Taking snapshot from [{self.name}]...")
        try:
            img,_ = self.get_snapshot()
            filename=datetime.datetime.now().strftime(f"/tmp/pnp_{self.name}_{tag}_%Y%m%d_%H%M%S.jpeg")
            cv2.imwrite(filename, img)
            gcmd.respond_info(f"Snapshot saved to {filename}.")
        except RuntimeError as e:
            gcmd.error(f"Failed to capture snapshot: {e}")
            return

    cmd_CAM_HUD_help = "Options for camera HUD and overlay"
    def cmd_CAM_HUD(self, gcmd):
        self.overlay_on=gcmd.get('OVERLAY', str(self.overlay_on)).lower() in ['1','t','true','on']
        self.hud_on=gcmd.get('HUD', str(self.hud_on)).lower() in ['1','t','true','on']
        self.fps_on=gcmd.get('FPS', str(self.fps_on)).lower() in ['1','t','true','on']


    def _cancel_light_hold(self):
        if self._light_hold_timer is not None:
            self.reactor.unregister_timer(self._light_hold_timer)
            self._light_hold_timer = None

    def _bump_light_hold(self):
        waketime = self.reactor.monotonic() + self.light_hold
        if self._light_hold_timer is None:
            self._light_hold_timer = self.reactor.register_timer(
                self._light_hold_event, waketime)
        else:
            self.reactor.update_timer(self._light_hold_timer, waketime)

    def _light_hold_event(self, eventtime):
        self._light_hold_timer = None
        self.set_light(output_s=self.light_idle)
        return self.reactor.NEVER

    def set_light(self, s=None, color=None, output_s=None):
        """Remember s/color and transmit. output_s overrides level without
        changing the remembered working brightness (used for idle / exclusive)."""
        if s is not None:
            self._light_s = max(0., min(1., float(s)))
        if color is not None:
            self._light_color = color
        level = self._light_s if output_s is None else max(0., min(1., float(output_s)))
        if self.light is None:
            if level > 0.:
                raise self.printer.command_error(
                    "Camera [%s] has no light: configured" % (self.name,))
            self._output_s = level
            return False
        rgbw = map_rgb_to_led(self._light_color, level, self.light_channels)
        if rgbw == self._last_rgbw and level == self._output_s:
            return False
        self.light.led_helper._set_color(None, rgbw)
        self.light.led_helper._check_transmit(None)
        self._last_rgbw = rgbw
        self._output_s = level
        return True

    def acquire_light(self, s, color, hold=True, settle=True):
        changed = self.set_light(s=s, color=color)
        want = self._output_s
        if settle and changed and want > 0. and self.light_settle > 0.:
            self.reactor.pause(self.reactor.monotonic() + self.light_settle)
        if hold and want > 0. and self.light_hold > 0.:
            self._bump_light_hold()
        else:
            self._cancel_light_hold()

    cmd_CAM_LIGHT_help = "Camera light: LIGHT=<0..1 and/or hex RGB>"
    def cmd_CAM_LIGHT(self, gcmd):
        spec = gcmd.get('LIGHT', None)
        if spec is None:
            gcmd.respond_info(
                "CAM_LIGHT [%s] light=%s channels=%s s=%.3f output=%.3f color=%s"
                % (self.name,
                   self.light_name or '(none)',
                   channels_str(self.light_channels),
                   self._light_s, self._output_s,
                   format_hex_color(self._light_color)))
            return
        try:
            s, color = parse_light(spec)
        except ValueError as e:
            raise gcmd.error(str(e))
        self.acquire_light(s, color, hold=False, settle=False)
        gcmd.respond_info(
            "CAM_LIGHT [%s] channels=%s s=%.3f output=%.3f color=%s rgbw=%s"
            % (self.name, channels_str(self.light_channels),
               self._light_s, self._output_s,
               format_hex_color(self._light_color),
               None if self._last_rgbw is None else
               tuple(round(c, 3) for c in self._last_rgbw)))

    cmd_CAM_CALIB_help = "Configure camera rectification matrices"
    def cmd_CAM_CALIB(self, gcmd):
        class sentinel:
            pass
        m=gcmd.get_ast_eval('M', sentinel)
        d=gcmd.get_ast_eval('D', sentinel)
        r=gcmd.get_ast_eval('R', sentinel)
        v=gcmd.get_ast_eval('V', sentinel)
        a=gcmd.get_float('A', None)
        if m is not sentinel:
            try: #clear val if 'None' was passed else parse matrix
                self.camera_matrix=None if m is None else numpy.array(m, dtype=numpy.float32).reshape((3, 3))
            except Exception as e: raise gcmd.error(f"error {str(e)}")
        if d is not sentinel:
            try: #clear val if 'None' was passed else parse matrix
                self.dist_coeffs=None if d is None else numpy.array(d, dtype=numpy.float32)
            except Exception as e: raise gcmd.error(f"error {str(e)}")
        if r is not sentinel:
            try: #clear val if 'None' was passed else parse matrix
                self.rectif_matrix=None if r is None else numpy.array(r, dtype=numpy.float32).reshape((3, 3))
            except Exception as e: raise gcmd.error(f"error {str(e)}")
        if v is not sentinel:
            try: #clear val if 'None' was passed else parse matrix
                self.virt_matrix=None if v is None else numpy.array(v, dtype=numpy.float32).reshape((3, 3))
            except Exception as e: raise gcmd.error(f"error {str(e)}")
        gcmd.respond_info(f'M="{None if self.camera_matrix is None else self.camera_matrix.tolist()}"')
        gcmd.respond_info(f'D="{None if self.dist_coeffs is None else self.dist_coeffs.tolist()}"')
        gcmd.respond_info(f'R="{None if self.rectif_matrix is None else self.rectif_matrix.tolist()}"')
        if self.camera_matrix is not None and self.virt_matrix is None and a is not None:
            gcmd.respond_info(f"Creating virtal_matrix with a={a}")
            self.virt_matrix, roi = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, self.dist_coeffs,
                (self.frame_width, self.frame_height),
                alpha=a, newImgSize=(self.frame_width, self.frame_height))
            self.virt_matrix[0, 2] = self.virt_matrix[0, 0] / 2.0  # force cx to center
            self.virt_matrix[1, 2] = self.virt_matrix[1, 1] / 2.0  # force cy to center
            cx = self.virt_matrix[0, 2]
            cy = self.virt_matrix[1, 2]
            gcmd.respond_info(f"ROI={roi} c={(cx,cy)}")
        gcmd.respond_info(f'V="{None if self.virt_matrix is None else self.virt_matrix.tolist()}"')
        if (self.camera_matrix is None or self.dist_coeffs is None or
            self.rectif_matrix is None or self.virt_matrix is None):
            gcmd.respond_info(f"Not enough params to generate maps.")
            return
        try: self.vision_worker_thread.generate_maps(
                self.camera_matrix,self.dist_coeffs,
                self.rectif_matrix,self.virt_matrix)
        except Exception as e:
            raise gcmd.error(f"Error in generate_maps {e}")

    def get_mmpx_on_z(self, z):
        """
        Positive mm/px magnitudes for motion prediction.
        Config and calib store +/+ ; image-Y flip is applied in predict/px→mm.
        """
        if self.mm_per_px is not None and len(self.mm_per_px) == 2:
            (x1,y1,z1),(x2,y2,z2)= self.mm_per_px
            k = ((z - z1) / (z2 - z1)) if abs(z2 - z1) > 0.001 else 0
            upp_x = x1 + k * (x2 - x1)
            upp_y = y1 + k * (y2 - y1)
            return abs(float(upp_x)), abs(float(upp_y))
        return 0., 0.

    def pixels_to_mm_offset(self, x_px, y_px, z_working_height=None):
        """
        Pixel offset from principal point → machine XY offset (mm).
        mm_per_px samples are positive magnitudes; axis signs:
          X: always +dx_px * upp_x
          Y: looking=down (topcam) → -dy_px * upp_y  (image Y down vs machine Y)
             looking=up   (bottom) → +dy_px * upp_y
        """
        if self.virt_matrix is None or self.mm_per_px is None or len(self.mm_per_px)!=2:
            return 0., 0.
        if z_working_height is None:
            z_working_height=self.def_z_plane
        dx_px = x_px - self.virt_matrix[0, 2]
        dy_px = y_px - self.virt_matrix[1, 2]
        upp_x, upp_y = self.get_mmpx_on_z(z_working_height)
        delta_x_mm = float(dx_px) * upp_x
        # down-looking: image +Y is machine −Y; up-looking: same sense
        y_sign = 1.0 if self.looking_up else -1.0
        delta_y_mm = y_sign * float(dy_px) * upp_y
        return float(delta_x_mm), float(delta_y_mm)

    def get_status(self, eventtime):
        return {
            'on': self.vision_worker_thread is not None,
            'cam_res': self.resolution,
            'light_name': self.light_name,
            'light_rgb': format_hex_color(self._last_rgbw),
            'light_val':self._output_s,
        }

def load_config_prefix(config):
    return V4L2Camera(config)
