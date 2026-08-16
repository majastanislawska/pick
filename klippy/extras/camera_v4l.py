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
        self.loookingup = config.getchoice('looking',{'up':True,'down':False},default=False)
        self.httpaddr = config.get('http_addr', '0.0.0.0')
        self.httpport = config.getint('http_port', 8081)
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
        gcode.register_mux_command('CAM_CALIB', 'CAM', self.name,
                        self.cmd_CAM_CALIB, desc=self.cmd_CAM_CALIB_help)

    def _handle_connect(self):
        self.vision = self.printer.lookup_object('pnp_vision')
        self.vision.register_camera(self.name,self)
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

    def pixels_to_mm_offset(self, x_px, y_px, z_working_height=None):
        if self.virt_matrix is None or self.mm_per_px is None or len(self.mm_per_px)!=2:
            return x_px, y_px
        if z_working_height is None:
            z_working_height=self.def_z_plane
        dx_px = x_px - self.virt_matrix[0, 2]
        dy_px = y_px - self.virt_matrix[1, 2]
        (x1,y1,z1),(x2,y2,z2)= self.mm_per_px
        k = ((z_working_height - z1) / (z2 - z1)) if abs(z2 - z1) > 0.001 else 0
        upp_x = x1 + k * (x2 - x1)
        upp_y = y1 + k * (y2 - y1)
        upp_x, upp_y = abs(float(upp_x)), abs(float(upp_y))
        delta_x_mm = float(dx_px) * upp_x
        delta_y_mm = -float(dy_px) * upp_y
        return float(delta_x_mm), float(delta_y_mm)

def load_config_prefix(config):
    return V4L2Camera(config)
