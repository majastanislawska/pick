# klippy/extras/pnp_vision.py
# Computer vision manager with dynamic vision pipeline support
#
# Copyright (C) 2026 Maja Stanislawska <maja@makershop.ie>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import numpy
import cv2
from .pnp_vision_pipeline import PnpVisionPipeline

#tape detection helpers
# EIA-481 sprocket-line → pocket-center (F), mm
EIA_481_F={8:3.5, 12:5.5, 16:7.5, 24:11.5}
#not F per se but range we're interested in
EIA_481_F_fol={8:(3.25, 3.75), 12:(5.25, 5.75), 16:(7.25, 7.75), 24:(11.25, 11.75)}
# Shoelace theorem but returns double triangle area
def get_signed_area_2d(c0, c1, p):
    return ((c1[0] - c0[0]) * (p[1] - c0[1]) - (c1[1] - c0[1]) * (p[0] - c0[0]))

class PnPVision:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.pnp = self.printer.lookup_object('pnp')
        self.cams = {}
        self.pipelines = {}
        self.last_detection = {
            'success': False,
            'pipeline': None,
            'cam': None,
            'cx': None,
            'cy': None,
            'r': None,
            'coords': None,
        }
        self.last_home = {
            'success': False,
            'fiducial': None,
            'toolhead': None,
            'gcode_offset': None,
            'err_mm': None,
        }
        self.home_speed = config.getfloat('homing_speed', 50.)
        self.pinpoint_accuracy = config.getfloat('pinpoint_accuracy', 0.05)
        self.max_tries = config.getint('max_tries', 8)
        webhooks = self.printer.lookup_object('webhooks')
        webhooks.register_endpoint("pnp/light", self._webhooks_light)
        webhooks.register_endpoint("pnp/px2mach", self._webhooks_px2mach)
        self.gcode.register_command(
            'PNP_VISION_PIPELINE', self.cmd_PNP_VISION_PIPELINE,
            desc=self.cmd_PNP_VISION_PIPELINE_help)
        self.gcode.register_command(
            'PNP_VISION_DETECT', self.cmd_PNP_VISION_DETECT,
            desc=self.cmd_PNP_VISION_DETECT_help)
        self.gcode.register_command(
            'PNP_VISION_LIST', self.cmd_PNP_VISION_LIST,
            desc=self.cmd_PNP_VISION_LIST_help)
        self.gcode.register_command(
            'PNP_VISION_HOME', self.cmd_PNP_VISION_HOME,
            desc=self.cmd_PNP_VISION_HOME_help)
        self.gcode.register_command(
            'PNP_VISION_RUNOUT', self.cmd_PNP_VISION_RUNOUT,
            desc=self.cmd_PNP_VISION_RUNOUT_help)
        self.gcode.register_command(
            'PNP_VISION_TAPE', self.cmd_PNP_VISION_TAPE,
            desc=self.cmd_PNP_VISION_TAPE_help)

    def _webhooks_light(self, web_request):
        on=web_request.get('on', True, types=(bool,))
        s=web_request.get('s',None, types=(float, None,))
        color=web_request.get_str('color',None)
        cam=self.lookup_cam(web_request.get_str('cam',None))
        if s is not None:
            cam.acquire_light(s, color, hold=False, settle=False)
        else:
            if on: cam.acquire_light(cam.light_s_default, color, hold=False, settle=False)
            else: cam.acquire_light(cam.light_idle, color, hold=False, settle=False)
    def _webhooks_px2mach(self, web_request):
        x_px=web_request.get_float('px')
        y_px=web_request.get_float('py')
        cam=self.lookup_cam(web_request.get_str('cam',None))
        (dx,dy)=cam.pixels_to_mm_offset(x_px, y_px)#, z_working_height=None)
        if cam.looking_up:
            web_request.send({'dx':-dx, 'dy':-dy})
        else:
            web_request.send({'dx':-dx, 'dy':-dy})

    def register_camera(self, name, cam):
        logging.info(f"register_camera: {name} {cam.resolution}")
        self.cams[name.upper()] = cam

    def lookup_cam(self, cam_name, up=None):
        key = cam_name.upper()
        if key not in self.cams:
            raise self.printer.command_error(f"Unknown camera '{cam_name}'. Known: {list(self.cams.keys())}")
        cam=self.cams[key]
        if up is None:           return cam
        elif up==cam.looking_up: return cam
        else:
            up_str="up" if up is True else "down"
            raise self.printer.command_error(f"Can't use'{cam_name}' here. Camera needs to be looking {up_str}")

    def other_cameras(self,me):
        return [c for c in self.cams.values() if c is not me]

    def parse_runtime_params(self, gcmd, skip=[]):
        """G-code key=value (except skip) -> dict for pipeline defaults override."""
        params = {}
        skip = [k.upper() for k in skip]
        for k in gcmd.get_command_parameters():
            if k.upper() in skip: continue
            val_str = gcmd.get(k)
            try: params[k.lower()] = (float(val_str) if '.' in val_str else int(val_str))
            except ValueError: params[k.lower()] = val_str
        return params

    def get_size_params(self, gcmd, params, skip):
        params['dia'] = gcmd.get_float('DIA', params.get('dia', None))
        params['w'] = gcmd.get_float('W', params.get('w', None))
        params['h'] = gcmd.get_float('H', params.get('h', None))
        params['rot'] = gcmd.get_float('ROT', params.get('rot', None))
        params['tol'] = gcmd.get_float('TOL', params.get('tol', 0.1), minval=0.)
        params['rot_tol'] = gcmd.get_float('ROT_TOL', params.get('rot_tol', None))
        skip.extend(['DIA', 'W', 'H', 'ROT', 'TOL', 'ROT_TOL'])
        return skip

    def size_params_from_mm(self, cam, z, params, tol=0.1):
        """
        Convert physical sizes (mm) at z_plane → pipeline params (px).

        Uses average |mm/px| from cam.get_mmpx_on_z (affine A).
        Circle: dia → r, min_r, max_r, rr, dia_px
        Rect:   width/height → w_px, h_px, min/max_width_px, aspect band
        """
        if z is None:
            return params
        ux, uy = cam.get_mmpx_on_z(z)
        upp = 0.5 * (abs(ux) + abs(uy))
        if upp < 1e-12:
            return params
        dia = params.get('dia',None)
        width_mm = params.get('w',None)
        height_mm = params.get('h',None)
        auto = {}
        if dia is not None:
            # filled circle: equivalent r ≈ dia/2 in px
            r_px = 0.5 * float(dia) / upp
            auto['min_r'] = r_px * (1.0 - tol)
            auto['max_r'] = r_px * (1.0 + tol)
            auto['dia_px'] = float(dia) / upp
            auto['min_area'] = float(2.*numpy.pi*auto['min_r']**2)
            auto['max_area'] = float(2.*numpy.pi*auto['max_r']**2)
        if width_mm is not None:
            w_px = float(width_mm) / upp
            auto['w_px'] = w_px
            auto['min_w_px'] = w_px * (1.0 - tol)
            auto['max_w_px'] = w_px * (1.0 + tol)
        if height_mm is not None:
            h_px = float(height_mm) / upp
            auto['h_px'] = h_px
            auto['min_h_px'] = h_px * (1.0 - tol)
            auto['max_h_px'] = h_px * (1.0 + tol)
        if width_mm is not None and height_mm is not None:
            long_mm = max(float(width_mm), float(height_mm))
            short_mm = min(float(width_mm), float(height_mm))
            if short_mm > 1e-9:
                aspect = long_mm / short_mm
                auto['min_aspect'] = aspect * (1.0 - tol)
                auto['max_aspect'] = aspect * (1.0 + tol)
                auto['long_px'] = long_mm / upp
                auto['short_px'] = short_mm / upp
                auto['min_area'] = auto['min_w_px']*auto['min_h_px']
                auto['max_area'] = auto['max_w_px']*auto['max_h_px']
        auto.update(params) # overwrite with any user-provided parameters
        return auto

    def run_detect(self, cam, pipe_name, gcmd, runtime_params=None):
        """
        Run a named pipeline on a camera snapshot.
        Returns a result dict; also updates self.last_detection.
        Result shapes (no silent multi→single pick — filter in the pipeline):
          - exactly 1 circle, 0 rects → flat {kind:'circle', cx, cy, r, score}
          - exactly 1 rect, 0 circles → flat {kind:'rect', cx, cy, w, h, angle, ratio}
          - multi / mixed → {kind:'circles'|'rects'|'mixed', lists via todict()
        mixed is for tape: sprocket holes (circles) + pocket (rect) in one shot.
        Consumers that need a single feature use ensure_singular_result().
            res = vision.run_detect(cam, 'fiducial', gcmd, {'min_r': 20})
            cx, cy = vision.ensure_singular_result(res, gcmd, 'label')
        runtime_params: dict of key=value overrides for pipeline defaults (optional).
        """
        if pipe_name not in self.pipelines:
            raise gcmd.error(f"Pipeline '{pipe_name}' does not exist.")
        pipe = self.pipelines[pipe_name]
        runtime_params = dict(runtime_params or {})
        try:
            s, color = pipe.resolve_light(
                runtime_params.pop('light', None), gcmd.get('LIGHT', None))
            if s is not None or color is not None:
                cam.acquire_light(s, color)
        except ValueError as e:
            raise gcmd.error(str(e))
        img, eventtime = cam.get_snapshot()
        ret = pipe.run(eventtime, img, gcmd, cam, runtime_params)
        cam.set_overlay(pipe.img)
        pos = list(self.pnp.get_position())
        r = {
            'success': ret,
            'pipeline': pipe_name,
            'cam': cam.name,
            'params': runtime_params,
            'z': runtime_params.get('z', pos[2] if pos else None),
            'pos': pos,
        }
        match (len(pipe.detected_circles),len(pipe.detected_rects)):
            case (1,0):
                item=pipe.detected_circles[0]
                r['kind']='circle'
                for i in ['cx','cy','r','c_score']:
                    r[i]=getattr(item, i)
            case (0,1):
                item=pipe.detected_rects[0]
                r['kind']='rect'
                for i in ['cx','cy','w','h','angle','ratio']:
                    r[i]=getattr(item, i)
            case (_,0):
                r['kind']='circles'
                r['circles']=[i.todict() for i in pipe.detected_circles]
            case (0,_):
                r['kind']='rects'
                r['rects']=[i.todict() for i in pipe.detected_rects]
            case (_,_):
                r['kind']='mixed'
                r['circles']=[i.todict() for i in pipe.detected_circles]
                r['rects']=[i.todict() for i in pipe.detected_rects]
        self.last_detection = r
        pipe.cleanup()
        return r

    def ensure_singular_result(self, result, gcmd, label):
        """
        Strict single-feature result for HOME / CALIB_PXMM / visual_home.
        Exactly one circle or one rect (flat cx/cy). Multi, mixed, or empty → error.
        Debris must be filtered in the pipeline (min_r, circularity, …).
        """
        if not result['success']:
            raise gcmd.error(f"{label}: no feature center detected. Check PIPELINE / scene.")
        kind = result.get('kind')
        match kind:
            case 'circle': return float(result['cx']), float(result['cy'])
            case 'rect':   return float(result['cx']), float(result['cy'])
            case 'circles': err=f"{len(result['circles'])} circles"
            case 'rects':   err=f"{len(result['rects'])} rects"
            case 'mixed':   err=f"{len(result['circles'])} circles and {len(result['rects'])} rects"
            case _:         err="some bogus response"
        raise gcmd.error(
            f"{label}: expected exactly 1 feature, got {err}. "
            f"Tighten PIPELINE filters")

    def pinpoint_target(self, gcmd, startpos, cam, pipe_name, speed, settle, max_tries, accuracy, params, tag="pinpoint"):
        """
        Loop a pipeline until machine converges over a feature.
        """
        self.pnp.move_and_settle(startpos, speed, settle)
        ret={}
        z=startpos[2]
        err = None
        dx_mm = dy_mm = 0.
        px = py = None
        for i in range(max_tries):
            result = self.run_detect(cam, pipe_name, gcmd, params)
            px, py = self.ensure_singular_result(result, gcmd, f"{tag}[{i+1}]")
            dx_mm, dy_mm = cam.pixels_to_mm_offset(px, py, z)
            err = numpy.hypot(dx_mm, dy_mm).astype(float)
            gcmd.respond_info(f"{tag}[{i+1}/{max_tries}]: px=({px:.2f},{py:.2f}) d_mm=({dx_mm:.4f},{dy_mm:.4f}) err={err:.4f}mm")
            if err <= accuracy:
                gcmd.respond_info(f"converged: err={err:.4f} <= ACCURACY={accuracy}")
                ret['success'] = True
                ret['pos'] = list(self.pnp.get_position())
                ret['px'], ret['py'] = float(px), float(py)
                ret['err_mm'] = err
                break
            pos = self.pnp.get_position()
            (pos[0],pos[1])=(pos[0] - dx_mm, pos[1] - dy_mm,)
            self.pnp.move_and_settle(pos, speed, settle)
        else:
            ret['success'] = False
            ret['pos'] = list(self.pnp.get_position())
            if px is not None:
                ret['px'], ret['py'] = float(px), float(py)
            ret['err_mm'] = err
        return ret

    cmd_PNP_VISION_PIPELINE_help = "Define or update a vision pipeline"
    def cmd_PNP_VISION_PIPELINE(self, gcmd):
        """
        PNP_VISION_PIPELINE NAME=fiducial LIGHT=0.8 PARAMS="min_r:10;max_r:50;circ_sim:0.75" \
            STEPS="gray()|otsu()|find_contours_by_r(min_r,max_r)|circular_symmetry(circ_sim)"
        LIGHT= is 0..1 brightness, hex RGB(W), or both (0.8,F80).
        """
        name = gcmd.get('NAME', None)
        steps = gcmd.get('STEPS', None)
        if not name:
            raise gcmd.error("NAME is required.")
        if not steps:
            raise gcmd.error("STEPS is required.")
        params_str = gcmd.get('PARAMS', '')
        light_spec = gcmd.get('LIGHT', None)
        defaults = {}
        if params_str:
            for item in params_str.split(';'):
                if ':' in item:
                    k, v = item.split(':', 1)
                    try:
                        defaults[k.strip().lower()] = (
                            float(v.strip()) if '.' in v.strip()
                            else int(v.strip()))
                    except ValueError:
                        defaults[k.strip().lower()] = v.strip()
        self.pipelines[name] = PnpVisionPipeline(name, steps, defaults, light_spec)
        gcmd.respond_info(f"PnPVision: pipeline registered: '{name}")

    cmd_PNP_VISION_LIST_help = "List registered vision pipelines"
    def cmd_PNP_VISION_LIST(self, gcmd):
        if not self.pipelines:
            gcmd.respond_info("(None)")
            return
        for name, pipe in self.pipelines.items():
            light = getattr(pipe, 'light_spec', None)
            gcmd.respond_info(
                f"Pipeline: {name}\n"
                f"  Light: {light}\n"
                f"  Params: {pipe.defaults}\n"
                f"  Steps: {pipe.steps_str}\n")

    cmd_PNP_VISION_DETECT_help = (
        "Run a vision pipeline on a camera snapshot. "
        "Optional LIGHT= (0..1 and/or hex) before snap. "
        "Optional mm→px: FIDUCIAL= / Z= DIA= W= H= SIZE= TOL= "
        "(console R= / MIN_R= etc. override auto sizes).")
    def cmd_PNP_VISION_DETECT(self, gcmd):
        """
        PNP_VISION_DETECT CAM=topcam PIPELINE=fiducial
        PNP_VISION_DETECT CAM=topcam PIPELINE=visual_home FIDUCIAL=primary
        PNP_VISION_DETECT CAM=topcam PIPELINE=visual_home Z=-10.3 DIA=2
        PNP_VISION_DETECT ... PIPELINE=pax W=2.0 H=1.25
        PNP_VISION_DETECT ... R=50   # overrides auto r from DIA
        """
        cam = self.lookup_cam(gcmd.get('CAM', 'topcam'))
        pipe_name = gcmd.get('PIPELINE', None)
        if not pipe_name:
            raise gcmd.error("PIPELINE is required.")
        # fiducial have x,y,z and dia or w,h and optionally tol;
        # we use z for mm→px conversion and dia/w/h for auto size params
        fid_name, fid = self.pnp.lookup_fiducial(gcmd.get('FIDUCIAL', None))
        params=fid.copy()
        #now update with overrides from commandline
        z = gcmd.get_float('Z', params.get('z', getattr(cam, 'def_z_plane', None)))
        skip= self.get_size_params(gcmd, params, ['CAM', 'PIPELINE', 'FIDUCIAL', 'Z'])
        params= self.size_params_from_mm(cam, z, params, params['tol'])
        params['cx']=cam.cx
        params['cy']=cam.cy
        runtime = self.parse_runtime_params(gcmd, skip)
        params.update(runtime)
        keys = ', '.join(
                f'{k}={v:.3f}' if isinstance(v, float) else f'{k}={v}'
                for k, v in sorted(params.items()))
        gcmd.respond_info(f"size@Z={float(z):g} mm/px → {keys}")
        result = self.run_detect(cam, pipe_name, gcmd, params)
        if not result['success']:
            raise gcmd.error("DETECT finished — no feature found.")
        elif result.get('kind') in ('circle', 'rect') and result.get('cx') is not None:
            gcmd.respond_info(
                f"DETECT ok kind={result.get('kind')} "
                f"cx={result.get('cx'):.3f} cy={result.get('cy'):.3f} "
                f"r={result.get('r')} angle={result.get('angle')}")
        elif result.get('kind') in ('circles', 'rects', 'mixed'):
            gcmd.respond_info(
                f"DETECT multi kind={result.get('kind')} "
                f"circles={len(result.get('circles',[]))} "
                f"rects={len(result.get('rects',[]))} "
                f"(ok for tape/pocket; HOME/CALIB need exactly 1 feature)")
        else:
            gcmd.respond_info("DETECT finished — no feature found")

    cmd_PNP_VISION_HOME_help = (
        "Vision-home XY on a fiducial: move to nominal, detect, jog to center, "
        "loop until offset≈0, then SET_GCODE_OFFSET so gcode matches fiducial XY.")
    def cmd_PNP_VISION_HOME(self, gcmd):
        """
        PNP_VISION_HOME FIDUCIAL=primary CAM=topcam PIPELINE=fiducial

        1) optional SET_GCODE_OFFSET X=0 Y=0
        2) move toolhead to fiducial nominal XY (Z unchanged)
        3) detect -> pixels_to_mm_offset (using fiducial nominal Z) -> jog
        4) repeat until |err| <= ACCURACY
        5) SET_GCODE_OFFSET so M114 shows fiducial (fx,fy) at this physical pose

        Optional: ACCURACY=0.05 MAX_TRIES=8 SPEED=50 SETTLE=0.2 SET_OFFSET=1
        Size band: uses fiducial dia@z → r/min_r/max_r unless R=/DIA=/TOL= on console.
        Other pipeline runtime params are forwarded.
        """
        cam = self.lookup_cam(gcmd.get('CAM', 'topcam'), False)
        #need another validation for tool?
        self.pnp.select_tool(cam,False)
        pipe_name = gcmd.get('PIPELINE', 'visual_home')
        if pipe_name not in self.pipelines:
            raise gcmd.error(f"Pipeline '{pipe_name}' does not exist. Register with PNP_VISION_PIPELINE first.")
        accuracy = gcmd.get_float('ACCURACY', self.pinpoint_accuracy, above=0.)
        max_tries = gcmd.get_int('MAX_TRIES', self.max_tries, minval=1)
        speed = gcmd.get_float('SPEED', self.home_speed, above=0.)
        settle = gcmd.get_float('SETTLE', 0.2, minval=0.)
        set_offset = gcmd.get_int('SET_OFFSET', 1)
        # fiducial have x,y,z and dia or w/h and optionally tol;
        # we use z for mm→px conversion and dia/w/h for auto size params
        fid_name, fid = self.pnp.lookup_fiducial(gcmd.get('FIDUCIAL', 'primary'))
        params=fid.copy()
        #now update with overrides from commandline
        z = gcmd.get_float('Z', params.get('z', getattr(cam, 'def_z_plane', None)))
        fx = gcmd.get_float('X', params.get('x', None))
        fy = gcmd.get_float('Y', params.get('y', None))
        ux, uy = cam.get_mmpx_on_z(z)
        upp = 0.5 * (abs(float(ux)) + abs(float(uy)))
        if upp < 1e-12:
            raise gcmd.error(f"Camera [{cam.name}] has unusable mm/px at Z={z:g} (upp≈{upp:.5g}) — check PNP_CALIB_PXMM or config.")
        skip= self.get_size_params(gcmd, params, ['CAM', 'PIPELINE', 'FIDUCIAL', 'Z'])
        params= self.size_params_from_mm(cam, z, params, params['tol'])
        params['cx']=cam.cx
        params['cy']=cam.cy
        runtime = self.parse_runtime_params(gcmd, skip)
        params.update(runtime)
        toolhead = self.printer.lookup_object('toolhead')
        # Nominal approach — XY only; keep toolhead Z (camera look height)
        pos = self.pnp.get_position()
        gcmd.respond_info(f"home on {fid_name}: nominal XY=({fx:.4f},{fy:.4f}) Z_plane={z:g}")
        pos[0],pos[1],pos[2]=fx,fy,z
        self.last_home=self.pinpoint_target(gcmd,pos,cam, pipe_name, speed, settle, max_tries, accuracy, params, "home")
        self.last_home['fiducial'] = fid_name
        if self.last_home['success'] == False:
            raise gcmd.error(f"PNP_VISION_HOME: not converged after {max_tries} iters (err={self.last_home['err_mm']:.4f}mm, accuracy={accuracy})")
        # Final re-check (no move) for reported residual
        result = self.run_detect(cam, pipe_name, gcmd, params)
        if result.get('kind') in ('circle', 'rect') and result.get('cx') is not None:
            cx, cy = float(result['cx']), float(result['cy'])
            dx_mm, dy_mm = cam.pixels_to_mm_offset(cx, cy, z)
            err = numpy.hypot(dx_mm, dy_mm).astype(float)
            gcmd.respond_info(f"final residual: px=({cx:.2f},{cy:.2f}) d_mm=({dx_mm:.4f},{dy_mm:.4f}) err={err:.4f}mm")
        ox = self.last_home['pos'][0] - fx
        oy = self.last_home['pos'][1] - fy
        self.last_home['gcode_offset'] = [ox, oy]
        cmd=f'SET_GCODE_OFFSET X={ox:.6f} Y={oy:.6f}'
        if set_offset:
            self.gcode.run_script_from_command(cmd)
            gcmd.respond_info(f"toolhead=({pos[0]:.4f},{pos[1]:.4f}) will now report as ({fx:.4f},{fy:.4f})")
            gcmd.respond_info(cmd)
        else:
            gcmd.respond_info(f"SET_OFFSET=0 — would use: {cmd}")
        gcmd.respond_info(f"PNP_VISION_HOME done on {fid_name} err={err:.4f}mm")

    def _fit_circle(self, pts):
        n = float(len(pts))
        sx = sum(p[0] for p in pts)
        sy = sum(p[1] for p in pts)
        sxx = sum(p[0]*p[0] for p in pts)
        syy = sum(p[1]*p[1] for p in pts)
        sxy = sum(p[0]*p[1] for p in pts)
        sx3 = sum(p[0]*(p[0]*p[0] + p[1]*p[1]) for p in pts)
        sy3 = sum(p[1]*(p[0]*p[0] + p[1]*p[1]) for p in pts)
        C = n*sxx - sx*sx
        D = n*sxy - sx*sy
        E = n*syy - sy*sy
        G = 0.5 * (n*sx3 - sx*(sxx+syy))
        H = 0.5 * (n*sy3 - sy*(sxx+syy))
        den = C*E - D*D
        # Identical / collinear samples (perfect tip, or no XY motion):
        # scatter matrix is singular → centroid + rms radius, not /0.
        scale = C*C + E*E
        if abs(den) <= 1e-18 * max(scale, 1.0):
            xc = sx / n
            yc = sy / n
            r2 = (sxx + syy) / n - xc*xc - yc*yc
            return xc, yc, float(numpy.sqrt(max(r2, 0.)))
        xc = (G*E - D*H) / den
        yc = (C*H - D*G) / den
        r2 = ((sxx+syy) - 2.*xc*sx - 2.*yc*sy)/n + xc*xc + yc*yc
        return xc, yc, float(numpy.sqrt(max(r2, 0.)))

    def runout(self, cam, pipe_name, params, gcmd):
        r_idx = self.pnp.r_index()
        if r_idx is None:
            raise gcmd.error("runout: no R axis for tool %s" % (self.pnp.active_tool.name,))
        if getattr(cam, 'camera_xy', None) is None:
            cur = self.pnp.get_position()
            cx, cy = float(cur[0]), float(cur[1])
            gcmd.respond_info(
                "runout: camera_xy unset, using current XY (%.4f, %.4f)"
                % (cx, cy))
        else:
            cx, cy =cam.camera_xy
        cam.z_plane = params['z']
        speed = params['speed']
        nsteps = params['steps']
        pos_min=params.get('pos_min',self.pnp.active_tool.r_stepper.pos_min)
        pos_max=params.get('pos_max',self.pnp.active_tool.r_stepper.pos_max)
        steps=numpy.linspace(pos_min,pos_max,nsteps).astype(float)
        self.pnp.active_tool.tip.runout_radius = 0.
        self.pnp.active_tool.tip.runout_phase = 0.
        self.pnp.active_tool.tip.runout_data = []
        pos = self.pnp.get_position()
        pos[0], pos[1], pos[2] = float(cx), float(cy), float(params['z'])
        pos[r_idx] = 0.
        self.pnp.safe_move(pos, speed)
        samples = []
        xy = []
        for k,r_k in enumerate(steps):
            pos[r_idx] = float(r_k)
            self.pnp.move_and_settle(pos, speed, params['settle'])
            result = self.run_detect(cam, pipe_name, gcmd, params)
            px, py = self.ensure_singular_result(result, gcmd, 'runout[%d]' % (k,))
            dx, dy = cam.pixels_to_mm_offset(px, py, cam.z_plane)
            x = float(cx) + dx
            y = float(cy) + dy
            samples.append((r_k, x, y))
            xy.append((x, y))
            gcmd.respond_info(
                "runout[%d/%d]: R=%s pxy=(%.4f,%.4f) d=(%.4f,%.4f) xy=(%.4f,%.4f)" % (k+1, nsteps, r_k, px, py, dx, dy, x, y))
        xc, yc, rho = self._fit_circle(xy)
        self.pnp.active_tool.tip.runout_radius = float(rho)
        self.pnp.active_tool.tip.runout_data = samples
        if rho > 0.:
            angles = []
            for r_k, x, y in samples:
                angles.append(float(numpy.atan2(y - yc, x - xc) - self.pnp.active_tool.r_to_rad(r_k)))
            # unwrap around first sample
            phi0 = float(angles[0])
            phi = float(sum(phi0 + numpy.atan2(numpy.sin(a-phi0), numpy.cos(a-phi0))
                        for a in angles) / len(angles))
            self.pnp.active_tool.tip.runout_phase = float(self.pnp.active_tool.r_from_rad(phi))
        gcmd.respond_info(
            "runout %s: radius=%.4f phase=%.4f %s  center=(%.4f,%.4f)"
            % (self.pnp.active_tool.tip.name, self.pnp.active_tool.tip.runout_radius,
            self.pnp.active_tool.tip.runout_phase, self.pnp.active_tool.r_units, xc, yc))
        pos[0], pos[1], pos[2] = float(cx), float(cy), 0.
        pos[r_idx] = 0.
        self.pnp.safe_move(pos, speed)

    cmd_PNP_VISION_RUNOUT_help = (
        "Spin loaded tip over the up-looking camera and fit live runout.")
    def cmd_PNP_VISION_RUNOUT(self, gcmd):
        cam = self.lookup_cam(gcmd.get('CAM', 'bottomcam'),True)
        tool = self.pnp.lookup_tool(None)
        if tool is None:
            raise gcmd.error("No tool loaded for runout")
        if tool.tip is None:
            raise gcmd.error(f"Tool {tool.short_name} has no tip loaded")
        params = tool.tip.get_runout_params()
        pipe_name = gcmd.get('PIPELINE', params.get('pipeline', 'runout'))
        if pipe_name not in self.pipelines:
            raise gcmd.error(f"Pipeline '{pipe_name}' does not exist. Register with PNP_VISION_PIPELINE first.")
        params['z'] = gcmd.get_float('Z', params.get('z', getattr(cam, 'def_z_plane', 0)))
        ux, uy = cam.get_mmpx_on_z(params['z'])
        upp = 0.5 * (abs(float(ux)) + abs(float(uy)))
        if upp < 1e-12:
            raise gcmd.error(f"Camera [{cam.name}] has unusable mm/px at Z={params['z']:g} (upp≈{upp:.5g}) — check PNP_CALIB_PXMM or config.")
        params['steps']= gcmd.get_int('STEPS', 16, minval=3)
        params['speed']= gcmd.get_float('SPEED', self.home_speed, above=0.)
        params['settle']= gcmd.get_float('SETTLE', 0.2, minval=0.)
        skip = self.get_size_params(gcmd, params, ['CAM', 'PIPELINE', 'Z', 'STEPS', 'SPEED','SETTLE'])
        params= self.size_params_from_mm(cam, params['z'], params, params['tol'])
        params['cx']=cam.cx
        params['cy']=cam.cy
        runtime = self.parse_runtime_params(gcmd, skip)
        params.update(runtime)
        gcmd.respond_info(f"params {params}")
        self.runout(cam, pipe_name, params, gcmd)

    def _seed_pitch_at_z(self, cam, c0, c1, z):
        x0, y0 = cam.pixels_to_mm_offset(float(c0['cx']), float(c0['cy']), z)
        x1, y1 = cam.pixels_to_mm_offset(float(c1['cx']), float(c1['cy']), z)
        return float(numpy.hypot(x1 - x0, y1 - y0))

    def _tune_z_to_pitch(self, cam, c0, c1, z_used, nom=4.0):
        """Z where the seed hole pair maps to EIA sprocket pitch."""
        p0 = self._seed_pitch_at_z(cam, c0, c1, z_used)
        if p0 < 0.2 or nom <= 0:
            return z_used, p0
        pair = None
        try:
            pair = cam._parse_px_per_mm(cam.px_per_mm)
        except Exception:
            pair = None
        if pair:
            zs = [pair[0][1], pair[1][1]]
            lo, hi = min(zs) - 10., max(zs) + 10.
        else:
            lo, hi = float(z_used) - 20., float(z_used) + 20.

        def err(z):
            return self._seed_pitch_at_z(cam, c0, c1, z) - nom

        elo, ehi = err(lo), err(hi)
        if elo * ehi > 0:
            best_z, best_e = z_used, abs(p0 - nom)
            z = lo
            while z <= hi + 1e-9:
                e = err(z)
                if abs(e) < best_e:
                    best_e, best_z = abs(e), z
                z += 0.5
            return best_z, p0
        a, b, fa = lo, hi, elo
        for _ in range(35):
            m = 0.5 * (a + b)
            fm = err(m)
            if abs(fm) < 1e-4:
                return m, p0
            if fa * fm <= 0:
                b = m
            else:
                a, fa = m, fm
        return 0.5 * (a + b), p0

    def analyze_tape(self, gcmd, cam, z, circles, rects, width=8):
        """
        Picks closest two circles and closest pocket to determine tape orientation
        than gathers inline circles and pockets and arrange them in order.
        objects are removed from source lists, calling this method again can detect other tapes in scene
        source needs to be sorted by proximity to optical center or POI
        """
        if not width in EIA_481_F_fol:
            raise gcmd.error("analyze_tape: Invalid tape width")
        if len(circles) < 2 or len(rects) < 1:
            gcmd.respond_info("len(circles) < 2 or len(rects) < 1")
            return None
        c0 = circles.pop(0)
        c0['mx'],c0['my'] = cam.pixels_to_mm_offset(float(c0['cx']), float(c0['cy']), z)
        c1 = None
        i=0
        while i<len(circles): #find second seed circle
            mx, my = cam.pixels_to_mm_offset(float(circles[i]['cx']), float(circles[i]['cy']), z)
            dist = numpy.hypot(c0['mx']-mx,c0['my']-my).astype(float)
            if 3.0 < dist < 5.0:
                c1=circles.pop(i)
                c1['mx'],c1['my'] = mx, my
                # gcmd.respond_info(f"match {dist} {c1}")
                break
            i+=1
        else:
            gcmd.respond_info("could not find seed holes")
            return None
        z_star, p_used = self._tune_z_to_pitch(cam, c0, c1, z, 4.0)
        c0['mx'], c0['my'] = cam.pixels_to_mm_offset(float(c0['cx']), float(c0['cy']), z_star)
        c1['mx'], c1['my'] = cam.pixels_to_mm_offset(float(c1['cx']), float(c1['cy']), z_star)
        base= numpy.hypot(c1['mx']-c0['mx'],c1['my']-c0['my']).astype(float)
        gcmd.respond_info(
            f"base pitch {p_used:.3f} at z={z:.3f} -> {base:.3f} at z={z_star:.3f} (nom 4.00)")
        z = z_star
        fmin, fmax = EIA_481_F_fol[width]
        i=0
        while i<len(rects): #find seed pocket
            mx, my = cam.pixels_to_mm_offset(float(rects[i]['cx']), float(rects[i]['cy']), z)
            area=get_signed_area_2d((c0['mx'],c0['my']),(c1['mx'],c1['my']),(mx,my))
            h=abs(area/base)
            if fmin < h < fmax:
                p0=rects[i]
                # gcmd.respond_info(f"seed pocket: {dist} {i} {area} {mx}, {my}")
                break
            i+=1
        else:
            gcmd.respond_info("could not find seed pocket")
            return None
        if area<0: #swap tape direction
            holes=[c1,c0]
            c0,c1=holes
            base= numpy.hypot(c1['mx']-c0['mx'],c1['my']-c0['my']).astype(float)
        else:
            holes=[c0,c1]
        ux = (c1['mx'] - c0['mx']) / base
        uy = (c1['my'] - c0['my']) / base
        tape_angle_rad = numpy.atan2(ux,uy).astype(float)
        c0['dist']=0.
        c1['dist']=float(base)
        c0['c0']=True; c1['c1']=True #mark them for draw_tape
        i=0
        while i<len(circles):
            mx, my = cam.pixels_to_mm_offset(float(circles[i]['cx']), float(circles[i]['cy']), z)
            area=get_signed_area_2d((c0['mx'],c0['my']),(c1['mx'],c1['my']),(mx,my))
            h=area/base
            if -0.8 < h <0.8:
                c=circles.pop(i)
                c['mx'], c['my'] = mx, my
                c['dist'] = float((mx - c0['mx']) * ux + (my - c0['my']) * uy)
                holes.append(c)
            else: i+=1
        pockets=[]
        holes.sort(key=lambda s: s["dist"])
        i=0
        while i<len(rects):
            mx, my = cam.pixels_to_mm_offset(float(rects[i]['cx']), float(rects[i]['cy']), z)
            area=get_signed_area_2d((c0['mx'],c0['my']),(c1['mx'],c1['my']),(mx,my))
            h=area/base
            if fmin < h < fmax:
                p=rects.pop(i)
                p['mx'], p['my'] = mx, my
                p['dist'] = float((mx - c0['mx']) * ux + (my - c0['my']) * uy)
                pockets.append(p)
            elif -0.8 < h <0.8:
                rects.pop(i)
            else: i+=1
        pockets.sort(key=lambda s: s["dist"])
        return {
            'pockets': pockets, 'holes': holes,
            'angle': float(tape_angle_rad), 'z': z,
            'ux': float(ux), 'uy': float(uy), 'width': width,
            'c0': c0, 'c1': c1, 'pitch': float(base),
        }

    def draw_tape(self, cam, pockets, holes, angle, z, tapenum=0):
        #TODO: need something more elegant than peeking into cam.worker
        overlay = getattr(cam.vision_worker_thread, 'frameoverlay', None)
        if overlay is None:
            return
        def make_px(x,y):
            dx,dy=cam.mm_to_px_delta(x,y, z)
            return (int(round(cam.cx - dx)), int(round(cam.cy + dy)))
        c0=[c for c in holes if c.get('c0',False)][0]
        c1=[c for c in holes if c.get('c1',False)][0]
        c0_px = make_px(c0['mx'],c0['my'])
        c1_px = make_px(c1['mx'],c1['my'])
        cv2.circle(overlay, c0_px, 50, (255, 255, 0), 3)
        cv2.circle(overlay, c1_px, 50, (255, 255, 0), 3)
        cv2.line(overlay, c0_px, c1_px, (255, 0, 0), 3)
        for i,p in enumerate(holes):
            cv2.circle(overlay, (int(p['cx']), int(p['cy'])), 40, (255, 255, 0), 3)
            cv2.putText(overlay, f"T{tapenum}-h{i}", (int(p['cx'] + 5), int(p['cy'] - 5)),cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 1)
        for i,p in enumerate(pockets):
            cv2.putText(overlay, f"T{tapenum}-p{i}", (int(p['cx'] + 5), int(p['cy'] - 5)),cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 1)
        cv2.putText(overlay, f"Tape{tapenum}: {numpy.degrees(angle):.1f} deg",  (c0_px[0] + 15, c0_px[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 1)

    def _median_span(self, items):
        if len(items) < 2:
            return None
        ds = sorted(it['dist'] for it in items)
        diffs = [ds[i + 1] - ds[i] for i in range(len(ds) - 1) if ds[i + 1] - ds[i] > 0.4]
        if not diffs:
            return None
        s = sorted(diffs)
        m = len(s) // 2
        return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])

    def _tape_across(self, tape, mx, my):
        c0 = tape['c0']
        ux, uy = tape['ux'], tape['uy']
        return float(ux * (my - c0['my']) - uy * (mx - c0['mx']))

    def generate_feeder_config(self, gcmd, cam, pos0, tape, count):
        """Extrapolate COUNT pockets on the EIA lattice and dump a config snippet."""
        nominal_pitch_mm=4.0
        holes = tape['holes']
        pockets = tape['pockets']
        z = tape['z']
        ux, uy = tape['ux'], tape['uy']
        c0 = tape['c0']
        width = tape.get('width', 8)
        f_nom = EIA_481_F[width]
        pk_meas = self._median_span(pockets)
        #snap to standard pocket pitch
        pk = min((2., 4., 8., 12., 16.), key=lambda g: abs(g - pk_meas))
        # EIA: first part is 2 mm along-tape from the reference hole when P1=4
        if pockets:
            d0 = pockets[0]['dist']
        elif holes:
            d0 = holes[0]['dist'] + 2.0
        else:
            return []
        # left of travel (CCW from axis) = +F
        hx, hy = -uy * f_nom, ux * f_nom
        extras = []
        for k in range(count):
            target = d0 + k * pk
            hit = None
            for p in pockets:
                if abs(p['dist'] - target) < 0.8:
                    hit = p
                    break
            if hit is not None:
                mx, my = hit['mx'], hit['my']
                extra = False
            else:
                mx = c0['mx'] + target * ux + hx
                my = c0['my'] + target * uy + hy
                extra = True
            extras.append({
                'dist': target, 'mx': mx, 'my': my,
                'x': pos0[0] + mx, 'y': pos0[1] + my,
                'extrapolated': extra, 'index': k,
            })
        rot = float(numpy.degrees(tape['angle']))
        pick = extras[0]
        name = tape.get('feeder') or ('tape%d' % tape.get('tapenum', 0))
        ftype = tape.get('feeder_type') or 'strip'
        gcmd.respond_info("config:  # paste into [pnp_feeder %s]" % (name,))
        gcmd.respond_info("  type: %s" % (ftype,))
        gcmd.respond_info("  tape_width: %d" % (width,))
        gcmd.respond_info("  part_count: %d" % (count,))
        gcmd.respond_info("  pocket_pitch: %.2f  # measured %s"% (pk, '—' if pk_meas is None else '%.3f' % pk_meas))
        gcmd.respond_info("  pocket0: {'x': %.4f, 'y': %.4f, 'z': %.3f, 'rot': %.2f}"% (pick['x'], pick['y'], z, rot))
        if len(holes) >= 2:
            gcmd.respond_info("  hole0: {'x': %.4f, 'y': %.4f}"% (pos0[0] + holes[0]['mx'], pos0[1] + holes[0]['my']))
            gcmd.respond_info("  hole1: {'x': %.4f, 'y': %.4f}"% (pos0[0] + holes[1]['mx'], pos0[1] + holes[1]['my']))
            gcmd.respond_info("  # axis=(%.4f, %.4f)" % (ux, uy))
        tape['pocket_pitch'] = pk
        return extras

    cmd_PNP_VISION_TAPE_help = (
        "Take a look at table and try to detect component tapes"
        "and match them to feeders or generate config snippet.")
    def cmd_PNP_VISION_TAPE(self, gcmd):
        cam = self.lookup_cam(gcmd.get('CAM', 'topcam'), False)
        pipe_name = gcmd.get('PIPELINE', 'visual_feeder')
        if pipe_name not in self.pipelines:
            raise gcmd.error(f"Pipeline '{pipe_name}' does not exist. Register with PNP_VISION_PIPELINE first.")
        speed = gcmd.get_float('SPEED', self.home_speed, above=0.)
        settle = gcmd.get_float('SETTLE', 0.2, minval=0.)
        count = gcmd.get_int('COUNT', 10)
        width = gcmd.get_int('WIDTH', 8)
        match_tol = gcmd.get_float('MATCH', 15.)
        params={} #maybe get something from user input?
        pos=self.pnp.get_position()
        x = gcmd.get_float('X', params.get('x', pos[0]))
        y = gcmd.get_float('Y', params.get('y', pos[1]))
        z = gcmd.get_float('Z', params.get('z', pos[2]))
        ux, uy = cam.get_mmpx_on_z(z)
        upp = 0.5 * (abs(float(ux)) + abs(float(uy)))
        if upp < 1e-12:
            raise gcmd.error(f"Camera [{cam.name}] has unusable mm/px at Z={z:g} (upp≈{upp:.5g}) — check PNP_CALIB_PXMM or config.")
        skip= self.get_size_params(gcmd, params, [
            'CAM', 'PIPELINE', 'FIDUCIAL', 'Z', 'X', 'Y',
            'SPEED', 'SETTLE', 'COUNT', 'WIDTH', 'MATCH'])
        params= self.size_params_from_mm(cam, z, params, params['tol'])
        params['cx']=cam.cx
        params['cy']=cam.cy
        runtime = self.parse_runtime_params(gcmd, skip)
        params.update(runtime)
        params['z'] = z
        self.pnp.move_and_settle(pos, speed, settle)
        result = self.run_detect(cam, pipe_name, gcmd, params)
        if not result['success']:
            raise gcmd.error("DETECT finished — no feature found.")
        if not result['kind']=='mixed':
            raise gcmd.error("DETECT finished — need both rects and circles detected.")
        circles=sorted(result['circles'], key=lambda c: numpy.hypot(params['cx']-c['cx'],params['cy']-c['cy']).astype(float))
        rects=sorted(result['rects'], key=lambda c: numpy.hypot(params['cx']-c['cx'],params['cy']-c['cy']).astype(float))
        gcmd.respond_info(f"circles={len(circles)} rects={len(rects)}")
        self.last_detection['tapes']=[]
        tapenum=0
        while len(circles) >= 2 and len(rects) >= 1:
            tape = self.analyze_tape(gcmd, cam, z, circles, rects, width=width)
            gcmd.respond_info(f"circles={len(circles)} rects={len(rects)}")
            if tape is None:
                gcmd.respond_info("tape_data is None")
                continue #current seed failed try again as it was popped,loop until we run out of circles
            tape['tapenum'] = tapenum
            self.draw_tape(cam, tape['pockets'], tape['holes'], tape['angle'], tape['z'], tapenum)
            self.last_detection['tapes'].append(tape)
            tapenum+=1
        self.last_detection['kind']='tape'
        tapes = self.last_detection['tapes']
        gcmd.respond_info(f"PNP_VISION_TAPE: Detected total of {len(tapes)} tapes in scene.")
        for tape in tapes:
            self.generate_feeder_config(gcmd, cam, pos, tape, count)

    def get_status(self, eventtime):
        return {
            'pipelines': list(self.pipelines.keys()),
            'last_detection': self.last_detection,
            'last_home': self.last_home,
        }

def load_config(config):
    return PnPVision(config)
