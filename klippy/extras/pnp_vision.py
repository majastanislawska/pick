# klippy/extras/pnp_vision.py
# Computer vision manager with dynamic vision pipeline support
#
# Copyright (C) 2026 Maja Stanislawska <maja@makershop.ie>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import math
from .pnp_vision_pipeline import PnpVisionPipeline

class PnPVision:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
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
        self.home_tolerance = config.getfloat('homing_tolerance', 0.05)
        self.home_max_iter = config.getint('homing_max_iter', 8)

        self.gcode.register_command(
            'PNP_VISION_PIPELINE', self.cmd_PNP_VISION_PIPELINE,
            desc=self.cmd_PNP_VISION_PIPELINE_help)
        self.gcode.register_command(
            'PNP_VISION_DETECT', self.cmd_PNP_VISION_DETECT,
            desc=self.cmd_PNP_VISION_DETECT_help)
        self.gcode.register_command(
            'PNP_VISION_LIST', self.cmd_PNP_VISION_LIST,
            desc=self.cmd_PNP_VISION_LIST_help)

    def register_camera(self, name, cam):
        logging.info(f"register_camera: {name} {cam.resolution}")
        self.cams[name.upper()] = cam

    def lookup_cam(self, cam_name):
        key = cam_name.upper()
        if key not in self.cams:
            raise self.printer.command_error(
                f"Unknown camera '{cam_name}'. Known: {list(self.cams.keys())}")
        return self.cams[key]

    def other_cameras(self,me):
        return [c for c in self.cams.values() if c is not me]

    def _lookup_fiducial(self, name='primary'):
        """Return (name, dict) for fiducial from [pnp] config."""
        if name is None:
            return None, {}
        pnp = self.printer.lookup_object('pnp')
        key = (name or 'primary').strip().lower()
        if key in ('primary', '1', 'a', 'p'):
            return 'primary', pnp.fiducial_primary
        if key in ('secondary', '2', 'b', 's'):
            return 'secondary', pnp.fiducial_secondary
        raise self.printer.command_error(
            f"Unknown FIDUCIAL='{name}'. Use primary or secondary.")

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

    def _size_params_from_mm(self, cam, z, params, tol=0.1):
        """
        Convert physical sizes (mm) at z_plane → pipeline params (px).

        Uses average |mm/px| from cam.mm_per_px (same as calib/home).
        Circle: dia → r, min_r, max_r, rr, dia_px
        Rect:   width/height → w_px, h_px, min/max_width_px, aspect band

        Returns {} if mm_per_px missing or no sizes given.
        """
        if z is None:
            return {}
        ux, uy = cam.get_mmpx_on_z(z)
        upp = 0.5 * (abs(ux) + abs(uy))
        if upp < 1e-12:
            return {}
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
        return auto

    def run_detect(self, cam, pipe_name, gcmd, runtime_params=None):
        """
        Run a named pipeline on a camera snapshot.
        Returns a result dict; also updates self.last_detection.
        Result shapes (no silent multi→single pick — filter in the pipeline):
          - exactly 1 circle, 0 rects → flat {kind:'circle', cx, cy, r, score}
          - exactly 1 rect, 0 circles → flat {kind:'rect', cx, cy, w, h, angle, ratio}
          - multi / mixed → {kind:'circles'|'rects'|'mixed', lists via todict()
        mixed is intentional for tape: sprocket holes (circles) + pocket (rect) in one shot.
        Consumers that need a single feature use ensure_singular_result.
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
        r = {
            'success': ret,
            'pipeline': pipe_name,
            'cam': cam.name,
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
        fid_name, params = self._lookup_fiducial(gcmd.get('FIDUCIAL', None))
        #now update with overrides from commandline
        z = gcmd.get_float('Z', params.get('z', getattr(cam, 'def_z_plane', None)))
        params['dia'] = gcmd.get_float('DIA', params.get('dia', None))
        params['w'] = gcmd.get_float('W', params.get('w', None))
        params['h'] = gcmd.get_float('H', params.get('h', None))
        size_tol = gcmd.get_float('TOL', params.get('tol', 0.1), minval=0.)
        params= self._size_params_from_mm(cam, z, params, size_tol)
        if not params:
            gcmd.respond_info(f"Coversion from mm yielded nothing")
            params={}
        params['cx']=cam.cx
        params['cy']=cam.cy
        runtime = self.parse_runtime_params(gcmd,
            skip=['CAM', 'PIPELINE', 'FIDUCIAL', 'Z', 'DIA', 'W', 'H',
                   'TOL', 'LIGHT'])
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

    def get_status(self, eventtime):
        return {
            'pipelines': list(self.pipelines.keys()),
            'last_detection': self.last_detection,
        }

def load_config(config):
    return PnPVision(config)
