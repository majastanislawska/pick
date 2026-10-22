# klippy/extras/pnp_calibration.py
# Interactive calibration for camera scale / pose (bed_screws-style session).
#
# Copyright (C) 2026 Maja Stanislawska <maja@makershop.ie>
# (Math curtesy of Grok)
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import math
import numpy
import cv2


def _mean_stdev(values):
    n = len(values)
    if n == 0:
        return None, None
    mean = sum(values) / float(n)
    if n == 1:
        return mean, 0.
    var = sum((v - mean) ** 2 for v in values) / float(n - 1)
    return mean, math.sqrt(var)

def _fmt_A(A):
    ((a, b), (c, d)) = A
    return f"(({a:.8g}, {b:.8g}), ({c:.8g}, {d:.8g}))"

def _fmt_px_per_mm(ppm):
    (A1, z1), (A2, z2) = ppm
    return (f"px_per_mm: (({_fmt_A(A1)}, {z1:.6g}), ({_fmt_A(A2)}, {z2:.6g}))")

def _update_px_per_mm(cam, A, z):
    A = numpy.array(A, dtype=numpy.float64).reshape(2, 2)
    sample = (A.tolist(), float(z))
    if getattr(cam, 'px_per_mm', None) is None:
        cam.px_per_mm = (sample, sample)
        return cam.px_per_mm
    try:
        a, b = cam.px_per_mm[0], cam.px_per_mm[1]
        z_a, z_b = float(a[1]), float(b[1])
    except Exception:
        cam.px_per_mm = (sample, sample)
        return cam.px_per_mm
    eps = 1e-3
    if abs(z_a - sample[1]) < eps:
        cam.px_per_mm = (sample, b if abs(z_b - sample[1]) >= eps else sample)
    elif abs(z_b - sample[1]) < eps:
        cam.px_per_mm = (a, sample)
    elif abs(z_a - z_b) < eps:
        cam.px_per_mm = (a, sample)
    elif abs(z_a - sample[1]) <= abs(z_b - sample[1]):
        cam.px_per_mm = (sample, b)
    else:
        cam.px_per_mm = (a, sample)
    return cam.px_per_mm

def _upp_from_A(A):
    A = numpy.asarray(A, dtype=numpy.float64)
    hx = math.hypot(float(A[0, 0]), float(A[1, 0]))
    hy = math.hypot(float(A[0, 1]), float(A[1, 1]))
    if hx < 1e-12 or hy < 1e-12:
        return None, None
    return 1. / hx, 1. / hy

def _pixel_grid(w, h, pstep, margin=0., cx0=None, cy0=None):
    """Cell-center pixel grid. PSTEP=50 on 1600×1200 → 32×24."""
    pstep = float(pstep)
    margin = float(margin)
    x0, x1 = margin, float(w) - margin
    y0, y1 = margin, float(h) - margin
    if x1 <= x0 or y1 <= y0:
        raise ValueError("MARGIN=%.0f leaves no grid in %dx%d" % (margin, w, h))
    nx = max(1, int(round((x1 - x0) / pstep)))
    ny = max(1, int(round((y1 - y0) / pstep)))
    xs = [x0 + (x1 - x0) * (i + 0.5) / nx for i in range(nx)]
    ys = [y0 + (y1 - y0) * (j + 0.5) / ny for j in range(ny)]
    pts = []
    for j, y in enumerate(ys):
        row = [(x, y) for x in xs]
        if j % 2:
            row.reverse()
        pts.extend(row)
    if cx0 is not None and cy0 is not None and pts:
        i = min(range(len(pts)),
                key=lambda k: math.hypot(pts[k][0] - cx0, pts[k][1] - cy0))
        pts = pts[i:] + pts[:i]
    return pts


def _rosette_pattern(step, n, rings=1, include_center=False):
    n = int(n)
    step = float(step)
    rings = max(1, int(rings))
    if n == 4:
        unit = [(1., 0.), (-1., 0.), (0., 1.), (0., -1.)]
    elif n == 8:
        unit = [(1., 0.), (-1., 0.), (0., 1.), (0., -1.),
                (1., 1.), (1., -1.), (-1., 1.), (-1., -1.)]
    else:
        unit = [(math.cos(2. * math.pi * i / n),
                 math.sin(2. * math.pi * i / n)) for i in range(n)]
    out = []
    if include_center:
        out.append((0., 0.))
    for r in range(1, rings + 1):
        s = step * r
        out.extend((s * ux, s * uy) for ux, uy in unit)
    return out

class CalibWizard:
    """One exclusive interactive session. Owns SAMPLE / DONE / ABORT for
    its lifetime so a later toolchanger (or paper-disc) wizard can use
    the same slot without PnPCalibration knowing the verbs."""
    kind = None

    def __init__(self, host, gcmd):
        self.host = host
        self.printer = host.printer
        self.gcode = host.gcode
        self.gcmd = gcmd
        self.pnp = host.pnp
        self.vision = host.vision
        self._helpers = []

    def attach(self):
        registered = []
        try:
            for name, func, desc in (
                    ('SAMPLE', self.cmd_SAMPLE, self.cmd_SAMPLE_help),
                    ('DONE', self.cmd_DONE, self.cmd_DONE_help),
                    ('ABORT', self.cmd_ABORT, self.cmd_ABORT_help)):
                self.gcode.register_command(name, func, desc=desc)
                registered.append(name)
        except self.printer.config_error:
            for name in registered:
                self.gcode.register_command(name, None)
            raise self.printer.command_error(
                "SAMPLE/DONE/ABORT already in use — finish the other wizard")
        self._helpers = registered

    def detach(self):
        for name in self._helpers:
            self.gcode.register_command(name, None)
        self._helpers = []

    def status(self):
        return {'is_active': True, 'kind': self.kind,
                'cam': None, 'samples': 0, 'heights': []}

    def prompt(self, gcmd):
        raise NotImplementedError

    def sample(self, gcmd):
        raise NotImplementedError

    def finish(self, gcmd):
        raise NotImplementedError

    cmd_SAMPLE_help = "Take a calibration sample at the current pose"
    def cmd_SAMPLE(self, gcmd):
        self.sample(gcmd)
        self.prompt(gcmd)

    cmd_DONE_help = "Finish this wizard and print paste-able results"
    def cmd_DONE(self, gcmd):
        try:
            self.finish(gcmd)
        finally:
            self.host.end_session()

    cmd_ABORT_help = "Abort this wizard (no further commit)"
    def cmd_ABORT(self, gcmd):
        self.host.end_session()
        gcmd.respond_info("%s aborted" % (self.kind or 'calibration',))


class CalibSession(CalibWizard):
    kind = None

    def __init__(self, host, cam, pipe_name, gcmd, skip):
        CalibWizard.__init__(self, host, gcmd)
        self.reactor = host.reactor
        self.cam = cam
        self.pipe_name = pipe_name
        self.skip=skip
        self.speed = gcmd.get_float('SPEED', host.calib_speed, above=0.)
        self.settle = gcmd.get_float('SETTLE', 0.15, minval=0.)
        self.accuracy = gcmd.get_float('ACCURACY', host.vision.pinpoint_accuracy, minval=0.)
        self.max_tries = gcmd.get_int('MAX_TRIES', host.vision.max_tries, minval=1)
        self.skip.extend(['CAM', 'PIPELINE', 'SPEED', 'SETTLE', 'ACCURACY',' MAX_TRIES'])
        # self.params = host.vision.parse_runtime_params(gcmd, skip=skip)
        # self.partial = None
        self.groups = []

    def status(self):
        return {
            'is_active': True,
            'kind': self.kind,
            'cam': self.cam.name,
            'samples': 0,
            'heights': [],
        }

    def move_to(self, x, y, z, r,speed, settle):
        pos = self.pnp.get_position()
        if x is not None:
            pos[0] = float(x)
        if y is not None:
            pos[1] = float(y)
        if z is not None:
            pos[2] = float(z)
        if r is not None:
            pos[self.pnp.r_index()] = float(r)
        try:
            self.pnp.move_and_settle(pos, speed,settle)
        except self.printer.command_error:
            self.host.end_session()
            raise

    def move_xy(self, x, y, speed, settle):
        self.move_to(x, y, None, None, speed, settle)

    def sample_params(self, gcmd):
        fid_name, fid = self.pnp.lookup_fiducial(gcmd.get('FIDUCIAL', None))
        params=fid.copy()
        z = gcmd.get_float('Z', params.get('z', self.cam.z_plane))
        if z is None:
            raise gcmd.error("SAMPLE needs Z=<plane> or FIDUCIAL=primary|secondary")
        skip= self.vision.get_size_params(gcmd, params, ['FIDUCIAL', 'Z'])
        params= self.vision.size_params_from_mm(self.cam, z, params, params['tol'])
        params['cx']=self.cam.cx
        params['cy']=self.cam.cy
        runtime = self.vision.parse_runtime_params(gcmd, skip)
        params.update(runtime)
        return float(z), fid_name, params

    # def _predict_px(self, z, dx, dy, cx0, cy0):
    #     A = None
    #     part = self.partial
    #     if part is not None and abs(float(part.get('z', z)) - float(z)) < 1e-3:
    #         if part.get('A') is not None:
    #             A = numpy.array(part['A'], dtype=numpy.float64)
    #     if A is None:
    #         A = self.cam.get_affine_on_z(z)
    #     if A is None:
    #         return None
    #     d = A.dot([float(dx), float(dy)])
    #     return float(cx0) + float(d[0]), float(cy0) + float(d[1])

    def _optical_dist(self, g, fx):
        return float(fx) * 0.5 * (float(g['upp_x']) + float(g['upp_y']))

class PxMmSession(CalibSession):
    kind = 'pxmm'

    def __init__(self, host, cam, pipe_name, gcmd):
        super().__init__(host, cam, pipe_name, gcmd, ['STEP','N'])
        self.step = gcmd.get_float('STEP', 8., above=0.)
        self.n = gcmd.get_int('N', 4)
        if self.n not in (4, 8):
            raise gcmd.error("N must be 4 (cardinal) or 8 (+diagonals).")

    def status(self):
        d = super().status()
        d['heights'] = [h['z'] for h in self.groups]
        d['samples'] = len(self.groups)
        return d

    def prompt(self, gcmd):
        if getattr(self.cam, 'looking_up', False):
            gcmd.respond_info(
                "PXMM session [%s] bottom. Nozzle over the camera.\n"
                "  SAMPLE Z=<nozzle height> moves Z, then a rosette (scale).\n"
                "  Second SAMPLE at another Z for scale vs height. DONE / ABORT.\n"
                "  Tool û/offset: PNP_CALIB_ALIGN after the camera is calibrated.\n"
                "  heights so far: %d  %s"
                % (self.cam.name, len(self.groups),
                   [h['z'] for h in self.groups]))
            return
        gcmd.respond_info(
            "PXMM session [%s]. Jog over a fiducial (toolhead Z = camera height).\n"
            "  SAMPLE Z=<plane>   or   SAMPLE FIDUCIAL=primary|secondary\n"
            "  then DONE (one height is ok) or SAMPLE another height. ABORT cancels.\n"
            "  heights so far: %d  %s"
            % (self.cam.name, len(self.groups),
               [h['z'] for h in self.groups]))

    def sample(self, gcmd):
        z, fid_name, params = self.sample_params(gcmd)
        if getattr(self.cam, 'looking_up', False): #false topcam true bottom
            pos = self.pnp.get_position()
            gcmd.respond_info(f"bottomcam: moving nozzle Z {pos[2]:.4f} → {z:g}")
            self.move_to(pos[0], pos[1], z, 0, self.speed, 0.)
        elif 'x' in params and 'y' in params:
            gcmd.respond_info(f"Moving to {fid_name}")
            self.move_to(params['x'], params['y'], z, 0, self.speed, 0.)
        kept = []
        for h in self.groups:
            if abs(h['z'] - z) < 1e-3:
                gcmd.respond_info(f"replacing previous sample at Z={z:g}")
            else:
                kept.append(h)
        self.groups = kept
        upp_x, upp_y, A,views = self._rosette(gcmd, z, params)
        self.commit_affine(gcmd, z, A)
        self.groups.append({'z': float(z), 'upp_x': upp_x, 'upp_y': upp_y, 'A': A, 'views': views})

    def _commit_z_from_scale(self, gcmd):
        """PXMM: Topcam → camera_z; bottomcam → camera_z. Uses fx from K."""
        cam = self.cam
        try:
            fx = float(cam.camera_matrix[0, 0])
        except Exception:
            return
        if fx < 50.:
            return
        dists = []
        for g in self.groups:
            if g.get('upp_x') is None:
                continue
            d = self._optical_dist(g, fx)
            dists.append((float(g['z']), d))
            gcmd.respond_info(
                f"Z={g['z']:g} optical dist≈{d:.3f}mm (fx·upp, fx={fx:.4g})")
        if len(dists) < 1:
            return
        if len(dists) < 2:
            gcmd.respond_info("pinhole needs two SAMPLE heights (fx is a placeholder until CAMERA)")
            return
        zs = numpy.array([z for z, _d in dists], dtype=numpy.float64)
        us = numpy.array(
            [0.5 * (g['upp_x'] + g['upp_y'])
             for g in self.groups if g.get('upp_x') is not None],
            dtype=numpy.float64)
        if len(us) < 2:
            return
        du = float(numpy.max(us) - numpy.min(us))
        dz = float(numpy.max(zs) - numpy.min(zs))
        if dz < 0.5 or du / max(float(numpy.mean(us)), 1e-9) < 0.05:
            gcmd.respond_info(
                "pinhole not written: scale barely changed over ΔZ=%.2fmm" % dz)
            return
        A = numpy.column_stack([zs, numpy.ones_like(zs)])
        (a, b), *_ = numpy.linalg.lstsq(A, us, rcond=None)
        if abs(a) < 1e-12:
            return
        mount = float(-b / a)
        cam.camera_z = mount
        gcmd.respond_info(f"camera_z={mount:.4f} (upp intercept)")

    def finish(self, gcmd):
        if not self.groups:
            gcmd.respond_info("PXMM: no heights sampled.")
            return
        self._commit_z_from_scale(gcmd)
        gcmd.respond_info(f"PXMM done heights={len(self.groups)}")
        gcmd.respond_info(f"Paste into [{self.cam.name}] section of machine.cfg:")
        gcmd.respond_info(f"camera_z: {self.cam.camera_z:.4f})")
        if getattr(self.cam, 'px_per_mm', None) is not None:
            gcmd.respond_info(_fmt_px_per_mm(self.cam.px_per_mm))

    def _rosette(self, gcmd, z, params):
        n, step = self.n, float(self.step)
        offsets = _rosette_pattern(step, n, rings=1, include_center=False)
        x0, y0 = self.pnp.get_position()[:2]
        gcmd.respond_info(f"rosette STEP={step:g}mm N={n} center=({x0:.4f},{y0:.4f}) Z_tag={z:g}")
        legs = []
        views = []
        try:
            for i, (dx, dy) in enumerate(offsets):
                label = f"leg[{i+1}/{n}] Δ=({dx:g},{dy:g})"
                self.move_xy(x0, y0, self.speed, self.settle)
                params['cx'], params['cy'] = self.cam.cx, self.cam.cy
                r1 = self.vision.run_detect(self.cam, self.pipe_name, gcmd, params)
                cx1, cy1 = self.vision.ensure_singular_result(r1, gcmd, f"{label} center")
                gcmd.respond_info(f"{label} center px=({cx1:.3f},{cy1:.3f})")
                self.move_xy(x0 + dx, y0 + dy, self.speed, self.settle)
                # pred = self._predict_px(z, dx, dy, cx1, cy1)
                # if pred is None:
                #     params['cx'], params['cy'] = self.cam.cx, self.cam.cy
                #     gcmd.respond_info(f"{label} expect px not yet")
                # else:
                #     params['cx'], params['cy'] = pred
                #     gcmd.respond_info(f"{label} expect px≈({params['cx']:.1f},{params['cy']:.1f})")
                r2 = self.vision.run_detect(self.cam, self.pipe_name, gcmd, params)
                cx2, cy2 = self.vision.ensure_singular_result(r2, gcmd, f"{label} offset")
                dcx, dcy = float(cx2) - float(cx1), float(cy2) - float(cy1)
                d_px = math.hypot(dcx, dcy)
                gcmd.respond_info(f"{label} offset px=({cx2:.3f},{cy2:.3f}) d_px={d_px:.3f}")
                # if d_px < self.min_px:
                #     raise gcmd.error(f"{label}: |d_px|={d_px:.2f} < MIN_PX={self.min_px}")
                legs.append((float(dx), float(dy), dcx, dcy))
                views.append({'x': float(x0),     'y': float(y0),       'cx': float(cx1), 'cy': float(cy1), 'z': float(z)})
                views.append({'x': float(x0 + dx), 'y': float(y0 + dy), 'cx': float(cx2), 'cy': float(cy2), 'z': float(z)})
                # A_part = self._fit_affine_delta(legs)
                # if A_part is not None:
                #     self.partial = {'z': float(z), 'A': A_part.tolist()}
        finally:
            self.move_xy(x0, y0, self.speed, 0.)
        A = self._fit_affine_delta(legs)
        if A is None:
            raise gcmd.error(f"rosette incomplete: {len(legs)} legs, need two non-collinear")
        errs = []
        for dx, dy, dcx, dcy in legs:
            p = A.dot([dx, dy])
            errs.append(math.hypot(p[0] - dcx, p[1] - dcy))
        rms = math.sqrt(sum(e * e for e in errs) / float(len(errs)))
        upp_x, upp_y = _upp_from_A(A)
        rot = math.degrees(math.atan2(float(A[1, 0]), float(A[0, 0])))
        gcmd.respond_info(
            f"rosette summary STEP={step:g} N={n}: A={_fmt_A(A.tolist())} "
            f"rot≈{rot:.3f}° rms={rms:.2f}px  "
            f"upp_x={upp_x:.8g} upp_y={upp_y:.8g}")
        return upp_x, upp_y, A, views

    def _fit_affine_delta(self, legs):
        """Least-squares 2×2 A: (dcx, dcy) = A @ (dx, dy). Needs two non-collinear legs."""
        rows, y = [], []
        for dx, dy, dcx, dcy in legs:
            dx, dy = float(dx), float(dy)
            if math.hypot(dx, dy) < 1e-9:
                continue
            rows.append([dx, dy, 0., 0.])
            y.append(float(dcx))
            rows.append([0., 0., dx, dy])
            y.append(float(dcy))
        if len(y) < 4:
            return None
        X = numpy.asarray(rows, dtype=numpy.float64)
        try:
            if numpy.linalg.matrix_rank(X, tol=1e-9) < 4:
                return None
            coef, *_ = numpy.linalg.lstsq(X, numpy.asarray(y, dtype=numpy.float64), rcond=None)
        except Exception:
            return None
        A = numpy.array([[coef[0], coef[1]], [coef[2], coef[3]]], dtype=numpy.float64)
        if abs(float(numpy.linalg.det(A))) < 1e-12:
            return None
        return A

    def commit_affine(self, gcmd, z, A):
        # self.partial = None
        A = numpy.array(A, dtype=numpy.float64).reshape(2, 2)
        upp_x, upp_y = _upp_from_A(A)
        if upp_x is None:
            raise gcmd.error("affine has a degenerate column")
        ppm = _update_px_per_mm(self.cam, A, z)
        rot = math.degrees(math.atan2(float(A[1, 0]), float(A[0, 0])))
        gcmd.respond_info(
            f"[{self.cam.name}] sample at Z={z:g}: A={_fmt_A(A.tolist())} "
            f"rot≈{rot:.3f}° upp=({upp_x:.8g}, {upp_y:.8g})")
        gcmd.respond_info("in-memory px_per_mm updated.")
        gcmd.respond_info(
            f"paste into camera section in printer.cfg:\n{_fmt_px_per_mm(ppm)}")
        if abs(ppm[0][1] - ppm[1][1]) < 1e-3:
            gcmd.respond_info("note: only one unique Z so far — SAMPLE a second height for Z-interpolation of scale.")
        return ppm

class CameraSession(CalibSession):
    kind = 'camera'
    def __init__(self, host, cam, pipe_name, gcmd):
        super().__init__(host, cam, pipe_name, gcmd, ['PSTEP','MARGIN'])
        self.gcmd.respond_info("Starting CAMERA session")
        self.pstep = gcmd.get_float('PSTEP', 50., above=0.)
        self.margin = gcmd.get_float('MARGIN', 0., minval=0.)

    def status(self):
        d = super().status()
        d['heights'] = [g['z'] for g in self.groups]
        d['samples'] = sum(len(g.get('views', [])) for g in self.groups)
        return d

    def finish(self, gcmd):
        if not self.groups:
            raise gcmd.error("CAMERA: no SAMPLE yet. SAMPLE a plane or ABORT.")
        if len(self.groups) < 2:
            raise gcmd.error("CAMERA: Need at least two samples. SAMPLE a plane or ABORT.")
        self._commit_matrices(gcmd)

    def _collect_views(self, gcmd, z, params, home=True):
        cam = self.cam
        if cam.px_to_mm_delta(1., 0., z) is None:
            raise gcmd.error("CAMERA pixel grid needs px_per_mm — run PNP_CALIB_PXMM first")
        startpos = self.pnp.get_position()
        w, h = (cam.frame_width, cam.frame_height)
        margin = self.margin
        params['cx'] = cam.cx
        params['cy'] = cam.cy
        if home:
            ret = self.vision.pinpoint_target(
                gcmd, startpos, cam, self.pipe_name, self.speed, self.settle,
                self.max_tries, self.accuracy, params, tag="calib_origin")
            if not ret['success']:
                raise gcmd.error("Could not converge at origin: %s" % ret)
            x0, y0, z0 = ret['pos'][:3]
            cx_ref = float(ret.get('px', cam.cx))
            cy_ref = float(ret.get('py', cam.cy))
        else:
            gcmd.respond_info(
                "skipping origin home — same XY as previous SAMPLE so the "
                "nozzle can walk in pixels with Z")
            x0, y0, z0 = startpos[0], startpos[1], startpos[2]
            try:
                result = self.vision.run_detect(cam, self.pipe_name, gcmd, params)
                cx_ref, cy_ref = self.vision.ensure_singular_result(
                    result, gcmd, "calib_origin")
            except Exception as e:
                gcmd.respond_info("origin detect failed (%s); using virt PP" % e)
                cx_ref, cy_ref = float(cam.cx), float(cam.cy)
        try:
            targets = _pixel_grid(w, h, self.pstep, margin, cx0=cam.cx, cy0=cam.cy)
        except ValueError as e:
            raise gcmd.error(str(e))
        gcmd.respond_info(
            f"camera pixel grid PSTEP={self.pstep:g} MARGIN={margin:g} "
            f"{w}x{h} → {len(targets)} cells "
            f"origin px=({cx_ref:.1f},{cy_ref:.1f}) "
            f"xy=({x0:.4f},{y0:.4f}) Z={z:g}")
        views = [{
            'x': float(x0), 'y': float(y0),
            'th_z': float(z0), 'z': float(z),
            'cx': float(cx_ref), 'cy': float(cy_ref),
            'tx': float(cam.cx), 'ty': float(cam.cy),
        }]
        gcmd.respond_info(f"view[0/{len(targets)}] origin th=({x0:.4f},{y0:.4f}) px=({cx_ref:.3f},{cy_ref:.3f})")
        try:
            for i, (tx, ty) in enumerate(targets):
                label = f"view[{len(views)+1}/{len(targets)}] px→({tx:.0f},{ty:.0f})"
                dx, dy = cam.px_to_mm_delta(tx - cx_ref, ty - cy_ref, z)
                try:
                    self.move_xy(x0 + dx, y0 + dy, self.speed, self.settle)
                except self.printer.command_error as e:
                    gcmd.respond_info(f"{label} skip move: {e}")
                    continue
                params['cx'], params['cy'] = float(tx), float(ty)
                gcmd.respond_info(f"{label} expect px≈({tx:.1f},{ty:.1f}) Δxy=({dx:.4f},{dy:.4f})")
                try:
                    result = self.vision.run_detect(
                        cam, self.pipe_name, gcmd, params)
                    cx, cy = self.vision.ensure_singular_result(
                        result, gcmd, label)
                except Exception as e:
                    gcmd.respond_info(f"{label} skip: {e}")
                    continue
                pos = self.pnp.get_position()
                views.append({'x': float(pos[0]), 'y': float(pos[1]), 'th_z': float(pos[2]), 'z': float(z),
                    'cx': float(cx), 'cy': float(cy), 'tx': float(tx), 'ty': float(ty),})
                # fit = self._fit_affine(views)
                # if fit is not None:
                #     self.partial = {'z': float(z), 'A': fit['A']}
                gcmd.respond_info(f"{label} th=({pos[0]:.4f},{pos[1]:.4f}) px=({cx:.3f},{cy:.3f})")
        finally:
            try:
                self.move_to(x0, y0, z0, 0, self.speed, 0)
            except Exception:
                logging.exception("camera grid return to origin failed")
        return views

    def _solve_group(self, gcmd, z, views):
        group={'z':z, 'views':views}
        fit = self._fit_affine(views)
        if fit is not None:
            gcmd.respond_info(
                f"Z={z:g} affine px←xy  A={_fmt_A(fit['A'])} t={fit['t']} "
                f"rot≈{fit['rot']:.3f}° rms={fit['rms']:.2f}px")
        group['fit'] = fit
        nview = len(views)
        ok = nview >= 8 and fit is not None
        group['ok'] = ok
        if not ok:
            raise gcmd.error(
                f"Z={z:g}: group rejected n={nview} "
                f"rms={None if fit is None else '%.1fpx' % fit['rms']} "
                f"(need n>=8 and an affine). "
                f"Center the mark and SAMPLE again — not committed.")
        if fit is not None and fit['rms'] < 20.:
            upp_x, upp_y = fit['upp_x'], fit['upp_y']
        else:
            raise gcmd.error(f"Z={z:g}: {nview} views but no usable motion.")
        group['upp_x'], group['upp_y'] = upp_x, upp_y
        pose = self._pose_from_affine(self.cam, fit) if fit else None
        if pose is None:
            px, py = [], []
            for v in views:
                px.append(v['x'] + (float(v['cx']) - self.cam.cx) * float(upp_x))
                py.append(v['y'] + (float(v['cy']) - self.cam.cy) * float(upp_y))
            mx, sx = _mean_stdev(px)
            my, sy = _mean_stdev(py)
        else:
            mx, my = pose
            sx = sy = fit['rms'] * 0.5 * (upp_x + upp_y)
        group['est_x'],  group['est_y']  = mx, my
        group['est_sx'], group['est_sy'] = sx, sy
        group['th_z'] = sum(v['th_z'] for v in views) / float(len(views))
        return group

    def _fit_affine(self, views):
        n = len(views)
        if n < 3:
            return None
        X = numpy.zeros((2 * n, 6), dtype=numpy.float64)
        y = numpy.zeros(2 * n, dtype=numpy.float64)
        for i, p in enumerate(views):
            X[2*i, 0] = p['x']
            X[2*i, 1] = p['y']
            X[2*i, 4] = 1.
            X[2*i+1, 2] = p['x']
            X[2*i+1, 3] = p['y']
            X[2*i+1, 5] = 1.
            y[2*i] = p['cx']
            y[2*i+1] = p['cy']
        try:
            coef, *_ = numpy.linalg.lstsq(X, y, rcond=None)
        except Exception:
            return None
        a, b, c, d, tx, ty = (float(v) for v in coef)
        hx, hy = math.hypot(a, c), math.hypot(b, d)
        if hx < 1e-12 or hy < 1e-12:
            return None
        pred = X.dot(coef)
        rms = float(numpy.sqrt(numpy.mean((pred - y) ** 2)))
        rot = numpy.degrees(numpy.atan2(c, a)).astype(float) if abs(a) + abs(c) > 1e-12 else 0.
        return {
            'A': [[a, b], [c, d]], 't': (tx, ty),
            'upp_x': 1.0 / hx, 'upp_y': 1.0 / hy, 'rot': rot, 'rms': rms,
        }

    def _pose_from_affine(self, cam, fit):
        A = numpy.array(fit['A'], dtype=numpy.float64)
        t = numpy.array(fit['t'], dtype=numpy.float64)
        try:
            det = float(numpy.linalg.det(A))
        except Exception:
            return None
        if abs(det) < 1e-12:
            return None
        th = numpy.linalg.inv(A).dot(numpy.array([cam.cx, cam.cy]) - t)
        return float(th[0]), float(th[1])

    def _rectification_from_affine(self, A):
        A = numpy.array(A, dtype=numpy.float64)
        U, _S, Vt = numpy.linalg.svd(A)
        R2 = U.dot(Vt)
        if numpy.linalg.det(R2) < 0:
            R2 = numpy.array([[-1., 0.], [0., 1.]], dtype=numpy.float64).dot(R2)
        R = numpy.eye(3, dtype=numpy.float64)
        R[0:2, 0:2] = R2
        return R

    def _K_from_planes(self, good, w, h):
        def _fx_from_scale(cam):
            """Pinhole fx in px from PXMM upp(Z). Independent of CAMERA group affine."""
            ppm = getattr(cam, 'px_per_mm', None)
            if ppm is None:
                return None
            try:
                z1, z2 = float(ppm[0][1]), float(ppm[1][1])
            except Exception:
                return None
            dz = z2 - z1
            if abs(dz) < 0.5:
                return None
            u1x, u1y = cam.get_mmpx_on_z(z1)
            u2x, u2y = cam.get_mmpx_on_z(z2)
            u1 = 0.5 * (abs(float(u1x)) + abs(float(u1y)))
            u2 = 0.5 * (abs(float(u2x)) + abs(float(u2y)))
            du = u2 - u1
            if abs(du) < 1e-6:
                return None
            fx = abs(dz / du)
            if not (200. < fx < 8000.):
                return None
            return float(fx)
        g0 = min(good, key=lambda g: g['fit']['rms'])
        A = numpy.array(g0['fit']['A'], dtype=numpy.float64)
        t = numpy.array(g0['fit']['t'], dtype=numpy.float64)
        F = numpy.array([g0['est_x'], g0['est_y']], dtype=numpy.float64)
        pp = A.dot(F) + t
        cx, cy = float(pp[0]), float(pp[1])
        fx =None
        # fx = _fx_from_scale(self.cam)
        if fx is None:
            fx = float(max(w, h))
        K = numpy.array(
            [[fx, 0., cx], [0., fx, cy], [0., 0., 1.]], dtype=numpy.float64)
        return K, A, t

    def _calibrate_camera(self, K):
        obj_pts, img_pts = [], []
        for g in self.groups:
            if not g.get('ok') or g.get('est_x') is None:
                continue
            fx, fy = float(g['est_x']), float(g['est_y'])
            o, p = [], []
            for v in g['views']:
                o.append([v['x'] - fx, v['y'] - fy, 0.0])
                p.append([v['cx'], v['cy']])
            if len(o) < 6:
                continue
            obj_pts.append(numpy.asarray(o, dtype=numpy.float32))
            img_pts.append(numpy.asarray(p, dtype=numpy.float32))
        # assert obj_pts != []
        flags = (cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO
                 | cv2.CALIB_FIX_FOCAL_LENGTH | cv2.CALIB_FIX_PRINCIPAL_POINT)
                #  | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3)
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 80, 1e-7)
        rms, Kf, Df, _rv, _tv = cv2.calibrateCamera(
            obj_pts, img_pts, (self.cam.frame_width, self.cam.frame_height), numpy.array(K, dtype=numpy.float64),
            None, flags=flags, criteria=crit)
        return {'rms': float(rms), 'K':Kf, 'D':Df}

    def _commit_matrices(self, gcmd):
        good = [g for g in self.groups if g.get('ok') and g.get('fit')]
        if not good:
            gcmd.respond_info("no good group for R / K (need affine rms<20, n>=8)")
            return
        cam = self.cam
        K, A, T = self._K_from_planes(good, cam.frame_width, cam.frame_height)
        gcmd.respond_info(f"_K_from_planes K={K} A={_fmt_A(A)} T={T}")
        cx, cy = float(K[0, 2]), float(K[1, 2])
        gcmd.respond_info(f"K from ΔZ/Δ(mm/px): fx=fy={float(K[0,0]):.4g} c=({cx:.2f},{cy:.2f})")
        calib = self._calibrate_camera(K)
        D = calib['D']
        gcmd.respond_info(f"Brown k1,k2 with K held fixed, reproj rms={calib['rms']:.3f}px")
        R = self._rectification_from_affine(A).astype(numpy.float32)
        w, h = cam.frame_width, cam.frame_height
        virt, roi = cv2.getOptimalNewCameraMatrix( K, D, (w, h), alpha=1, newImgSize=(w, h))
        # alpha=1 keeps the whole sensor (black borders OK). generate_maps
        # turns virt into a square canvas of side fx with PP at the center.
        virt[0, 2] = virt[0, 0] / 2.0  # force cy to center
        virt[1, 2] = virt[1, 1] / 2.0  # force cy to center
        cam.camera_matrix = numpy.asarray(K, dtype=numpy.float32)
        cam.virt_matrix = numpy.asarray(virt, dtype=numpy.float32)
        cam.rectif_matrix = R
        cam.dist_coeffs = numpy.asarray(D, dtype=numpy.float32)
        cam.set_overlay(None)
        gcmd.respond_info(cam.generate_maps())
        gcmd.respond_info(f"rectification_matrix: {numpy.asarray(cam.rectif_matrix).tolist()}")
        gcmd.respond_info(f"camera_matrix: {numpy.asarray(cam.camera_matrix).tolist()}")
        gcmd.respond_info(f"virtual_camera_matrix: {numpy.asarray(cam.virt_matrix).tolist()}")
        gcmd.respond_info(f"dist_coeffs: {numpy.asarray(cam.dist_coeffs).reshape(-1).tolist()}")

class TopCameraSession(CameraSession):
    def prompt(self, gcmd):
        n = len(self.groups)
        gcmd.respond_info(
            "CAMERA session [%s] top (looking down). Jog over a fiducial "
            "(toolhead Z = camera height).\n"
            "  SAMPLE Z=<plane>   or   SAMPLE FIDUCIAL=primary|secondary\n"
            "  Pixel grid (PSTEP/MARGIN, needs px_per_mm), then fiducial XY.\n"
            "  groups so far: %d"
            % (self.cam.name, n))

    def _apply_fiducial_xy(self, gcmd, fid_name, x, y, z):
        if not fid_name:
            gcmd.respond_info(
                f"unnamed mark: {{'x': {x:.4f}, 'y': {y:.4f}, 'z': {z:g}}}")
            return
        key = 'fiducial_primary' if fid_name == 'primary' else 'fiducial_secondary'
        f = getattr(self.pnp, key, {})
        old = (f.get('x',None), f.get('y',None))
        f['x'], f['y'] = float(x), float(y)
        f['z'] = float(z)
        gcmd.respond_info(f"{key}: was XY=({old[0]},{old[1]}) now ({x:.4f},{y:.4f}) z={z:g}")
        gcmd.respond_info(f"{key}: {f}")

    def sample(self, gcmd):
        z, fid_name, params = self.sample_params(gcmd)
        if 'x' in params and 'y' in params:
            gcmd.respond_info(f"Moving to {fid_name}")
            self.move_to(params['x'], params['y'], z, 0, self.speed, 0.)
        views = self._collect_views(gcmd, z, params)
        if len(views) < 3:
            raise gcmd.error(f"CAMERA: only {len(views)} good views (need ≥3). Stay over the mark / nozzle and SAMPLE again.")
        group = self._solve_group(gcmd, z, views)
        mx, my = group['est_x'], group['est_y']
        sx, sy = group['est_sx'], group['est_sy']
        tag = fid_name or f"z={z:g}"
        gcmd.respond_info(f"fiducial [{tag}] XY=({mx:.4f},{my:.4f}) σ=({sx:.4f},{sy:.4f}) z_plane={z:g}")
        self._apply_fiducial_xy(gcmd, fid_name, mx, my, z)
        self.groups = [g for g in self.groups if not (abs(g['z'] - z) < 1e-3 and g.get('fid') == fid_name)]
        self.groups.append(group)

    def finish(self, gcmd):
        super().finish(gcmd)
        gcmd.respond_info(f"Paste to [pnp]:")
        for key in ['fiducial_primary','fiducial_secondary']:
            f = getattr(self.pnp, key, {})
            gcmd.respond_info(f"{key}: {f}")
        gcmd.respond_info(f"CAMERA top done [{self.cam.name}] groups={len(self.groups)}")

class BottomCameraSession(CameraSession):
    def prompt(self, gcmd):
        gcmd.respond_info(
            "CAMERA session [%s] bottom (looking up). Lower the nozzle "
            "and SAMPLE Z=<nozzle height> — Z is moved to that plane, "
            "then a pixel grid (PSTEP/MARGIN, needs px_per_mm).\n"
            "  Second SAMPLE at another Z. DONE / ABORT.\n"
            "  heights so far: %d"
            % (self.cam.name, len(self.groups)))

    def sample(self, gcmd):
        z, fid_name, params = self.sample_params(gcmd)
        pos = self.pnp.get_position()
        gcmd.respond_info(f"{self.cam.name}: moving nozzle Z {pos[2]:.4f} → {z:g}")
        self.move_to(pos[0], pos[1], z, 0, self.speed, 0.)
        views = self._collect_views(gcmd, z, params, home=not self.groups)
        if len(views) < 3:
            raise gcmd.error(f"CAMERA: only {len(views)} good views (need ≥3). Stay over the mark / nozzle and SAMPLE again.")
        group = self._solve_group(gcmd, z, views)
        mx, my = group['est_x'], group['est_y']
        sx, sy = group['est_sx'], group['est_sy']
        gcmd.respond_info(f"camera XY=({mx:.4f},{my:.4f}) σ=({sx:.4f},{sy:.4f}) from {len(views)} views at nozzle Z={z:g}")
        self.groups = [g for g in self.groups if not (abs(g['z'] - z) < 1e-3 and g.get('fid') == fid_name)]
        self.groups.append(group)

    def finish(self, gcmd):
        super().finish(gcmd)
        for group in self.groups:
            mx, my = group['est_x'], group['est_y']
            sx, sy = group['est_sx'], group['est_sy']
            l,z = len(group['views']), group['z']
            gcmd.respond_info(f"camera_xy=({mx:.4f},{my:.4f}) σ=({sx:.4f},{sy:.4f}) from {l} views at nozzle Z={z:g}")
        self.cam.camera_xy = (mx,my)
        gcmd.respond_info(f"CAMERA bottom done [{self.cam.name}]")

class AlignSession(CalibSession):
    """Looking-up camera already calibrated. Pinpoint each nozzle at several
    Z. First SAMPLE's tool is the TOOLOFFSET reference: its offset stays,
    camera_xy and the other tools are relative to it."""
    kind = 'align'

    def __init__(self, host, cam, pipe_name, gcmd):
        # skip = ['CAM', 'Z', 'PIPELINE', 'FIDUCIAL',
        #         'DIA', 'TOOL', 'MAX_ITER']
        super().__init__(host, cam, pipe_name, gcmd, ['CAM', 'PIPELINE'])
        if cam.px_to_mm_delta(1., 0., cam.z_plane) is None:
            raise gcmd.error("ALIGN needs px_per_mm — run PNP_CALIB_PXMM first")
        self.hits = {}
        self.master = None
        self.saved_vectors = {}
        # So PNP_TOOL / TIP / RUNOUT inside the wizard see û=0 on the
        # nozzle that's already selected.
        tool = self.pnp.active_tool
        if tool is not None and hasattr(tool, 'tip'):
            self.saved_vectors[tool.short_name] = tuple(tool.axis_vector)
            tool.axis_vector = [0., 0., 1.]

    def status(self):
        d = super().status()
        d['tools'] = list(self.hits.keys())
        d['reference'] = self.master
        d['samples'] = sum(len(v) for v in self.hits.values())
        d['heights'] = sorted({h['z'] for v in self.hits.values() for h in v})
        return d

    def prompt(self, gcmd):
        bits = []
        for k, hs in self.hits.items():
            bits.append("%s:%s" % (k, [h['z'] for h in hs]))
        gcmd.respond_info(
            "ALIGN session [%s]. Camera already calibrated.\n"
            "  First SAMPLE TOOL= is the TOOLOFFSET reference (do it with "
            "the nozzle you TOOLOFFSET on the fiducials). That offset stays; "
            "camera_xy and the other tools follow it. Iterate: fiducials ↔ "
            "ALIGN until both look right.\n"
            "  Per tool: PNP_TOOL SET=  PNP_TIP LOAD=  PNP_VISION_RUNOUT, "
            "then SAMPLE TOOL=… Z=<h> twice (SAMPLE force R=0).\n"
            "  so far: %s  reference: %s"
            % (self.cam.name, bits or 'nothing', self.master or '—'))

    def detach(self):
        for key, vec in self.saved_vectors.items():
            try:
                self.pnp.lookup_tool(key).axis_vector = vec
            except Exception:
                logging.exception("ALIGN restore axis_vector [%s]", key)
        CalibWizard.detach(self)

    def _ensure_tool(self, gcmd, name):
        tool = self.pnp.lookup_tool(name)
        if not hasattr(tool, 'tip'):
            raise gcmd.error(
                "ALIGN TOOL= needs a nozzle, not %s" % (tool.short_name,))
        if self.pnp.active_tool is not tool:
            gcmd.respond_info("ALIGN: selecting %s" % (tool.short_name,))
            self.pnp.select_tool(tool, True)
        key = tool.short_name
        if key not in self.saved_vectors:
            self.saved_vectors[key] = tuple(tool.axis_vector)
            tool.axis_vector = [0., 0., 1.]
        return tool

    def sample(self, gcmd):
        name = gcmd.get('TOOL', None)
        if name is None:
            raise gcmd.error("TOOL= is required")
        tool = self._ensure_tool(gcmd, name)
        z, _fid, params = self.sample_params(gcmd)
        params['cx'] = self.cam.cx
        params['cy'] = self.cam.cy
        key = tool.short_name
        if self.master is None:
            self.master = key
            gcmd.respond_info(
                "ALIGN: [%s] is the reference (TOOLOFFSET tool). "
                "Its offset stays; camera_xy and other tools follow it."
                % (key,))
        prev = self.hits.get(key, [])
        if prev:
            x, y = prev[-1]['x'], prev[-1]['y']
        elif getattr(self.cam, 'camera_xy', None) is not None:
            x,y = self.cam.camera_xy
        else:
            pos = self.pnp.get_position()
            x, y = pos[0], pos[1]
        gcmd.respond_info(
            "ALIGN [%s]: Z %g at XY=(%.4f,%.4f)" % (key, z, x, y))
        self.move_to(x, y, z, 0, self.speed, 0.)
        px_fixed = py_fixed = None
        if prev:
            try:
                r = self.vision.run_detect(self.cam, self.pipe_name, gcmd, params)
                px_fixed, py_fixed = self.vision.ensure_singular_result(r, gcmd, "align-fixed-xy")
                gcmd.respond_info("ALIGN [%s] Z=%g same-XY px=(%.2f,%.2f)"% (key, z, px_fixed, py_fixed))
            except Exception as e:
                gcmd.respond_info("ALIGN: fixed-XY detect Exception (%s)" % e)
        startpos = self.pnp.get_position()
        ret = self.vision.pinpoint_target(
            gcmd, startpos, self.cam, self.pipe_name,
            self.speed, self.settle, self.max_tries, self.accuracy, params,
            tag="align[%s]" % key)
        if not ret.get('success'):
            raise gcmd.error("ALIGN [%s] Z=%g: pinpoint did not converge (err=%s)"% (key, z, ret.get('err_mm')))
        gx, gy = float(ret['pos'][0]), float(ret['pos'][1])
        rec = {
            'z': float(z), 'x': gx, 'y': gy,
            'px': float(ret['px']), 'py': float(ret['py']),
            'px_fixed': px_fixed, 'py_fixed': py_fixed,
            'err_mm': float(ret['err_mm']),
        }
        gcmd.respond_info(f"{rec}")
        hits = [h for h in prev if abs(h['z'] - z) >= 1e-3]
        hits.append(rec)
        self.hits[key] = hits
        gcmd.respond_info(
            "ALIGN [%s] Z=%g gcode=(%.4f,%.4f) px=(%.2f,%.2f) err=%.4fmm"
            % (key, z, gx, gy, rec['px'], rec['py'], rec['err_mm']))

    def finish(self, gcmd):
        if not self.hits:
            raise gcmd.error("ALIGN: no SAMPLE. SAMPLE TOOL=left Z= ...")
        cam = self.cam
        for key, hits in self.hits.items():
            hs = sorted(hits, key=lambda h: h['z'])
            for a, b in zip(hs, hs[1:]):
                if b.get('px_fixed') is None:
                    continue
                dpx = math.hypot(float(b['px_fixed']) - a['px'], float(b['py_fixed']) - a['py'])
                gcmd.respond_info(
                    "ALIGN [%s]: hole walked %.1f px over ΔZ=%.1f mm at fixed XY "
                    "(tool tilt; camera maps left alone)"
                    % (key, dpx, b['z'] - a['z']))
        intercepts = {}
        for key, hits in self.hits.items():
            tool = self.pnp.lookup_tool(key)
            hs = sorted(hits, key=lambda h: h['z'])
            zs = numpy.array([h['z'] for h in hs], dtype=numpy.float64)
            xs = numpy.array([h['x'] for h in hs], dtype=numpy.float64)
            ys = numpy.array([h['y'] for h in hs], dtype=numpy.float64)
            # Pinpoint with û=0: g(Z) = a + Z·u_g. gcode_to_machine subtracts
            # Z·û, so û = −u_g. camera_xy is the Z=0 intercept of the
            # reference tool; its TOOLOFFSET stays. Other tools absorb
            # (a − camera_xy) into offset.
            ax, ay = float(xs.mean()), float(ys.mean())
            if len(hs) >= 2 and float(zs.max() - zs.min()) > 0.5:
                G = numpy.column_stack([numpy.ones_like(zs), zs])
                bx, *_ = numpy.linalg.lstsq(G, xs, rcond=None)
                by, *_ = numpy.linalg.lstsq(G, ys, rcond=None)
                ax, ux_g = float(bx[0]), float(bx[1])
                ay, uy_g = float(by[0]), float(by[1])
                vec = (-ux_g, -uy_g, 1.0)
                tool.axis_vector = vec
                self.saved_vectors[key] = vec
                gcmd.respond_info(
                    "ALIGN [%s]: axis_vector %s  "
                    "Z=%g→(%.4f,%.4f) Z=%g→(%.4f,%.4f)  g(Z=0)=(%.4f,%.4f)"
                    % (key, vec, float(zs[0]), float(xs[0]), float(ys[0]),
                       float(zs[-1]), float(xs[-1]), float(ys[-1]), ax, ay))
            intercepts[key] = (ax, ay)
        master = self.master or next(iter(self.hits))
        mx, my = intercepts[master]
        cam.camera_xy = (mx, my)
        gcmd.respond_info(
            "ALIGN: camera_xy from reference [%s] g(Z=0)=(%.4f,%.4f) "
            "— [%s] offset unchanged (TOOLOFFSET)"
            % (master, mx, my, master))
        for key, (ax, ay) in intercepts.items():
            tool = self.pnp.lookup_tool(key)
            if key != master:
                dx, dy = ax - mx, ay - my
                if math.hypot(dx, dy) > 1e-4 and hasattr(tool, 'offset_x'):
                    tool.offset_x = float(tool.offset_x) - dx
                    tool.offset_y = float(tool.offset_y) - dy
                    gcmd.respond_info("ALIGN [%s]: offset residue (%.4f,%.4f) vs [%s]"%(key, dx, dy, master))
            gcmd.respond_info(
                "paste into [%s]:\naxis_vector: %s\noffset_x: %.4f\noffset_y: %.4f"
                % (tool.name, tool.axis_vector, tool.offset_x, tool.offset_y))
        gcmd.respond_info(
            "paste into [%s]:\ncamera_xy: (%.4f, %.4f)"
            % (cam.name, cam.camera_xy[0], cam.camera_xy[1]))
        gcmd.respond_info("ALIGN done [%s] tools=%s"% (cam.name, list(self.hits.keys())))

class PnPCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.pnp = self.printer.lookup_object('pnp')
        self.vision = self.printer.lookup_object('pnp_vision')
        self.calib_speed = config.getfloat('calib_speed', 50.)
        self.session = None
        self.gcode.register_command('PNP_CALIB_PXMM', self.cmd_PNP_CALIB_PXMM, desc=self.cmd_PNP_CALIB_PXMM_help)
        self.gcode.register_command('PNP_CALIB_CAMERA', self.cmd_PNP_CALIB_CAMERA,desc=self.cmd_PNP_CALIB_CAMERA_help)
        self.gcode.register_command('PNP_CALIB_TOOLOFFSET', self.cmd_PNP_CALIB_TOOLOFFSET,desc=self.cmd_PNP_CALIB_TOOLOFFSET_help)
        self.gcode.register_command('PNP_CALIB_ALIGN', self.cmd_PNP_CALIB_ALIGN,desc=self.cmd_PNP_CALIB_ALIGN_help)

    def get_status(self, eventtime):
        if self.session is None:
            return {'is_active': False, 'kind': None, 'cam': None,
                    'samples': 0, 'heights': []}
        return self.session.status()

    def start_session(self, session, gcmd):
        if self.session is not None:
            raise self.printer.command_error("Already in PNP_CALIB_%s; DONE or ABORT first"% (self.session.kind.upper(),))
        self.session = session
        try:
            session.attach()
        except Exception:
            self.end_session()
            raise
        session.prompt(gcmd)

    def end_session(self):
        if self.session is not None:
            self.session.detach()
        self.session = None

    cmd_PNP_CALIB_PXMM_help = (
        "Interactive mm/pixel calibration. SAMPLE a rosette at Z=; DONE "
        "writes px_per_mm (and camera_z from two heights). Tool tilt/offset: "
        "PNP_CALIB_ALIGN.")
    def cmd_PNP_CALIB_PXMM(self, gcmd):
        cam = self.vision.lookup_cam(gcmd.get('CAM', 'topcam'))
        pipe_name = gcmd.get('PIPELINE', None)
        if not pipe_name:
            raise gcmd.error("PIPELINE is required.")
        if pipe_name not in self.vision.pipelines:
            raise gcmd.error(f"Pipeline '{pipe_name}' does not exist.")
        self.start_session(PxMmSession(self, cam, pipe_name, gcmd), gcmd)

    cmd_PNP_CALIB_CAMERA_help = (
        "Interactive camera calib. SAMPLE runs a dense rosette: topcam "
        "solves fiducial XY, bottomcam moves nozzle to Z= and solves camera pose.")
    def cmd_PNP_CALIB_CAMERA(self, gcmd):
        cam = self.vision.lookup_cam(gcmd.get('CAM', 'topcam'))
        pipe_name = gcmd.get('PIPELINE', None)
        if not pipe_name:
            raise gcmd.error("PIPELINE is required.")
        if pipe_name not in self.vision.pipelines:
            raise gcmd.error(f"Pipeline '{pipe_name}' does not exist.")
        if cam.looking_up:
            self.start_session(BottomCameraSession(self, cam, pipe_name, gcmd), gcmd)
        else:
            self.start_session(TopCameraSession(self, cam, pipe_name, gcmd), gcmd)

    cmd_PNP_CALIB_TOOLOFFSET_help = (
        "Tool offset for the active nozzle. PNP_TOOL SET=<nozzle> first, "
        "have tip loaded (PNP_TIP LOAD=<tip>) and runout (PNP_VISIOON_RUNOUT) done "
        "jog onto fiducial and pass which one is it by passing FIDUCIAL=primary|secondary")
    def cmd_PNP_CALIB_TOOLOFFSET(self, gcmd):
        if self.pnp.active_tool is None:
            raise gcmd.error("PNP_CALIB_TOOLOFFSET: no active tool. PNP_TOOL SET=<tool> first")
        if not hasattr(self.pnp.active_tool, 'tip'):
            raise gcmd.error("PNP_CALIB_TOOLOFFSET needs a nozzle (active tool is %s)"% (self.pnp.active_tool.short_name,))
        name, f = self.pnp.lookup_fiducial(gcmd.get('FIDUCIAL', None))
        if name is None:
            gcmd.respond_info(self.cmd_PNP_CALIB_TOOLOFFSET_help)
            raise gcmd.error("PNP_CALIB_TOOLOFFSET FIDUCIAL=primary|secondary")
        pos = self.pnp.get_position()
        dx = f['x'] - pos[0]
        dy = f['y'] - pos[1]
        gcmd.respond_info("TOOLOFFSET [%s] Δ=(%.4f,%.4f)"%(self.pnp.active_tool.short_name, dx, dy))
        self.pnp.active_tool.offset_x += dx
        self.pnp.active_tool.offset_y += dy
        gcmd.respond_info(
            "paste into [pnp_tool %s]:\n"
            "offset_x: %.4f\noffset_y: %.4f\naxis_vector: %s"
            % (self.pnp.active_tool.short_name,
                self.pnp.active_tool.offset_x,
                self.pnp.active_tool.offset_y,
                self.pnp.active_tool.axis_vector))

    cmd_PNP_CALIB_ALIGN_help = (
        "Align nozzles to a calibrated looking-up camera. "
        "First SAMPLE TOOL= is the TOOLOFFSET reference: that offset stays, "
        "camera_xy and the other tools follow it. SAMPLE TOOL= Z=<h> twice "
        "per tool. Iterate with PNP_CALIB_TOOLOFFSET on the fiducials.")
    def cmd_PNP_CALIB_ALIGN(self, gcmd):
        cam = self.vision.lookup_cam(gcmd.get('CAM', 'bottomcam'),True)
        pipe_name = gcmd.get('PIPELINE', None)
        if not pipe_name:
            raise gcmd.error("PIPELINE is required.")
        if pipe_name not in self.vision.pipelines:
            raise gcmd.error(f"Pipeline '{pipe_name}' does not exist.")
        self.start_session(AlignSession(self, cam, pipe_name, gcmd), gcmd)

def load_config(config):
    return PnPCalibration(config)
