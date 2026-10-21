# klippy/extras/pnp_vision.py
# Dynamic vision pipelines processor
#
# Copyright (C) 2026 Maja Stanislawska <maja@makershop.ie>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import re,datetime
import cv2,numpy,math
import logging
from .camera_v4l import parse_light, parse_hex_color

class detection_item:
    def __init__(self, c):
        self.type = "contour"
        M = cv2.moments(c)
        self.cx = None if M["m00"] == 0 else float(M["m10"] / M["m00"])
        self.cy = None if M["m00"] == 0 else float(M["m01"] / M["m00"])
        self.w = None
        self.h = None
        self.angle = None
        self.area = float(cv2.contourArea(c))
        # equivalent radius from area (circularity path)
        self.r = float(numpy.sqrt(self.area / numpy.pi))
        self.symmetry = None       # radial optical symmetry [0..1]
        self.fill_ratio = None     # area / (pi * r^2)
        self.ratio = None
        self.c = c  # raw contour; omitted from todict()/__repr__
        self.perimeter = float(cv2.arcLength(c, True))
        # circularity 4πA/P² — NOT optical symmetry (see radial_symmetry)
        self.c_score = 0. if self.perimeter == 0 else float(
            4 * numpy.pi * self.area / (self.perimeter * self.perimeter))
    @classmethod
    def from_circle(cls, cx, cy, r, score=None, symmetry=None, fill_ratio=None, area=None):
        """Circle without a contour (blob detector / distance-transform peaks)."""
        item = object.__new__(cls)
        item.type = 'circle'
        item.cx = float(cx)
        item.cy = float(cy)
        item.r = float(r)
        item.area = float(area if area is not None else math.pi * r * r)
        item.w = item.h = item.angle = item.ratio = None
        item.symmetry = None if symmetry is None else float(symmetry)
        item.fill_ratio = None if fill_ratio is None else float(fill_ratio)
        item.c_score = 1.0 if score is None else float(score)
        item.c = None
        item.perimeter = 0.
        item.box = None
        item.rect = None
        return item
    @classmethod
    def from_ellipse(cls, cx, cy, axes_w, axes_h, angle, symmetry=None):
        """Build a high-precision subpixel detection item from cv2.fitEllipse."""
        item = object.__new__(cls)
        item.type = 'circle'
        item.cx = float(cx)
        item.cy = float(cy)
        item.r = float((axes_w + axes_h) / 4.0) # average radius
        item.area = float(numpy.pi * item.r * item.r)
        item.w = float(axes_w)
        item.h = float(axes_h)
        item.angle = float(angle)
        item.symmetry = float(symmetry if symmetry is not None
             else min(axes_w, axes_h) / max(axes_w, axes_h) if max(axes_w, axes_h) > 0 else 0)
        item.fill_ratio = 1.0
        item.c_score = item.symmetry # reuse as score
        item.c = None
        item.perimeter = float(numpy.pi * 2 * item.r)
        item.box = None
        item.rect = None
        return item
    @classmethod
    def from_rect(cls, cx, cy, w, h, angle, c=None):
        """Axis-normalized rect (long side = w), with box for drawing."""
        item = object.__new__(cls)
        item.type = 'rect'
        item.cx = float(cx)
        item.cy = float(cy)
        item.w = float(w)
        item.h = float(h)
        item.angle = float(angle)
        if item.w < item.h:
            item.angle -= 90.
            item.w, item.h = item.h, item.w
        if item.h == 0:
            item.h = 1e-6
        item.ratio = item.w / item.h
        item.area = item.w * item.h
        item.r = float(numpy.sqrt(item.area / numpy.pi))
        item.rect = ((item.cx, item.cy), (item.w, item.h), item.angle)
        item.box = numpy.int64(cv2.boxPoints(item.rect))
        item.c = c
        item.symmetry = None
        item.fill_ratio = None
        item.c_score = None
        item.perimeter = 0.
        return item
    def __repr__(self):
        s = f"{self.type} xy={self.cx,self.cy}"
        if self.type == 'circle':
            if self.r:
                s += f" r={self.r:.2f}"
            if self.c_score is not None:
                s += f" c_score={self.c_score:.3f}"
            if self.symmetry is not None:
                s += f" sym={self.symmetry:.3f}"
            if self.fill_ratio is not None:
                s += f" fill={self.fill_ratio:.3f}"
        elif self.type == 'rect':
            s += f" wh={self.w,self.h} angle={self.angle} ratio={self.ratio}"
        else:
            n = 0 if self.c is None else len(self.c)
            s += f" contour={n} points"
        s += f" a={self.area:.1f}"
        return s
    def todict(self):
        """Plain dict for webhooks/status — no contour ndarray, plain floats."""
        d = {'type': self.type, 'cx': self.cx, 'cy': self.cy, 'area': self.area}
        if self.type == 'circle':
            d['r'] = self.r
            d['c_score'] = self.c_score
            if self.symmetry is not None:
                d['symmetry'] = self.symmetry
            if self.fill_ratio is not None:
                d['fill_ratio'] = self.fill_ratio
        elif self.type == 'rect':
            d['w'] = self.w
            d['h'] = self.h
            d['angle'] = self.angle
            d['ratio'] = self.ratio
        else:
            d['r'] = self.r
            d['c_score'] = self.c_score
            d['n_pts'] = 0 if self.c is None else len(self.c)
        return d
    def test_radial_symmetry(self, img, min_sym, n_rays=32, edge_threshold=64, max_r=None):
        """
        Optical radial symmetry: raycast from (cx,cy) on a gray/binary image,
        measure edge distances, score = 1 - 2·CV (clamped to [0,1]).
        Fills self.symmetry; does not change type.
        """
        if self.cx is None or self.cy is None: return False
        h, w = img.shape[:2]
        icx = int(round(self.cx))
        icy = int(round(self.cy))
        if not (0 <= icx < w and 0 <= icy < h): return False
        center_val = int(img[icy, icx])
        if max_r is None:
            max_search = max(10, int(2.5 * (self.r or 20)))
        else:
            max_search = int(max_r)
        radii = []
        n_rays = max(4, int(n_rays))
        for i in range(n_rays):
            ang = 2. * math.pi * i / n_rays
            dx, dy = math.cos(ang), math.sin(ang)
            found = None
            for step in range(1, max_search + 1):
                x = int(round(self.cx + dx * step))
                y = int(round(self.cy + dy * step))
                if x < 0 or y < 0 or x >= w or y >= h:
                    found = step - 1
                    break
                if abs(int(img[y, x]) - center_val) > edge_threshold:
                    found = step
                    break
            if found is not None and found > 0:
                radii.append(float(found))
        if len(radii) < max(4, n_rays // 2): return False
        mean_r = sum(radii) / len(radii)
        if mean_r < 1e-6: return False
        var = sum((rr - mean_r) ** 2 for rr in radii) / len(radii)
        cv_coeff = math.sqrt(var) / mean_r
        # cv=0 → 1.0; cv=0.5 → 0.0
        sym = float(max(0., min(1., 1. - 2. * cv_coeff)))
        if sym <= min_sym: return False
        self.type = 'circle'
        self.symmetry=sym
        return True
    def test_circularity(self, min_circularity):
        """Circularity score (4πA/P²) gate — not optical symmetry."""
        if self.c_score < float(min_circularity):
            return False
        self.type = 'circle'
        return True
    def test_enclosing_circle(self,min_fill):
        (x, y), er = cv2.minEnclosingCircle(self.c)
        er = float(er)
        fill = float(self.area / (math.pi * er * er)) if er > 1e-6 else 0.
        if fill < min_fill: return False
        self.type = 'circle'
        # overwrite cv2.moments values with new detection.
        self.cx, self.cy = float(x), float(y)
        self.r = er
        self.fill_ratio = fill
        return True
    def test_rect(self):
        if self.type == 'circle' or self.c is None:
            return False
        self.rect = cv2.minAreaRect(self.c)
        self.box = numpy.int64(cv2.boxPoints(self.rect))
        ((self.cx, self.cy), (self.w, self.h), self.angle) = self.rect
        self.cx = float(self.cx)
        self.cy = float(self.cy)
        self.w = float(self.w)
        self.h = float(self.h)
        self.angle = float(self.angle)
        if self.w < self.h:
            self.angle -= 90.
            self.w, self.h = self.h, self.w
        if self.h == 0:
            return False
        self.type = 'rect'
        self.ratio = self.w / self.h
        return True

class PnpVisionPipeline:
    def __init__(self, name, steps_str, defaults, light_spec):
        self.name = name
        self.steps_str = steps_str
        self.defaults = defaults
        self.light_spec = light_spec
        self.compiled_steps = self._compile(steps_str)
        # context
        self.cam=None
        self.img = None
        # self.orig_img=None
        self.image_stash = {}
        self.detected_contours=[]
        self.detected_circles = []
        self.detected_rects = []

    def resolve_light(self, runtime, cmd):
        """Fold LIGHT= for acquire_light: (s, color), either may be None."""
        s = color = None
        for spec in (self.light_spec, runtime, cmd):
            if spec is None or spec == '':
                continue
            ps, pc = parse_light(spec)
            if ps is not None:
                s = ps
            if pc is not None:
                color = pc
        return s, color
    def _compile(self, steps_str):
        """unwinds pipeline definition string to list of tuples (name, [parameters..])"""
        compiled = []
        for step in steps_str.split('|'):
            step = step.strip()
            if not step: continue
            match = re.match(r'([a-zA-Z0-9_]+)\((.*)\)', step)
            if match:
                func_name = match.group(1)
                args_str = match.group(2)
                # save as raw string (will be casted or substituted by values in run())
                args_list = [x.strip().strip("'\"") for x in args_str.split(',') if x.strip()]
                compiled.append((func_name, args_list))
        return compiled
    def run(self, eventtime, img, gcmd, cam, runtime_params):
        self.cleanup()
        self.img = img
        self.cam=cam
        if self.img is None:
            raise gcmd.error("Failed to get frame")
        # self.orig_img=img.copy()

        active_params = self.defaults.copy()
        params={k:v for k,v in runtime_params.items() if v is not None}
        active_params.update(params)
        self.verbose=active_params.get('verbose',False)
        if self.verbose:
            gcmd.respond_info(f"active_params={active_params}")
        for func_name, raw_args in self.compiled_steps:
            method = getattr(self, f"_filter_{func_name}", None)
            if not method:
                raise gcmd.error(f"Unknown filter '{func_name}'")
            # replace params (ex. 'max_r') with values from active_params
            evaluated_args = [gcmd] #first arg is always gcmd for logging
            for arg in raw_args:
                if arg.lower() in active_params:
                    evaluated_args.append(active_params[arg.lower()])
                else: # convert to int/float or keep as text
                    try: evaluated_args.append(float(arg) if '.' in arg else int(arg))
                    except ValueError: evaluated_args.append(arg)
            try: method(*evaluated_args)
            except Exception as e:
                raise gcmd.error(f"step:{func_name}({raw_args}) Error:{e}")
        return bool(self.detected_circles or self.detected_rects)

    def cleanup(self):
        """discard stale detection data and cached frames"""
        self.cam=None
        self.img = None
        # self.orig_img = None
        self.image_stash = {}
        self.detected_contours = []
        self.detected_circles = []
        self.detected_rects = []

    def _filter_adjust(self, gcmd, brightness, contrast=1.):
        "brightnes:[-255..255], contrast [0..1..3]"
        mid = numpy.full_like(self.img, 128)
        self.img = cv2.addWeighted(self.img, contrast, mid, 1.0 - contrast, brightness)
        if self.verbose: gcmd.respond_info(f"adjust({brightness, contrast})")
    def _filter_blurmedian(self, gcmd, size):
        self.img = cv2.medianBlur(self.img, int(size))
        if self.verbose: gcmd.respond_info(f"blurmedian({size})")
    def _filter_blurgausian(self, gcmd, size,sigmaX=0,sigmaY=None):
        if sigmaY is None: sigmaY=sigmaX
        self.img = cv2.GaussianBlur(self.img, (int(size), int(size)), sigmaX, sigmaY)
        if self.verbose: gcmd.respond_info(f"blurgausian({size})")
    def _is_gray(self):
        return self.img.ndim == 2 or (self.img.ndim == 3 and self.img.shape[2] == 1)
    def _require_gray(self, gcmd):
        if not self._is_gray():
            raise gcmd.error("need a gray frame")
    def _require_bgr(self, gcmd):
        if self._is_gray():
            raise gcmd.error("need a BGR frame")
    def _filter_color(self, gcmd):
        self._require_gray(gcmd)
        self.img = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        if self.verbose: gcmd.respond_info(f"color()")
    def _filter_bgr(self, gcmd):
        self._filter_color(gcmd)
    def _filter_gray(self, gcmd):
        """gray() — BGR → luma. Errors if the frame is already gray."""
        self._require_bgr(gcmd)
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        if self.verbose: gcmd.respond_info(f"gray()")
    def _filter_threshold(self, gcmd,thresh_val=128, max_val=255, method=cv2.THRESH_BINARY):
        self._require_gray(gcmd)
        _, self.img = cv2.threshold(self.img, thresh_val, max_val, method)
        if self.verbose: gcmd.respond_info(f"threshold({thresh_val, max_val})")
    def _filter_otsu(self, gcmd,thresh_val=0, max_val=255, method=cv2.THRESH_BINARY):
        self._require_gray(gcmd)
        _, self.img = cv2.threshold(self.img, thresh_val, max_val, method+cv2.THRESH_OTSU)
        if self.verbose: gcmd.respond_info(f"otsu({thresh_val, max_val})")
    def _filter_threshmean(self, gcmd, block_size, c, max_value=225, method=cv2.THRESH_BINARY):
        self._require_gray(gcmd)
        self.img = cv2.adaptiveThreshold(self.img, max_value, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, block_size, c)
        if self.verbose: gcmd.respond_info(f"threshmean({block_size, c, max_value})")
    def _filter_threshgaus(self, gcmd, block_size, c, max_value=225, method=cv2.THRESH_BINARY):
        self._require_gray(gcmd)
        self.img = cv2.adaptiveThreshold(self.img, max_value, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, block_size, c)
        if self.verbose: gcmd.respond_info(f"threshgaus({block_size, c, max_value})")
    def _filter_bilateral(self, gcmd, block_size, sigCol, sigSiz, border=cv2.BORDER_DEFAULT):
        self.img = cv2.bilateralFilter(self.img,block_size,sigCol, sigSiz, border)
        if self.verbose: gcmd.respond_info(f"bilateral({block_size, sigCol, sigSiz, border})")
    def _filter_laplacian(self,gcmd):
        self.img = cv2.Laplacian(self.img, cv2.CV_64F)
        if self.verbose: gcmd.respond_info(f"laplacian()")
    def _filter_normalize(self, gcmd,alpha=0,beta=255):
        self.img = cv2.normalize(self.img, None, alpha, beta, norm_type=cv2.NORM_MINMAX)
        if self.verbose: gcmd.respond_info(f"normalize({alpha},{beta})")
    def _filter_equalize_hist(self, gcmd):
        if self._is_gray():
            self.img = cv2.equalizeHist(self.img)
        else:
            chans = list(cv2.split(self.img))
            for i, ch in enumerate(chans):
                chans[i] = cv2.equalizeHist(ch)
            self.img = cv2.merge(chans)
        if self.verbose: gcmd.respond_info(f"equalize_hist()")
    def _filter_clahe(self, gcmd, size=8,clip=5):
        clahe = cv2.createCLAHE(tileGridSize=(size, size), clipLimit=clip)
        if self._is_gray():
            self.img = clahe.apply(self.img)
        else:
            l, a, b = cv2.split(cv2.cvtColor(self.img, cv2.COLOR_BGR2LAB))
            self.img = cv2.cvtColor(cv2.merge((clahe.apply(l), a, b)), cv2.COLOR_LAB2BGR)
        if self.verbose: gcmd.respond_info(f"clahe({size},{clip})")
    # mask_* stash a 0/255 mask (name=mask). bitwise_* combine stashes.
    # apply_mask() ANDs the result onto the frame; fill()/use_mask() also consume it.
    def _stash_mask(self, mask, invert=0, name='mask'):
        if isinstance(invert, str):
            name, invert = invert, 0
        if int(invert) == 1:
            mask = cv2.bitwise_not(mask)
        self.image_stash[name] = mask
    def _mask_from_stash(self, name):
        entry = self.image_stash.get(name)
        if entry is None:
            raise ValueError("stash '%s' not found — run mask_* first" % (name,))
        return entry
    _FILL = {'black': 0, 'white': 255, 'gray': 128}
    _INV = {'inv', 'not', 'invert'}
    def _swatch(self, tok):
        """Named fill, scalar, or RGB hex (same parser as LIGHT=). BGR on 3ch."""
        n = 1 if self._is_gray() else self.img.shape[2]
        if isinstance(tok, (int, float)):
            v = int(tok)
            return v if n == 1 else (v, v, v)
        key = str(tok)
        if key in self._FILL:
            v = self._FILL[key]
            return v if n == 1 else (v, v, v)
        rgb = parse_hex_color(key)
        if rgb is not None:
            r, g, b = [int(round(c * 255)) for c in rgb[:3]]
            if n == 1:
                return int(0.299 * r + 0.587 * g + 0.114 * b)
            return (b, g, r)
        raise ValueError(
            "unknown swatch '%s' (black/white/gray, hex F0F/FFF, or 0..255)"
            % (key,))
    def _swatch_img(self, color):
        out = numpy.empty_like(self.img)
        out[:] = color
        return out
    def _match_channels(self, src):
        if src.shape[:2] != self.img.shape[:2]:
            raise ValueError("stash size %s != img %s"
                             % (src.shape[:2], self.img.shape[:2]))
        img_ch = 1 if self.img.ndim == 2 else self.img.shape[2]
        src_ch = 1 if src.ndim == 2 else src.shape[2]
        if img_ch == src_ch:
            return src
        if src_ch == 1 and img_ch == 3:
            return cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
        if src_ch == 3 and img_ch == 1:
            return cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        return src
    def _other_src(self, other):
        if other is None:
            return self.img
        if isinstance(other, (int, float)):
            return numpy.full_like(self.img, int(other))
        key = str(other)
        if key in self._FILL or parse_hex_color(key) is not None:
            return self._swatch_img(self._swatch(key))
        if key in self._INV:
            return cv2.bitwise_not(self.img)
        if key in self.image_stash:
            return self._match_channels(self.image_stash[key])
        raise ValueError(f"unknown other {key} (black/white/gray/inv/hex/0..255 or stash name)")
    def _arith(self, gcmd, op, name, other, label):
        mask = self._mask_from_stash(name)
        src2 = self._other_src(other)
        self.img = op(self.img, src2, mask=mask)
        if self.verbose:
            gcmd.respond_info("%s(%s,%s)" % (label, name, other))
    def _filter_mask_hsv(self, gcmd, h_min, h_max, s_min, s_max,  v_min, v_max, invert=0,name='mask'):
        """
        mask_hsv(h_min, h_max, s_min, s_max,  v_min, v_max [, invert])
        h=[0..180], s=[0..255],v=[0..255]
        Stashes binary mask as 'mask'; does not change self.img.
        Follow with bitwise_*(a,b), apply_mask(), fill(), or use_mask().
        """
        self._require_bgr(gcmd)
        hsv_frame = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        lower_bound = numpy.array([int(h_min), int(s_min), int(v_min)], dtype=numpy.uint8)
        upper_bound = numpy.array([int(h_max), int(s_max), int(v_max)], dtype=numpy.uint8)
        self._stash_mask(cv2.inRange(hsv_frame, lower_bound, upper_bound), invert, name)
        if self.verbose: gcmd.respond_info(f"hsv({h_min, h_max, s_min, s_max,  v_min, v_max, invert, name})")
    def _filter_mask_lab(self, gcmd, l_min, l_max, a_min=0, a_max=255, b_min=0, b_max=255, invert=0,name='mask'):
        """
        mask_lab(l_min, l_max [, a_min, a_max, b_min, b_max, invert])
        OpenCV Lab uint8: L=[0..255], a/b=[0..255] (128 = neutral).
        a/b default to full range, so mask_lab(0,40) is black, mask_lab(50,140) is gray.
        Stashes binary mask as 'mask'; does not change self.img.
        invert=1 needs a/b placeholders; or use_mask()|invert().
        """
        self._require_bgr(gcmd)
        lab = cv2.cvtColor(self.img, cv2.COLOR_BGR2LAB)
        lower_bound = numpy.array([int(l_min), int(a_min), int(b_min)], dtype=numpy.uint8)
        upper_bound = numpy.array([int(l_max), int(a_max), int(b_max)], dtype=numpy.uint8)
        self._stash_mask(cv2.inRange(lab, lower_bound, upper_bound), invert, name)
        if self.verbose: gcmd.respond_info(
            f"lab({l_min, l_max, a_min, a_max, b_min, b_max, invert, name})")

    def _filter_sharpen_2D9(self,gcmd):
        kernel = numpy.array([[-1.,-1.,-1.], [-1.,9.,-1.], [-1.,-1.,-1.]])
        self.img = cv2.filter2D(self.img, -1, kernel)
        if self.verbose: gcmd.respond_info(f"sharpen2D9() ({kernel.tolist()})")
    def _filter_sharpen_2D5(self,gcmd):
        kernel = numpy.array([[0.,-1.,0.], [-1.,5.,-1.], [0.,-1.,0.]])
        self.img = cv2.filter2D(self.img, -1, kernel)
        if self.verbose: gcmd.respond_info(f"sharpen2D5() ({kernel.tolist()})")
    def _filter_sharpen_mask(self,gcmd, size=0,sigma=1.0, strength=1.5):
        blur = cv2.GaussianBlur(self.img, (int(size), int(size)), sigma)
        self.img = cv2.addWeighted(self.img, 1.0 + strength, blur, -strength, 0)
        if self.verbose: gcmd.respond_info(f"unsharp_mask({size}, {sigma}, {strength})")
    def _filter_sharpen_gaus(self,gcmd, size=0, sigma=1.0, strength=1.5):
        blur = cv2.GaussianBlur(self.img, (int(size), int(size)), sigma)
        self.img = cv2.addWeighted(self.img, 1.0 + strength, blur, -strength, 0)
        if self.verbose: gcmd.respond_info(f"sharpen_mask({size}, {sigma}, {strength})")
    def _filter_sharpen_highpass(self,gcmd, size=0, sigma=1.0, strength=1.5):
        blur = cv2.GaussianBlur(self.img, (int(size), int(size)), sigma)
        high_pass = cv2.subtract(self.img, blur)
        self.img = cv2.addWeighted(self.img, 1.0 + strength, high_pass, -strength, 0)
        if self.verbose: gcmd.respond_info(f"sharpen_highpass({size}, {sigma}, {strength})")
    def _filter_sharpen_lapl_add(self,gcmd, sigma=1.0, strength=1.5):
        laplacian = cv2.Laplacian(self.img, cv2.CV_64F)
        lap = cv2.convertScaleAbs(laplacian)
        self.img = cv2.addWeighted(self.img, 1.0 + strength, lap, -strength, 0)
        if self.verbose: gcmd.respond_info(f"sharpen_lapl_add()")
    def _filter_sharpen_lapl_sub(self,gcmd):
        laplacian = cv2.Laplacian(self.img, cv2.CV_64F)
        lap = cv2.convertScaleAbs(laplacian)
        self.img =cv2.subtract(self.img, lap)
        if self.verbose: gcmd.respond_info(f"sharpen_lapl_sub()")
    def _filter_sharpen_bilateral(self, gcmd, block_size, sigCol, sigSiz, border=cv2.BORDER_DEFAULT):
        bilateral_filter = cv2.bilateralFilter(self.img,block_size,sigCol, sigSiz, border)
        self.img = cv2.subtract(self.img, bilateral_filter)
        if self.verbose: gcmd.respond_info(f"sharpen_bilateral({block_size, sigCol, sigSiz, border})")

    def _filter_morph_close(self, gcmd, kernel_size=5, iterations=1):
        size = int(kernel_size)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        self.img = cv2.morphologyEx(self.img, cv2.MORPH_CLOSE, kernel, iterations=int(iterations))
        if self.verbose: gcmd.respond_info(f"morph_close({kernel_size, iterations})")
    def _filter_morph_open(self, gcmd, kernel_size=5, iterations=1):
        size = int(kernel_size)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        self.img = cv2.morphologyEx(self.img, cv2.MORPH_OPEN, kernel, iterations=int(iterations))
        if self.verbose: gcmd.respond_info(f"morph_open({kernel_size, iterations})")
    def _filter_morph_tophat(self, gcmd, kernel_size=5, iterations=1):
        size = int(kernel_size)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        self.img = cv2.morphologyEx(self.img, cv2.MORPH_TOPHAT, kernel, iterations=int(iterations))
        if self.verbose: gcmd.respond_info(f"morph_tophat({kernel_size, iterations})")
    def _filter_morph_blackhat(self, gcmd, kernel_size=5, iterations=1):
        size = int(kernel_size)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        self.img = cv2.morphologyEx(self.img, cv2.MORPH_BLACKHAT, kernel, iterations=int(iterations))
        if self.verbose: gcmd.respond_info(f"morph_blackhat({kernel_size, iterations})")
    def _filter_erode(self, gcmd,count):
        self.img = cv2.erode(self.img, numpy.ones((3, 3), numpy.uint8), iterations=count)
        if self.verbose: gcmd.respond_info(f"erode({count})")
    def _filter_dilate(self, gcmd, count):
        self.img = cv2.dilate(self.img, numpy.ones((3, 3), numpy.uint8), iterations=count)
        if self.verbose: gcmd.respond_info(f"dilate({count})")
    def _filter_invert(self,gcmd):
        self.img = cv2.bitwise_not(self.img)
        if self.verbose: gcmd.respond_info(f"invert()")
    def _mask_op(self, gcmd, op, a, b, dst, label):
        ma = self._mask_from_stash(a)
        mb = self._mask_from_stash(b)
        if ma.shape != mb.shape:
            raise gcmd.error("%s: mask %s %s != %s %s"% (label, a, ma.shape, b, mb.shape))
        if dst is None:
            dst = a
        self.image_stash[dst] = op(ma, mb)
        if self.verbose:
            gcmd.respond_info("%s(%s,%s -> %s)" % (label, a, b, dst))
    def _filter_bitwise_and(self, gcmd, a, b, dst=None):
        """AND two stashed masks. bitwise_and(tape, roi [, dst]). dst defaults to a."""
        self._mask_op(gcmd, cv2.bitwise_and, a, b, dst, 'bitwise_and')
    def _filter_bitwise_or(self, gcmd, a, b, dst=None):
        self._mask_op(gcmd, cv2.bitwise_or, a, b, dst, 'bitwise_or')
    def _filter_bitwise_xor(self, gcmd, a, b, dst=None):
        self._mask_op(gcmd, cv2.bitwise_xor, a, b, dst, 'bitwise_xor')
    def _filter_bitwise_not(self, gcmd, a='mask', dst=None):
        ma = self._mask_from_stash(a)
        if dst is None:
            dst = a
        self.image_stash[dst] = cv2.bitwise_not(ma)
        if self.verbose:
            gcmd.respond_info("bitwise_not(%s -> %s)" % (a, dst))
    def _filter_apply_mask(self, gcmd, name='mask'):
        """AND the working image with a stashed mask (zeros outside)."""
        mask = self._mask_from_stash(name)
        self.img = cv2.bitwise_and(self.img, self.img, mask=mask)
        if self.verbose:
            gcmd.respond_info("apply_mask(%s)" % (name,))
    def _filter_add(self, gcmd, name='mask', other=None):
        self._arith(gcmd, cv2.add, name, other, 'add')
    def _filter_subtract(self, gcmd, name='mask', other=None):
        self._arith(gcmd, cv2.subtract, name, other, 'subtract')
    def _filter_use_mask(self, gcmd, name='mask'):
        """Replace the working image with the stashed 0/255 mask (gray)."""
        self.img = self._mask_from_stash(name).copy()
        if self.verbose: gcmd.respond_info(f"use_mask({name})")
    def _filter_fill(self, gcmd, name='mask', other='white'):
        """Paint other onto img where mask!=0. In-place — fuzzy outside stays.
        other: black/white/gray, scalar, or RGB hex (F0F magenta, FFF white)."""
        mask = self._mask_from_stash(name)
        self.img[mask != 0] = self._swatch(other)
        if self.verbose:
            gcmd.respond_info("fill(%s,%s)" % (name, other))
    def _filter_mask_circ(self, gcmd, cx, cy, radius, invert=0):
        mask = numpy.zeros(self.img.shape[:2], dtype=numpy.uint8)
        cv2.circle(mask, (int(cx), int(cy)), int(radius), 255, -1)
        self._stash_mask(mask, invert)
        if self.verbose: gcmd.respond_info(f"mask_circ({cx}, {cy}, {radius}, {invert})")
    def _filter_mask_rect(self, gcmd, x, y, w, h, invert=0):
        mask = numpy.zeros(self.img.shape[:2], dtype=numpy.uint8)
        w2=w/2;h2=h/2
        cv2.rectangle(mask, (int(x-w2), int(y-h2)), (int(x+w2), int(y+h2)), 255, -1)
        self._stash_mask(mask, invert)
        if self.verbose: gcmd.respond_info(f"mask_rect({x}, {y}, {w}, {h}, {invert})")
    def _filter_mask_centcirc(self, gcmd, radius, invert=0):
        mask = numpy.zeros(self.img.shape[:2], dtype=numpy.uint8)
        cv2.circle(mask, (int(self.cam.cx), int(self.cam.cy)), int(radius), 255, -1)
        self._stash_mask(mask, invert)
        if self.verbose: gcmd.respond_info(f"mask_centcirc({radius}, {invert})")
    def _filter_mask_centrect(self, gcmd, w, h=None, invert=0):
        if h is None: h=w
        cx=self.cam.cx; cy=self.cam.cy
        w2=w/2; h2=h/2
        mask = numpy.zeros(self.img.shape[:2], dtype=numpy.uint8)
        cv2.rectangle(mask, (int(cx-w2), int(cy-h2)), (int(cx+w2), int(cy+h2)), 255, -1)
        self._stash_mask(mask, invert)
        if self.verbose: gcmd.respond_info(f"mask_centrect({w}, {h}, {invert})")
    def _filter_canny(self, gcmd, threshold1=50, threshold2=150,apertureSize=3,L2gradient=0):
        self._require_gray(gcmd)
        self.img = cv2.Canny(self.img, float(threshold1), float(threshold2), apertureSize=apertureSize, L2gradient=L2gradient)
        if self.verbose: gcmd.respond_info(f"canny({threshold1}, {threshold2})")
    def _filter_sobelxy(self, gcmd,ksize):
        # Apply Sobel operator
        sobel_x = cv2.Sobel(self.img, cv2.CV_64F, 1, 0, ksize=ksize)  # Horizontal edges
        sobel_y = cv2.Sobel(self.img, cv2.CV_64F, 0, 1, ksize=ksize)  # Vertical edges
        grad = cv2.magnitude(sobel_x, sobel_y)
        self.img = cv2.convertScaleAbs(grad)
    def _filter_laplacian(self,gcmd):
        laplacian = cv2.Laplacian(self.img, cv2.CV_64F)
        self.img = cv2.convertScaleAbs(laplacian)
        if self.verbose: gcmd.respond_info(f"laplacian()")

    def _filter_find_contours(self, gcmd,min_area=50,max_area=999999):
        self._require_gray(gcmd)
        contours, _ = cv2.findContours(self.img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        self.detected_contours = []
        for c in contours:
            item=detection_item(c)
            if min_area <= item.area <= max_area:
                self.detected_contours.append(item)
        if self.verbose: gcmd.respond_info(f"find_contours({min_area}, {max_area}) all={len(contours)} -> fit={len(self.detected_contours)}")
    def _filter_find_contours_ex(self, gcmd,min_area=50,max_area=999999):
        """find_contours_ex() uses cv2.RETR_EXTERNAL to ignore nested contours."""
        self._require_gray(gcmd)
        contours, _ = cv2.findContours(self.img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.detected_contours = []
        for c in contours:
            item=detection_item(c)
            if min_area <= item.area <= max_area:
                self.detected_contours.append(item)
        if self.verbose: gcmd.respond_info(f"find_contours_ex({min_area}, {max_area}) all={len(contours)} -> fit={len(self.detected_contours)}")
    def _filter_find_contours_by_r(self, gcmd,min_r=50.,max_r=None):
        self._require_gray(gcmd)
        if max_r is None: max_r=1.2*min_r
        min_area=numpy.pi*min_r*min_r
        max_area=numpy.pi*max_r*max_r
        contours, _ = cv2.findContours(self.img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        self.detected_contours = []
        for c in contours:
            item=detection_item(c)
            if min_area <= item.area <= max_area:
                self.detected_contours.append(item)
        if self.verbose: gcmd.respond_info(f"find_contours_by_r({min_r}, {max_r}) all={len(contours)} -> fit={len(self.detected_contours)}")
    def _filter_find_contours_ex_by_r(self, gcmd,min_r=50.,max_r=None):
        """find_contours_ex() uses cv2.RETR_EXTERNAL to ignore nested contours."""
        self._require_gray(gcmd)
        if max_r is None: max_r=1.2*min_r
        min_area=numpy.pi*min_r*min_r
        max_area=numpy.pi*max_r*max_r
        contours, _ = cv2.findContours(self.img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.detected_contours = []
        for c in contours:
            item=detection_item(c)
            if min_area <= item.area <= max_area:
                self.detected_contours.append(item)
        if self.verbose: gcmd.respond_info(f"find_contours_ex_by_r({min_r}, {max_r}) all={len(contours)} -> fit={len(self.detected_contours)}")
    def _filter_circularity(self, gcmd, min_score=0.75):
        """
        Promote contours with high *circularity* 4πA/P² to circles.
        Usage: ...|find_contours_by_r(30,60)|circularity(0.75)|...
        """
        self.detected_circles = [i for i in self.detected_contours if i.test_circularity(min_score)]
        if self.verbose: gcmd.respond_info(f"circularity({min_score}) -> {len(self.detected_circles)} circles")
    def _filter_enclosing_circle(self, gcmd, min_fill=0.75):
        """
        minEnclosingCircle + fill_ratio = area/(πR²).
        Contours with fill_ratio >= min_fill become circles (cx,cy,r from enclosing).
        Stable under glare-ragged edges vs pure perimeter circularity.
        Usage: ...|find_contours_by_r(20,80)|enclosing_circle(0.75)|...
        """
        self.detected_circles = [i for i in self.detected_contours if i.test_enclosing_circle(float(min_fill))]
        if self.verbose: gcmd.respond_info(f"enclosing_circle({min_fill}) -> {len(self.detected_circles)} circles")
    def _filter_radial_symmetry(self, gcmd, min_sym=0.75, n_rays=32, edge_threshold=64, max_r=None):
        """
        Optical radial symmetry via raycast on current gray/binary frame.
        Runs on detected_circles if non-empty, else on detected_contours
        (promotes passers to circles). Fills item.symmetry.
        Usage: ...|circular_symmetry(0.7)|radial_symmetry(0.8)|...
               ...|find_contours_by_r(30,60)|radial_symmetry(0.85,48)|...
        """
        src = self.detected_circles if self.detected_circles else list(self.detected_contours)
        info=f"{len(src)} {'circles' if self.detected_circles else 'contours'}"
        self.detected_circles = [i for i in src if i.test_radial_symmetry(self.img, float(min_sym), n_rays=int(n_rays), edge_threshold=edge_threshold, max_r=max_r)]
        if self.verbose: gcmd.respond_info(f"radial_symmetry({min_sym}, {n_rays}) {info} -> {len(self.detected_circles)} circles")
    def _filter_blob_circles(self, gcmd, min_r=10., max_r=100.,
                             min_circularity=0.7, min_convexity=0.8,
                             min_inertia=0.5, blob_color=255):
        """
        OpenCV SimpleBlobDetector Works on current gray frame.
        (min_r=10., max_r=100., min_circularity=0.7,
        min_convexity=0.8, min_inertia=0.5, blob_color=255).
        blob_color: 255=bright blobs, 0=dark
        Usage: ...|gray()|otsu()|blob_circles(20,80)|...
               ...|blob_circles(15,50,0.8,0.85,0.5,0)|  # dark holes
        """
        self._require_gray(gcmd)
        min_r, max_r = float(min_r), float(max_r)
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = max(1.0, math.pi * min_r * min_r)
        params.maxArea = math.pi * max_r * max_r
        params.filterByCircularity = True
        params.minCircularity = float(min_circularity)
        params.filterByConvexity = True
        params.minConvexity = float(min_convexity)
        params.filterByInertia = True
        params.minInertiaRatio = float(min_inertia)
        params.filterByColor = True
        params.blobColor = int(blob_color)
        params.minThreshold = 0
        params.maxThreshold = 255
        # fewer threshold steps — cheaper on large frames
        params.thresholdStep = 20
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(self.img)
        self.detected_circles = []
        for kp in keypoints:
            r = float(kp.size) * 0.5
            if r < min_r or r > max_r:
                continue
            resp = float(kp.response) if kp.response else None
            self.detected_circles.append(
                detection_item.from_circle(kp.pt[0], kp.pt[1], r, score=resp, fill_ratio=1.0))
        if self.verbose: gcmd.respond_info(
            f"blob_circles({min_r}, {max_r}, {min_circularity}, {min_convexity}, "
            f"{min_inertia}, {int(blob_color)}) -> {len(self.detected_circles)} circles")
    def _filter_dist_peaks(self, gcmd, min_r=10., max_r=100.,
                           min_dist=3., threshold_rel=0.3, invert=0):
        """
        Distance-transform local maxima → circle centers.
        Great for regular sprocket holes / tape perforations (no Hough).
        Expects (or Otsu-builds) a binary where FG is the disk interior.
        (min_r=10., max_r=100., min_dist=None, threshold_rel=0.3, invert=0)
        invert=1 flips FG/BG before transform.
        large min_dist causes long runs
        Usage: ...|gray()|otsu()|dist_peaks(8,25)|...
               ...|dist_peaks(10,40,20,0.4)|
        """
        self._require_gray(gcmd)
        min_r, max_r = float(min_r), float(max_r)
        # binary FG=255
        # if self.img.max() <= 1:
        #     binary = (self.img * 255).astype(numpy.uint8)
        # else:
        #     # already binary-ish or gray — Otsu to be safe
        #     _, binary = cv2.threshold(
        #         self.img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # if int(invert) == 1:
        #     binary = cv2.bitwise_not(binary)
        dist = cv2.distanceTransform( self.img, cv2.DIST_L2, 5)
        ksize = max(3, int(round(min_dist)) | 1)  # odd
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
        local_max = cv2.dilate(dist, kernel)
        # local maxima plateaus within band
        peak_mask = (
            (dist >= min_r) & (dist <= max_r)
            & ((local_max - dist) < 1e-3)
        )
        dmax = float(dist.max()) if dist.size else 0.
        if dmax > 0:
            peak_mask &= dist >= (float(threshold_rel) * min(dmax, max_r))
        peak_u8 = peak_mask.astype(numpy.uint8) * 255
        nlab, _labels, _stats, centroids = cv2.connectedComponentsWithStats(
            peak_u8, connectivity=8)
        self.detected_circles = []
        h, w = dist.shape[:2]
        for i in range(1, nlab):
            cx, cy = float(centroids[i][0]), float(centroids[i][1])
            ix = min(w - 1, max(0, int(round(cx))))
            iy = min(h - 1, max(0, int(round(cy))))
            r = float(dist[iy, ix])
            if r < min_r or r > max_r:
                continue
            self.detected_circles.append(
                detection_item.from_circle(
                    cx, cy, r, score=r / max_r if max_r > 0 else 1., fill_ratio=1.0))
        if self.verbose: gcmd.respond_info(
            f"dist_peaks({min_r},{max_r},{min_dist}, {threshold_rel}, {invert}) "
            f"-> {len(self.detected_circles)} circles")
    def _filter_fit_ellipses(self, gcmd, min_symmetry=0.75):
        """
        Fit ellipses to detected contours (ideally after canny + find_contours).
        Promotes fitting contours to self.detected_circles if they match min_symmetry.
        """
        min_symmetry = float(min_symmetry)
        before = len(self.detected_contours)
        self.detected_circles = []
        for item in self.detected_contours:
            # cv2.fitEllipse needs at least 5 points to fit an ellipse
            if item.c is None or len(item.c) < 5:
                continue
            (cx, cy), (w, h), angle = cv2.fitEllipse(item.c.astype(numpy.float32))
            # Geometry quality (ratio of minor axis to major axis)
            sym = min(w, h) / max(w, h) if max(w, h) > 0 else 0
            if sym >= min_symmetry:
                ellipse_item = detection_item.from_ellipse(cx, cy, w, h, angle, symmetry=sym)
                self.detected_circles.append(ellipse_item)
        if self.verbose:
            gcmd.respond_info(f"fit_ellipses({min_symmetry}) contours_in={before} -> circles_out={len(self.detected_circles)}")

    def _filter_filter_circles_r(self, gcmd, min_r=0., max_r=9999.):
        """Keep circles whose r is in [min_r, max_r]."""
        min_r, max_r = float(min_r), float(max_r)
        before = len(self.detected_circles)
        self.detected_circles = [
            i for i in self.detected_circles
            if i.r is not None and min_r <= float(i.r) <= max_r]
        if self.verbose: gcmd.respond_info(
            f"filter_circles_r({min_r}, {max_r}): "
            f"in={before} -> out={len(self.detected_circles)}")
    def _filter_filter_circles_score(self, gcmd, min_score=0.75):
        """Keep circles with circularity score >= min_score."""
        min_score = float(min_score)
        before = len(self.detected_circles)
        self.detected_circles = [
            i for i in self.detected_circles
            if i.score is not None and float(i.score) >= min_score]
        if self.verbose: gcmd.respond_info(
            f"filter_circles_score({min_score}): "
            f"in={before} -> out={len(self.detected_circles)}")
    def _cluster_items(self, items, pair_ok):
        """Union-find groups; pair_ok(a, b) is True when a and b should merge."""
        n = len(items)
        if n <= 1:
            return [items] if n else []
        parent = list(range(n))
        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i
        for i in range(n):
            for j in range(i + 1, n):
                if pair_ok(items[i], items[j]):
                    ri, rj = find(i), find(j)
                    if ri != rj:
                        parent[rj] = ri
        groups = {}
        for i in range(n):
            groups.setdefault(find(i), []).append(items[i])
        return list(groups.values())
    def _filter_merge_concentric(self, gcmd, max_dist=8., how='mean'):
        """
        Collapse concentric circle detections (inner+outer edge of a hole/ring)
        into one. Centers always averaged (weighted by c_score); radius by how:
        mean (default), min, max, score (keep r of the best-scoring member).
        Two circles merge when center distance <= max_dist (pixels).
        Singletons pass through unchanged.
        Usage: ...|circularity(0.8)|filter_circles_r(min_r,max_r)|merge_concentric(8)|...
               ...|fit_ellipses(0.8)|merge_concentric(5,max)|...
        """
        max_dist = float(max_dist)
        how = str(how).lower()
        src = [i for i in self.detected_circles
               if i.cx is not None and i.cy is not None and i.r is not None]
        n = len(src)
        if n <= 1:
            self.detected_circles = src
            if self.verbose:
                gcmd.respond_info(f"merge_concentric({max_dist}, {how}): n={n}")
            return
        max_d2 = max_dist * max_dist
        def pair_ok(a, b):
            dx = a.cx - b.cx
            dy = a.cy - b.cy
            return dx * dx + dy * dy <= max_d2
        merged = []
        n_collapsed = 0
        for items in self._cluster_items(src, pair_ok):
            if len(items) == 1:
                merged.append(items[0])
                continue
            n_collapsed += len(items)
            merged.append(self._average_circles(items, how))
        self.detected_circles = merged
        if self.verbose:
            gcmd.respond_info(
                f"merge_concentric({max_dist}, {how}): "
                f"in={n} groups={len(merged)} collapsed={n_collapsed}")
    def _average_circles(self, items, how):
        """Weighted mean of a concentric group → one detection_item."""
        weights = []
        for it in items:
            w = float(it.c_score) if it.c_score is not None and it.c_score > 0 else 1.0
            weights.append(w)
        sw = sum(weights)
        cx = sum(it.cx * w for it, w in zip(items, weights)) / sw
        cy = sum(it.cy * w for it, w in zip(items, weights)) / sw
        if how == 'min':
            r = min(it.r for it in items)
            seed = min(items, key=lambda it: it.r)
        elif how in ('max', 'outer'):
            r = max(it.r for it in items)
            seed = max(items, key=lambda it: it.r)
        elif how in ('score', 'best'):
            seed = max(items, key=lambda it: it.c_score or 0.)
            r = seed.r
        else:
            r = sum(it.r * w for it, w in zip(items, weights)) / sw
            seed = max(items, key=lambda it: it.c_score or 0.)
        score = sum((it.c_score or 0.) * w for it, w in zip(items, weights)) / sw
        fills = [it.fill_ratio for it in items if it.fill_ratio is not None]
        fill = (sum(fills) / len(fills)) if fills else None
        syms = [it.symmetry for it in items if it.symmetry is not None]
        sym = (sum(syms) / len(syms)) if syms else None
        out = detection_item.from_circle(
            cx, cy, r, score=score, symmetry=sym, fill_ratio=fill)
        out.c = seed.c
        if seed.c is not None:
            out.perimeter = seed.perimeter
        return out
    @staticmethod
    def _angle_diff_deg(a, b):
        """Smallest difference of long-side angles, period 180°."""
        diff = abs(float(a) - float(b)) % 180.
        if diff > 90.:
            diff = 180. - diff
        return diff
    def _filter_merge_rects(self, gcmd, max_dist=8., how='mean', max_ang=15.):
        """
        Collapse nested/concentric rects (inner+outer edge of one feature) into one.
        Pads, body and pocket share a center — run filter_rects_width / aspect
        first so only same-size inner+outer edges remain to merge.
        Merge when center distance <= max_dist and long-side angle differs by
        <= max_ang (deg, 180° period). Centers averaged; w/h by how:
        mean (default), min, max, score (keep geometry of largest-area member).
        Singletons pass through unchanged.
        Usage: ...|make_rects()|filter_rects_width(min_w_px,max_w_px)|merge_rects(8)|...
               ...|filter_rects_rot(rot,rot_tol)|merge_rects(8,max,20)|...
        """
        max_dist = float(max_dist)
        how = str(how).lower()
        max_ang = float(max_ang)
        src = [i for i in self.detected_rects
               if i.cx is not None and i.cy is not None
               and i.w is not None and i.h is not None]
        n = len(src)
        if n <= 1:
            self.detected_rects = src
            if self.verbose:
                gcmd.respond_info(
                    f"merge_rects({max_dist}, {how}, {max_ang}): n={n}")
            return
        max_d2 = max_dist * max_dist
        def pair_ok(a, b):
            dx = a.cx - b.cx
            dy = a.cy - b.cy
            if dx * dx + dy * dy > max_d2:
                return False
            return self._angle_diff_deg(a.angle, b.angle) <= max_ang
        merged = []
        n_collapsed = 0
        for items in self._cluster_items(src, pair_ok):
            if len(items) == 1:
                merged.append(items[0])
                continue
            n_collapsed += len(items)
            merged.append(self._average_rects(items, how))
        self.detected_rects = merged
        if self.verbose:
            gcmd.respond_info(
                f"merge_rects({max_dist}, {how}, {max_ang}): "
                f"in={n} groups={len(merged)} collapsed={n_collapsed}")
    def _average_rects(self, items, how):
        """Mean of a nested-rect group → one detection_item."""
        n = float(len(items))
        cx = sum(it.cx for it in items) / n
        cy = sum(it.cy for it in items) / n
        if how == 'min':
            seed = min(items, key=lambda it: it.area)
            w, h, angle = seed.w, seed.h, seed.angle
        elif how in ('max', 'outer'):
            seed = max(items, key=lambda it: it.area)
            w, h, angle = seed.w, seed.h, seed.angle
        elif how in ('score', 'best'):
            seed = max(items, key=lambda it: it.area)
            w, h, angle = seed.w, seed.h, seed.angle
        else:
            seed = max(items, key=lambda it: it.area)
            w = sum(it.w for it in items) / n
            h = sum(it.h for it in items) / n
            # circular mean, 180° period (long-side orientation)
            s = c = 0.
            for it in items:
                rad = math.radians(2. * it.angle)
                s += math.sin(rad)
                c += math.cos(rad)
            angle = 0.5 * math.degrees(math.atan2(s, c))
        out = detection_item.from_rect(cx, cy, w, h, angle, c=seed.c)
        if seed.c is not None:
            out.perimeter = seed.perimeter
        return out
    def _filter_sort_proximity(self, gcmd, cx, cy):
        """
        Sort detected_circles and detected_rects nearest-first to (cx,cy).
        cx/cy = camera principal point (HUD center). but may be expected location form motion-prediction,
        Usage: ...|circular_symmetry(0.75)|sort_proximity(cx,cy)|...
               ...|sort_proximity(800,600)|...
        """
        def _dist2(i):
            return (i.cx - cx) ** 2 + (i.cy - cy) ** 2
        if self.detected_circles:
            self.detected_circles = sorted(self.detected_circles, key=_dist2)
        if self.detected_rects:
            self.detected_rects = sorted(self.detected_rects, key=_dist2)
    def _filter_pick_nearest(self, gcmd, cx, cy, keep=1):
        """
        Keep only the N nearest circles/rects to (cx,cy); drop the rest.
        cx/cy are in auto-params and typically the camera principal point (HUD center).,
        but may be expected location form motion-prediction,
        Usage: ...|circular_symmetry(0.75)|pick_nearest(cx,cy)|
        """
        self._filter_sort_proximity(gcmd, cx, cy)
        n = max(1, int(keep))
        if self.detected_circles:
            before = len(self.detected_circles)
            self.detected_circles = self.detected_circles[:n]
            if self.verbose: gcmd.respond_info(
                f"pick_nearest circles keep={n} (was {before}) "
                f"-> {self.detected_circles}")
        if self.detected_rects:
            before = len(self.detected_rects)
            self.detected_rects = self.detected_rects[:n]
            if self.verbose: gcmd.respond_info(
                f"pick_nearest rects keep={n} (was {before}) "
                f"-> {self.detected_rects}")

    def _filter_subpixel_circle(self, gcmd, stash_name="subpix", min_symmetry=0.75, search_padding=5):
        """
        Takes raw contours or rough circles, cuts ROI from stashed gray image,
        calculates high-precision subpixel edges and fits mathematical ellipse.
        """
        min_symmetry = float(min_symmetry)
        gray_img = self.image_stash.get(stash_name)
        if gray_img is None:
            raise gcmd.error(f"Stash '{stash_name}' not found for subpixel refinement")
        h_img, w_img = gray_img.shape[:2]
        src = self.detected_circles if self.detected_circles else list(self.detected_contours)
        before = len(src)
        self.detected_circles = []
        for item in src:
            if item.cx is None or item.cy is None:
                continue
            r_search = item.r if item.r and item.r > 0 else 15
            pad = int(search_padding)
            x1 = max(0, int(item.cx - r_search - pad))
            y1 = max(0, int(item.cy - r_search - pad))
            x2 = min(w_img, int(item.cx + r_search + pad))
            y2 = min(h_img, int(item.cy + r_search + pad))
            if (x2 - x1) < 5 or (y2 - y1) < 5:
                continue
            roi = gray_img[y1:y2, x1:x2]
            edges = cv2.Canny(roi, 40, 130)
            pts = numpy.argwhere(edges > 0)
            if len(pts) < 5:
                continue
            # return to frame coordinates
            pts_global = numpy.fliplr(pts) + numpy.array([x1, y1])
            (cx_sub, cy_sub), (w_sub, h_sub), angle = cv2.fitEllipse(pts_global.astype(numpy.float32))
            sym = min(w_sub, h_sub) / max(w_sub, h_sub) if max(w_sub, h_sub) > 0 else 0
            if sym >= min_symmetry:
                sub_item = detection_item.from_ellipse(cx_sub, cy_sub, w_sub, h_sub, angle, symmetry=sym)
                self.detected_circles.append(sub_item)
        if self.verbose:
            gcmd.respond_info(f"subpixel_circle('{stash_name}', {min_symmetry}): in={before} -> out={len(self.detected_circles)}")

    def _filter_make_rects(self,gcmd):
        self.detected_rects=[item for item in self.detected_contours if item.test_rect()]
        if self.verbose: gcmd.respond_info(f"rect_sym detected_rects={len(self.detected_rects)}")

    def _filter_subpixel_rect(self, gcmd, stash_name="subpix", search_padding=8):
        """
        Takes rough rects from self.detected_rects, separates points into 4 clusters
        (one per side) using the stashed gray image gradient, fits high-precision
        subpixel lines via cv2.fitLine, and recomputes the exact rect intersections.
        """
        gray_img = self.image_stash.get(stash_name)
        if gray_img is None:
            raise gcmd.error(f"Stash '{stash_name}' not found for subpixel rect refinement")
        h_img, w_img = gray_img.shape[:2]
        before = len(self.detected_rects)
        refined_rects = []
        for item in self.detected_rects:
            if item.box is None or item.cx is None:
                continue
            # make cutout ROI around the rough rectangle with padding and Canny it
            box = item.box
            x1 = max(0, int(numpy.min(box[:, 0]) - int(search_padding)))
            y1 = max(0, int(numpy.min(box[:, 1]) - int(search_padding)))
            x2 = min(w_img, int(numpy.max(box[:, 0]) + int(search_padding)))
            y2 = min(h_img, int(numpy.max(box[:, 1]) + int(search_padding)))
            if (x2 - x1) < 10 or (y2 - y1) < 10:
                continue
            roi_gray = gray_img[y1:y2, x1:x2]
            edges = cv2.Canny(roi_gray, 40, 130)
            pts = numpy.argwhere(edges > 0)
            if len(pts) < 20: # not enough edge points to fit lines
                continue
            # return to frame coordinates
            pts_global = numpy.fliplr(pts) + numpy.array([x1, y1])
            # sort points by their distance to each side
            sides_pts = [[] for _ in range(4)]
            for pt in pts_global:
                pt_f = pt.astype(numpy.float64)
                min_dist = float('inf')
                best_side = 0
                for s in range(4):
                    p1 = box[s].astype(numpy.float64)
                    p2 = box[(s + 1) % 4].astype(numpy.float64)
                    num = abs((p2[1]-p1[1])*pt_f[0] - (p2[0]-p1[0])*pt_f[1] + p2[0]*p1[1] - p2[1]*p1[0])
                    den = math.sqrt((p2[1]-p1[1])**2 + (p2[0]-p1[0])**2)
                    dist = num / den if den > 0 else float('inf')
                    if dist < min_dist:
                        min_dist = dist
                        best_side = s
                if min_dist < int(search_padding):
                    sides_pts[best_side].append(pt_f)
            # find subpixel lines for each side using cv2.fitLine
            lines = []
            valid_rect = True
            for s in range(4):
                if len(sides_pts[s]) < 4: # not enough points on some side
                    valid_rect = False
                    break
                pts_side = numpy.array(sides_pts[s], dtype=numpy.float32)
                vx, vy, x0, y0 = cv2.fitLine(pts_side, cv2.DIST_L2, 0, 0.01, 0.01)
                lines.append((float(vx), float(vy), float(x0), float(y0)))
            if not valid_rect:
                continue
            # find intersections of the 4 lines to get refined corners
            refined_corners = []
            for i in range(4):
                vx1, vy1, x1_0, y1_0 = lines[i]
                vx2, vy2, x2_0, y2_0 = lines[(i + 1) % 4]
                det = vx1 * vy2 - vy1 * vx2
                if abs(det) < 1e-5:
                    valid_rect = False
                    break
                t = ((x2_0 - x1_0) * vy2 - (y2_0 - y1_0) * vx2) / det
                cx_corner = x1_0 + vx1 * t
                cy_corner = y1_0 + vy1 * t
                refined_corners.append([cx_corner, cy_corner])
            if not valid_rect or len(refined_corners) != 4:
                continue
            # assemble refined rectangle from corners
            corners_arr = numpy.array(refined_corners, dtype=numpy.float32)
            new_rect = cv2.minAreaRect(corners_arr)
            # construct a new detection_item for the refined rectangle
            refined_item = detection_item(item.c) # zachowujemy oryginalny kontur w razie czego
            refined_item.rect = new_rect
            refined_item.box = numpy.int64(cv2.boxPoints(new_rect)) # int64 dla rysowania, ale dane wewnątrz są float
            ((refined_item.cx, refined_item.cy), (refined_item.w, refined_item.h), refined_item.angle) = new_rect
            # normalize angle and dimensions
            if refined_item.w < refined_item.h:
                refined_item.angle -= 90.
                refined_item.w, refined_item.h = refined_item.h, refined_item.w
            if refined_item.h == 0:
                continue
            refined_item.type = 'rect'
            refined_item.ratio = refined_item.w / refined_item.h
            refined_item.area = refined_item.w * refined_item.h
            refined_item.r = float(numpy.sqrt(refined_item.area / numpy.pi))
            refined_rects.append(refined_item)
        self.detected_rects = refined_rects
        if self.verbose:
            gcmd.respond_info(f"subpixel_rect('{stash_name}'): in={before} -> out={len(self.detected_rects)}")

    def _filter_filter_rects_aspect(self, gcmd,min_aspect=0.1, max_aspect=10):
        valid_rects = [item for item in self.detected_rects if float(min_aspect) <= item.ratio <= float(max_aspect)]
        if self.verbose: gcmd.respond_info(f"filter_rects_aspect: in={len(self.detected_rects)} out={len(valid_rects)}")
        self.detected_rects = valid_rects
    def _filter_filter_rects_width(self, gcmd,min_width_px=0, max_width_px=9999):
        valid_rects = [item for item in self.detected_rects if float(min_width_px) <= item.w <= float(max_width_px)]
        if self.verbose: gcmd.respond_info(f"filter_rects_width: in={len(self.detected_rects)} out={len(valid_rects)}")
        self.detected_rects = valid_rects
    def _filter_filter_rects_rot(self, gcmd, rot=0., tol=90.):
        """Keep rects whose angle aligns with rot within ± tol."""
        def _ok(a):
            diff = abs(a - rot)
            if diff > 90:
                diff = 180 - diff
            return diff <= tol
        valid = [item for item in self.detected_rects if _ok(item.angle)]
        if self.verbose:
            gcmd.respond_info(
                "filter_rects_rot(%s,%s): in=%d out=%d"
                % (rot, tol, len(self.detected_rects), len(valid)))
        self.detected_rects = valid
    def _filter_filter_square_rot(self, gcmd, rot=0., tol=10.):
        """Keep rects whose angle (folded to [0,90)) sits in [min_rot, max_rot].
        use for square-like rects (0.9<ratio<1.1) to catch flickering 90° rotations."""
        rot = rot % 90
        def _ok(a):
            diff = abs((a % 90) - rot)
            if diff > 45:
                diff = 90 - diff
            return diff <= tol
        valid = [item for item in self.detected_rects if _ok(item.angle)]
        if self.verbose:
            gcmd.respond_info(
                "filter_square_rot(%s,%s): in=%d out=%d"
                % (rot, tol, len(self.detected_rects), len(valid)))
        self.detected_rects = valid

    def _filter_stash(self, gcmd, name="stash"):
        self.image_stash[name] = self.img.copy()
    def _filter_recall(self, gcmd, name="stash"):
        self.img = self.image_stash[name].copy()
    def _filter_photo(self, gcmd, fname="/tmp/pnp_pipeline_{self.cam.name}_{self.name}_%Y%m%d_%H%M%S.jpeg"):
        filename = datetime.datetime.now().strftime(fname.format(self=self))
        cv2.imwrite(filename, self.img)
        gcmd.respond_info(f"Snapshot saved to {filename}.")

    def _filter_draw_circles(self, gcmd):
        self._require_bgr(gcmd)
        for item in self.detected_circles:
            cv2.circle(self.img, (int(item.cx), int(item.cy)), int(item.r), (0, 255, 0), 2)
            cv2.circle(self.img, (int(item.cx), int(item.cy)), 2, (0, 0, 255), 3)
        if self.verbose: gcmd.respond_info(f"draw_circles: {len(self.detected_circles)} items.")
    def _filter_draw_rects(self, gcmd):
        self._require_bgr(gcmd)
        for item in self.detected_rects:
            cv2.drawContours(self.img, [item.box], 0, (255, 0, 255), 2)
            rad = math.radians(item.angle)
            dir_x = math.sin(rad)
            dir_y = -math.cos(rad)
            offset_distance = item.h / 2.0
            arrow_start = (int(item.cx), int(item.cy))
            arrow_end = (int(item.cx + dir_x * (offset_distance + 20)), int(item.cy + dir_y * (offset_distance + 20)))
            cv2.arrowedLine(self.img, arrow_start, arrow_end, (255, 255, 0), 2, tipLength=0.2)
            cv2.drawMarker(self.img, (int(item.cx), int(item.cy)), (0, 0, 255), cv2.MARKER_CROSS, 15, 1)
        if self.verbose: gcmd.respond_info(f"draw_rects: {len(self.detected_rects)} items.")
    def _filter_draw_contours(self, gcmd, thickness=1):
        self._require_bgr(gcmd)
        for item in self.detected_contours:
            cv2.drawMarker(self.img, (int(item.cx), int(item.cy)), (255, 0, 0), cv2.MARKER_CROSS, 10, 1)
            cv2.drawContours(self.img, [item.c], -1, (0, 255, 0), int(thickness))
        # cv2.drawContours(self.img, [item.c for item in self.detected_contours], -1, (0, 255, 0), int(thickness))
        if self.verbose: gcmd.respond_info(f"draw_contours: {len(self.detected_contours)} items.")

    def _filter_draw_circmask(self, gcmd,cx,cy,r):
        cv2.circle(self.img, (int(cx), int(cy)), int(r), (0, 255, 255), 2)
    def _filter_list_circles(self,gcmd):
        for item in self.detected_circles:
            gcmd.respond_info(str(item))
    def _filter_list_rects(self,gcmd):
        for item in self.detected_rects:
            gcmd.respond_info(str(item))
