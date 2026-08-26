# klippy/extras/pnp_vision.py
# Dynamic vision pipelines processor
#
# Copyright (C) 2026 Maja Stanislawska <maja@makershop.ie>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import re,datetime
import cv2,numpy,math
import logging
from .camera_v4l import parse_light

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
    def test_radial_symmetry(self, img, min_sym, n_rays=32, max_r=None):
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
                if abs(int(img[y, x]) - center_val) > 64:
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
        active_params.update(runtime_params)
        self.verbose=active_params.get('verbose',False)
        for func_name, raw_args in self.compiled_steps:
            method = getattr(self, f"_filter_{func_name}", None)
            if not method:
                raise gcmd.error(f"Unknown filter '{func_name}'")
            # replace params (ex. 'max_r') with values from active_params
            evaluated_args = [gcmd]
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

    def _filter_blurmedian(self, gcmd, size):
        self.img = cv2.medianBlur(self.img, int(size))
        if self.verbose: gcmd.respond_info(f"blurmedian({size})")
    def _filter_blurgausian(self, gcmd, size):
        self.img = cv2.GaussianBlur(self.img, (int(size), int(size)), 0)
        if self.verbose: gcmd.respond_info(f"blurgausian({size})")
    def _filter_color(self, gcmd):
        self.img = cv2.cvtColor( self.img, cv2.COLOR_GRAY2BGR)
        if self.verbose: gcmd.respond_info(f"color()")
    def _filter_gray(self, gcmd):
        self.img = cv2.cvtColor( self.img, cv2.COLOR_BGR2GRAY)
        if self.verbose: gcmd.respond_info(f"gray()")
    def _filter_threshold(self, gcmd,thresh_val=128, max_val=255, method=cv2.THRESH_BINARY):
        _, self.img = cv2.threshold(self.img, thresh_val, max_val, method)
        if self.verbose: gcmd.respond_info(f"threshold({thresh_val, max_val})")
    def _filter_otsu(self, gcmd,thresh_val=0, max_val=255, method=cv2.THRESH_BINARY):
        _, self.img = cv2.threshold(self.img, thresh_val, max_val, method+cv2.THRESH_OTSU)
        if self.verbose: gcmd.respond_info(f"otsu({thresh_val, max_val})")
    def _filter_threshmean(self, gcmd, block_size, c, max_value=225, method=cv2.THRESH_BINARY):
        self.img = cv2.adaptiveThreshold(self.img, max_value, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, block_size, c)
        if self.verbose: gcmd.respond_info(f"threshmean({block_size, c, max_value})")
    def _filter_threshgaus(self, gcmd, block_size, c, max_value=225, method=cv2.THRESH_BINARY):
        self.img = cv2.adaptiveThreshold(self.img, max_value, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, block_size, c)
        if self.verbose: gcmd.respond_info(f"threshgaus({block_size, c, max_value})")
    def _filter_mask_hsv(self, gcmd, h_min, h_max, s_min, s_max,  v_min, v_max, invert=0):
        """
        mask_hsv(h_min, h_max, s_min, s_max,  v_min, v_max)
        h=[0..180], s=[0..255],v=[0..255]
        """
        hsv_frame = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        lower_bound = numpy.array([int(h_min), int(s_min), int(v_min)], dtype=numpy.uint8)
        upper_bound = numpy.array([int(h_max), int(s_max), int(v_max)], dtype=numpy.uint8)
        mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
        if int(invert) == 1: mask = cv2.bitwise_not(mask)
        self.img = cv2.bitwise_and(self.img, self.img, mask=mask)
        if self.verbose: gcmd.respond_info(f"hsv({h_min, h_max, s_min, s_max,  v_min, v_max, invert})")

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
    def _filter_mask_circ(self, gcmd, cx, cy, radius, invert=0):
        mask = numpy.zeros(self.img.shape[:2], dtype=numpy.uint8)
        cv2.circle(mask, (int(cx), int(cy)), int(radius), 255, -1)
        if int(invert) == 1: mask = cv2.bitwise_not(mask)
        self.img = cv2.bitwise_and(self.img, self.img, mask=mask)
        if self.verbose: gcmd.respond_info(f"mask_circ({cx}, {cy}, {radius}, {invert})")
    def _filter_mask_rect(self, gcmd, x, y, w, h, invert=0):
        mask = numpy.zeros(self.img.shape[:2], dtype=numpy.uint8)
        w2=w/2;h2=h/2
        cv2.rectangle(mask, (int(x-w2), int(y-h2)), (int(x+w2), int(y+h2)), 255, -1)
        if int(invert) == 1: mask = cv2.bitwise_not(mask)
        self.img = cv2.bitwise_and(self.img, self.img, mask=mask)
        if self.verbose: gcmd.respond_info(f"mask_rect({x}, {y}, {w}, {h}, {invert})")
    def _filter_mask_centcirc(self, gcmd, radius, invert=0):
        mask = numpy.zeros(self.img.shape[:2], dtype=numpy.uint8)
        cv2.circle(mask, (int(self.cam.cx), int(self.cam.cy)), int(radius), 255, -1)
        if int(invert) == 1: mask = cv2.bitwise_not(mask)
        self.img = cv2.bitwise_and(self.img, self.img, mask=mask)
        if self.verbose: gcmd.respond_info(f"mask_centcirc({radius}, {invert})")
    def _filter_mask_centrect(self, gcmd, w, h=None, invert=0):
        if h is None: h=w
        cx=self.cam.cx; cy=self.cam.cy
        w2=w/2; h2=h/2
        mask = numpy.zeros(self.img.shape[:2], dtype=numpy.uint8)
        cv2.rectangle(mask, (int(cx-w2), int(cy-h2)), (int(cx+w2), int(cy+h2)), 255, -1)
        if int(invert) == 1: mask = cv2.bitwise_not(mask)
        self.img = cv2.bitwise_and(self.img, self.img, mask=mask)
        if self.verbose: gcmd.respond_info(f"mask_centrect({w}, {h}, {invert})")
    def _filter_canny(self, gcmd, threshold1=50, threshold2=150):
        self.img = cv2.Canny(self.img, float(threshold1), float(threshold2))
        if self.verbose: gcmd.respond_info(f"canny({threshold1}, {threshold2})")

    def _filter_find_contours(self, gcmd,min_area=50,max_area=999999):
        contours, _ = cv2.findContours(self.img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        self.detected_contours = []
        for c in contours:
            item=detection_item(c)
            if min_area <= item.area <= max_area:
                self.detected_contours.append(item)
        if self.verbose: gcmd.respond_info(f"find_contours({min_area}, {max_area}) all={len(contours)} -> fit={len(self.detected_contours)}")
    def _filter_find_contours_ex(self, gcmd,min_area=50,max_area=999999):
        """find_contours_ex() uses cv2.RETR_EXTERNAL to ignore nested contours."""
        contours, _ = cv2.findContours(self.img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.detected_contours = []
        for c in contours:
            item=detection_item(c)
            if min_area <= item.area <= max_area:
                self.detected_contours.append(item)
        if self.verbose: gcmd.respond_info(f"find_contours_ex({min_area}, {max_area}) all={len(contours)} -> fit={len(self.detected_contours)}")
    def _filter_find_contours_by_r(self, gcmd,min_r=50.,max_r=None):
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
    def _filter_radial_symmetry(self, gcmd, min_sym=0.75, n_rays=32, max_r=None):
        """
        Optical radial symmetry via raycast on current gray/binary frame.
        Runs on detected_circles if non-empty, else on detected_contours
        (promotes passers to circles). Fills item.symmetry.
        Usage: ...|circular_symmetry(0.7)|radial_symmetry(0.8)|...
               ...|find_contours_by_r(30,60)|radial_symmetry(0.85,48)|...
        """
        src = self.detected_circles if self.detected_circles else list(self.detected_contours)
        info=f"{len(src)} {'circles' if self.detected_circles else 'contours'}"
        self.detected_circles = [i for i in src if i.test_radial_symmetry(self.img, float(min_sym), n_rays=int(n_rays), max_r=max_r)]
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

    def _filter_make_rects(self,gcmd):
        self.detected_rects=[item for item in self.detected_contours if item.test_rect()]
        if self.verbose: gcmd.respond_info(f"rect_sym detected_rects={len(self.detected_rects)}")
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
        for item in self.detected_circles:
            cv2.circle(self.img, (int(item.cx), int(item.cy)), int(item.r), (0, 255, 0), 2)
            cv2.circle(self.img, (int(item.cx), int(item.cy)), 2, (0, 0, 255), 3)
        if self.verbose: gcmd.respond_info(f"draw_circles: {len(self.detected_circles)} items.")
    def _filter_draw_rects(self, gcmd):
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
        for item in self.detected_contours:
            cv2.drawMarker(self.img, (int(item.cx), int(item.cy)), (255, 0, 0), cv2.MARKER_CROSS, 10, 1)
            cv2.drawContours(self.img, [item.c], -1, (0, 255, 0), int(thickness))
        # cv2.drawContours(self.img, [item.c for item in self.detected_contours], -1, (0, 255, 0), int(thickness))
        if self.verbose: gcmd.respond_info(f"draw_contours: {len(self.detected_contours)} items.")

    def _filter_list_circles(self,gcmd):
        for item in self.detected_circles:
            gcmd.respond_info(str(item))
    def _filter_list_rects(self,gcmd):
        for item in self.detected_rects:
            gcmd.respond_info(str(item))
