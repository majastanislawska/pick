# PnP vision and V4L2 cameras

This document describes the pick-and-place vision stack added on top of
Klipper: native V4L2 cameras (`camera_v4l`), dynamic detection pipelines
(`pnp_vision` / `pnp_vision_pipeline`), mm/pixel scale calibration, and
vision-based XY home on machine fiducials.

See also [G-Codes](G-Codes.md#pnp_vision) for a command reference.

## Architecture

```text
[camera_v4l name]
  V4L2 FD (reactor) → frame queue → VisionWorker (decode, undistort, HUD)
                                 → MJPEG HTTP (per-camera server)
                                 → get_snapshot() / set_overlay()
                                 → pixels_to_mm_offset()

[pnp_vision]
  pipelines (named filter graphs)
  PNP_VISION_DETECT / HOME
  uses cameras registered at connect

[pnp_calibration]
  PNP_CALIB_PXMM / PNP_CALIB_CAMERA  (ACCEPT / DONE / ABORT)

[pnp]
  fiducial_primary / fiducial_secondary
  nozzle offsets, etc.
```

- Capture runs on Klipper’s reactor via non-blocking V4L2 + MMAP.
- Heavy OpenCV work runs in a daemon worker thread so the main loop is
  not blocked.
- there's `fps_divider` (default 3) to decrease system load if your camera cannot lower framerate. (you can also set longer exposure and lower brightness and gain)
- Worker's frame queue is short to avoid buildup of latency
- Each camera instance has its own HTTP port and MJPEG stream state
- get_snapshot() pulls undistorted frame from VisionWorker via Klipper's reactor.completion()

### [pnp_vision]

- Capable cameras register themselves at startup (klipper's connect) `CAM=` looks there instead of typical klipper's `self.printer.lookup_object()`
- pipelines are registered on creation by gcode command `PNP_VISION_PIPELINE`.
- calling `PNP_VISION_PIPELINE` with same name overwrites/"updates" it.
- pipeline steps are parsed by regexp to list of tuples (name, [parameters..])
- those are evaluated in `run()`: method `_filter_{name}` is called
- `gcmd` is pushed as first parameter so filter can do `gcmd.respond_info()`
- remaining are substituted with values from `active_params` or cast to int/float if possible or left as strings (_filters can raise an error if they dont like type)
- `active_params` dict is created from pipeline's`PARAMS=` (which work as defaults) updated by `runtime_params` dict passed to `run()` by caller.
- caller typically gathers `runtime_params` from config section of what it's trying to detect (ex fiducial) and it's own gcode parmeters

### Parameters conventions

All gcode commands can take arbitrary number of key=value pairs that are then passed in `params` dict to pipeline, method `parse_runtime_params()` does that. This can be used to either set params for pipeline or owerwrite values set elsewhere.  
Some of them are picked earlier and passed through `_size_params_from_mm()` to compute additional parameters using current camera pose (z height) those are pixel units `min_`/`max_` parameter pairs.
Some parameters may orginate from config file sections (fiducial sizes, nozzle tip diameters etc), those are passed through `_size_params_from_mm()` too.
But `_size_params_from_mm()` does not overwrite any existing config params (you can put pixel param to config to refine computed value)

Commandline params have precedence over any other source.

| name | source | description | purpose |
| --- | --- | --- | --- |
| `cx`,`cy` | code | (pixels) typically camera principal point but sometimes center of ROI (expected target position) | `mask_circ`, `mask_rect`, `pick_nearest` |
| `VERBOSE` | gcode | Set to `1` to make pipeline more noisy (shows params substitutions) | |
| `LIGHT` | pipeline,gcode | default lightspec if on pipeline or override of it if on GCODE commad | `run_detect()` |
| `TOL` | config,gcode | a 'tolerance': sets spread for `min_`/`max_` parameter pairs | `_size_params_from_mm` |
| `DIA` | config,gcode | diameter in mm for circle detection pipelines (fiducials) | `_size_params_from_mm` |
| `W` | config,gcode | width in mm for rect detection pipelines | `_size_params_from_mm` |
| `H` | config,gcode | height in mm for rect detection pipelines | `_size_params_from_mm` |
| `ROT` | config,gcode | rotation (deg) for rect detection pipelines | `filter_rects_rot`,`filter_square_rot` |
| `ROT_TOL` | config,gcode | tolerance for rotation for rect detection pipelines | `filter_rects_rot`,`filter_rects_rot` |
| `min_area`,`max_area` | `_size_params_from_mm` | area in px for contour detection filters (either  Pi * (r_px)^2 or w_px * h_px) | `find_contours` |
| `min_r`, `max_r` | `_size_params_from_mm` | radius in px for circle detection | `find_contours_by_r`, `filter_circles_r` |
| `min_w_px`, `max_w_px` | `_size_params_from_mm` | width in px for rect detection | `filter_rects_width` |
| `min_h_px`, `max_h_px` | `_size_params_from_mm` | height in px for rect detection | |
| `min_aspect`, `max_aspect` | `_size_params_from_mm` | aspect ratio for rect detection | `filter_rects_aspect` |

## Coordinate conventions

| Quantity | Convention on this machine |
| --- | --- |
| Machine Z | Up is positive; `Z=0` is the highest nozzle position |
| Work / board / fiducial Z | Negative (e.g. `-10.3`, `-7.4`) |
| Top camera | On the head, looking down (`looking: down`) |
| Bottom camera | Fixed, looking up at nozzle (`looking: up`) |
| Pixel → machine XY | `δx = dx_px · upp_x`; Y sign from `looking` (down: `−dy_px`, up: `+dy_px`) |

## Config sketch

```ini
[pnp_vision]
# optional defaults
# calib_speed: 50
# home_tolerance: 0.05
# home_max_iter: 8

[camera_v4l topcam]
device: /dev/video0
port: 8081
resolution: 1600x1200
light: neopixel topled
# light_color: FFF          # hex RGB or RGBW, no '#" (comment char)
# light_s: 1.0              # remembered brightness when LIGHT= is only a color
# light_idle: 0             # level after hold timeout / when another cam acquires
# light_settle: 0.1         # seconds after a level change before snapshot
# light_hold: 5.0           # seconds after last detect before idle
# ... V4L2 controls from CAM_GET ...
camera_matrix: [[...], [...], [...]]
dist_coeffs: [...]
virtual_camera_matrix: [[...], [...], [...]]
rectification_matrix: [[...], [...], [...]]
# two Z samples for linear scale vs height (positive mm/px)
mm_per_px: ((0.0221, 0.0221, -10.3),
            (0.0216, 0.0216, -7.4))
def_z: -10.3

[pnp]
fiducial_primary: {'x': 215.75, 'y': 387.185, 'z': -10.3, 'rot': 0.0, 'dia': 2.0}
fiducial_secondary: {'x': 235.56, 'y': 387.43, 'z': -7.4, 'rot': 0.0, 'dia': 2.0}
# nozzle offsets from top camera, etc.
```

Paste `mm_per_px` lines printed by `PNP_CALIB_PXMM` yourself — there
is no `SAVE` command by design.

Stream URL: `http://<host>:<port>/stream` (MJPEG multipart).

---

## camera_v4l G-code

Mux commands take `CAM=<section_suffix>` (e.g. `CAM=topcam`).

### CAM_SNAP

`CAM_SNAP CAM=<name> [TAG=<string>]`

Grab one processed frame (undistorted path) and write
`/tmp/pnp_<cam>_<tag>_….jpeg`.

### CAM_GET / CAM_SET

`CAM_GET CAM=<name>` — list V4L2 controls (copy into config).  
`CAM_SET CAM=<name> <ctrl>=<value> …` — set controls via `v4l2-ctl`.

### CAM_HUD

`CAM_HUD CAM=<name> [OVERLAY=0|1] [HUD=0|1] [FPS=0|1]`

Toggle overlay, crosshair HUD, and FPS text on the stream.

### CAM_LIGHT

`CAM_LIGHT CAM=<name> [LIGHT=<0..1 and/or hex>]`

Same `LIGHT=` parser as pipelines: a float is brightness, a 3/4/6/8-digit
hex string is RGB or RGBW (no bare `#` — that is a G-code comment)
if string start with K rest is white temerature (K6500).
Both may be combined: `LIGHT=0.8`, `LIGHT=F80`, `LIGHT=0.8,FFF`.

Turns this camera’s light on and the other cameras’ lights to their
`light_idle`. Manual `CAM_LIGHT` does not auto-idle. Omit `LIGHT=` to
print current s / output / color / detected LED channels.

The camera probes the LED object (`pins` on `[led]`, `color_map` /
`color_order` on neopixel) and folds RGB(W)×s onto whatever channels
exist: white-only PWM gets W=s, RGBW extracts common white onto W,
RGB-only folds W back into RGB.

### CAM_CALIB

`CAM_CALIB CAM=<name> [M=…] [D=…] [R=…] [V=…] [A=<alpha>]`

Load/clear camera matrices (`None` clears). If `V` is omitted and `A` is
set, builds a virtual matrix via `getOptimalNewCameraMatrix` and forces
the principal point to the frame center (full FOV, no ROI crop). Rebuilds
undistort maps.

---

## pnp_vision G-code

Requires `[pnp_vision]` and at least one registered `camera_v4l`.

### PNP_VISION_PIPELINE

```text
PNP_VISION_PIPELINE NAME=fiducial LIGHT=0.8
  PARAMS="symmetry:0.75;r:45."
  STEPS="blurgausian(5)|gray()|otsu()|morph_open(9,5)|find_contours_by_r(30,60)|circular_symmetry(symmetry)|pick_nearest(cx,cy)|color()|draw_circles()"
```

- `NAME` — pipeline id used by DETECT / CALIB / HOME  
- `LIGHT` — optional; applied in `run_detect` *before* the snapshot (not a filter). Same tokens as `CAM_LIGHT`. DETECT/HOME may override (`LIGHT=0.4` merges onto the pipeline spec). Turns peer cameras off.  
- `PARAMS` — `key:value;…` defaults (referenced by name inside `STEPS`)  
- `STEPS` — `|`-separated filters: `name(arg,arg,…)`

Arguments that match a `PARAMS` key (case-insensitive) are substituted
with that value; otherwise numbers are parsed as int/float and other
tokens stay as strings.
Gcode comands that use pipelines can pass arbitrary key=value pairs that are also be used for substitution of `args` in `STEPS`. they have higher preference than `PARAMS` defaults and will overwrite those.
Some commands pull key=value pairs for substitutiion from config file  

Pipeline working state:

| Field | Filled by | Used by |
| --- | --- | --- |
| `img` | all image filters | next steps, overlay at end |
| `detected_contours` | `find_contours*` | symmetry / draw_contours |
| `detected_circles` | circularity / enclosing / radial / blob / dist_peaks | pick / draw / result |
| `detected_rects` | rectilinear* / filters | draw_rects / classify |
| `image_stash` | stash / recall | branching preprocessing |

Typical flow:

```text
preprocess (blur, gray, threshold, morph, mask)
  → find_contours / find_contours_by_r
      → circular_symmetry / enclosing_circle / radial_symmetry
    OR blob_circles / dist_peaks   (no contour path)
  → filter_circles_r / filter_circles_score
  → make_rects + filter_rects_*
  → sort_proximity / pick_nearest
  → color() + draw_*   (for overlay)
```

#### Pipeline filter reference

Names are as used in `STEPS` (without the `_filter_` prefix).

##### Color / blur / threshold

| Filter | Args | Description |
| --- | --- | --- |
| `blurmedian(size)` | odd kernel size | Median blur |
| `blurgaussian(size)` | odd kernel size | Gaussian blur |
| `gray()` | — | BGR → gray |
| `color()` | — | Gray → BGR (needed before color draws if pipeline went gray) |
| `invert()` | — | Bitwise NOT |
| `threshold(t[, max[, method]])` | default t=128, max=255 | Fixed threshold |
| `otsu([t[, max[, method]]])` | t ignored by Otsu, default 0 | Otsu threshold |
| `threshmean(block, c[, max])` | block odd, c offset | Adaptive mean |
| `threshgaus(block, c[, max])` | block odd, c offset | Adaptive Gaussian |
| `canny([t1[, t2[, pertureSize[, L2gradient]]]])` | default 50, 150, 3, 0 | Canny edges |

##### Morphology

| Filter | Args | Description |
| --- | --- | --- |
| `morph_open(k[, iters])` | ellipse kernel, default k=5, iters=1 | Open (remove speckles) |
| `morph_close(k[, iters])` | same | Close (fill holes) |
| `morph_tophat(k[, iters])` | same | Top-hat |
| `morph_blackhat(k[, iters])` | same | Black-hat |
| `erode(count)` | iterations | 3×3 erode |
| `dilate(count)` | iterations | 3×3 dilate |

##### ROI / color masks

| Filter | Args | Description |
| --- | --- | --- |
| `mask_hsv(h_min,h_max,s_min,s_max,v_min,v_max[,invert])` | H 0–180, S/V 0–255; invert=0/1 | Keep pixels in HSV range |
| `mask_rect(x,y,w,h[,invert])` | center (x,y), full size w×h | Rectangular ROI mask |
| `mask_centrect(w[,h[,invert]])` | size around optical center; h defaults to w | Rect ROI at principal point |
| `mask_circ(cx,cy,radius[,invert])` | circle ROI | Keep disk at (cx,cy) |
| `mask_centcirc(radius[,invert])` | circle at optical center | Disk at principal point |

##### Contours

| Filter | Args | Description |
| --- | --- | --- |
| `find_contours([min_area[, max_area]])` | default 50 … 999999 | External contours by area → `detected_contours` |
| `find_contours_by_r(min_r[, max_r])` | max_r default 1.2·min_r | Contours by equivalent-radius area band |

##### Circles

| Filter | Args | Description |
| --- | --- | --- |
| `circularity([min_score])` | default ~0.75 | Score Circularity by \(4\pi A/P^2\) gate. Promotes contours → `detected_circles` with `score` |
| `enclosing_circle([min_fill[, min_r[, max_r]]])` | fill default 0.7 | `minEnclosingCircle` + `fill_ratio=A/(πR²)`; center/r from enclosing circle. Robust to ragged edges |
| `radial_symmetry([min_sym[, n_rays[, max_r]]])` | default 0.75, 32 rays | **Optical** radial symmetry (raycast on current gray/binary). Fills `item.symmetry`. Uses circles if set (to refine results), else contours (slower if many) |
| `blob_circles(min_r, max_r[, min_circ[, min_conv[, min_in[, blob_color]]]])` | color 255=bright, 0=dark | `SimpleBlobDetector` → circles (no Hough). Area/circularity/convexity/inertia filters |
| `dist_peaks(min_r, max_r[, min_dist[, threshold_rel[, invert]]])` | min_dist default min_r | Distance-transform local maxima → circle centers. Best for sprocket / regular holes |
| `filter_circles_r(min_r[, max_r])` | — | Keep circles by radius |
| `filter_circles_score(min_score)` | — | Keep by circularity `score` |
| `sort_proximity([cx[, cy]])` | default `auto` = principal point | Sort circles **and** rects nearest-first |
| `pick_nearest([cx, cy[, keep]]])` | keep default 1 | Keep N nearest (optional; HOME wants pipeline to already yield n=1) |

**No Hough** — bad params can freeze large frames for minutes.

##### Rectangles (packages / pads)

| Filter | Args | Description |
| --- | --- | --- |
| `make_rects()` | — | Each contour → `minAreaRect` → `detected_rects` |
| `rectilinear_merged()` | — | All contour points → one `minAreaRect` |
| `filter_rects_aspect(min[, max])` | default 1.4 … 2.1 | Keep rects by long/short side ratio |
| `filter_rects_width(min_px[, max_px])` | default 0 … 9999 | Keep by longer side length (pixels) |
| `classify_rects(model[, threshold])` | e.g. polarity net | Crop/straighten each rect and run inference (**stub / incomplete** — needs model hook) |

Recommended stacks:

- **Fiducial / visual_home (exactly 1):**  
  `find_contours_by_r` → `circular_symmetry` and/or `enclosing_circle` → optional `radial_symmetry` → tight `filter_circles_r`
- **Tape sprockets (many circles):**  
  `otsu` → `dist_peaks(min_r,max_r)` or `blob_circles(...)`
- **Packages:**  
  `find_contours` → `make_rects` → `filter_rects_*`

##### Draw / debug / stash

| Filter | Args | Description |
| --- | --- | --- |
| `draw_circles()` | — | Green circle + red center on each `detected_circles` |
| `draw_rects()` | — | Magenta box, orientation arrow, red cross |
| `draw_contours([thickness])` | default 1 | Contours + blue centroid markers |
| `stash([name])` | default `stash` | Save current `img` to named slot |
| `recall([name])` | default `stash` | Restore `img` from slot |
| `photo([fname])` | strftime path template | Write current `img` to disk |

`photo` default path pattern:
`/tmp/pnp_pipeline_{self.cam.name}_{self.name}_%Y%m%d_%H%M%S.jpeg`

##### Example pipelines

Fiducial / visual_home (strict single circle — filter paprochy, no multi-pick):

```text
STEPS="|blurgausian(5)|gray()|otsu()|morph_open(9,5)|find_contours_by_r(30,60)|circular_symmetry(0.75)|enclosing_circle(0.75,30,60)|radial_symmetry(0.8)|color()|draw_circles()"
```

Sprocket / tape holes (many circles via distance transform):

```text
STEPS="|blurgausian(3)|gray()|otsu()|morph_open(3,1)|dist_peaks(8,30)|color()|draw_circles()"
```

Bright blobs (SimpleBlobDetector):

```text
STEPS="|blurgausian(3)|gray()|otsu()|blob_circles(15,50,0.75,0.8,0.5,255)|color()|draw_circles()"
```

Rectangular part on tray:

```text
STEPS="|blurgausian(3)|gray()|threshold(100,255)|erode(1)|find_contours(100,5000)|make_rects()|filter_rects_aspect(1.3,1.8)|filter_rects_width(35,65)|color()|draw_rects()"
```

HSV tray mask then parts:

```text
STEPS="|mask_hsv(75,96,50,255,150,255)|gray()|invert()|mask_centrect(900)|morph_close(7,2)|threshold(127,255)|find_contours(20,400000)|make_rects()|color()|draw_contours(1)|draw_rects()"
```

### PNP_VISION_LIST

List registered pipelines.

### PNP_VISION_DETECT

```text
PNP_VISION_DETECT CAM=topcam PIPELINE=fiducial [LIGHT=…] [param=value …]
```

Snapshot → run pipeline → overlay. Extra parameters override pipeline
defaults. `LIGHT=` merges onto the pipeline light spec (same parser as
`CAM_LIGHT`) and is applied before the snapshot. Result is also
available to Python via
`vision.run_detect(cam, name, gcmd, params)` (or `params={'light': 0.8}`).

### PNP_CALIB_PXMM / PNP_CALIB_CAMERA

Both are **interactive sessions** (same idea as `BED_SCREWS_ADJUST`):
start the command, jog, then `ACCEPT` / `DONE` / `ABORT`. Session state
lives on `[pnp_calibration]`.

`Z=` is the **plane tag** (fiducial / board height) for a top camera —
toolhead Z stays at camera height. For a bottom camera `ACCEPT Z=` is
the nozzle height and the head is moved there. `ACCEPT FIDUCIAL=primary`
pulls `z` and `dia` from `[pnp]`.

```text
PNP_CALIB_PXMM CAM=topcam PIPELINE=fiducial STEP=8
; optional: FIDUCIAL=primary on start moves to that XY

; jog if needed, then:
ACCEPT FIDUCIAL=primary          ; rosette at that z_plane
; jog to the other mark
ACCEPT FIDUCIAL=secondary
DONE
```

- `STEP` (default 8) / `N=4|8` — rosette jog size after each ACCEPT  
- Each ACCEPT: detect center → ±X/±Y (and diagonals if `N=8`) → average `|mm/px|`  
- Scale comes from **center motion**, not radius (glare makes `r` unstable)  
- Prints `mm_per_px:` to paste (no SAVE); RAM is updated immediately  

```text
PNP_CALIB_CAMERA CAM=topcam PIPELINE=fiducial STEP=4 N=8 RINGS=2
ACCEPT FIDUCIAL=primary     ; dense rosette → mm/px + fiducial XY
ACCEPT FIDUCIAL=secondary
DONE
```

Each ACCEPT runs a **dense rosette** (`RINGS` radii × `N` headings +
center; default 4 mm × 8 × 2 = 17 views). Failed detections are skipped.
Then: pairwise scale, affine residual, and the machine XY of whatever
was on the optical axis:

- **Top** (`looking: down`): fiducial XY (updates `[pnp]` fiducial dict
  in RAM if `FIDUCIAL=` was given). `fx ≈ |Z_tool − z_plane| / mm_per_px`.
- **Bottom** (`looking: up`): `ACCEPT Z=` **moves the nozzle to that Z**,
  then the rosette. Solved camera XY (paste `camera_x` / `camera_y`).
  A second ACCEPT at another nozzle height also fits `camera_z` and `fx`.

### PNP_VISION_HOME

```text
PNP_VISION_HOME FIDUCIAL=primary CAM=topcam PIPELINE=fiducial
```

1. Optional `SET_GCODE_OFFSET X=0 Y=0` (`RESET_OFFSET=1`)  
2. Move to fiducial nominal XY (toolhead Z unchanged)  
3. Detect → `pixels_to_mm_offset` → jog; loop until error ≤ `TOLERANCE`  
4. `SET_GCODE_OFFSET` so G-code XY matches the fiducial coordinates  

| Parameter | Default | |
| --- | --- | --- |
| `FIDUCIAL` | primary | `primary` / `secondary` |
| `TOLERANCE` | 0.05 | mm residual |
| `MAX_ITER` | 8 | |
| `SPEED` | calib_speed | |
| `SETTLE` | 0.2 | s |
| `SET_OFFSET` | 1 | set gcode offset when done |
| `RESET_OFFSET` | 1 | clear XY offset first |

Requires valid `mm_per_px` on the camera.

---

## Typical session

```gcode
SET_KINEMATIC_POSITION X=215.75 Y=387.185 Z=0

PNP_VISION_PIPELINE NAME=fiducial LIGHT=0.8 PARAMS="symmetry:0.75;" \
  STEPS="|blurgausian(5)|gray()|otsu()|morph_open(9,5)|find_contours_by_r(30,60)|circular_symmetry(symmetry)|pick_nearest(cx,cy)|color()|draw_circles()"

PNP_CALIB_PXMM CAM=topcam PIPELINE=fiducial STEP=8
ACCEPT FIDUCIAL=primary
ACCEPT FIDUCIAL=secondary
DONE

; paste printed mm_per_px into printer.cfg

PNP_VISION_HOME FIDUCIAL=primary CAM=topcam PIPELINE=fiducial
```

## API notes for other modules

```python
vision = printer.lookup_object('pnp_vision')
cam = vision.lookup_cam('topcam')
res = vision.run_detect(cam, 'fiducial', gcmd, {'symmetry': 0.75})
# res: success, cx, cy, r, kind, …
dx, dy = cam.pixels_to_mm_offset(res['cx'], res['cy'], z_working_height=fz)
```

Always pass a real `gcmd` (pipeline filters use it for messages/errors).
`pixels_to_mm_offset` returns plain Python `float` (JSON/webhooks safe).

## Related modules

- `klippy/extras/camera_v4l.py` — V4L2 + MJPEG + undistort + scale  
- `klippy/extras/pnp_vision.py` — pipelines, detect, home  
- `klippy/extras/pnp_vision_pipeline.py` — filter implementations  
- `klippy/extras/pnp_calibration.py` — interactive PXMM / camera calib  
- `klippy/extras/pnp.py` — fiducials, nozzle offsets  
- Pressure-based Z (`VALVE_PROBE` etc.) can later measure true fiducial
  heights for the `Z=` tags.
