# Orthogonal Tool-Space `move_transform` for Pick (𝝅𝑐𝑘)

This document describes `move_transform` layer added on top of
Klipper to accommodate for Pick and Place machines specific needs.
Primarily support for Peter's Head and compensation for machine offsets, misalignments and runouts.

Framework creates and maintains consistent Cartesian `XYZR` "gcode space" for tip of currently selected tool with currently loaded tooltip (or camera principal point) and adjusts machine coordinates to compensate for all known offsets and misalignments
(tool offsets, Z-axis tilt, camera optical-axis tilt, and tooltip runout).
It also hides all "technical details" like which manual_stepper rotates which tool - you always operate with `G1R<angle>`, while remembering rotation for time given tool is unselected. Tool Z axis position is retracted to "safe position" of 0 on toolchange though.

When selected tool is camera it swallows all moves on `Z` and `R` (machine doesnt move, besides XY moves to compensate misalignments) and uses them for own purposes: `Z` is used as reference plane for px to mm calculations, and `R` is displayed on HUD. However those coordinates are still returned in `get_position()` so during switchinng to different tool, that tool can land exactly where camera was looking (`PNP_TOOL ... MOVE=1`).

See also [G-Codes](G-Codes.md#pnp) for a command reference
and [Config Reference](Config_Reference.md#pnp) for configuration options.

## Disclaimers

- Kinematics only calculates final position (commanded pos) of move not whole toolpath - if you have `XYR` move nozzle tip may actually dance around (by runout r) on its way to destination.
There is no really need for toolpath as Pick and Place machines operate on point-to-point basis, and it can be commanded from 'client side' by move segmentation if needed.
- Arc commands (`G2`, `G3`, `G17`, `G18`, and `G19`) are out of scope of interest for this specification for same reason as above.
- use of `R` as tool rotation is a theoretically collision with ArcMoves (where R is supposed to be arc radius) but Klipper doesn't support ArcMoves with `R` anyway and throws error.
- `R` can be replaced with some other letter by setting `r_gcode_letter` in `[pnp]` config section
- Similarly to OpenPNP `Z=0` is highest plane in machine space - a "safe height" - where tools are when retracted, and board and feeders sit somewhere at bottom of working space in far ends of negative range of `Z`
- Therefore **Nozzle Z is actuator stroke from park**, not Cartesian tip height above the board. `G1 Z-10` is 10 mm of extension of the **active** tool, regardless of tip length. Thankfully tips are typically same lengths and any imbalance between tools on Perter's Head can by accommodated by moving endstop position. See [Calibration](PnP_Calibration.md).
- There are `z_offset` config options on `[pnp_tool]` and `[pnp_tooltip]` but they are not currently used.
- `G0` and `G1` still do same thing (as klipper does), but eventually `G0` will be "safe *travel* move" (with retract to z=0 before any XY move and then z dwell do destination), while `G1` will remain "freestyle" *work* move i.ex. for sneaking up towards toolchanger.
- some caution is required when using `PNP_TOOL SET=` with `MOVE=1` and when swapping tools back and forth with and without `MOVE=1`. `MOVE=1` moves toolhead to last gcode pos (which will be diffent machine pos due to new offsets/transforms), while without updates gcode posistion to what new offsets and transforms compute from last machine pos.

## Overview

In config file you can create arbitrary number of tools, define steppers that make them move and offsets (there will be commands for calibrating offsets):

```ini
[pnp_tool left]
z_axis: +Z
r_axis: stepper_a
offset_x: -9.3839
offset_y: -23.5784
axis_vector: (0., 0., 1.0)
```

those tools along with down-looking camera can be selected using `PNP_TOOL SET=[name]` to be acted upon by other gcode commands (like `G0` or `G1`)

`z_axis` parameter holds name of stepper motor responsible for moving this tool along z axis, along with sign that indicates direction. On Peter's Head toolheads one tool will be `+Z` other `-Z`. First toolhead (pair of tools) should use 'standard' `stepper_z` config section, others should use `stepper_z[n]` (`stepper_z1`, `stepper_z2`) (Internally this uses klipper's MultiRail), Ofc this also works on heads that have own independent steppers.
you can use either `Z` or `stepper_z` here.

Analogously `r_axis` holds name of stepper motor responsible for rotating that tool.
this will be name of some `manual_stepper` config section.
both long form (`manual_stepper stepper_a`) and short (`stepper_a`) work.

Tools can have arbitrary number of tooltips created for them:

```ini
[pnp_tooltip CN020_left]
name: CN020
tool: left
#[..]
[pnp_tooltip CN020_right]
name: CN020
tool: right
#[..]
```

`name` is to be referenced in gcode command `PNP_TIP LOAD=[name]` can be same for all tips of same type to ease use from operator's perspective, while separate sections can hold specific data like toolchanger position or sequence.
tooltip holds data about its own runout that is 'last stage' of transform, however it's not intended to be stored permanenetly in config only calibrated on (first) load.

## Kinematic chain

G-code is a **Cartesian world / machine frame**: XY about 0…500 mm, nozzle Z typically −15…−5 mm of work with **Z=0 = safe / park**. The host maps that frame through machine gantry XY (plus the visual-home offset) onto the **active tool**:

- a **down-looking camera**, whose optical axis may be slightly tilted (`axis_vector` / rotation matrix from calibration), or
- a **nozzle** that travels “vertically” on the seesaw but may also be slightly tilted - also `axis_vector`, and whose rotating tip may have runout. No nozzle or nozzletip specific `z_offset`s at this point.

The transform absorbs those errors so that:

- changing **R** keeps the **tooltip centre fixed in world XY** (gantry walks the runout circle);
- changing **Z** moves the tip **along world-vertical** (gantry walks tilt);
- **`PNP_TOOL MOVE=1`** puts the newly selected tool on the **same world point** — for a camera, so that its optical axis **intersects** that point.
