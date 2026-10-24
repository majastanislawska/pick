Welcome to the Pick (&#x1D745;&#x1D450;&#x1D458;) project!

Pick (&#x1D745;&#x1D450;&#x1D458;) is a spinoff of [Klipper](https://www.klipper3d.org/) that focuses on Pick and Place machines.

That is:

* Geometric transformations to accommodate tool offsets and compensate for machine misalignments, and easy context switching between tools and (top)camera. (`move_transform`)
* camera support and Vision
* configurable vision pipelines
* USB Power Delivery (CH224Q configurable over i2c) for a slim toolhead board
* central endstop
* support for pumps, valves, and pressure sensors, not just by reusing features related to hotend/hotbed temperature
* feeders control (servo for now)

more to come.

[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/majastanislawska/pick)

# Features

## Vision

Pick can now see stuff.

Vision Pipelines can be created and modified using gcode command and executed to snap photo and detect objects.
specialized camera module can take any VL4 camera, apply undistortion, provide frames both to vision pipleines and to MJPEG server where it can be viewed by operator, that supports HUD display and overlays.
see [PnP Vision](docs/PnP_Vision.md) for details.

## Camera Support

`camera_v4l` is main vision engine for Pick, it should work with any camera supported by Linux V4L stack. it provides snapshots for vision engine as well as own MJPEG streaming server that provides undistorted view along with other features like overlay where results of detection pipelines can be displayed.
you can use this camera from your web browser.

`camera_ustreamer` module allows to spawn ustreamer for as many cameras as you need and provide some control of it's controls. Those can be received by OpenPNP's MJPGCaptureCamera.
Intent is to allow Pick to sit on RaspberryPi mounted on machine and operate OpenPNP from desktop. This module is intended for legacy setup with OpenPNP, which does it's own camera calibration.
this is Linux specific module, uses V4L.

## Calibration

You can now calibrate whole machine from scratch, including cameras, using several commands on text console and a text editor to edit your config files

see [PnP Calibration](docs/PNP_Calibration.md) for details.

## Kinematics

Framework that maintains consistent Cartesian `XYZR` "gcode space" for tip of currently selected tool with currently loaded tooltip (or camera principal point) and adjusts machine coordinates to compensate for all known offsets and misalignments (tool offsets, Z-axis tilt, camera optical-axis tilt, and tooltip runout) has been implemented using Klipper's `move_transform`. `PnPManager` (in `pnp.py`) installs itself as that transform and redispatches each move/position query to the currently active tool. Unlike a static bed-tilt/mesh correction, this transform mutates to accommodate for currently selected `active tool` at runtime (`PNP_TOOL SET=[name]`).
it also provides way to switch between tools and camera.

see [PnP_Move_Transform.md](docs/PnP_Move_Transform.md) for details.

## Central Endstop

Central endstop is a feature that allows you to put endstop not at ends of axes but somewhere in (near) the middle.
Homing will cause the stepper to move in a positive direction if endstop is in triggered state in current position and in negative direction if it's open \
it works best with optical (either slot or reflective) sensors.\
original puropse of this feature is a z-axis of pick and place machine in configuration known as 'Peter's Head'.

only implemented for CoreXY kinematics for now, (ask if you need others)

## USB Power Delivery

Intended for a slim toolhead mcu board with only one cable going to it (and vacuum pipe). 20V@3A is well enough to drive motors of 2 rotary axes, z axis, and some auxiliary functions. Nema8 are rated at 0.5A.
currently only CH224Q configurable over i2c controller is supported

Module provides gcode commands to quuery for stattus and to enable extra power when it's actually needed (i.ex. go down to 5V in idle shutdown macro).
See [commands reference](docs/G-Codes.md#ch224q_pd).

There can be many instances of this chip configured. I2C address in not configurable so there can be one per i2c bus, but one per controller board is only thing that makes sense, therefore name config section same as relevant mcu and use `MCU=<name>` param do differentiate between them.

## Pneumatics

Framework very similar to Klipper's `heaters` is being developed, it's called `[pneumatics]`. it's intended to exist parallel to heaters/temperature framework instead of reusing it for pressure so you can still use heaters and temperature sensors for whatever reason.

### Pressure sensors

Following sensors are supported for now:

* `wf100dp` i2c sensor (2 variants: 100kpa and 40kpa, if you know scaling parameters for other in that series let me know).
* Analog `XGZP6859A` series, ranges for several are in `klippy/extras/pressure_sensors.cfg`, they are named with their respective part number (ie. `XGZP6859A100KPGPN33`), other can be added with data from datasheet.
* Differential wheatstone bridge based pressure sensors should work too with ADS1115 in differential mode (`sensor_pin: ads1:DIFF01`) and some config in `klippy/extras/pressure_sensors.cfg`

Support for more sensors is on the way, along support for pump that can maintain pressure and valves that can monitor pressure.

Sensors can be used as 'standalone' `[pressure_sensor my_sensor]` analogous to klipper's `[temperature_sensor my_sensor]`, or attached to pump or valve.

### Pump

Pump object can be set to maintain pressure in a system, similar to what heater does with temperature. This works really nice if you have pump with reservoir (tank) and separate valves for nozzles.
Currently only BanBang algorithm is implemented.
`cycle_time` is hardcoded to `report_interval` of sensor.
Only command for now is `SET_PUMP_PRESSURE PUMP=<name> TARGET=<value>`

### Valve

Valve is rather simple object that (for now) operates like a binary `output_pin` but lets you configure `kick_start_time` during which `max_power` will be supplied (to force valve to open), after which switches to `hold_value` so solenoid doesn't overheat. It can contain pressure sensor config.

There's command to open/close valve: `VALVE_SET VALVE=<gcode_id> VALUE=[0|1|ON|OFF|OPEN|CLOSE]`
and to read its status and value of attached sensor: `VALVE_GET VALVE=<gcode_id>`, response looks like this: `<gcode_id>:<value(%.2f)> OPEN|CLOSED`

## Webhooks

### gcode/session

This endpoint just like `gcode/script` allows one to run a series of G-Code commands.However, unlike like `gcode/script` if the G-Code command produces terminal output, that output is provided in the response.

For example: `{"id": 123, "method": "gcode/session", "params": {"command": "M114"}}` might return `{"id":123,"result":{'output': ['X:0.000 Y:0.000 Z:0.000 E:0.000 A:0.000 B:0.000', 'ok']}}`

No special distinction for error messages is done at response format,
just parse output like it would came from terminal.

Same considerations regarding queuing and delay as for `gcode/script` apply.

### pnp related

Those webhooks are intended for desktop gui app (see wxPick). More to come.

* `pnp/jog` in `pnp` for jogging around (instead of `G91;G1;G90` sequence)
* `pnp/move` in `pnp` for absolute coordinates move (mostly used to move `Z` and `R` to `0`)
* `pnp/tool` in `pnp` to switch `active tool` (equivalent of `PNP_TOOL SET=[name]`)
* `pnp/light` in `pnp_vision` to manually control camera lights
* `pnp/px2mach` in `pnp_vision` to query for translation of pixel coordinates to machinepos delta (for jogging)

## Miscellaneous

* Overall brightness of Neopixel strip is configurable.
* klipper's `API server` can now bind to *tcp socket* too (`-a tcp://host:port` commandline parameter). so GUI app can be on desktop and you don't need stuff like Moonraker, Fluidd or Mainsail.
* Errors in gcode responses are more pronounced and easier to parse:
`Error:` instead of `!!`

## legacy OpenPNP compatibility

If you want to use Pick with OpenPNP don't use `PnP_Move_Transform` feature as it's incompatible with OpenPNP. Instead:

* Configure rotary axes for Pick and Place toolhead using `MANUAL_STEPPER` command.
* Use `camera_ustreamer` module for cameras.
* Use "standalone" pressure sensors. (you may need to use regexp in openpnp to strip minus sign from pressure values.)
* you can use ordinary `[output_pin]` for valves and pump

see [example config](config/openpnp.cfg)
