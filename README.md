Welcome to the Pick (&#x1D745;&#x1D450;&#x1D458;) project!

Pick (&#x1D745;&#x1D450;&#x1D458;) is a spinoff of [Klipper](https://www.klipper3d.org/) that focuses on Pick and Place machines.

That is:
 * support for extra axes (working M114)
 * USB Power Delivery (CH224Q configurable over i2c) for a slim toolhead board
 * central endstop
 * support for pumps, valves, and pressuure sensors, not just by reusing feaures related to hotend/hotbed temperature (todo)
 * feeders control (servo for now)

more to come.

# Features

## Kinematics

Rotary axes for Pick and Place toolhead shal be immplemented using `MANUAL_STEPPER' command it's good to put it in a "startup macro" like this
```
[delayed_gcode init_axes]
initial_duration: 0.1
gcode:
    MANUAL_STEPPER STEPPER=stepper_a GCODE_AXIS=A LIMIT_VELOCITY=5000 LIMIT_ACCEL=3000
    MANUAL_STEPPER STEPPER=stepper_b GCODE_AXIS=B LIMIT_VELOCITY=5000 LIMIT_ACCEL=3000
    RESPOND PREFIX='info' MSG='initializing A and B axes: {printer.toolhead.position}'
```
Of course steppers need to be defined in confiruration file, like this:
```
[manual_stepper stepper_a]
step_pin: toolhead:A_STEP
dir_pin: !toolhead:A_DIR
enable_pin: !toolhead:A_EN
#endstop_pin: ^toolhead:A_STOP
microsteps: 16
full_steps_per_rotation: 200
rotation_distance: 360
# position_min:-360
# position_endstop: 0
# position_max: 360
```
(this setup treats degrees like milimeters, speeds and accells need to be adjusted to match motor.)

some effort has been made to make those steppers show up and be acted upon in relevant gcode commands so OpenPNP can see and use them.
Maybe some deeper integration into kinematics will be done in future, but it's good enough for now.

## Central Endstop

Central endstopo is a feature that allows you to put endstop not at ends of axes but somewhere in (near) the middle.
Homing will cause the stepper to move in a positive direction if endstop is in triggered state in current position and in negative direction if it's open \
it works best with optical (either slot or reflective) sensors.\
original puropse of this feature is a z-axis of pick and place machine in configuration known as 'Peter's Head'.

only implemented for CoreXY kinemmatics for now, (ask if you need others)

## USB Power Delivery

Intended for a slim toolhead mcu board with only one cable going to it (and vacuum pipe). 20V@3A is well enough to drive motors of 2 rotary axes, z axis, and some auxilary functions. Nema8 are rated at 0.5A.
currently only CH224Q configurable over i2c controller is supported

Module provides 3 gcode commands:

- `PD_CAPS` Dumps PD Source Capabilities: available power profiles (PDOs)
- `PD_SET VOLTAGE=<value>` Reguets voltage fromm supply (charger)
- `PD_GET` Returns status as a single line of space separated key:value format that can be parsed

This layout allows flexibility to enable extra power when it's actually needed (use macros).\
There can be many instances of this chip configured. I2C address in not configurable so there can be one per i2c bus, but one per controller board is only thing that makes sense, therefore name config section same as relevant mcu and use `MCU=<name>` param do differentiate between them.
```
[ch224q_pd toolhead]
i2c_mcu: toolhead
i2c_bus: i2c1e
```

## Pneumatics

in progress.
