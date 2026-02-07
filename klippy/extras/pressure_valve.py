# Control pressure (vacuum) valve with optional sensor via PWM
#
# Based on fan.py  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2026 Maja Stanislawska <maja@makershop.ie>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import output_pin

class Valve:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.short_name = self.name.split()[-1]
        self.gcode_id = config.get('gcode_id', None)
        pneu = self.printer.load_object(config, "pneumatics")
        pneu.register_valve(self, config)
        # Setup sensor
        self.sensor = pneu.setup_sensor(config)
        self.offset = config.getfloat('offset', 0.0)
        self.min_pressure = config.getfloat('min_pressure', -100000.0)
        self.max_pressure = config.getfloat('max_pressure', 100000.0, above=self.min_pressure)
        self.sensor.setup_pressure_minmax(self.min_pressure, self.max_pressure)
        self.sensor.setup_pressure_callback(self.pressure_callback)
        pneu.register_sensor(config, self, self.gcode_id)
        self.report_interval = self.sensor.get_report_time_delta()
        self.smooth_time = config.getfloat('smooth_time', 1., above=0.)
        self.inv_smooth_time = 1. / self.smooth_time
        self.last_pressure_time = 0.
        self.last_pressure = 0.
        self.measured_min = 99999999.
        self.measured_max = -99999999.
        self.smoothed_pressure = 0.
        self.last_pwm_value = self.last_req_value = 0.
        self.max_power = config.getfloat('max_power', 1., above=0., maxval=1.)
        self.hold_value = config.getfloat('hold_value', 1., above=0., maxval=1.)
        self.kick_start_time = config.getfloat('kick_start_time', 0.1,minval=0.)
        self.kick_start=False
        cycle_time = config.getfloat('cycle_time', 0.010, above=0.)
        hardware_pwm = config.getboolean('hardware_pwm', False)
        ppins = self.printer.lookup_object('pins')
        self.mcu_pwm = ppins.setup_pin('pwm', config.get('pin'))
        self.mcu_pwm.setup_max_duration(0.)
        self.mcu_pwm.setup_cycle_time(cycle_time, hardware_pwm)
        self.mcu_pwm.setup_start_value(0., 0.)
        self.gcrq = output_pin.GCodeRequestQueue(config, self.mcu_pwm.get_mcu(),
                                                 self._apply_pwm)
        self.printer.register_event_handler("gcode:request_restart",self._kill)
        self.printer.register_event_handler("klippy:shutdown", self._kill)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("VALVE_SET", "VALVE",
                                   self.gcode_id, self.cmd_VALVE_SET,
                                   desc=self.cmd_VALVE_SET_help)
        gcode.register_mux_command("VALVE_GET", "VALVE",
                                   self.gcode_id, self.cmd_VALVE_GET,
                                   desc=self.cmd_VALVE_GET_help)

    def _kill(self, print_time=None):
        self.set_pwm(0., print_time)
    def get_mcu(self):
        return self.mcu_pwm.get_mcu()
    def _apply_pwm(self, print_time, value):
        if value == 0.:
            self.last_pwm_value = value
            self.mcu_pwm.set_pwm(print_time, value)
            return
        if self.kick_start and self.kick_start_time:
            # Run at max_power for specified kick_start_time
            self.last_pwm_value = self.max_power
            self.mcu_pwm.set_pwm(print_time, self.max_power)
            self.kick_start=False
            return "delay", self.kick_start_time
        value = max(0., min(self.max_power, value * self.max_power))
        self.last_pwm_value = value
        self.mcu_pwm.set_pwm(print_time, value)

    def set_pwm(self, value, print_time=None):
        self.gcrq.send_async_request(value, print_time)
    def open(self):
        self.kick_start=True
        self.gcrq.queue_gcode_request(self.hold_value)
    def close(self):
        self.gcrq.queue_gcode_request(0.)

    def pressure_callback(self, read_time, pressure):
        if pressure:
            pressure +=self.offset
            time_diff = read_time - self.last_pressure_time
            self.last_pressure_time = read_time
            self.last_pressure = pressure
            self.measured_min = min(self.measured_min, self.last_pressure)
            self.measured_max = max(self.measured_max, self.last_pressure)
            pressure_diff = pressure - self.smoothed_pressure
            adj_time = min(time_diff * self.inv_smooth_time, 1.)
            self.smoothed_pressure += pressure_diff * adj_time
    def stats(self, eventtime):
        is_active = self.last_pwm_value != 0.
        return is_active, '%s: on=%s pressure=%.3f pwm=%.3f' % (
            self.short_name,  is_active, self.last_pressure,
            self.last_pwm_value)
    def get_pressure(self, eventtime):
        return self.smoothed_pressure, 0. #, self.target_pressure
    def get_status(self, eventtime):
        return {'on': self.last_pwm_value != 0.,
                'pressure': round(self.smoothed_pressure, 2),
                'power': self.last_pwm_value,
        }
    cmd_VALVE_SET_help= "open or close valve. VALVE_SET VALVE=<name> VALUE=[0|1|ON|OFF|OPEN|CLOSE]"
    def cmd_VALVE_SET(self, gcmd):
        value = gcmd.get('VALUE', None)
        if value is None:
            raise gcmd.error("VALVE_SET requires VALUE")
        open= value.upper() in ['1', 'ON', 'OPEN']
        close= value.upper() in ['0', 'OFF', 'CLOSE']
        if open:    self.open()
        elif close: self.close()
        else: raise gcmd.error("VALVE_SET VALUE parameter must be 0/1 or ON/OFF or OPEN/CLOSE")
    cmd_VALVE_GET_help= "get valve status. VALVE_GET VALVE=<name>"
    def cmd_VALVE_GET(self, gcmd):
        status= "CLOSED" if self.last_pwm_value == 0. else "OPEN"
        gcmd.respond_raw("%s:%.2f %s" % (self.gcode_id,
            round(self.smoothed_pressure, 2), status))

def load_config_prefix(config):
    return Valve(config)
