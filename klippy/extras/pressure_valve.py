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
        self.part_on = False
        self.part_on_min = config.getfloat('part_on_min', -50.0)
        self.part_on_max = config.getfloat('part_on_max', -20.0, above=self.part_on_min)
        self.part_off_min = config.getfloat('part_off_min', -20.0)
        self.part_off_max = config.getfloat('part_off_max', 0, above=self.part_off_min)
        self.fail_pick= config.getboolean('fail_pick', False)
        self.fail_check= config.getboolean('fail_check', False)
        self.timeout= config.getfloat('timeout', 1., above=0.)
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
        gcode.register_mux_command("VALVE_WAIT", "VALVE",
                                   self.gcode_id, self.cmd_VALVE_WAIT,
                                   desc=self.cmd_VALVE_WAIT_help)
        gcode.register_mux_command("VALVE_CONFIG", "VALVE",
                                   self.gcode_id, self.cmd_VALVE_CONFIG,
                                   desc=self.cmd_VALVE_CONFIG_help)
        gcode.register_mux_command("VALVE_PICK", "VALVE",
                                   self.gcode_id, self.cmd_VALVE_PICK,
                                   desc=self.cmd_VALVE_PICK_help)
        gcode.register_mux_command("VALVE_CHECK", "VALVE",
                                   self.gcode_id, self.cmd_VALVE_CHECK,
                                   desc=self.cmd_VALVE_CHECK_help)

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
        return is_active, '%s: on=%s part_on=%s pressure=%.3f pwm=%.3f' % (
            self.short_name,  is_active, self.part_on, self.last_pressure,
            self.last_pwm_value)
    def get_pressure(self, eventtime):
        return self.last_pressure, 0. #, self.target_pressure
    def get_status(self, eventtime):
        return {'on': self.last_pwm_value != 0.,
                'part_on': self.part_on,
                'pressure': round(self.smoothed_pressure, 2),
                'power': self.last_pwm_value,
                'min_pressure': self.min_pressure,
                'max_pressure': self.max_pressure,
                'part_on_min': self.part_on_min,
                'part_on_max': self.part_on_max,
                'part_off_min': self.part_off_min,
                'part_off_max': self.part_off_max,
                'timeout': self.timeout,
                'fail_pick': self.fail_pick,
                'fail_check': self.fail_check
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
        reactor = self.printer.get_reactor()
        wait = gcmd.get_float('WAIT', 0.1)
        if wait>0.:
            reactor.pause(reactor.monotonic() + wait)
        cur, _ = self.get_pressure(reactor.monotonic())
        gcmd.respond_raw(self.get_getstr(cur))
    cmd_VALVE_WAIT_help = "Wait for a valve to reach a specific state (params dont override config)"
    def cmd_VALVE_WAIT(self, gcmd):
        min_pressure = gcmd.get_float('MINIMUM', float('-inf'))
        max_pressure = gcmd.get_float('MAXIMUM', float('inf'), above=min_pressure)
        timeout = gcmd.get_float('TIMEOUT', 1)
        fail= gcmd.get_boolean('FAIL', False)
        if min_pressure == float('-inf') and max_pressure == float('inf'):
            raise gcmd.error(
                "Error on 'VALVE_WAIT': missing MINIMUM or MAXIMUM.")
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        self.open()
        reactor = self.printer.get_reactor()
        starttime= eventtime = reactor.monotonic()
        while not self.printer.is_shutdown():
            cur, _ = self.get_pressure(eventtime)
            if cur >= self.min_pressure and cur <= self.max_pressure:
                gcmd.ack(self.get_getstr(cur))
                return
            time_diff = eventtime - starttime
            if time_diff > timeout: break
            gcmd.respond_raw(self.get_getstr(cur))
            eventtime = reactor.pause(eventtime + 0.1)
        self.close()
        gcmd.respond_raw(self.get_getstr(cur))
        if fail:
            raise gcmd.error("VALVE_WAIT timeout after %.1f seconds" % (time_diff,))

    cmd_VALVE_CONFIG_help = "Configure valve parameters Use on nozzle swaps. VALVE_CONFIG VALVE=<name> [ON_MIN=<pressure>] [ON_MAX=<pressure>] [OFF_MIN=<pressure>] [OFF_MAX=<pressure>] [TIMEOUT=<seconds>] [FAIL_PICK=[0|1]] [FAIL_CHECK=[0|1]]"
    def cmd_VALVE_CONFIG(self, gcmd):
        self.part_on_min = gcmd.get_float('ON_MIN', self.part_on_min)
        self.part_on_max = gcmd.get_float('ON_MAX', self.part_on_max, above=self.part_on_min)
        self.part_off_min = gcmd.get_float('OFF_MIN', self.part_off_min)
        self.part_off_max = gcmd.get_float('OFF_MAX', self.part_off_max, above=self.part_off_min)
        self.timeout = gcmd.get_float('TIMEOUT', self.timeout)
        self.fail_pick= gcmd.get_int('FAIL_PICK', self.fail_pick)
        self.fail_check= gcmd.get_int('FAIL_CHECK', self.fail_check)
        gcmd.ack()
    cmd_VALVE_PICK_help = "pick using current config and set part_on if successful."
    def cmd_VALVE_PICK(self, gcmd):
        self.open()
        reactor = self.printer.get_reactor()
        ret, last, eventtime = self.wait_for_pressure(gcmd, reactor, self.part_on_min, self.part_on_max, self.timeout)
        if ret:
            self.part_on = True
            gcmd.respond_raw(self.get_getstr(last))
            gcmd.ack()
        else:
            self.close()
            self.part_on = False
            eventtime = reactor.pause(eventtime + 0.1)
            gcmd.respond_raw(self.get_getstr(last))
            if self.fail_pick: raise gcmd.error("VALVE_PICK No part detected.")
            else: gcmd.ack()
    cmd_VALVE_CHECK_help = "check if nozzle is clear and unset part_on"
    def cmd_VALVE_CHECK(self, gcmd):
        self.open()
        reactor = self.printer.get_reactor()
        ret,last,eventtime=self.wait_for_pressure(gcmd, reactor, self.part_off_min, self.part_off_max, self.timeout)
        self.close()
        if ret:
            self.part_on = False
            eventtime = reactor.pause(eventtime + 0.1)
            gcmd.respond_raw(self.get_getstr(last))
            gcmd.ack()
        else:
            gcmd.respond_raw(self.get_getstr(last))
            if self.fail_check: raise gcmd.error("VALVE_CHECK Nozzle Not Clear")
            else: gcmd.ack()
    def get_getstr(self, val):
        status= "CLOSED" if self.last_pwm_value == 0. else "OPEN"
        part = "PART_ON" if self.part_on else "PART_OFF"
        return "%s:%.2f %s %s" % (self.gcode_id, val, status, part)
    def wait_for_pressure(self, gcmd, reactor, min, max, timeout):
        starttime= eventtime = reactor.monotonic()
        while not self.printer.is_shutdown():
            cur, _ = self.get_pressure(eventtime)
            gcmd.respond_raw(self.get_getstr(cur))
            if cur >= min and cur <= max:
                return True,cur,eventtime
            time_diff = eventtime - starttime
            if time_diff > timeout: break
            eventtime = reactor.pause(eventtime + 0.1)
        return False, cur, eventtime

def load_config_prefix(config):
    return Valve(config)
