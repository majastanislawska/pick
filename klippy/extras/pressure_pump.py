# Control pressure (vacuum) pump with attached sensor via PWM
#
# Based on heaters.py by Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2025 Maja Stanislawska <maja@makershop.ie>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import output_pin

class PressurePump:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.short_name = self.name.split()[-1]
        self.gcode_id = config.get('gcode_id', None)
        pneu = self.printer.load_object(config, "pneumatics")
        pneu.register_pump(self, config)
        # Setup sensor
        self.sensor = pneu.setup_sensor(config)
        self.offset = config.getfloat('offset', 0.0)
        self.min_pressure = config.getfloat('min_pressure', -100000.0)
        self.max_pressure = config.getfloat('max_pressure', 100000.0, above=self.min_pressure)
        self.sensor.setup_pressure_minmax(self.min_pressure, self.max_pressure)
        self.sensor.setup_pressure_callback(self.pressure_callback)
        pneu.register_sensor(config, self, self.gcode_id)
        self.report_interval = self.sensor.get_report_time_delta()
        self.max_power = config.getfloat('max_power', 1., above=0., maxval=1.)
        # self.kick_start_time = config.getfloat('kick_start_time', 0.1, minval=0.)
        self.off_below = config.getfloat('off_below', default=0., minval=0., maxval=1.)
        self.smooth_time = config.getfloat('smooth_time', 1., above=0.)
        self.inv_smooth_time = 1. / self.smooth_time
        self.last_pressure = 0.
        self.measured_min = 99999999.
        self.measured_max = -99999999.
        self.smoothed_pressure = 0.
        self.target_pressure = 0.
        self.last_pressure_time = 0.
        self.last_pwm_value = 0.
        # Setup control algorithm sub-class
        algos = {'watermark': ControlBangBang} #, 'pid': ControlPID}
        algo = config.getchoice('control', algos)
        self.control = algo(self, config)
        # Setup output pump pin
        pump_pin = config.get('pump_pin')
        ppins = self.printer.lookup_object('pins')
        self.mcu_pwm = ppins.setup_pin('pwm', pump_pin)
        hardware_pwm = config.getboolean('hardware_pwm', False)
        max_duration = self.mcu_pwm.get_mcu().max_nominal_duration()
        pwm_cycle_time = config.getfloat('pwm_cycle_time', self.report_interval, above=0.,
                                         maxval=max_duration)
        self.mcu_pwm.setup_cycle_time(pwm_cycle_time, hardware_pwm)
        self.mcu_pwm.setup_max_duration(2*self.report_interval)
        self.gcrq = output_pin.GCodeRequestQueue(config, self.mcu_pwm.get_mcu(),
                                                 self._apply_pwm)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("SET_PUMP_PRESSURE", "PUMP",
                                   self.short_name, self.cmd_SET_PUMP_PRESSURE,
                                   desc=self.cmd_SET_PUMP_PRESSURE_help)
        self.printer.register_event_handler("gcode:request_restart",self._kill)
        self.printer.register_event_handler("klippy:shutdown", self._kill)

    def _kill(self, print_time=None):
        self.set_pwm(print_time, 0.)
    def _apply_pwm(self, print_time, value):
        if value < self.off_below:
            value = 0.
        value = max(0., min(self.max_power, value * self.max_power))
        self.last_pwm_value = value
        self.mcu_pwm.set_pwm(print_time, value)
    def set_pwm(self,read_time, value):
        if self.mcu_pwm.get_mcu().is_shutdown(): return
        self.gcrq.queue_gcode_request(value)
    def pressure_callback(self, read_time, pressure):
        if pressure:
            pressure +=self.offset
            time_diff = read_time - self.last_pressure_time
            self.last_pressure_time = read_time
            self.last_pressure = pressure
            self.measured_min = min(self.measured_min, self.last_pressure)
            self.measured_max = max(self.measured_max, self.last_pressure)
            self.control.update(read_time, pressure, self.target_pressure)
            pressure_diff = pressure - self.smoothed_pressure
            adj_time = min(time_diff * self.inv_smooth_time, 1.)
            self.smoothed_pressure += pressure_diff * adj_time

    def get_name(self):
        return self.name
    # def get_pwm_delay(self):
    #     return self.report_interval
    def get_max_power(self):
        return self.max_power
    def get_smooth_time(self):
        return self.smooth_time
    def set_pressure(self, pressure):
        if pressure and (pressure < self.min_pressure or pressure > self.max_pressure):
            raise self.printer.command_error(
                "Requested pressure (%.1f) out of range (%.1f:%.1f)"
                % (pressure, self.min_pressure, self.max_pressure))
        self.target_pressure = pressure
    def get_pressure(self, eventtime):
        return self.smoothed_pressure, self.target_pressure
    def check_busy(self, eventtime):
        return self.control.check_busy(
            eventtime, self.smoothed_pressure, self.target_pressure)
    def set_control(self, control):
        old_control = self.control
        self.control = control
        self.target_pressure = 0.
        return old_control
    def stats(self, eventtime):
        is_active = self.target_pressure != 0.
        return is_active, '%s: target=%.0f pressure=%.3f pwm=%.3f' % (
            self.short_name,  self.target_pressure, self.last_pressure,
            self.last_pwm_value)
    def get_status(self, eventtime):
        return {'pressure': round(self.smoothed_pressure, 2),
                'target': self.target_pressure,
                'power': self.last_pwm_value}
    cmd_SET_PUMP_PRESSURE_help = "Sets pump pressure"
    def cmd_SET_PUMP_PRESSURE(self, gcmd):
        pressure = gcmd.get_float('TARGET', 0.)
        # wait = gcmd.get_bool('WAIT', False)
        pneu = self.printer.lookup_object("pneumatics")
        pneu.set_pressure(self, pressure) #, wait)

class ControlBangBang:
    def __init__(self, pump, config):
        self.pump = pump
        self.max_delta = config.getfloat('max_delta', 2.0, above=0.)
        self.on = False
    def update(self, read_time, pressure, target_pressure):
        # logging.debug("bangbang: %f@%.3f -> target=%.3f heating=%s", pressure, read_time, target_pressure, self.on)
        if not self.on and pressure >= target_pressure+self.max_delta: self.on = True
        elif   self.on and pressure <= target_pressure-self.max_delta: self.on = False
        self.pump.set_pwm(read_time, 1. if self.on else 0.)
    def check_busy(self, eventtime, smoothed_pressure, target_pressure):
        return smoothed_pressure < target_pressure-self.max_delta

def load_config_prefix(config):
    logging.info("pressure_pump loadconfig %s" % (config.get_name()))
    return PressurePump(config)
