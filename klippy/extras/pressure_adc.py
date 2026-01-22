# Obtain pressure using linear interpolation of ADC values
#
# Based on adc_temperature.py by Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2025 Maja Stanislawska <maja@makershop.ie>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, bisect

SAMPLE_TIME = 0.001
SAMPLE_COUNT = 8
REPORT_TIME = 1
RANGE_CHECK_COUNT = 4

class ADCtoPressure:
    def __init__(self, config, adc_convert):
        self.adc_convert = adc_convert
        self.name = " ".join(config.get_name().split()[1:])
        self.sensor_pin=config.get('sensor_pin')
        ppins = config.get_printer().lookup_object('pins')
        self.mcu_adc = ppins.setup_pin('adc', self.sensor_pin)
        self.gcode_id = config.get('gcode_id', None)
        self.mcu_adc.setup_adc_callback(REPORT_TIME, self.adc_callback)
        self.diag_helper = HelperPressureDiagnostics(
            config, self.mcu_adc, adc_convert.calc_pressure)
    def setup_pressure_callback(self, pressure_callback):
        self.pressure_callback = pressure_callback
    def get_report_time_delta(self):
        return REPORT_TIME
    def adc_callback(self, read_time, read_value):
        val = self.adc_convert.calc_pressure(read_value)
        # logging.info("adc_pressureadc_callback %s %s %s" % (self.name, read_value,val))
        self.pressure_callback(read_time + SAMPLE_COUNT * SAMPLE_TIME, val)
    def setup_pressure_minmax(self, min_pressure, max_pressure):
        arange = [self.adc_convert.calc_adc(t) for t in [min_pressure, max_pressure]]
        min_adc, max_adc = sorted(arange)
        self.mcu_adc.setup_adc_sample(SAMPLE_TIME, SAMPLE_COUNT,
                                      minval=min_adc, maxval=max_adc,
                                      range_check_count=RANGE_CHECK_COUNT)
        self.diag_helper.setup_diag_minmax(min_pressure, max_pressure, min_adc, max_adc)

# Tool to register with query_adc and report extra info on ADC range errors
class HelperPressureDiagnostics:
    def __init__(self, config, mcu_adc, calc_pressure_cb):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.mcu_adc = mcu_adc
        self.calc_pressure_cb = calc_pressure_cb
        self.min_pressure = self.max_pressure = self.min_adc = self.max_adc = None
        query_adc = self.printer.load_object(config, 'query_adc')
        query_adc.register_adc(self.name, self.mcu_adc)
        error_mcu = self.printer.load_object(config, 'error_mcu')
        error_mcu.add_clarify("ADC out of range", self._clarify_adc_range)
    def setup_diag_minmax(self, min_pressure, max_pressure, min_adc, max_adc):
        self.min_pressure, self.max_pressure = min_pressure, max_pressure
        self.min_adc, self.max_adc = min_adc, max_adc
    def _clarify_adc_range(self, msg, details):
        if self.min_pressure is None:
            return None
        last_value, last_read_time = self.mcu_adc.get_last_value()
        if not last_read_time:
            return None
        if last_value >= self.min_adc and last_value <= self.max_adc:
            return None
        tempstr = "?"
        try:
            last_pressure = self.calc_pressure_cb(last_value)
            tempstr = "%.3f" % (last_pressure,)
        except:
            logging.exception("Error in calc_pressure callback")
        return ("Sensor '%s' pressure %s not in range %.3f:%.3f"
                % (self.name, tempstr, self.min_pressure, self.max_pressure))


######################################################################
# Linear interpolation
######################################################################

# Helper code to perform linear interpolation
class LinearInterpolate:
    def __init__(self, samples):
        self.keys = []
        self.slopes = []
        last_key = last_value = None
        for key, value in sorted(samples):
            if last_key is None:
                last_key = key
                last_value = value
                continue
            if key <= last_key:
                raise ValueError("duplicate value")
            gain = (value - last_value) / (key - last_key)
            offset = last_value - last_key * gain
            if self.slopes and self.slopes[-1] == (gain, offset):
                continue
            last_value = value
            last_key = key
            self.keys.append(key)
            self.slopes.append((gain, offset))
        if not self.keys:
            raise ValueError("need at least two samples")
        self.keys.append(9999999999999.)
        self.slopes.append(self.slopes[-1])
    def interpolate(self, key):
        pos = bisect.bisect(self.keys, key)
        gain, offset = self.slopes[pos]
        return key * gain + offset
    def reverse_interpolate(self, value):
        values = [key * gain + offset for key, (gain, offset) in zip(
            self.keys, self.slopes)]
        if values[0] < values[-2]:
            valid = [i for i in range(len(values)) if values[i] >= value]
        else:
            valid = [i for i in range(len(values)) if values[i] <= value]
        gain, offset = self.slopes[min(valid + [len(values) - 1])]
        return (value - offset) / gain


######################################################################
# Linear voltage to pressure converter
######################################################################

# Linear style conversion chips calibrated from pressure measurements
class LinearVoltage:
    def __init__(self, config, params):
        adc_voltage = config.getfloat('adc_voltage', 5., above=0.)
        voltage_offset = config.getfloat('voltage_offset', 0.0)
        samples = []
        for pressure, volt in params:
            adc = (volt - voltage_offset) / adc_voltage
            logging.info("adc sample %.3f %.3f/%.3f pressure sensor %s",
                                adc,pressure, volt, config.get_name())
            if adc < -5. or adc > 5.:
                logging.warning("Ignoring adc sample %.3f/%.3f in sensor %s",
                                pressure, volt, config.get_name())
                continue
            samples.append((adc, pressure))
        try:
            li = LinearInterpolate(samples)
        except ValueError as e:
            raise config.error("adc_pressure %s in sensor %s" % (
                str(e), config.get_name()))
        self.calc_pressure = li.interpolate
        self.calc_adc = li.reverse_interpolate

# Custom defined sensors from the config file
class CustomLinear:
    def __init__(self, config):
        self.name = " ".join(config.get_name().split()[1:])
        self.params = []
        for i in range(1, config.getint('calibration_points', 2) + 1):
            t = config.getfloat("pressure%d" % (i,), None)
            if t is None:
                break
            v = config.getfloat("voltage%d" % (i,))
            self.params.append((t, v))
    def create(self, config):
        lv = LinearVoltage(config, self.params)
        return ADCtoPressure(config, lv)

# DefaultVoltageSensors = [
#     # ("XGZP6859A100KPGPN", [
#     #     (-260, -0.785), (-240, -0.773)
#     # ]),
#     #  ("AD8494", AD8494), ("AD8495", AD8495),
#     # ("AD8496", AD8496), ("AD8497", AD8497),
#     # ("PT100 INA826", calc_ina826_pt100())
# ]

def load_config(config):
    # Register default sensors
    pneu = config.get_printer().load_object(config, "pneumatics")
    # for sensor_type, params in DefaultVoltageSensors:
    #     func = (lambda config, params=params:
    #             ADCtoPressure(config, LinearVoltage(config, params)))
    #     pneu.add_sensor_factory(sensor_type, func)

def load_config_prefix(config):
    custom_sensor = CustomLinear(config)
    vac = config.get_printer().load_object(config, "pneumatics")
    # logging.info("adc_pressure custom load_config_prefix '%s'" % (vac))
    vac.add_sensor_factory(custom_sensor.name, custom_sensor.create)
