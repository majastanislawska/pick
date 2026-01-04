import logging

class PressureSensorGeneric:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        #self.sensor_type = config.get('sensor_type')
        #self.gcode_id = config.get('gcode_id', None)
        pneu = self.printer.load_object(config, "pneumatics")
        self.sensor = pneu.setup_sensor(config)
        self.offset = config.getfloat('offset', 0.0)
        self.min_pressure = config.getfloat('min_pressure', -100000.0)
        self.max_pressure = config.getfloat('max_pressure', 100000.0,
                                            above=self.min_pressure)
        self.report_interval = config.getfloat('report_interval', 1.0,
                                                    minval=0.1)
        self.sensor.setup_minmax(self.min_pressure, self.max_pressure)
        self.sensor.setup_pressure_callback(self.pressure_callback)
        pneu.register_sensor(config, self)#, self.gcode_id)
        self.last_pressure = 0.
        self.measured_min = 99999999.
        self.measured_max = -99999999.
        #self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def pressure_callback(self, read_time, pressure):
        if pressure:
            self.last_pressure = pressure +self.offset
            self.measured_min = min(self.measured_min, self.last_pressure)
            self.measured_max = max(self.measured_max, self.last_pressure)
    def get_pressure(self, eventtime):
        return self.last_pressure, 0.
    def stats(self, eventtime):
        return False, '%s: pressure=%.5f' % (self.name, self.last_pressure)
    def get_status(self, eventtime):
        return {
            'pressure': round(self.last_pressure, 2),
            'measured_min_pressure': round(self.measured_min, 5),
            'measured_max_pressure': round(self.measured_max, 5)
        }

def load_config_prefix(config):
    logging.info("pressure_sensor loadconfig %s" % (config.get_name()))
    return PressureSensorGeneric(config)
