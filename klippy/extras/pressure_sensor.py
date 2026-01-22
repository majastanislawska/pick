import logging

class PressureSensorGeneric:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
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
        if hasattr(self.sensor, '_temp_callback'):
            self.have_temp = True
            pheaters = self.printer.load_object(config, 'heaters')
            self.min_temp = config.getfloat('min_temp', -273.15, minval=-273.15)
            self.max_temp = config.getfloat('max_temp', 99999999.9, above=self.min_temp)
            self.sensor.setup_minmax(self.min_temp, self.max_temp)
            self.sensor.setup_callback(self.temperature_callback)
            pheaters.register_sensor(config, self.sensor)
            self.last_temp = 0.
            self.measured_min = 99999999.
            self.measured_max = 0.
        else:
            self.have_temp = False
        #self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def pressure_callback(self, read_time, pressure):
        if pressure:
            self.last_pressure = pressure +self.offset
            self.measured_min = min(self.measured_min, self.last_pressure)
            self.measured_max = max(self.measured_max, self.last_pressure)
    def get_pressure(self, eventtime):
        return self.last_pressure, 0.
    def temperature_callback(self, read_time, temp):
        self.last_temp = temp
        if temp:
            self.measured_min = min(self.measured_min, temp)
            self.measured_max = max(self.measured_max, temp)
    def get_temp(self, eventtime):
        return self.last_temp, 0.
    def stats(self, eventtime):
        if self.have_temp:
              s='pressure=%.5f temp=%.2f' % (self.last_pressure,self.last_temp)
        else: s='pressure=%.5f' % (self.last_pressure,)
        return False, '%s: %s' % (self.name, s)
    def get_status(self, eventtime):
        ret={'pressure': round(self.last_pressure, 2),
             'measured_min_pressure': round(self.measured_min, 5),
             'measured_max_pressure': round(self.measured_max, 5)}
        if self.have_temp:
            ret.update({
            'temperature': round(self.last_temp, 2),
            'measured_min_temp': round(self.measured_min, 2),
            'measured_max_temp': round(self.measured_max, 2)})
        return ret

def load_config_prefix(config):
    logging.info("pressure_sensor loadconfig %s" % (config.get_name()))
    return PressureSensorGeneric(config)
