import os
import logging

class Pneumatics:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.sensor_factories = {}
        self.gcode_id_to_sensor = {}
        self.available_sensors = []
        self.has_started = self.have_load_sensors = False
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("VAC", self.cmd_VAC, when_not_ready=True)
    def load_config(self, config):
        logging.info("pneumatics.load_config")
        self.have_load_sensors = True
        #Load default sensors
        pconfig = self.printer.lookup_object('configfile')
        dir_name = os.path.dirname(__file__)
        filename = os.path.join(dir_name, 'pressure_sensors.cfg')
        try:
            dconfig = pconfig.read_config(filename)
        except Exception:
            logging.exception("Unable to load pressure_sensors.cfg")
            raise config.error("Cannot load config '%s'" % (filename,))
        for c in dconfig.get_prefix_sections(''):
            logging.info("pneumatics.load_config loading '%s'" % (c.get_name()))
            self.printer.load_object(dconfig, c.get_name())
    def add_sensor_factory(self, sensor_type, sensor_factory):
        logging.info("pneumatics.add_sesor_factory %s %s" % (
            sensor_type,sensor_factory))
        self.sensor_factories[sensor_type] = sensor_factory
    def setup_sensor(self, config):
        if not self.have_load_sensors:
            self.load_config(config)
        sensor_type = config.get('sensor_type')
        logging.info("setup_sensor %s %s %s" % (sensor_type,self,config))
        if sensor_type not in self.sensor_factories:
            raise self.printer.config_error(
                "Unknown pressure sensor '%s'" % (sensor_type,))
        return self.sensor_factories[sensor_type](config)
    def register_sensor(self, config, psensor, gcode_id=None):
        logging.info("register_sensor %s %s %s %s" % (
            self,config,psensor,gcode_id))
        self.available_sensors.append(config.get_name())
        if gcode_id is None:
            gcode_id = config.get('gcode_id', None)
            if gcode_id is None:
                return
        if gcode_id in self.gcode_id_to_sensor:
            raise self.printer.config_error(
                "G-Code sensor id %s already registered" % (gcode_id,))
        self.gcode_id_to_sensor[gcode_id] = psensor
    def get_status(self, eventtime):
        return {'available_sensors': self.available_sensors}
    def _handle_ready(self):
        self.has_started = True
    def _get_pressure(self, eventtime):
        # <gcode_id>:<val> .....
        out = []
        if self.has_started:
            for gcode_id, sensor in sorted(self.gcode_id_to_sensor.items()):
                cur, target = sensor.get_pressure(eventtime)
                out.append("%s:%.2f/%.2f" % (gcode_id, cur, target))
        if not out:
            return "None:0"
        return " ".join(out)
    def cmd_VAC(self, gcmd):
        reactor = self.printer.get_reactor()
        msg = self._get_pressure(reactor.monotonic())
        # did_ack = gcmd.ack(msg)
        # if not did_ack:
        gcmd.respond_raw(msg)

def load_config(config):
    logging.info("pneumatics loadconfig %s" % (config.get_name()))
    return Pneumatics(config)
