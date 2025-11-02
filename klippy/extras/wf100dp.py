# klippy/extras/wf100dp.py
# WF100DPZ Digital Pressure Sensor support for Klipper (I2C)
#
# Copyright (C) 2025 Maja Stanislawska <maja@makershop.ie>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from . import bus
import logging

class WF100DPSensor:
    def __init__(self, config, params):
        self._printer = config.get_printer()
        self._reactor = self._printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self._i2c_addr = 0x6D  # Fixed address
        self._i2c = bus.MCU_I2C_from_config(config, self._i2c_addr)
        self._mcu = self._i2c.get_mcu()
        self.pm=params['pm'] #pressure_multiplier
        self.po=params['po'] #pressure_offset
        self.to=params['to'] #temp_offset
        self.td=params['td'] #temp_divider
        self._sleep_time = config.getchoice('sleep_time',
            {f"{i*0.0625:.4f}s": i for i in range(16)}, '0.1250s')
        self._last_value = 0  # Pressure
        self._last_temp = 0   # Temperature
        self._pressure_callback = None
        self._temp_callback = None
        self._i2c_addr = 0x6D  # Fixed address
        self._i2c = bus.MCU_I2C_from_config(config, self._i2c_addr)
        self._mcu = self._i2c.get_mcu()
        # pheaters = self.printer.load_object(config, 'heaters')
        # pheaters.register_sensor(config, self)
        self.sample_timer = self._reactor.register_timer(self._sample)
        self._printer.register_event_handler("klippy:connect",
                                             self._handle_connect)

    def _handle_connect(self):
        self._i2c.i2c_write([0x1C])  # Reset
        self._reactor.pause(self._reactor.monotonic() + 0.01)
        mode = 0x0B | (self._sleep_time << 4)  # Combined pressure + temp
        self._i2c.i2c_write([0x30, mode])
        # Set sample interval based on sleep_time (in seconds)
        self._sample_interval = (
            self._sleep_time * 0.0625 if self._sleep_time
                                      else 10 ) # Default 10s for 0s
        self._reactor.update_timer(self.sample_timer, self._reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp
    def setup_presssure_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp
    def _sensor_read(self):
        return self._i2c.i2c_read([0x06], 5)['response']
        #  d1=self._i2c.i2c_read([0x06], 3)['response']
        #  d2=self._i2c.i2c_read([0x09], 2)['response']
        #  return d1+d2 #[0x0,0x0,0x0,0x0,0x0] #
    def _sample(self, eventtime):
        data=d1=self._i2c.i2c_read([0x06], 5)['response']
        press=float(int.from_bytes(data[:3],'big',signed=True))
        self._last_value=((press * self.pm)/float(1<<23)) + self.po
        temp=float(int.from_bytes(data[3:],'big',signed=True))
        self._last_temp=(temp + self.to) / self.td
        # logging.info("WF100DPZ %s: pres:%s temp:%s"%
        #         (self.name, self._last_value,self._last_temp))
        if self._pressure_callback:
            #logging.info("WF100DPZ %s:calling pressure callback"% (self.name))
            self._pressure_callback(eventtime, self._last_value)
        if self._temp_callback:
            #logging.info("WF100DPZ %s:calling temp callback"% (self.name))
            self._temp_callback(eventtime, self._last_temp)
        return eventtime + self._sample_interval
    def setup_callback(self, cb):
        self._temp_callback = cb
    def setup_pressure_callback(self, cb):
        self._pressure_callback = cb
    def get_status(self, eventtime):
        return {'pressure': self._last_value, 'temperature': self._last_temp}

class WF100dpFactory:
    def __init__(self, config):
        self.name = " ".join(config.get_name().split()[1:])
        self.params={}
        self.params['pm'] = config.getfloat("wf100dp_pressure_multiplier")
        self.params['po'] = config.getfloat("wf100dp_pressure_offset")
        self.params['to'] = config.getfloat("wf100dp_temp_offset")
        self.params['td'] = config.getfloat("wf100dp_temp_divider")
        logging.info("WF100dpFactory %s"%(self.name))
    def create(self, config):
        logging.info("WF100dpFactory creating sensor %s %s"%(self.name,config))
        return WF100DPSensor(config, self.params)

def load_config_prefix(config):
    sensor= WF100dpFactory(config)
    name = config.get_name().split()[-1]
    pneu = config.get_printer().load_object(config, "pneumatics")
    logging.info("wf100dp factory load_config_prefix '%s'" % (pneu))
    pneu.add_sensor_factory(name, sensor.create)
