# klippy/extras/pnp_tooltip.py
# holds tooltip (nozzle) config.
#
# Copyright (C) 2026 Maja Stanislawska <maja@makershop.ie>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import math


class PnPToolTip:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.long_name = config.get_name()
        self.name = config.get('name')
        self.toolname=config.get('tool')
        self.tool=None #in connect
        self.in_dia=config.get('in_dia')
        self.out_dia=config.get('out_dia')
        self.z_offset=config.getfloat('z_offset', 0.0)
        #dict with x,y,z where tip is stored
        self.toolchanger_pos=config.get('toolchanger_pos',None)
        #list of positions head needs to go through to perform load or unload (backwards)
        self.toolchanger_seq=config.getasteval('toolchanger_seq', [])
        #dict with params for vacuum part_on/part_off sensing by valve object
        self.vacuum_conf=config.getasteval('vacuum_conf', {})
        self.runout_conf=config.getasteval('runout_conf', {})
        # live, after PNP_VISION_RUNOUT — not a config key
        self.runout_radius = 0.
        self.runout_phase = 0.
        self.runout_data = []

        self.printer.register_event_handler("klippy:connect", self._handle_connect)

    def _handle_connect(self):
        self.pnp = self.printer.lookup_object('pnp')
        self.tool = self.pnp.lookup_tool(self.toolname)
        self.tool.register_tip(self)

    def get_runout_params(self):
        """Return dict with runout params for this tip."""
        conf=self.runout_conf.copy()
        if not 'dia' in conf:
            conf['dia'] = self.in_dia
        return conf

    def get_vacuum_params(self):
        """Return dict with vacuum params for this tip."""
        return self.vacuum_conf.copy()

    def __str__(self):
        return "%s: runout r=%.4f phi=%.4f %s"%(
            self.name, self.runout_radius, self.runout_phase, self.tool.r_units)

    def get_status(self, eventtime):
        return {
            'name': self.name,
            'in_dia': self.in_dia,
            'out_dia': self.out_dia,
            'runout_conf': self.runout_conf,
            'vacuum_conf': self.vacuum_conf,
            'runout_radius': self.runout_radius,
            'runout_phase': self.runout_phase
        }
def load_config_prefix(config):
    return PnPToolTip(config)
