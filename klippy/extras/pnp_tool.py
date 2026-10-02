# klippy/extras/pnp_tool.py
# Object for managing PnP tool operations.
#
# Copyright (C) 2026 Maja Stanislawska <maja@makershop.ie>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
import logging
import gcode

class PnPTool:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.short_name = self.name.split()[-1]
        self.pnp = self.printer.load_object(config, 'pnp')
        self.pnp.register_tool(self)
        self.tip=None #currently loaded
        self.tooltips={}
        self.z_sign, self.z_stepper_name = self._parse_z_axis(config, config.get('z_axis'))
        self.z_stepper = None
        self.z_offset=config.getfloat('z_offset', 0.0)
        self.r_axis=config.get('r_axis', None)
        self.r_stepper = None
        self.r_units=config.getchoice('r_units', {'deg': 'deg', 'rad': 'rad'}, 'deg')
        self.r_limit_velocity = config.getfloat('r_limit_velocity', 999999.9, above=0.)
        self.r_limit_accel = config.getfloat('r_limit_accel', 999999.9, minval=0.)
        valve=config.get('valve', None)
        self.valve=self.printer.load_object(config, valve) if valve else None
        pump=config.get('pump', None)
        self.pump=self.printer.load_object(config, pump) if pump else None
        self.offset_x=config.getfloat('offset_x',0.)
        self.offset_y=config.getfloat('offset_y',0.)
        self.axis_vector = config.getasteval('axis_vector', (0., 0., 1.))

    def resolve_tool_axes(self, z_pool):
        if self.z_stepper_name not in z_pool:
            raise self.printer.config_error(
                "pnp: tool %s z_axis stepper '%s' not in Z pool %s"
                % (self.name, self.z_stepper_name, list(z_pool.keys())))
        self.z_stepper = z_pool[self.z_stepper_name]
        self.r_stepper = self._lookup_r_stepper()

    def _parse_z_axis(self, config, value):
        raw = value.strip()
        sign = 1.0
        if raw[:1] == '-':
            sign = -1.0
            raw = raw[1:].strip()
        elif raw[:1] == '+':
            raw = raw[1:].strip()
        if not raw:
            raise config.error("pnp: empty z_axis")
        key = raw.lower()
        if key.startswith('stepper_'):
            return sign, key
        letter, rest = key[:1], key[1:]
        if letter != 'z' or (rest and not rest.isdigit()):
            raise config.error(
                "pnp: z_axis '%s' must be [+-]Z[n] or [+-]stepper_z[n]"
                % (value,))
        return sign, 'stepper_z' + rest

    def _lookup_r_stepper(self):
        name = self.r_axis.strip()
        candidates = [name]
        if not name.startswith('manual_stepper '):
            candidates.append('manual_stepper ' + name)
        if len(name) == 1 and name.isalpha():
            candidates.append('manual_stepper stepper_' + name.lower())
        for n in candidates:
            obj = self.printer.lookup_object(n, None)
            if obj is not None:
                return obj
        raise self.printer.config_error(
            "pnp: r_axis '%s' is not a [manual_stepper]" % (name,))

    def register_tip(self, tip):
        name=tip.name.upper()
        if name in self.tooltips:
            raise self.printer.config_error("tip %s already registered for tool %s" % (tip.name,self.name))
        self.tooltips[name] = tip
        return tip

    def r_index(self):
        return gcode.axis_map.get(self.pnp.r_gcode_letter)

    def get_r(self, pos):
        idx = self.r_index()
        if idx is not None and idx < len(pos):
            return pos[idx]
        return 0.

    def r_to_rad(self, v):
        if self.r_units == 'deg':
            return v * math.pi / 180.
        return v

    def r_from_rad(self, v):
        if self.r_units == 'deg':
            return v * 180. / math.pi
        return v

    def gcode_to_machine(self, gpos):
        m= list(gpos)
        rho = 0. if self.tip is None else self.tip.runout_radius
        phi = 0. if self.tip is None else self.tip.runout_phase
        r_idx = self.pnp.r_index()
        r_g = gpos[r_idx] # if r_idx is not None and r_idx < len(gpos) else 0.
        th = self.r_to_rad(r_g) + self.r_to_rad(phi)
        z_g = gpos[2]
        ux, uy, uz = self.axis_vector
        m[0] = gpos[0] - self.offset_x - rho * math.cos(th) - z_g * ux / uz
        m[1] = gpos[1] - self.offset_y - rho * math.sin(th) - z_g * uy / uz
        m[2] = z_g
        if r_idx is not None:
            m[r_idx] = r_g
        return m

    def machine_to_gcode(self, mpos):
        rho = 0. if self.tip is None else self.tip.runout_radius
        phi = 0. if self.tip is None else self.tip.runout_phase
        r_idx = self.pnp.r_index()
        r_g = mpos[r_idx] if r_idx is not None and r_idx < len(mpos) else 0.
        th = self.r_to_rad(r_g) + self.r_to_rad(phi)
        z_g = mpos[2]
        ux, uy, uz = self.axis_vector
        g = list(mpos)
        g[0] = mpos[0] + self.offset_x + rho * math.cos(th) + z_g * ux / uz
        g[1] = mpos[1] + self.offset_y + rho * math.sin(th) + z_g * uy / uz
        g[2] = z_g
        return g

    def get_position(self):
        mpos = self.pnp.get_toolhead_pos()
        logging.info(f"PNPTool {self.name} get_position {mpos}")
        return self.machine_to_gcode(mpos)
    def move(self, newpos, speed):
        logging.info(f"PNPTool {self.name} move {newpos} {speed}")
        #here we can implement more sophisticated moves
        #like honoring safe z on move
        #or camera ignoring z moves and adjusting it's ref plane instead.
        mpos = self.gcode_to_machine(newpos)
        self.pnp.next_transform.move(mpos, speed)

    def get_status(self, eventtime):
        return {
            'tips': list(self.tooltips.keys()),
            'tip': None if self.tip is None else self.tip.name,
            'tip_runout': {} if self.tip is None else {
                'radius': self.tip.runout_radius,
                'phase': self.tip.runout_phase,
            },
            'valve': None if self.valve is None else self.valve.name,
            'pump': None if self.pump is None else self.pump.name,
        }
def load_config_prefix(config):
    return PnPTool(config)
