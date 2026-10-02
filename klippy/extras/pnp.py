# klippy/extras/pnp.py
# gcode transform for quick (tool) offsets swapping and misalignment compensations
# also Central hub for all PNP related operations.
#
# Copyright (C) 2026 Maja Stanislawska <maja@makershop.ie>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import gcode
import chelper
import logging

class VirtualGcodeAxis:
    """Extra-axis slot with no stepper. Camera R lives here for M114/G1."""
    def __init__(self, owner, printer, letter):
        self.owner = owner
        self.printer = printer
        self.name = '%s_virtual_axis_%s' % (owner.name, letter.lower())
        self.axis_gcode_id = None
        self.commanded_pos = 0.
    def get_axis_gcode_id(self):
        return self.axis_gcode_id
    def process_move(self, print_time, move, ea_index):
        self.commanded_pos = move.end_pos[ea_index]
        logging.info("pnp_virtual_axis %s gcode:%s commanded_pos=%.4f", self.name, self.axis_gcode_id, self.commanded_pos)
    def check_move(self, move, ea_index):
        logging.info("pnp_virtual_axis %s gcode:%s check_move=%.4f", self.name, self.axis_gcode_id, move.end_pos[ea_index])
        return
    def calc_junction(self, prev_move, move, ea_index):
        return move.max_cruise_v2

class PnPManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name='pnp'
        self.gcode = self.printer.load_object(config, 'gcode')
        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.fiducial_primary = config.getasteval('fiducial_primary', {})
        self.fiducial_secondary = config.getasteval('fiducial_secondary', {})
        self.discard_location = config.getasteval('discard_location', {})
        self.default_board_location = config.getasteval('default_board_location', {})
        self.r_gcode_letter = config.get('r_gcode_letter', 'R').upper()
        if self.r_gcode_letter in ('X', 'Y', 'Z', 'E', 'F', 'N'):
            raise config.error("pnp: r_gcode_letter cannot be %s"
                               % (self.r_gcode_letter,))

        self.tools = {}
        self.active_tool = None
        self.toolhead = None
        self.next_transform = None
        self._prev_G1 = None
        self._prev_G0 = None
        self._prev_SAVE = None
        self._prev_RESTORE = None
        self.z_pool = {}
        self._fallback_z = None #cache for first stepper_z, used when tool is none
        self._active_z = (None, 1.0)
        self._bound_r = None
        self._virtual_r = self.new_virtual_axis(self)
        self._ffi_lib = None

        self.gcode.register_command('PNP_TOOL', self.cmd_PNP_TOOL,
                                    desc=self.cmd_PNP_TOOL_help)
        self.gcode.register_command('PNP_TIP', self.cmd_PNP_TIP,
                                    desc=self.cmd_PNP_TIP_help)

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self._setup_z_pool()
        for tool in self.tools.values():
            tool.resolve_tool_axes(self.z_pool)
        logging.info("pnp: binding virtual R axis to %s", self.r_gcode_letter)
        self.bind_gcode_axis(self._virtual_r, self.r_gcode_letter)
        self._bound_r=self._virtual_r

    def _handle_ready(self):
        self.next_transform = self.gcode_move.set_move_transform(self, force=True)
        if self.gcode_move.move_transform is not self:
            raise self.printer.config_error(
                "pnp: failed to become outermost move_transform")
        if (self.next_transform is not None
                and self.next_transform is not self.toolhead):
            logging.warning("pnp: wrapping existing move_transform %s",
                            type(self.next_transform).__name__)
        self._wrap_commands()


    def unbind_gcode_axis(self, stepper):
        if stepper.axis_gcode_id is None:
            return
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.remove_extra_axis(stepper)
        stepper.axis_gcode_id = None
    def bind_gcode_axis(self, stepper, gcode_axis, instant_corner_v=1.,
                        limit_velocity=999999.9, limit_accel=999999.9):
        gcode_axis = gcode_axis.upper()
        if stepper.axis_gcode_id is not None:
            if stepper.axis_gcode_id == gcode_axis:
                return
            self.unbind_gcode_axis(stepper)
        if (len(gcode_axis) != 1 or not gcode_axis.isupper()
                or gcode_axis in "XYZEFN"):
            raise self.printer.command_error("Not a valid GCODE_AXIS")
        toolhead = self.printer.lookup_object('toolhead')
        for ea in toolhead.get_extra_axes():
            if ea is not None and ea.get_axis_gcode_id() == gcode_axis:
                raise self.printer.command_error(
                    "Axis '%s' already registered" % (gcode_axis,))
        stepper.axis_gcode_id = gcode_axis
        stepper.instant_corner_v = instant_corner_v
        stepper.gaxis_limit_velocity = limit_velocity
        stepper.gaxis_limit_accel = limit_accel
        toolhead.add_extra_axis(stepper, stepper.commanded_pos)
        if len(self.gcode_move.last_position)<len(self.toolhead.pos_axes):
            self.gcode_move.last_position.append(self.r_index())
        self.reset_last_position()

    def _setup_z_pool(self):
        kin = self.toolhead.get_kinematics()
        ffi_main, ffi_lib = chelper.get_ffi()
        self._ffi_lib = ffi_lib
        z_steppers = []
        for s in kin.get_steppers():
            if (s.is_active_axis('z')
                    and not s.is_active_axis('x')
                    and not s.is_active_axis('y')):
                z_steppers.append(s)
        if not z_steppers:
            raise self.printer.config_error(
                "pnp: no dedicated Z steppers found for tool rebind")
        pos = list(self.toolhead.get_position())
        for i, s in enumerate(z_steppers):
            s.setup_itersolve('generic_cartesian_stepper_alloc',
                              0.0, 0.0, 1.0 if i == 0 else 0.0)
            s.set_position(pos)
            self.z_pool[s.get_name()] = s
        self._fallback_z = z_steppers[0]
        ident = self.z_pool.get('stepper_z')
        if ident is not None:
            self._fallback_z = ident
        self._active_z = (self._fallback_z, 1.0)
        orig_calc = kin.calc_position
        def calc_position(stepper_positions):
            pos = list(orig_calc(stepper_positions))
            stepper, az = self._active_z
            if stepper is not None and az:
                name = stepper.get_name()
                if name in stepper_positions:
                    pos[2] = stepper_positions[name] / az
            return pos
        kin.calc_position = calc_position
        logging.info("pnp: Z pool %s (fallback %s)",
                     list(self.z_pool.keys()), self._fallback_z.get_name())

    def _set_z_coeffs(self, stepper, az):
        self._ffi_lib.generic_cartesian_stepper_set_coeffs(
            stepper.get_stepper_kinematics(), 0.0, 0.0, float(az))

    def _park_z_actuator(self):
        tool = self.active_tool
        pos = self.get_position()
        if abs(pos[2]) > 1e-6:
            pos[2] = 0.
            self.move_and_settle(pos, self.gcode_move.speed, 0.)

    def _rebind_z(self, tool):
        self.toolhead.flush_step_generation()
        logging.info("pnp: rebind Z for tool %s", None if tool is None else tool.name)
        pos = list(self.toolhead.get_position())
        pos[2] = 0.
        target = self._fallback_z
        az = 1.0
        if tool is not None:
            zs = getattr(tool, 'z_stepper', None)
            if zs is not None:
                target = zs
                az = float(getattr(tool, 'z_sign', 1.0))
        for stepper in self.z_pool.values():
            self._set_z_coeffs(stepper, az if stepper is target else 0.0)
            stepper.set_position(pos)
        self._active_z = (target, az)
        self.printer.send_event('dual_carriage:update_kinematics')
        logging.info("pnp: rebind Z %s coeffs=%.0f",
                     target.get_name() if target is not None else None, az)

    def r_index(self):
        return gcode.axis_map.get(self.r_gcode_letter)

    def new_virtual_axis(self, owner):
        return VirtualGcodeAxis(owner, self.printer, self.r_gcode_letter)

    def _rebind_r(self, tool):
        want = self._virtual_r if tool is None else tool.r_stepper
        logging.info("pnp: rebind R from tool %s to %s(%s)",
            self.active_tool.name if self.active_tool else 'None',
            None if tool is None else tool.name,
            want.name)
        r_kw = {}
        if tool is not None and want is not self._virtual_r:
            r_kw['limit_velocity'] = getattr(tool, 'r_limit_velocity', 999999.9)
            r_kw['limit_accel'] = getattr(tool, 'r_limit_accel', 999999.9)
        if self._bound_r is not None:
            self.unbind_gcode_axis(self._bound_r)
        self.bind_gcode_axis(want, self.r_gcode_letter, **r_kw)
        self._bound_r = want
        logging.info("pnp: rebind R done %s", want.name)

    def _steal(self, name, wrapper):
        prev = self.gcode.register_command(name, None)
        self.gcode.register_command(name, wrapper)
        return prev

    def _wrap_commands(self):
        self._prev_G1 = self._steal("G1", self.cmd_G1_wrap)
        self._prev_G0 = self._steal("G0", self.cmd_G0_wrap)
        if self._prev_G1 is None:
            raise self.printer.config_error("pnp: G1 not registered")
        # if self._prev_SAVE is None or self._prev_RESTORE is None:
        #     raise self.printer.config_error(
        #         "pnp: SAVE/RESTORE_GCODE_STATE not registered")

    def cmd_G0_wrap(self, gcmd):
        params = gcmd.get_command_parameters()
        tool = self.active_tool
        letter = self.r_gcode_letter
        return self._prev_G0(gcmd)

    def cmd_G1_wrap(self, gcmd):
        params = gcmd.get_command_parameters()
        tool = self.active_tool
        letter = self.r_gcode_letter
        return self._prev_G1(gcmd)

    def _tool_key(self):
        if self.active_tool is None:
            return None
        return self.active_tool.short_name.upper()

    def register_tool(self, tool):
        toolname = tool.short_name.upper()
        if toolname in self.tools:
            raise self.printer.config_error("tool %s (%s) already registered" % (tool.name,tool.short_name))
        self.tools[toolname] = tool
        return tool
    def lookup_tool(self, toolname):
        if toolname is None:
            return self.active_tool
        key = toolname.upper()
        if key not in self.tools:
            raise self.printer.command_error(
                f"Unknown tool '{toolname}'. Known: {list(self.tools.keys())}. Use 'NONE' to unset")
        return self.tools[key]

    def select_tool(self, tool,want_move):
        pos = list(self.get_position())
        old = self.active_tool
        old_tool=old.name if old else 'None'
        new_tool=tool.name if tool else 'None'
        logging.info(f"pnp.select_tool {old_tool}->{new_tool}, move={want_move} pos={pos}")
        self._park_z_actuator()
        self._rebind_z(tool)
        self._rebind_r(tool)
        self.active_tool = tool
        if want_move:
            self.safe_move(pos,self.gcode_move.speed)
        self.reset_last_position()
        logging.info(f"pnp.select_tool done={self.get_position()} last={self.gcode_move.last_position}")

    def reset_last_position(self):
        return self.gcode_move.reset_last_position()

    def get_position(self):
        if self.active_tool is None:
            return self.next_transform.get_position()
        return self.active_tool.get_position()

    def get_toolhead_pos(self):
        return self.toolhead.get_position()

    def move(self, newpos, speed):
        if self.active_tool is None:
            self.next_transform.move(newpos, speed)
        else:
            self.active_tool.move(newpos, speed)
        self.reset_last_position()

    def safe_move(self, dest, speed):
        curr = self.get_position()
        newpos=curr.copy()
        for i in range(len(dest)):
            if dest[i] is not None:
                newpos[i] = dest[i]
        logging.info(f"pnp.safe_move {curr} -> {newpos} ({dest}) {speed}")
        if self.active_tool is None:
            self.next_transform.move(newpos, speed)
            return
        curr[2] = 0
        self.active_tool.move(curr, speed)
        self.active_tool.move(newpos[:2]+[0]+newpos[3:], speed)
        self.active_tool.move(newpos, speed)
        self.gcode_move.last_position = list(newpos)

    def move_and_settle(self, pos, speed, settle):
        self.move(pos, speed)
        self.toolhead.wait_moves()
        if settle:
            self.reactor.pause(self.reactor.monotonic() + settle)

    cmd_PNP_TOOL_help = "Select the active PnP tool (camera or nozzle)."
    def cmd_PNP_TOOL(self, gcmd):
        toolname = gcmd.get('SET', None)
        if toolname is None:
            if self.active_tool is None:
                gcmd.respond_info("No tool selected")
            else:
                gcmd.respond_info("Current tool: %s" % (self.active_tool.name,))
            return
        want_move = gcmd.get('MOVE', '0').upper() in ['1', 'T', 'TRUE']
        tool=None
        if toolname.upper() == 'NONE':
            gcmd.respond_info("Current tool cleared")
            self.select_tool(tool, want_move)
            return
        tool = self.lookup_tool(toolname)
        if tool == self.active_tool:
            gcmd.respond_info("already active")
            return
        gcmd.respond_info("Setting tool to %s" % (tool.name,))
        gcmd.respond_info("Offsets: %s,%s"% (tool.offset_x,tool.offset_y))
        self.select_tool(tool, want_move)
        tip=getattr(self.active_tool, 'tip', 'meh')
        match tip:
            case 'meh': gcmd.respond_info("tool %s dont support tips" % (self.active_tool.name,))
            case None: gcmd.respond_info("No tip loaded on %s" % (self.active_tool.name,))
            case _: gcmd.respond_info("Tip: %s " % (str(self.active_tool.tip,)))

    cmd_PNP_TIP_help = "Load or unload a tooltip on a nozzle."
    def cmd_PNP_TIP(self, gcmd):
        #TODO: switch tool if different that active
        #TODO: unload old tip if present
        #TODO: run auto toolchanger sequence
        #TODO: `MANUAL=1` to override above (maybe move to 'manual location' for human to swap like OpenPNP)
        if gcmd.get('TOOL', None) is not None:
            tool = self.lookup_tool(gcmd.get('TOOL'))
        else:
            tool = self.active_tool
        if tool is None:
            raise gcmd.error("No tool selected")
        unload = gcmd.get_int('UNLOAD', 0)
        if unload:
            tool.tip = None
            return
        load = gcmd.get('LOAD', None)
        if load is None:
            gcmd.respond_info("tip %s" % (None if tool.tip is None else tool.tip.name,))
            return
        tip=load.upper()
        if not tip in tool.tooltips:
            raise gcmd.error("invalid tip '%s' for tool %s" % (tip, tool.name))
        tool.tip = tool.tooltips[tip]
        gcmd.respond_info("loaded %s on %s" % (tool.tip.name, tool.short_name))

    def get_status(self, eventtime):
        z_axis = None if self._active_z[0] is None else "%s%s"%(
            '+' if self._active_z[1] >= 0 else '-', self._active_z[0].get_name())
        if self.active_tool is None:
            tip = "tool not selected"
        else:
            tip = str(getattr(self.active_tool, 'tip', 'tool dont support tips'))
        return {
            'tools': list(self.tools.keys()),
            'active_tool': self._tool_key(),
            'active_tip': tip,
            'z_axis': z_axis,
            'r_axis': None if self._bound_r is None else self._bound_r.name,
            'fiducial_primary': self.fiducial_primary,
            'fiducial_secondary': self.fiducial_secondary,
        }

def load_config(config):
    return PnPManager(config)
