# IDEX per-tool runtime Z offset helper
#
# Applies configured T0/T1 Z deltas when the active extruder changes,
# while preserving any user babystepping (SET_GCODE_OFFSET).
#
# Copyright (C) 2026  Custom Extension
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import probe as probe_module

COPY_POLICIES = {
    'strict': 'strict',
    'warn': 'warn',
    'ignore': 'ignore',
}
COPY_MODES = ('COPY', 'MIRROR')


class IDEXZOffset:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.name = config.get_name()

        # Tool mapping
        self.t0_extruder = config.get('t0_extruder', 'extruder')
        self.t1_extruder = config.get('t1_extruder', 'extruder1')

        # Runtime deltas (mm)
        self.t0_z_offset = config.getfloat('t0_z_offset', 0.)
        self.t1_z_offset = config.getfloat('t1_z_offset', 0.)

        # Behavior
        self.auto_apply = config.getboolean('auto_apply', True)
        self.apply_move_speed = config.getfloat('apply_move_speed', 5., above=0.)
        self.copy_mode_policy = config.getchoice(
            'copy_mode_policy', COPY_POLICIES, 'strict')
        self.copy_mode_tolerance = config.getfloat(
            'copy_mode_tolerance', 0.05, minval=0.)
        self.calibration_use_parking = config.getboolean(
            'calibration_use_parking', True)
        self.calibration_park_t0_gcode = config.get(
            'calibration_park_t0_gcode', 'PARK_T0')
        self.calibration_park_t1_gcode = config.get(
            'calibration_park_t1_gcode', 'PARK_T1')

        # Runtime state
        self.toolhead = None
        self.gcode_move = None
        self.dual_carriage = None
        self.last_applied_tool_delta = 0.
        self.is_ready = False
        self.in_apply = False
        self.in_calibration = False

        self.printer.register_event_handler('klippy:connect', self._handle_connect)
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler(
            'extruder:activate_extruder', self._handle_activate_extruder)

        self.gcode.register_command(
            'IDEX_Z_OFFSETS_STATUS',
            self.cmd_IDEX_Z_OFFSETS_STATUS,
            desc=self.cmd_IDEX_Z_OFFSETS_STATUS_help)
        self.gcode.register_command(
            'SET_IDEX_Z_OFFSETS',
            self.cmd_SET_IDEX_Z_OFFSETS,
            desc=self.cmd_SET_IDEX_Z_OFFSETS_help)
        self.gcode.register_command(
            'APPLY_IDEX_Z_OFFSET',
            self.cmd_APPLY_IDEX_Z_OFFSET,
            desc=self.cmd_APPLY_IDEX_Z_OFFSET_help)
        self.gcode.register_command(
            'CHECK_IDEX_COPY_Z',
            self.cmd_CHECK_IDEX_COPY_Z,
            desc=self.cmd_CHECK_IDEX_COPY_Z_help)
        self.gcode.register_command(
            'SYNC_IDEX_Z_FROM_PROBES',
            self.cmd_SYNC_IDEX_Z_FROM_PROBES,
            desc=self.cmd_SYNC_IDEX_Z_FROM_PROBES_help)
        self.gcode.register_command(
            'CALIBRATE_IDEX_Z_OFFSETS',
            self.cmd_CALIBRATE_IDEX_Z_OFFSETS,
            desc=self.cmd_CALIBRATE_IDEX_Z_OFFSETS_help)
        self.gcode.register_command(
            'ASSERT_IDEX_COPY_SAFE',
            self.cmd_ASSERT_IDEX_COPY_SAFE,
            desc=self.cmd_ASSERT_IDEX_COPY_SAFE_help)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gcode_move = self.printer.lookup_object('gcode_move')
        self.dual_carriage = self.printer.lookup_object('dual_carriage', None)

    def _handle_ready(self):
        self.is_ready = True
        if self.auto_apply:
            self._apply_for_active_extruder(move=False)

    def _handle_activate_extruder(self):
        if not self.is_ready or not self.auto_apply or self.in_calibration:
            return
        self._apply_for_active_extruder(move=True)

    def _require_xyz_homed(self, gcmd):
        curtime = self.printer.get_reactor().monotonic()
        homed_axes = self.toolhead.get_status(curtime).get('homed_axes', '')
        if 'x' not in homed_axes or 'y' not in homed_axes or 'z' not in homed_axes:
            raise gcmd.error('Must home XYZ before calibration')

    def _switch_to_tool(self, tool_index):
        if tool_index == 0:
            extruder = self.t0_extruder
        else:
            extruder = self.t1_extruder
        if self.dual_carriage is not None:
            self.gcode.run_script_from_command(
                'SET_DUAL_CARRIAGE CARRIAGE=%d' % (tool_index,))
        self.gcode.run_script_from_command(
            'ACTIVATE_EXTRUDER EXTRUDER=%s' % (extruder,))

    def _restore_tool(self, extruder_name):
        if extruder_name == self.t0_extruder:
            self._switch_to_tool(0)
        elif extruder_name == self.t1_extruder:
            self._switch_to_tool(1)
        elif extruder_name:
            self.gcode.run_script_from_command(
                'ACTIVATE_EXTRUDER EXTRUDER=%s' % (extruder_name,))

    def _get_probe_for_tool(self, tool_index):
        dual_probe = self.printer.lookup_object('dual_probe', None)
        if dual_probe is not None:
            probe = dual_probe.get_probe(tool_index)
            if probe is not None:
                return probe
        return self.printer.lookup_object('probe', None)

    def _park_inactive_tool(self, target_tool_index, gcmd):
        if target_tool_index == 0:
            park_cmd = self.calibration_park_t1_gcode
        else:
            park_cmd = self.calibration_park_t0_gcode
        if not park_cmd:
            return
        try:
            self.gcode.run_script_from_command(park_cmd)
        except self.printer.command_error as e:
            raise gcmd.error(
                "Calibration parking command '%s' failed: %s"
                % (park_cmd, e))

    def _extract_probe_z(self, pos):
        if hasattr(pos, 'bed_z'):
            return pos.bed_z
        if hasattr(pos, 'z'):
            return pos.z
        try:
            if len(pos) >= 3:
                return pos[2]
        except Exception:
            pass
        raise self.printer.command_error('Unable to parse probe result')

    def _probe_tool_at_xy(self, gcmd, tool_index, x, y, speed, lift_z,
                          use_parking):
        probe = self._get_probe_for_tool(tool_index)
        if probe is None:
            raise gcmd.error('No probe available for T%d' % (tool_index,))

        if use_parking:
            self._park_inactive_tool(tool_index, gcmd)

        self._switch_to_tool(tool_index)
        pos = self.toolhead.get_position()
        safe_z = max(pos[2], lift_z)
        self.toolhead.manual_move([None, None, safe_z], speed)

        offsets = probe.get_offsets()
        target = [x - offsets[0], y - offsets[1], None]
        self.toolhead.manual_move(target, speed)
        self.toolhead.wait_moves()

        result = probe_module.run_single_probe(probe, gcmd)
        trigger_z = self._extract_probe_z(result)
        probe_z_offset = probe.get_offsets()[2]

        pos = self.toolhead.get_position()
        safe_z = max(pos[2], lift_z)
        self.toolhead.manual_move([None, None, safe_z], speed)
        self.toolhead.wait_moves()
        return trigger_z, probe_z_offset

    def _get_active_extruder_name(self):
        if self.toolhead is None:
            return ''
        try:
            return self.toolhead.get_extruder().get_name()
        except Exception:
            return ''

    def _get_delta_for_extruder(self, extruder_name):
        if extruder_name == self.t0_extruder:
            return self.t0_z_offset
        if extruder_name == self.t1_extruder:
            return self.t1_z_offset
        return 0.

    def _is_z_homed(self):
        curtime = self.printer.get_reactor().monotonic()
        homed_axes = self.toolhead.get_status(curtime).get('homed_axes', '')
        return 'z' in homed_axes

    def _get_homing_origin_z(self):
        return self.gcode_move.get_status()['homing_origin'].z

    def _run_set_gcode_offset(self, target_origin_z, move):
        move = 1 if move and self._is_z_homed() else 0
        cmd = 'SET_GCODE_OFFSET Z=%.6f MOVE=%d' % (target_origin_z, move)
        if move:
            cmd += ' MOVE_SPEED=%.3f' % (self.apply_move_speed,)
        self.gcode.run_script_from_command(cmd)

    def _apply_for_extruder(self, extruder_name, move):
        if self.in_apply:
            return
        new_delta = self._get_delta_for_extruder(extruder_name)
        cur_origin = self._get_homing_origin_z()
        target_origin = cur_origin - self.last_applied_tool_delta + new_delta
        if abs(target_origin - cur_origin) < 1.0e-9:
            self.last_applied_tool_delta = new_delta
            return
        self.in_apply = True
        try:
            self._run_set_gcode_offset(target_origin, move)
            self.last_applied_tool_delta = new_delta
            logging.info(
                'idex_z_offset: applied %.4f for %s (origin %.5f->%.5f)',
                new_delta, extruder_name, cur_origin, target_origin)
        finally:
            self.in_apply = False

    def _apply_for_active_extruder(self, move):
        extruder = self._get_active_extruder_name()
        self._apply_for_extruder(extruder, move)

    def _get_dual_modes(self):
        if self.dual_carriage is None:
            return []
        curtime = self.printer.get_reactor().monotonic()
        status = self.dual_carriage.get_status(curtime)
        carriages = status.get('carriages', None)
        if isinstance(carriages, dict):
            return list(carriages.values())
        modes = []
        for key, value in status.items():
            if key.startswith('carriage_'):
                modes.append(value)
        return modes

    def _check_copy_mismatch(self):
        delta = abs(self.t1_z_offset - self.t0_z_offset)
        in_copy = any(mode in COPY_MODES for mode in self._get_dual_modes())
        exceeds = delta > self.copy_mode_tolerance
        return in_copy, delta, exceeds

    def _format_status(self):
        active_extruder = self._get_active_extruder_name()
        in_copy, delta, exceeds = self._check_copy_mismatch()
        return (
            'IDEX Z offsets:\n'
            '  t0_extruder: %s\n'
            '  t1_extruder: %s\n'
            '  t0_z_offset: %.5f\n'
            '  t1_z_offset: %.5f\n'
            '  active_extruder: %s\n'
            '  active_delta: %.5f\n'
            '  auto_apply: %s\n'
            '  apply_move_speed: %.3f\n'
            '  copy_mode_policy: %s\n'
            '  copy_mode_tolerance: %.5f\n'
            '  calibration_use_parking: %s\n'
            '  calibration_park_t0_gcode: %s\n'
            '  calibration_park_t1_gcode: %s\n'
            '  copy_mode_active: %s\n'
            '  copy_delta_abs: %.5f\n'
            '  copy_exceeds_tolerance: %s' % (
                self.t0_extruder, self.t1_extruder,
                self.t0_z_offset, self.t1_z_offset,
                active_extruder, self.last_applied_tool_delta,
                self.auto_apply, self.apply_move_speed,
                self.copy_mode_policy, self.copy_mode_tolerance,
                self.calibration_use_parking,
                self.calibration_park_t0_gcode,
                self.calibration_park_t1_gcode,
                in_copy, delta, exceeds))

    def get_status(self, eventtime):
        in_copy, delta, exceeds = self._check_copy_mismatch()
        return {
            't0_extruder': self.t0_extruder,
            't1_extruder': self.t1_extruder,
            't0_z_offset': self.t0_z_offset,
            't1_z_offset': self.t1_z_offset,
            'active_extruder': self._get_active_extruder_name(),
            'active_delta': self.last_applied_tool_delta,
            'auto_apply': self.auto_apply,
            'apply_move_speed': self.apply_move_speed,
            'copy_mode_policy': self.copy_mode_policy,
            'copy_mode_tolerance': self.copy_mode_tolerance,
            'calibration_use_parking': self.calibration_use_parking,
            'calibration_park_t0_gcode': self.calibration_park_t0_gcode,
            'calibration_park_t1_gcode': self.calibration_park_t1_gcode,
            'copy_mode_active': in_copy,
            'copy_delta_abs': delta,
            'copy_exceeds_tolerance': exceeds,
        }

    cmd_IDEX_Z_OFFSETS_STATUS_help = 'Report current IDEX Z offset state'
    def cmd_IDEX_Z_OFFSETS_STATUS(self, gcmd):
        gcmd.respond_info(self._format_status())

    cmd_SET_IDEX_Z_OFFSETS_help = 'Set T0/T1 offsets (SET_IDEX_Z_OFFSETS T0=<mm> T1=<mm> [SAVE=1])'
    def cmd_SET_IDEX_Z_OFFSETS(self, gcmd):
        t0 = gcmd.get_float('T0', None)
        t1 = gcmd.get_float('T1', None)
        save = gcmd.get_int('SAVE', 0, minval=0, maxval=1)

        if t0 is not None:
            self.t0_z_offset = t0
        if t1 is not None:
            self.t1_z_offset = t1

        if save:
            configfile = self.printer.lookup_object('configfile')
            configfile.set(self.name, 't0_z_offset', '%.6f' % (self.t0_z_offset,))
            configfile.set(self.name, 't1_z_offset', '%.6f' % (self.t1_z_offset,))

        if self.auto_apply and self.is_ready:
            self._apply_for_active_extruder(move=False)

        gcmd.respond_info(
            'Set offsets: T0=%.5f T1=%.5f%s' % (
                self.t0_z_offset, self.t1_z_offset,
                ' (saved)' if save else ''))

    cmd_APPLY_IDEX_Z_OFFSET_help = 'Apply current offset for active extruder (or EXTRUDER=<name>)'
    def cmd_APPLY_IDEX_Z_OFFSET(self, gcmd):
        extruder_name = gcmd.get('EXTRUDER', None)
        if extruder_name is None:
            t = gcmd.get_int('T', None, minval=0, maxval=1)
            if t is not None:
                extruder_name = self.t0_extruder if t == 0 else self.t1_extruder
            else:
                extruder_name = self._get_active_extruder_name()
        move = gcmd.get_int('MOVE', 1, minval=0, maxval=1)
        self._apply_for_extruder(extruder_name, bool(move))
        gcmd.respond_info('Applied offset for extruder %s' % (extruder_name,))

    cmd_CHECK_IDEX_COPY_Z_help = 'Validate T0/T1 delta against copy_mode_tolerance'
    def cmd_CHECK_IDEX_COPY_Z(self, gcmd):
        in_copy, delta, exceeds = self._check_copy_mismatch()
        msg = ('COPY/MIRROR Z delta: %.5f (tol=%.5f) active=%s'
               % (delta, self.copy_mode_tolerance, in_copy))
        if not exceeds:
            gcmd.respond_info(msg + ' OK')
            return
        if self.copy_mode_policy == 'strict':
            raise gcmd.error(msg + ' exceeds tolerance')
        if self.copy_mode_policy == 'warn':
            gcmd.respond_info('WARNING: ' + msg + ' exceeds tolerance')
            return
        gcmd.respond_info(msg + ' (ignored by policy)')

    cmd_ASSERT_IDEX_COPY_SAFE_help = (
        'Hard fail if absolute T0/T1 mismatch exceeds copy_mode_tolerance')
    def cmd_ASSERT_IDEX_COPY_SAFE(self, gcmd):
        in_copy, delta, exceeds = self._check_copy_mismatch()
        if exceeds:
            raise gcmd.error(
                'COPY/MIRROR blocked: delta %.5f exceeds tolerance %.5f'
                % (delta, self.copy_mode_tolerance))
        gcmd.respond_info(
            'COPY/MIRROR safe: delta %.5f within tolerance %.5f (active=%s)'
            % (delta, self.copy_mode_tolerance, in_copy))

    cmd_SYNC_IDEX_Z_FROM_PROBES_help = (
        'Set T0/T1 offsets from probe z_offset values '
        '(T0=probe z_offset, T1=dual_probe z_offset if available)')
    def cmd_SYNC_IDEX_Z_FROM_PROBES(self, gcmd):
        t0_probe = self.printer.lookup_object('probe', None)
        dual_probe = self.printer.lookup_object('dual_probe', None)
        if t0_probe is None:
            raise gcmd.error('Primary probe object not found')

        t0_offsets = t0_probe.get_offsets()
        t0_z = t0_offsets[2]

        t1_z = t0_z
        if dual_probe is not None:
            t1_probe = dual_probe.get_probe(1)
            if t1_probe is not None:
                t1_offsets = t1_probe.get_offsets()
                t1_z = t1_offsets[2]

        # Keep T0 as runtime reference and apply T1 delta relative to T0.
        self.t0_z_offset = 0.
        self.t1_z_offset = t1_z - t0_z

        save = gcmd.get_int('SAVE', 0, minval=0, maxval=1)
        if save:
            configfile = self.printer.lookup_object('configfile')
            configfile.set(self.name, 't0_z_offset', '%.6f' % (self.t0_z_offset,))
            configfile.set(self.name, 't1_z_offset', '%.6f' % (self.t1_z_offset,))

        if self.auto_apply and self.is_ready:
            self._apply_for_active_extruder(move=False)

        gcmd.respond_info(
            'Synced offsets from probes: T0=%.5f T1=%.5f%s' % (
                self.t0_z_offset, self.t1_z_offset,
                ' (saved)' if save else ''))

    cmd_CALIBRATE_IDEX_Z_OFFSETS_help = (
        'Probe with T0 and T1 at same XY and set runtime offsets '
        '(CALIBRATE_IDEX_Z_OFFSETS X=<mm> Y=<mm> [LIFT_Z=<mm>] '
        '[SPEED=<mm/s>] [PARK=<0|1>] [SAVE=<0|1>] [APPLY=<0|1>])')
    def cmd_CALIBRATE_IDEX_Z_OFFSETS(self, gcmd):
        x = gcmd.get_float('X', None)
        y = gcmd.get_float('Y', None)
        if x is None or y is None:
            raise gcmd.error('CALIBRATE_IDEX_Z_OFFSETS requires X and Y')
        lift_z = gcmd.get_float('LIFT_Z', 5., above=0.)
        speed = gcmd.get_float('SPEED', 50., above=0.)
        park = gcmd.get_int(
            'PARK', 1 if self.calibration_use_parking else 0,
            minval=0, maxval=1)
        use_parking = bool(park)
        save = gcmd.get_int('SAVE', 0, minval=0, maxval=1)
        apply_now = gcmd.get_int('APPLY', 1, minval=0, maxval=1)

        self._require_xyz_homed(gcmd)
        orig_extruder = self._get_active_extruder_name()

        self.in_calibration = True
        t0_trigger_z = t1_trigger_z = 0.
        t0_probe_z_offset = t1_probe_z_offset = 0.
        t0_nozzle_z = t1_nozzle_z = 0.
        try:
            pos = self.toolhead.get_position()
            safe_z = max(pos[2], lift_z)
            self.toolhead.manual_move([None, None, safe_z], speed)
            self.toolhead.wait_moves()

            t0_trigger_z, t0_probe_z_offset = self._probe_tool_at_xy(
                gcmd, 0, x, y, speed, lift_z, use_parking)
            t1_trigger_z, t1_probe_z_offset = self._probe_tool_at_xy(
                gcmd, 1, x, y, speed, lift_z, use_parking)

            # Normalize trigger heights to nozzle reference so probe mounting
            # differences do not appear as toolhead nozzle mismatch.
            t0_nozzle_z = t0_trigger_z - t0_probe_z_offset
            t1_nozzle_z = t1_trigger_z - t1_probe_z_offset
            self.t0_z_offset = 0.
            self.t1_z_offset = t1_nozzle_z - t0_nozzle_z

            if save:
                configfile = self.printer.lookup_object('configfile')
                configfile.set(self.name, 't0_z_offset',
                               '%.6f' % (self.t0_z_offset,))
                configfile.set(self.name, 't1_z_offset',
                               '%.6f' % (self.t1_z_offset,))
        finally:
            try:
                self._restore_tool(orig_extruder)
            except Exception:
                logging.exception('idex_z_offset: failed restoring tool')
            self.in_calibration = False

        if apply_now and self.is_ready:
            self._apply_for_active_extruder(move=False)

        gcmd.respond_info(
            'Calibrated at X=%.3f Y=%.3f: '
            'T0(trigger=%.6f z_offset=%.6f nozzle=%.6f) '
            'T1(trigger=%.6f z_offset=%.6f nozzle=%.6f) '
            '=> runtime offsets T0=%.5f T1=%.5f (PARK=%d)%s' % (
                x, y,
                t0_trigger_z, t0_probe_z_offset, t0_nozzle_z,
                t1_trigger_z, t1_probe_z_offset, t1_nozzle_z,
                self.t0_z_offset, self.t1_z_offset,
                park,
                ' (saved)' if save else ''))


def load_config(config):
    return IDEXZOffset(config)
