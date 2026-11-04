# BFP 8 Dual Nozzle Bed Mesh Tracking - active development prototype
#
# This module keeps normal Klipper bed_mesh as the T0 reference and drives
# stepper_z..stepper_z3 as synchronized extra axes. A damped four-corner
# gantry solver creates only the differential correction needed by T1.
#
# Development use only. Enable explicitly after homing, QGL, mesh load,
# IDEX mode selection, and positioning both nozzles inside the mesh.

import math
import logging
import chelper
from . import bfp_dual_mesh_math


class _BFPMotorAxis:
    def __init__(self, owner, stepper, index, start_pos):
        self.owner = owner
        self.printer = owner.printer
        self.stepper = stepper
        self.index = index
        self.name = "bfp_dual_z%d" % (index,)
        self.axis_gcode_id = None
        self.commanded_pos = start_pos
        self.instant_corner_v = owner.motor_corner_velocity
        self.limit_velocity = owner.max_motor_velocity
        self.limit_accel = owner.max_motor_accel
        self.pos_min = owner.z_min
        self.pos_max = owner.z_max

        self.motion_queuing = self.printer.lookup_object('motion_queuing')
        self.trapq = self.motion_queuing.allocate_trapq()
        self.trapq_append = self.motion_queuing.lookup_trapq_append()

        self.orig_sk = stepper.get_stepper_kinematics()
        self.orig_trapq = stepper.get_trapq()

        ffi_main, ffi_lib = chelper.get_ffi()
        self.ffi_lib = ffi_lib
        self.sk = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b'z'), ffi_lib.free)

        # Swap this physical Z stepper onto its own Z-coordinate solver.
        # set_stepper_kinematics() and set_position() both preserve the MCU
        # physical position, so enabling tracking itself does not move a motor.
        stepper.set_stepper_kinematics(self.sk)
        stepper.set_trapq(self.trapq)
        stepper.set_position([0., 0., start_pos])

        print_time = owner.toolhead.get_last_move_time()
        ffi_lib.trapq_set_position(
            self.trapq, print_time, 0., 0., start_pos)

    def get_name(self):
        return self.name

    def get_axis_gcode_id(self):
        # Deliberately not exposed as a user G-code axis.
        return self.axis_gcode_id

    def get_trapq(self):
        return self.trapq

    def process_move(self, print_time, move, ea_index):
        axis_r = move.axes_r[ea_index]
        start_pos = move.start_pos[ea_index]
        accel = move.accel * axis_r
        start_v = move.start_v * axis_r
        cruise_v = move.cruise_v * axis_r
        self.trapq_append(
            self.trapq, print_time,
            move.accel_t, move.cruise_t, move.decel_t,
            0., 0., start_pos,
            0., 0., 1.,
            start_v, cruise_v, accel)
        self.commanded_pos = move.end_pos[ea_index]

    def check_move(self, move, ea_index):
        movepos = move.end_pos[ea_index]
        if movepos < self.pos_min or movepos > self.pos_max:
            raise move.move_error(
                "BFP dual tracking Z motor target out of range")
        delta = abs(move.axes_d[ea_index])
        if delta < 1.0e-12:
            return
        # BFP motor axes are always changed together with XYZ motion. Limit
        # the whole move so the individual Z motor obeys its own v/a limits.
        if move.move_d < 1.0e-12:
            raise move.move_error(
                "BFP dual tracking attempted an unsupported extra-axis-only move")
        axis_ratio = move.move_d / delta
        move.limit_speed(self.limit_velocity * axis_ratio,
                         self.limit_accel * axis_ratio)

    def calc_junction(self, prev_move, move, ea_index):
        diff_r = move.axes_r[ea_index] - prev_move.axes_r[ea_index]
        if abs(diff_r) > 1.0e-15:
            return (self.instant_corner_v / abs(diff_r)) ** 2
        return move.max_cruise_v2

    def restore(self, xyz_position):
        # Restore the exact solver/trapq that Klipper had before tracking.
        self.stepper.set_trapq(None)
        self.stepper.set_stepper_kinematics(self.orig_sk)
        self.stepper.set_trapq(self.orig_trapq)
        self.stepper.set_position(xyz_position[:3])


class _BFPBedMeshToolheadProxy:
    """Placed under bed_mesh so we see every post-bed_mesh endpoint."""
    def __init__(self, owner, real_toolhead):
        self.owner = owner
        self.real_toolhead = real_toolhead

    def move(self, newpos, speed):
        return self.owner._tracking_move(newpos, speed)

    def get_position(self):
        return self.real_toolhead.get_position()

    def __getattr__(self, name):
        return getattr(self.real_toolhead, name)


class BFPDualTracking:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()

        # Existing proven calibration object.
        self.calibration_object_name = config.get(
            'calibration_object', 'bfp_dual_calibration')

        # IDEX rail names. Klipper versions have exposed the primary rail as
        # either "x" or "stepper_x", so connection performs a safe alias
        # fallback if the configured name is not present.
        self.t0_carriage_name = config.get('t0_carriage_name', 'stepper_x')
        self.t1_carriage_name = config.get('t1_carriage_name', 'dual_carriage')

        # New machines must have their physical Z corner order checked before
        # active tracking is allowed. Existing BFP8 configs retain their old
        # behavior because the compatibility default is True; V2 explicitly
        # sets this False until its motor-map test is complete.
        self.motor_map_verified = config.getboolean(
            'motor_map_verified', True)

        # Optional once-per-interval Z motor position reporting. Zero disables
        # periodic reporting. V2 config enables this at 1.0 second.
        self.motor_status_interval = config.getfloat(
            'motor_status_interval', 0.0, minval=0.0)

        # Four-corner gantry support model. Ordering follows Klipper QGL:
        # z0=front-left, z1=rear-left, z2=rear-right, z3=front-right.
        self.left_x = config.getfloat('left_x', -129.25)
        self.right_x = config.getfloat('right_x', 949.23)
        self.front_y = config.getfloat('front_y', 70.65)
        self.rear_y = config.getfloat('rear_y', 884.25)
        if self.right_x <= self.left_x:
            raise config.error('bfp_dual_tracking: right_x must be > left_x')
        if self.rear_y <= self.front_y:
            raise config.error('bfp_dual_tracking: rear_y must be > front_y')

        # Damped authority. When both nozzles approach the center seam,
        # w0 and w1 become nearly identical. The lambda term intentionally
        # reduces differential correction instead of demanding huge motor travel.
        self.authority_lambda = config.getfloat(
            'authority_lambda', 0.0025, above=0.)

        # Reject geometries where T0 and T1 are too close for the gantry
        # to generate a well-conditioned differential Z correction.
        self.min_condition_q = config.getfloat(
            'min_condition_q', 0.10, minval=0.0)
        self.max_tracking_residual = config.getfloat(
            'max_tracking_residual', 0.010, above=0.0)


        # Absolute tracking-only correction allowed on any one Z motor.
        # Start conservatively; this is measured relative to the QGL baseline.
        self.max_motor_offset = config.getfloat(
            'max_motor_offset', 0.20, above=0.)
        self.max_nozzle_differential = config.getfloat(
            'max_nozzle_differential', 1.0, above=0.)

        # Runtime T1-vs-T0 first-layer trim.
        # Added to the calibrated mechanical delta and NOT mesh-faded.
        self.max_t1_live_trim = config.getfloat(
            'max_t1_live_trim', 0.50, minval=0.)

        # Additional segmentation so T1's mesh is sampled even if normal
        # bed_mesh does not need to split a particular T0 move.
        self.segment_length = config.getfloat(
            'segment_length', 3.0, above=0.)

        # Individual Z-motor motion limits. These limit the complete XYZ move
        # through Klipper lookahead instead of allowing the correction to lag.
        self.max_motor_velocity = config.getfloat(
            'max_motor_velocity', 5.0, above=0.)
        self.max_motor_accel = config.getfloat(
            'max_motor_accel', 100.0, above=0.)
        self.motor_corner_velocity = config.getfloat(
            'motor_corner_velocity', 0.10, minval=0.)

        self.mesh_margin = config.getfloat('mesh_margin', 0.01, minval=0.)

        # Allow TAM to use Klipper's mesh surface slightly beyond the
        # physically probed rectangle. This is needed because the printable
        # build plate extends beyond the aluminum region visible to the Eddy.
        self.mesh_extrapolation = config.getfloat(
            'mesh_extrapolation', 0.0, minval=0.)

        self.printable_min_x = config.getfloat(
            'printable_min_x', 0.0)
        self.printable_max_x = config.getfloat(
            'printable_max_x', 806.0)
        self.printable_min_y = config.getfloat(
            'printable_min_y', 0.0)
        self.printable_max_y = config.getfloat(
            'printable_max_y', 800.0)

        self.toolhead = None
        self.bed_mesh = None
        self.dual_carriage = None
        self.calibration = None
        self.t0_dc = None
        self.t1_dc = None
        self.z_steppers = []
        self.z_min = -999999.0
        self.z_max = 999999.0

        self.enabled = False
        self.baseline_dirty = False
        self.motor_axes = []
        self.motor_axis_indices = []
        self.proxy = None
        self.last_offsets = [0., 0., 0., 0.]
        self.last_eval = None
        self.disable_reason = ''
        self.t1_live_trim = 0.0
        self.tool_t0_trim = 0.0
        self.tool_t1_trim = 0.0
        self.tool_common_compensation = 0.0
        self.tracking_suspended = False
        self.suspended_offsets = [0.0, 0.0, 0.0, 0.0]
        self.wrapped_commands = {}
        self.status_timer = self.reactor.register_timer(
            self._motor_status_timer)

        # Optional continuous BFP_DUAL_TRACKING_STATUS streaming, started or
        # stopped at runtime with the INTERVAL= parameter (0 disables it).
        self.status_stream_interval = 0.0
        self.status_stream_timer = self.reactor.register_timer(
            self._status_stream_timer)

        self.printer.register_event_handler(
            'klippy:connect', self._handle_connect)
        self.printer.register_event_handler(
            'klippy:ready', self._handle_ready)
        self.printer.register_event_handler(
            'klippy:shutdown', self._handle_shutdown)

        self.gcode.register_command(
            'BFP_DUAL_TRACKING', self.cmd_BFP_DUAL_TRACKING,
            desc='Enable or disable active BFP dual-nozzle Z tracking')
        self.gcode.register_command(
            'BFP_DUAL_TRACKING_STATUS', self.cmd_BFP_DUAL_TRACKING_STATUS,
            desc='Report BFP dual-nozzle tracking state and current solution. '
                 'Add INTERVAL=<seconds> to stream this report repeatedly '
                 '(INTERVAL=0 stops streaming).')


        self.gcode.register_command(
            'BFP_SECONDARY_TRIM', self.cmd_BFP_T1_TRIM,
            desc='Set, adjust, or reset live T1-vs-T0 Z trim')


        self.gcode.register_command(
            'BFP_TRACKING_SUSPEND',
            self.cmd_BFP_TRACKING_SUSPEND,
            desc='Suspend TAM while preserving current Z motor differential')
        self.gcode.register_command(
            'BFP_TRACKING_RESUME',
            self.cmd_BFP_TRACKING_RESUME,
            desc='Resume TAM from a preserved suspended differential')

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.bed_mesh = self.printer.lookup_object('bed_mesh')
        self.dual_carriage = self.printer.lookup_object('dual_carriage')
        self.calibration = self.printer.lookup_object(
            self.calibration_object_name)

        dc_rails = self.dual_carriage.dc_rails
        t0_name = self._resolve_carriage_name(
            dc_rails, self.t0_carriage_name, ('x', 'stepper_x'))
        t1_name = self._resolve_carriage_name(
            dc_rails, self.t1_carriage_name, ('dual_carriage',))
        if t0_name is None:
            raise self.printer.config_error(
                "bfp_dual_tracking: no T0 rail '%s' (available: %s)" %
                (self.t0_carriage_name, ', '.join(sorted(dc_rails.keys()))))
        if t1_name is None:
            raise self.printer.config_error(
                "bfp_dual_tracking: no T1 rail '%s' (available: %s)" %
                (self.t1_carriage_name, ', '.join(sorted(dc_rails.keys()))))
        self.t0_carriage_name = t0_name
        self.t1_carriage_name = t1_name
        self.t0_dc = dc_rails[t0_name]
        self.t1_dc = dc_rails[t1_name]

        expected = ['stepper_z', 'stepper_z1', 'stepper_z2', 'stepper_z3']
        by_name = {
            s.get_name(): s
            for s in self.toolhead.get_kinematics().get_steppers()
        }
        missing = [name for name in expected if name not in by_name]
        if missing:
            raise self.printer.config_error(
                'bfp_dual_tracking: missing Z stepper(s): %s' %
                (', '.join(missing),))
        self.z_steppers = [by_name[name] for name in expected]

        kin = self.toolhead.get_kinematics()
        try:
            self.z_min, self.z_max = kin.rails[2].get_range()
        except Exception:
            logging.warning(
                'bfp_dual_tracking: unable to read Z rail range; using wide fallback')

    def _handle_ready(self):
        # Any command that changes homing, QGL, mesh, IDEX kinematics, or input
        # shaping must not run while the four Z motors are detached from the
        # normal Z trapq. Auto-disable first. QGL also establishes a clean
        # baseline again after it succeeds.
        self._wrap_command('G28', clear_baseline=False)
        self._wrap_command('QUAD_GANTRY_LEVEL', clear_baseline=True)
        self._wrap_command('SET_DUAL_CARRIAGE', clear_baseline=False)
        self._wrap_command('SET_INPUT_SHAPER', clear_baseline=False)
        self._wrap_command('BED_MESH_CLEAR', clear_baseline=False)
        self._wrap_command('BED_MESH_CALIBRATE', clear_baseline=False)
        self._wrap_command('BED_MESH_PROFILE', clear_baseline=False)
        self._wrap_command('BED_MESH_OFFSET', clear_baseline=False)
        self._wrap_command('PROBE_CALIBRATE', clear_baseline=False)
        self._wrap_command('BFP_SPLIT_Z_TEST', clear_baseline=False)
        self._install_tool_offset_probe()
        if self.motor_status_interval > 0.0:
            self.reactor.update_timer(
                self.status_timer,
                self.reactor.monotonic() + self.motor_status_interval)


    def _install_tool_offset_probe(self):
        # Retain Klipper's complete existing SET_GCODE_OFFSET handler.
        original = self.gcode.register_command(
            'SET_GCODE_OFFSET', None
        )

        if original is None:
            raise self.printer.config_error(
                'BFP: native SET_GCODE_OFFSET handler unavailable'
            )

        self._native_set_gcode_offset = original

        self.gcode.register_command(
            'SET_GCODE_OFFSET',
            self.cmd_BFP_SET_GCODE_OFFSET_PROBE,
            desc='Native Z offset or BFP tool-specific offset test'
        )


    def cmd_BFP_SET_GCODE_OFFSET_PROBE(self, gcmd):
        tool = gcmd.get('TOOL', None)

        # No TOOL: original, unmodified Klipper behavior.
        if tool is None:
            return self._native_set_gcode_offset(gcmd)

        tool = tool.upper()

        if tool not in ('T0', 'T1'):
            raise gcmd.error('TOOL must be T0 or T1')

        if not self.enabled or self.tracking_suspended:
            raise gcmd.error(
                'Tool-specific Z adjustment requires active TAM'
            )

        if gcmd.get('Z', None) is not None:
            raise gcmd.error(
                'Test implementation supports Z_ADJUST only'
            )

        for axis in ('X', 'Y', 'E', 'X_ADJUST',
                     'Y_ADJUST', 'E_ADJUST'):
            if gcmd.get(axis, None) is not None:
                raise gcmd.error(
                    'Tool-specific test accepts Z_ADJUST only'
                )

        delta = gcmd.get_float('Z_ADJUST', None)

        if delta is None:
            raise gcmd.error('Z_ADJUST is required')

        if abs(delta) > 0.0500001:
            raise gcmd.error(
                'Test adjustment limited to +/-0.05 mm per command'
            )

        if gcmd.get_int('MOVE', 0) != 1:
            raise gcmd.error(
                'Tool-specific adjustment requires MOVE=1'
            )

        old_relative = self.t1_live_trim
        old_comp = self.tool_common_compensation
        old_t0 = self.tool_t0_trim
        old_t1 = self.tool_t1_trim

        # Read the existing physical solution before changing state.
        current = list(self.toolhead.get_position())
        old_eval = self._evaluate(current)

        if tool == 'T0':
            # Native Klipper moves common Z by delta.
            # Opposite differential holds T1 at its old height.
            new_relative = old_relative - delta
            new_comp = old_comp
            new_t0 = old_t0 + delta
            new_t1 = old_t1
        else:
            # Native Klipper moves common Z by delta.
            # Cancel common movement at T0, retaining it at T1.
            new_relative = old_relative + delta
            new_comp = old_comp - delta
            new_t0 = old_t0
            new_t1 = old_t1 + delta

        if (abs(new_t0) > 0.5000001
                or abs(new_t1) > 0.5000001
                or abs(new_relative) > self.max_t1_live_trim):
            raise gcmd.error(
                'Tool-specific trim exceeds configured 0.50 mm limit'
            )

        # Temporarily install the new solution for validation.
        self.t1_live_trim = new_relative
        self.tool_common_compensation = new_comp

        try:
            target = list(current)
            target[2] += delta

            new_eval = self._evaluate(target)

            # Calculate the actual predicted physical displacement
            # at both nozzles from all four motor corrections.
            motor_changes = [
                delta + new_eval['offsets'][i]
                      - old_eval['offsets'][i]
                for i in range(4)
            ]

            t0_motion = self._dot(
                new_eval['weights_t0'], motor_changes
            )
            t1_motion = self._dot(
                new_eval['weights_t1'], motor_changes
            )

            expected_t0 = delta if tool == 'T0' else 0.0
            expected_t1 = delta if tool == 'T1' else 0.0

            tolerance = 0.003

            if (abs(t0_motion - expected_t0) > tolerance
                    or abs(t1_motion - expected_t1) > tolerance):
                raise gcmd.error(
                    'Tool Z adjustment rejected: predicted '
                    'T0=%+.5f T1=%+.5f mm; '
                    'requested T0=%+.5f T1=%+.5f mm'
                    % (
                        t0_motion, t1_motion,
                        expected_t0, expected_t1
                    )
                )

            # Delegate the real physical move to native Klipper.
            # This is the same MOVE=1 path used by Mainsail.
            self._native_set_gcode_offset(gcmd)

        except Exception:
            self.t1_live_trim = old_relative
            self.tool_common_compensation = old_comp
            raise

        self.tool_t0_trim = new_t0
        self.tool_t1_trim = new_t1

        gcmd.respond_info(
            'BFP INDEPENDENT TOOL OFFSET\\n'
            'Selected: %s\\n'
            'Logical trims: T0=%+.4f T1=%+.4f mm\\n'
            'Predicted motion: T0=%+.5f T1=%+.5f mm\\n'
            'Native Klipper MOVE=1 queued'
            % (
                tool, new_t0, new_t1,
                t0_motion, t1_motion
            )
        )


    def _handle_shutdown(self):
        self.enabled = False
        self.status_stream_interval = 0.0
        self.reactor.update_timer(self.status_stream_timer, self.reactor.NEVER)

    @staticmethod
    def _resolve_carriage_name(rails, requested, aliases):
        if requested in rails:
            return requested
        for name in aliases:
            if name in rails:
                return name
        return None

    def _motor_positions(self):
        if self.enabled and self.motor_axes:
            return [axis.commanded_pos for axis in self.motor_axes]
        return [stepper.get_commanded_position() for stepper in self.z_steppers]

    def _motor_status_timer(self, eventtime):
        if self.enabled and len(self.z_steppers) == 4:
            pos = self._motor_positions()
            self.gcode.respond_info(
                'BFP tracking Z positions: '
                'Z0(FL)=%.6f Z1(RL)=%.6f Z2(RR)=%.6f Z3(FR)=%.6f' %
                tuple(pos))
        if self.motor_status_interval <= 0.0:
            return self.reactor.NEVER
        return eventtime + self.motor_status_interval

    def _wrap_command(self, command, clear_baseline=False):
        old_handler = self.gcode.register_command(command, None)
        if old_handler is None:
            return
        self.wrapped_commands[command] = old_handler

        def wrapped(gcmd):
            if self.enabled:
                self._disable_internal(
                    'auto-disabled before %s' % (command,))
            elif (
                self.tracking_suspended
                and command in ('G28', 'QUAD_GANTRY_LEVEL')
            ):
                # Homing/QGL deliberately establishes new machine geometry.
                # Any paused TAM snapshot is no longer valid.
                self.tracking_suspended = False
                self.suspended_offsets = [0.0, 0.0, 0.0, 0.0]
                self.disable_reason = (
                    'suspended state cleared before %s' % command
                )

            result = old_handler(gcmd)
            if clear_baseline:
                self.baseline_dirty = False
            return result

        self.gcode.register_command(command, wrapped)

    def _get_mechanical_delta(self):
        # Current bfp_dual_calibration exposes the saved value as an attribute.
        # Keep status-key fallbacks so this tracking module remains tolerant of
        # small calibration-module refactors.
        if hasattr(self.calibration, 't1_z_delta'):
            return float(self.calibration.t1_z_delta)
        if hasattr(self.calibration, 'get_t1_z_delta'):
            return float(self.calibration.get_t1_z_delta())
        if hasattr(self.calibration, 'get_status'):
            status = self.calibration.get_status(self.reactor.monotonic())
            for key in ('t1_z_delta', 'mechanical_delta', 'delta'):
                if key in status and status[key] is not None:
                    return float(status[key])
        raise self.printer.command_error(
            'BFP dual tracking cannot read t1_z_delta from %s' %
            (self.calibration_object_name,))

    def _carriage_mode_ok(self):
        return (
            self.t0_dc.mode == 'PRIMARY'
            and self.t1_dc.mode in ('COPY', 'MIRROR')
        )

    def _carriage_xy(self, pos):
        x0 = self.t0_dc.get_axis_position(pos)
        x1 = self.t1_dc.get_axis_position(pos)
        return x0, x1, pos[1]

    def _mesh_inside(self, mesh, x, y):
        params = mesh.get_mesh_params()
        mx = getattr(mesh, 'mesh_offsets', (0., 0.))

        xx = x + mx[0]
        yy = y + mx[1]

        # Absolute printable-area guard.
        if not (
            self.printable_min_x <= xx <= self.printable_max_x
            and self.printable_min_y <= yy <= self.printable_max_y
        ):
            return False

        # Permit controlled extrapolation outside the measured mesh.
        e = self.mesh_extrapolation
        m = self.mesh_margin

        return (
            params['min_x'] - e - m <= xx <= params['max_x'] + e + m
            and params['min_y'] - e - m <= yy <= params['max_y'] + e + m
        )

    def _weights(self, x, y):
        return bfp_dual_mesh_math.gantry_weights(
            x, y, self.left_x, self.right_x, self.front_y, self.rear_y)

    @staticmethod
    def _dot(a, b):
        return sum(x * y for x, y in zip(a, b))

    def _fade_factor_from_post_mesh_z(self, mesh, x, y, post_z, mesh_z):
        # This mirrors BedMesh.get_position()'s inverse fade calculation so
        # the tracking differential uses the same factor as normal bed_mesh.
        bm = self.bed_mesh
        fade_z_pos = post_z + bm.tool_offset
        if min(fade_z_pos, fade_z_pos - mesh_z) >= bm.fade_end:
            return 0.0
        if max(fade_z_pos, fade_z_pos - mesh_z) >= bm.fade_start:
            z_adj = mesh_z - bm.fade_target
            denom = bm.fade_dist - z_adj
            if abs(denom) < 1.0e-12:
                return 0.0
            factor = (
                (bm.fade_end + bm.fade_target - fade_z_pos) / denom
            )
            return min(1.0, max(0.0, factor))
        return 1.0

    def _solve_offsets(self, w0, w1, requested_d):
        try:
            return bfp_dual_mesh_math.solve_offsets(
                w0, w1, requested_d, self.authority_lambda,
                self.min_condition_q, self.max_motor_offset)
        except ValueError as e:
            raise self.printer.command_error(str(e))

    def _evaluate(self, pos):
        if not self._carriage_mode_ok():
            raise self.printer.command_error(
                'BFP dual tracking requires T0=PRIMARY and T1=COPY or MIRROR')

        mesh = self.bed_mesh.get_mesh()
        if mesh is None:
            raise self.printer.command_error(
                'BFP dual tracking requires an active bed mesh')

        x0, x1, y = self._carriage_xy(pos)

        if not self._mesh_inside(mesh, x0, y):
            raise self.printer.command_error(
                'BFP dual tracking: T0 (%.3f, %.3f) is outside active mesh' %
                (x0, y))

        if not self._mesh_inside(mesh, x1, y):
            raise self.printer.command_error(
                'BFP dual tracking: T1 (%.3f, %.3f) is outside active mesh' %
                (x1, y))

        m0 = mesh.calc_z(x0, y)
        m1 = mesh.calc_z(x1, y)

        factor = self._fade_factor_from_post_mesh_z(
            mesh, x0, y, pos[2], m0)

        mech = self._get_mechanical_delta()

        requested_d = mech + self.t1_live_trim + factor * (m1 - m0)

        if abs(requested_d) > self.max_nozzle_differential:
            raise self.printer.command_error(
                'BFP dual tracking: required nozzle differential %.4f exceeds '
                'limit %.4f mm' %
                (requested_d, self.max_nozzle_differential))

        w0 = self._weights(x0, y)
        w1 = self._weights(x1, y)

        # Diagnostic conditioning check with geometry information.
        # Mirrors bfp_dual_mesh_math.solve_offsets().
        sw0 = [w0[0] + w0[1], w0[2] + w0[3]]
        sw1 = [w1[0] + w1[1], w1[2] + w1[3]]

        sw0_norm2 = self._dot(sw0, sw0)

        if sw0_norm2 > 1.0e-15:
            projection = self._dot(sw1, sw0) / sw0_norm2
            p_condition = [
                sw1[i] - projection * sw0[i]
                for i in range(2)
            ]
            diagnostic_q = self._dot(p_condition, p_condition)

            if (
                abs(requested_d) >= 1.0e-12
                and diagnostic_q < self.min_condition_q
            ):
                raise self.printer.command_error(
                    'BFP dual tracking: geometry too poorly conditioned '
                    '(q=%.6f < %.6f).\n'
                    'T0 X=%.3f  T1 X=%.3f  Y=%.3f\n'
                    'Nozzle separation=%.3f mm\n'
                    'Requested T1-T0 D=%+.6f mm'
                    % (
                        diagnostic_q,
                        self.min_condition_q,
                        x0,
                        x1,
                        y,
                        abs(x1 - x0),
                        requested_d
                    )
                )

        try:
            (
                offsets,
                q,
                achieved_d,
                residual,
                authority,
                gain,
                uncapped_max,
                hard_scale
            ) = self._solve_offsets(w0, w1, requested_d)

        except Exception as e:
            msg = str(e)

            if 'poorly conditioned' in msg:
                raise self.printer.command_error(
                    '%s\n'
                    'T0 X=%.3f  T1 X=%.3f  Y=%.3f\n'
                    'Nozzle separation=%.3f mm\n'
                    'Requested T1-T0 D=%+.6f mm'
                    % (
                        msg,
                        x0,
                        x1,
                        y,
                        abs(x1 - x0),
                        requested_d
                    )
                )

            raise

        # Common compensation for native TOOL=T1 Z adjustments.
        # The relative solver still defines T1-T0 separation.
        offsets = [
            value + self.tool_common_compensation
            for value in offsets
        ]

        if max(abs(v) for v in offsets) > self.max_motor_offset:
            raise self.printer.command_error(
                'BFP tool offset: motor correction exceeds %.4f mm'
                % self.max_motor_offset
            )

        # TAM safety: never accept a clamped solution or one that cannot
        # attain the desired differential within the configured tolerance.
        # Run after the original solver, before any motor move is queued.
        if not all(math.isfinite(v) for v in (
                q, achieved_d, residual, uncapped_max, hard_scale,
                *offsets)):
            raise self.printer.command_error(
                'BFP TAM: non-finite constrained solution at '
                'T0 X=%.3f T1 X=%.3f Y=%.3f' % (x0, x1, y))
        # Permit degraded tracking using the solver's bounded offsets.
        # Actual motor displacement limits remain enforced.
        if any(abs(v) > self.max_motor_offset + 1.0e-6
               for v in offsets):
            raise self.printer.command_error(
                'BFP TAM: final motor offset exceeds configured limit')

        degraded = (
            hard_scale < 1.0 - 1.0e-9 or
            uncapped_max > self.max_motor_offset + 1.0e-6 or
            abs(residual) > self.max_tracking_residual)

        was_degraded = getattr(self, '_tam_degraded_last', False)

        if degraded and not was_degraded:
            self.gcode.respond_info(
                'BFP TAM DEGRADED: motor scale %.3f, residual %.4f mm; '
                'T0 X=%.3f T1 X=%.3f. Printing continues with bounded '
                'Z corrections.' % (hard_scale, residual, x0, x1))
            self._tam_degraded_last = True

        elif (was_degraded and not degraded and
              abs(residual) < 0.75 * self.max_tracking_residual):
            self.gcode.respond_info(
                'BFP TAM full tracking restored; residual %.4f mm.'
                % residual)
            self._tam_degraded_last = False

        # Account for intentional common Z compensation.
        # Validate the T0 differential anchor, not the common shift.
        t0_anchor_error = self._dot(
            w0, [v - self.tool_common_compensation for v in offsets])
        if abs(t0_anchor_error) > self.max_tracking_residual:
            raise self.printer.command_error(
                'BFP TAM: T0 anchor error exceeds allowed tolerance '
                '(T0 X=%.3f T1 X=%.3f Y=%.3f)' % (x0, x1, y))

        return {
            'mode': self.t1_dc.mode,
            'x0': x0,
            'x1': x1,
            'y': y,
            'separation': abs(x1 - x0),
            'mesh_t0': m0,
            'mesh_t1': m1,
            'mesh_diff': m1 - m0,
            'fade': factor,
            'mechanical_delta': mech,
            'live_trim': self.t1_live_trim,
            'requested_d': requested_d,
            'weights_t0': w0,
            'weights_t1': w1,
            'conditioning_q': q,
            'offsets': offsets,
            'achieved_d': achieved_d,
            'residual': residual,
            'authority': authority,
            'gain': gain,
            'uncapped_max': uncapped_max,
            'hard_scale': hard_scale,
        }

    def _tracking_move(self, newpos, speed):
        if not self.enabled:
            return self.toolhead.move(newpos, speed)

        start = self.toolhead.get_position()
        target = list(newpos)

        if len(target) < len(start):
            target += start[len(target):]
        elif len(target) > len(start):
            raise self.printer.command_error(
                'BFP dual tracking: unexpected toolhead axis count')

        dx = target[0] - start[0]
        dy = target[1] - start[1]
        dz = target[2] - start[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)

        segments = max(
            1,
            int(math.ceil(distance / self.segment_length))
        )

        # Plan every sub-segment first. If any safety check fails, no partial
        # tracking move is queued.
        planned = []

        for seg in range(1, segments + 1):
            t = float(seg) / float(segments)

            pos = [
                start[i] + (target[i] - start[i]) * t
                for i in range(len(start))
            ]

            ev = self._evaluate(pos)

            for i, axis_index in enumerate(self.motor_axis_indices):
                motor_target = pos[2] + ev['offsets'][i]

                if motor_target < self.z_min or motor_target > self.z_max:
                    raise self.printer.command_error(
                        'BFP dual tracking: %s target %.4f outside Z range '
                        '[%.4f, %.4f]' %
                        (
                            self.z_steppers[i].get_name(),
                            motor_target,
                            self.z_min,
                            self.z_max
                        )
                    )

                pos[axis_index] = motor_target

            planned.append((pos, ev))

        for pos, ev in planned:
            self.toolhead.move(pos, speed)

            self.last_eval = ev
            self.last_offsets = list(ev['offsets'])

            if max(abs(v) for v in self.last_offsets) > 1.0e-6:
                self.baseline_dirty = True

    def _enable_internal(self):
        if self.enabled:
            return

        if not self.motor_map_verified:
            raise self.printer.command_error(
                'BFP dual tracking motor map is not verified. Confirm '
                'Z0=FL, Z1=RL, Z2=RR, Z3=FR, then set '
                'motor_map_verified: True in [bfp_dual_mesh].')

        if self.baseline_dirty:
            raise self.printer.command_error(
                'BFP dual tracking baseline is dirty. Run QUAD_GANTRY_LEVEL '
                'before re-enabling tracking.')

        eventtime = self.reactor.monotonic()
        kin_status = self.toolhead.get_kinematics().get_status(eventtime)

        if 'z' not in kin_status.get('homed_axes', ''):
            raise self.printer.command_error(
                'BFP dual tracking requires Z to be homed')

        if not self._carriage_mode_ok():
            raise self.printer.command_error(
                'Activate COPY or MIRROR mode with T0 primary before tracking')

        if self.bed_mesh.get_mesh() is None:
            raise self.printer.command_error(
                'Load a bed mesh before enabling BFP dual tracking')

        # Require the current position itself to be valid before changing any
        # kinematics. This keeps the first active correction deterministic.
        current = self.toolhead.get_position()
        self._evaluate(current)

        self.toolhead.flush_step_generation()
        self.toolhead.wait_moves()

        current = self.toolhead.get_position()
        start_z = current[2]

        axes = []

        try:
            for i, stepper in enumerate(self.z_steppers):
                axes.append(
                    _BFPMotorAxis(
                        self,
                        stepper,
                        i,
                        start_z
                    )
                )

            self.motor_axes = axes
            self.motor_axis_indices = []

            for axis in self.motor_axes:
                idx = len(self.toolhead.get_position())

                self.toolhead.add_extra_axis(
                    axis,
                    start_z
                )

                self.motor_axis_indices.append(idx)

            self.proxy = _BFPBedMeshToolheadProxy(
                self,
                self.toolhead
            )

            self.bed_mesh.toolhead = self.proxy

            self.last_offsets = [0., 0., 0., 0.]

            self.last_eval = self._evaluate(
                self.toolhead.get_position()
            )

            self.enabled = True
            self.disable_reason = ''

            self.printer.lookup_object(
                'gcode_move'
            ).reset_last_position()

            # Resume completed successfully. The frozen snapshot is no
            # longer needed; live TAM state is now held in last_offsets.
            self.suspended_offsets = [0.0, 0.0, 0.0, 0.0]

        except Exception:
            logging.exception(
                'BFP dual tracking enable failed; restoring Z'
            )

            self.bed_mesh.toolhead = self.toolhead

            for axis in reversed(self.motor_axes):
                try:
                    self.toolhead.remove_extra_axis(axis)
                except Exception:
                    logging.exception(
                        'BFP tracking rollback remove axis'
                    )

            for axis in axes:
                try:
                    axis.restore(current)
                except Exception:
                    logging.exception(
                        'BFP tracking rollback restore stepper'
                    )

            self.motor_axes = []
            self.motor_axis_indices = []
            self.proxy = None
            self.enabled = False

            raise

    def _suspend_internal(self):
        if self.tracking_suspended:
            return

        if not self.enabled:
            raise self.printer.command_error(
                'BFP tracking cannot suspend because TAM is not enabled'
            )

        self.toolhead.flush_step_generation()
        self.toolhead.wait_moves()

        current = self.toolhead.get_position()

        if len(self.motor_axes) != 4:
            raise self.printer.command_error(
                'BFP tracking suspend expected four active Z motor axes'
            )

        # Capture the REAL commanded differential relative to the current
        # logical Z. Do not rely only on last_eval because we want the
        # exact position each extra motor axis currently holds.
        frozen = [
            axis.commanded_pos - current[2]
            for axis in self.motor_axes
        ]

        # Stop interception before restoring normal Klipper Z kinematics.
        self.enabled = False
        self.tracking_suspended = True
        self.suspended_offsets = list(frozen)
        self.disable_reason = 'TAM suspended for pause'

        self.bed_mesh.toolhead = self.toolhead

        xyz = current[:3]

        for axis in reversed(self.motor_axes):
            self.toolhead.remove_extra_axis(axis)

        # Restoring normal Z kinematics does NOT physically level the gantry.
        # The four motors remain where TAM left them. Subsequent ordinary Z
        # moves therefore move all four equally and preserve this differential.
        for axis in self.motor_axes:
            axis.restore(xyz)

        self.motor_axes = []
        self.motor_axis_indices = []
        self.proxy = None

        self.printer.lookup_object(
            'gcode_move'
        ).reset_last_position()


    def _resume_suspended_internal(self):
        if self.enabled:
            return

        if not self.tracking_suspended:
            raise self.printer.command_error(
                'BFP tracking is not suspended'
            )

        if not self.motor_map_verified:
            raise self.printer.command_error(
                'BFP dual tracking motor map is not verified'
            )

        eventtime = self.reactor.monotonic()
        kin_status = self.toolhead.get_kinematics().get_status(eventtime)

        if 'z' not in kin_status.get('homed_axes', ''):
            raise self.printer.command_error(
                'BFP tracking resume requires Z to remain homed'
            )

        if not self._carriage_mode_ok():
            raise self.printer.command_error(
                'Restore COPY or MIRROR mode with T0 primary before TAM resume'
            )

        if self.bed_mesh.get_mesh() is None:
            raise self.printer.command_error(
                'BFP tracking resume requires the original active bed mesh'
            )

        if len(self.suspended_offsets) != 4:
            raise self.printer.command_error(
                'BFP tracking resume has invalid suspended motor offsets'
            )

        # The toolheads must already be back at the saved print location
        # before this is called. Validate that TAM can evaluate that location.
        current = self.toolhead.get_position()
        self._evaluate(current)

        self.toolhead.flush_step_generation()
        self.toolhead.wait_moves()

        current = self.toolhead.get_position()
        start_z = current[2]

        # Critical difference from a normal enable:
        #
        # A normal enable starts all four extra axes at start_z because QGL
        # established a clean baseline.
        #
        # A suspended resume starts each extra axis at:
        #
        #     current Z + its frozen TAM correction
        #
        # because that differential is still physically present.
        starts = [
            start_z + offset
            for offset in self.suspended_offsets
        ]

        axes = []

        try:
            for i, stepper in enumerate(self.z_steppers):
                axes.append(
                    _BFPMotorAxis(
                        self,
                        stepper,
                        i,
                        starts[i]
                    )
                )

            self.motor_axes = axes
            self.motor_axis_indices = []

            for i, axis in enumerate(self.motor_axes):
                idx = len(self.toolhead.get_position())

                self.toolhead.add_extra_axis(
                    axis,
                    starts[i]
                )

                self.motor_axis_indices.append(idx)

            self.proxy = _BFPBedMeshToolheadProxy(
                self,
                self.toolhead
            )

            self.bed_mesh.toolhead = self.proxy

            # Preserve the exact frozen solution as the starting point.
            self.last_offsets = list(self.suspended_offsets)

            # Record what TAM currently wants at this XY/Z. We do NOT apply
            # it instantaneously. The normal segmented tracking move will
            # transition from the frozen correction to the live solution.
            self.last_eval = self._evaluate(
                self.toolhead.get_position()
            )

            self.enabled = True
            self.tracking_suspended = False
            self.disable_reason = ''

            self.printer.lookup_object(
                'gcode_move'
            ).reset_last_position()

        except Exception:
            logging.exception(
                'BFP suspended tracking resume failed; restoring normal Z'
            )

            self.bed_mesh.toolhead = self.toolhead

            for axis in reversed(self.motor_axes):
                try:
                    self.toolhead.remove_extra_axis(axis)
                except Exception:
                    logging.exception(
                        'BFP suspended resume rollback remove axis'
                    )

            for axis in axes:
                try:
                    axis.restore(current)
                except Exception:
                    logging.exception(
                        'BFP suspended resume rollback restore stepper'
                    )

            self.motor_axes = []
            self.motor_axis_indices = []
            self.proxy = None
            self.enabled = False

            # Keep the snapshot alive so resume can be retried.
            self.tracking_suspended = True

            raise


    def cmd_BFP_TRACKING_SUSPEND(self, gcmd):
        self._suspend_internal()

        gcmd.respond_info(
            'BFP TAM SUSPENDED\n'
            'Frozen motor offsets: '
            'Z0=%+.6f Z1=%+.6f Z2=%+.6f Z3=%+.6f\n'
            'Normal synchronized Z movement may now be used.'
            % tuple(self.suspended_offsets)
        )


    def cmd_BFP_TRACKING_RESUME(self, gcmd):
        frozen = list(self.suspended_offsets)

        self._resume_suspended_internal()

        gcmd.respond_info(
            'BFP TAM RESUMED\n'
            'Restored motor offsets: '
            'Z0=%+.6f Z1=%+.6f Z2=%+.6f Z3=%+.6f'
            % tuple(frozen)
        )


    def _disable_internal(self, reason='user request'):
        if self.tracking_suspended:
            self.tracking_suspended = False
            self.suspended_offsets = [0.0, 0.0, 0.0, 0.0]
            self.disable_reason = reason
            return

        if not self.enabled:
            self.disable_reason = reason
            return

        # Stop future interception first, flush all queued tracking moves, then
        # restore Klipper's original Z solvers/trapq. Existing differential
        # gantry position is intentionally preserved physically; therefore the
        # baseline is marked dirty and QGL is required before another enable.
        self.enabled = False
        self.disable_reason = reason
        self.bed_mesh.toolhead = self.toolhead

        self.toolhead.flush_step_generation()
        self.toolhead.wait_moves()

        xyz = self.toolhead.get_position()[:3]

        for axis in reversed(self.motor_axes):
            self.toolhead.remove_extra_axis(axis)

        for axis in self.motor_axes:
            axis.restore(xyz)

        self.motor_axes = []
        self.motor_axis_indices = []
        self.proxy = None

        self.printer.lookup_object(
            'gcode_move'
        ).reset_last_position()

    def cmd_BFP_DUAL_TRACKING(self, gcmd):
        enable = gcmd.get_int(
            'ENABLE',
            None,
            minval=0,
            maxval=1
        )

        if enable is None:
            raise gcmd.error(
                'Use BFP_DUAL_TRACKING ENABLE=0 or ENABLE=1'
            )

        if enable:
            # A genuine PAUSE/RESUME carries a frozen four-motor TAM
            # differential. In that case the dirty baseline is expected and
            # must NOT force a QGL. Restore from the suspended snapshot instead
            # of using the normal clean-baseline enable path.
            if self.tracking_suspended:
                self._resume_suspended_internal()

                gcmd.respond_info(
                    'BFP dual nozzle mesh tracking RESUMED from suspended state\n'
                    'Dirty baseline intentionally preserved for pause/resume.\n'
                    'Frozen Z-motor differential restored.'
                )

            else:
                # Normal enable remains protected by baseline_dirty.
                self._enable_internal()

                gcmd.respond_info(
                    'BFP dual nozzle mesh tracking ENABLED\n'
                    'T0 remains the normal Klipper bed_mesh reference.\n'
                    'T1 differential correction is ACTIVE on Z0/Z1/Z2/Z3.'
                )

        else:
            self._disable_internal('user request')

            gcmd.respond_info(
                'BFP dual nozzle mesh tracking DISABLED\n'
                'Run QUAD_GANTRY_LEVEL before enabling it again if tracking moved the gantry.'
            )

    def cmd_BFP_T1_TRIM(self, gcmd):
        reset = gcmd.get_int('RESET', 0, minval=0, maxval=1)
        set_value = gcmd.get_float('SET', None)
        adjust = gcmd.get_float('ADJUST', None)

        supplied = (
            (1 if reset else 0)
            + (1 if set_value is not None else 0)
            + (1 if adjust is not None else 0)
        )

        if supplied != 1:
            raise gcmd.error(
                'Use BFP_SECONDARY_TRIM ADJUST=<mm>, SET=<mm>, or RESET=1'
            )

        old = self.t1_live_trim

        if reset:
            new = 0.0
        elif set_value is not None:
            new = set_value
        else:
            new = old + adjust

        if abs(new) > self.max_t1_live_trim:
            raise gcmd.error(
                'T1 live trim %+.4f exceeds safety limit +/-%.4f mm'
                % (new, self.max_t1_live_trim)
            )

        self.t1_live_trim = new

        mech = self._get_mechanical_delta()

        gcmd.respond_info(
            'BFP T1 LIVE TRIM\n'
            'Previous trim: %+.4f mm\n'
            'Current trim:  %+.4f mm\n'
            'Mechanical T1-T0: %+.4f mm\n'
            'Effective baseline: %+.4f mm'
            % (
                old,
                self.t1_live_trim,
                mech,
                mech + self.t1_live_trim
            )
        )

    def _status_eval(self):
        if self.bed_mesh is None or self.bed_mesh.get_mesh() is None:
            return None

        try:
            return self._evaluate(
                self.toolhead.get_position()
            )
        except Exception as e:
            return {'error': str(e)}

    def _status_stream_timer(self, eventtime):
        if self.status_stream_interval <= 0.0:
            return self.reactor.NEVER
        self.gcode.respond_info('\n'.join(self._build_status_lines()))
        return eventtime + self.status_stream_interval

    def cmd_BFP_DUAL_TRACKING_STATUS(self, gcmd):
        interval = gcmd.get_float('INTERVAL', None, minval=0.)
        if interval is not None:
            self.status_stream_interval = interval
            if interval > 0.0:
                self.reactor.update_timer(
                    self.status_stream_timer, self.reactor.NOW)
                gcmd.respond_info(
                    'BFP_DUAL_TRACKING_STATUS streaming every %.3f s. '
                    'Send BFP_DUAL_TRACKING_STATUS INTERVAL=0 to stop.' %
                    (interval,))
            else:
                self.reactor.update_timer(
                    self.status_stream_timer, self.reactor.NEVER)
                gcmd.respond_info('BFP_DUAL_TRACKING_STATUS streaming stopped.')
            return

        gcmd.respond_info('\n'.join(self._build_status_lines()))

    def _build_status_lines(self):
        ev = self._status_eval()

        lines = [
            'BFP Dual Nozzle Mesh Tracking',
            'ACTIVE MOTOR CONTROL: %s' %
            ('ENABLED' if self.enabled else 'DISABLED'),
            'TAM suspended: %s' %
            ('YES' if self.tracking_suspended else 'NO'),
            'Suspended offsets: Z0=%+.6f Z1=%+.6f Z2=%+.6f Z3=%+.6f' %
            tuple(self.suspended_offsets),
            'Baseline dirty: %s' %
            ('YES' if self.baseline_dirty else 'NO'),
            'Motor map: Z0=FL Z1=RL Z2=RR Z3=FR',
            'Tool trims: T0=%+.6f T1=%+.6f mm' %
            (self.tool_t0_trim, self.tool_t1_trim),
            'Tool common compensation=%+.6f mm' %
            self.tool_common_compensation,
            'Hard max tracking offset: %.3f mm / motor' %
            self.max_motor_offset,
            'Authority lambda: %.6f' %
            self.authority_lambda,
            'Minimum condition q: %.6f' %
            self.min_condition_q,
            'Tracking segment length: %.3f mm' %
            self.segment_length,
            'Motor map verified: %s' %
            ('YES' if self.motor_map_verified else 'NO'),
        ]

        if self.disable_reason:
            lines.append(
                'Last disable reason: %s' %
                self.disable_reason
            )

        if ev is None:
            lines.append('No active bed mesh.')

        elif 'error' in ev:
            lines.append(
                'Current solution unavailable: %s' %
                ev['error']
            )

        else:
            lines += [
                'Mode: %s' %
                ev['mode'],

                'T0 X=%.3f  T1 X=%.3f  Y=%.3f  separation=%.3f' %
                (
                    ev['x0'],
                    ev['x1'],
                    ev['y'],
                    ev['separation']
                ),

                'Mesh T0=%+.6f  T1=%+.6f  diff=%+.6f' %
                (
                    ev['mesh_t0'],
                    ev['mesh_t1'],
                    ev['mesh_diff']
                ),

                'Fade=%.6f  mechanical T1-T0=%+.6f' %
                (
                    ev['fade'],
                    ev['mechanical_delta']
                ),

                'Live T1 trim=%+.6f mm' %
                ev['live_trim'],

                'Requested T1-T0 D=%+.6f mm' %
                ev['requested_d'],

                'Achieved D=%+.6f mm  residual=%+.6f mm  authority=%.1f%%' %
                (
                    ev['achieved_d'],
                    ev['residual'],
                    100.0 * ev['authority']
                ),

                'Condition q=%.9f  max-motor/request gain=%.3f' %
                (
                    ev['conditioning_q'],
                    ev['gain']
                ),

                'Uncapped max motor=%+.6f mm  hard-limit scale=%.1f%%' %
                (
                    ev['uncapped_max'],
                    100.0 * ev['hard_scale']
                ),

                'Tracking offsets: Z0=%+.6f Z1=%+.6f Z2=%+.6f Z3=%+.6f' %
                tuple(ev['offsets']),

                'Motor positions: Z0=%.6f Z1=%.6f Z2=%.6f Z3=%.6f' %
                tuple(self._motor_positions()),

                'T0 anchor error=%+.9f mm' %
                self._dot(
                    ev['weights_t0'],
                    ev['offsets']
                ),
            ]

        return lines

    def get_status(self, eventtime):
        status = {
            'enabled': self.enabled,
            'suspended': self.tracking_suspended,
            'suspended_offsets': list(self.suspended_offsets),
            'baseline_dirty': self.baseline_dirty,
            'max_motor_offset': self.max_motor_offset,
            'max_nozzle_differential': self.max_nozzle_differential,
            'authority_lambda': self.authority_lambda,
            'min_condition_q': self.min_condition_q,
            'last_offsets': list(self.last_offsets),
            'disable_reason': self.disable_reason,
            'motor_map_verified': self.motor_map_verified,
            'motor_positions': self._motor_positions()
                if len(self.z_steppers) == 4 else [],
        }

        if self.last_eval is not None:
            status.update({
                'mode': self.last_eval['mode'],
                't0_x': self.last_eval['x0'],
                't1_x': self.last_eval['x1'],
                'y': self.last_eval['y'],
                'mesh_t0': self.last_eval['mesh_t0'],
                'mesh_t1': self.last_eval['mesh_t1'],
                'mesh_diff': self.last_eval['mesh_diff'],
                'fade': self.last_eval['fade'],
                'mechanical_delta': self.last_eval['mechanical_delta'],
                'requested_d': self.last_eval['requested_d'],
                'achieved_d': self.last_eval['achieved_d'],
                'residual': self.last_eval['residual'],
                'authority': self.last_eval['authority'],
                'conditioning_q': self.last_eval['conditioning_q'],
                'anchor_error': self._dot(
                    self.last_eval['weights_t0'], self.last_offsets),
                'uncapped_max': self.last_eval['uncapped_max'],
                'hard_scale': self.last_eval['hard_scale'],
            })

        return status


def load_config(config):
    return BFPDualTracking(config)
