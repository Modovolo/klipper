# Dual probe support for IDEX printers
#
# Manages two independent probes (one per toolhead) without modifying
# the core Klipper probe module.
#
# Copyright (C) 2026  Custom Extension
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import collections

# Define ProbeResult locally (not all Klipper versions export it)
ProbeResult = collections.namedtuple(
    'probe_result', ['bed_x', 'bed_y', 'bed_z', 'test_x', 'test_y', 'test_z'])


class SecondaryProbeEndstopWrapper:
    """
    Endstop wrapper for secondary probe.
    Matches the interface that homing.probing_move() expects:
    - probe_prepare(hmove)
    - probe_finish(hmove)
    - multi_probe_begin()
    - multi_probe_end()
    - get_position_endstop()
    
    Also wraps the MCU endstop methods needed for homing:
    - get_mcu()
    - add_stepper()
    - get_steppers()
    - home_start()
    - home_wait()
    - query_endstop()
    """
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.position_endstop = config.getfloat('z_offset')
        self.stow_on_each_sample = config.getboolean(
            'deactivate_on_each_sample', True)
        
        # Create MCU endstop from probe pin
        ppins = self.printer.lookup_object('pins')
        pin = config.get('pin')
        pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
        mcu = pin_params['chip']
        self.mcu_endstop = mcu.setup_pin('endstop', pin_params)
        
        # Wrap MCU endstop methods (required by homing module)
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_start = self.mcu_endstop.home_start
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop
        
        # Deploy/stow gcode templates
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.activate_gcode = gcode_macro.load_template(
            config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(
            config, 'deactivate_gcode', '')
        
        # Multi-probe state (matches ProbeEndstopWrapper pattern)
        self.multi = 'OFF'
        
        # Tracking
        self.last_state = False
        self.last_z_result = 0.
        
    def _lower_probe(self):
        """Deploy probe"""
        logging.info("dual_probe: Deploying probe '%s'" % self.name)
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.activate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe activate_gcode script")
    
    def _raise_probe(self):
        """Stow probe"""
        logging.info("dual_probe: Stowing probe '%s'" % self.name)
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.deactivate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe deactivate_gcode script")
    
    def multi_probe_begin(self):
        if self.stow_on_each_sample:
            return
        self.multi = 'FIRST'
    
    def multi_probe_end(self):
        if self.stow_on_each_sample:
            return
        self._raise_probe()
        self.multi = 'OFF'
    
    def probe_prepare(self, hmove):
        logging.info("dual_probe: probe_prepare called, multi=%s" % self.multi)
        if self.multi == 'OFF' or self.multi == 'FIRST':
            self._lower_probe()
            if self.multi == 'FIRST':
                self.multi = 'ON'
    
    def probe_finish(self, hmove):
        logging.info("dual_probe: probe_finish called, multi=%s" % self.multi)
        if self.multi == 'OFF':
            self._raise_probe()
    
    def get_position_endstop(self):
        return self.position_endstop


class SecondaryProbe:
    """
    Secondary probe for T1 toolhead.
    Mimics Klipper's PrinterProbe / HomingViaProbeHelper interface.
    
    Configuration example:
    [dual_probe t1]
    pin: ^!HermitCrab2_Board_2_right:gpio24
    deactivate_on_each_sample: False
    x_offset: -35
    y_offset: -27
    z_offset: 0.750
    speed: 2.0
    lift_speed: 15
    samples: 1
    sample_retract_dist: 2.0
    samples_result: median
    samples_tolerance: 0.100
    samples_tolerance_retries: 0
    activate_gcode:
        SET_PIN PIN=probe_enable_t1 VALUE=1
        G4 P500
    deactivate_gcode:
        SET_PIN PIN=probe_enable_t1 VALUE=0
    """
    
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object('gcode')
        
        # Create endstop wrapper (handles deploy/stow and MCU endstop)
        self.mcu_probe = SecondaryProbeEndstopWrapper(config)
        
        # Probe offsets
        self.x_offset = config.getfloat('x_offset', 0.)
        self.y_offset = config.getfloat('y_offset', 0.)
        self.z_offset = config.getfloat('z_offset', 0.)
        
        # Probe parameters
        self.speed = config.getfloat('speed', 5., above=0.)
        self.lift_speed = config.getfloat('lift_speed', self.speed, above=0.)
        self.samples = config.getint('samples', 1, minval=1)
        self.sample_retract_dist = config.getfloat(
            'sample_retract_dist', 2., above=0.)
        self.samples_result = config.getchoice('samples_result',
            {'median': 'median', 'average': 'average'}, 'median')
        self.samples_tolerance = config.getfloat(
            'samples_tolerance', 0.100, minval=0.)
        self.samples_tolerance_retries = config.getint(
            'samples_tolerance_retries', 0, minval=0)
        
        # Z limits for probing
        self.z_position = config.getfloat('z_position', -2.)
        
        # Tracking
        self.last_z_result = 0.
        self.last_probe_position = None
        
        # Will be set on connect
        self.toolhead = None
        
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect)
        self.printer.register_event_handler(
            'klippy:mcu_identify', self._handle_mcu_identify)
        # Register homing events so probe_prepare/probe_finish are called
        # at the correct time during probing_move()
        self.printer.register_event_handler(
            "homing:homing_move_begin",
            self._handle_homing_move_begin)
        self.printer.register_event_handler(
            "homing:homing_move_end",
            self._handle_homing_move_end)
        self.printer.register_event_handler(
            "gcode:command_error",
            self._handle_command_error)
        self.multi_probe_pending = False
        
        # Register PROBE_xx and QUERY_PROBE_xx commands
        # Extract suffix from name: "dual_probe t1" -> "T1"
        name_parts = self.name.split(None, 1)  # Split on whitespace
        if len(name_parts) > 1:
            suffix = name_parts[1].upper()
        else:
            suffix = self.name.upper()
        
        self.gcode.register_command(
            'PROBE_%s' % suffix,
            self.cmd_PROBE,
            desc="Probe Z-height using %s" % self.name)
        self.gcode.register_command(
            'QUERY_PROBE_%s' % suffix,
            self.cmd_QUERY_PROBE,
            desc="Query state of %s" % self.name)
        
        logging.info("dual_probe: Registered PROBE_%s and QUERY_PROBE_%s"
                     % (suffix, suffix))
    
    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
    
    def _handle_mcu_identify(self):
        # Register Z steppers with this endstop
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        z_steppers = []
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.mcu_probe.add_stepper(stepper)
                z_steppers.append(stepper.get_name())
        logging.info("dual_probe: Added %d Z steppers to '%s' endstop: %s"
                     % (len(z_steppers), self.name, z_steppers))
    
    def _handle_homing_move_begin(self, hmove):
        if self.mcu_probe in hmove.get_mcu_endstops():
            self.mcu_probe.probe_prepare(hmove)
    
    def _handle_homing_move_end(self, hmove):
        if self.mcu_probe in hmove.get_mcu_endstops():
            self.mcu_probe.probe_finish(hmove)
    
    def _handle_command_error(self):
        if self.multi_probe_pending:
            self.multi_probe_pending = False
            try:
                self.mcu_probe.multi_probe_end()
            except:
                logging.exception("Multi-probe end failed on error")
    
    def get_offsets(self, gcmd=None):
        return self.x_offset, self.y_offset, self.z_offset
    
    def get_lift_speed(self, gcmd=None):
        if gcmd is not None:
            return gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.)
        return self.lift_speed
    
    def get_probe_params(self, gcmd=None):
        if gcmd is None:
            return {
                'probe_speed': self.speed,
                'lift_speed': self.lift_speed,
                'samples': self.samples,
                'sample_retract_dist': self.sample_retract_dist,
                'samples_tolerance': self.samples_tolerance,
                'samples_tolerance_retries': self.samples_tolerance_retries,
                'samples_result': self.samples_result,
            }
        return {
            'probe_speed': gcmd.get_float(
                'PROBE_SPEED', self.speed, above=0.),
            'lift_speed': gcmd.get_float(
                'LIFT_SPEED', self.lift_speed, above=0.),
            'samples': gcmd.get_int(
                'SAMPLES', self.samples, minval=1),
            'sample_retract_dist': gcmd.get_float(
                'SAMPLE_RETRACT_DIST', self.sample_retract_dist, above=0.),
            'samples_tolerance': gcmd.get_float(
                'SAMPLES_TOLERANCE', self.samples_tolerance, minval=0.),
            'samples_tolerance_retries': gcmd.get_int(
                'SAMPLES_TOLERANCE_RETRIES',
                self.samples_tolerance_retries, minval=0),
            'samples_result': gcmd.get(
                'SAMPLES_RESULT', self.samples_result),
        }
    
    def _probe_single(self, gcmd, speed):
        """Run a single probe using homing module's probing_move"""
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        if 'z' not in toolhead.get_status(curtime)['homed_axes']:
            raise self.printer.command_error("Must home before probe")
        
        pos = toolhead.get_position()
        logging.info("dual_probe: _probe_single start pos=%.3f,%.3f,%.3f"
                     " target_z=%.3f speed=%.1f"
                     % (pos[0], pos[1], pos[2], self.z_position, speed))
        pos[2] = self.z_position  # Target Z (how far down to probe)
        
        phoming = self.printer.lookup_object('homing')
        # probing_move expects the endstop wrapper with
        # probe_prepare/probe_finish methods
        curpos = phoming.probing_move(self.mcu_probe, pos, speed)
        
        logging.info("dual_probe: probing_move returned pos=%.3f,%.3f,%.6f"
                     % (curpos[0], curpos[1], curpos[2]))
        
        # Create ProbeResult
        bed_x = curpos[0] + self.x_offset
        bed_y = curpos[1] + self.y_offset
        bed_z = curpos[2]
        result = ProbeResult(
            bed_x=bed_x, bed_y=bed_y, bed_z=bed_z,
            test_x=curpos[0], test_y=curpos[1], test_z=curpos[2])
        
        logging.info("dual_probe: probe result bed=%.3f,%.3f,%.6f"
                     % (bed_x, bed_y, bed_z))
        self.gcode.respond_info(
            "%s: at %.3f,%.3f bed will contact at z=%.6f"
            % (self.name, bed_x, bed_y, bed_z))
        
        return result
    
    def start_probe_session(self, gcmd):
        """Start probe session - returns a session helper"""
        return SecondaryProbeSession(self, gcmd)
    
    def get_status(self, eventtime):
        return {
            'name': self.name,
            'last_z_result': self.last_z_result,
            'last_probe_position': self.last_probe_position,
        }
    
    # G-Code commands
    def cmd_PROBE(self, gcmd):
        """PROBE_T1 command - probe at current position"""
        logging.info("dual_probe: PROBE_%s command called" % self.name)
        params = self.get_probe_params(gcmd)
        logging.info("dual_probe: probe params: speed=%.1f samples=%d"
                     " retract=%.1f tolerance=%.3f"
                     % (params['probe_speed'], params['samples'],
                        params['sample_retract_dist'],
                        params['samples_tolerance']))
        
        self.mcu_probe.multi_probe_begin()
        try:
            results = []
            retries = 0
            sample_count = params['samples']
            toolhead = self.printer.lookup_object('toolhead')
            
            while len(results) < sample_count:
                result = self._probe_single(gcmd, params['probe_speed'])
                results.append(result)
                
                # Check tolerance
                z_vals = [r.bed_z for r in results]
                if (max(z_vals) - min(z_vals)
                        > params['samples_tolerance']):
                    if retries >= params['samples_tolerance_retries']:
                        raise gcmd.error(
                            "Probe samples exceed samples_tolerance")
                    gcmd.respond_info(
                        "Probe samples exceed tolerance. Retrying...")
                    retries += 1
                    results = []
                
                # Retract between samples
                if len(results) < sample_count:
                    cur_z = toolhead.get_position()[2]
                    toolhead.manual_move(
                        [None, None, cur_z + params['sample_retract_dist']],
                        params['lift_speed'])
        finally:
            self.mcu_probe.multi_probe_end()
        
        # Calculate final result
        if params['samples_result'] == 'median':
            results.sort(key=lambda r: r.bed_z)
            final = results[len(results) // 2]
        else:
            avg_z = sum([r.bed_z for r in results]) / len(results)
            final = ProbeResult(
                bed_x=results[0].bed_x, bed_y=results[0].bed_y,
                bed_z=avg_z,
                test_x=results[0].test_x, test_y=results[0].test_y,
                test_z=avg_z)
        
        self.last_z_result = final.bed_z
        self.last_probe_position = self.gcode.Coord(
            final.bed_x, final.bed_y, final.bed_z, 0.)
        
        logging.info("dual_probe: PROBE_%s complete: z=%.6f at %.3f,%.3f"
                     % (self.name, final.bed_z, final.bed_x, final.bed_y))
        gcmd.respond_info(
            "Result: at %.3f,%.3f estimate contact at z=%.6f"
            % (final.bed_x, final.bed_y, final.bed_z))
    
    def cmd_QUERY_PROBE(self, gcmd):
        """QUERY_PROBE_T1 command"""
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        res = self.mcu_probe.query_endstop(print_time)
        self.mcu_probe.last_state = res
        state_str = ["open", "TRIGGERED"][not not res]
        logging.info("dual_probe: QUERY_PROBE_%s: %s" % (self.name, state_str))
        gcmd.respond_info(
            "%s: %s" % (self.name, state_str))


class SecondaryProbeSession:
    """
    Probe session for SecondaryProbe.
    Matches Klipper's ProbeSessionHelper interface:
    - start_probe_session(gcmd)
    - run_probe(gcmd)
    - pull_probed_results()
    - end_probe_session()
    """
    def __init__(self, probe, gcmd):
        self.probe = probe
        self.gcmd = gcmd
        self.results = []
        self.probe.mcu_probe.multi_probe_begin()
    
    def start_probe_session(self, gcmd):
        return self
    
    def run_probe(self, gcmd):
        """Perform probing with configured samples"""
        params = self.probe.get_probe_params(gcmd)
        toolhead = self.probe.printer.lookup_object('toolhead')
        probexy = toolhead.get_position()[:2]
        
        retries = 0
        positions = []
        sample_count = params['samples']
        
        while len(positions) < sample_count:
            pos = self.probe._probe_single(gcmd, params['probe_speed'])
            positions.append(pos)
            
            # Check tolerance
            z_vals = [p.bed_z for p in positions]
            if (max(z_vals) - min(z_vals)
                    > params['samples_tolerance']):
                if retries >= params['samples_tolerance_retries']:
                    raise gcmd.error(
                        "Probe samples exceed samples_tolerance")
                gcmd.respond_info(
                    "Probe samples exceed tolerance. Retrying...")
                retries += 1
                positions = []
            
            # Retract between samples
            if len(positions) < sample_count:
                cur_z = toolhead.get_position()[2]
                toolhead.manual_move(
                    probexy + [cur_z + params['sample_retract_dist']],
                    params['lift_speed'])
        
        # Calculate final result
        if params['samples_result'] == 'median':
            positions.sort(key=lambda r: r.bed_z)
            final = positions[len(positions) // 2]
        else:
            avg_z = sum([r.bed_z for r in positions]) / len(positions)
            final = ProbeResult(
                bed_x=positions[0].bed_x, bed_y=positions[0].bed_y,
                bed_z=avg_z,
                test_x=positions[0].test_x, test_y=positions[0].test_y,
                test_z=avg_z)
        
        self.results.append(final)
        self.probe.last_z_result = final.bed_z
    
    def pull_probed_results(self):
        res = self.results
        self.results = []
        return res
    
    def end_probe_session(self):
        self.probe.mcu_probe.multi_probe_end()
        self.results = []


class DualProbeManager:
    """
    Manages two probes for T0 and T1 toolheads.
    Provides unified interface for dual_bed_mesh.
    
    Configuration example:
    [dual_probe]
    t0_probe: probe
    t1_probe: dual_probe t1
    auto_switch: True
    """
    
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object('gcode')
        
        # Probe section names
        self.t0_probe_name = config.get('t0_probe', 'probe')
        self.t1_probe_name = config.get('t1_probe', 'dual_probe t1')
        
        # Auto-switch behavior
        self.auto_switch = config.getboolean('auto_switch', True)
        
        # Probe objects (populated on connect)
        self.t0_probe = None
        self.t1_probe = None
        self.dual_carriage = None
        self.toolhead = None
        
        # Current active probe
        self.active_probe_id = 0
        
        # Register event handlers
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect)
        
        # Register commands
        self.gcode.register_command(
            'DUAL_PROBE_SELECT', self.cmd_DUAL_PROBE_SELECT,
            desc="Select active probe (T0 or T1)")
        self.gcode.register_command(
            'DUAL_PROBE_STATUS', self.cmd_DUAL_PROBE_STATUS,
            desc="Show dual probe status")
        self.gcode.register_command(
            'DUAL_PROBE_CALIBRATE', self.cmd_DUAL_PROBE_CALIBRATE,
            desc="Calibrate Z offset between T0 and T1 probes")
    
    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        
        # Look up T0 probe
        try:
            self.t0_probe = self.printer.lookup_object(self.t0_probe_name)
            logging.info("dual_probe: T0 probe '%s' found" 
                         % self.t0_probe_name)
        except Exception as e:
            logging.warning("dual_probe: T0 probe '%s' not found: %s"
                            % (self.t0_probe_name, e))
        
        # Look up T1 probe
        try:
            self.t1_probe = self.printer.lookup_object(self.t1_probe_name)
            logging.info("dual_probe: T1 probe '%s' found" 
                         % self.t1_probe_name)
        except Exception as e:
            logging.info("dual_probe: T1 probe '%s' not found, "
                         "will use T0 for both" % self.t1_probe_name)
        
        # Look up dual carriage
        try:
            self.dual_carriage = self.printer.lookup_object('dual_carriage')
        except:
            self.dual_carriage = None
    
    def get_probe(self, carriage_id=None):
        """Get probe for specified carriage"""
        if carriage_id is None:
            carriage_id = self._get_active_carriage()
        if carriage_id == 1 and self.t1_probe is not None:
            return self.t1_probe
        return self.t0_probe
    
    def get_probe_offsets(self, carriage_id=None):
        probe = self.get_probe(carriage_id)
        if probe is None:
            return (0., 0., 0.)
        return probe.get_offsets()
    
    def _get_active_carriage(self):
        if self.dual_carriage is None:
            return 0
        dc_status = self.dual_carriage.get_status(
            self.printer.get_reactor().monotonic())
        active = dc_status.get('active_carriage', 'CARRIAGE_0')
        return 0 if active == 'CARRIAGE_0' else 1
    
    def _switch_to_carriage(self, carriage_id):
        self.gcode.run_script_from_command(
            "SET_DUAL_CARRIAGE CARRIAGE=%d" % carriage_id)
        extruder = 'extruder' if carriage_id == 0 else 'extruder1'
        self.gcode.run_script_from_command(
            "ACTIVATE_EXTRUDER EXTRUDER=%s" % extruder)
        self.active_probe_id = carriage_id
    
    def cmd_DUAL_PROBE_SELECT(self, gcmd):
        carriage = gcmd.get_int('T', None)
        if carriage is None:
            gcmd.respond_info(
                "Active probe: T%d" % self.active_probe_id)
            return
        if carriage not in [0, 1]:
            raise gcmd.error("T must be 0 or 1")
        self._switch_to_carriage(carriage)
        gcmd.respond_info("Switched to T%d probe" % carriage)
    
    def cmd_DUAL_PROBE_STATUS(self, gcmd):
        gcmd.respond_info("=== Dual Probe Status ===")
        gcmd.respond_info(
            "  Active carriage: T%d" % self._get_active_carriage())
        gcmd.respond_info(
            "  Active probe: T%d" % self.active_probe_id)
        gcmd.respond_info(
            "  Auto-switch: %s" % self.auto_switch)
        
        if self.t0_probe:
            offsets = self.t0_probe.get_offsets()
            gcmd.respond_info(
                "  T0 probe (%s): X=%.2f Y=%.2f Z=%.3f"
                % (self.t0_probe_name,
                   offsets[0], offsets[1], offsets[2]))
        else:
            gcmd.respond_info("  T0 probe: NOT FOUND")
        
        if self.t1_probe:
            offsets = self.t1_probe.get_offsets()
            gcmd.respond_info(
                "  T1 probe (%s): X=%.2f Y=%.2f Z=%.3f"
                % (self.t1_probe_name,
                   offsets[0], offsets[1], offsets[2]))
        else:
            gcmd.respond_info("  T1 probe: NOT CONFIGURED")
    
    def cmd_DUAL_PROBE_CALIBRATE(self, gcmd):
        """Probe same point with both probes, report Z difference"""
        if self.t0_probe is None:
            raise gcmd.error("T0 probe not configured")
        if self.t1_probe is None:
            raise gcmd.error("T1 probe not configured")
        
        x = gcmd.get_float('X', 400.)
        y = gcmd.get_float('Y', 400.)
        
        gcmd.respond_info(
            "Calibrating probe Z difference at X=%.1f Y=%.1f" % (x, y))
        
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        if 'xyz' not in toolhead.get_status(curtime)['homed_axes']:
            raise gcmd.error("Must home first")
        
        # Probe with T0
        gcmd.respond_info("Probing with T0...")
        self._switch_to_carriage(0)
        t0_offsets = self.t0_probe.get_offsets()
        toolhead.manual_move(
            [x - t0_offsets[0], y - t0_offsets[1], None], 50.)
        toolhead.manual_move([None, None, 5.], 50.)
        toolhead.wait_moves()
        
        from . import probe as probe_module
        pos0 = probe_module.run_single_probe(self.t0_probe, gcmd)
        z_t0 = pos0.bed_z
        gcmd.respond_info("  T0 result: Z=%.4f" % z_t0)
        
        toolhead.manual_move([None, None, 10.], 50.)
        toolhead.wait_moves()
        
        # Probe with T1
        gcmd.respond_info("Probing with T1...")
        self._switch_to_carriage(1)
        t1_offsets = self.t1_probe.get_offsets()
        toolhead.manual_move(
            [x - t1_offsets[0], y - t1_offsets[1], None], 50.)
        toolhead.manual_move([None, None, 5.], 50.)
        toolhead.wait_moves()
        
        pos1 = probe_module.run_single_probe(self.t1_probe, gcmd)
        z_t1 = pos1.bed_z
        gcmd.respond_info("  T1 result: Z=%.4f" % z_t1)
        
        diff = z_t1 - z_t0
        gcmd.respond_info(
            "\nZ difference (T1 - T0): %+.4f mm" % diff)
        
        # Return to T0
        self._switch_to_carriage(0)
        toolhead.manual_move([None, None, 10.], 50.)
    
    def get_status(self, eventtime):
        return {
            't0_probe_available': self.t0_probe is not None,
            't1_probe_available': self.t1_probe is not None,
            'active_probe_id': self.active_probe_id,
            'auto_switch': self.auto_switch,
        }


def load_config(config):
    return DualProbeManager(config)

def load_config_prefix(config):
    # Handles [dual_probe t1] or similar sections
    # Klipper calls this for any [dual_probe XXX] section
    return SecondaryProbe(config)
