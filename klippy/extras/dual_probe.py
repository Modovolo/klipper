# Dual probe support for IDEX printers
#
# Manages two independent probes (one per toolhead) without modifying
# the core Klipper probe module.
#
# Copyright (C) 2026  Custom Extension
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

class DualProbeManager:
    """
    Manages two independent probes for IDEX printers.
    
    This module provides:
    - Registration and lookup of T0 and T1 probes
    - Probe switching based on active carriage
    - Unified interface for dual-probe operations
    - Independent offset management per probe
    
    Configuration example:
    [dual_probe]
    # T0 probe (left toolhead) - references the main [probe] section
    t0_probe: probe
    # T1 probe (right toolhead) - references [dual_probe t1] section
    t1_probe: dual_probe t1
    # Auto-switch probe based on active carriage
    auto_switch: True
    
    # Define your secondary probe in printer.cfg:
    # NOTE: Use 'dual_probe ' prefix with SPACE for Klipper to recognize it
    [dual_probe t1]
    pin: ^!HermitCrab2_Board_2_right:gpio24
    x_offset: -35
    y_offset: -27
    z_offset: 0.750
    speed: 2.0
    samples: 1
    # ... etc
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
        self.active_probe_id = 0  # 0=T0, 1=T1
        
        # Register event handlers
        self.printer.register_event_handler("klippy:connect", 
                                            self._handle_connect)
        
        # Register commands
        self.gcode.register_command(
            'DUAL_PROBE_SELECT',
            self.cmd_DUAL_PROBE_SELECT,
            desc="Select active probe (T0 or T1)")
        self.gcode.register_command(
            'DUAL_PROBE_STATUS',
            self.cmd_DUAL_PROBE_STATUS,
            desc="Show dual probe status")
        self.gcode.register_command(
            'DUAL_PROBE_QUERY',
            self.cmd_DUAL_PROBE_QUERY,
            desc="Query both probes")
        self.gcode.register_command(
            'DUAL_PROBE_CALIBRATE',
            self.cmd_DUAL_PROBE_CALIBRATE,
            desc="Calibrate Z offset between T0 and T1 probes")
            
    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        
        # Look up T0 probe (main probe)
        try:
            self.t0_probe = self.printer.lookup_object(self.t0_probe_name)
            logging.info(f"dual_probe: T0 probe '{self.t0_probe_name}' found")
        except Exception as e:
            logging.warning(f"dual_probe: T0 probe '{self.t0_probe_name}' not found: {e}")
            self.t0_probe = None
            
        # Look up T1 probe (secondary probe)
        try:
            self.t1_probe = self.printer.lookup_object(self.t1_probe_name)
            logging.info(f"dual_probe: T1 probe '{self.t1_probe_name}' found")
        except Exception as e:
            logging.info(f"dual_probe: T1 probe '{self.t1_probe_name}' not found - "
                        "T1 probing will use T0 probe with carriage switch")
            self.t1_probe = None
            
        # Look up dual carriage
        try:
            self.dual_carriage = self.printer.lookup_object('dual_carriage')
        except:
            self.dual_carriage = None
            
    def get_probe(self, carriage_id=None):
        """
        Get probe object for specified carriage.
        If carriage_id is None, returns probe for active carriage.
        """
        if carriage_id is None:
            carriage_id = self._get_active_carriage()
            
        if carriage_id == 1 and self.t1_probe is not None:
            return self.t1_probe
        elif carriage_id == 0 and self.t0_probe is not None:
            return self.t0_probe
        else:
            # Fallback to whatever probe is available
            return self.t0_probe or self.t1_probe
            
    def get_probe_offsets(self, carriage_id=None):
        """Get probe offsets for specified carriage"""
        probe = self.get_probe(carriage_id)
        if probe is None:
            return (0.0, 0.0, 0.0)
        return probe.get_offsets()
        
    def _get_active_carriage(self):
        """Return 0 for T0/left, 1 for T1/right"""
        if self.dual_carriage is None:
            return 0
        dc_status = self.dual_carriage.get_status(
            self.printer.get_reactor().monotonic())
        mode = dc_status.get('carriage_1', 'PRIMARY')
        if mode in ['COPY', 'MIRROR']:
            # In COPY/MIRROR, T0 is primary
            return 0
        active = dc_status.get('active_carriage', 'CARRIAGE_0')
        return 0 if active == 'CARRIAGE_0' else 1
        
    def _switch_to_carriage(self, carriage_id):
        """Switch to specified carriage"""
        self.gcode.run_script_from_command(
            f"SET_DUAL_CARRIAGE CARRIAGE={carriage_id} MODE=PRIMARY")
        extruder = 'extruder' if carriage_id == 0 else 'extruder1'
        self.gcode.run_script_from_command(
            f"ACTIVATE_EXTRUDER EXTRUDER={extruder}")
        self.active_probe_id = carriage_id
        
    def probe_at_position(self, x, y, carriage_id, gcmd, 
                          speed=None, samples=None):
        """
        Probe at a specific position with the specified carriage's probe.
        Handles carriage switching and probe offset compensation.
        Returns the probed Z position (bed height at that XY).
        """
        # Switch to correct carriage if needed
        current_carriage = self._get_active_carriage()
        if current_carriage != carriage_id:
            self._switch_to_carriage(carriage_id)
            
        probe = self.get_probe(carriage_id)
        if probe is None:
            raise self.gcode.error(
                f"No probe available for carriage {carriage_id}")
            
        # Get probe offsets
        offsets = probe.get_offsets()
        x_offset, y_offset, z_offset = offsets[0], offsets[1], offsets[2]
        
        # Calculate probe position (where toolhead needs to be)
        probe_x = x - x_offset
        probe_y = y - y_offset
        
        # Move to probe position at safe height
        horizontal_move_z = 5.0  # Could be configurable
        self.toolhead.manual_move([probe_x, probe_y, None], 120.0)
        self.toolhead.manual_move([None, None, horizontal_move_z], 120.0)
        self.toolhead.wait_moves()
        
        # Perform probe
        try:
            probe_session = probe.start_probe_session(gcmd)
            probe_session.run_probe(gcmd)
            results = probe_session.pull_probed_results()
            probe_session.end_probe_session()
            
            if results:
                z_result = results[0].pos_z
            else:
                z_result = 0.0
                
        except Exception as e:
            raise self.gcode.error(f"Probe failed: {e}")
            
        # Lift after probe
        self.toolhead.manual_move([None, None, horizontal_move_z], 120.0)
        
        return z_result
        
    def cmd_DUAL_PROBE_SELECT(self, gcmd):
        """Select which probe to use"""
        carriage = gcmd.get_int('T', None)
        if carriage is None:
            gcmd.respond_info(f"Current active probe: T{self.active_probe_id}")
            return
            
        if carriage not in [0, 1]:
            raise gcmd.error("T must be 0 or 1")
            
        self._switch_to_carriage(carriage)
        gcmd.respond_info(f"Switched to T{carriage} probe")
        
    def cmd_DUAL_PROBE_STATUS(self, gcmd):
        """Show status of both probes"""
        gcmd.respond_info("Dual Probe Status:")
        gcmd.respond_info(f"  Active carriage: T{self._get_active_carriage()}")
        gcmd.respond_info(f"  Active probe ID: T{self.active_probe_id}")
        gcmd.respond_info(f"  Auto-switch: {self.auto_switch}")
        
        # T0 probe info
        if self.t0_probe:
            offsets = self.t0_probe.get_offsets()
            gcmd.respond_info(f"  T0 probe ({self.t0_probe_name}):")
            gcmd.respond_info(f"    Offsets: X={offsets[0]:.2f}, Y={offsets[1]:.2f}, Z={offsets[2]:.3f}")
        else:
            gcmd.respond_info(f"  T0 probe: NOT CONFIGURED")
            
        # T1 probe info
        if self.t1_probe:
            offsets = self.t1_probe.get_offsets()
            gcmd.respond_info(f"  T1 probe ({self.t1_probe_name}):")
            gcmd.respond_info(f"    Offsets: X={offsets[0]:.2f}, Y={offsets[1]:.2f}, Z={offsets[2]:.3f}")
        else:
            gcmd.respond_info(f"  T1 probe: NOT CONFIGURED (will use T0 probe)")
            
    def cmd_DUAL_PROBE_QUERY(self, gcmd):
        """Query both probes at current position"""
        gcmd.respond_info("Querying both probes...")
        
        # Store current position
        pos = self.toolhead.get_position()
        
        # Query T0
        if self.t0_probe:
            try:
                self._switch_to_carriage(0)
                self.toolhead.wait_moves()
                # Note: This is a simplified query - actual implementation
                # would need to handle endstop query properly
                gcmd.respond_info(f"  T0 probe: ready")
            except Exception as e:
                gcmd.respond_info(f"  T0 probe: error - {e}")
        else:
            gcmd.respond_info(f"  T0 probe: not configured")
            
        # Query T1
        if self.t1_probe:
            try:
                self._switch_to_carriage(1)
                self.toolhead.wait_moves()
                gcmd.respond_info(f"  T1 probe: ready")
            except Exception as e:
                gcmd.respond_info(f"  T1 probe: error - {e}")
        else:
            gcmd.respond_info(f"  T1 probe: not configured")
            
    def cmd_DUAL_PROBE_CALIBRATE(self, gcmd):
        """
        Calibrate Z offset difference between T0 and T1 probes.
        Probes the same point with both probes and reports the difference.
        """
        x = gcmd.get_float('X', 400.0)  # Default to center of 800mm bed
        y = gcmd.get_float('Y', 400.0)
        
        if self.t0_probe is None:
            raise gcmd.error("T0 probe not configured")
        if self.t1_probe is None:
            raise gcmd.error("T1 probe not configured - nothing to calibrate")
            
        gcmd.respond_info(f"Calibrating probe Z offset difference at X={x}, Y={y}")
        
        # Probe with T0
        gcmd.respond_info("Probing with T0...")
        z_t0 = self.probe_at_position(x, y, 0, gcmd)
        gcmd.respond_info(f"  T0 result: Z={z_t0:.4f}")
        
        # Probe with T1
        gcmd.respond_info("Probing with T1...")
        z_t1 = self.probe_at_position(x, y, 1, gcmd)
        gcmd.respond_info(f"  T1 result: Z={z_t1:.4f}")
        
        # Calculate difference
        diff = z_t1 - z_t0
        gcmd.respond_info(f"\nZ offset difference (T1 - T0): {diff:.4f}mm")
        gcmd.respond_info(f"If T1 reads higher, its z_offset should be increased by {diff:.4f}")
        gcmd.respond_info(f"Current T0 z_offset: {self.t0_probe.get_offsets()[2]:.4f}")
        gcmd.respond_info(f"Current T1 z_offset: {self.t1_probe.get_offsets()[2]:.4f}")
        suggested_t1_offset = self.t1_probe.get_offsets()[2] - diff
        gcmd.respond_info(f"Suggested T1 z_offset: {suggested_t1_offset:.4f}")
        
    def get_status(self, eventtime):
        """Return status for Moonraker/Mainsail"""
        return {
            't0_probe_available': self.t0_probe is not None,
            't1_probe_available': self.t1_probe is not None,
            'active_probe_id': self.active_probe_id,
            'auto_switch': self.auto_switch,
        }


# Secondary probe class - allows defining a second probe without conflicting
# with the main [probe] section
class SecondaryProbe:
    """
    Secondary probe for T1 toolhead.
    
    This is a wrapper that provides probe functionality without
    conflicting with the main [probe] module's singleton registration.
    
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
    # Optional deploy/stow for servo-actuated probes
    # activate_gcode:
    # deactivate_gcode:
    """
    
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object('gcode')
        
        # Get pin and create MCU endstop
        ppins = self.printer.lookup_object('pins')
        pin = config.get('pin')
        pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
        mcu = pin_params['chip']
        self.mcu_endstop = mcu.setup_pin('endstop', pin_params)
        
        # Probe parameters
        self.x_offset = config.getfloat('x_offset', 0.0)
        self.y_offset = config.getfloat('y_offset', 0.0)
        self.z_offset = config.getfloat('z_offset', 0.0)
        self.speed = config.getfloat('speed', 5.0)
        self.lift_speed = config.getfloat('lift_speed', 
                                          self.speed, above=0.)
        self.samples = config.getint('samples', 1, minval=1)
        self.sample_retract_dist = config.getfloat('sample_retract_dist', 
                                                    2.0, above=0.)
        self.samples_result = config.getchoice('samples_result',
            {'median': 'median', 'average': 'average'}, 'median')
        self.samples_tolerance = config.getfloat('samples_tolerance', 
                                                  0.100, minval=0.)
        self.samples_tolerance_retries = config.getint(
            'samples_tolerance_retries', 0, minval=0)
        self.deactivate_on_each_sample = config.getboolean(
            'deactivate_on_each_sample', False)
            
        # Deploy/stow gcode
        self.activate_gcode = config.get('activate_gcode', '')
        self.deactivate_gcode = config.get('deactivate_gcode', '')
        
        # Position tracking
        self.last_z_result = 0.0
        
        # Will be set on connect
        self.toolhead = None
        self.printer.register_event_handler("klippy:connect", 
                                            self._handle_connect)
        
        # Register Z steppers with this endstop
        self.printer.register_event_handler("klippy:mcu_identify",
                                            self._handle_mcu_identify)
        
    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        
    def _handle_mcu_identify(self):
        # Register Z steppers with this endstop
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.mcu_endstop.add_stepper(stepper)
                
    def get_offsets(self):
        """Return probe offsets as tuple"""
        return (self.x_offset, self.y_offset, self.z_offset)
        
    def get_lift_speed(self, gcmd=None):
        return self.lift_speed
        
    def _deploy_probe(self):
        """Run activate gcode if defined"""
        if self.activate_gcode:
            self.gcode.run_script_from_command(self.activate_gcode)
            
    def _stow_probe(self):
        """Run deactivate gcode if defined"""
        if self.deactivate_gcode:
            self.gcode.run_script_from_command(self.deactivate_gcode)
            
    def start_probe_session(self, gcmd):
        """Start a probe session - returns self as the session object"""
        self._deploy_probe()
        return SecondaryProbeSession(self, gcmd)
        
    def get_status(self, eventtime):
        return {
            'name': self.name,
            'last_z_result': self.last_z_result,
            'x_offset': self.x_offset,
            'y_offset': self.y_offset,
            'z_offset': self.z_offset,
        }


class SecondaryProbeSession:
    """Probe session for SecondaryProbe"""
    
    def __init__(self, probe, gcmd):
        self.probe = probe
        self.gcmd = gcmd
        self.results = []
        
    def run_probe(self, gcmd):
        """Perform the actual probing"""
        toolhead = self.probe.toolhead
        
        # Get current position
        pos = toolhead.get_position()
        
        # Perform multiple samples if configured
        positions = []
        retries = 0
        
        while True:
            # Probe down
            try:
                phoming = self.probe.printer.lookup_object('homing')
                epos = phoming.probing_move(
                    self.probe.mcu_endstop, 
                    [pos[0], pos[1], -10.0],  # Move down to -10
                    self.probe.speed)
                positions.append(epos[2])
            except Exception as e:
                raise gcmd.error(f"Probe failed: {e}")
                
            # Retract
            liftpos = [None, None, pos[2]]
            toolhead.manual_move(liftpos, self.probe.lift_speed)
            
            if len(positions) >= self.probe.samples:
                break
                
        # Calculate result
        if self.probe.samples_result == 'median':
            positions.sort()
            z = positions[len(positions) // 2]
        else:
            z = sum(positions) / len(positions)
            
        # Store result
        self.probe.last_z_result = z
        
        # Create result object matching Klipper's ProbeResult
        from . import manual_probe
        result = manual_probe.ProbeResult(
            pos[0] + self.probe.x_offset,
            pos[1] + self.probe.y_offset,
            z,
            pos[0], pos[1], z)
        self.results.append(result)
        
    def pull_probed_results(self):
        """Return probe results"""
        return self.results
        
    def end_probe_session(self):
        """End probe session"""
        self.probe._stow_probe()
        self.results = []


def load_config(config):
    return DualProbeManager(config)

def load_config_prefix(config):
    # This allows [dual_probe t1] or similar sections
    # Klipper's load_config_prefix handles sections like:
    #   [module_name suffix] - with a SPACE separator
    return SecondaryProbe(config)
