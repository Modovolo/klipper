# BFP / Modovolo dual-nozzle Z calibration
#
# Eddy:
#   Provides the machine Z reference and homing trigger.
#   Its probe z_offset / descend height is NOT modified here.
#
# T0:
#   MANUAL_PROBE determines the persistent T0 print Z offset.
#
# T1:
#   MANUAL_PROBE at the exact same physical XY determines
#   the T1 nozzle height relative to T0.
#
# Result:
#   t0_z_offset = T0 contact Z
#   t1_z_delta  = T1 contact Z - T0 contact Z

class BFPDualCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.config_name = config.get_name()

        # Configuration
        self.lift = config.getfloat("lift", 10.0, above=0.0)
        self.lift_speed = config.getfloat(
            "lift_speed", 10.0, above=0.0)
        self.travel_speed = config.getfloat(
            "travel_speed", 100.0, above=0.0)
        self.manual_speed = config.getfloat(
            "manual_speed", 5.0, above=0.0)

        self.auto_save = config.getboolean("auto_save", False)

        # Persistent calibrated values
        self.t0_z_offset = config.getfloat(
            "t0_z_offset", 0.0)

        self.t1_z_delta = config.getfloat(
            "t1_z_delta", 0.0)

        # Runtime calibration state
        self.phase = "idle"

        self.t0_contact_z = None
        self.t1_contact_z = None

        self.reference_x = None
        self.reference_y = None

        self.phase_start_position = None

        # Temporary copies of Klipper's manual-probe handlers
        self.base_accept = None
        self.base_next = None
        self.base_abort = None

        # Take over PROBE_CALIBRATE only after all normal Klipper
        # objects have been loaded.
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect)

        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready)

        self.gcode.register_command(
            "BFP_DUAL_CALIBRATION_STATUS",
            self.cmd_BFP_DUAL_CALIBRATION_STATUS,
            desc="Report BFP T0/T1 Z calibration status"
        )

    ##################################################################
    # Replace PROBE_CALIBRATE while retaining Klipper's original
    ##################################################################

    def _handle_connect(self):
        base = self.gcode.register_command(
            "PROBE_CALIBRATE", None)

        # Native PROBE_CALIBRATE may be absent with Eddy.
        # BFP uses its own MANUAL_PROBE calibration routine.
        self.base_probe_calibrate = base

        self.gcode.register_command(
            "PROBE_CALIBRATE",
            self.cmd_PROBE_CALIBRATE,
            desc="Calibrate T0 print Z offset and T1 relative Z offset"
        )

    def _handle_ready(self):
        # Restore the persistent T0 print offset.
        #
        # This intentionally does NOT modify the Eddy probe's own
        # z_offset / descend height.
        self.gcode.run_script_from_command(
            "SET_GCODE_OFFSET Z=%.6f" % self.t0_z_offset
        )

    ##################################################################
    # Helpers
    ##################################################################

    def _get_toolhead(self):
        return self.printer.lookup_object("toolhead")

    def _get_kinematic_position(self):
        # This duplicates the position method Klipper's
        # ManualProbeHelper uses internally so our stored Z values
        # match the actual ACCEPT result.
        toolhead = self._get_toolhead()

        toolhead.flush_step_generation()

        kin = toolhead.get_kinematics()

        kin_spos = {
            stepper.get_name(): stepper.get_commanded_position()
            for stepper in kin.get_steppers()
        }

        return kin.calc_position(kin_spos)

    def _check_idex_state(self, gcmd):
        toolhead = self._get_toolhead()
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()

        status = toolhead.get_status(eventtime)

        if "xyz" not in status["homed_axes"]:
            raise gcmd.error(
                "BFP dual Z calibration requires XYZ to be homed"
            )

        state_obj = self.printer.lookup_object(
            "gcode_macro _IDEX_STATE", None)

        if state_obj is not None:
            state = state_obj.get_status(eventtime)

            if str(state.get("mode", "PRIMARY")) != "PRIMARY":
                raise gcmd.error(
                    "BFP dual Z calibration requires PRIMARY mode. "
                    "Disable COPY/MIRROR first."
                )

    ##################################################################
    # Manual probe handler interception
    ##################################################################

    def _install_manual_probe_hooks(self):
        # MANUAL_PROBE / PROBE_CALIBRATE dynamically register these
        # commands. Remove Klipper's handlers temporarily and keep them.
        self.base_accept = self.gcode.register_command(
            "ACCEPT", None)
        self.base_next = self.gcode.register_command(
            "NEXT", None)
        self.base_abort = self.gcode.register_command(
            "ABORT", None)

        if self.base_accept is None:
            raise self.printer.command_error(
                "No active manual probe ACCEPT command found"
            )

        self.gcode.register_command(
            "ACCEPT",
            self._cmd_ACCEPT,
            desc="Accept BFP dual Z calibration position"
        )

        self.gcode.register_command(
            "NEXT",
            self._cmd_ACCEPT
        )

        self.gcode.register_command(
            "ABORT",
            self._cmd_ABORT,
            desc="Abort BFP dual Z calibration"
        )

    def _clear_handler_references(self):
        self.base_accept = None
        self.base_next = None
        self.base_abort = None

    def _accept_is_valid(self):
        toolhead = self._get_toolhead()
        current = toolhead.get_position()
        start = self.phase_start_position

        if start is None:
            return False

        # Same validity test used by Klipper ManualProbeHelper:
        # XY cannot move and Z must have been lowered from start.
        return (
            current[:2] == start[:2]
            and current[2] < start[2]
        )

    ##################################################################
    # Main PROBE_CALIBRATE replacement
    ##################################################################

    def cmd_PROBE_CALIBRATE(self, gcmd):
        if self.phase != "idle":
            raise gcmd.error(
                "BFP dual Z calibration is already running"
            )

        self._check_idex_state(gcmd)

        # Explicitly make T0 primary before calibration.
        self.gcode.run_script_from_command("T0")

        self.phase = "t0"

        self.t0_contact_z = None
        self.t1_contact_z = None
        self.reference_x = None
        self.reference_y = None

        self.gcode.respond_info(
            "BFP DUAL Z: Starting T0 manual Z calibration. "
            "Eddy probe calibration will NOT be modified."
        )

        # Start a plain manual probe for T0.
        # Unlike Klipper's original PROBE_CALIBRATE this does not
        # write a new probe_eddy_current z_offset.
        self.gcode.run_script_from_command(
            "MANUAL_PROBE SPEED=%.4f" % self.manual_speed
        )

        self.phase_start_position = list(
            self._get_toolhead().get_position()
        )

        # Replace ACCEPT/NEXT/ABORT temporarily so that accepting T0
        # automatically advances into the T1 calibration.
        self._install_manual_probe_hooks()

    ##################################################################
    # ACCEPT handler for both stages
    ##################################################################

    def _cmd_ACCEPT(self, gcmd):
        if self.phase not in ("t0", "t1"):
            raise gcmd.error(
                "BFP dual calibration ACCEPT called "
                "without an active calibration"
            )

        phase = self.phase
        valid = self._accept_is_valid()

        # Capture the exact kinematic contact location BEFORE the
        # normal Klipper ACCEPT handler finalizes the manual probe.
        kinpos = self._get_kinematic_position()

        if gcmd.get_command() == "NEXT":
            base_handler = self.base_next
        else:
            base_handler = self.base_accept

        if base_handler is None:
            raise gcmd.error(
                "Original manual probe ACCEPT handler is unavailable"
            )

        # Let Klipper finish the active MANUAL_PROBE normally.
        # No Eddy probe configuration is changed.
        base_handler(gcmd)

        self._clear_handler_references()

        if not valid:
            self.phase = "idle"
            self.gcode.respond_info(
                "BFP DUAL Z: Manual probe was not accepted; "
                "calibration stopped."
            )
            return

        if phase == "t0":
            self.t0_contact_z = float(kinpos[2])
            self.reference_x = float(kinpos[0])
            self.reference_y = float(kinpos[1])

            self.gcode.respond_info(
                "BFP DUAL Z: T0 accepted at "
                "X=%.4f Y=%.4f Z=%.4f"
                % (
                    self.reference_x,
                    self.reference_y,
                    self.t0_contact_z
                )
            )

            self._start_t1_calibration()

        elif phase == "t1":
            self.t1_contact_z = float(kinpos[2])

            self.gcode.respond_info(
                "BFP DUAL Z: T1 accepted at Z=%.4f"
                % self.t1_contact_z
            )

            self._finish_calibration()

    ##################################################################
    # Start T1 immediately after T0 ACCEPT
    ##################################################################

    def _start_t1_calibration(self):
        self.phase = "transition"

        lift_feed = int(self.lift_speed * 60.0)
        travel_feed = int(self.travel_speed * 60.0)

        script = """
G91
G1 Z%.4f F%d
G90

PARK_T0

ACTIVATE_EXTRUDER EXTRUDER=extruder1
SET_DUAL_CARRIAGE CARRIAGE=1 MODE=PRIMARY

SET_GCODE_VARIABLE MACRO=_IDEX_STATE VARIABLE=mode VALUE='"PRIMARY"'
SET_GCODE_VARIABLE MACRO=_IDEX_STATE VARIABLE=primary VALUE=1
SET_GCODE_VARIABLE MACRO=_IDEX_STATE VARIABLE=active_extruder VALUE='"extruder1"'

G1 X%.4f Y%.4f F%d

MANUAL_PROBE SPEED=%.4f
""" % (
            self.lift,
            lift_feed,
            self.reference_x,
            self.reference_y,
            travel_feed,
            self.manual_speed
        )

        self.gcode.respond_info(
            "BFP DUAL Z: T0 complete. "
            "Raising Z %.2f mm, parking T0, and positioning T1."
            % self.lift
        )

        self.gcode.run_script_from_command(script)

        # MANUAL_PROBE is now active for T1.
        self.phase = "t1"

        self.phase_start_position = list(
            self._get_toolhead().get_position()
        )

        self._install_manual_probe_hooks()

        self.gcode.respond_info(
            "BFP DUAL Z: T1 ready at T0 reference XY. "
            "Perform paper test and press ACCEPT."
        )

    ##################################################################
    # Finish T1, calculate and stage delta
    ##################################################################

    def _finish_calibration(self):
        # Eddy homing establishes the machine-coordinate Z reference.
        # Therefore the physical T0 paper-contact coordinate is the
        # persistent G-code Z offset required for T0.
        self.t0_z_offset = float(self.t0_contact_z)

        # T1 is stored relative to T0.
        self.t1_z_delta = (
            self.t1_contact_z - self.t0_contact_z
        )

        configfile = self.printer.lookup_object("configfile")

        configfile.set(
            self.config_name,
            "t0_z_offset",
            "%.6f" % self.t0_z_offset
        )

        configfile.set(
            self.config_name,
            "t1_z_delta",
            "%.6f" % self.t1_z_delta
        )

        self.gcode.respond_info(
            "\n"
            "BFP DUAL Z CALIBRATION COMPLETE\n"
            "T0 contact Z     : %.6f\n"
            "T0 print offset  : %+.6f mm\n"
            "T1 contact Z     : %.6f\n"
            "T1-T0 delta      : %+.6f mm\n"
            % (
                self.t0_contact_z,
                self.t0_z_offset,
                self.t1_contact_z,
                self.t1_z_delta
            )
        )

        # Safely get T1 off the bed and return T0 to PRIMARY.
        lift_feed = int(self.lift_speed * 60.0)

        restore_script = """
G91
G1 Z%.4f F%d
G90

PARK_T1

ACTIVATE_EXTRUDER EXTRUDER=extruder
SET_DUAL_CARRIAGE CARRIAGE=0 MODE=PRIMARY

SET_GCODE_VARIABLE MACRO=_IDEX_STATE VARIABLE=mode VALUE='"PRIMARY"'
SET_GCODE_VARIABLE MACRO=_IDEX_STATE VARIABLE=primary VALUE=0
SET_GCODE_VARIABLE MACRO=_IDEX_STATE VARIABLE=active_extruder VALUE='"extruder"'
""" % (
            self.lift,
            lift_feed
        )

        self.gcode.run_script_from_command(restore_script)

        # Apply the newly calibrated T0 offset immediately.
        # No physical move occurs here; it changes the G-code coordinate
        # system used for subsequent moves.
        self.gcode.run_script_from_command(
            "SET_GCODE_OFFSET Z=%.6f" % self.t0_z_offset
        )

        self.phase = "idle"
        self.phase_start_position = None

        if self.auto_save:
            self.gcode.respond_info(
                "BFP DUAL Z: Saving T0 print offset and "
                "T1 relative offset."
            )
            self.gcode.run_script_from_command("SAVE_CONFIG")
        else:
            self.gcode.respond_info(
                "BFP DUAL Z: Values are staged and active. "
                "Run SAVE_CONFIG to persist both offsets."
            )

    ##################################################################
    # ABORT
    ##################################################################

    def _cmd_ABORT(self, gcmd):
        phase = self.phase
        base_handler = self.base_abort

        if base_handler is not None:
            base_handler(gcmd)

        self._clear_handler_references()

        self.phase = "idle"
        self.phase_start_position = None

        self.gcode.respond_info(
            "BFP DUAL Z: Calibration aborted during %s stage."
            % phase.upper()
        )

    ##################################################################
    # Status
    ##################################################################

    def cmd_BFP_DUAL_CALIBRATION_STATUS(self, gcmd):
        gcmd.respond_info(
            "BFP Dual Z Calibration\n"
            "phase: %s\n"
            "Saved T0 print offset: %+.6f mm\n"
            "T0 contact Z: %s\n"
            "T1 contact Z: %s\n"
            "T1-T0 delta: %+.6f mm"
            % (
                self.phase,
                self.t0_z_offset,
                "not measured"
                if self.t0_contact_z is None
                else "%.6f" % self.t0_contact_z,
                "not measured"
                if self.t1_contact_z is None
                else "%.6f" % self.t1_contact_z,
                self.t1_z_delta
            )
        )

    def get_status(self, eventtime):
        return {
            "phase": self.phase,
            "t0_z_offset": self.t0_z_offset,
            "t0_contact_z": self.t0_contact_z,
            "t1_contact_z": self.t1_contact_z,
            "t1_z_delta": self.t1_z_delta,
            "reference_x": self.reference_x,
            "reference_y": self.reference_y,
        }


def load_config(config):
    return BFPDualCalibration(config)
