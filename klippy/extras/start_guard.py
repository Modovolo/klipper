class BFPStartGuard:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')

        self.gcode.register_command(
            'BFP_START_STEP',
            self.cmd_BFP_START_STEP,
            desc='Run a guarded BFP START_PRINT preparation step'
        )

    def _cancel_failed_start(self, step, error):
        self.gcode.respond_info(
            'START_PRINT FAILED during %s: %s\n'
            'Cancelling print.' % (step, error)
        )

        # Startup is deliberately paused while the operator selects a mode.
        # A failure in a button-triggered macro therefore does not reliably
        # reach virtual_sdcard on_error_gcode. Clear the job explicitly.
        cleanup = [
            'BFP_DUAL_TRACKING ENABLE=0',
            'TURN_OFF_HEATERS',
            'BED_MESH_CLEAR',
            'CLEAR_PAUSE',
            'CANCEL_PRINT_BASE',
        ]

        for command in cleanup:
            try:
                self.gcode.run_script_from_command(command)
            except Exception:
                # Continue attempting the remaining cleanup commands.
                pass

    def cmd_BFP_START_STEP(self, gcmd):
        step = gcmd.get('STEP').strip().upper()

        scripts = {
            'HOME': 'G28',
            'QGL': 'QUAD_GANTRY_LEVEL',
            'HOME_Z': 'G28 Z',
            'MESH': 'BED_MESH_CLEAR\nRapid_Bed_Scan',
        }

        if step not in scripts:
            raise gcmd.error(
                'BFP_START_STEP STEP must be HOME, QGL, HOME_Z, or MESH'
            )

        try:
            self.gcode.run_script_from_command(scripts[step])
        except Exception as e:
            self._cancel_failed_start(step, e)
            raise


def load_config(config):
    return BFPStartGuard(config)
