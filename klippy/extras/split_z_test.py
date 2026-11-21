from . import z_tilt


class BFPSplitZTest:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')

        self.speed = config.getfloat('speed', 2.0, above=0.0)
        self.max_step = config.getfloat('max_step', 2.0, above=0.0)
        self.max_total_offset = config.getfloat(
            'max_total_offset', 5.0, above=0.0)

        self.z_helper = z_tilt.ZAdjustHelper(config, 4)

        # Accumulated test offsets by actual Klipper stepper name.
        self.offsets = {
            'stepper_z': 0.0,
            'stepper_z1': 0.0,
            'stepper_z2': 0.0,
            'stepper_z3': 0.0,
        }

        self.gcode.register_command(
            'BFP_SPLIT_Z_TEST',
            self.cmd_BFP_SPLIT_Z_TEST,
            desc='Move individual BFP Z motors or motor groups'
        )

        self.gcode.register_command(
            'BFP_SPLIT_Z_RESET',
            self.cmd_BFP_SPLIT_Z_RESET,
            desc='Return all BFP Z motors to the test baseline'
        )

        self.gcode.register_command(
            'BFP_SPLIT_Z_STATUS',
            self.cmd_BFP_SPLIT_Z_STATUS,
            desc='Show accumulated BFP Z motor test offsets'
        )

    def _check_ready(self):
        toolhead = self.printer.lookup_object('toolhead')
        eventtime = self.printer.get_reactor().monotonic()
        status = toolhead.get_status(eventtime)

        if 'z' not in status.get('homed_axes', ''):
            raise self.gcode.error(
                'BFP split Z test requires Z to be homed first'
            )

        if len(self.z_helper.z_steppers) != 4:
            raise self.gcode.error(
                'BFP split Z test requires exactly four Z steppers'
            )

        names = [s.get_name() for s in self.z_helper.z_steppers]

        expected = {
            'stepper_z',
            'stepper_z1',
            'stepper_z2',
            'stepper_z3',
        }

        if set(names) != expected:
            raise self.gcode.error(
                'Unexpected Z steppers: %s' % ', '.join(names)
            )

    def _build_adjustments(self, selected, delta):
        adjustments = []

        for stepper in self.z_helper.z_steppers:
            name = stepper.get_name()

            if name in selected:
                adjustments.append(delta)
            else:
                adjustments.append(0.0)

        return adjustments

    def _check_total_limits(self, selected, delta):
        for name in selected:
            new_value = self.offsets[name] + delta

            if abs(new_value) > self.max_total_offset:
                raise self.gcode.error(
                    '%s would exceed maximum test offset of %.3f mm'
                    % (name, self.max_total_offset)
                )

    def _apply(self, selected, delta):
        self._check_total_limits(selected, delta)

        adjustments = self._build_adjustments(
            selected,
            delta
        )

        self.z_helper.adjust_steppers(
            adjustments,
            self.speed
        )

        for name in selected:
            self.offsets[name] += delta

    def cmd_BFP_SPLIT_Z_TEST(self, gcmd):
        self._check_ready()

        target = gcmd.get('MOTOR', '').upper()
        distance = gcmd.get_float('DISTANCE', None)

        if distance is None:
            raise gcmd.error(
                'Use DISTANCE=<mm>'
            )

        if abs(distance) > self.max_step:
            raise gcmd.error(
                'Maximum movement per command is %.3f mm'
                % self.max_step
            )

        targets = {
            'Z0': {'stepper_z'},
            'Z1': {'stepper_z1'},
            'Z2': {'stepper_z2'},
            'Z3': {'stepper_z3'},

            # These are the EXPECTED groups.
            # Verify individual motors before using them.
            'LEFT': {
                'stepper_z',
                'stepper_z1',
            },

            'RIGHT': {
                'stepper_z2',
                'stepper_z3',
            },
        }

        if target not in targets:
            raise gcmd.error(
                'MOTOR must be Z0, Z1, Z2, Z3, LEFT, or RIGHT'
            )

        self._apply(
            targets[target],
            distance
        )

        gcmd.respond_info(
            'BFP Z MOTOR TEST\n'
            'Target: %s\n'
            'Movement: %+.3f mm'
            % (target, distance)
        )

        self._report_status(gcmd)

    def cmd_BFP_SPLIT_Z_RESET(self, gcmd):
        self._check_ready()

        adjustments = []

        for stepper in self.z_helper.z_steppers:
            name = stepper.get_name()
            adjustments.append(
                -self.offsets[name]
            )

        if max(abs(v) for v in adjustments) < 0.000001:
            gcmd.respond_info(
                'BFP Z motor test already at baseline'
            )
            return

        self.z_helper.adjust_steppers(
            adjustments,
            self.speed
        )

        for name in self.offsets:
            self.offsets[name] = 0.0

        gcmd.respond_info(
            'BFP Z motors returned to test baseline'
        )

    def _report_status(self, gcmd):
        gcmd.respond_info(
            'BFP Z Motor Test Offsets\n'
            'Z0 / stepper_z  = %+.3f mm\n'
            'Z1 / stepper_z1 = %+.3f mm\n'
            'Z2 / stepper_z2 = %+.3f mm\n'
            'Z3 / stepper_z3 = %+.3f mm'
            % (
                self.offsets['stepper_z'],
                self.offsets['stepper_z1'],
                self.offsets['stepper_z2'],
                self.offsets['stepper_z3'],
            )
        )

    def cmd_BFP_SPLIT_Z_STATUS(self, gcmd):
        self._check_ready()
        self._report_status(gcmd)


def load_config(config):
    return BFPSplitZTest(config)
