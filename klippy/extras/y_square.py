"""BFP V2 supervised Y squaring. No startup motion; G28 is not overridden.

All distances are commanded distances, not encoder measurements. Mechanical
starting alignment and endstop placement must be validated by the operator.
"""
import logging
import chelper
from . import homing
from . import y_phase

GROUPS = {'left': ('stepper_y', 'stepper_y1'),
          'right': ('stepper_y2', 'stepper_y3')}
NAMES = tuple(n for pair in GROUPS.values() for n in pair)
FULL_PHASES = (128, 384, 640, 896)


def circular_delta(target, source):
    return (target - source + 512) % 1024 - 512


def full_vector_phase(a, b):
    # MSCURACT is the LUT current vector, not a rotor encoder.
    # The installed LUT reports e.g. 174/-176 at exact MSCNT384.
    if min(abs(a), abs(b)) < 100 or abs(abs(a) - abs(b)) > 2:
        return None
    return {(True, True): 128, (True, False): 384,
            (False, False): 640, (False, True): 896}[(a > 0, b > 0)]


class TriggerRecord:
    def __init__(self, endstop):
        self.endstop = endstop
        self.triggered = False

    def __getattr__(self, name):
        return getattr(self.endstop, name)

    def home_wait(self, end_time):
        value = self.endstop.home_wait(end_time)
        self.triggered = value > 0.
        return value


class BFPYSquare:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.commissioned = config.getboolean('commissioned', False)
        self.last_result = None
        self.busy = False
        self.test_used = False
        self.phase_trace = None
        self.motion_queuing = self.printer.load_object(config, 'motion_queuing')
        self.phase_trapq = self.motion_queuing.allocate_trapq()
        self.phase_append = self.motion_queuing.lookup_trapq_append()
        ffi_main, ffi_lib = chelper.get_ffi()
        self.phase_solvers = {n: ffi_main.gc(ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free)
                              for n in NAMES}
        self.max_delta = config.getfloat(
            'max_skew_mm', 3.0, above=0.0, maxval=50.0)
        self.section = config.getfloat(
            'section_distance', 2., above=0., maxval=10.)
        self.tracking_tolerance = config.getfloat(
            'tracking_tolerance', self.section + .05, above=0.,
            maxval=self.section + .1)
        self.approach_speed = config.getfloat(
            'approach_speed', 20., above=0., maxval=200.)
        self.slow_speed = config.getfloat(
            'slow_speed', 2., above=0., maxval=50.)
        self.backoff_speed = config.getfloat(
            'backoff_speed', 5., above=0., maxval=50.)
        self.home_travel = config.getfloat(
            'home_travel', 866., above=0., maxval=900.)
        self.backoff = 2.0
        for name, method, desc in (
                # Preferred generic command names
                ('Y_SQUARE_REPORT', self.cmd_report, 'Read-only Y groups and driver phase report'),
                ('Y_PHASE_TEST', self.cmd_phase, 'Nearest full-step reference at 64 microsteps; invalidates Y'),
                ('Y_SQUARE_TEST', self.cmd_test, 'Bounded Y squaring and phase test; leaves Y unhomed'),
                ('Y_SQUARE_HOME', self.cmd_home, 'Commissioned Y-only square home'),
                # Backwards-compatible command names
                ('BFP_Y_SQUARE_REPORT', self.cmd_report, 'Read-only Y groups and driver phase report'),
                ('BFP_Y_PHASE_TEST', self.cmd_phase, 'Nearest full-step reference at 64 microsteps; invalidates Y'),
                ('BFP_Y_SQUARE_TEST', self.cmd_test, 'Bounded Y squaring and phase test; leaves Y unhomed'),
                ('BFP_Y_SQUARE_HOME', self.cmd_home, 'Commissioned Y-only square home')):
            self.gcode.register_command(name, method, desc=desc)

    def error(self, message):
        return self.printer.command_error('BFP Y: ' + message)

    def hardware(self):
        head = self.printer.lookup_object('toolhead')
        kin = head.get_kinematics()
        rail = kin.rails[1]
        steps = {s.get_name(): s for s in rail.get_steppers()}
        if set(steps) != set(NAMES):
            raise self.error('unexpected Y motor names')
        ends = {}
        for es, name in rail.get_endstops():
            group = frozenset(s.get_name() for s in es.get_steppers())
            matches = [side for side, names in GROUPS.items() if group == frozenset(names)]
            if len(matches) != 1 or matches[0] in ends:
                raise self.error('endstop motor groups do not match verified pairs')
            ends[matches[0]] = (es, name)
        if set(ends) != set(GROUPS) or ends['left'][0] is ends['right'][0]:
            raise self.error('two independent Y endstops required')
        if len({s.get_mcu() for s in steps.values()}) != 1:
            raise self.error('Y motors must share one MCU')
        for side, names in GROUPS.items():
            if steps[names[0]].get_dir_inverted()[0] != steps[names[1]].get_dir_inverted()[0]:
                raise self.error(side + ' pair has differing direction polarity; phase procedure needs calibration')
        if rail.get_homing_info().positive_dir:
            raise self.error('expected front/negative Y home')
        drivers = {n: self.printer.lookup_object('tmc5160 ' + n) for n in NAMES}
        for n, s in steps.items():
            if abs(s.get_step_dist() - .003125) > 1.e-9:
                raise self.error('unexpected Y step distance: ' + n)
        return head, kin, rail, steps, ends, drivers

    def read_driver(self, driver):
        f = driver.fields
        def reg(name):
            return driver.mcu_tmc.get_register(name)
        chop = reg('CHOPCONF')
        phase = reg('MSCNT') & 1023
        current = reg('MSCURACT')
        status = reg('DRV_STATUS')
        for fault in ('ot', 's2ga', 's2gb', 's2vsa', 's2vsb'):
            if f.lookup_register(fault, None) is not None and f.get_field(fault, status):
                raise self.error('driver fault: ' + fault)
        return {'chopconf': chop, 'mres': f.get_field('mres', chop),
                'intpol': f.get_field('intpol', chop), 'mscnt': phase,
                'cur_a': f.get_field('cur_a', current),
                'cur_b': f.get_field('cur_b', current)}

    def phases(self, drivers):
        data = {n: self.read_driver(d) for n, d in drivers.items()}
        for side, names in GROUPS.items():
            a, b = (data[n] for n in names)
            if abs(circular_delta(a['mscnt'], b['mscnt'])) > 1:
                raise self.error(side + ' motor phases disagree; do not recalibrate pulleys in software')
            if (a['cur_a'], a['cur_b']) != (b['cur_a'], b['cur_b']):
                raise self.error(side + ' current vectors disagree')
        return data

    def inputs(self, head, ends):
        when = head.get_last_move_time()
        return {side: bool(es.query_endstop(when)) for side, (es, name) in ends.items()}

    def cmd_report(self, gcmd):
        if self.busy:
            raise self.error('transaction active')
        head, kin, rail, steps, ends, drivers = self.hardware()
        head.wait_moves()
        rows = self.phases(drivers)
        plans = {side: y_phase.plan(rows[names[0]]['mscnt'], steps[names[0]].get_dir_inverted()[0])
                 for side, names in GROUPS.items()}
        result = {'inputs': self.inputs(head, ends), 'drivers': rows, 'next_phase_plan': plans,
                  'commissioned': self.commissioned, 'last_result': self.last_result}
        gcmd.respond_info('BFP Y REPORT: %s' % result)

    def begin(self, gcmd, commissioning=True):
        if gcmd.get_int('ARM', 0) != 1:
            raise self.error('requires ARM=1 and an operator ready at the printer')
        if self.busy or (commissioning and self.test_used):
            raise self.error('busy or commissioning attempt already used; inspect before restarting')
        if gcmd.get_int('START_ALIGNED', 0) != 1:
            raise self.error('requires START_ALIGNED=1: initial mechanical skew at most 1mm')
        now = self.printer.get_reactor().monotonic()
        print_state = self.printer.lookup_object('print_stats').get_status(now)['state']
        # START_PRINT pauses for operator mode selection before homing.
        # Allow production homing while paused for that flow and layer-shift recovery.
        if print_state == 'printing':
            if commissioning:
                raise self.error('commissioning refused during printing')
            raise self.error('homing refused during printing')
        if commissioning and print_state == 'paused':
            raise self.error('commissioning refused while paused')
        tam = self.printer.lookup_object('dual_mesh', None)
        if tam is not None and getattr(tam, 'enabled', False):
            raise self.error('disable TAM first')
        hw = self.hardware()
        head, kin, rail, steps, ends, drivers = hw
        head.wait_moves()
        homed_axes = head.get_status(self.printer.get_reactor().monotonic())['homed_axes']
        if 'x' not in homed_axes:
            raise self.error('home X before starting Y squaring')
        if any(self.inputs(head, ends).values()):
            raise self.error('both Y switches must start released')
        # Read-only validation before any enable or movement.
        data = self.phases(drivers)
        for n, row in data.items():
            if row['mres'] != 2 or row['intpol'] != 1:
                raise self.error('expected original 64-microstep/interpolated setting: ' + n)
        self.busy = True
        self.test_used = True
        self.last_result = {'state': 'running'}
        kin.clear_homing_state('y')
        return hw

    def enable_and_settle(self, head, steps):
        self.printer.lookup_object('stepper_enable').set_motors_enable(list(steps), True)
        head.dwell(.25)
        head.wait_moves()

    def phase_transaction(self, hw):
        return y_phase.align(self, hw)

    def move_section(self, hw, active, distance, speed, ledger, toward=True):
        head, kin, rail, steps, ends, drivers = hw
        if not active or distance <= 0. or distance > self.section + 1.e-8:
            raise self.error('invalid movement section')

        if abs(ledger['left'] - ledger['right']) + distance > self.max_delta + 1.e-8:
            raise self.error('commanded skew allowance exhausted')
        target = list(head.get_position())
        target[1] += -distance if toward else distance
        start_y = target[1] - (-distance if toward else distance)
        detached = []
        try:
            head.flush_step_generation()
            for side, names in GROUPS.items():
                if side not in active:
                    for n in names:
                        detached.append((steps[n], steps[n].set_trapq(None)))
            recorded = [(TriggerRecord(ends[side][0]), side) for side in active]
            move = homing.HomingMove(self.printer, recorded, head)
            move.homing_move(target, speed, check_triggered=False, triggered=toward)
            travel = {sp.stepper_name: abs(sp.halt_pos - sp.start_pos) * sp.stepper.get_step_dist()
                      for sp in move.stepper_positions}
            active_travel = [sum(travel[n] for n in GROUPS[side]) / len(GROUPS[side])
                             for side in active]
            tracked_distance = sum(active_travel) / len(active_travel)
            tracking_drift = abs(distance - tracked_distance)
            if tracking_drift > self.tracking_tolerance:
                raise self.error('section position tracking drift %.3fmm exceeds %.3fmm tolerance'
                                 % (tracking_drift, self.tracking_tolerance))
            corrected = list(head.get_position())
            corrected[1] = start_y + (-1. if toward else 1.) * tracked_distance
            head.set_position(corrected)
            for side in active:
                values = [travel[n] for n in GROUPS[side]]
                if abs(values[0] - values[1]) > .01 or max(values) > distance + .05:
                    raise self.error('unexpected motor-pair travel')
                ledger[side] += (-1. if toward else 1.) * sum(values) / 2.
            if abs(ledger['left'] - ledger['right']) > self.max_delta + .05:
                raise self.error('measured MCU step difference exceeded allowance')
            return {side for es, side in recorded if es.triggered}
        finally:
            try:
                head.flush_step_generation()
            finally:
                errors = []
                for s, queue in detached:
                    try:
                        s.set_trapq(queue)
                    except Exception as exc:
                        errors.append(str(exc))
                if errors:
                    self.printer.invoke_shutdown('BFP Y movement queue restoration failed')
                    raise self.error('movement queue restoration failed: ' + '; '.join(errors))

    def latch(self, hw, maximum, speed, ledger, section=None):
        pending = set(GROUPS)
        remaining = maximum
        order = []
        while pending and remaining > 1.e-6:
            allowance = self.max_delta - abs(ledger['left'] - ledger['right'])
            distance = min(self.section if section is None else section, remaining, allowance)
            if distance < .01:
                raise self.error('second switch did not trigger within skew allowance')
            hit = self.move_section(hw, pending, distance, speed, ledger)
            if hit:
                order.append(sorted(hit))
                pending -= hit
            remaining -= distance
        if pending:
            raise self.error('homing travel exhausted before both switches triggered')
        return order

    def latch_continuous(self, hw, maximum, speed, ledger):
        # Single continuous homing move (no start/stop sections). This preserves
        # the classic "one smooth push" feel while still doing dual-endstop
        # racking correction.
        head, kin, rail, steps, ends, drivers = hw
        if maximum <= 0.0:
            raise self.error('invalid latch travel')

        rail_min, rail_max = rail.get_range()
        before = list(head.get_position())

        if getattr(self, '_debug', False):
            self.gcode.respond_info(
                'BFP Y DEBUG: latch_continuous start rail_min=%.3f rail_max=%.3f before_y=%.3f maximum=%.3f'
                % (rail_min, rail_max, before[1], maximum))

        # Ensure the commanded target stays within [rail_min, rail_max] by
        # inflating the reported coordinate (no physical motion).
        required_start_y = rail_min + maximum + 1.0
        if required_start_y > rail_max + 1.e-6:
            raise self.error('configured Y range too small for latch travel')
        if before[1] < required_start_y - 1.e-6:
            head.flush_step_generation()
            before[1] = required_start_y
            head.set_position(before, homing_axes='y')

        if getattr(self, '_debug', False):
            self.gcode.respond_info(
                'BFP Y DEBUG: latch_continuous after_inflate before_y=%.3f required_start_y=%.3f'
                % (before[1], required_start_y))

        target = list(before)
        target[1] += -maximum

        if getattr(self, '_debug', False):
            self.gcode.respond_info(
                'BFP Y DEBUG: latch_continuous target_y=%.3f'
                % (target[1],))

        head.flush_step_generation()
        recorded = [(TriggerRecord(ends[side][0]), side) for side in GROUPS]
        move = homing.HomingMove(self.printer, recorded, head)
        move.homing_move(target, speed, check_triggered=False, triggered=True)

        travel = {sp.stepper_name: abs(sp.halt_pos - sp.start_pos) * sp.stepper.get_step_dist()
                  for sp in move.stepper_positions}
        side_travel = {}
        for side in GROUPS:
            values = [travel[n] for n in GROUPS[side]]
            if abs(values[0] - values[1]) > .01:
                raise self.error('unexpected motor-pair travel')
            side_travel[side] = sum(values) / 2.

        for side in GROUPS:
            ledger[side] += -side_travel[side]

        if abs(ledger['left'] - ledger['right']) > self.max_delta + .05:
            raise self.error('measured MCU step difference exceeded allowance')

        if not all(es.triggered for es, _ in recorded):
            raise self.error('homing travel exhausted before both switches triggered')

        # Order estimate: whichever side traveled less hit first.
        if abs(side_travel['left'] - side_travel['right']) < 1.0e-6:
            return [['left', 'right']]
        if side_travel['left'] < side_travel['right']:
            return [['left'], ['right']]
        return [['right'], ['left']]

    def back_away(self, hw, ledger):
        # Release monitoring must not stop the motors immediately on a released
        # switch. Backoff uses an ordinary synchronized move, bounded to 2mm.
        head = hw[0]
        before = list(head.get_position())
        target = list(before)
        target[1] += self.backoff
        head.move(target, self.backoff_speed)
        head.wait_moves()
        for side in GROUPS:
            ledger[side] += self.backoff
        if any(self.inputs(head, hw[4]).values()):
            raise self.error('Y switch still triggered after 2mm backoff')

    def cmd_phase(self, gcmd):
        if gcmd.get_int('ALIGN64', 0) != 1:
            raise self.error('phase test requires ALIGN64=1; up to 0.1mm either direction per side')
        hw = self.begin(gcmd)
        try:
            self.enable_and_settle(hw[0], hw[3])
            if any(self.inputs(hw[0], hw[4]).values()):
                raise self.error('switch triggered before phase test')
            result = self.phase_transaction(hw)
            self.last_result = {'state': 'phase_test_complete', 'phase': result, 'homed': False}
            gcmd.respond_info('BFP Y PHASE TEST: %s; Y remains unhomed' % result)
        except Exception as exc:
            self.last_result = {'state': 'failed', 'stage': 'phase', 'error': str(exc)}
            logging.exception('BFP Y PHASE TEST FAILED')
            raise
        finally:
            hw[1].clear_homing_state('y')
            self.busy = False

    def cmd_test(self, gcmd):
        self.square(gcmd, establish_home=False)

    def cmd_home(self, gcmd):
        if not self.commissioned:
            raise self.error('production home disabled until mechanical/phase commissioning is complete')
        self.square(gcmd, establish_home=True)

    def square(self, gcmd, establish_home):
        if not self.commissioned:
            raise self.error('Y square motion is disabled until endstop stopping is validated')
        if gcmd.get_int('PHASE_ALIGN64', 0) != 1:
            raise self.error('requires PHASE_ALIGN64=1 acknowledgement of up to 0.1mm phase movement')
        maximum = gcmd.get_float('TRAVEL', self.home_travel if establish_home else 100.,
                                above=0., maxval=self.home_travel if establish_home else 150.)
        section = gcmd.get_float('SECTION', self.section, above=0., maxval=self.section)
        smooth = gcmd.get_int('SMOOTH', 0) == 1
        if smooth:
            raise self.error('SMOOTH=1 is disabled; dual-endstop homing requires sectioned moves (use SMOOTH=0)')
        bias = gcmd.get('BIAS', 'NONE').upper()
        if bias not in ('NONE', 'LEFT', 'RIGHT'):
            raise self.error('BIAS must be NONE, LEFT or RIGHT')
        if establish_home and bias != 'NONE':
            raise self.error('deliberate side bias is commissioning-only')
        self._debug = gcmd.get_int('DEBUG', 0) == 1
        hw = self.begin(gcmd, commissioning=not establish_home)
        head, kin, rail, steps, ends, drivers = hw
        position = list(head.get_position())
        ledger = dict.fromkeys(GROUPS, 0.)
        slow_maximum = self.backoff + self.max_delta
        success = False
        stage = 'enable'
        try:
            if maximum > rail.get_range()[1] - rail.get_homing_info().position_endstop + 1.e-8:
                raise self.error('requested travel exceeds configured Y homing span')
            self.enable_and_settle(head, steps)
            synthetic = list(position)
            synthetic[1] = min(rail.get_homing_info().position_endstop + maximum + 10.,
                               rail.get_range()[1])
            head.set_position(synthetic, homing_axes='y')
            if bias != 'NONE':
                stage = 'commissioning_bias'
                # Fixed bounded nudge, counted in the SAME cumulative skew
                # ledger. Never reset that allowance after introducing bias.
                hit = self.move_section(hw, {bias.lower()}, .5, 2., ledger)
                if hit:
                    raise self.error('switch reached during 0.5mm bias; inspect before continuing')
            stage = 'first_latch'
            first = (self.latch_continuous(hw, maximum, self.approach_speed, ledger)
                     if smooth else self.latch(hw, maximum, self.approach_speed, ledger, section))
            stage = 'release'
            self.back_away(hw, ledger)

            stage = 'slow_latch'
            second = (self.latch_continuous(hw, slow_maximum, self.slow_speed, ledger)
                      if smooth else self.latch(hw, slow_maximum, self.slow_speed, ledger))
            stage = 'phase_clearance'
            self.back_away(hw, ledger)
            stage = 'phase'
            phase = self.phase_transaction(hw)
            # The phase-aligned endpoint establishes a fixed cartesian reference.
            # Per-run correction removes switch-timing variation; adding it to
            # this coordinate would reintroduce that variation into the origin.
            # Endstop placement/full-step grids still require mechanical calibration.
            final = list(position)
            final[1] = rail.get_homing_info().position_endstop + self.backoff
            head.set_position(final, homing_axes='y' if establish_home else '')
            self.last_result = {'state': 'complete', 'homed': establish_home,
                                'commissioning_bias': bias, 'approach_section': section,
                                'first_order': first, 'slow_order': second,
                                'signed_travel_mm': ledger, 'phase': phase}
            success = True
            logging.info('BFP Y SQUARE COMPLETE: %s', self.last_result)
            gcmd.respond_info('BFP Y SQUARE: %s' % self.last_result)
        except Exception as exc:
            self.last_result = {'state': 'failed', 'stage': stage,
                                'signed_travel_mm': dict(ledger), 'error': str(exc)}
            logging.exception('BFP Y SQUARE FAILED')
            raise
        finally:
            self._debug = False
            try:
                head.flush_step_generation()
            finally:
                try:
                    if not success:
                        head.set_position(position)
                finally:
                    try:
                        if not (success and establish_home):
                            kin.clear_homing_state('y')
                    finally:
                        self.busy = False

    def get_status(self, eventtime):
        return {'commissioned': self.commissioned, 'busy': self.busy,
                'test_used': self.test_used, 'last_result': self.last_result,
                'phase_trace': self.phase_trace,
                'profile': {'approach_speed': self.approach_speed,
                            'slow_speed': self.slow_speed,
                            'backoff_speed': self.backoff_speed,
                            'section_distance': self.section,
                            'tracking_tolerance': self.tracking_tolerance,
                            'home_travel': self.home_travel,
                            'maximum_commanded_skew': self.max_delta}}


def load_config(config):
    return BFPYSquare(config)
