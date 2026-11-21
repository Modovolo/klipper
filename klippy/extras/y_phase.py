"""Nearest representable full-step reference at unchanged 64 microsteps.

No coarse full-step mode. At most 32 signed microsteps (0.1mm) per side.
Imported by y_square; never performs actions during import/startup.
"""
FULL = (128, 384, 640, 896)
PAIRS = {'left': ('stepper_y', 'stepper_y1'),
         'right': ('stepper_y2', 'stepper_y3')}


def delta(target, source):
    return (target - source + 512) % 1024 - 512


def plan(phase, inverted):
    """Nearest reachable reference on this motor's four-count step lattice."""
    residue = phase % 4
    bias = residue if residue <= 2 else residue - 4
    candidates = [(p + bias) % 1024 for p in FULL]
    raw_per_step = 4 if inverted else -4
    target = min(candidates, key=lambda p: (abs(delta(p, phase)),
                                            delta(p, phase) / raw_per_step < 0))
    difference = delta(target, phase)
    if difference % 4:
        raise ValueError('reference not reachable on 64-microstep lattice')
    steps = difference // raw_per_step
    if abs(steps) > 32:
        raise ValueError('phase plan exceeds 0.1mm')
    return {'target': target, 'steps': steps, 'correction_mm': steps * .003125,
            'reference_error_mm': min(abs(delta(target, p)) for p in FULL) * .2 / 256.}


def verify_reference(owner, rows, plans, label):
    for side, names in PAIRS.items():
        for n in names:
            row = rows[n]
            if min(abs(delta(row['mscnt'], p)) for p in FULL) > 2:
                raise owner.error(label + ': phase is not a nearest full-step reference: ' + n)
            if abs(delta(row['mscnt'], plans[side]['target'])) > 2:
                raise owner.error(label + ': phase target was not retained: ' + n)
            # Real LUT entries are asymmetric by up to two counts even at an
            # exact full step. Allow the additional 2/256-step lattice offset.
            a, b = row['cur_a'], row['cur_b']
            if min(abs(a), abs(b)) < 100 or abs(abs(a) - abs(b)) > 8:
                raise owner.error(label + ': current vector does not match near-full-step phase: ' + n)


def clock_pair(owner, head, steps, signed_count):
    if not isinstance(signed_count, int) or abs(signed_count) > 32:
        raise owner.error('phase pulse count outside 32-microstep bound')
    if signed_count == 0:
        return dict.fromkeys(steps, 0)
    previous, starts, errors = [], {}, []
    try:
        head.flush_step_generation()
        for n, step in steps.items():
            previous.append((step, step.get_stepper_kinematics(), step.get_trapq()))
            starts[n] = step.get_mcu_position()
            step.set_stepper_kinematics(owner.phase_solvers[n])
            step.set_trapq(owner.phase_trapq)
            step.set_position((0., 0., 0.))
        duration = .15
        direction = 1. if signed_count > 0 else -1.
        speed = abs(signed_count) * .003125 / duration
        when = head.get_last_move_time()
        owner.phase_append(owner.phase_trapq, when, 0., duration, 0.,
                           0., 0., 0., direction, 0., 0., 0., speed, 0.)
        owner.motion_queuing.note_mcu_movequeue_activity(when + duration)
        head.dwell(duration)
        head.flush_step_generation()
        head.wait_moves()
        counts = {}
        for n, step in steps.items():
            step._query_mcu_position()
            counts[n] = step.get_mcu_position() - starts[n]
        if any(value != signed_count for value in counts.values()):
            raise owner.error('phase microstep-count mismatch: %s' % counts)
        return counts
    finally:
        for step, solver, queue in previous:
            try:
                step.set_trapq(queue)
                step.set_stepper_kinematics(solver)
            except Exception as exc:
                errors.append(str(exc))
        try:
            owner.motion_queuing.wipe_trapq(owner.phase_trapq)
        except Exception as exc:
            errors.append(str(exc))
        if errors:
            owner.printer.invoke_shutdown('BFP Y phase microstep cleanup failed')
            raise owner.error('phase pulse cleanup failed: ' + '; '.join(errors))


def align(owner, hw):
    head, kin, rail, steps, ends, drivers = hw
    head.wait_moves()
    before = owner.phases(drivers)
    owner.phase_trace = {'method': 'nearest_reference_64', 'before': before}
    for n, row in before.items():
        if row['mres'] != 2 or row['intpol'] != 1:
            raise owner.error('expected normal MRES2/intpol1 before phase alignment')
        if drivers[n].fields.registers['CHOPCONF'] != row['chopconf']:
            raise owner.error('driver cache/hardware mismatch: ' + n)
    detached, touched, restoration_errors = [], [], []
    plans, counts = {}, {}
    try:
        head.flush_step_generation()
        for step in steps.values():
            detached.append((step, step.set_trapq(None)))
        # Keep MRES=2 throughout. Only interpolation is temporarily disabled.
        for n, driver in drivers.items():
            touched.append(n)
            driver.mcu_tmc.set_register('CHOPCONF', before[n]['chopconf'] & ~(1 << 28))
        disabled = owner.phases(drivers)
        owner.phase_trace['interpolation_disabled'] = disabled
        for row in disabled.values():
            if row['mres'] != 2 or row['intpol'] != 0:
                raise owner.error('interpolation-disable readback failed')
        for side, names in PAIRS.items():
            first = names[0]
            plans[side] = plan(disabled[first]['mscnt'], steps[first].get_dir_inverted()[0])
        owner.phase_trace['plans'] = plans
        owner.phase_trace['microstep_counts'] = counts
        for side, names in PAIRS.items():
            counts.update(clock_pair(owner, head, {n: steps[n] for n in names}, plans[side]['steps']))
            # Keep the held side detached throughout the other side's motion.
            if any(owner.inputs(head, ends).values()):
                raise owner.error('Y endstop activated during phase alignment')
        aligned = owner.phases(drivers)
        owner.phase_trace['aligned'] = aligned
        verify_reference(owner, aligned, plans, 'before interpolation restore')
    finally:
        for n in touched:
            try:
                drivers[n].mcu_tmc.set_register('CHOPCONF', before[n]['chopconf'])
                if drivers[n].mcu_tmc.get_register('CHOPCONF') != before[n]['chopconf']:
                    raise owner.error('CHOPCONF restore mismatch')
            except Exception as exc:
                restoration_errors.append('%s: %s' % (n, exc))
        if not restoration_errors:
            for n, driver in drivers.items():
                try:
                    driver.get_phase_offset.__self__._handle_sync_mcu_pos(steps[n])
                except Exception as exc:
                    restoration_errors.append('%s phase cache: %s' % (n, exc))
        for step, queue in detached:
            try:
                step.set_trapq(queue)
            except Exception as exc:
                restoration_errors.append('queue: %s' % exc)
        if restoration_errors:
            owner.printer.invoke_shutdown('BFP Y phase restoration failed: ' + '; '.join(restoration_errors))
            raise owner.error('restoration failed; printer shut down')
    after = owner.phases(drivers)
    owner.phase_trace['after'] = after
    verify_reference(owner, after, plans, 'after interpolation restore')
    for n, row in after.items():
        if row['mres'] != 2 or row['intpol'] != 1:
            raise owner.error('normal driver settings not restored: ' + n)
        direction = 4 if steps[n].get_dir_inverted()[0] else -4
        if abs(delta(row['mscnt'], before[n]['mscnt']) - counts[n] * direction) > 2:
            raise owner.error('driver phase change does not match signed MCU step count: ' + n)
    correction = {n: value * .003125 for n, value in counts.items()}
    return {'method': 'nearest_reference_64', 'before': before, 'aligned': aligned,
            'after': after, 'plans': plans, 'microstep_counts': counts,
            'estimated_correction_mm': correction,
            'maximum_reference_error_mm': .0015625}
