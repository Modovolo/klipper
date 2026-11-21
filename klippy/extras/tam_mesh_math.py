def gantry_weights(x, y, left_x, right_x, front_y, rear_y):
    u = (x - left_x) / (right_x - left_x)
    v = (y - front_y) / (rear_y - front_y)
    return [
        (1. - u) * (1. - v),
        (1. - u) * v,
        u * v,
        u * (1. - v)
    ]


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def solve_offsets(w0, w1, requested_d, authority_lambda,
                  min_condition_q, max_motor_offset):
    sw0 = [w0[0] + w0[1], w0[2] + w0[3]]
    sw1 = [w1[0] + w1[1], w1[2] + w1[3]]

    sw0_norm2 = _dot(sw0, sw0)
    if sw0_norm2 < 1.0e-15:
        raise ValueError('BFP dual tracking invalid T0 side weights')

    projection = _dot(sw1, sw0) / sw0_norm2
    p = [sw1[i] - projection * sw0[i] for i in range(2)]
    q = _dot(p, p)

    if abs(requested_d) < 1.0e-12:
        return [0., 0., 0., 0.], q, 0., 0., 0., 0., 0., 1.

    if q < min_condition_q:
        raise ValueError(
            'BFP dual tracking: geometry too poorly conditioned '
            '(q=%.6f < %.6f). Increase nozzle separation.'
            % (q, min_condition_q))

    alpha = requested_d / (q + authority_lambda)
    side_offsets = [alpha * p[0], alpha * p[1]]

    anchor_err = _dot(sw0, side_offsets)
    if abs(anchor_err) > 1.0e-15:
        corr = anchor_err / sw0_norm2
        side_offsets = [
            side_offsets[i] - corr * sw0[i]
            for i in range(2)
        ]

    offsets = [
        side_offsets[0], side_offsets[0],
        side_offsets[1], side_offsets[1]
    ]

    raw_max = max(abs(v) for v in offsets)
    hard_scale = 1.
    if raw_max > max_motor_offset:
        hard_scale = max_motor_offset / raw_max
        offsets = [v * hard_scale for v in offsets]

    t0_achieved = _dot(w0, offsets)
    t1_achieved = _dot(w1, offsets)
    achieved_d = t1_achieved - t0_achieved
    residual = requested_d - achieved_d
    authority = achieved_d / requested_d
    gain = max(abs(v) for v in offsets) / abs(requested_d)

    return (
        offsets, q, achieved_d, residual, authority, gain,
        raw_max, hard_scale
    )