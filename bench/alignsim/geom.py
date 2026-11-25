"""2D pose math. Poses are plain (x, y, theta) tuples for speed in the hot loop."""

import math

TAU = 2.0 * math.pi


def wrap(a):
    """Angle to (-pi, pi]."""
    return (a + math.pi) % TAU - math.pi


def compose(a, b):
    """a (+) b : apply b in a's frame (WPILib Pose2d.transformBy)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def relative(a, b):
    """b expressed in a's frame (WPILib b.relativeTo(a))."""
    return compose(inverse(a), b)


def rot(v, th):
    c, s = math.cos(th), math.sin(th)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1])


def twist_exp(dx, dy, dth):
    """WPILib Twist2d.exp(): pose change for a constant-curvature move."""
    if abs(dth) < 1e-9:
        s, c = 1.0 - dth * dth / 6.0, dth / 2.0
    else:
        s, c = math.sin(dth) / dth, (1.0 - math.cos(dth)) / dth
    return (dx * s - dy * c, dx * c + dy * s, dth)


def twist_log(t):
    """WPILib Transform2d.log(): inverse of twist_exp."""
    dth = t[2]
    half = dth / 2.0
    cos_minus_one = math.cos(dth) - 1.0
    if abs(cos_minus_one) < 1e-9:  # WPILib's guard: 1 - cos underflows long before dth is tiny
        hcot = 1.0 - dth * dth / 12.0
    else:
        hcot = -half * math.sin(dth) / cos_minus_one
    x, y = t[0], t[1]
    # rotate (x, y) by -half and scale by hypot(hcot, half)
    c, s = math.cos(-half), math.sin(-half)
    rx, ry = c * x - s * y, s * x + c * y
    k = math.hypot(hcot, half)
    return (rx * k, ry * k, dth)


# ---------------------------------------------------------------------------- 3D (numpy)
