"""Shared numbers and small helpers for the path-following bench.

Everything here is plain Python so the followers stay close to the Java they port.
"""

import math
import os
import sys
from dataclasses import dataclass

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from alignsim.geom import rot, wrap  # noqa: E402,F401
from alignsim.params import LoopParams, RobotParams  # noqa: E402

RP = RobotParams()
LP = LoopParams()
BASE_R = math.hypot(*RP.module_positions[0])  # m, robot centre to a wheel


@dataclass(frozen=True)
class Limits:
    """The limits every method gets (fairness rule)."""

    max_speed: float = 4.0  # m/s
    max_accel: float = 6.0  # m/s^2
    max_omega_deg: float = 540.0  # deg/s
    max_alpha_deg: float = 720.0  # deg/s^2

    @property
    def max_omega(self):
        return math.radians(self.max_omega_deg)

    @property
    def max_alpha(self):
        return math.radians(self.max_alpha_deg)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def field_to_robot(v, heading):
    """Field-relative (vx, vy, w) -> robot-relative."""
    r = rot((v[0], v[1]), -heading)
    return (r[0], r[1], v[2])


def robot_to_field(v, heading):
    r = rot((v[0], v[1]), heading)
    return (r[0], r[1], v[2])
