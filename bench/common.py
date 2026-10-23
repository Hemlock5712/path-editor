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
from dataclasses import replace  # noqa: E402

from alignsim.params import LoopParams, RobotParams  # noqa: E402

# The robot's tread: SDS MK5 Spike Grip, mu 1.5 static (Joe's grip decision, 2026-10). Kinetic keeps the old
# plant's ratio (1.15 / 1.0 -> 1.5 / 1.3). alignsim's own defaults stay as they were for its earlier studies.
# The centre of mass is 0.2 m up (core-builder's figure for the library's tip limit), so the plant transfers
# weight too: tip = g x 0.254 / 0.2 = 12.5 m/s^2 against mu g = 14.7.
# The modules sit at +-0.254 m, Joe's robot (alignsim's default is 0.29): the library's traction limit is then
# 0.9 x 12.5 = 11.2 m/s^2, the robot's AccelerationLimiter number.
# The drive motors' stator (slip) current limit is the robot's 240 A: at 120 A the loaded back modules run short
# of current on a hard launch (Precision-Videos record/v2-race/MU15.md and q11_mu15.md: an 11.2 m/s^2 launch draws
# 105-110 A mean, 134-142 A peak on the back motors).
MU_STATIC, MU_KINETIC = 1.5, 1.3
CG_HEIGHT = 0.2
STATOR_LIMIT = 240.0
MODULE_XY = 0.254
RP = replace(RobotParams(), mu_static=MU_STATIC, mu_kinetic=MU_KINETIC, cg_height=CG_HEIGHT, stator_limit=STATOR_LIMIT,
             module_xy=MODULE_XY)
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
