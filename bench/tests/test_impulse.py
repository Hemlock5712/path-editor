"""The external-force hook moves the robot; without it the robot stays put."""

import math

import numpy as np

from common import RP
from courses import COURSES, Course, Impulse, NeutralPath, Waypoint
from sim import DisturbedPlant, run_one


def _hold_still(impulse):
    course = Course("t", (Waypoint(0.0, 0.0, 0.0), Waypoint(3.0, 0.0, 0.0)))
    plant = DisturbedPlant(RP, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), None)
    if impulse:
        plant.arm(impulse, NeutralPath(course))
    for _ in range(int(1.5 / 0.002)):
        plant.set_chassis_robot(0.0, 0.0, 0.0)
        plant.poll((plant.x, plant.y))
        plant.step(0.002)
    return plant


def test_impulse_moves_robot():
    p = _hold_still(Impulse(at_fraction=0.0, force_n=800.0, duration=0.3))
    assert p.fired_at is not None
    assert p.y > 0.10  # pushed to the left of travel (+y), well past tread creep
    assert abs(p.x) < 0.02


def test_no_impulse_no_motion():
    p = _hold_still(None)
    assert p.fired_at is None
    assert math.hypot(p.x, p.y) < 1e-6


def test_hit_course_fires_mid_path():
    r = run_one(("hit_mid_path", "drive_to_pose_chain", None, 0, "truth"))
    L = NeutralPath(COURSES["hit_mid_path"]).length
    assert not np.isnan(r["hit_s"]) and abs(r["hit_s"] - 0.5 * L) < 0.15
    calm = run_one(("s_curve", "drive_to_pose_chain", None, 0, "truth"))
    assert r["ct_max_cm"] > calm["ct_max_cm"] + 10.0
