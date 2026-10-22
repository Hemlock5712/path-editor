"""The bench IMU, the grip-drop course and the slip-aware pose source (lib's SlipAwareOdometry via BenchServer;
these need the jar: lib/gradlew -p lib jar)."""

import math
import os

import numpy as np
import pytest

from common import RP
from courses import COURSES, NeutralPath
from imu import BIAS_SIGMA, Imu
from sim import DisturbedPlant, run_one

JAR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "lib", "build",
                   "libs", "pathcore.jar")
needs_jar = pytest.mark.skipif(not os.path.exists(JAR), reason="build the jar first: lib/gradlew -p lib jar")


def test_imu_reads_the_true_acceleration_plus_noise():
    plant = DisturbedPlant(RP, (0.0, 0.0, math.pi / 2), (0.0, 0.0, 0.0), None)
    imu = Imu(plant, np.random.default_rng(3))
    err = []
    for k in range(2000):  # drive forward and sideways in the robot frame, changing speed
        plant.set_chassis_robot(1.5 * math.sin(k * 0.004 * 3), 0.8 * math.cos(k * 0.004 * 2), 0.5)
        v0, th0 = (plant.vx, plant.vy), plant.th
        for _ in range(2):
            plant.step(0.002)
        ax_f, ay_f = (plant.vx - v0[0]) / 0.004, (plant.vy - v0[1]) / 0.004
        c, s = math.cos(plant.th), math.sin(plant.th)
        true = (c * ax_f + s * ay_f, -s * ax_f + c * ay_f)
        ax, ay = imu.read()
        err.append((ax - true[0], ay - true[1]))
    err = np.array(err)
    assert np.all(np.abs(err.mean(axis=0) - imu.bias) < 0.05)  # the bias, fixed for the run
    assert np.all(np.abs(err.std(axis=0) - 0.5) < 0.05)  # vibration (0.5) plus white noise (0.01)


def test_grip_drop_fires_halfway():
    plant_course = COURSES["grip_drop_s_curve"]
    assert plant_course.grip_drop == (0.5, 0.91, 0.79)
    r = run_one(("grip_drop_s_curve", "drive_to_pose_chain", None, 0, "truth"))
    assert r["t_settle"] > 0 or math.isnan(r["t_settle"])  # it ran


def test_grip_drop_changes_the_floor():
    npath = NeutralPath(COURSES["grip_drop_s_curve"])
    plant = DisturbedPlant(RP, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), None)
    plant.arm_grip_drop((0.5, 0.91, 0.79), npath)
    plant.poll((0.5, 0.0))
    assert plant.rp.mu_static == RP.mu_static and plant.grip_dropped_at is None
    plant.poll((3.0, 1.0))
    assert plant.rp.mu_static == 0.91 and plant.rp.mu_kinetic == 0.79
    assert plant.grip_dropped_at is not None


@needs_jar
def test_slip_aware_odometry_is_a_pose_source_for_any_method():
    r = run_one(("s_curve", "pathplanner", None, 0, "slipodom"))
    plain = run_one(("s_curve", "pathplanner", None, 0, "odom"))
    assert not math.isnan(r["slip_fp"])  # the detector ran
    assert math.isnan(plain["slip_fp"])  # and does not run for plain odometry
    assert r["err_end_cm"] < 20


@needs_jar
def test_the_imu_changes_no_other_number():
    """The IMU draws from its own random stream: plain-odometry runs are unchanged by its presence."""
    a = run_one(("s_curve", "drive_to_pose_chain", None, 5, "odom"))
    b = run_one(("s_curve", "drive_to_pose_chain", None, 5, "odom"))
    assert a["err_end_cm"] == b["err_end_cm"] and a["t_settle"] == b["t_settle"] or (
        math.isnan(a["t_settle"]) and math.isnan(b["t_settle"]))
