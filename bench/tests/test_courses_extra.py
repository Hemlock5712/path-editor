"""The harder courses (courses_extra.py) and what scenarios.py adds for them."""

import math

import numpy as np
import pytest

from courses import CHOREO_TRAJ, COURSES, NeutralPath
from courses_extra import EXTRA_COURSES, EXTRAS, THREE_PIECE_LEGS, WALL_X, WALL_Y
from scenarios import BUMPER_HALF, MovingEnd, make_follower, plant_params
from sim import make_robot, run_one


def test_every_extra_course_is_registered_with_a_look_line_and_a_choreo_trajectory():
    for name in EXTRA_COURSES:
        assert COURSES[name] is EXTRA_COURSES[name]
        assert EXTRAS[name]["look"]
        assert name in CHOREO_TRAJ


def test_the_three_piece_legs_join_and_match_core_builders_route():
    for a, b in zip(THREE_PIECE_LEGS, THREE_PIECE_LEGS[1:]):
        assert a.waypoints[-1] == b.waypoints[0]
        assert a.end_speed == b.plan_start_speed == 2.0
    c = COURSES["three_piece_chain"]
    assert (c.waypoints[0].x, c.waypoints[0].y) == (-4.70, -3.45)
    assert (c.waypoints[-1].x, c.waypoints[-1].y) == (-5.70, 2.20)
    assert len(c.waypoints) == sum(len(leg.waypoints) for leg in THREE_PIECE_LEGS) - 2


def test_the_figure_eight_crosses_itself_once():
    p = NeutralPath(COURSES["figure_eight"])
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(p.points, axis=0).T))])
    near = s[np.hypot(p.points[:, 0], p.points[:, 1]) < 0.01]
    assert near.max() - near.min() > 3.0  # two passes through the origin, metres apart


def test_the_wall_hug_path_leaves_15_cm_to_each_wall():
    p = NeutralPath(COURSES["wall_hug"]).points
    gap = min((WALL_Y - p[:, 1]).min(), (WALL_X - p[:, 0]).min()) - BUMPER_HALF
    assert gap == pytest.approx(0.15, abs=0.005)


def test_the_heavy_robot_reaches_the_plant_and_the_methods_with_a_robot_model():
    rp, _ = make_robot(COURSES["heavy_robot"], 0)
    rp = plant_params("heavy_robot", rp)
    assert (rp.mass, rp.inertia, rp.stator_limit) == (68.0, 6.8, 96.0)
    pp, _ = make_follower("heavy_robot", "pathplanner", None)
    assert (pp.cfg.mass, pp.cfg.moi, pp.cfg.current_limit) == (68.0, 6.8, 96.0)
    ours, _ = make_follower("heavy_robot", "ours_v2", None)
    assert ours.robot_overrides["massKg"] == 68.0
    plain, _ = make_follower("s_curve", "ours_v2", None)
    assert plain.robot_overrides == {}


@pytest.mark.parametrize("key", ["ours_v2", "pathplanner_hold", "bline_readme", "choreo", "drive_to_pose_chain"])
def test_the_tag_moves_the_goal_and_every_method_settles_on_the_true_one(key):
    r = run_one(("tag_align_moving_goal", key, None, 0, "truth"))
    assert r["t_reveal"] > 0
    assert not math.isnan(r["t_settle"]) and r["err_end_cm"] < 2.0 and r["yaw_end_deg"] < 1.0


def test_the_true_goal_is_8_cm_and_3_deg_off_the_map():
    f, _ = make_follower("tag_align_moving_goal", "ours_v2", None)
    assert isinstance(f, MovingEnd)
    m, t = f.map_end, f.true_end
    assert math.hypot(t[0] - m[0], t[1] - m[1]) == pytest.approx(0.08)
    assert math.degrees(t[2] - m[2]) == pytest.approx(3.0)


@pytest.mark.parametrize("key", ["ours_v2", "pathplanner", "bline_readme"])
def test_methods_that_take_the_piece_update_drive_through_it(key):
    for seed in (0, 1):  # the piece moves left on even seeds, right on odd ones
        r = run_one(("intake_drive_through", key, None, seed, "truth"))
        assert r["miss_cm"] < 2.0 and r["keeps_going"] == 1.0


def test_without_the_update_the_robot_misses_by_the_offset():
    r = run_one(("intake_drive_through", "choreo", None, 0, "truth"))
    assert r["miss_cm"] == pytest.approx(10.0, abs=1.0) and r["keeps_going"] == 1.0
    r = run_one(("intake_drive_through", "drive_to_pose_chain", None, 0, "truth"))
    assert r["miss_cm"] < 1.0 and r["keeps_going"] == 0.0  # DriveToPose stops on the piece


def test_a_bline_leg_with_a_rolling_end_finishes_moving():
    from followers.bline import BLine

    leg = THREE_PIECE_LEGS[0]
    f = BLine()
    native = f.adapt(leg)
    assert [e for e in native["elements"] if e["kind"] == "T"][-1]["vmin"] == 2.0
    r = run_one(("three_piece_chain", "bline_caps", None, 0, "truth"))
    assert not math.isnan(r["t_done"])


def test_ours_runs_a_chain_with_slip_aware_odometry():
    r = run_one(("three_piece_chain", "ours_v2", None, 0, "slipodom"))
    assert r["err_end_cm"] < 5.0 and r["overshoot_cm"] < 2.0  # the session survives the handoffs


def test_overshoot_counts_only_after_the_end_is_reached():
    r = run_one(("figure_eight", "ours_v2", None, 0, "truth"))
    assert r["overshoot_cm"] < 2.0  # the loop itself reaches past the end's tangent line
