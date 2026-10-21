"""The PathPlanner comparison: the path2 port, PathPlanner's rolling ends, chains, the 2026 routes, the
optimiser objective and the report."""

import math

import numpy as np
import pytest

import ppcompare
from courses import COURSES, NeutralPath, to_bezier
from followers.pathplanner import PathPlannerPath, RobotConfig, generate_trajectory
from followers.pathplanner2 import GUI_STEER_RATE, Path2Follower, SetpointGenerator, plan_time, to_path2
from common import Limits
from pp_objective import Objective


def test_setpoint_generator_keeps_to_the_cap_and_the_grip():
    for cap in (6.0, None):
        gen = SetpointGenerator(RobotConfig(), GUI_STEER_RATE,
                                None if cap is None else (4.0, cap, Limits().max_omega, Limits().max_alpha))
        sp = gen.at((0.0, 0.0, 0.0))
        prev = 0.0
        for _ in range(40):
            sp = gen.generate(sp, (4.0, 0.0, 0.0))
            step = (sp.speeds[0] - prev) / 0.02
            prev = sp.speeds[0]
            assert step <= (cap or 1.15 * 9.8) + 1e-6
        assert sp.speeds[0] == pytest.approx(4.0, abs=1e-6)


def test_path2_plans_and_finishes_inside_its_end_tolerance():
    native = to_path2(COURSES["s_curve"])
    assert [len(p) for p in native["paths"]] == [5]
    assert native["paths"][0][-1]["handoff"] == 0.0 and native["paths"][0][1]["handoff"] == 0.25
    t = plan_time(native)
    assert 1.0 < t < 3.0
    tight = dict(native, end_distance=0.02, end_angle=math.radians(1.0))
    assert plan_time(tight) > t


def test_path2_rejects_a_heading_between_waypoints():
    from courses import Course, HeadingTarget, Waypoint

    c = Course("x", (Waypoint(0, 0, 0.0), Waypoint(2, 0, 0.0)), heading_targets=(HeadingTarget(0.5, 90.0),))
    with pytest.raises(ValueError):
        to_path2(c)


def test_path2_follower_hands_over_within_the_handoff_distance():
    wps = to_path2(COURSES["slalom"])["paths"][0]
    f = Path2Follower(wps, 0.1, math.radians(2), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert f.idx == 1
    x, y = wps[1]["xy"]
    f.calculate((x - 0.2, y, 0.0))
    assert f.idx == 2


def test_pathplanner_goal_end_velocity():
    legs = ppcompare.ROUTES["neutral_three_piece"]["legs"]
    n = to_bezier(legs[0])
    assert n["goal_velocity"] == 2.0
    path = PathPlannerPath(n)
    rolling = generate_trajectory(path, (0.0, 0.0, 0.0), 0.0, RobotConfig(), Limits(), 2.0)
    stopping = generate_trajectory(path, (0.0, 0.0, 0.0), 0.0, RobotConfig(), Limits(), 0.0)
    assert rolling[-1].lin == 2.0 and stopping[-1].lin == 0.0
    assert rolling[-1].t < stopping[-1].t


def test_a_set_tangent_turns_the_neutral_curve():
    leg = ppcompare.ROUTES["neutral_three_piece"]["legs"][0]
    p = NeutralPath(leg)
    tx, ty = p.tangent_at_s(p.length)
    assert tx == pytest.approx(0.0, abs=0.01) and ty == pytest.approx(1.0, abs=0.01)
    # chained legs join without a kink
    nxt = NeutralPath(ppcompare.ROUTES["neutral_three_piece"]["legs"][1])
    assert np.allclose(nxt.tangent_at_s(0.0), (tx, ty), atol=0.01)


def test_routes_stay_on_the_field_and_start_behind_the_line():
    for name, r in ppcompare.ROUTES.items():
        for leg in r["legs"]:
            for w in leg.waypoints:
                assert abs(w.x) < 8.27 - 0.43 and abs(w.y) < 4.03 - 0.43, (name, w)
            assert leg.start_pose and leg.end_pose
        assert r["legs"][-1].end_speed == 0.0


@pytest.mark.parametrize("key", ["pp2027_hold", "pp_path2_tight", "ours_v2"])
def test_a_chain_with_rolling_handoffs_reaches_its_end(key):
    r = ppcompare.run_job(("neutral_three_piece", key, 0, "truth"))
    assert r["plan_s"] > 3.0
    assert r["err_end_cm"] < 5.0


def test_planned_times_follow_each_tool():
    legs = ppcompare.targets()["start_to_shot"]
    for key in ppcompare.RUN_METHODS:
        f = ppcompare.make(key)
        t = ppcompare.planned_seconds(key, legs, f.adapt(legs))
        assert 0.5 < t < 3.0, key


def test_pending_rows_say_why():
    assert ppcompare.optimiser_status() is not None or ppcompare.os.path.exists(ppcompare.OPTIMISER)
    with pytest.raises(ValueError):
        ppcompare.make("ours_optimised")


@pytest.mark.parametrize("tool", ["pp2027", "path2"])
def test_objective_scores_the_drawn_path_and_moves_within_the_corridor(tool):
    o = Objective("around_hub_trench", tool)
    assert o.dims == 3 * 5
    assert o.score([0.0] * o.dims) == pytest.approx(o.before)
    x = [0.0] * o.dims
    x[0], x[1] = 3.0, 4.0  # outside the unit disc: clipped onto the corridor's edge
    c = o.candidate(x)
    if tool == "pp2027":
        a0 = np.array(o.base[0]["waypoints"][1]["anchor"])
        a1 = np.array(c[0]["waypoints"][1]["anchor"])
    else:
        a0 = np.array(o.base["paths"][0][1]["xy"])
        a1 = np.array(c["paths"][0][1]["xy"])
    assert np.linalg.norm(a1 - a0) == pytest.approx(0.5)
    assert o.score(x) is not None


def test_report_has_every_row():
    from run import agg

    rows = {m: agg([ppcompare.run_job(("start_to_shot", m, s, "truth")) for s in (0, 1)])
            for m in ppcompare.RUN_METHODS}
    out = {"meta": {"seeds": [0, 1], "targets": ["start_to_shot"], "methods": ppcompare.RUN_METHODS,
                    "pending": {m: "pending optimiser" for m in ppcompare.PENDING}, "wall_s": 1.0,
                    "t_sim": {"start_to_shot": 5.0}},
           "runs": {"truth": {"start_to_shot": rows}}}
    md = ppcompare.section_md(out)
    assert md.startswith(ppcompare.PPC_START) and md.endswith(ppcompare.PPC_END)
    for m in ppcompare.RUN_METHODS + ppcompare.PENDING:
        assert ppcompare.METHODS[m][1] in md
    assert md.count("pending optimiser") >= 3
