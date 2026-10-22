"""The PathPlanner comparison: the path2 port, PathPlanner's rolling ends, chains, the 2026 routes, the
optimiser objective and the report."""

import math

import numpy as np
import pytest

import ppcompare
from courses import COURSES, NeutralPath, to_bezier
from followers.pathplanner import PathPlannerPath, RobotConfig, generate_trajectory
from followers.pathplanner2 import GUI_STEER_RATE, Path2Follower, SetpointGenerator, plan_time, to_path2
from common import RP, Limits
from pp_objective import Optimiser, PathFile, mulberry32


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
            assert step <= (cap or RP.mu_static * 9.8) + 1e-6
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


@pytest.mark.parametrize("key", ["pp2027_hold", "pp_path2", "ours_v2"])
def test_a_chain_with_rolling_handoffs_reaches_its_end(key):
    r = ppcompare.run_job(("neutral_three_piece", key, 0, "truth"))
    assert r["plan_s"] > 3.0
    assert r["err_end_cm"] < (12.0 if key == "pp_path2" else 5.0)  # path2 stops inside its 10 cm tolerance


@pytest.fixture
def tuned(monkeypatch):
    monkeypatch.setattr(ppcompare, "path2_tuned", lambda: {"accel_cap": 6.0, "kp": 3.0, "end_distance": 0.02,
                                                          "end_degrees": 1.0})


def test_planned_times_follow_each_tool(tuned):
    legs = ppcompare.targets()["start_to_shot"]
    for key in ppcompare.RUN_METHODS:
        f = ppcompare.make(key)
        t = ppcompare.planned_seconds(key, legs, f.adapt(legs))
        assert 0.5 < t < 3.0, key


def test_optimised_rows_need_the_optimiser(monkeypatch):
    monkeypatch.setattr(ppcompare, "optimiser_status", lambda: "pending optimiser: test")
    with pytest.raises(ValueError):
        ppcompare.make("ours_optimised")


def test_the_seeded_generator_is_javascripts_mulberry32():
    r = mulberry32(1)
    assert [r() for _ in range(3)] == [0.6270739405881613, 0.002735721180215478, 0.5274470399599522]


def _files(tool):
    from courses import to_bezier

    legs = ppcompare.targets()["around_hub_trench"]
    if tool == "pp2027":
        return PathFile("pp2027", to_bezier(legs[0]), leg=legs[0])
    n = to_path2(legs)
    return PathFile("path2", n["paths"][0], end={"end_distance": n["end_distance"], "end_angle": n["end_angle"]})


@pytest.mark.parametrize("tool", ["pp2027", "path2"])
def test_objective_moves_free_waypoints_within_the_corridor(tool):
    o = _files(tool)
    assert o.free == [1, 2, 3, 4, 5] and o.dims == 15  # the ends (with headings) stay put
    assert o.seconds(o.candidate([0.0] * o.dims)) == pytest.approx(o.before)
    x = [0.0] * o.dims
    x[0], x[1], x[2] = 3.0, 4.0, 1.0  # outside the unit disc: onto the corridor's edge; handles doubled
    c = o.candidate(x)
    if tool == "pp2027":
        w0, w1 = o.base["waypoints"][1], c["waypoints"][1]
        assert np.linalg.norm(np.subtract(w1["anchor"], w0["anchor"])) == pytest.approx(0.5, abs=1e-5)
        assert np.linalg.norm(np.subtract(w1["next"], w1["anchor"])) == pytest.approx(
            2 * np.linalg.norm(np.subtract(w0["next"], w0["anchor"])), abs=1e-4)
    else:
        assert np.linalg.norm(np.subtract(c[1]["xy"], o.base[1]["xy"])) == pytest.approx(0.5, abs=1e-5)
    assert o.seconds(c) is not None


@pytest.mark.parametrize("tool", ["pp2027", "path2"])
def test_the_search_never_makes_it_slower_and_keeps_to_its_budget(tool):
    o = Optimiser(_files(tool), seed=1, max_evaluations=150)
    o.run()
    assert o.best_time <= o.before
    assert o.evaluations < 150 + 2 * o.dims  # a step may finish past the budget, as in optimiser.ts
    again = Optimiser(_files(tool), seed=1, max_evaluations=150)
    again.run()
    assert again.best_time == o.best_time  # deterministic for a seed and budget


def test_report_has_every_row(tuned):
    from run import agg

    rows = {m: agg([ppcompare.run_job(("start_to_shot", m, s, "truth")) for s in (0, 1)])
            for m in ppcompare.RUN_METHODS}
    out = {"meta": {"seeds": [0, 1], "targets": ["start_to_shot"], "methods": ppcompare.RUN_METHODS,
                    "pending": {m: "pending optimiser" for m in ppcompare.OPTIMISED}, "wall_s": 1.0,
                    "t_sim": {"start_to_shot": 5.0}},
           "runs": {"truth": {"start_to_shot": rows}}}
    md = ppcompare.section_md(out)
    assert md.startswith(ppcompare.PPC_START) and md.endswith(ppcompare.PPC_END)
    for m in ppcompare.RUN_METHODS + ppcompare.OPTIMISED:
        assert ppcompare.METHODS[m][1] in md
    assert md.count("pending optimiser") >= 3


def test_path2_runs_in_the_main_bench():
    from sim import run_one

    r = run_one(("straight_3m", "pp_path2", None, 0, "truth"))
    assert not math.isnan(r["t_5cm"]) or r["err_end_cm"] < 12.0
    r = run_one(("s_curve", "pp_path2", {"accel_cap": 6.0, "end_distance": 0.02, "end_degrees": 1.0}, 0, "truth"))
    assert r["err_end_cm"] < 20.0
