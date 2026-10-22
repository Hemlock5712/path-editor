"""BLine corner caps, the Choreo trajectories and sampling, and the pathfinding port."""

import math

import numpy as np
import pytest

from courses import CHOREO_TRAJ, COURSES, bline_corner_caps, to_bline
from followers.choreo import load_traj, sample_at
from pathfind import footprint, load_field, plan_once, poly_distance, swept_clearance
from followers.pathplanner import PathPlannerPath
from sim import run_one


def test_corner_cap_formula():
    # one 90 deg corner, handoff 0.45 m: r = max(0.45, 0.45 / tan 45 deg) = 0.45; cap = sqrt(6 * 0.45 * 0.85)
    els = [{"kind": "T", "xy": (0, 0), "radius": 0.45}, {"kind": "T", "xy": (2, 0), "radius": 0.45},
           {"kind": "T", "xy": (2, 2), "radius": 0.45}]
    caps = [e["vmax"] for e in bline_corner_caps(els, 4.0, 6.0)]
    want = math.floor(math.sqrt(6 * 0.45 * 0.85) * 100) / 100
    assert caps == [2.0, want, want]


def test_corner_caps_straight_path_untouched():
    caps = [e["vmax"] for e in bline_corner_caps(to_bline(COURSES["straight_3m"])["elements"], 4.0, 6.0)
            if e["kind"] == "T"]
    assert caps[1:] == [4.0]


@pytest.mark.parametrize("course", sorted(CHOREO_TRAJ))
def test_choreo_trajectory_ends(course):
    c = COURSES[course]
    s = load_traj(course)["samples"]
    assert math.dist(s[0][1:3], c.start_pose[:2]) < 1e-3 and math.dist(s[-1][1:3], c.end_pose[:2]) < 1e-3
    assert abs(math.remainder(s[-1][3] - c.end_pose[2], 2 * math.pi)) < 1e-3
    assert abs(math.hypot(s[-1][4], s[-1][5]) - c.end_speed) < 2e-2  # StopPoint at the end, or the rolling end


def test_choreo_sampling_hits_knots():
    s = load_traj("s_curve")["samples"]
    for k in (1, 10, 40):
        got = sample_at(s, s[k][0])
        assert np.allclose(got[1:7], s[k][1:7], atol=1e-4)  # the .traj rounds to 1e-5


def test_choreo_followers_reach_end():
    for key in ("choreo", "choreo_hybrid"):
        r = run_one(("straight_3m", key, None, 0, "truth"))
        assert not math.isnan(r["t_5cm"]) and r["err_end_cm"] < 3.0


def test_footprint_distance():
    sq = footprint(0.0, 0.0, 0.0, 0.85, 0.85)
    far = [(2, -1), (3, -1), (3, 1), (2, 1)]
    assert poly_distance(sq, far) == pytest.approx(2 - 0.425)
    near = [(0.3, -1), (3, -1), (3, 1), (0.3, 1)]
    assert poly_distance(sq, near) == 0.0


def test_pathfinding_plans_around_obstacles():
    field = load_field()
    planner, steps, _ = plan_once(field)
    assert [s[0] for s in steps] == [2.5, 2.0, 1.5, 1.0]
    r = field["route"]
    path = PathPlannerPath({"waypoints": steps[-1][1], "rotation_targets": [], "goal_rotation": 0.0})
    pts = np.array([p.position for p in path.points])
    assert math.dist(pts[0], r["start"][:2]) < 1e-9 and math.dist(pts[-1], r["goal"][:2]) < 1e-9
    clear, hits = swept_clearance(pts[:, 0], pts[:, 1], np.zeros(len(pts)), field)
    assert hits == 0 and clear > 0.0
