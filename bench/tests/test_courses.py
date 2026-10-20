"""Course adapters produce paths that start and end on the course's poses."""

import math

import numpy as np
import pytest

from common import wrap
from courses import COURSES, NeutralPath, hermite, to_bezier, to_bline, to_bline_curve, to_catmull, to_goal_chain
from followers.old_followpath import SplinePath
from followers.pathplanner import PathPlannerPath, cubic_lerp

NAMES = list(COURSES)


def close(a, b, tol=1e-9):
    return math.hypot(a[0] - b[0], a[1] - b[1]) < tol


@pytest.mark.parametrize("name", NAMES)
def test_neutral_path_ends(name):
    c = COURSES[name]
    p = NeutralPath(c)
    assert close(p.points[0], c.start_pose) and close(p.points[-1], c.end_pose)
    assert p.heading_at_pos(0.0) == pytest.approx(c.start_pose[2])
    assert abs(wrap(p.heading_at_pos(len(c.waypoints) - 1.0) - c.end_pose[2])) < 1e-12


@pytest.mark.parametrize("name", NAMES)
def test_old_spline_ends(name):
    c = COURSES[name]
    s = SplinePath(to_catmull(c)["points"])
    assert close(s.point(0.0), c.start_pose) and close(s.point(s.total), c.end_pose)
    assert s.total == pytest.approx(NeutralPath(c).length, abs=2e-3)


@pytest.mark.parametrize("name", NAMES)
def test_bezier_is_the_same_curve(name):
    c = COURSES[name]
    wps = to_bezier(c)["waypoints"]
    assert close(wps[0]["anchor"], c.start_pose) and close(wps[-1]["anchor"], c.end_pose)
    pts = [np.array((w.x, w.y)) for w in c.waypoints]
    from courses import catmull_rom_tangents

    tans = catmull_rom_tangents(pts)
    for i in range(len(wps) - 1):
        for t in (0.1, 0.37, 0.8):
            b = cubic_lerp(wps[i]["anchor"], wps[i]["next"], wps[i + 1]["prev"], wps[i + 1]["anchor"], t)
            h = hermite(pts[i], pts[i + 1], tans[i][0], tans[i][1], [t])[0]
            assert close(b, h, 1e-9)
    path = PathPlannerPath(to_bezier(c))
    assert close(path.points[0].position, c.start_pose, 1e-9)
    assert close(path.points[-1].position, c.end_pose, 1e-9)
    assert path.points[-1].rotation_target[1] == pytest.approx(c.end_pose[2])


@pytest.mark.parametrize("name", NAMES)
@pytest.mark.parametrize("curve", [False, True])
def test_bline_ends(name, curve):
    c = COURSES[name]
    els = (to_bline_curve(c) if curve else to_bline(c))["elements"]
    ts = [e for e in els if e["kind"] == "T"]
    assert close(ts[0]["xy"], c.start_pose) and close(ts[-1]["xy"], c.end_pose)
    rs = [e for e in els if e["kind"] == "R"]
    assert rs[0]["rot"] == pytest.approx(c.start_pose[2]) and rs[0]["t_ratio"] == 0.0
    assert rs[-1]["rot"] == pytest.approx(c.end_pose[2]) and rs[-1]["t_ratio"] == 1.0
    assert els[-1]["kind"] == "T"


@pytest.mark.parametrize("name", NAMES)
def test_goal_chain_ends(name):
    c = COURSES[name]
    n = to_goal_chain(c)
    assert close(n["start"], c.start_pose)
    g = n["goals"][-1]
    assert close(g, c.end_pose) and g[2] == pytest.approx(c.end_pose[2])
