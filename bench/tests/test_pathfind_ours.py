"""Our pathfinder through the bench bridge, and the routes file. Needs lib/build/libs/pathcore.jar."""

import math
import os

import pytest

from followers.ours_v2 import JAR
from pathfind import load_field, load_routes, run_pathfind

needs_jar = pytest.mark.skipif(not os.path.exists(JAR), reason="build the jar: lib/gradlew -p lib jar")


def test_routes_are_on_free_ground():
    from pathfind import LocalADStar

    planner = LocalADStar(load_field())
    for name, (start, goal, _) in load_routes().items():
        for p in (start, goal):
            g = planner.grid_pos((p[0] + planner.off[0], p[1] + planner.off[1]))
            assert g not in planner.obstacles, name


@needs_jar
@pytest.mark.parametrize("route", ["across", "moving_start"])
def test_ours_reaches_the_goal_without_contact(route):
    r = run_pathfind(("ours_pathfind", route, 0, "truth"))
    assert r["err_end_cm"] < 2.0 and not math.isnan(r["t_settle"])
    assert r["contact_ms"] == 0.0 and r["plan_clear"] == 1.0
    assert r["path_m"] >= r["straight_m"]
