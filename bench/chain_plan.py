"""Prototype: plan a chain of paths jointly, so each handoff's speed and direction come from the whole route.

A chain of path files hands over at a speed and in a direction each file sets on its own (three_piece_chain:
2 m/s and the route data's 90 deg at both joins). Here the route is first planned as ONE path through every
leg's waypoints, with the direction of travel at each join left to the spline rule (the neighbouring
waypoints across the join), and profiled once: the forward/backward pass then runs across the joins. Each
leg then takes, at its ends, that path's direction of travel (`tangent_deg`) and planned speed
(`end_speed` / `plan_start_speed`). Run as a chain, each leg's own profile starts and ends where the joint
one does, so the chain drives (almost) the one path's plan with the handoffs where the auto needs them.

What the editor would need (see RESULTS.md): an auto made of linked paths, and a "plan across the chain"
step that writes each join's direction and handoff speed into the two files from the joint profile, and
re-runs when either file changes.
"""

import json
import math
import os
import subprocess
import tempfile
from dataclasses import replace

from courses import Course
from followers.ours_v2 import JAR, robot_json, to_ours


def _profile(course, mu=0.7):
    with tempfile.TemporaryDirectory() as d:
        p, r = os.path.join(d, "p.json"), os.path.join(d, "r.json")
        json.dump(to_ours(course), open(p, "w"))
        json.dump(robot_json(mu), open(r, "w"))
        out = subprocess.run(["java", "-cp", JAR, "frc.team5712.path.tools.ProfileCli", p, "--robot", r,
                              "--step", "0.01"], capture_output=True, text=True, check=True).stdout
    return json.loads(out)["samples"]


def joint_legs(legs, mu=0.7, free_joins=True):
    """-> the legs with each join's direction of travel and speed from the joint profile.

    free_joins: the joins' directions come from the spline rule across the join (the authored ones are
    dropped); False keeps the authored directions and only takes the speeds."""
    wps = list(legs[0].waypoints)
    joins = []
    for leg in legs[1:]:
        joins.append(len(wps) - 1)
        wps += list(leg.waypoints[1:])
    if free_joins:
        wps = [replace(w, tangent_deg=None) if i in joins else w for i, w in enumerate(wps)]
    whole = Course("joint", tuple(wps))
    smp = _profile(whole, mu)
    out = []
    at = {}
    for i in joins:
        w = wps[i]
        k = min(range(len(smp["s"])), key=lambda j: (smp["x"][j] - w.x) ** 2 + (smp["y"][j] - w.y) ** 2)
        at[i] = (math.degrees(math.atan2(smp["tangentY"][k], smp["tangentX"][k])), smp["velocity"][k])
    first = 0
    for n, leg in enumerate(legs):
        last = first + len(leg.waypoints) - 1
        lw = list(leg.waypoints)
        start_speed, end_speed = 0.0, 0.0
        if first in at:
            lw[0] = replace(lw[0], tangent_deg=at[first][0])
            start_speed = at[first][1]
        if last in at:
            lw[-1] = replace(lw[-1], tangent_deg=at[last][0])
            end_speed = at[last][1]
        out.append(replace(leg, waypoints=tuple(lw), plan_start_speed=start_speed, end_speed=end_speed))
        first = last
    return tuple(out), at
