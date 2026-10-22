"""Where ours loses on the harder courses, and what closes the gap: `python bench/losses.py [--pose truth|odom]`.

figure_eight: ours planned at mu 0.7 (as shipped), 0.85 and 1.0, against Choreo and PathPlanner + hold at COF 0.9.
three_piece_chain: ours as the chain the course defines (2 m/s handoffs, 90 deg at both joins), as one path
through every waypoint, and as a chain planned jointly (chain_plan.joint_legs: each join's speed and, optionally,
direction of travel from the one path's profile).

Runs in one process, one seed after another, so it can swap course definitions; prints means over 10 seeds.
"""

import argparse
import os
import sys
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import courses  # noqa: E402
import courses_extra  # noqa: E402
from chain_plan import joint_legs  # noqa: E402
from sim import run_one  # noqa: E402

SEEDS = range(10)


def row(label, cname, key, opts, pose):
    rs = [run_one((cname, key, opts, s, pose)) for s in SEEDS]
    st = np.array([r["t_settle"] for r in rs])
    ok = ~np.isnan(st)
    settle = f"{np.nanmean(st):.3f}" if ok.any() else "never"
    print(f"| {label} | {settle} ({ok.sum()}/{len(rs)}) | {np.nanmean([r['t_5cm'] for r in rs]):.3f} | "
          f"{np.mean([r['err_end_cm'] for r in rs]):.1f} | {np.mean([r['ct_rms_cm'] for r in rs]):.1f} | "
          f"{np.mean([r['wheel_slip_ms'] for r in rs]):.0f} |", flush=True)


def free(c):
    return replace(c, waypoints=tuple(replace(w, tangent_deg=None) for w in c.waypoints))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="truth")
    a = ap.parse_args()
    hdr = "| variant | settled s (n/N) | to 5 cm s | end error cm | cross-track RMS cm | wheel slip ms |\n|---|---|---|---|---|---|"
    print(f"**figure_eight** ({a.pose})\n\n{hdr}")
    row("ours, planning mu 0.7 (as shipped)", "figure_eight", "ours_v2", None, a.pose)
    row("ours, planning mu 0.85", "figure_eight", "ours_v2", {"mu": 0.85}, a.pose)
    row("ours, planning mu 1.0", "figure_eight", "ours_v2", {"mu": 1.0}, a.pose)
    row("Choreo + ChoreoLib", "figure_eight", "choreo", None, a.pose)
    row("PathPlanner + hold, COF 0.9", "figure_eight", "pathplanner", {"hold": True, "cof": 0.9}, a.pose)
    print(f"\n**three_piece_chain** ({a.pose})\n\n{hdr}")
    name = "three_piece_chain"
    c, legs, ex = courses.COURSES[name], courses_extra.THREE_PIECE_LEGS, courses_extra.EXTRAS[name]
    try:
        row("Choreo geometry + DriveToPose finish (hybrid)", name, "choreo_hybrid", None, a.pose)
        row("ours, chain as authored (2 m/s, 90 deg joins)", name, "ours_v2", None, a.pose)
        ex.pop("legs")
        row("ours, one path (authored join directions)", name, "ours_v2", None, a.pose)
        courses.COURSES[name] = free(c)
        row("ours, one path, join directions from the spline", name, "ours_v2", None, a.pose)
        courses.COURSES[name] = c
        ex["legs"] = joint_legs(legs, free_joins=False)[0]
        row("ours, chain, joint plan (speeds only)", name, "ours_v2", None, a.pose)
        courses.COURSES[name] = free(c)
        ex["legs"] = joint_legs(legs)[0]
        row("ours, chain, joint plan (speeds and directions)", name, "ours_v2", None, a.pose)
    finally:
        courses.COURSES[name], ex["legs"] = c, legs


if __name__ == "__main__":
    main()
