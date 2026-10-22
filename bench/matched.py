"""Choreo and ours planned to the SAME limits, so the plans can be compared like for like.

    python bench/matched.py make --cli path/to/choreo-cli.exe   # write and generate bench/choreo/matched/*
    python bench/matched.py plans                               # planned time and each plan's envelope
    python bench/matched.py run [--procs 4] [--seeds 10]        # perfect pose runs -> out/matched.json
    python bench/matched.py md                                  # out/matched.md from the two above

Why: the main grid gives every method "6 m/s^2", but the libraries read it differently. Choreo's MaxAcceleration
caps the WHOLE acceleration vector, cornering included; ours caps only speeding up and slowing down along the
path and corners at its traction limit (12.8 m/s^2 under tipOrGrip). So ours cornered twice as hard.

The matched block, per traction rule (ours' 0.9 law, RobotConfig.tractionMetersPerSecondSquared):
- ours: the course's acceleration cap is raised to the traction limit, so the 0.9 law alone sets |a|, speeding
  up, braking and cornering together (a friction circle, spin sharing it through the I / (m r) lever);
- Choreo: wheel_cof = traction / 9.8 (Choreo's own g), so each module's grip circle is a quarter of the same
  force and the four together give the same circle, spin sharing it through the module forces; MaxAcceleration
  = traction as well (redundant, a guard); MaxVelocity and MaxAngularVelocity the course's, as before.
What still differs (the planners' models, not the limits):
- the motors: ours has the Kraken curve (back-EMF cuts the push near free speed), Choreo a flat torque up to the
  wheel speed cap. With equalPush (6.5 m/s^2) the curve only binds above about 4.2 m/s, so at the 4 m/s courses
  the plans are bound by the same limits; with tipOrGrip (12.8 m/s^2) it binds from about 3.8 m/s, and on
  fast_long (5 m/s cap) under both rules. `plans` prints where each plan exceeds the other's envelope;
- Choreo picks the shape between waypoints and the free headings; ours runs its spline and heading blend.
  That is the planner difference this block is meant to measure.
Courses: the ones that start and end at rest and have a Choreo trajectory of their own (moving_start, the chain,
the moving goal and the drive-through are left out: Choreo plans them differently by design).
"""

import argparse
import copy
import json
import math
import multiprocessing as mp
import os
import subprocess
import sys
from dataclasses import replace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

import courses  # noqa: E402  (adds the extra courses)
from common import BASE_R, RP  # noqa: E402
from courses import CHOREO_TRAJ, COURSES  # noqa: E402
from courses_extra import EXTRAS  # noqa: E402

OUT = os.path.join(HERE, "out")
CHOREO_DIR = os.path.join(HERE, "choreo", "matched")
BASE = ["straight_3m", "s_curve", "hairpin", "slalom", "long_auto", "figure_eight", "wall_hug",
        "spin_while_driving", "fast_long", "heavy_robot"]
RULES = {"ep": "equalPush", "tg": "tipOrGrip"}
CHOREO_G = 9.8  # trajoptlib swerve_trajectory_generator.cpp: normal force = mass * 9.8 / wheels
OURS_G, MARGIN = 9.81, 0.9  # RobotConfig.GRAVITY, TRACTION_MARGIN
METHODS = {"choreo": "Choreo + ChoreoLib follower", "choreo_hybrid": "Choreo geometry + DriveToPose finish",
           "ours": "Ours v2"}


def traction(rule):
    """RobotConfig.tractionMetersPerSecondSquared for the bench plant (tip distance: the square base's)."""
    mu, h, x = RP.mu_static, RP.cg_height, BASE_R / math.sqrt(2.0)
    if rule == "equalPush":
        return MARGIN * mu * OURS_G / (1.0 + mu * h / x)
    return MARGIN * min(mu * OURS_G, OURS_G * x / h)


def name(tag, base):
    return f"mt_{tag}_{base}"


def register():
    """Add the matched copies to COURSES, CHOREO_TRAJ and EXTRAS (every process that runs them calls this)."""
    for tag, rule in RULES.items():
        a = traction(rule)
        for b in BASE:
            c = COURSES[b]
            n = name(tag, b)
            COURSES[n] = replace(c, name=n, limits=replace(c.limits, max_accel=a),
                                 description=c.description + f"; matched limits, {rule} {a:.2f} m/s^2")
            folder = f"{tag}_heavy" if b == "heavy_robot" else tag
            CHOREO_TRAJ[n] = f"matched/{folder}/{b}"
            if b in EXTRAS:
                EXTRAS[n] = EXTRAS[b]


register()


# ============================================================================================ make
def make(cli):
    sys.path.insert(0, os.path.join(HERE, "choreo"))
    from make_extra import heavy_project
    from make_project import e, project, trajectory

    for tag, rule in RULES.items():
        a = traction(rule)
        for folder, proj, bases in ((tag, project(), [b for b in BASE if b != "heavy_robot"]),
                                    (f"{tag}_heavy", heavy_project(), ["heavy_robot"])):
            d = os.path.join(CHOREO_DIR, folder)
            os.makedirs(d, exist_ok=True)
            proj = copy.deepcopy(proj)
            proj["name"] = folder
            proj["config"]["cof"] = e(a / CHOREO_G)
            chor = os.path.join(d, folder + ".chor")
            json.dump(proj, open(chor, "w"), indent=1)
            for b in bases:
                path = os.path.join(d, b + ".traj")
                for variant in VARIANTS:
                    json.dump(_variant(trajectory(name(tag, b)), b, a, variant, e), open(path, "w"), indent=1)
                    if not cli:
                        break
                    subprocess.run([cli, "--chor", chor, "--trajectory", b, "--generate"],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    s = json.load(open(path))["trajectory"]["samples"]
                    print(f"{folder}/{b} [{variant}]: " + (f"{len(s)} samples, {s[-1]['t']:.3f} s" if s
                                                          else "solver failed"), flush=True)
                    if s:
                        break


# Choreo's solver fails now and then ("line search failed") on the same problem. These variants have the same
# limits (the grip circle already caps |a| at the traction limit, so MaxAcceleration at that value changes nothing)
# and are tried in order until one converges: no guard, the guard, then targetDt 0.04 s / 0.06 s.
VARIANTS = ["plain", "guard", "dt0.04", "dt0.06", "guard dt0.04"]


def _variant(t, b, a, variant, e):
    t["name"] = b
    cons = t["params"]["constraints"]
    for con in cons:
        if con["data"]["type"] == "MaxAcceleration":
            con["data"]["props"]["max"] = e(a, "m / s ^ 2")
    if "guard" not in variant:
        t["params"]["constraints"] = [c for c in cons if c["data"]["type"] != "MaxAcceleration"]
    if "dt" in variant:
        t["params"]["targetDt"] = e(float(variant.split("dt")[1]), "s")
    return t


# ============================================================================================ plans
def choreo_plan(n):
    from followers.choreo import load_traj

    s = np.array(load_traj(n)["samples"])  # t, x, y, heading, vx, vy, omega, ax, ay, alpha
    v = np.hypot(s[:, 4], s[:, 5])
    acc = np.hypot(s[:, 7], s[:, 8])
    vs = np.where(v > 1e-6, v, 1.0)
    lat = np.abs(s[:, 4] * s[:, 8] - s[:, 5] * s[:, 7]) / vs
    tan = (s[:, 4] * s[:, 7] + s[:, 5] * s[:, 8]) / vs
    return {"T": float(s[-1, 0]), "vmax": float(v.max()), "amax": float(acc.max()), "latmax": float(lat.max()),
            "tanmax": float(np.abs(tan).max()), "alphamax": float(np.abs(s[:, 9]).max()),
            "length": float(np.sum(np.hypot(np.diff(s[:, 1]), np.diff(s[:, 2]))))}


def ours_plan(n, rule):
    from followers.ours_v2 import call_fresh, robot_json, server, to_ours, OURS_MU

    robot = dict(robot_json(OURS_MU), tractionRule=rule)
    r = EXTRAS.get(n, {}).get("robot")
    if r:
        robot.update(massKg=r["mass"], momentOfInertiaKgM2=r["inertia"], statorCurrentLimitAmps=r["stator_limit"])
    rep = call_fresh(op="load", path=to_ours(COURSES[n]), robot=robot)
    g = server().call(op="geometry", step=0.01, profile=True)
    v, a, k = np.array(g["v"]), np.array(g["a"]), np.array(g["k"])
    lat = v * v * np.abs(k)
    return {"T": rep["totalTimeSeconds"], "vmax": float(v.max()), "amax": float(np.hypot(a, lat).max()),
            "latmax": float(lat.max()), "tanmax": float(np.abs(a).max()), "length": rep["lengthMeters"]}


def plans():
    out = {}
    for tag, rule in RULES.items():
        for b in BASE:
            n = name(tag, b)
            out[n] = {"choreo": choreo_plan(n), "ours": ours_plan(n, rule), "traction": traction(rule)}
    json.dump(out, open(os.path.join(OUT, "matched_plans.json"), "w"), indent=1)
    return out


# ============================================================================================ run
def _job(job):
    from sim import run_one

    return job, run_one(job)


def run(procs, seeds):
    jobs = []
    for tag, rule in RULES.items():
        for b in BASE:
            n = name(tag, b)
            for s in range(seeds):
                jobs.append((n, "choreo", None, s, "truth"))
                jobs.append((n, "choreo_hybrid", None, s, "truth"))
                jobs.append((n, "ours_v2", {"robot": {"tractionRule": rule}}, s, "truth"))
    with mp.get_context("spawn").Pool(procs) as pool:
        rows = pool.map(_job, jobs, chunksize=2)
    res = {}
    for (n, m, _, s, _), r in rows:
        key = "ours" if m == "ours_v2" else m
        res.setdefault(n, {}).setdefault(key, []).append({k: v for k, v in r.items() if k != "trace"})
    json.dump(res, open(os.path.join(OUT, "matched.json"), "w"), indent=1)
    return res


# ============================================================================================ md
def _mean(rows, k):
    a = np.array([r[k] for r in rows], dtype=float)
    g = a[~np.isnan(a)]
    return (float(g.mean()) if len(g) else float("nan")), int(np.isnan(a).sum())


def md():
    P = json.load(open(os.path.join(OUT, "matched_plans.json")))
    R = json.load(open(os.path.join(OUT, "matched.json"))) if os.path.exists(os.path.join(OUT, "matched.json")) else {}
    lines = ["# Choreo and ours at matched limits", "",
             "Generated by `bench/matched.py` (see its docstring for how the limits are matched).", ""]
    for tag, rule in RULES.items():
        a = traction(rule)
        lines += [f"## {rule}: |a| <= {a:.2f} m/s^2 for both", "",
                  "Plans (planned time; peak total / sideways acceleration, m/s^2; peak speed, m/s; path length, m):", "",
                  "| course | Choreo plan | ours plan | ours - Choreo | Choreo peak a / side / v / length | "
                  "ours peak a / side / v / length |", "|---|---|---|---|---|---|"]
        for b in BASE:
            p = P[name(tag, b)]
            c, o = p["choreo"], p["ours"]
            lines.append(f"| {b} | {c['T']:.3f} | {o['T']:.3f} | {o['T'] - c['T']:+.3f} | "
                         f"{c['amax']:.2f} / {c['latmax']:.2f} / {c['vmax']:.2f} / {c['length']:.2f} | "
                         f"{o['amax']:.2f} / {o['latmax']:.2f} / {o['vmax']:.2f} / {o['length']:.2f} |")
        if R:
            n0 = len(next(iter(R.values()))["ours"])
            lines += ["", f"Runs, perfect pose, {n0} seeds (time to 5 cm / time settled, s (runs that never "
                      "settle); off its own path RMS / max, cm; off the drawn course RMS, cm):", "",
                      "| course | Choreo + ChoreoLib | hybrid | ours |", "|---|---|---|---|"]
            for b in BASE:
                cells = []
                for m in ("choreo", "choreo_hybrid", "ours"):
                    rows = R[name(tag, b)][m]
                    t5, _ = _mean(rows, "t_5cm")
                    ts, nn = _mean(rows, "t_settle")
                    own, _ = _mean(rows, "ct_own_rms_cm")
                    own_max, _ = _mean(rows, "ct_own_max_cm")
                    ct, _ = _mean(rows, "ct_rms_cm")
                    cells.append(f"{t5:.2f} / {ts:.2f}" + (f" ({nn})" if nn else "")
                                 + f"; {own:.1f} / {own_max:.1f}; {ct:.1f}")
                lines.append(f"| {b} | " + " | ".join(cells) + " |")
        lines.append("")
    open(os.path.join(OUT, "matched.md"), "w", encoding="utf-8").write("\n".join(lines))
    print("\n".join(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "plans", "run", "md"])
    ap.add_argument("--cli")
    ap.add_argument("--procs", type=int, default=4)
    ap.add_argument("--seeds", type=int, default=10)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    if a.cmd == "make":
        make(a.cli)
    elif a.cmd == "plans":
        plans()
        md()
    elif a.cmd == "run":
        run(a.procs, a.seeds)
        md()
    else:
        md()


if __name__ == "__main__":
    main()
