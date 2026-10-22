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
# the S-curve with the plant disturbed (a shove, low grip, a grip drop): the S-curve's Choreo trajectory and optimised path
DISTURB = ["hit_mid_path", "low_grip_s_curve", "grip_drop_s_curve"]
POSES = ["truth", "odom", "slipodom"]
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
        for b in DISTURB:
            c = COURSES[b]
            n = name(tag, b)
            COURSES[n] = replace(c, name=n, limits=replace(c.limits, max_accel=a),
                                 description=c.description + f"; matched limits, {rule} {a:.2f} m/s^2")
            CHOREO_TRAJ[n] = f"matched/{tag}/s_curve"


def plan_name(n):
    """The course whose plan a matched course uses: the disturbed ones fly the S-curve's."""
    for b in DISTURB:
        if n.endswith("_" + b):
            return n[: -len(b)] + "s_curve"
    return n


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


def _robot(n, rule):
    """Ours' robot file for a matched course: the bench plant, the rule, the course's robot where it has one."""
    from followers.ours_v2 import OURS_MU, robot_json

    robot = dict(robot_json(OURS_MU), tractionRule=rule)
    r = EXTRAS.get(n, {}).get("robot")
    if r:
        robot.update(massKg=r["mass"], momentOfInertiaKgM2=r["inertia"], statorCurrentLimitAmps=r["stator_limit"])
    return robot


def _envelope(rep):
    from followers.ours_v2 import server

    g = server().call(op="geometry", step=0.01, profile=True)
    v, a, k = np.array(g["v"]), np.array(g["a"]), np.array(g["k"])
    lat = v * v * np.abs(k)
    return {"T": rep["totalTimeSeconds"], "vmax": float(v.max()), "amax": float(np.hypot(a, lat).max()),
            "latmax": float(lat.max()), "tanmax": float(np.abs(a).max()), "length": rep["lengthMeters"]}


def ours_plan(n, rule, tree=None):
    from followers.ours_v2 import call_fresh, to_ours

    return _envelope(call_fresh(op="load", path=tree or to_ours(COURSES[n]), robot=_robot(n, rule)))


def ours_on_choreo_plan(n, rule):
    """Path.fromTrajectory on the course's .traj: its planned time should be Choreo's."""
    from followers.ours_v2 import call_fresh

    return _envelope(call_fresh(op="load", trajectory=_traj(n), robot=_robot(n, rule)))


def _traj(n):
    from followers.choreo import load_traj

    s = load_traj(n)["samples"]
    return {k: [float(r[i]) for r in s] for i, k in enumerate(("t", "x", "y", "heading", "vx", "vy"))}


def plans():
    opt = _optimised()
    out = {}
    for tag, rule in RULES.items():
        for b in BASE:
            n = name(tag, b)
            out[n] = {"choreo": choreo_plan(n), "ours": ours_plan(n, rule), "traction": traction(rule),
                      "ours_on_choreo": ours_on_choreo_plan(n, rule)}
            if n in opt:
                out[n]["ours_opt"] = ours_plan(n, rule, opt[n]["path"])
    json.dump(out, open(os.path.join(OUT, "matched_plans.json"), "w"), indent=1)
    return out


# ============================================================================================ optimise
OPT_FILE = os.path.join(OUT, "matched_opt.json")
OPT_BUDGET_MS = 3000  # the optimiser's default budget (1500 plans), as in ppcompare


def _optimise_job(n):
    """tools/optimise.mjs on the course as ours draws it, curves only (waypoints stay, like Choreo's), waypoints
    with a course heading pinned (ppcompare.optimise_ours), on a large empty field."""
    import subprocess
    import tempfile

    from followers.ours_v2 import to_ours

    rule = RULES[n.split("_")[1]]
    course = COURSES[n]
    pins = [str(i) for i, w in enumerate(course.waypoints) if w.heading_deg is not None]
    with tempfile.TemporaryDirectory() as d:
        p, r, f, o = (os.path.join(d, x) for x in ("path.json", "robot.json", "field.json", "out.json"))
        json.dump(to_ours(course), open(p, "w"))
        json.dump(_robot(n, rule), open(r, "w"))
        json.dump({"field": {"length_m": 60.0, "width_m": 40.0}, "obstacles": []}, open(f, "w"))
        cmd = ["node", os.path.join(os.path.dirname(HERE), "tools", "optimise.mjs"), p, "--robot", r, "--field", f,
               "--budget-ms", str(OPT_BUDGET_MS), "--seed", "1", "--out", o]
        if pins:
            cmd += ["--pin", ",".join(pins)]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode not in (0, 3):
            raise RuntimeError(f"{n}: optimise.mjs exit {res.returncode}: {res.stderr.strip()}")
        return n, {"path": json.load(open(o)), "summary": res.stderr.strip()}


def optimise(procs):
    names = [name(tag, b) for tag in RULES for b in BASE]
    with mp.get_context("spawn").Pool(procs) as pool:
        got = dict(pool.map(_optimise_job, names))
    json.dump(got, open(OPT_FILE, "w"), indent=1)
    for n, v in got.items():
        print(n, v["summary"])


_opt_cache = None


def _optimised():
    global _opt_cache
    if _opt_cache is None:
        _opt_cache = json.load(open(OPT_FILE)) if os.path.exists(OPT_FILE) else {}
    return _opt_cache


from followers.ours_v2 import OursV2  # noqa: E402


class OursOptimised(OursV2):
    """Ours v2 on the path our optimiser made for the course (out/matched_opt.json)."""

    @staticmethod
    def adapt(course):
        from followers.ours_v2 import call_fresh, robot_json, server, OURS_MU

        tree = _optimised()[plan_name(course.name)]["path"]
        call_fresh(op="load", path=tree, robot=robot_json(OURS_MU))
        g = server().call(op="geometry", step=0.02)
        return {"path": tree, "geometry": np.column_stack([g["x"], g["y"]])}


def _register_methods():
    from followers import LABELS, METHODS

    METHODS["ours_opt"] = ("matched", "OursOptimised", {})
    LABELS["ours_opt"] = "Ours v2, optimised (curves only)"


_register_methods()


# ============================================================================================ run
# method key in the results -> (registry key, whether it takes ours' traction rule)
RUN_METHODS = {"choreo": ("choreo", False), "choreo_hybrid": ("choreo_hybrid", False), "ours": ("ours_v2", True),
               "ours_on_choreo": ("ours_on_choreo", True), "ours_opt": ("ours_opt", True),
               "pathplanner": ("pathplanner", "pp"), "pp_path2": ("pp_path2", "pp2"), "bline": ("bline_readme", "bline")}
# The rivals keep the gains the main sweep tuned (run.py, out/results.json "tuned"), and plan to the shared limit:
# PathPlanner and path2 get wheel COF = traction / 9.8 (their friction model's g) in place of the tuned COF, path2's
# acceleration cap is the traction limit; BLine's acceleration cap is the course's (the traction limit), applied
# to the whole command vector like Choreo's.
TUNED = {"pp": {"kp": 10.0, "kp_rot": 10.0, "hold": True},
         "pp2": {"kp": 2.0, "kp_ct": 2.0, "handoff": 0.75, "end_distance": 0.02, "end_degrees": 1.0},
         "bline": {"kp": 2.0, "kp_ct": 6.0, "tol": 0.01, "radius": 0.6, "caps": True}}


def run_options(m, rule):
    """-> (registry key, options) for a results method under a traction rule."""
    key, kind = RUN_METHODS[m]
    a = traction(rule)
    if kind is True:
        return key, {"robot": {"tractionRule": rule}}
    if kind == "pp":
        return key, dict(TUNED["pp"], cof=a / CHOREO_G)
    if kind == "pp2":
        return key, dict(TUNED["pp2"], cof=a / CHOREO_G, accel_cap=a)
    if kind == "bline":
        return key, dict(TUNED["bline"])
    return key, None
RESULTS = os.path.join(OUT, "matched.json")


def _job(job):
    from sim import run_one

    return job, run_one(job)


def _load_results():
    """out/matched.json: {pose: {course: {method: [run rows]}}} (an older file without poses is the truth runs)."""
    if not os.path.exists(RESULTS):
        return {}
    res = json.load(open(RESULTS))
    if res and not set(res) <= set(POSES):
        res = {"truth": res}
    return res


def run(procs, seeds, methods, poses, courses):
    res = _load_results()
    jobs, keys = [], []
    for pose in poses:
        for tag, rule in RULES.items():
            for b in courses:
                n = name(tag, b)
                for m in methods:
                    key, opts = run_options(m, rule)
                    if len(res.get(pose, {}).get(n, {}).get(m, [])) == seeds:
                        continue  # already run
                    for sd in range(seeds):
                        jobs.append((n, key, opts, sd, pose))
                        keys.append(m)
    print(len(jobs), "runs", flush=True)
    with mp.get_context("spawn").Pool(procs) as pool:
        rows = pool.map(_job, jobs, chunksize=2)
    fresh = {}
    for m, ((n, _, _, _, pose), r) in zip(keys, rows):
        fresh.setdefault((pose, n, m), []).append({k: v for k, v in r.items() if k != "trace"})
    for (pose, n, m), rs in fresh.items():
        res.setdefault(pose, {}).setdefault(n, {})[m] = rs
    json.dump(res, open(RESULTS, "w"), indent=1)
    return res


# ============================================================================================ export
EXPORT = os.path.join(os.path.dirname(HERE), "editor", "app", "public", "bench", "matched.json")
EXPORT_METHODS = {"ours_opt": "Ours, optimised", "ours": "Ours, as drawn", "choreo": "Choreo + ChoreoLib",
                  "pathplanner": "PathPlanner (tuned) + hold", "pp_path2": "PathPlanner path2 (tuned)",
                  "bline": "BLine (tuned, corner caps)", "ours_on_choreo": "Ours following Choreo's plan"}
STATS = ("t_5cm", "t_settle", "err_end_cm", "ct_own_rms_cm", "ct_own_max_cm", "wheel_slip_ms", "overshoot_cm")


def _stats(rows):
    out = {"n": len(rows)}
    for k in STATS:
        a = np.array([r[k] for r in rows], dtype=float)
        g = a[~np.isnan(a)]
        out[k] = round(float(g.mean()), 4) if len(g) else None
        out[k + "_worst"] = round(float(g.max()), 4) if len(g) else None
        out[k + "_nan"] = int(np.isnan(a).sum())
    return out


def export():
    """The Bench page's head-to-head snapshot: the plans and the run statistics per rule, pose, course and method."""
    P = json.load(open(os.path.join(OUT, "matched_plans.json")))
    R = _load_results()
    out = {"schema": 1, "source": "bench/matched.py (see bench/MATCHED.md)", "seeds": None,
           "methods": [{"key": k, "label": v} for k, v in EXPORT_METHODS.items()],
           "courses": {"plan": BASE, "disturbed": DISTURB},
           "courseLabels": {b: COURSES[b].description for b in BASE + DISTURB},
           "settleRule": "within 2 cm and 1 deg of the end pose, true speed under 5 cm/s, held 0.25 s",
           "rules": {}}
    for tag, rule in RULES.items():
        r_out = {"traction": round(traction(rule), 3), "plans": {}, "runs": {}}
        for b in BASE:
            p = P[name(tag, b)]
            r_out["plans"][b] = {"choreo": round(p["choreo"]["T"], 4), "ours": round(p["ours"]["T"], 4),
                                 "ours_opt": round(p["ours_opt"]["T"], 4),
                                 "ours_on_choreo": round(p["ours_on_choreo"]["T"], 4)}
        for pose in POSES:
            r_out["runs"][pose] = {}
            for b in BASE + DISTURB:
                by = R.get(pose, {}).get(name(tag, b), {})
                r_out["runs"][pose][b] = {m: _stats(by[m]) for m in EXPORT_METHODS if m in by}
                if out["seeds"] is None and by:
                    out["seeds"] = len(next(iter(by.values())))
        out["rules"][rule] = r_out
    os.makedirs(os.path.dirname(EXPORT), exist_ok=True)
    with open(EXPORT, "w") as fh:
        json.dump(out, fh, separators=(",", ":"))
        fh.write("\n")
    print("wrote", EXPORT, f"({os.path.getsize(EXPORT) // 1024} kB)")


# ============================================================================================ md
def _mean(rows, k):
    a = np.array([r[k] for r in rows], dtype=float)
    g = a[~np.isnan(a)]
    return (float(g.mean()) if len(g) else float("nan")), int(np.isnan(a).sum())


PLAN_COLS = [("choreo", "Choreo"), ("ours", "ours as drawn"), ("ours_opt", "ours optimised"),
             ("ours_on_choreo", "ours' path from the .traj")]
RUN_COLS = [("choreo", "Choreo + ChoreoLib"), ("choreo_hybrid", "hybrid prototype"),
            ("ours_on_choreo", "ours on Choreo"), ("ours", "ours as drawn"), ("ours_opt", "ours optimised")]


def md():
    P = json.load(open(os.path.join(OUT, "matched_plans.json")))
    R = _load_results().get("truth", {})
    lines = ["# Choreo and ours at matched limits", "",
             "Generated by `bench/matched.py` (see its docstring for how the limits are matched).", ""]
    for tag, rule in RULES.items():
        a = traction(rule)
        cols = [c for c in PLAN_COLS if c[0] in P[name(tag, BASE[0])]]
        lines += [f"## {rule}: |a| <= {a:.2f} m/s^2 for both", "",
                  "Planned time, s (peak total / sideways acceleration, m/s^2):", "",
                  "| course | " + " | ".join(h for _, h in cols) + " |", "|---" * (len(cols) + 1) + "|"]
        for b in BASE:
            p = P[name(tag, b)]
            lines.append(f"| {b} | " + " | ".join(
                f"{p[k]['T']:.3f} ({p[k]['amax']:.2f} / {p[k]['latmax']:.2f})" for k, _ in cols) + " |")
        if R:
            cols = [c for c in RUN_COLS if c[0] in R[name(tag, BASE[0])]]
            n0 = len(R[name(tag, BASE[0])]["choreo"])
            lines += ["", f"Runs, perfect pose, {n0} seeds: time to 5 cm / time settled, s (runs that never "
                      "settle); off its own path RMS / max, cm:", "",
                      "| course | " + " | ".join(h for _, h in cols) + " |", "|---" * (len(cols) + 1) + "|"]
            sums = {m: 0.0 for m, _ in cols}
            for b in BASE:
                cells = []
                for m, _ in cols:
                    rows = R[name(tag, b)][m]
                    t5, _ = _mean(rows, "t_5cm")
                    ts, nn = _mean(rows, "t_settle")
                    own, _ = _mean(rows, "ct_own_rms_cm")
                    own_max, _ = _mean(rows, "ct_own_max_cm")
                    sums[m] += t5
                    cells.append(f"{t5:.2f} / {ts:.2f}" + (f" ({nn})" if nn else "") + f"; {own:.1f} / {own_max:.1f}")
                lines.append(f"| {b} | " + " | ".join(cells) + " |")
            lines.append("| sum of time to 5 cm | " + " | ".join(f"{sums[m]:.2f}" for m, _ in cols) + " |")
        lines.append("")
    open(os.path.join(OUT, "matched.md"), "w", encoding="utf-8").write("\n".join(lines))
    print("\n".join(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "optimise", "plans", "run", "md", "export"])
    ap.add_argument("--cli")
    ap.add_argument("--procs", type=int, default=4)
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--methods", default=",".join(RUN_METHODS))
    ap.add_argument("--poses", default="truth")
    ap.add_argument("--courses", default="plan", help="plan (the planning courses), disturbed, or both")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    if a.cmd == "make":
        make(a.cli)
    elif a.cmd == "optimise":
        optimise(a.procs)
    elif a.cmd == "plans":
        plans()
        md()
    elif a.cmd == "run":
        courses = {"plan": BASE, "disturbed": DISTURB, "both": BASE + DISTURB}[a.courses]
        run(a.procs, a.seeds, a.methods.split(","), a.poses.split(","), courses)
        md()
    elif a.cmd == "export":
        export()
    else:
        md()


if __name__ == "__main__":
    main()
