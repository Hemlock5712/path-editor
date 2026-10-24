"""The Bench page's comparison: every library planning to the SAME limits on the same simulated robot.

    python bench/matched.py make --cli path/to/choreo-cli.exe   # write and generate bench/choreo/matched/*
    python bench/matched.py optimise [--procs 4]                # ours optimised (tools/optimise.mjs) -> out/matched_opt.json
    python bench/matched.py plans                               # planned times -> out/matched_plans.json
    python bench/matched.py run [--procs 4] [--seeds 10] [--poses truth,odom] [--courses both]   # -> out/matched.json
    python bench/matched.py export                              # the page's snapshot, editor/app/public/bench/matched.json

The limits: 95 % of the robot's grip-or-tipping limit (AccelerationLimiter's rule, 11.84 m/s^2 with the plant's
0.254 m modules and 0.2 m centre of mass) and up to 4.5 m/s, just under the motors' 4.57 m/s free speed.
- ours: the course's acceleration cap is raised to that limit, so the traction rule alone sets |a|, speeding up,
  braking and cornering together (a friction circle, spin sharing it through the I / (m r) lever);
- Choreo: wheel_cof = traction / 9.8 (Choreo's own g), so the four modules' grip circles add up to the same circle;
- PathPlanner path2 and BLine: the same limit as their acceleration cap.
What still differs is each planner's model: the motors (ours has the Kraken curve, Choreo a flat torque), and the
shape and headings Choreo chooses between waypoints. That is what this comparison measures.

Courses: the ten that start and end at rest with a Choreo trajectory of their own, plus the S-curve with the plant
disturbed (a shove halfway, a slippery floor). Poses: perfect ("truth") and wheel odometry ("odom").
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

import courses  # noqa: E402,F401  (adds the extra courses)
from common import BASE_R, RP  # noqa: E402
from courses import CHOREO_TRAJ, COURSES  # noqa: E402
from courses_extra import EXTRAS  # noqa: E402

OUT = os.path.join(HERE, "out")
CHOREO_DIR = os.path.join(HERE, "choreo", "matched")
BASE = ["straight_3m", "s_curve", "hairpin", "slalom", "long_auto", "figure_eight", "wall_hug",
        "spin_while_driving", "fast_long", "heavy_robot"]
# the S-curve with the plant disturbed: it flies the S-curve's Choreo trajectory and optimised path
DISTURB = ["hit_mid_path", "low_grip_s_curve"]
POSES = ["truth", "odom"]
TAG, RULE = "tg", "tipOrGrip"  # the robot's traction rule (the only one since 2026-10-23); course names carry the tag
CHOREO_G = 9.8  # trajoptlib swerve_trajectory_generator.cpp: normal force = mass * 9.8 / wheels
OURS_G = 9.81  # RobotConfig.GRAVITY
# Every planner plans to the robot's real limits (2026-10-23, Joe): 95 % of grip-or-tip and up to 4.5 m/s.
MARGIN = 0.95
TOP_SPEED = 4.5


def traction():
    """RobotConfig.tractionMetersPerSecondSquared for the bench plant (tip distance: the square base's)."""
    mu, h, x = RP.mu_static, RP.cg_height, BASE_R / math.sqrt(2.0)
    return MARGIN * min(mu * OURS_G, OURS_G * x / h)


def name(base):
    return f"mt_{TAG}_{base}"


def register():
    """Add the matched copies to COURSES, CHOREO_TRAJ and EXTRAS (every process that runs them calls this)."""
    a = traction()
    for b in BASE + DISTURB:
        c = COURSES[b]
        n = name(b)
        COURSES[n] = replace(c, name=n, limits=replace(c.limits, max_accel=a, max_speed=TOP_SPEED),
                             description=c.description + f"; matched limits, {a:.2f} m/s^2")
        if b in DISTURB:
            CHOREO_TRAJ[n] = f"matched/{TAG}/s_curve"
        else:
            CHOREO_TRAJ[n] = f"matched/{TAG}_heavy/{b}" if b == "heavy_robot" else f"matched/{TAG}/{b}"
        if b in EXTRAS:
            EXTRAS[n] = EXTRAS[b]


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

    a = traction()
    for folder, proj, bases in ((TAG, project(), [b for b in BASE if b != "heavy_robot"]),
                                (f"{TAG}_heavy", heavy_project(), ["heavy_robot"])):
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
                json.dump(_variant(trajectory(name(b)), b, a, variant, e), open(path, "w"), indent=1)
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

    return float(np.array(load_traj(n)["samples"])[-1, 0])


def _robot(n):
    """Ours' robot file for a matched course: the bench plant at the shared margin, the course's robot if it has one."""
    from followers.ours_v2 import OURS_MU, robot_json

    robot = dict(robot_json(OURS_MU), tractionShare=MARGIN)
    r = EXTRAS.get(n, {}).get("robot")
    if r:
        robot.update(massKg=r["mass"], momentOfInertiaKgM2=r["inertia"], statorCurrentLimitAmps=r["stator_limit"])
    return robot


def ours_plan(n, tree=None):
    from followers.ours_v2 import call_fresh, to_ours

    return call_fresh(op="load", path=tree or to_ours(COURSES[n]), robot=_robot(n))["totalTimeSeconds"]


def plans():
    """Planned time per course: Choreo, ours as drawn, ours optimised -> out/matched_plans.json."""
    opt = _optimised()
    out = {}
    for b in BASE:
        n = name(b)
        out[n] = {"choreo": choreo_plan(n), "ours": ours_plan(n)}
        if n in opt:
            out[n]["ours_opt"] = ours_plan(n, opt[n]["path"])
        print(b, {k: round(v, 3) for k, v in out[n].items()}, flush=True)
    json.dump(out, open(os.path.join(OUT, "matched_plans.json"), "w"), indent=1)
    return out


# ============================================================================================ optimise
OPT_FILE = os.path.join(OUT, "matched_opt.json")
OPT_BUDGET_MS = 3000  # the optimiser's default budget (1500 plans)


def _optimise_job(n):
    """tools/optimise.mjs on the course as ours draws it, curves only (waypoints stay, like Choreo's), waypoints
    with a course heading pinned, on a large empty field."""
    import tempfile

    from followers.ours_v2 import to_ours

    course = COURSES[n]
    pins = [str(i) for i, w in enumerate(course.waypoints) if w.heading_deg is not None]
    with tempfile.TemporaryDirectory() as d:
        p, r, f, o = (os.path.join(d, x) for x in ("path.json", "robot.json", "field.json", "out.json"))
        json.dump(to_ours(course), open(p, "w"))
        json.dump(_robot(n), open(r, "w"))
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
    with mp.get_context("spawn").Pool(procs) as pool:
        got = dict(pool.map(_optimise_job, [name(b) for b in BASE]))
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
    """Ours on the path our optimiser made for the course (out/matched_opt.json)."""

    @staticmethod
    def adapt(course):
        from followers.ours_v2 import OURS_MU, call_fresh, robot_json, server

        tree = _optimised()[plan_name(course.name)]["path"]
        call_fresh(op="load", path=tree, robot=robot_json(OURS_MU))
        g = server().call(op="geometry", step=0.02)
        return {"path": tree, "geometry": np.column_stack([g["x"], g["y"]])}


def _register_methods():
    from followers import LABELS, METHODS

    METHODS["ours_opt"] = ("matched", "OursOptimised", {})
    LABELS["ours_opt"] = "Ours, optimised (curves only)"


_register_methods()


# ============================================================================================ run
# results key -> (registry key, options). Ours runs the library's defaults: nothing but the robot file.
# The rivals plan to the shared limit (PathPlanner path2: wheel COF = traction / 9.8, its friction model's g, and an
# acceleration cap of the traction limit; BLine: the course's acceleration cap, applied to the whole command vector
# like Choreo's). Choreo sends its plan's wheel forces as feedforward, as CTRE's template for it does; ours sends
# velocities only, as the robot code does. Every end tolerance in heading is 0.5 deg, half the finish rule's 1 deg,
# as the position ones are under its 2 cm.
# The rivals' gains: the best total at these limits (2026-10-23, 2 seeds x the 10 courses) with every run finished
# on target AND through every waypoint (within 10 cm; sim.py wp_miss_cm):
# - Choreo + hold: ChoreoLib's documented kP 10 / 7.5 (stiffer gains miss waypoints and slip far more);
# - path2: kP 4, cross-track 1, handoff 0.25 m, waypoints capped at their corner's speed. Handoffs of 0.75-1.25 m
#   are faster only by cutting 20-45 cm inside the waypoints;
# - BLine: kP 4, cross-track 4, radius 0.1 m; its 0.6 m radius cut past the waypoints the same way.
def run_options(m):
    a = traction()
    return {
        "ours": ("ours_v2", {"robot": {"tractionShare": MARGIN}}),
        "ours_opt": ("ours_opt", {"robot": {"tractionShare": MARGIN}}),
        "choreo": ("choreo", {"ff": True}),
        "choreo_hold": ("choreo", {"kp": 10.0, "kp_rot": 7.5, "ff": True, "hold": True, "hold_deg": 0.5}),
        "pp_path2": ("pp_path2", {"kp": 4.0, "kp_ct": 1.0, "handoff": 0.25, "end_distance": 0.02, "end_degrees": 0.5,
                                  "corner_caps": True, "cof": a / CHOREO_G, "accel_cap": a}),
        "bline": ("bline_readme", {"kp": 4.0, "kp_ct": 4.0, "tol": 0.01, "tol_deg": 0.5, "radius": 0.1, "caps": True}),
    }[m]


RUN_METHODS = ["ours_opt", "ours", "choreo_hold", "choreo", "pp_path2", "bline"]
RESULTS = os.path.join(OUT, "matched.json")


def _job(job):
    from sim import run_one

    return job, run_one(job)


def _load_results():
    """out/matched.json: {pose: {course: {method: [run rows]}}}."""
    return json.load(open(RESULTS)) if os.path.exists(RESULTS) else {}


def run(procs, seeds, methods, poses, course_list):
    res = _load_results()
    jobs, keys = [], []
    for pose in poses:
        for b in course_list:
            n = name(b)
            for m in methods:
                if len(res.get(pose, {}).get(n, {}).get(m, [])) == seeds:
                    continue  # already run
                key, opts = run_options(m)
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
                  "choreo_hold": "Choreo + ChoreoLib + final hold",
                  "pp_path2": "PathPlanner path2 (pre-release, tuned)", "bline": "BLine (tuned, corner caps)"}
STATS = ("t_5cm", "t_settle", "err_end_cm", "ct_own_rms_cm", "ct_own_max_cm", "wheel_slip_ms", "tip_ms", "wp_miss_cm", "overshoot_cm")


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
    """The Bench page's snapshot: the plans and the run statistics per pose, course and method."""
    P = json.load(open(os.path.join(OUT, "matched_plans.json")))
    R = _load_results()
    out = {"schema": 1, "source": "bench/matched.py (see bench/RESULTS.md)", "seeds": None,
           "methods": [{"key": k, "label": v} for k, v in EXPORT_METHODS.items()],
           "courses": {"plan": BASE, "disturbed": DISTURB},
           "courseLabels": {b: COURSES[b].description for b in BASE + DISTURB},
           "settleRule": "within 2 cm and 1 deg of the end pose, true speed under 5 cm/s, held 0.25 s",
           "rules": {}}
    r_out = {"traction": round(traction(), 3), "plans": {}, "runs": {}}
    for b in BASE:
        r_out["plans"][b] = {k: round(v, 4) for k, v in P[name(b)].items()}
    for pose in POSES:
        r_out["runs"][pose] = {}
        for b in BASE + DISTURB:
            by = R.get(pose, {}).get(name(b), {})
            r_out["runs"][pose][b] = {m: _stats(by[m]) for m in EXPORT_METHODS if m in by}
            if out["seeds"] is None and by:
                out["seeds"] = len(next(iter(by.values())))
    out["rules"][RULE] = r_out
    os.makedirs(os.path.dirname(EXPORT), exist_ok=True)
    with open(EXPORT, "w") as fh:
        json.dump(out, fh, separators=(",", ":"))
        fh.write("\n")
    print("wrote", EXPORT, f"({os.path.getsize(EXPORT) // 1024} kB)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "optimise", "plans", "run", "export"])
    ap.add_argument("--cli")
    ap.add_argument("--procs", type=int, default=4)
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--methods", default=",".join(RUN_METHODS))
    ap.add_argument("--poses", default="truth")
    ap.add_argument("--courses", default="plan", help="plan (the planning courses), disturbed, or both")
    ap.add_argument("--out", help="the results file (default out/matched.json), e.g. to compare two library versions")
    a = ap.parse_args()
    if a.out:
        global RESULTS
        RESULTS = os.path.abspath(a.out)
    os.makedirs(OUT, exist_ok=True)
    if a.cmd == "make":
        make(a.cli)
    elif a.cmd == "optimise":
        optimise(a.procs)
    elif a.cmd == "plans":
        plans()
    elif a.cmd == "run":
        run(a.procs, a.seeds, a.methods.split(","), a.poses.split(","),
            {"plan": BASE, "disturbed": DISTURB, "both": BASE + DISTURB}[a.courses])
    elif a.cmd == "export":
        export()


if __name__ == "__main__":
    main()
