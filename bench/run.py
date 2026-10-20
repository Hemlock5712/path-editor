"""Path-following bench: every method x course x seed on the alignsim plant.

    python bench/run.py                 # full run: sweeps, then the main grid, pose = wheel odometry + gyro
    python bench/run.py --both          # ... with odometry AND perfect pose (what RESULTS.md is built from)
    python bench/run.py --perfect-pose  # ... with the true pose only
    python bench/run.py --quick         # 2 seeds, 3 courses, tiny sweeps; prints a table to bench/out/results_quick.json
                                        # and stdout, leaves results.json and RESULTS.md alone
    python bench/run.py --md-only       # rebuild RESULTS.md's generated part from bench/out/results.json

Results: bench/out/results.json (gitignored). RESULTS.md keeps its hand-written text; only the part
between the 'generated' markers is replaced.
"""

import argparse
import json
import math
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

from courses import COURSES, QUICK_COURSES, NeutralPath, t_end  # noqa: E402
from followers import LABELS, METHODS, build  # noqa: E402

OUT = os.path.join(HERE, "out")
MD = os.path.join(HERE, "RESULTS.md")
START, END = "<!-- generated:start -->", "<!-- generated:end -->"

MAIN_METHODS = ["old_followpath", "bline_readme", "bline_readme_r45", "bline_caps", "bline_tuned", "bline_tuned_caps",
                "pathplanner", "pathplanner_hold", "pathplanner_tuned", "pathplanner_hold_cof09",
                "drive_to_pose_chain", "choreo", "choreo_hybrid", "ours_v2", "ours_noadapt", "ours_tuned"]
TUNED_BASE = {"bline_tuned": "bline_readme", "bline_tuned_caps": "bline_readme", "ours_tuned": "ours_v2"}
EXTRA = {  # registry entries that only exist in the bench driver
    "pathplanner_tuned": ("pathplanner", None, "PathPlanner (tuned) + hold"),
    "pathplanner_hold_cof09": ("pathplanner", {"hold": True, "cof": 0.9}, "PathPlanner (kP 5) + hold, wheel COF 0.9"),
}
SWEEP_SEEDS = [100, 101, 102]  # tuning seeds, kept apart from the evaluation seeds 0..N-1
SWEEP_POSE = "truth"  # tune the control law, not the odometry calibration luck of three seeds
TIME_PER_METRE_CT = 10.0  # s per metre of RMS cross-track in the tuning score (10 cm costs 1 s)


def bline_grid(quick, caps=False):
    if quick:
        return [{"kp": kp, "kp_ct": 2.0, "tol": 0.01, "radius": 0.45, "caps": caps} for kp in (2.0, 5.0)]
    if caps:  # BLine-Web's seeding assumes the corner's handoff radius, so the radius is swept with it
        return [{"kp": kp, "kp_ct": ct, "tol": 0.01, "radius": r, "caps": True}
                for kp in (1.5, 2.0, 3.0, 4.0, 5.0) for ct in (2.0, 4.0, 6.0)
                for r in (0.30, 0.45, 0.60, 0.90, 1.20, 1.60)]
    return [{"kp": kp, "kp_ct": ct, "tol": tol, "radius": r}
            for kp in (1.5, 2.0, 3.0, 4.0, 5.0) for ct in (2.0, 4.0, 6.0) for tol in (0.03, 0.01)
            for r in (0.60, 0.90, 1.20, 1.60, 2.00, 2.50, 3.00)]


def pp_grid(quick):
    if quick:
        return [{"kp": kp, "kp_rot": 5.0, "hold": True} for kp in (5.0, 10.0)]
    return [{"kp": kp, "kp_rot": kr, "hold": True, "cof": cof} for kp in (5.0, 7.0, 10.0, 15.0, 20.0)
            for kr in (5.0, 10.0) for cof in (0.4, 0.5, 0.6, 0.7, 0.9, 1.15)]


def ours_grid(quick):
    """Ours v2 (lib/ PathFollower): the end's ease-in line, the latency allowance and the output limiter's grip.
    The other FollowerConfig gains stay at their defaults."""
    if quick:
        return [{"easeInPerSecond": e} for e in (6.0, 12.0)]
    # planning mu 0.7 was the interior optimum in phase 3's sweep, so it stays fixed here
    return [{"easeInPerSecond": e, "latencySeconds": lat, "limiterFrictionCoefficient": lim}
            for e in (9.0, 12.0, 16.0) for lat in (0.03, 0.05, 0.07) for lim in (1.0, 1.4, 2.0)]


def resolve(key, tuned):
    """method key -> (registry key, options)."""
    if key in EXTRA:
        base, opts, _ = EXTRA[key]
        return base, (tuned[key] if opts is None else opts)
    if METHODS[key][2] is None:
        return TUNED_BASE[key], tuned[key]
    return key, None


def label(key):
    return EXTRA[key][2] if key in EXTRA else LABELS[key]


# ============================================================================================ jobs
def _job(args):
    from sim import run_one

    return run_one(args)


def run_jobs(pool, jobs):
    return pool.map(_job, jobs, chunksize=2)


def agg(rows):
    out = {"n": len(rows)}
    for k in rows[0]:
        if k == "trace":
            continue
        a = np.array([r[k] for r in rows], dtype=float)
        good = a[~np.isnan(a)]
        out[k + "_mean"] = float(good.mean()) if len(good) else None
        out[k + "_worst"] = float(good.max()) if len(good) else None
        out[k + "_median"] = float(np.median(good)) if len(good) else None
        out[k + "_min"] = float(good.min()) if len(good) else None
        out[k + "_nan"] = int(np.isnan(a).sum())
    return out


def tuning_score(rows_by_course):
    """(settled fraction, time score) over every course and seed: settle time (unsettled runs count as the
    course's whole simulated time) plus TIME_PER_METRE_CT s per metre of RMS cross-track."""
    settled, total, score = 0, 0, 0.0
    for rows in rows_by_course.values():
        for r in rows:
            total += 1
            ok = not math.isnan(r["t_settle"])
            settled += ok
            score += (r["t_settle"] if ok else r["t_sim"]) + TIME_PER_METRE_CT * r["ct_rms_cm"] / 100.0
    return settled / total, score / total


def sweep(pool, base_key, grid, courses):
    jobs, keys = [], []
    for i, opts in enumerate(grid):
        for c in courses:
            for s in SWEEP_SEEDS:
                jobs.append((c, base_key, opts, s, SWEEP_POSE))
                keys.append((i, c))
    res = run_jobs(pool, jobs)
    by = {}
    for (i, c), r in zip(keys, res):
        by.setdefault(i, {}).setdefault(c, []).append(r)
    table = []
    for i, opts in enumerate(grid):
        frac, score = tuning_score(by[i])
        table.append({"opts": opts, "settled_frac": frac, "score": score})
    best = max(table, key=lambda e: (e["settled_frac"], -e["score"]))
    return best["opts"], table


def _pf_job(args):
    from pathfind import run_pathfind

    return run_pathfind(args)


def pathfind_grid(pool, seeds, poses):
    """Every pathfinding method on every route of bench/fields/routes_v1.json, plus our planner's Java timings
    (PlanCli: first query in a fresh JVM, then warm) and the planned geometry."""
    from pathfind import PATHFIND_METHODS, java_plan_timing, load_routes, planned_geometry

    routes = load_routes()
    out = {"geometry": planned_geometry(), "java": {r: java_plan_timing(v) for r, v in routes.items()},
           "seeds": seeds, "runs": {}}
    for pose in poses:
        jobs = [(m, r, s, pose) for r in routes for m in PATHFIND_METHODS for s in seeds]
        res = pool.map(_pf_job, jobs, chunksize=1)
        by = {}
        for (m, r, _, _), row in zip(jobs, res):
            by.setdefault(r, {}).setdefault(m, []).append(row)
        out["runs"][pose] = {r: {m: agg(v) for m, v in ms.items()} for r, ms in by.items()}
    return out


# ============================================================================================ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--seeds", type=int, default=None)
    ap.add_argument("--perfect-pose", action="store_true")
    ap.add_argument("--both", action="store_true")
    ap.add_argument("--procs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--md-only", action="store_true")
    ap.add_argument("--pathfind-only", action="store_true",
                    help="run only the pathfinding course (both pose sources) into out/pathfind.json and its "
                         "RESULTS.md block")
    a = ap.parse_args()
    path = os.path.join(OUT, "results_quick.json" if a.quick else "results.json")
    if a.pathfind_only:
        seeds = list(range(a.seeds if a.seeds else 10))
        with mp.get_context("spawn").Pool(min(a.procs, 8)) as pool:  # one JVM per worker: keep memory in check
            pf = pathfind_grid(pool, seeds, ["truth", "odom"])
        os.makedirs(OUT, exist_ok=True)
        with open(os.path.join(OUT, "pathfind.json"), "w") as fh:
            json.dump(pf, fh)
        write_pathfind_md(pf)
        print("wrote", os.path.join(OUT, "pathfind.json"), "and the pathfinding block of", MD)
        return
    if a.md_only:
        write_md(json.load(open(path)))
        return
    seeds = list(range(a.seeds if a.seeds else (2 if a.quick else 10)))
    courses = QUICK_COURSES if a.quick else list(COURSES)
    poses = ["odom", "slipodom", "truth"] if a.both else (["truth"] if a.perfect_pose else ["odom"])
    t0 = time.time()
    out = {"meta": {"seeds": seeds, "sweep_seeds": SWEEP_SEEDS, "courses": courses, "poses": poses,
                    "quick": a.quick, "t_sim": {c: t_end(COURSES[c]) for c in courses},
                    "tuning_score": f"settled fraction first, then mean(settle time or sim time) + "
                                    f"{TIME_PER_METRE_CT:g} s per metre of RMS cross-track"},
           "sweeps": {}, "tuned": {}, "runs": {}, "geometry": {}, "traces": {}}
    with mp.get_context("spawn").Pool(a.procs) as pool:
        best, table = sweep(pool, "bline_readme", bline_grid(a.quick), courses)
        out["tuned"]["bline_tuned"], out["sweeps"]["bline"] = best, table
        print("BLine tuned:", best, f"({time.time() - t0:.0f} s)", flush=True)
        best, table = sweep(pool, "bline_readme", bline_grid(a.quick, caps=True), courses)
        out["tuned"]["bline_tuned_caps"], out["sweeps"]["bline_caps"] = best, table
        print("BLine tuned with caps:", best, f"({time.time() - t0:.0f} s)", flush=True)
        best, table = sweep(pool, "pathplanner", pp_grid(a.quick), courses)
        out["tuned"]["pathplanner_tuned"], out["sweeps"]["pathplanner"] = best, table
        print("PathPlanner tuned:", best, f"({time.time() - t0:.0f} s)", flush=True)
        best, table = sweep(pool, "ours_v2", ours_grid(a.quick), courses)
        out["tuned"]["ours_tuned"], out["sweeps"]["ours"] = best, table
        print("Ours v2 tuned:", best, f"({time.time() - t0:.0f} s)", flush=True)
        for pose in poses:
            jobs, keys = [], []
            for c in courses:
                for m in MAIN_METHODS:
                    key, opts = resolve(m, out["tuned"])
                    for s in seeds:
                        jobs.append((c, key, opts, s, pose))
                        keys.append((c, m))
            res = run_jobs(pool, jobs)
            by = {}
            for (c, m), r in zip(keys, res):
                by.setdefault(c, {}).setdefault(m, []).append(r)
            out["runs"][pose] = {c: {m: agg(v) for m, v in ms.items()} for c, ms in by.items()}
            print(pose, "grid done", f"({time.time() - t0:.0f} s)", flush=True)
        if not a.quick:
            out["pathfind"] = pathfind_grid(pool, seeds, poses)
            print("pathfinding done", f"({time.time() - t0:.0f} s)", flush=True)
    # geometry each method authored, and one true path per method/course (seed 0, odometry)
    from sim import run_one

    for c in courses:
        g = {"neutral": NeutralPath(COURSES[c]).points[::40].round(4).tolist()}
        tr = {}
        for m in MAIN_METHODS:
            key, opts = resolve(m, out["tuned"])
            cls, kw = build(key, opts)
            f = cls(**kw)
            g[m] = np.asarray(cls.geometry(f.adapt(COURSES[c]))).round(4).tolist()
            tr[m] = run_one((c, key, opts, 0, poses[0]), trace=True)["trace"]
        out["geometry"][c] = g
        out["traces"][c] = tr
    out["meta"]["wall_s"] = time.time() - t0
    os.makedirs(OUT, exist_ok=True)
    with open(path, "w") as fh:
        json.dump(out, fh)
    print(f"wrote {path} in {time.time() - t0:.0f} s")
    if a.quick:
        print(tables(out))
    else:
        write_md(out)
        if "pathfind" in out:
            write_pathfind_md(out["pathfind"])
        print("wrote", MD)


# ============================================================================================ report
def fmt(v, f="{:.1f}"):
    return "-" if v is None else f.format(v)


HDR = ("| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst "
       "| final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |\n"
       "|---|---|---|---|---|---|---|---|---|---|")


def row(m, a):
    n, ns = a["n"], a["t_settle_nan"]
    settled = "never" if ns == n else f"{a['t_settle_mean']:.2f} ({n - ns}/{n})"
    n5 = a["t_5cm_nan"]
    t5 = "never" if n5 == n else (f"{a['t_5cm_mean']:.2f}" + ("" if n5 == 0 else f" ({n - n5}/{n})"))
    return (f"| {label(m)} | {fmt(a['ct_rms_cm_mean'])} / {fmt(a['ct_max_cm_worst'])} | "
            f"{fmt(a['ct_own_rms_cm_mean'])} / {fmt(a['ct_own_max_cm_worst'])} | "
            f"{fmt(a['err_end_cm_mean'])} / {fmt(a['err_end_cm_worst'])} | {fmt(a['yaw_end_deg_worst'])} | "
            f"{fmt(a['overshoot_cm_worst'])} | {t5} | {settled} | {fmt(a['wheel_slip_ms_mean'], '{:.0f}')} | "
            f"{fmt(a['gen_ms_median'], '{:.2f}')} |")


def overview(out, pose):
    """One line per method over every course and seed."""
    runs = out["runs"][pose]
    courses = list(runs)
    L = ["| method | runs settled | courses settled every seed | mean RMS cross-track cm | worst cross-track cm | "
         "mean final error cm | mean wheel slip ms |", "|---|---|---|---|---|---|---|"]
    for m in MAIN_METHODS:
        rows = [runs[c][m] for c in courses]
        n = sum(r["n"] for r in rows)
        ok = sum(r["n"] - r["t_settle_nan"] for r in rows)
        every = sum(1 for r in rows if r["t_settle_nan"] == 0)
        L.append(f"| {label(m)} | {ok}/{n} | {every}/{len(rows)} | {np.mean([r['ct_rms_cm_mean'] for r in rows]):.1f} | "
                 f"{max(r['ct_max_cm_worst'] for r in rows):.1f} | {np.mean([r['err_end_cm_mean'] for r in rows]):.1f} | "
                 f"{np.mean([r['wheel_slip_ms_mean'] for r in rows]):.0f} |")
    return "\n".join(L)


def tables(out):
    L = []
    titles = {"odom": "Wheel odometry + gyro (no vision): the main condition",
              "slipodom": "Slip-aware odometry + gyro + IMU (lib's SlipAwareOdometry, no vision)",
              "truth": "Perfect pose (the control law alone)"}
    for pose in out["runs"]:
        L.append(f"### {titles[pose]}\n")
        L.append(overview(out, pose) + "\n")
        for c, ms in out["runs"][pose].items():
            L.append(f"**{c}**: {COURSES[c].description} ({out['meta']['t_sim'][c]:.1f} s simulated)\n")
            L.append(HDR)
            for m, a in ms.items():
                L.append(row(m, a))
            L.append("")
    return "\n".join(L)


def geometry_md(out):
    from courses import CHOREO_TRAJ, bline_corner_caps, to_bezier, to_bline, to_goal_chain
    from followers.choreo import load_traj

    def p(xy):
        return f"({xy[0]:.2f}, {xy[1]:.2f})"

    gen_path = os.path.join(HERE, "choreo", "generation_times.json")
    gen = json.load(open(gen_path)) if os.path.exists(gen_path) else {}
    L = []
    for c in out["meta"]["courses"]:
        co = COURSES[c]
        L.append(f"**{c}**\n")
        wps = ", ".join(p((w.x, w.y)) + ("" if w.heading_deg is None else f" @{w.heading_deg:g} deg") for w in co.waypoints)
        L.append(f"- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): {wps}")
        if co.start_velocity != (0.0, 0.0, 0.0):
            L.append(f"- Start velocity (field): {co.start_velocity}")
        if co.impulse:
            L.append(f"- Shove: {co.impulse.force_n:g} N for {co.impulse.duration:g} s at {co.impulse.at_fraction:.0%} "
                     f"of the path, to the left of travel")
        if co.grip:
            L.append(f"- Grip: mu {co.grip[0]} static / {co.grip[1]} kinetic")
        bz = to_bezier(co)["waypoints"]
        L.append("- PathPlanner Bezier handles (prev, next): " + "; ".join(
            f"{p(w['anchor'])}: {'-' if w['prev'] is None else p(w['prev'])}, {'-' if w['next'] is None else p(w['next'])}"
            for w in bz))
        bl = to_bline(co)["elements"]
        L.append("- BLine (waypoints): " + ", ".join(
            p(e["xy"]) if e["kind"] == "T" else f"R {math.degrees(e['rot']):g} deg @t {e['t_ratio']:g}" for e in bl)
            + " (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)")
        caps = [e["vmax"] for e in bline_corner_caps([dict(e, radius=0.45) if e["kind"] == "T" else e for e in bl],
                                                     co.limits.max_speed, co.limits.max_accel) if e["kind"] == "T"]
        L.append("- BLine-Web corner caps at r 0.45, per target (m/s): " + ", ".join(f"{v:g}" for v in caps))
        tr = load_traj(c)
        name = CHOREO_TRAJ[c]
        g_s = gen.get(name)
        L.append(f"- Choreo `{name}.traj`: {len(tr['samples'])} samples, {tr['samples'][-1][0]:.3f} s"
                 + (f", generated in {g_s:.1f} s by choreo-cli" if g_s is not None else ""))
        g = to_goal_chain(co)["goals"]
        L.append("- DriveToPose chain goals: " + ", ".join(f"{p(x)} @{math.degrees(x[2]):.0f} deg" for x in g))
        L.append(f"- Neutral path length {NeutralPath(co).length:.2f} m; old path-editor and PathPlanner author the same "
                 "cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json")
        L.append("")
    return "\n".join(L)


def sweep_md(out):
    L = []
    for name, key, edge, title in (("bline", "bline_tuned", "radius", "BLine, no caps"),
                                   ("bline_caps", "bline_tuned_caps", "radius", "BLine with corner caps"),
                                   ("pathplanner", "pathplanner_tuned", "cof", "PathPlanner + hold"),
                                   ("ours", "ours_tuned", "easeInPerSecond", "Ours v2")):
        if name not in out["sweeps"]:
            continue
        tab = sorted(out["sweeps"][name], key=lambda e: (-e["settled_frac"], e["score"]))
        L.append(f"**{title}** ({len(tab)} variants, perfect pose, seeds {out['meta']['sweep_seeds']}, all courses). "
                 f"Chosen: `{out['tuned'][key]}`. Best five:\n")
        L.append("| options | settled fraction | score s |\n|---|---|---|")
        for e in tab[:5]:
            L.append(f"| `{e['opts']}` | {e['settled_frac']:.2f} | {e['score']:.2f} |")
        best = {}
        for e in tab:
            v = e["opts"].get(edge)
            if v not in best:
                best[v] = e
        L.append(f"\nBest variant at each `{edge}`: " + "; ".join(
            f"{v:g}: {best[v]['settled_frac']:.2f} settled, {best[v]['score']:.2f} s" for v in sorted(best)) + "\n")
    return "\n".join(L)


def pathfind_md(out):
    return ("The pathfinding course (PathPlanner AD* and ours, four start/goal pairs) has its own section below, "
            "written by `python bench/run.py --pathfind-only` (and by the full run).\n")


PF_START, PF_END = "<!-- pathfind:start -->", "<!-- pathfind:end -->"
MARGINS = {"ours_pathfind": 0.05, "ours_pathfind_m10": 0.10, "ours_pathfind_m15": 0.15}
MARGIN_ROWS = ["ours_pathfind_m10", "ours_pathfind_m15"]


def pathfind_section_md(pf):
    from pathfind import PATHFIND_METHODS, load_routes

    routes = load_routes()
    with open(os.path.join(HERE, "fields", "routes_v1.json")) as fh:
        notes = {r["name"]: r["note"] for r in json.load(fh)["routes"]}
    L = [PF_START, f"_Generated by `python bench/run.py --pathfind-only` ({len(pf['seeds'])} seeds per cell)._\n",
         "Field `bench/fields/obstacles_v1.json` (six convex obstacles, centre origin), routes "
         "`bench/fields/routes_v1.json`, robot 0.85 x 0.85 m with bumpers, limits 4 m/s and 6 m/s^2.\n",
         "| route | start -> goal | straight line m | note |\n|---|---|---|---|"]
    for name, (st, go, v0) in routes.items():
        mv = f", moving ({v0[0]:g}, {v0[1]:g}) m/s" if math.hypot(v0[0], v0[1]) > 0 else ""
        L.append(f"| {name} | ({st[0]:g}, {st[1]:g}, {math.degrees(st[2]):g} deg){mv} -> ({go[0]:g}, {go[1]:g}, "
                 f"{math.degrees(go[2]):g} deg) | {math.dist(st[:2], go[:2]):.2f} | {notes[name]} |")
    L += ["", "**Planning time.** Ours in Java: the first query in a fresh JVM (cold) and the median / 95th "
          "percentile of 2,000 more (warm), from `PlanCli`; boot = growing the obstacles and building the visibility "
          "graph. In the bench: our Java time per query in the long-running BenchServer, and the whole round trip "
          "from Python through the pipe (both medians). PathPlanner's AD* is the bench's Python port: its first path and "
          "all four anytime paths (medians), relative only.\n",
          "| route | ours cold ms | ours warm median / p95 ms | ours boot ms | ours in bench: Java / round trip ms | "
          "PathPlanner (Python) first / all ms |\n|---|---|---|---|---|---|"]
    runs = pf["runs"][next(iter(pf["runs"]))]
    for name in routes:
        j = pf["java"][name]
        o, p = runs[name]["ours_pathfind"], runs[name]["pp_pathfind"]
        L.append(f"| {name} | {j['coldMs']:.1f} | {j['warmMedianMs']:.2f} / {j['warmP95Ms']:.2f} | {j['bootMs']:.0f} | "
                 f"{o['plan_ms_median']:.2f} / {o['bridge_ms_median']:.1f} | "
                 f"{p['plan_first_ms_median']:.0f} / {p['plan_ms_median']:.0f} |")
    L.append("")
    hdr = ("| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | "
           "min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |\n"
           "|---|---|---|---|---|---|---|---|")
    titles = {"truth": "Perfect pose", "odom": "Wheel odometry + gyro", "slipodom": "Slip-aware odometry + IMU"}
    for pose in ("truth", "odom", "slipodom"):
        if pose not in pf["runs"]:
            continue
        L.append(f"### {titles[pose]}\n")
        for name in routes:
            L.append(f"**{name}**\n")
            L.append(hdr)
            for m, a in pf["runs"][pose][name].items():
                if m in MARGIN_ROWS:
                    continue
                n, ns = a["n"], a["t_settle_nan"]
                st = "never" if ns == n else f"{a['t_settle_mean']:.2f} ({n - ns}/{n})"
                n5 = a["t_5cm_nan"]
                t5 = "never" if n5 == n else f"{a['t_5cm_mean']:.2f}" + ("" if n5 == 0 else f" ({n - n5}/{n})")
                plen = f"{a['path_m_mean']:.2f}" + (f" (corners {a['route_m_mean']:.2f})" if "route_m_mean" in a else "")
                L.append(f"| {PATHFIND_METHODS[m][2]} | {plen} | {t5} | {st} | "
                         f"{a['err_end_cm_mean']:.1f} / {a['err_end_cm_worst']:.1f} | {a['clearance_cm_min']:.1f} | "
                         f"{a['contact_ms_worst']:.0f} | {a['wheel_slip_ms_mean']:.0f} |")
            L.append("")
    margins = [m for m in ["ours_pathfind", *MARGIN_ROWS] if all(m in r for r in pf["runs"][next(iter(pf["runs"]))].values())]
    if len(margins) > 1:
        L += ["### Ours: bumper margin\n",
              "The margin is how far beyond the bumpers' half-diagonal the obstacles are grown, and what the swept check "
              "demands of the planned path. Contact runs = runs (of the seeds) where the TRUE bumper square touched an "
              "obstacle or wall.\n",
              "| route | pose | margin m | path m | t to 5 cm s | t settled s (n/N) | min clearance cm (worst run) | "
              "contact runs | contact ms (worst run) |\n|---|---|---|---|---|---|---|---|---|"]
        for name in routes:
            for pose in ("truth", "odom", "slipodom"):
                if pose not in pf["runs"]:
                    continue
                for m in margins:
                    a = pf["runs"][pose][name][m]
                    n, ns, n5 = a["n"], a["t_settle_nan"], a["t_5cm_nan"]
                    st = "never" if ns == n else f"{a['t_settle_mean']:.2f} ({n - ns}/{n})"
                    t5 = "never" if n5 == n else f"{a['t_5cm_mean']:.2f}" + ("" if n5 == 0 else f" ({n - n5}/{n})")
                    L.append(f"| {name} | {titles[pose]} | {MARGINS[m]:.2f} | {a['path_m_mean']:.2f} | {t5} | {st} | "
                             f"{a['clearance_cm_min']:.1f} | {round(a['contact_run_mean'] * n)}/{n} | "
                             f"{a['contact_ms_worst']:.0f} |")
        L.append("")
    L.append(PF_END)
    return "\n".join(L)


def write_pathfind_md(pf):
    """Replaces the pathfinding block of RESULTS.md (added before '## Fidelity caveats' if missing)."""
    text = open(MD).read()
    block = pathfind_section_md(pf)
    if PF_START in text:
        i, j = text.index(PF_START), text.index(PF_END) + len(PF_END)
        text = text[:i] + block + text[j:]
    else:
        k = text.index("## Fidelity caveats")
        text = text[:k] + "## Pathfinding: PathPlanner AD* and ours\n\n" + block + "\n\n" + text[k:]
    open(MD, "w").write(text)


def write_md(out):
    body = "\n".join([
        START,
        f"_Generated by `python bench/run.py --both` ({len(out['meta']['seeds'])} seeds, "
        f"{out['meta'].get('wall_s', 0):.0f} s wall)._\n",
        "## Tuning sweeps\n", f"Score: {out['meta']['tuning_score']}.\n", sweep_md(out),
        "## Per-course tables\n",
        "Accuracy is always against the TRUE plant pose. Cross-track is measured from the start until the robot first "
        "reaches the end of the path. Times are means over the runs that got there; 'settled' = within 2 cm and 1 deg "
        "of the end pose, true speed under 5 cm/s, held 0.25 s. Wheel slip = milliseconds of slip summed over the four "
        "wheels. Path prep = median wall-clock of the method's own preparation in Python (relative only; for Choreo it "
        "is only loading the .traj, its solver time is in the geometry section).\n",
        tables(out),
        "## Pathfinding course\n", pathfind_md(out),
        "## Geometry each method authored\n", geometry_md(out), END])
    text = open(MD).read() if os.path.exists(MD) else f"# Path-following bench: results\n\n{START}\n{END}\n"
    i, j = text.index(START), text.index(END) + len(END)
    open(MD, "w").write(text[:i] + body + text[j:])


if __name__ == "__main__":
    main()
