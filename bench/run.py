"""Path-following bench: every method x course x seed on the alignsim plant.

    python bench/run.py                 # full run: sweeps, then the main grid, pose = wheel odometry + gyro
    python bench/run.py --both          # ... with odometry AND perfect pose (what RESULTS.md is built from)
    python bench/run.py --perfect-pose  # ... with the true pose only
    python bench/run.py --quick         # 2 seeds, 3 courses, tiny sweeps; prints a table to bench/out/results_quick.json
                                        # and stdout, leaves results.json and RESULTS.md alone
    python bench/run.py --md-only       # rebuild RESULTS.md's generated part from bench/out/results.json
    python bench/run.py --pp-compare    # PathPlanner against ours on path speed (ppcompare.py), its own block
    python bench/run.py --mu-sweep      # ours at four planning grips against PathPlanner (musweep.py), its own block
    python bench/run.py --export-editor # bench/out/editor_bench.json for the editor's Bench view (from results.json;
                                        # --trace-seeds 0,1,2 for more seeds, --trace-pose for traces from another
                                        # pose source than the default perfect pose), and
                                        # editor/app/public/bench/results-lite.json, the small committed snapshot
                                        # the deployed editor opens with

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

from courses import COURSES, MAIN_COURSES, QUICK_COURSES, NeutralPath, t_end  # noqa: E402
from courses_extra import EXTRA_COURSES, EXTRAS, FULL_TRACTION, FULL_TRACTION_ACCEL  # noqa: E402
from common import Limits  # noqa: E402
from followers import LABELS, METHODS, build  # noqa: E402

OUT = os.path.join(HERE, "out")
MD = os.path.join(HERE, "RESULTS.md")
START, END = "<!-- generated:start -->", "<!-- generated:end -->"

MAIN_METHODS = ["old_followpath", "bline_readme", "bline_readme_r45", "bline_caps", "bline_tuned", "bline_tuned_caps",
                "pathplanner", "pathplanner_hold", "pathplanner_tuned", "pathplanner_hold_cof09",
                "drive_to_pose_chain", "choreo", "choreo_hybrid", "ours_v2", "ours_noadapt", "ours_tuned"]
MAIN_METHODS += ["pp_path2", "pp_path2_tuned"]  # PathPlanner's new path system (new-path branch, pre-release)
# the full-traction block: ours under each traction rule with its own acceleration (FT_OURS, on the copies with the
# path cap raised), against the others at the shared limits (FT_OTHERS, the main grid's rows)
# ours under each traction rule of the library (robot file setting); the first is the shipped default
OURS_RULES = ["ours_v2"]
FT_OURS = OURS_RULES
FT_OTHERS = ["pathplanner_tuned", "choreo", "choreo_hybrid"]
TUNED_BASE = {"bline_tuned": "bline_readme", "bline_tuned_caps": "bline_readme", "ours_tuned": "ours_v2"}
TUNED_BASE["pp_path2_tuned"] = "pp_path2"
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
            for kr in (5.0, 10.0) for cof in (0.4, 0.5, 0.6, 0.7, 0.9, 1.15, 1.5)]


def ours_grid(quick):
    """Ours v2 (lib/ PathFollower): the end's ease-in line and the latency allowance. The grip is the robot's
    (traction rule in the library, limiter at the same limit); the other FollowerConfig gains stay at their
    defaults."""
    if quick:
        return [{"easeInPerSecond": e} for e in (6.0, 12.0)]
    return [{"easeInPerSecond": e, "latencySeconds": lat}
            for e in (9.0, 12.0, 16.0) for lat in (0.03, 0.05, 0.07)]


def path2_grid(quick):
    """PathPlanner path2 (new-path): its three P gains, the handoff distance and the end tolerance, inside the
    shared limits (the setpoint generator capped at 6 m/s^2, as every other method's)."""
    if quick:
        return [{"accel_cap": 6.0, "end_distance": e, "end_degrees": 1.0} for e in (0.02, 0.05)]
    return [{"accel_cap": 6.0, "kp": kp, "kp_ct": ct, "handoff": h, "end_distance": e, "end_degrees": d}
            for kp in (2.0, 3.0, 4.0, 6.0) for ct in (1.0, 2.0, 4.0) for h in (0.1, 0.25, 0.5, 0.75)
            for e, d in ((0.02, 1.0), (0.05, 2.0))]


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
    ap.add_argument("--resume", action="store_true",
                    help="carry on from out/results.partial.json (saved after each sweep and pose grid) if its "
                         "seeds, courses and pose sources match")
    ap.add_argument("--export-editor", action="store_true",
                    help="write out/editor_bench.json (tables, geometry and traces) for the editor's Bench view")
    ap.add_argument("--trace-pose", default="truth",
                    help="with --export-editor: the pose source the traces are simulated with (perfect pose by default)")
    ap.add_argument("--trace-seeds", default="0",
                    help="with --export-editor: seeds to give traces for, e.g. 0,1,2 (seed 0 comes from results.json)")
    ap.add_argument("--pathfind-only", action="store_true",
                    help="run only the pathfinding course (both pose sources) into out/pathfind.json and its "
                         "RESULTS.md block")
    ap.add_argument("--pp-compare", action="store_true",
                    help="PathPlanner (2027 and the new-path branch's path2) against ours on path speed, over the 2026 "
                         "routes and the courses, into out/ppcompare.json and its RESULTS.md block (ppcompare.py)")
    ap.add_argument("--mu-sweep", action="store_true",
                    help="ours planned at mu 0.7, 0.85, 1.0 and 1.15, drawn and optimised, on the --pp-compare targets, "
                         "into out/musweep.json and its RESULTS.md block (musweep.py; needs out/ppcompare.json)")
    a = ap.parse_args()
    if a.mu_sweep:
        import musweep

        musweep.main(list(range(a.seeds if a.seeds else 10)), min(a.procs, 6))
        return
    if a.pp_compare:
        import ppcompare

        ppcompare.main(list(range(a.seeds if a.seeds else 10)), min(a.procs, 4))  # a JVM per worker: keep memory low
        return
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
    if a.export_editor:
        export_editor(path, [int(x) for x in a.trace_seeds.split(",") if x != ""], min(a.procs, 4), a.trace_pose)
        return
    seeds = list(range(a.seeds if a.seeds else (2 if a.quick else 10)))
    courses = QUICK_COURSES if a.quick else list(MAIN_COURSES)
    poses = ["odom", "slipodom", "truth"] if a.both else (["truth"] if a.perfect_pose else ["odom"])
    t0 = time.time()
    out = {"meta": {"seeds": seeds, "sweep_seeds": SWEEP_SEEDS, "courses": courses, "poses": poses,
                    "quick": a.quick, "t_sim": {c: t_end(COURSES[c]) for c in courses},
                    "tuning_score": f"settled fraction first, then mean(settle time or sim time) + "
                                    f"{TIME_PER_METRE_CT:g} s per metre of RMS cross-track"},
           "sweeps": {}, "tuned": {}, "runs": {}, "geometry": {}, "traces": {}}
    sweep_courses = [c for c in courses if c not in EXTRA_COURSES]  # tuned on the original nine
    partial = os.path.join(OUT, "results.partial.json")
    if a.resume and os.path.exists(partial):
        prev = json.load(open(partial))
        if all(prev["meta"][k] == out["meta"][k] for k in ("seeds", "courses", "poses", "quick")):
            out.update({k: prev[k] for k in ("sweeps", "tuned", "runs", "full_traction", "pathfind") if k in prev})
            print("resuming: sweeps", sorted(out["sweeps"]), "poses", sorted(out["runs"]), flush=True)

    def checkpoint():
        os.makedirs(OUT, exist_ok=True)
        with open(partial, "w") as fh:
            json.dump(out, fh)

    with mp.get_context("spawn").Pool(a.procs) as pool:
        for name, key, base, grid in (("bline", "bline_tuned", "bline_readme", bline_grid(a.quick)),
                                      ("bline_caps", "bline_tuned_caps", "bline_readme", bline_grid(a.quick, caps=True)),
                                      ("pathplanner", "pathplanner_tuned", "pathplanner", pp_grid(a.quick)),
                                      ("ours", "ours_tuned", "ours_v2", ours_grid(a.quick)),
                                      ("pp_path2", "pp_path2_tuned", "pp_path2", path2_grid(a.quick))):
            if name in out["sweeps"]:
                continue
            best, table = sweep(pool, base, grid, sweep_courses)
            out["tuned"][key], out["sweeps"][name] = best, table
            print(f"{key}:", best, f"({time.time() - t0:.0f} s)", flush=True)
            checkpoint()
        for pose in poses:
            if pose in out["runs"]:
                continue
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
            checkpoint()
        if not a.quick and "full_traction" not in out:
            ft = list(FULL_TRACTION)
            out["full_traction"] = {}
            for pose in poses:
                jobs, keys = [], []
                for c in ft:
                    for m in FT_OURS:
                        key, opts = resolve(m, out["tuned"])
                        for s in seeds:
                            jobs.append((c, key, opts, s, pose))
                            keys.append((c, m))
                by = {}
                for (c, m), r in zip(keys, run_jobs(pool, jobs)):
                    by.setdefault(c, {}).setdefault(m, []).append(r)
                out["full_traction"][pose] = {c: {m: agg(v) for m, v in ms.items()} for c, ms in by.items()}
            print("full traction done", f"({time.time() - t0:.0f} s)", flush=True)
            checkpoint()
        if not a.quick and "pathfind" not in out:
            out["pathfind"] = pathfind_grid(pool, seeds, poses)
            print("pathfinding done", f"({time.time() - t0:.0f} s)", flush=True)
            checkpoint()
    # geometry each method authored, and one true path per method/course (seed 0, odometry)
    from scenarios import make_follower
    from sim import run_one

    for c in courses:
        g = {"neutral": NeutralPath(COURSES[c]).points[::40].round(4).tolist()}
        tr = {}
        for m in MAIN_METHODS:
            key, opts = resolve(m, out["tuned"])
            f, native = make_follower(c, key, opts)
            g[m] = np.asarray(f.geometry(native)).round(4).tolist()
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


# ============================================================================================ editor export
EDITOR_EXPORT = os.path.join(OUT, "editor_bench.json")
LITE_EXPORT = os.path.join(HERE, "..", "editor", "app", "public", "bench", "results-lite.json")
PPCOMPARE = os.path.join(OUT, "ppcompare.json")

# What each course shows, for the editor's Bench page: one line, what to look at.
EDITOR_LOOK = {
    "straight_3m": "the baseline: how fast each gets there and settles, with nothing to steer round",
    "s_curve": "cross-track through the two bends, and whether the finish is clean",
    "hairpin": "the 180° turn: who keeps to the line at speed, and who swings wide or slows to a crawl",
    "slalom": "three gates in a row: cross-track at each gate, and the time it costs to hit them",
    "long_auto": "a whole auto with heading changes: time, and the final pose after 9 m of driving",
    "moving_start": "a rolling start (2 m/s sideways): how quickly each takes the path over",
    "hit_mid_path": "how far each ends after a shove halfway, and how fast it recovers",
    "low_grip_s_curve": "a slippery floor: who still settles, and how much wheel slip each causes",
    "grip_drop_s_curve": "the grip drops halfway: who adapts, and who slides past the end",
    # the 2026 routes (run.py --pp-compare)
    "start_to_shot": "a short hop that turns to face the hub: who settles on the scoring pose first",
    "depot_and_back": "two legs with a stop at the depot: the total time, and how cleanly each stop lands",
    "neutral_three_piece": "three legs with rolling handoffs at 2 m/s: smooth handoffs, and the final pose",
    "around_hub_trench": "threading the trench round the hub: the time, and keeping to the line in the trench",
    "shot_to_tower": "to the tower, facing the wall: the final pose for the climb",
}
EDITOR_LOOK.update({c: e["look"] for c, e in EXTRAS.items()})  # the harder courses (courses_extra.py)
LITE_TRACE_STEP = 5  # every 5th trace sample: 0.2 s apart
EDITOR_SCHEMA = 1
# what the editor's tables show: name in results.json -> (label, unit)
EDITOR_METRICS = {
    "t_done": ("time to done", "s"),
    "t_5cm": ("time to 5 cm", "s"),
    "t_settle": ("time settled", "s"),
    "ct_rms_cm": ("cross-track RMS", "cm"),
    "ct_max_cm": ("cross-track max", "cm"),
    "err_end_cm": ("final error", "cm"),
    "yaw_end_deg": ("final yaw", "deg"),
    "overshoot_cm": ("overshoot", "cm"),
    "wheel_slip_ms": ("wheel slip", "ms"),
}
# the harder courses' own numbers (scenarios.course_metrics), exported apart from the tables above (where lower
# is always better): name -> (label, unit, higher is better)
COURSE_METRICS = {
    "wall_clear_cm": ("closest bumper to a wall", "cm", True),
    "miss_cm": ("miss at the piece", "cm", False),
    "piece_speed": ("speed at the piece", "m/s", True),
    "keeps_going": ("runs still going at the piece", "fraction", True),
}


def look_line(name):
    """The course's "what to look at" line; a course without one yet gets a line from its description."""
    if name in EDITOR_LOOK:
        return EDITOR_LOOK[name]
    return f"how each copes with {COURSES[name].description}" if name in COURSES else ""


def _ppcompare_block():
    """The PathPlanner comparison (run.py --pp-compare, out/ppcompare.json) for the editor: its rows per
    pose source and target, and the 2026 routes' waypoints on the field. None if it hasn't been run."""
    if not os.path.exists(PPCOMPARE):
        return None
    d = json.load(open(PPCOMPARE))
    try:
        import ppcompare
    except ImportError:
        return None
    keys = list(EDITOR_METRICS) + ["plan_s"]
    runs = {}
    for p, by_target in d["runs"].items():
        runs[p] = {}
        for t, by_method in by_target.items():
            runs[p][t] = {}
            for m, r in by_method.items():
                row = {"n": r.get("n", 0)}
                for k in keys:
                    if k + "_mean" in r:
                        row[k] = {"mean": r.get(k + "_mean"), "worst": r.get(k + "_worst"), "nan": r.get(k + "_nan", 0)}
                runs[p][t][m] = row
    targets = []
    for t in d["meta"]["targets"]:
        route = ppcompare.ROUTES.get(t)
        legs = [[[round(w.x, 3), round(w.y, 3)] for w in leg.waypoints] for leg in route["legs"]] if route else []
        targets.append({"key": t, "description": ppcompare.description(t), "onField": route is not None, "legs": legs,
                        "look": EDITOR_LOOK.get(t, "planned time against settled time: does a faster plan still finish clean?")})
    return {
        "source": os.path.relpath(PPCOMPARE, HERE),
        "meta": {"seeds": d["meta"]["seeds"], "poses": d["meta"]["poses"]},
        "metrics": [{"key": "plan_s", "label": "planned time", "unit": "s"}]
        + [{"key": k, "label": v[0], "unit": v[1]} for k, v in EDITOR_METRICS.items()],
        "methods": [{"key": k, "label": v[1]} for k, v in ppcompare.METHODS.items() if k in d["meta"]["methods"]],
        "targets": targets,
        "runs": runs,
    }


def export_editor(results_path, trace_seeds, procs, trace_pose="truth"):
    """The editor's Bench view input: per pose source, course and method the mean and worst of a few metrics,
    each method's authored geometry, and true traces ([t, x, y, heading] every 40 ms) for the chosen seeds.
    Seed 0's traces come from results.json; other seeds are simulated again here (same plant, same pose source)."""
    d = json.load(open(results_path))
    meta = d["meta"]
    # results.json keeps seed 0's traces for its first pose source; any other is simulated again here
    pose = trace_pose if trace_pose in meta["poses"] else meta["poses"][0]
    courses = meta["courses"]
    methods = [m for m in MAIN_METHODS if m in d["runs"][pose][courses[0]]]
    runs = {}
    for p, by_course in d["runs"].items():
        runs[p] = {}
        for c, by_method in by_course.items():
            runs[p][c] = {}
            for m, r in by_method.items():
                row = {"n": r["n"]}
                for k in EDITOR_METRICS:
                    row[k] = {"mean": r.get(k + "_mean"), "worst": r.get(k + "_worst"), "nan": r.get(k + "_nan", 0)}
                runs[p][c][m] = row
    traces = {c: {} for c in courses}
    extra_jobs, extra_keys = [], []
    for c in courses:
        for s in trace_seeds:
            if s == 0 and c in d["traces"] and pose == meta["poses"][0]:
                traces[c]["0"] = d["traces"][c]
                continue
            for m in methods:
                key, opts = resolve(m, d["tuned"])
                extra_jobs.append(((c, key, opts, s, pose), True))
                extra_keys.append((c, s, m))
    if extra_jobs:
        print(f"simulating {len(extra_jobs)} traces for seeds {trace_seeds}", flush=True)
        with mp.get_context("spawn").Pool(procs) as pool:
            res = pool.map(_trace_job, extra_jobs, chunksize=2)
        for (c, s, m), r in zip(extra_keys, res):
            traces[c].setdefault(str(s), {})[m] = r["trace"]
    out = {
        "schema": EDITOR_SCHEMA,
        "source": os.path.relpath(results_path, HERE),
        "meta": {"seeds": meta["seeds"], "courses": courses, "poses": meta["poses"], "tracePose": pose,
                 "traceSeeds": sorted(int(k) for k in traces[courses[0]]), "tSim": meta["t_sim"]},
        "metrics": [{"key": k, "label": v[0], "unit": v[1]} for k, v in EDITOR_METRICS.items()],
        "methods": [{"key": m, "label": label(m)} for m in methods],
        "courses": [{"key": c, "description": COURSES[c].description, "look": look_line(c)} for c in courses],
        "runs": runs,
        "geometry": d["geometry"],
        "traces": traces,
    }
    cm = _course_metrics_block(d)
    if cm:
        out["courseMetrics"] = cm
    pp = _ppcompare_block()
    if pp:
        out["ppcompare"] = pp
    os.makedirs(OUT, exist_ok=True)
    with open(EDITOR_EXPORT, "w") as fh:
        json.dump(out, fh, separators=(",", ":"))
    print("wrote", EDITOR_EXPORT, f"({os.path.getsize(EDITOR_EXPORT) // 1024} kB)")
    os.makedirs(os.path.dirname(LITE_EXPORT), exist_ok=True)
    with open(LITE_EXPORT, "w", newline="\n") as fh:
        json.dump(lite_export(out), fh, separators=(",", ":"))
        fh.write("\n")
    print("wrote", os.path.normpath(LITE_EXPORT), f"({os.path.getsize(LITE_EXPORT) // 1024} kB)")


def _course_metrics_block(d):
    """The harder courses' own numbers for the editor: per course the metrics it has, and per pose source and
    method their mean and worst run (the lowest where higher is better). None if no course has any."""
    metrics, runs = {}, {}
    for p, by_course in d["runs"].items():
        for c, by_method in by_course.items():
            for m, r in by_method.items():
                for k, (lab, unit, higher) in COURSE_METRICS.items():
                    if k + "_mean" not in r:
                        continue
                    metrics.setdefault(c, {})[k] = {"key": k, "label": lab, "unit": unit, "higherIsBetter": higher}
                    runs.setdefault(p, {}).setdefault(c, {}).setdefault(m, {})[k] = {
                        "mean": r[k + "_mean"], "worst": r[k + ("_min" if higher else "_worst")], "nan": r[k + "_nan"]}
    if not metrics:
        return None
    return {"metrics": {c: list(v.values()) for c, v in metrics.items()}, "runs": _round(runs, 3)}


def _round(v, places):
    """Numbers in nested lists and dicts rounded (None, NaN and strings kept)."""
    if isinstance(v, float):
        return v if math.isnan(v) or math.isinf(v) else round(v, places)
    if isinstance(v, list):
        return [_round(x, places) for x in v]
    if isinstance(v, dict):
        return {k: _round(x, places) for k, x in v.items()}
    return v


def lite_export(out):
    """The editor export made small enough to commit and deploy: the same tables (3 decimals), geometry to the
    millimetre, and only the trace pose's first trace seed, every LITE_TRACE_STEP-th sample (and the last)."""
    seed = str(out["meta"]["traceSeeds"][0])
    traces = {}
    for c, by_seed in out["traces"].items():
        if seed not in by_seed:
            continue
        traces[c] = {seed: {m: _round(tr[::LITE_TRACE_STEP] + ([tr[-1]] if (len(tr) - 1) % LITE_TRACE_STEP else []), 3)
                            for m, tr in by_seed[seed].items()}}
    lite = dict(out)
    lite["source"] = out["source"] + " (lite: one trace seed, traces every 0.2 s)"
    lite["meta"] = {**out["meta"], "traceSeeds": [int(seed)]}
    lite["runs"] = _round(out["runs"], 3)
    lite["geometry"] = _round(out["geometry"], 3)
    lite["traces"] = traces
    if "ppcompare" in out:
        lite["ppcompare"] = {**out["ppcompare"], "runs": _round(out["ppcompare"]["runs"], 3)}
    return lite


def _trace_job(args):
    from sim import run_one

    job, trace = args
    return run_one(job, trace=trace)


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
    courses = [c for c in runs if c not in EXTRA_COURSES]
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
        L.append("- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a "
                 "heading is set), joined by distance transitions; it drives the straight segments between them")
        g = to_goal_chain(co)["goals"]
        L.append("- DriveToPose chain goals: " + ", ".join(f"{p(x)} @{math.degrees(x[2]):.0f} deg" for x in g))
        L.append(f"- Neutral path length {NeutralPath(co).length:.2f} m; old path-editor and PathPlanner author the same "
                 "cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json")
        L.append("")
    return "\n".join(L)


END_TOLERANCE_CM = 5.0  # the editor's validity rule (editor/app/src/bench/benchData.ts)


def _winner(rows, key, lowest=True, valid=None):
    """(method, value) with the best mean of `key` among the methods where every run has it (and that pass
    `valid`); None if none. On time settled, also every final error within END_TOLERANCE_CM, as the editor."""
    best = None
    for m, a in rows.items():
        v = a.get(key + "_mean")
        if v is None or a.get(key + "_nan", 0) or (valid and not valid(a)):
            continue
        if key == "t_settle" and a["err_end_cm_worst"] > END_TOLERANCE_CM:
            continue
        if best is None or (v < best[1] if lowest else v > best[1]):
            best = (m, v)
    return best


def full_traction_md(out):
    """Ours with its own traction on the original nine (FULL_TRACTION), against the others at the shared limits."""
    ft = out.get("full_traction")
    if not ft:
        return "Not run (--quick, or a run from before the block existed).\n"
    titles = {"truth": "Perfect pose", "odom": "Wheel odometry + gyro", "slipodom": "Slip-aware odometry + IMU"}
    L = [f"Ours on the original nine courses with the path's acceleration cap raised from {Limits().max_accel:g} to "
         f"{FULL_TRACTION_ACCEL:g} m/s^2, so its own traction rule decides (speed cap {Limits().max_speed:g} m/s), "
         f"against {', '.join(label(m) for m in FT_OTHERS)} at the shared limits (the main grid's rows). Each cell: "
         "time settled s (runs settled), mean wheel slip ms, mean end error cm.\n"]
    cols = [("ft", m) for m in FT_OURS] + [("shared", m) for m in ["ours_v2"] + FT_OTHERS]

    def cell(a):
        if a is None:
            return "-"
        n, ns = a["n"], a["t_settle_nan"]
        st = "never" if ns == n else f"{a['t_settle_mean']:.2f} ({n - ns}/{n})"
        return f"{st}, {a['wheel_slip_ms_mean']:.0f} ms, {a['err_end_cm_mean']:.1f} cm"
    for pose in ft:
        L.append(f"### {titles[pose]}\n")
        L.append("| course | " + " | ".join(f"{label(m)}{' (own traction)' if k == 'ft' else ' (shared limits)'}"
                                         for k, m in cols) + " |")
        L.append("|---|" + "---|" * len(cols))
        for c, ms in ft[pose].items():
            base = out["runs"][pose].get(c[3:], {})
            L.append(f"| {c[3:]} | " + " | ".join(cell(ms.get(m) if k == "ft" else base.get(m)) for k, m in cols) + " |")
        L.append("")
    return "\n".join(L)


def harder_md(out):
    """The harder courses: what each tests, how each method meets it, and per pose source the winner on each
    course with ours' numbers next to it."""
    from scenarios import BUMPER_HALF, KEEPS_GOING

    courses = [c for c in out["meta"]["courses"] if c in EXTRA_COURSES]
    if not courses:
        return ""
    L = ["Eight courses beyond the original nine (bench/courses_extra.py; what they need beyond a path is in "
         "bench/scenarios.py, whose docstring says how each method meets a moving end). Every method in the main "
         "grid runs every course; none is skipped, because each either has a way to do the task or shows what "
         "happens without one. The tuning sweeps use only the original nine courses.\n",
         "| course | what it tests | what to look at |", "|---|---|---|"]
    for c in courses:
        L.append(f"| {c} | {COURSES[c].description} | {EXTRAS[c]['look']} |")
    L.append("")
    L.append(f"Course-specific numbers: **wall clearance** = the closest the TRUE bumper square ({2 * BUMPER_HALF:g} m) "
             "comes to either wall over the run (the plant has no walls, so negative = through it); **miss** = the "
             "closest the robot's centre comes to the TRUE piece; **at piece** = true speed there; **keeps going** = "
             f"runs at least {KEEPS_GOING:g} m/s at the piece. On tag_align_moving_goal every time and error is "
             "against the TRUE goal. intake_drive_through ends moving, so it has no settle time.\n")
    titles = {"truth": "Perfect pose", "odom": "Wheel odometry + gyro", "slipodom": "Slip-aware odometry + IMU"}
    for pose in ("truth", "odom", "slipodom"):
        if pose not in out["runs"]:
            continue
        runs = out["runs"][pose]
        L.append(f"### {titles[pose]}\n")
        L.append("| course | winner, time settled s (every run settled, final error within 5 cm) | ours v2 as shipped | "
                 "ours v2 tuned | "
                 "course-specific: best / ours v2 / ours tuned |")
        L.append("|---|---|---|---|---|")
        for c in courses:
            rows = runs[c]

            def st(m):
                a = rows[m]
                n, ns = a["n"], a["t_settle_nan"]
                return "never" if ns == n else f"{a['t_settle_mean']:.2f}" + ("" if ns == 0 else f" ({n - ns}/{n})")

            spec = "-"
            valid = None
            if c == "wall_hug":
                valid = lambda a: a["wall_clear_cm_min"] >= 0  # noqa: E731
                b = max(rows, key=lambda m: rows[m]["wall_clear_cm_min"])
                spec = (f"wall clearance worst run cm: best {label(b)} {rows[b]['wall_clear_cm_min']:.1f}; "
                        f"ours {rows['ours_v2']['wall_clear_cm_min']:.1f} / {rows['ours_tuned']['wall_clear_cm_min']:.1f}")
            if c == "intake_drive_through":
                b = _winner(rows, "miss_cm", valid=lambda a: a["keeps_going_mean"] == 1.0)
                o, t = rows["ours_v2"], rows["ours_tuned"]
                spec = (f"miss cm (keeps going in every run): best {label(b[0])} {b[1]:.1f} at "
                        f"{rows[b[0]]['piece_speed_mean']:.2f} m/s; ours {o['miss_cm_mean']:.1f} at "
                        f"{o['piece_speed_mean']:.2f} m/s / {t['miss_cm_mean']:.1f} at {t['piece_speed_mean']:.2f} m/s"
                        if b else "nobody keeps going in every run")
                L.append(f"| {c} | n/a (ends moving) | - | - | {spec} |")
                continue
            w = _winner(rows, "t_settle", valid=valid)
            win = f"{label(w[0])} {w[1]:.2f}" if w else "nobody settles every run"
            L.append(f"| {c} | {win} | {st('ours_v2')} | {st('ours_tuned')} | {spec} |")
        L.append("")
        L.append("Every method on the course-specific numbers (mean; wall clearance and speed at the piece: worst "
                 "run = lowest):\n")
        L.append("| method | wall clearance cm, mean / worst | miss at piece cm, mean / worst | at piece m/s, mean / "
                 "lowest | keeps going |")
        L.append("|---|---|---|---|---|")
        wc, ip = runs.get("wall_hug"), runs.get("intake_drive_through")
        for m in MAIN_METHODS:
            cells = []
            if wc:
                a = wc[m]
                cells.append(f"{a['wall_clear_cm_mean']:.1f} / {a['wall_clear_cm_min']:.1f}")
            else:
                cells.append("-")
            if ip:
                a = ip[m]
                cells += [f"{a['miss_cm_mean']:.1f} / {a['miss_cm_worst']:.1f}",
                          f"{a['piece_speed_mean']:.2f} / {a['piece_speed_min']:.2f}",
                          f"{round(a['keeps_going_mean'] * a['n'])}/{a['n']}"]
            else:
                cells += ["-", "-", "-"]
            L.append(f"| {label(m)} | " + " | ".join(cells) + " |")
        L.append("")
    return "\n".join(L)


def sweep_md(out):
    L = []
    for name, key, edge, title in (("bline", "bline_tuned", "radius", "BLine, no caps"),
                                   ("bline_caps", "bline_tuned_caps", "radius", "BLine with corner caps"),
                                   ("pathplanner", "pathplanner_tuned", "cof", "PathPlanner + hold"),
                                   ("ours", "ours_tuned", "easeInPerSecond", "Ours v2"),
                                   ("pp_path2", "pp_path2_tuned", "handoff", "PathPlanner path2")):
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
        "## Harder courses\n", harder_md(out),
        "## Full traction\n", full_traction_md(out),
        "## Pathfinding course\n", pathfind_md(out),
        "## Geometry each method authored\n", geometry_md(out), END])
    text = open(MD).read() if os.path.exists(MD) else f"# Path-following bench: results\n\n{START}\n{END}\n"
    i, j = text.index(START), text.index(END) + len(END)
    open(MD, "w").write(text[:i] + body + text[j:])


if __name__ == "__main__":
    main()
