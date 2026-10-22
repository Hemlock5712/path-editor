"""PathPlanner against ours on path speed: `python bench/run.py --pp-compare`.

Rows per route (five 2026 autos, below) and per existing course:
  (a) PathPlanner as drawn: the 2027 release line (Bezier paths, trajectory generator, PPHolonomicDriveController,
      + hold) as shipped and as tuned in the main sweep, and the new path system on the new-path branch
      ("path2", pre-release: bare waypoints, a reactive follower and the setpoint generator) as shipped and as
      tuned in its sweep (run.path2_grid, inside the shared 6 m/s^2);
  (b) ours as drawn (lib/'s PathFollower, as shipped);
  (c) ours after its own optimiser (tools/optimise.mjs) as it ships: only the curves change (the tangent
      direction and scale at every waypoint); waypoints and headings stay put;
  (d) PathPlanner 2027 after PathPlanner's own optimiser (pp_optimizer.py, a port of the GUI's genetic
      algorithm): only the handles change (heading and lengths); anchors never move.
Extra rows, labelled as such: ours with --move-waypoints, and PathPlanner 2027's anchors and handles moved by
OUR search (pp_objective.py, a port of optimiser.ts with PathPlanner's generator as the objective). path2 has no
optimiser in PathPlanner; its only optimised row is the extra one, our search on its waypoints.

Metrics: the planned time from each tool's own generator (ours: Path.totalTimeSeconds; PathPlanner 2027: the
trajectory's total time; path2: the GUI's simulated time), then on the bench plant: time to 5 cm of the end,
time settled, the end error, wheel slip, cross-track. Perfect pose and wheel odometry, the main bench's
evaluation seeds, robot and limits.

The routes are on the 2026 field (REBUILT), centre origin, blue alliance (x < 0). Field elements come from
WPILib's 2026-rebuilt-welded AprilTag layout (allwpilib fields/.../2026-rebuilt-welded.csv, c610ffd17f): the
blue hub's tags put its 1.21 m square at x -4.25..-3.04, y -0.60..0.60; the trench arms carry tags at
y +-3.39 (x -3.64), so the trench lanes run at y +-3.45 under them; the outpost tags sit on the blue wall
at y -3.37 / -2.94 and the tower's at y -0.29 / 0.14. The starting line is the hub's alliance face (x -4.25),
so a robot starts with its centre at x -4.70. The depot's position is assumed (on the blue wall, y +1.5..+2.5).
Scoring poses face the hub's centre. The bench doesn't collide with field elements: they only shape the
routes.
"""

import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

from common import LP, Limits  # noqa: E402
from courses import COURSES, Course, NeutralPath, Waypoint, polyline_distance  # noqa: E402

ROOT = os.path.dirname(HERE)
OUT = os.path.join(HERE, "out")
MD = os.path.join(HERE, "RESULTS.md")
PPC_START, PPC_END = "<!-- ppcompare:start -->", "<!-- ppcompare:end -->"
HUB = (-3.645, 0.0)


def aim(x, y):
    """Heading (deg) from (x, y) to the blue hub's centre."""
    return math.degrees(math.atan2(HUB[1] - y, HUB[0] - x))


def W(x, y, h=None, t=None):
    return Waypoint(x, y, h, t)


def leg(name, wps, end_speed=0.0, plan_start_speed=0.0, description=""):
    return Course(name, tuple(wps), end_speed=end_speed, plan_start_speed=plan_start_speed, description=description)


# ============================================================================================ routes
SHOT_A = (-5.90, -1.40)
SHOT_B = (-5.80, 1.30)
SHOT_C = (-5.70, 2.20)
ROLL = 2.0  # m/s at a rolling handoff

ROUTES = {
    "start_to_shot": {
        "legs": [leg("start_to_shot", [W(-4.70, -2.20, 0.0), W(-5.30, -2.05), W(*SHOT_A, aim(*SHOT_A))])],
        "description": "from the starting line back to a scoring pose, turning to face the hub (1.5 m)"},
    "depot_and_back": {
        "legs": [leg("depot_out", [W(-4.70, 2.20, 0.0), W(-6.20, 2.45), W(-7.50, 2.00, 180.0)]),
                 leg("depot_back", [W(-7.50, 2.00, 180.0), W(-6.70, 1.55), W(*SHOT_B, aim(*SHOT_B))])],
        "description": "starting line to the depot (stop, intake facing the wall), then back to a scoring pose"},
    "neutral_three_piece": {
        "legs": [
            leg("trench_out", [W(-4.70, -3.45, 0.0, 0.0), W(-3.645, -3.45), W(-2.40, -3.20), W(-1.20, -2.20, 90.0, 90.0)],
                end_speed=ROLL),
            leg("sweep", [W(-1.20, -2.20, 90.0, 90.0), W(-1.20, -0.80), W(-1.20, 0.80), W(-1.20, 2.20, 90.0, 90.0)],
                plan_start_speed=ROLL, end_speed=ROLL),
            leg("trench_back", [W(-1.20, 2.20, 90.0, 90.0), W(-2.40, 3.20), W(-3.645, 3.45), W(-4.90, 3.30),
                                W(*SHOT_C, aim(*SHOT_C))], plan_start_speed=ROLL)],
        "description": "through the lower trench to the neutral zone, a sweep along the fuel facing +y, back "
                       "through the upper trench to a scoring pose: three paths with rolling handoffs at 2 m/s"},
    "around_hub_trench": {
        "legs": [leg("around_hub", [W(-6.00, -0.90, 0.0), W(-5.20, -2.70), W(-4.25, -3.45), W(-3.04, -3.45),
                                    W(-2.40, -2.60), W(-2.20, -1.00), W(-2.20, 1.20, 90.0)])],
        "description": "from behind the hub round it through the lower trench to the hub's neutral side"},
    "shot_to_tower": {
        "legs": [leg("to_tower", [W(*SHOT_C, aim(*SHOT_C)), W(-6.60, 1.00), W(-7.15, -0.07, 180.0)])],
        "description": "from a scoring pose to the tower for a level-1 climb, turning to face the wall"},
}


def targets():
    """Every row target: the 2026 routes, then the existing courses (one leg each)."""
    out = {name: r["legs"] for name, r in ROUTES.items()}
    from courses_extra import EXTRA_COURSES

    for name, c in COURSES.items():
        if name not in EXTRA_COURSES:  # the harder courses need more than a path (scenarios.py)
            out[name] = [c]
    return out


def description(name):
    return ROUTES[name]["description"] if name in ROUTES else COURSES[name].description


# ============================================================================================ methods
# PathPlanner 2027 tuned: the main bench's sweep (tuning seeds 100-102, all courses; RESULTS.md "Tuning sweeps")
PP_TUNED = {"kp": 10.0, "kp_rot": 10.0, "hold": True, "cof": 0.5}

METHODS = {
    "pp2027_hold": ("(a)", "PathPlanner 2027 (Bezier), kP 5 + hold, as shipped", ("chain", "pathplanner", {"hold": True})),
    "pp2027_tuned": ("(a)", "PathPlanner 2027 (Bezier), tuned + hold", ("chain", "pathplanner", PP_TUNED)),
    "pp_path2": ("(a)", "PathPlanner path2 (new-path, pre-release), as shipped", ("path2", {})),
    "pp_path2_tuned": ("(a)", "PathPlanner path2, tuned (6 m/s^2 cap)", ("path2", "tuned")),
    "ours_v2": ("(b)", "Ours v2, as shipped", ("chain", "ours_v2", {})),
    "ours_optimised": ("(c)", "Ours v2 after its optimiser (curves only, as it ships)", ("optimised", "ours")),
    "pp2027_optimised": ("(d)", "PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold",
                         ("optimised", "pp2027ga")),
    "ours_optimised_moved": ("extra", "Ours v2 after its optimiser with --move-waypoints",
                             ("optimised", "ours_moved")),
    "pp2027_optimised_moved": ("extra", "PathPlanner 2027, anchors + handles moved by OUR search (not "
                               "PathPlanner's), + hold", ("optimised", "pp2027")),
    "pp_path2_optimised": ("extra", "PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser)",
                           ("optimised", "path2")),
}
RUN_METHODS = ["pp2027_hold", "pp2027_tuned", "pp_path2", "pp_path2_tuned", "ours_v2"]
OPTIMISED = ["ours_optimised", "pp2027_optimised", "ours_optimised_moved", "pp2027_optimised_moved",
             "pp_path2_optimised"]
BASE = {"ours_optimised": "ours_v2", "pp2027_optimised": "pp2027_hold", "ours_optimised_moved": "ours_v2",
        "pp2027_optimised_moved": "pp2027_hold", "pp_path2_optimised": "pp_path2"}
TOOL = {"ours_optimised": "ours", "pp2027_optimised": "pp2027ga", "ours_optimised_moved": "ours_moved",
        "pp2027_optimised_moved": "pp2027", "pp_path2_optimised": "path2"}
# optimiser tools: ours (curves only), PathPlanner's GA, then the extras (ours moving waypoints, our search on
# PathPlanner 2027's anchors and handles, our search on path2's waypoints)
OPT_TOOLS = ("ours", "pp2027ga", "ours_moved", "pp2027", "path2")
OPT_TITLES = {"ours": "ours, curves only", "pp2027ga": "PathPlanner 2027, its own GA",
              "ours_moved": "extra: ours, --move-waypoints", "pp2027": "extra: PathPlanner 2027, our search",
              "path2": "extra: path2, our search"}
BUDGET_MS = 3000  # the optimiser's default budget (1500 plans), for every tool
OPTIMISED_FILE = os.path.join(OUT, "ppcompare_optimised.json")
MU_FILE = os.path.join(OUT, "musweep_optimised.json")

# The grip sweep (`run.py --mu-sweep`): ours with the robot file's grip at the shipped OURS_MU and lower, as
# drawn and optimised. The output limiter's grip option is max(1.0, mu), so it never clips the plan's corners.
from followers.ours_v2 import OURS_MU  # noqa: E402

MU_SWEEP = (OURS_MU, 1.25, 1.0, 0.7)  # the shipped grip first


def mu_key(mu, optimised=False):
    if mu == OURS_MU:
        return "ours_optimised" if optimised else "ours_v2"
    return f"ours_mu{round(mu * 100):03d}" + ("_optimised" if optimised else "")


for _mu in MU_SWEEP[1:]:
    METHODS[mu_key(_mu)] = ("(b)", f"Ours v2, planning mu {_mu:g}", ("chain", "ours_v2", {
        "mu": _mu, "limiterFrictionCoefficient": max(1.0, _mu)}))
    METHODS[mu_key(_mu, True)] = ("(c)", f"Ours v2 optimised, planning mu {_mu:g}", ("optimised", f"ours@{_mu:g}"))
    BASE[mu_key(_mu, True)] = mu_key(_mu)
    TOOL[mu_key(_mu, True)] = f"ours@{_mu:g}"


def tool_mu(tool):
    return float(tool.split("@")[1]) if "@" in tool else OURS_MU
TUNED_FILES = [os.path.join(OUT, "results.json"), os.path.join(OUT, "ppcompare_tuned.json")]


def path2_tuned():
    """path2's tuned options: from the main run (out/results.json), else from this mode's own sweep."""
    for f in TUNED_FILES:
        if os.path.exists(f):
            t = json.load(open(f)).get("tuned", {}).get("pp_path2_tuned")
            if t:
                return t
    return None


def options(key):
    kind = METHODS[BASE.get(key, key)][2]
    if kind[1] == "tuned":
        t = path2_tuned()
        if t is None:
            raise ValueError("path2 isn't tuned yet: run the main bench or ppcompare.main")
        return t
    return kind[1]
OPTIMISER = os.path.join(ROOT, "tools", "optimise.mjs")


def optimiser_status():
    """None if the headless optimiser can run (c) and (d), else why not."""
    if not os.path.exists(OPTIMISER):
        return "pending optimiser: tools/optimise.mjs doesn't exist yet"
    if shutil.which("node") is None:
        return "pending optimiser: node isn't on the PATH"
    return None


def make(key):
    from followers.chain import Chain
    from followers.pathplanner2 import PathPlanner2

    if key in BASE:  # the optimised rows run their tool's own follower, as shipped
        if optimiser_status() is not None:
            raise ValueError(f"{key}: {optimiser_status()}")
        key = BASE[key]
    kind = METHODS[key][2]
    if kind[0] == "chain":
        return Chain(kind[1], **kind[2])
    if kind[0] == "path2":
        return PathPlanner2(**options(key))
    raise ValueError(f"{key}: {optimiser_status() or 'not wired yet'}")


# ============================================================================================ planned time
def planned_seconds(key, legs, native):
    """Each tool's own generator time for the authored path(s), the way its editor shows it: from the plan's
    start speed (0, or the handoff speed), summed over the chain."""
    key = BASE.get(key, key)
    kind = METHODS[key][2]
    if kind[0] == "path2":
        from followers.pathplanner2 import plan_time

        o = options(key)  # native carries the row's handoffs and end tolerance
        return plan_time(native, o.get("accel_cap"), gains=(o.get("kp", 4.0), o.get("kp_ct", 2.0), o.get("kp_rot", 5.0)))
    if kind[1] == "ours_v2":
        from followers.ours_v2 import call_fresh, robot_json

        mu = kind[2].get("mu", OURS_MU)
        return sum(call_fresh(op="load", path=n["path"], robot=robot_json(mu))["totalTimeSeconds"] for n in native)
    from followers.pathplanner import PathPlannerPath, RobotConfig, generate_trajectory

    opts = kind[2]
    cfg = RobotConfig() if opts.get("cof") is None else RobotConfig(wheel_cof=opts["cof"])
    total = 0.0
    for course, n in zip(legs, native):
        h0 = course.start_pose[2]
        path = PathPlannerPath(n)
        tx, ty = np.subtract(path.points[1].position, path.points[0].position)
        d = math.hypot(tx, ty)
        v0 = course.plan_start_speed
        start = (v0 * tx / d, v0 * ty / d, 0.0)
        total += generate_trajectory(path, start, h0, cfg, Limits(), n.get("goal_velocity", 0.0))[-1].t
    return total


# ============================================================================================ one run
def reference(legs):
    """The authored reference: every leg's neutral curve, end to end."""
    pts = [NeutralPath(c).points for c in legs]
    return np.vstack(pts), sum(NeutralPath(c).length for c in legs)


def sim_seconds(legs):
    _, length = reference(legs)
    return 4.0 + length / 1.2


def run_job(job, trace=False):
    """job = (target name, method key, seed, pose source) -> metrics."""
    from sim import DisturbedPlant, end_metrics, make_robot, simulate

    name, key, seed, pose_src = job
    legs = targets()[name]
    first = legs[0]
    rp, rng = make_robot(first, seed)
    plant = DisturbedPlant(rp, first.start_pose, first.start_velocity, rng)
    if first.impulse is not None:
        plant.arm(first.impulse, NeutralPath(first))
    if first.grip_drop is not None:
        plant.arm_grip_drop(first.grip_drop, NeutralPath(first))
    f = make(key)
    native = optimised_native(name, key) if key in BASE else f.adapt(legs)
    plan = planned_seconds(key, legs, native)
    T = sim_seconds(legs)
    r = simulate(f, native, first.start_pose, first.start_velocity, rp, rng, T, pose_src, plant,
                 np.random.default_rng([seed, 7919]))
    end = legs[-1].end_pose
    dist, eth, t_5cm, t_settle = end_metrics(r, end)
    xs, ys = r["x"], r["y"]
    ref, _ = reference(legs)
    reached = np.flatnonzero(dist < 0.05)
    n_ct = (reached[0] + 1) if len(reached) else len(xs)
    d = polyline_distance(ref, np.column_stack([xs[:n_ct], ys[:n_ct]]))
    tail = legs[-1]
    npath = NeutralPath(tail)
    tx, ty = npath.tangent_at_s(npath.length)
    # overshoot: past the end along its tangent, once the robot is within 30 cm of it (a chain may cross that
    # line earlier, on another leg)
    near = np.flatnonzero(dist < 0.3)
    i0 = near[0] if len(near) else len(xs)
    over = float(max(0.0, ((xs[i0:] - end[0]) * tx + (ys[i0:] - end[1]) * ty).max())) if i0 < len(xs) else 0.0
    out = {
        "plan_s": float("nan") if plan is None else plan,
        "t_5cm": t_5cm,
        "t_settle": t_settle,
        "err_end_cm": 100 * float(dist[-1]),
        "yaw_end_deg": math.degrees(float(eth[-1])),
        "overshoot_cm": 100 * over,
        "ct_rms_cm": 100 * float(np.sqrt(np.mean(d ** 2))),
        "ct_max_cm": 100 * float(d.max()),
        "wheel_slip_ms": 1000 * r["wheel_slip"],
        "t_sim": T,
    }
    if trace:
        k = max(1, int(round(0.04 / LP.odom_period)))
        out["trace"] = [[round(float(a), 3), round(float(b), 4), round(float(c), 4)]
                        for a, b, c in zip(r["t"][::k], xs[::k], ys[::k])]
    return out


def _job(args):
    return run_job(args)


def run(seeds, procs, poses=("truth", "odom"), names=None, methods=None):
    import multiprocessing as mp

    from run import agg

    names = names or list(targets())
    pending = optimiser_status()
    methods = methods or RUN_METHODS + ([] if pending else OPTIMISED)
    t0 = time.time()
    out = {"meta": {"seeds": seeds, "poses": list(poses), "targets": names, "methods": methods,
                    "pending": {m: pending for m in OPTIMISED if m not in methods},
                    "t_sim": {n: sim_seconds(targets()[n]) for n in names}},
           "runs": {}}
    with mp.get_context("spawn").Pool(procs) as pool:
        if any(m in OPTIMISED for m in methods):
            out["optimiser"] = prepare_optimised(pool, names)
            print("optimised", f"({time.time() - t0:.0f} s)", flush=True)
        for pose in poses:
            jobs = [(n, m, s, pose) for n in names for m in methods for s in seeds]
            res = pool.map(_job, jobs, chunksize=1)
            by = {}
            for (n, m, _, _), row in zip(jobs, res):
                by.setdefault(n, {}).setdefault(m, []).append(row)
            out["runs"][pose] = {n: {m: agg(v) for m, v in ms.items()} for n, ms in by.items()}
            print(pose, f"done ({time.time() - t0:.0f} s)", flush=True)
    out["meta"]["wall_s"] = time.time() - t0
    return out


# ============================================================================================ report
def _t(a, key):
    n, k = a["n"], a[key + "_nan"]
    if k == n:
        return "never"
    return f"{a[key + '_mean']:.2f}" + ("" if k == 0 else f" ({n - k}/{n})")


def table(out, pose, name):
    runs = out["runs"][pose][name]
    L = ["| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | "
         "end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for m in out["meta"]["methods"]:
        a = runs[m]
        gap = "-" if a["t_5cm_mean"] is None or a["plan_s_mean"] is None else f"{a['t_5cm_mean'] - a['plan_s_mean']:+.2f}"
        plan = "fails" if a["plan_s_mean"] is None else f"{a['plan_s_mean']:.2f}"
        L.append(f"| {METHODS[m][0]} | {METHODS[m][1]} | {plan} | {_t(a, 't_5cm')} | "
                 f"{_t(a, 't_settle')} | {gap} | {a['err_end_cm_mean']:.1f} / {a['err_end_cm_worst']:.1f} | "
                 f"{a['yaw_end_deg_worst']:.1f} | {a['overshoot_cm_worst']:.1f} | "
                 f"{a['ct_rms_cm_mean']:.1f} / {a['ct_max_cm_worst']:.1f} | {a['wheel_slip_ms_mean']:.0f} |")
    for m, why in out["meta"]["pending"].items():
        L.append(f"| {METHODS[m][0]} | {METHODS[m][1]} | {why or 'not run'} |" + " |" * 8)
    return "\n".join(L)


def summary(out, pose):
    """Mean over the targets of each method's time to 5 cm and planned time, relative to ours."""
    names = out["meta"]["targets"]
    runs = out["runs"][pose]
    L = ["| method | mean plan s | mean to 5 cm s | vs ours to 5 cm | targets reached every seed | "
         "mean end error cm | mean wheel slip ms |", "|---|---|---|---|---|---|---|"]
    ours = [runs[n]["ours_v2"]["t_5cm_mean"] for n in names]
    for m in out["meta"]["methods"]:
        rows = [runs[n][m] for n in names]
        t5 = [r["t_5cm_mean"] for r in rows]
        every = sum(1 for r in rows if r["t_5cm_nan"] == 0)
        both = [(a, b) for a, b in zip(t5, ours) if a is not None and b is not None]
        rel = np.mean([a / b - 1 for a, b in both]) if both else float("nan")
        plans = [r["plan_s_mean"] for r in rows if r["plan_s_mean"] is not None]
        L.append(f"| {METHODS[m][1]} | {np.mean(plans):.2f} | "
                 f"{np.mean([x for x in t5 if x is not None]):.2f} | {rel:+.0%} | {every}/{len(rows)} | "
                 f"{np.mean([r['err_end_cm_mean'] for r in rows]):.1f} | "
                 f"{np.mean([r['wheel_slip_ms_mean'] for r in rows]):.0f} |")
    return "\n".join(L)


def section_md(out):
    titles = {"truth": "Perfect pose", "odom": "Wheel odometry + gyro"}
    L = [PPC_START,
         f"_Generated by `python bench/run.py --pp-compare` ({len(out['meta']['seeds'])} seeds per cell, "
         f"{out['meta']['wall_s']:.0f} s wall)._\n",
         "Plan = each tool's own figure for the path as drawn, and they are not measured the same way. Ours "
         "(`Path.totalTimeSeconds`) and PathPlanner 2027's (the trajectory's total time) are plans: a time-parameterised "
         "profile the follower then tracks. path2 has no plan: its figure is the GUI's 50 Hz simulation of its own "
         "follower and setpoint generator on an ideal robot, a simulated run that ends at the follower's end tolerance "
         "(10 cm as shipped). path2 is PathPlanner's new-path branch (af86ef2b), a pre-release: no PathPlannerLib "
         "release ships it yet. "
         "To 5 cm / settled: on the bench plant, from the start (settled = 2 cm, 1 deg, under 5 cm/s, held "
         "0.25 s). sim - plan = time to 5 cm minus the plan. Cross-track is against the authored reference "
         "(the neutral curve through the same waypoints; path2 drives the polyline between them, so its "
         "cross-track there is the cut of its corners).\n",
         "### 2026 routes\n",
         "| route | legs | what |\n|---|---|---|"]
    for name, r in ROUTES.items():
        legs = r["legs"]
        pts = "; ".join(" -> ".join(f"({w.x:g}, {w.y:g}" + ("" if w.heading_deg is None else f", {w.heading_deg:.0f} deg")
                                    + ")" for w in lg.waypoints)
                        + ("" if lg.end_speed == 0 else f" at {lg.end_speed:g} m/s") for lg in legs)
        L.append(f"| {name} | {pts} | {r['description']} |")
    L.append("")
    if "optimiser" in out:
        tools_run = [t for t in OPT_TOOLS if all(t in v for v in out["optimiser"].values())]
        L += ["### The optimisers, rows (c) and (d)\n",
              "Each tool's own optimiser, as it ships, scored by that tool's own figure. (c) ours: "
              f"`tools/optimise.mjs` at its default budget ({BUDGET_MS / 1000:g} s = {BUDGET_MS // 2} plans per path "
              "file), seed 1, curves only: the tangent direction and scale at every waypoint change; waypoints and "
              "headings never move. (d) PathPlanner 2027: `pp_optimizer.py`, a port of the GUI's optimiser "
              "(`lib/util/path_optimizer.dart`): a genetic algorithm, 50 paths for 100 generations (4,551 plans per "
              "path file), that turns each waypoint's handles by up to 5 deg and changes their lengths by up to "
              "0.1 m, anchors fixed; seeded. The extra rows move waypoints: ours with `--move-waypoints` (within "
              "0.5 m, headings kept), and our search (`pp_objective.py`, a port of `optimiser.ts`) on PathPlanner "
              "2027's anchors and handles and on path2's waypoints. PathPlanner has no optimiser that moves anchors, "
              "and none for path2. The bench has no obstacles, so a moved waypoint can cut into the hub, bumps or "
              "trench arms. Planned time before -> after, summed over the target's path files:\n",
              "| target | " + " | ".join(OPT_TITLES[t] for t in tools_run) + " |",
              "|---|" + "---|" * len(tools_run)]
        for name, tools in out["optimiser"].items():
            cells = []
            for t in tools_run:
                r = tools[t]
                if t.startswith("ours"):
                    pairs = [tuple(float(v) for v in x.split(": ")[1].split(" (")[0].replace(" s", "").split(" -> "))
                             for x in r]
                else:
                    pairs = [(x["before"], x["after"]) for x in r]
                b0, b1 = sum(p[0] for p in pairs), sum(p[1] for p in pairs)
                cells.append(f"{b0:.2f} -> {b1:.2f} ({(b1 - b0) / b0:+.0%})")
            L.append(f"| {name} | " + " | ".join(cells) + " |")
        L.append("")
    for pose in out["runs"]:
        L.append(f"### {titles[pose]}\n")
        L.append("All targets (the 2026 routes and the existing courses):\n")
        L.append(summary(out, pose) + "\n")
        for name in out["meta"]["targets"]:
            L.append(f"**{name}**: {description(name)} ({out['meta']['t_sim'][name]:.1f} s simulated)\n")
            L.append(table(out, pose, name) + "\n")
    L.append(PPC_END)
    return "\n".join(L)


def write_md(out):
    """Replaces the PathPlanner comparison block of RESULTS.md (added before '## Fidelity caveats' if missing)."""
    text = open(MD, encoding="utf-8").read()
    block = section_md(out)
    if PPC_START in text:
        i, j = text.index(PPC_START), text.index(PPC_END) + len(PPC_END)
        text = text[:i] + block + text[j:]
    else:
        k = text.index("## Fidelity caveats")
        text = text[:k] + "## Path speed: PathPlanner and ours\n\n" + block + "\n\n" + text[k:]
    open(MD, "w", encoding="utf-8").write(text)


def tune_path2(procs):
    """path2's sweep (run.path2_grid on the tuning seeds over the courses) when the main run hasn't done it."""
    import multiprocessing as mp

    import run as bench

    with mp.get_context("spawn").Pool(procs) as pool:
        best, table = bench.sweep(pool, "pp_path2", bench.path2_grid(False), list(COURSES))
    os.makedirs(OUT, exist_ok=True)
    with open(TUNED_FILES[1], "w") as fh:
        json.dump({"tuned": {"pp_path2_tuned": best}, "sweeps": {"pp_path2": table}}, fh)
    print("PathPlanner path2 tuned:", best, flush=True)
    return best


def main(seeds, procs, names=None):
    if path2_tuned() is None:
        tune_path2(procs)
    out = run(seeds, procs, names=names)
    out["meta"]["path2_tuned"] = path2_tuned()
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "ppcompare.json")
    with open(path, "w") as fh:
        json.dump(out, fh)
    write_md(out)
    print("wrote", path, "and the PathPlanner comparison block of", MD)
    return out


# ============================================================================================ the optimiser
def _optimise_job(args):
    name, tool = args
    if tool.startswith("ours"):
        return [optimise_ours(leg, mu=tool_mu(tool), move_waypoints=tool == "ours_moved") for leg in targets()[name]]
    if tool == "pp2027ga":
        import pp_optimizer

        return pp_optimizer.optimise_target(name)
    from pp_objective import optimise_target

    return optimise_target(name, tool, BUDGET_MS / 1000)


def prepare_optimised(pool, names, tools=OPT_TOOLS, path=OPTIMISED_FILE):
    """Runs the optimiser once per target and tool (it is deterministic) into `path`, which the bench runs
    read. Returns the before/after planned times per path file."""
    jobs = [(n, t) for n in names for t in tools]
    res = pool.map(_optimise_job, jobs, chunksize=1)
    store, report = {}, {}
    for (n, t), r in zip(jobs, res):
        if t.startswith("ours"):
            store.setdefault(n, {})[t] = [tree for tree, _ in r]
            report.setdefault(n, {})[t] = [info for _, info in r]
        else:
            store.setdefault(n, {})[t], report.setdefault(n, {})[t] = r
    os.makedirs(OUT, exist_ok=True)
    with open(path, "w") as fh:
        json.dump({"natives": store, "report": report}, fh)
    global _optimised
    _optimised = None
    return report


_optimised = None


def optimised_native(name, key):
    """The optimised path(s) for a target, as the row's follower takes them."""
    global _optimised
    if _optimised is None:
        _optimised = {}
        for f in (OPTIMISED_FILE, MU_FILE):
            if os.path.exists(f):
                for t, tools in json.load(open(f))["natives"].items():
                    _optimised.setdefault(t, {}).update(tools)
    tool = TOOL[key]
    n = _optimised[name][tool]
    if tool.startswith("ours"):
        from followers.ours_v2 import call_fresh, server

        out = []
        for tree in n:
            call_fresh(op="load", path=tree, robot=_robot(tool_mu(tool)))
            g = server().call(op="geometry", step=0.02)
            out.append({"path": tree, "geometry": np.column_stack([g["x"], g["y"]])})
        return out
    return n


def _robot(mu=OURS_MU):
    from followers.ours_v2 import robot_json

    return robot_json(mu)


def optimise_ours(course, budget_ms=BUDGET_MS, seed=1, mu=OURS_MU, move_waypoints=False):
    """The headless optimiser on one leg of ours: `node tools/optimise.mjs <path.json> --robot <robot.json>
    --field <field.json> --budget-ms N --seed S --out <file> --pin ...`. Waypoints where the course sets a
    heading are pinned (ours authors those headings as rotation targets, which the optimiser doesn't treat
    as a set heading; PathPlanner's side keeps them fixed too). The field is the 2026 field for the routes
    and a large empty one for the bench courses (they aren't laid out on the field). -> (path tree, summary)."""
    from followers.ours_v2 import to_ours

    pins = [str(i) for i, w in enumerate(course.waypoints) if w.heading_deg is not None]
    on_field = any(course.name == leg.name for r in ROUTES.values() for leg in r["legs"])
    field = {"field": {"length_m": 16.54, "width_m": 8.07} if on_field else {"length_m": 60.0, "width_m": 40.0},
             "obstacles": []}
    with tempfile.TemporaryDirectory() as d:
        p, r, f, o = (os.path.join(d, n) for n in ("path.json", "robot.json", "field.json", "out.json"))
        json.dump(to_ours(course), open(p, "w"))
        json.dump(_robot(mu), open(r, "w"))
        json.dump(field, open(f, "w"))
        cmd = ["node", OPTIMISER, p, "--robot", r, "--field", f, "--budget-ms", str(budget_ms), "--seed", str(seed),
               "--out", o]
        if pins:
            cmd += ["--pin", ",".join(pins)]
        if move_waypoints:
            cmd.append("--move-waypoints")
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode == 3:  # blocked: the optimiser writes the path unchanged and says what blocks it
            from followers.ours_v2 import call_fresh

            tree = json.load(open(o))
            t = call_fresh(op="load", path=tree, robot=_robot(mu))["totalTimeSeconds"]
            return tree, f"optimise: {t:.3f} s -> {t:.3f} s (unchanged: {res.stderr.strip()})"
        if res.returncode != 0:
            raise subprocess.CalledProcessError(res.returncode, cmd, res.stdout, res.stderr)
        return json.load(open(o)), res.stderr.strip()
