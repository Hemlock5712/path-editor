"""PathPlanner against ours on path speed: `python bench/run.py --pp-compare`.

Rows per route (five 2026 autos, below) and per existing course:
  (a) PathPlanner as drawn: the 2027 release line (Bezier paths, trajectory generator, PPHolonomicDriveController,
      + hold) as shipped and as tuned in the main sweep, and the new path system on the new-path branch
      ("path2": bare waypoints, a reactive follower and the setpoint generator) as shipped and with the
      shared 6 m/s^2 cap;
  (b) ours as drawn (lib/'s PathFollower, as shipped);
  (c) ours after the editor's waypoint optimiser;
  (d) PathPlanner after the same optimiser on its own control points (pp_objective.py), scored by its own
      generator's time.
(c) and (d) wait for the headless optimiser (tools/optimise.mjs); until it exists their rows say so.

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
    for name, c in COURSES.items():
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
    "pp_path2": ("(a)", "PathPlanner path2 (new-path branch), as shipped", ("path2", {})),
    "pp_path2_cap6": ("(a)", "PathPlanner path2, setpoint generator capped at 6 m/s^2", ("path2", {"accel_cap": 6.0})),
    "pp_path2_tight": ("(a)", "PathPlanner path2, 6 m/s^2 cap, end tolerance 2 cm / 1 deg",
                       ("path2", {"accel_cap": 6.0, "end_distance": 0.02, "end_degrees": 1.0})),
    "ours_v2": ("(b)", "Ours v2, as shipped", ("chain", "ours_v2", {})),
    "ours_optimised": ("(c)", "Ours v2 after the waypoint optimiser", ("optimised", "ours")),
    "pp2027_optimised": ("(d)", "PathPlanner 2027 after the optimiser (anchors + handles), + hold",
                         ("optimised", "pp2027")),
    "pp_path2_optimised": ("(d)", "PathPlanner path2 after the optimiser (waypoints)", ("optimised", "path2")),
}
RUN_METHODS = ["pp2027_hold", "pp2027_tuned", "pp_path2", "pp_path2_cap6", "pp_path2_tight", "ours_v2"]
PENDING = ["ours_optimised", "pp2027_optimised", "pp_path2_optimised"]
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

    kind = METHODS[key][2]
    if kind[0] == "chain":
        return Chain(kind[1], **kind[2])
    if kind[0] == "path2":
        return PathPlanner2(**kind[1])
    raise ValueError(f"{key}: {optimiser_status() or 'not wired yet'}")


# ============================================================================================ planned time
def planned_seconds(key, legs, native):
    """Each tool's own generator time for the authored path(s), the way its editor shows it: from the plan's
    start speed (0, or the handoff speed), summed over the chain."""
    kind = METHODS[key][2]
    if kind[0] == "path2":
        from followers.pathplanner2 import plan_time

        return plan_time(native, kind[1].get("accel_cap"))  # native carries the row's end tolerance
    if kind[1] == "ours_v2":
        from followers.ours_v2 import call_fresh, robot_json

        return sum(call_fresh(op="load", path=n["path"], robot=robot_json(0.7))["totalTimeSeconds"] for n in native)
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
    native = f.adapt(legs)
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
    methods = methods or RUN_METHODS
    t0 = time.time()
    out = {"meta": {"seeds": seeds, "poses": list(poses), "targets": names, "methods": methods,
                    "pending": {m: optimiser_status() for m in PENDING},
                    "t_sim": {n: sim_seconds(targets()[n]) for n in names}},
           "runs": {}}
    with mp.get_context("spawn").Pool(procs) as pool:
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
    for m in PENDING:
        L.append(f"| {METHODS[m][0]} | {METHODS[m][1]} | {out['meta']['pending'][m] or 'not run'} |"
                 + " |" * 8)
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
         "Plan = each tool's own generator for the path as drawn (ours: `Path.totalTimeSeconds`; PathPlanner 2027: "
         "the trajectory's total time; path2: the GUI's simulation, which stops at its 10 cm end tolerance). "
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


def main(seeds, procs, names=None):
    out = run(seeds, procs, names=names)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "ppcompare.json")
    with open(path, "w") as fh:
        json.dump(out, fh)
    write_md(out)
    print("wrote", path, "and the PathPlanner comparison block of", MD)
    return out


# ============================================================================================ optimiser hook
def optimise_ours(course, budget_ms=5000, seed=1):
    """Runs the headless optimiser on one leg: `node tools/optimise.mjs <path.json> --robot <robot.json>
    --field <field.json> --budget-ms N --seed S`, expected to print the optimised path file on stdout."""
    from followers.ours_v2 import robot_json, to_ours

    with tempfile.TemporaryDirectory() as d:
        p, r, f = (os.path.join(d, n) for n in ("path.json", "robot.json", "field.json"))
        json.dump(to_ours(course), open(p, "w"))
        json.dump(robot_json(0.7), open(r, "w"))
        json.dump({"lengthMeters": 16.54, "widthMeters": 8.07, "obstacles": []}, open(f, "w"))
        res = subprocess.run(["node", OPTIMISER, p, "--robot", r, "--field", f, "--budget-ms", str(budget_ms),
                              "--seed", str(seed)], capture_output=True, text=True, check=True)
        return json.loads(res.stdout)
