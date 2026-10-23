"""On-the-fly head to head: ours (OnTheFly) against PathPlanner's AD* + hold (the port) and BLine fed our route.

    python bench/h2h.py run [--pairs 40] [--procs 6] [--methods ours,pp,bline] [--out bench/out/h2h.jsonl]
    python bench/h2h.py summary [--out bench/out/h2h.jsonl]

Six fields, `--pairs` seeded start/goal pairs each: the 2026 field, an open field, narrow gaps, a U-shaped trap,
dense clutter and a dog-leg corridor. Every method plans at 4 m/s and 6 m/s^2. Ours runs the library defaults
the way the robot's Pathfinding commands build it: the 0.15 m pathfinder first, the 0.05 m one when the wide
route is much longer or there is none.

Modes (each pair):
- rest: start at rest, perfect pose ("truth") and wheel odometry ("odom");
- moving: the robot already moving at 1.5-3.5 m/s (odometry; the first 12 pairs);
- movegoal: the goal jumps 0.5 m and 20 deg one second in (both poses; the first 12 pairs);
- replan: a re-plan forced one second in (odometry, ours only; the first 12 pairs).

Success = finished, no bumper contact, and within 2 cm (perfect pose) or 10 cm (odometry) of the goal.
Results append one JSON line per run; `run` skips runs already in the file.
"""

import argparse
import json
import math
import os
import sys
import time
from collections import defaultdict
from dataclasses import replace
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from common import RP, Limits  # noqa: E402
from followers.base import Follower, Start  # noqa: E402
from pathfind import LocalADStar, PathfindingCommand, load_field, point_poly_distance, swept_clearance  # noqa: E402,F401

ROBOT = {"bumper_length_m": 0.85, "bumper_width_m": 0.85}
L, W = 16.54, 8.07
HL, HW = L / 2, W / 2
OUT = os.path.join(HERE, "out", "h2h.jsonl")
EXTRA_PAIRS = 12  # pairs per field that also run the moving, movegoal and replan modes


def rect(x0, y0, x1, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def mk(polys):
    return {"field": {"length_m": L, "width_m": W, "origin": "centre"}, "robot": ROBOT,
            "obstacles": [{"name": f"o{i}", "polygon": p} for i, p in enumerate(polys)]}


def fields():
    out = {}
    f26 = json.load(open(os.path.join(HERE, "..", "fixtures", "plan", "field_2026.json")))
    out["real2026"] = {"field": {"length_m": f26["field"]["length_m"], "width_m": f26["field"]["width_m"]},
                       "robot": ROBOT, "obstacles": [{"name": o["name"], "polygon": o["polygon"]} for o in f26["obstacles"]]}
    # an open field, a few big obstacles
    out["open"] = mk([rect(-5.0, -2.5, -3.0, -0.5), rect(-1.0, 0.5, 1.5, 3.0), rect(2.5, -3.0, 4.0, -1.0),
                      [[4.5, 1.0], [6.5, 0.5], [5.5, 2.8]], rect(-6.5, 1.5, -5.5, 3.2)])
    # narrow gaps. x=-3: a 1.2 m gap (the robot fits square-on: 0.85 m) and a 1.6 m gap by the top wall;
    # x=+3: a single 1.35 m gap (just over the planners' 2 x 0.65 m grown width)
    out["gap"] = mk([rect(-3.15, -HW, -2.85, -0.6), rect(-3.15, 0.6, -2.85, HW - 1.6),
                     rect(2.85, -HW, 3.15, -0.2), rect(2.85, 1.15, 3.15, HW)])
    # a U trap opening toward -x (toward the starts), goals behind its base
    out["utrap"] = mk([rect(1.0, -2.0, 1.3, 2.0), rect(-1.5, 1.7, 1.0, 2.0), rect(-1.5, -2.0, 1.0, -1.7)])
    # dense clutter: 20 small squares and triangles, 0.25-0.5 m, seeded
    rng = np.random.default_rng(7)
    polys, centres = [], []
    while len(polys) < 20:
        x, y = rng.uniform(-6.5, 6.5), rng.uniform(-3.2, 3.2)
        if any(math.hypot(x - a, y - b) < 1.6 for a, b in centres):
            continue
        s = rng.uniform(0.25, 0.5)
        centres.append((x, y))
        if rng.random() < 0.6:
            polys.append(rect(x - s / 2, y - s / 2, x + s / 2, y + s / 2))
        else:
            a = rng.uniform(0, 2 * math.pi)
            polys.append([[x + s * math.cos(a + k * 2 * math.pi / 3), y + s * math.sin(a + k * 2 * math.pi / 3)]
                          for k in range(3)])
    out["clutter"] = mk(polys)
    # a 2 m corridor with a dog-leg: along y=-1.5 from the left, up a 2 m wide leg at x=-1.5, along y=+1.6
    out["corridor"] = mk([rect(-HL, -HW, HL, -2.5), rect(-HL, -0.5, -2.5, HW), rect(-0.5, -2.5, HL, 0.6),
                          rect(-2.5, 2.6, HL, HW)])
    return out


FIELDS = fields()
FIELD_SEEDS = {"real2026": 1, "open": 2, "gap": 3, "utrap": 4, "clutter": 5, "corridor": 6}


def free_clear(field, x, y):
    hx, hy = field["field"]["length_m"] / 2, field["field"]["width_m"] / 2
    d = min(hx - abs(x), hy - abs(y))
    for o in field["obstacles"]:
        d = min(d, point_poly_distance((x, y), o["polygon"]))
    return d


def pairs(fname, n, seed):
    field = FIELDS[fname]
    rng = np.random.default_rng(seed)
    out = []
    need = 0.72
    while len(out) < n:
        if fname == "utrap" and len(out) % 2 == 0:
            s = (rng.uniform(-6.0, -2.5), rng.uniform(-1.2, 1.2))
            g = (rng.uniform(2.2, 5.0), rng.uniform(-1.2, 1.2))
        elif fname == "corridor":
            s = (rng.uniform(-7.4, -3.0), rng.uniform(-1.75, -1.25))
            g = (rng.uniform(0.5, 7.4), rng.uniform(1.35, 1.85))
            if len(out) % 2:
                s, g = g, s
        else:
            s = (rng.uniform(-7.4, 7.4), rng.uniform(-3.4, 3.4))
            g = (rng.uniform(-7.4, 7.4), rng.uniform(-3.4, 3.4))
        if free_clear(field, *s) < need or free_clear(field, *g) < need or math.dist(s, g) < 1.5:
            continue
        out.append(((s[0], s[1], rng.uniform(-math.pi, math.pi)), (g[0], g[1], rng.uniform(-math.pi, math.pi))))
    return out


def moved_goal(fname, goal, idx):
    """A goal 0.5 m away from `goal` (seeded direction) that is still free, for the moving-goal runs."""
    rng = np.random.default_rng(900 + idx)
    for _ in range(200):
        a = rng.uniform(-math.pi, math.pi)
        g = (goal[0] + 0.5 * math.cos(a), goal[1] + 0.5 * math.sin(a), goal[2] + math.radians(20))
        if free_clear(FIELDS[fname], g[0], g[1]) >= 0.72:
            return g
    return None


def robot_file():
    """The bench plant's robot file at the library's default traction share (0.9)."""
    from followers.ours_v2 import OURS_MU, robot_json

    return dict(robot_json(OURS_MU), tractionShare=0.9)


BLINE_TUNED = {"kp": 4.0, "kp_ct": 4.0, "tol": 0.01, "tol_deg": 0.5, "radius": 0.1, "caps": True}


class Ours(Follower):
    """OnTheFly with the library defaults: plan, follow, re-plan after a shove or a moved goal."""

    def __init__(self, replan_at=None, goal_move=None):
        self.replan_at, self.goal_move = replan_at, goal_move
        self.replanned = self.moved = False

    def reset(self, native, start):
        from followers.ours_v2 import call_fresh, server

        msg = dict(op="otf", field=native["field"], start=[float(v) for v in start.pose],
                   vel=[float(v) for v in start.vel], goal=[float(v) for v in native["goal"]],
                   robot=robot_file(), t=float(start.t), movingGoal=self.goal_move is not None)
        call_fresh(**msg)  # robot init's warm-up
        self.plan = call_fresh(**msg)
        self.srv = server()
        self.finished = not self.plan["planOk"]
        self.last = {}

    def step(self, t, pose, vel):
        if self.replan_at is not None and not self.replanned and t >= self.replan_at:
            self.srv.call(op="replan")
            self.replanned = True
        if self.goal_move is not None and not self.moved and t >= self.goal_move[0]:
            self.srv.call(op="otfgoal", goal=[float(v) for v in self.goal_move[1]])
            self.moved = True
        r = self.srv.call(op="step", t=float(t), pose=[float(v) for v in pose], vel=[float(v) for v in vel])
        self.finished = r["done"]
        self.last = r
        return tuple(r["cmd"])

    def done(self):
        return self.finished


_planners = {}


def adstar(fname):
    if fname not in _planners:
        _planners[fname] = LocalADStar(FIELDS[fname])
    return _planners[fname]


class PP(Follower):
    """The port's PathfindingCommand + hold. A moving goal restarts the command from the measured state (what a
    team does with pathfindToPose: PathPlanner's pathfinding takes a fixed goal)."""

    def __init__(self, fname, goal_move=None):
        self.fname, self.goal_move = fname, goal_move
        self.cmd = PathfindingCommand(hold=True)
        self.moved = False
        self.plan_ms = []

    def _plan(self, start_pose, goal):
        t0 = time.perf_counter()
        steps = adstar(self.fname).plan(start_pose[:2], goal[:2])
        self.plan_ms.append((steps[0][2], 1000 * (time.perf_counter() - t0)))
        return steps

    def reset(self, native, start):
        steps = self._plan(start.pose, native["goal"])
        self.steps = steps
        self.ok = bool(steps and steps[-1][1])
        self.cmd.reset({"paths": [s[1] for s in steps], "goal": native["goal"]}, start)

    def step(self, t, pose, vel):
        if self.goal_move is not None and not self.moved and t >= self.goal_move[0]:
            self.moved = True
            g = self.goal_move[1]
            steps = self._plan(pose, g)
            self.cmd = PathfindingCommand(hold=True)
            self.cmd.reset({"paths": [s[1] for s in steps], "goal": g}, Start(t, pose, vel))
        return self.cmd.step(t, pose, vel)

    def done(self):
        return self.cmd.done()


def bline_elements(points, start, goal):
    els = [{"kind": "T", "xy": tuple(points[0]), "radius": BLINE_TUNED["radius"]},
           {"kind": "R", "rot": start[2], "t_ratio": 0.0, "profiled": True}]
    for i, p in enumerate(points[1:], 1):
        if i == len(points) - 1:
            els.append({"kind": "R", "rot": goal[2], "t_ratio": 1.0, "profiled": True})
        els.append({"kind": "T", "xy": tuple(p), "radius": BLINE_TUNED["radius"]})
    return els


def polyline_len(pts):
    return float(sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)))


def ours_route(fname, start, goal, v0):
    """Ours' corner route (the polyline the spline is fitted through), from a throwaway plan."""
    from followers.ours_v2 import call_fresh

    return call_fresh(op="otf", field=FIELDS[fname], start=list(start), vel=list(v0), goal=list(goal),
                      robot=robot_file(), t=0.0)


def run(job):
    from courses import bline_corner_caps
    from followers.bline import BLine
    from sim import DisturbedPlant, end_metrics, simulate

    fname, method, idx, start, goal, v0, pose_src, mode = job
    field = FIELDS[fname]
    rng = np.random.default_rng(1000 + idx)
    rp = replace(RP, wheel_radius_error=rng.normal(0, 0.01), gyro_scale_error=rng.normal(0, 0.002))
    out = {"field": fname, "method": method, "idx": idx, "pose": pose_src, "mode": mode,
           "straight_m": math.dist(start[:2], goal[:2])}
    end_goal = goal
    goal_move = None
    if mode == "movegoal":
        g2 = moved_goal(fname, goal, idx)
        if g2 is None:
            out["skip"] = True
            return out
        goal_move = (1.0, g2)
        end_goal = g2
    ref = ours_route(fname, start, goal, v0)
    out["shortest_m"] = ref.get("routeLengthMeters") if ref.get("planOk") else None
    native = {"field": field, "goal": goal}
    if method == "ours":
        f = Ours(replan_at=1.0 if mode == "replan" else None, goal_move=goal_move)
    elif method == "pp":
        f = PP(fname, goal_move=goal_move)
    elif method == "bline":
        if not ref.get("planOk"):
            out["planOk"] = False
            out["failure"] = ref.get("failure")
            return out
        pts = [tuple(p) for p in ref["waypoints"]]
        lim = Limits()
        els = bline_corner_caps(bline_elements(pts, start, goal), lim.max_speed, lim.max_accel)
        f = BLine(kp=BLINE_TUNED["kp"], kp_ct=BLINE_TUNED["kp_ct"], tol=BLINE_TUNED["tol"],
                  tol_deg=BLINE_TUNED["tol_deg"], radius=BLINE_TUNED["radius"], caps=True, limits=lim)
        native = {"elements": els}
        out["path_m"] = polyline_len(pts)
    else:
        raise ValueError(method)
    plant = DisturbedPlant(rp, start, v0, rng)
    r = simulate(f, native, start, v0, rp, rng, 15.0, pose_src, plant)
    if method == "ours":
        p = f.plan
        out.update(planOk=p["planOk"], plan_ms=p.get("planMs"), path_m=p.get("lengthMeters"),
                   plan_min_clear_cm=100 * p.get("minClearanceMeters", float("nan")), failure=p.get("failure"),
                   replans=f.last.get("replans", 0), max_plan_ms=f.last.get("maxPlanMicros", 0) / 1e3)
        if not p["planOk"]:
            return out
    elif method == "pp":
        wps = f.steps[-1][1] if f.steps else []
        out.update(planOk=f.ok, plan_ms=f.plan_ms[0][1], path_m=polyline_len([w["anchor"] for w in wps]) if wps else None)
        if not f.ok:
            return out
    else:
        out["planOk"] = True
    dist, eth, t5, ts = end_metrics(r, end_goal)
    clear, hits = swept_clearance(r["x"], r["y"], r["th"], field)
    out.update(t_settle=ts, t_done=r["t_done"], err_cm=100 * float(dist[-1]), yaw_deg=math.degrees(float(eth[-1])),
               clear_cm=100 * clear, contact_ms=hits * 4.0, slip_ms=1000 * r["wheel_slip"], odo_err_cm=r["odo_err_cm"])
    return out


def jobs(n, methods):
    js = []
    z = (0.0, 0.0, 0.0)
    for fname in FIELDS:
        for i, (s, g) in enumerate(pairs(fname, n, FIELD_SEEDS[fname])):
            for m in methods:
                for pose in ("truth", "odom"):
                    js.append((fname, m, i, s, g, z, pose, "rest"))
            if i < EXTRA_PAIRS:
                rng = np.random.default_rng(50 + i)
                sp, a = rng.uniform(1.5, 3.5), rng.uniform(-math.pi, math.pi)
                v0 = (sp * math.cos(a), sp * math.sin(a), 0.0)
                for m in methods:
                    js.append((fname, m, i, s, g, v0, "odom", "moving"))
                    if m != "bline":  # BLine has no pathfinder to re-plan with
                        js.append((fname, m, i, s, g, z, "odom", "movegoal"))
                        js.append((fname, m, i, s, g, z, "truth", "movegoal"))
                if "ours" in methods:
                    js.append((fname, "ours", i, s, g, z, "odom", "replan"))
    return js


def safe_run(job):
    try:
        return run(job)
    except Exception as e:  # keep going; record it
        return {"field": job[0], "method": job[1], "idx": job[2], "pose": job[6], "mode": job[7], "error": repr(e)}


# ============================================================================================ summary
def ok(r):
    tol = 2.0 if r["pose"] == "truth" else 10.0
    return r.get("planOk") and r.get("t_done") is not None and r["contact_ms"] == 0 and r["err_cm"] < tol


def _fin(v):
    return [x for x in v if x is not None and not (isinstance(x, float) and math.isnan(x))]


def _pct(v, q):
    v = _fin(v)
    return float(np.percentile(v, q)) if v else float("nan")


def group_stats(rs):
    """One group's numbers: runs, routes found, successes, runs with contact, times and planning."""
    planned = [r for r in rs if r.get("planOk")]
    clear = [r["clear_cm"] for r in planned]
    times = [r.get("t_settle") if r["pose"] == "truth" else r.get("t_done") for r in planned]
    return {"runs": len(rs), "noRoute": len(rs) - len(planned), "success": sum(1 for r in planned if ok(r)),
            "contact": sum(1 for r in planned if r["contact_ms"] > 0),
            "unfinished": sum(1 for r in planned if r.get("t_done") is None),
            "minClearCm": min(clear, default=float("nan")),
            "endErrP50": _pct([r["err_cm"] for r in planned], 50), "endErrP95": _pct([r["err_cm"] for r in planned], 95),
            "timeP50": _pct(times, 50), "timeSum": float(sum(_fin(times))),
            "planP50": _pct([r.get("plan_ms") for r in planned], 50)}


def summary(path):
    rows = [json.loads(line) for line in open(path)]
    errors = [r for r in rows if "error" in r]
    rows = [r for r in rows if "error" not in r and not r.get("skip")]
    g = defaultdict(list)
    for r in rows:
        g[(r["mode"], r["pose"], r["method"])].append(r)
    print("| mode | pose | method | runs | no route | success | contact runs | unfinished | min clear cm | "
          "end err cm p50 / p95 | time p50 / sum s | plan ms p50 |")
    print("|" + "---|" * 12)
    for k in sorted(g):
        s = group_stats(g[k])
        print(f"| {k[0]} | {k[1]} | {k[2]} | {s['runs']} | {s['noRoute']} | {s['success']} | {s['contact']} | "
              f"{s['unfinished']} | {s['minClearCm']:.1f} | {s['endErrP50']:.1f} / {s['endErrP95']:.1f} | "
              f"{s['timeP50']:.2f} / {s['timeSum']:.1f} | {s['planP50']:.2f} |")
    print("\nerrors:", len(errors), errors[:3])
    return g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "summary"])
    ap.add_argument("--pairs", type=int, default=40)
    ap.add_argument("--procs", type=int, default=6)
    ap.add_argument("--methods", default="ours,pp,bline")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()
    if a.cmd == "summary":
        summary(a.out)
        return
    js = jobs(a.pairs, a.methods.split(","))
    done = set()
    if os.path.exists(a.out):
        for line in open(a.out):
            r = json.loads(line)
            done.add((r["field"], r["method"], r["idx"], r["pose"], r["mode"]))
    js = [j for j in js if (j[0], j[1], j[2], j[6], j[7]) not in done]
    print(len(js), "runs to go", flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    t0 = time.time()
    with Pool(a.procs) as pool, open(a.out, "a") as fh:
        for k, r in enumerate(pool.imap_unordered(safe_run, js, chunksize=2)):
            fh.write(json.dumps(r) + "\n")
            fh.flush()
            if k % 100 == 0:
                print(k, round(time.time() - t0), flush=True)
    print("done", round(time.time() - t0), "s", flush=True)
    summary(a.out)


if __name__ == "__main__":
    main()
