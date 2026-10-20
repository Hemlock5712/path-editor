"""Pathfinding course: four start/goal pairs (bench/fields/routes_v1.json) across bench/fields/obstacles_v1.json,
planned by PathPlanner's pathfinding (AD* on a navgrid, ported below) and by ours (lib/, through BenchServer's
`pathfind` op; OursPathfind).

Port of PathPlannerLib (mjansen4857/pathplanner, branch 2027, 7fbd798):
- pathfinding/LocalADStar: the grid, reset, computeOrImprovePath, updateState, key/topKey, cost, isCollision,
  neighbours, extractPath, createWaypoints (line-of-sight simplification with `walkable`, then the
  SMOOTHING_ANCHOR_PCT 0.8 corner anchors), findClosestNonObstacle, and the anytime schedule of its planning
  thread: eps 2.5 first, then 2.0, 1.5, 1.0;
- path/PathPlannerPath.waypointsFromPoses + Waypoint.autoControlPoints (handles at 0.4 of the anchor gap);
- commands/PathfindingCommand: initialize (finish at once within 0.5 m), execute (take a new path while more
  than 2 m from the end, regenerate the trajectory from the measured speeds, time offset from the two closest
  states, 0.02 s if stationary at the start), isFinished (timer >= total - offset), end (zero speeds).
Trajectory generation and the controller are the bench's PathPlanner port (followers/pathplanner.py).

Approximations:
- The planner runs synchronously before the command starts. The planning thread's four anytime paths are
  handed to the command one per 20 ms loop (the real thread runs asynchronously; on a roboRIO each step
  takes milliseconds to tens of milliseconds).
- PathPlanner's grid has its origin at the field corner; the field file is centre-origin, so the planner
  works in corner coordinates (x + length/2, y + width/2) and converts back.
- The navgrid is built from the polygons: PathPlanner's docs say a node is free only if the robot CENTRE can
  pass through it without hitting anything, so a node is blocked when its centre is within the bumpers'
  half-diagonal (0.60 m, any heading) of an obstacle or a field wall. PathPlanner's GUI has you paint this by
  hand (its default grids have a border along the walls).
"""

import json
import math
import os
import time

import numpy as np

from common import LP, RP, Limits, wrap
from followers.base import Follower
from followers.pathplanner import PathPlannerPath, RobotConfig, generate_trajectory, sample

HERE = os.path.dirname(os.path.abspath(__file__))
FIELD_FILE = os.path.join(HERE, "fields", "obstacles_v1.json")

SMOOTHING_ANCHOR_PCT = 0.8
EPS = 2.5
NODE_SIZE = 0.2
AUTO_CONTROL_DISTANCE_FACTOR = 0.4


def load_field(path=FIELD_FILE):
    with open(path) as fh:
        return json.load(fh)


# ============================================================================================ geometry
def _seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _inside(p, poly):
    """Point in convex CCW polygon."""
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        if (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) < 0:
            return False
    return True


def point_poly_distance(p, poly):
    if _inside(p, poly):
        return 0.0
    return min(_seg_dist(p, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))


def _segs_cross(a, b, c, d):
    def o(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    return o(a, b, c) * o(a, b, d) < 0 and o(c, d, a) * o(c, d, b) < 0


def poly_distance(P, Q):
    """Distance between two convex polygons; 0 when they touch or overlap."""
    if any(_inside(p, Q) for p in P) or any(_inside(q, P) for q in Q):
        return 0.0
    for i in range(len(P)):
        for j in range(len(Q)):
            if _segs_cross(P[i], P[(i + 1) % len(P)], Q[j], Q[(j + 1) % len(Q)]):
                return 0.0
    d = min(_seg_dist(p, Q[j], Q[(j + 1) % len(Q)]) for p in P for j in range(len(Q)))
    return min(d, min(_seg_dist(q, P[i], P[(i + 1) % len(P)]) for q in Q for i in range(len(P))))


def footprint(x, y, th, length, width):
    c, s = math.cos(th), math.sin(th)
    hl, hw = length / 2, width / 2
    # front-left, back-left, back-right, front-right: counter-clockwise
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]


def swept_clearance(xs, ys, ths, field):
    """Minimum distance from the bumper footprint to any obstacle or field wall along a trajectory, and the
    number of samples in contact."""
    L, W = field["robot"]["bumper_length_m"], field["robot"]["bumper_width_m"]
    hx, hy = field["field"]["length_m"] / 2, field["field"]["width_m"] / 2
    polys = [[tuple(p) for p in o["polygon"]] for o in field["obstacles"]]
    best, hits = math.inf, 0
    for x, y, th in zip(xs, ys, ths):
        fp = footprint(x, y, th, L, W)
        d = min(poly_distance(fp, q) for q in polys)
        d = min(d, min(min(hx - abs(px), hy - abs(py)) for px, py in fp))
        d = max(d, 0.0)
        best = min(best, d)
        hits += d <= 0.0
    return best, hits


# ============================================================================================ LocalADStar
class LocalADStar:
    def __init__(self, field, inflate=None):
        self.field = field
        fl, fw = field["field"]["length_m"], field["field"]["width_m"]
        self.off = (fl / 2, fw / 2)  # centre origin -> PathPlanner's corner origin
        self.nodes_x = int(math.ceil(fl / NODE_SIZE))
        self.nodes_y = int(math.ceil(fw / NODE_SIZE))
        r = field["robot"]
        self.inflate = math.hypot(r["bumper_length_m"], r["bumper_width_m"]) / 2 if inflate is None else inflate
        polys = [[(p[0] + self.off[0], p[1] + self.off[1]) for p in o["polygon"]] for o in field["obstacles"]]
        self.obstacles = set()
        for gx in range(self.nodes_x):
            for gy in range(self.nodes_y):
                c = self.grid_to_xy((gx, gy))
                near_wall = min(c[0], fl - c[0], c[1], fw - c[1]) < self.inflate
                if near_wall or any(point_poly_distance(c, q) < self.inflate for q in polys):
                    self.obstacles.add((gx, gy))

    # grid helpers ---------------------------------------------------------------------------------
    def grid_pos(self, p):
        return (int(math.floor(p[0] / NODE_SIZE)), int(math.floor(p[1] / NODE_SIZE)))

    @staticmethod
    def grid_to_xy(g):
        return (g[0] * NODE_SIZE + NODE_SIZE / 2.0, g[1] * NODE_SIZE + NODE_SIZE / 2.0)

    def _all_neighbors(self, s):
        return [(s[0] + dx, s[1] + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                if 0 <= s[0] + dx < self.nodes_x and 0 <= s[1] + dy < self.nodes_y]

    def _open_neighbors(self, s):
        return [n for n in self._all_neighbors(s) if n not in self.obstacles]

    def closest_free(self, pos):
        if pos not in self.obstacles:
            return pos
        visited, queue = set(), list(self._all_neighbors(pos))
        while queue:
            c = queue.pop(0)
            if c not in self.obstacles:
                return c
            visited.add(c)
            for n in self._all_neighbors(c):
                if n not in visited and n not in queue:
                    queue.append(n)
        return None

    @staticmethod
    def h(a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def _collision(self, a, b):
        obs = self.obstacles
        if a in obs or b in obs:
            return True
        if a[0] != b[0] and a[1] != b[1]:
            if b[0] - a[0] == a[1] - b[1]:
                s1, s2 = (min(a[0], b[0]), min(a[1], b[1])), (max(a[0], b[0]), max(a[1], b[1]))
            else:
                s1, s2 = (min(a[0], b[0]), max(a[1], b[1])), (max(a[0], b[0]), min(a[1], b[1]))
            return s1 in obs or s2 in obs
        return False

    def _cost(self, a, b):
        return math.inf if self._collision(a, b) else self.h(a, b)

    def _key(self, s, start):
        g, rhs = self.g[s], self.rhs[s]
        if g > rhs:
            return (rhs + self.eps * self.h(start, s), rhs)
        return (g + self.h(start, s), g)

    # AD* --------------------------------------------------------------------------------------------
    def reset(self, start, goal):
        self.g = {(x, y): math.inf for x in range(self.nodes_x) for y in range(self.nodes_y)}
        self.rhs = dict(self.g)
        self.open, self.incons, self.closed = {}, {}, set()
        self.rhs[goal] = 0.0
        self.eps = EPS
        self.open[goal] = self._key(goal, start)

    def _update_state(self, s, start, goal):
        if s != goal:
            self.rhs[s] = math.inf
            for x in self._open_neighbors(s):
                self.rhs[s] = min(self.rhs[s], self.g[x] + self._cost(s, x))
        self.open.pop(s, None)
        if self.g[s] != self.rhs[s]:
            if s not in self.closed:
                self.open[s] = self._key(s, start)
            else:
                self.incons[s] = (0.0, 0.0)

    def compute_or_improve(self, start, goal):
        while self.open:
            s, v = min(self.open.items(), key=lambda kv: kv[1])
            if v >= self._key(start, start) and self.rhs[start] == self.g[start]:
                break
            del self.open[s]
            if self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                self.closed.add(s)
                for sn in self._open_neighbors(s):
                    self._update_state(sn, start, goal)
            else:
                self.g[s] = math.inf
                for sn in self._open_neighbors(s):
                    self._update_state(sn, start, goal)
                self._update_state(s, start, goal)

    def extract(self, start, goal):
        if start == goal:
            return []
        path, s = [start], start
        for _ in range(200):
            best, bg = goal, math.inf
            for x in self._open_neighbors(s):
                if self.g[x] < bg:
                    best, bg = x, self.g[x]
            s = best
            path.append(s)
            if s == goal:
                break
        return path

    def walkable(self, s1, s2):
        x0, y0, x1, y1 = s1[0], s1[1], s2[0], s2[1]
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        x, y = x0, y0
        n = 1 + dx + dy
        xi, yi = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
        err = dx - dy
        dx, dy = dx * 2, dy * 2
        while n > 0:
            if (x, y) in self.obstacles:
                return False
            if err > 0:
                x += xi
                err -= dy
            elif err < 0:
                y += yi
                err += dx
            else:
                x += xi
                y += yi
                err -= dy
                err += dx
                n -= 1
            n -= 1
        return True

    def create_waypoints(self, path, real_start, real_goal):
        if not path:
            return []
        simp = [path[0]]
        for i in range(1, len(path) - 1):
            if not self.walkable(simp[-1], path[i + 1]):
                simp.append(path[i])
        simp.append(path[-1])
        pts = [self.grid_to_xy(g) for g in simp]
        if len(pts) < 2:
            return []
        pts[0], pts[-1] = real_start, real_goal

        def ang(a, b):
            return math.atan2(b[1] - a[1], b[0] - a[0])

        poses = [(pts[0], ang(pts[0], pts[1]))]
        for i in range(1, len(pts) - 1):
            last, cur, nxt = pts[i - 1], pts[i], pts[i + 1]
            a1 = (last[0] + (cur[0] - last[0]) * SMOOTHING_ANCHOR_PCT, last[1] + (cur[1] - last[1]) * SMOOTHING_ANCHOR_PCT)
            a2 = (nxt[0] + (cur[0] - nxt[0]) * SMOOTHING_ANCHOR_PCT, nxt[1] + (cur[1] - nxt[1]) * SMOOTHING_ANCHOR_PCT)
            poses.append((a1, ang(last, cur)))
            poses.append((a2, ang(a2, nxt)))
        poses.append((pts[-1], ang(pts[-2], pts[-1])))
        return waypoints_from_poses(poses)

    def plan(self, start_xy, goal_xy):
        """The planning thread's sequence for one request: returns [(eps, waypoints in centre coords, ms)]."""
        rs = (start_xy[0] + self.off[0], start_xy[1] + self.off[1])
        rg = (goal_xy[0] + self.off[0], goal_xy[1] + self.off[1])
        start, goal = self.closest_free(self.grid_pos(rs)), self.closest_free(self.grid_pos(rg))
        out = []
        t0 = time.perf_counter()
        self.reset(start, goal)
        self.compute_or_improve(start, goal)
        out.append((self.eps, self._to_centre(self.create_waypoints(self.extract(start, goal), rs, rg)),
                    1000 * (time.perf_counter() - t0)))
        while self.eps > 1.0:
            t0 = time.perf_counter()
            self.eps -= 0.5
            self.open.update(self.incons)
            self.open = {s: self._key(s, start) for s in self.open}
            self.closed.clear()
            self.compute_or_improve(start, goal)
            out.append((self.eps, self._to_centre(self.create_waypoints(self.extract(start, goal), rs, rg)),
                        1000 * (time.perf_counter() - t0)))
        return out

    def _to_centre(self, wps):
        ox, oy = self.off

        def f(p):
            return None if p is None else (p[0] - ox, p[1] - oy)

        return [{"prev": f(w["prev"]), "anchor": f(w["anchor"]), "next": f(w["next"])} for w in wps]


def waypoints_from_poses(poses):
    """PathPlannerPath.waypointsFromPoses with Waypoint.autoControlPoints."""
    out = []
    for i, (p, h) in enumerate(poses):
        prev = poses[i - 1][0] if i > 0 else None
        nxt = poses[i + 1][0] if i < len(poses) - 1 else None
        pc = nc = None
        if prev is not None:
            d = math.dist(p, prev) * AUTO_CONTROL_DISTANCE_FACTOR
            pc = (p[0] - d * math.cos(h), p[1] - d * math.sin(h))
        if nxt is not None:
            d = math.dist(p, nxt) * AUTO_CONTROL_DISTANCE_FACTOR
            nc = (p[0] + d * math.cos(h), p[1] + d * math.sin(h))
        out.append({"prev": pc, "anchor": p, "next": nc})
    return out


# ============================================================================================ follower
class PathfindingCommand(Follower):
    """PathfindingCommand with PPHolonomicDriveController (kP 5 / 5). hold=True adds the race's final hold."""

    def __init__(self, kp=5.0, kp_rot=5.0, hold=False, limits: Limits = Limits()):
        self.kp, self.kp_rot, self.hold = kp, kp_rot, hold
        self.lim = limits
        self.cfg = RobotConfig()

    def reset(self, native, start):
        self.paths = list(native["paths"])  # anytime paths, handed over one per loop
        self.goal = native["goal"]
        self.traj = None
        self.offset = 0.0
        self.t_timer = start.t
        self.finished = math.dist(start.pose[:2], self.goal[:2]) < 0.5
        self.holding = False

    def step(self, t, pose, vel):
        if self.finished:
            return (0.0, 0.0, 0.0)
        skip = self.traj is not None and math.dist(pose[:2], self.traj[-1].pose[:2]) < 2.0
        if not skip and self.paths:
            wps = self.paths.pop(0)
            path = PathPlannerPath({"waypoints": wps, "rotation_targets": [], "goal_rotation": self.goal[2]})
            traj = generate_trajectory(path, vel, pose[2], self.cfg, self.lim)
            if not math.isfinite(traj[-1].t):
                self.finished = True
                return (0.0, 0.0, 0.0)
            i1, i2 = 0, 1
            while i2 < len(traj) - 1:
                if math.dist(traj[i2 + 1].pose[:2], pose[:2]) < math.dist(traj[i2].pose[:2], pose[:2]):
                    i1, i2 = i1 + 1, i2 + 1
                else:
                    break
            d = math.dist(traj[i1].pose[:2], traj[i2].pose[:2])
            u = max(0.0, min(1.0, math.dist(pose[:2], traj[i1].pose[:2]) / d)) if d > 0 else 0.0
            self.offset = traj[i1].t + (traj[i2].t - traj[i1].t) * u
            if self.offset <= 0.02 and math.hypot(vel[0], vel[1]) < 0.1:
                self.offset = 0.02
            self.traj = traj
            self.t_timer = t
        if self.traj is None:
            return (0.0, 0.0, 0.0)
        now = t - self.t_timer
        if now >= self.traj[-1].t - self.offset or self.holding:  # isFinished
            if not self.hold:
                self.finished = True
                return (0.0, 0.0, 0.0)
            self.holding = True
            g = self.traj[-1].pose
            ex, ey, eth = g[0] - pose[0], g[1] - pose[1], wrap(g[2] - pose[2])
            if math.hypot(ex, ey) < 0.02 and abs(eth) < math.radians(2) and math.hypot(vel[0], vel[1]) < 0.05:
                self.finished = True
                return (0.0, 0.0, 0.0)
            return (self.kp * ex, self.kp * ey, self.kp_rot * eth)
        tp, ff = sample(self.traj, now + self.offset)
        return (ff[0] + self.kp * (tp[0] - pose[0]), ff[1] + self.kp * (tp[1] - pose[1]),
                ff[2] + self.kp_rot * wrap(tp[2] - pose[2]))

    def done(self):
        return self.finished


ROUTES_FILE = os.path.join(HERE, "fields", "routes_v1.json")


def load_routes():
    """The start/goal pairs on the field: name -> (start pose, goal pose, start velocity), radians."""
    with open(ROUTES_FILE) as fh:
        data = json.load(fh)
    out = {}
    for r in data["routes"]:
        s, g = r["start"], r["goal"]
        out[r["name"]] = ((s[0], s[1], math.radians(s[2])), (g[0], g[1], math.radians(g[2])),
                          tuple(r["start_velocity"]))
    return out


_warmed_pid = None
_warmed_server = None


def _warm_mark():
    from followers.ours_v2 import server

    global _warmed_pid, _warmed_server
    _warmed_pid, _warmed_server = os.getpid(), server()


class OursPathfind(Follower):
    """Our pathfinder (lib/, frc.team5712.path.plan) through BenchServer's `pathfind` op, then our follower
    as shipped (the same options and planning grip as the ours_v2 row). The plan is made in reset(), from
    the measured start state, as a command would at its start; the reply's Java timings and the Python
    wall-clock of the whole round trip are kept. Each BenchServer plans once before its first timed query,
    as robot init would (see lib/README.md); cold timings come from PlanCli in a fresh JVM."""

    def __init__(self, mu=0.7, margin=0.05, limits: Limits = Limits(), **follower_options):
        self.mu = mu
        self.margin = margin  # bumper margin the planner grows obstacles by and checks the sweep against
        self.lim = limits
        self.options = {"maxOmegaRadPerSec": limits.max_omega, "maxAlphaRadPerSecSquared": limits.max_alpha,
                        "brakeMetersPerSecondSquared": limits.max_accel}
        self.options.update(follower_options)

    def reset(self, native, start):
        from followers.ours_v2 import call_fresh, robot_json, server

        def plan(goal):
            return call_fresh(op="pathfind", field=native["field"], start=[float(v) for v in start.pose],
                              vel=[float(v) for v in start.vel], goal=[float(v) for v in goal],
                              robot=robot_json(self.mu), options=self.options, margin=self.margin,
                              maxSpeed=self.lim.max_speed, maxAccel=self.lim.max_accel)

        if _warmed_pid != os.getpid() or server() is not _warmed_server:
            # robot init: build the pathfinder and plan once so the JVM has loaded and compiled the code;
            # the cold numbers come from PlanCli in a fresh JVM instead
            plan(native["goal"])
            _warm_mark()
        t0 = time.perf_counter()
        r = plan(native["goal"])
        self.bridge_ms = 1000 * (time.perf_counter() - t0)
        self.plan = r
        self.srv = server()
        g = self.srv.call(op="geometry", step=0.05)
        self.geometry = [[round(x, 3), round(y, 3)] for x, y in zip(g["x"], g["y"])]
        self.srv.call(op="reset", t=start.t, pose=[float(v) for v in start.pose], vel=[float(v) for v in start.vel])
        self.finished = False

    def step(self, t, pose, vel):
        r = self.srv.call(op="step", t=float(t), pose=[float(v) for v in pose], vel=[float(v) for v in vel])
        self.finished = r["done"]
        return tuple(r["cmd"])

    def done(self):
        return self.finished


PATHFIND_METHODS = {
    "pp_pathfind": (PathfindingCommand, {}, "PathPlanner pathfinding (AD*, kP 5)"),
    "pp_pathfind_hold": (PathfindingCommand, {"hold": True}, "PathPlanner pathfinding (AD*, kP 5) + hold"),
    "ours_pathfind": (OursPathfind, {}, "Ours (visibility graph + A* + quintic, our follower)"),
    "ours_pathfind_m10": (OursPathfind, {"margin": 0.10}, "Ours, bumper margin 0.10 m"),
    "ours_pathfind_m15": (OursPathfind, {"margin": 0.15}, "Ours, bumper margin 0.15 m"),
}


def plan_once(field=None, route=None):
    """PathPlanner's planner for one route: (planner, anytime steps, grid build ms)."""
    field = field or load_field()
    start, goal, _ = route or load_routes()["across"]
    t0 = time.perf_counter()
    planner = LocalADStar(field)
    grid_ms = 1000 * (time.perf_counter() - t0)
    steps = planner.plan(start[:2], goal[:2])
    return planner, steps, grid_ms


def run_pathfind(job):
    """job = (method key, route name, seed, pose source). Plans, follows, and measures against the true pose."""
    from dataclasses import replace

    from sim import DisturbedPlant, end_metrics, simulate

    key, rname, seed, pose_src = job
    field = load_field()
    start, goal, v0 = load_routes()[rname]
    cls, kw, _ = PATHFIND_METHODS[key]
    rng = np.random.default_rng(seed)
    rp = replace(RP, wheel_radius_error=rng.normal(0, 0.01), gyro_scale_error=rng.normal(0, 0.002))
    follower = cls(**kw)
    extra = {}
    if cls is PathfindingCommand:
        _, steps, grid_ms = plan_once(field, (start, goal, v0))
        native = {"paths": [s[1] for s in steps], "goal": goal}
        final = PathPlannerPath({"waypoints": steps[-1][1], "rotation_targets": [], "goal_rotation": goal[2]})
        pts = np.array([p.position for p in final.points])
        extra.update(path_m=float(np.hypot(*np.diff(pts, axis=0).T).sum()), plan_first_ms=steps[0][2],
                     plan_ms=sum(s[2] for s in steps), grid_ms=grid_ms)
    else:
        native = {"field": field, "goal": goal}
    plant = DisturbedPlant(rp, start, v0, rng)
    r = simulate(follower, native, start, v0, rp, rng, 12.0, pose_src, plant)
    if cls is OursPathfind:
        p = follower.plan
        extra.update(path_m=p["lengthMeters"], route_m=p["routeLengthMeters"], plan_ms=p["planMs"],
                     bridge_ms=follower.bridge_ms, refits=p["refits"], plan_clear=float(p["clear"]),
                     plan_clearance_cm=100 * p["minClearanceMeters"])
    dist, eth, t_5cm, t_settle = end_metrics(r, goal)
    clear, hits = swept_clearance(r["x"], r["y"], r["th"], field)
    driven = np.hypot(np.diff(r["x"]), np.diff(r["y"]))
    stop = int(np.flatnonzero(dist < 0.05)[0]) if (dist < 0.05).any() else len(driven)
    out = {
        "straight_m": math.dist(start[:2], goal[:2]),
        "driven_m": float(driven[:stop].sum()),
        "t_5cm": t_5cm,
        "t_settle": t_settle,
        "t_done": float("nan") if r["t_done"] is None else r["t_done"],
        "err_end_cm": 100 * float(dist[-1]),
        "yaw_end_deg": math.degrees(float(eth[-1])),
        "clearance_cm": 100 * clear,
        "contact_ms": 1000 * hits * LP.odom_period,
        "contact_run": float(hits > 0),
        "wheel_slip_ms": 1000 * r["wheel_slip"],
        "odo_err_cm": r["odo_err_cm"],
    }
    out.update(extra)
    return out


def planned_geometry():
    """For results.json: PathPlanner's anytime paths per route, ours per route, and the blocked grid."""
    from followers.base import Start

    field = load_field()
    out = {"routes": {}}
    for name, route in load_routes().items():
        planner, steps, _ = plan_once(field, route)
        if "blocked" not in out:
            ox, oy = planner.off
            out["blocked"] = [[round(x - ox, 2), round(y - oy, 2)]
                              for x, y in sorted(planner.grid_to_xy(g) for g in planner.obstacles)]
        eps = []
        for e, wps, ms in steps:
            path = PathPlannerPath({"waypoints": wps, "rotation_targets": [], "goal_rotation": 0.0})
            eps.append({"eps": e, "ms": ms, "points": [[round(p.position[0], 3), round(p.position[1], 3)]
                                                       for p in path.points]})
        ours = OursPathfind()
        start, goal, v0 = route
        ours.reset({"field": field, "goal": goal}, Start(0.0, start, v0))
        out["routes"][name] = {"pathplanner": eps, "ours": ours.geometry,
                               "ours_waypoints": ours.plan["waypoints"]}
    return out


def java_plan_timing(route):
    """Cold (first query in a fresh JVM) and warm Java timings for one route, from PlanCli."""
    import subprocess

    from followers.ours_v2 import JAR

    start, goal, v0 = route
    java = os.path.join(os.environ["JAVA_HOME"], "bin", "java") if os.environ.get("JAVA_HOME") else "java"
    args = [java, "-cp", JAR, "frc.team5712.path.tools.PlanCli", FIELD_FILE,
            *[repr(v) for v in (start[0], start[1], math.degrees(start[2]), goal[0], goal[1], math.degrees(goal[2]))]]
    if math.hypot(v0[0], v0[1]) > 0:
        args += [repr(v0[0]), repr(v0[1])]
    out = subprocess.run(args + ["--repeat", "2000"], capture_output=True, text=True, check=True).stdout
    return json.loads(out)
