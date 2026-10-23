"""What the harder courses (courses_extra.py) need beyond a path: a chain of paths, an end that moves when a
sensor sees it, a heavier robot, walls to measure against. sim.run_one asks `make_follower` for the follower
and `course_metrics` for the extra numbers; every other course passes straight through.

How each method meets a moving end (a tag lock 1.5 m out, or a game piece seen 2 m out). The new end is
latched in the pose source's frame, as a tag or piece seen from the robot would be: the true offset from the
TRUE robot to the true end, applied to the MEASURED pose at that moment.
- Ours v2: its moving goal (`goal` op = `bindGoal`): the tail of the path is rebuilt to the new end.
- PathPlanner: an on-the-fly path, as its docs describe ("Create a Path On-the-fly"): two poses, the robot's
  (rotation = its direction of travel) and the new end (rotation = the original approach direction), made
  into a Bezier path with `PathPlannerPath.waypointsFromPoses` (control points a third of the way to the
  neighbour), a GoalEndState with the end's rotation and speed, and followed from the current speeds (no
  ideal starting state) by the same controller, so the hold variants hold on the new end. Pathfinding (AD*)
  is for getting round obstacles; its docs point to a path for the final line-up, so it isn't used here.
- BLine: for a tag, its path finishes on the map pose and a DriveToPose command then drives to the new goal
  (the handoff the bench gives every method without a moving end). For a piece, a new two-point path from
  the robot to the piece built in code (BLine paths can be), with the rolling end's minimum velocity.
- PathPlanner path2 (pre-release): no in-path way to move the end and no rolling end to a path, so it is
  treated like Choreo below.
- Choreo + ChoreoLib and the old FollowPath: trajectories and paths are fixed when made (Choreo solves
  offline), so a tag gets the DriveToPose handoff after the path and a piece gets nothing: they drive
  through where the map put it.
- DriveToPose chain and the Choreo hybrid: their last goal (the hybrid's DriveToPose finish) is retargeted.

Chains of paths (three_piece_chain): every method that follows a path per command runs one per leg
(followers.chain.Chain) with rolling handoffs where it supports them. Choreo plans the whole route as one
trajectory (how a ChoreoLib auto splits one trajectory), and the DriveToPose chain takes every waypoint.
"""

import math
from dataclasses import replace

import numpy as np

from common import Limits
from courses import COURSES, NeutralPath
from courses_extra import EXTRAS
from followers import build
from followers.base import Follower, Start

BUMPER_HALF = 0.425  # m: the 0.85 m square robot with bumpers
# one plan for a whole chain of paths (path2 makes one path of the legs: its handoffs inside a path roll)
WHOLE_ROUTE = ("ChoreoLib", "ChoreoHybrid", "DriveToPoseChain", "PathPlanner2")
KEEPS_GOING = 1.0  # m/s: at least this at the piece counts as driving through it


def extras(cname):
    return EXTRAS.get(cname, {})


def plant_params(cname, rp):
    r = extras(cname).get("robot")
    if r:
        rp = replace(rp, mass=r["mass"], inertia=r["inertia"], stator_limit=r["stator_limit"])
    return rp


def _give_robot(f, r):
    """Methods with a robot model plan with the course's robot; the rest have no such setting."""
    from followers.ours_v2 import OursV2

    name = type(f).__name__
    if name == "PathPlanner":
        from followers.pathplanner import RobotConfig

        f.cfg = RobotConfig(mass=r["mass"], moi=r["inertia"], current_limit=r["stator_limit"],
                            wheel_cof=f.cfg.wheel_cof)
    elif name == "PathPlanner2":
        from followers.pathplanner import RobotConfig
        from followers.pathplanner2 import SetpointGenerator

        f.cfg = RobotConfig(mass=r["mass"], moi=r["inertia"], current_limit=r["stator_limit"],
                            wheel_cof=f.cfg.wheel_cof)
        f.gen = SetpointGenerator(f.cfg, f.gen.steer_rate, f.gen.constraints)
    elif isinstance(f, OursV2):  # and its subclasses (optimised, on Choreo's trajectory)
        f.robot_overrides = dict(f.robot_overrides, massKg=r["mass"], momentOfInertiaKgM2=r["inertia"],
                                 statorCurrentLimitAmps=r["stator_limit"])


def make_follower(cname, key, opts, seed=0):
    """-> (follower, native path) for registry key `key` with options `opts` on course `cname`."""
    course = COURSES[cname]
    ex = extras(cname)
    if course.limits != Limits():  # the course's own limits reach the follower as well as its path
        opts = dict(opts or {}, limits=course.limits)
    cls, kw = build(key, opts)
    if ex.get("legs") and cls.__name__ not in WHOLE_ROUTE:
        from followers.chain import Chain

        f = Chain(key, **(opts or {}))
        inner = f.f
        f.wants_sensing = getattr(inner, "wants_sensing", False)
        native = f.adapt(ex["legs"])
    else:
        f = inner = cls(**kw)
        native = f.adapt(course)
    if ex.get("robot"):
        _give_robot(inner, ex["robot"])
    if ex.get("goal_shift") or ex.get("piece"):
        f = MovingEnd(f, course, ex, seed)
    return f, native


def _compose(a, b):
    """Pose a then the relative pose b."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def _relative(a, b):
    """b seen from a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (c * dx + s * dy, -s * dx + c * dy, b[2] - a[2])


def pp_on_the_fly(pose, vel, goal, approach, end_speed):
    """PathPlanner's on-the-fly path from the robot to `goal` (see the module docstring)."""
    p0, p1 = np.array(pose[:2], float), np.array(goal[:2], float)
    d = float(np.linalg.norm(p1 - p0))
    sp = math.hypot(vel[0], vel[1])
    h0 = math.atan2(vel[1], vel[0]) if sp > 0.1 else math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    nxt = p0 + d / 3.0 * np.array((math.cos(h0), math.sin(h0)))
    prv = p1 - d / 3.0 * np.array((math.cos(approach), math.sin(approach)))
    return {"waypoints": [{"prev": None, "anchor": tuple(p0), "next": tuple(nxt)},
                          {"prev": tuple(prv), "anchor": tuple(p1), "next": None}],
            "rotation_targets": [], "goal_rotation": goal[2], "ideal_start_rotation": pose[2],
            "goal_velocity": end_speed, "ideal_start_velocity": sp}


def bline_two_point(f, pose, goal, end_speed):
    """A BLine path built in code: from the robot to `goal`, rolling at `end_speed`."""
    from courses import BLINE_RADIUS

    r = f.radius if f.radius is not None else BLINE_RADIUS
    end = {"kind": "T", "xy": tuple(goal[:2]), "radius": r}
    if end_speed > 0:
        end["vmin"] = end_speed
    return {"elements": [{"kind": "T", "xy": tuple(pose[:2]), "radius": r},
                         {"kind": "R", "rot": pose[2], "t_ratio": 0.0, "profiled": True},
                         {"kind": "R", "rot": goal[2], "t_ratio": 1.0, "profiled": True}, end], "radius": r}


class MovingEnd(Follower):
    """The end of the course moves when the robot gets within `reveal` m of the map's end (measured pose):
    a tag lock (goal_shift: the true goal is off the map pose in the goal's own frame) or a game piece
    (piece: the true piece is off the map's sideways, left on even seeds, right on odd ones)."""

    needs_plant = True

    def __init__(self, f, course, ex, seed):
        self.f = f
        self.inner = getattr(f, "f", f)
        self.end_output = f.end_output
        self.wants_sensing = getattr(self.inner, "wants_sensing", False)
        npath = NeutralPath(course)
        tx, ty = npath.tangent_at_s(npath.length)
        self.approach = math.atan2(ty, tx)
        self.map_end = course.end_pose
        self.end_speed = course.end_speed
        if ex.get("goal_shift"):
            self.reveal, dx, dy, dth = ex["goal_shift"]
            self.true_end = _compose(self.map_end, (dx, dy, math.radians(dth)))
            self.piece = False
        else:
            self.reveal, off = ex["piece"]
            off = off if seed % 2 == 0 else -off
            e = self.map_end
            self.true_end = (e[0] - ty * off, e[1] + tx * off, e[2])
            self.piece = True
        name = type(self.inner).__name__
        if name == "OursV2":
            self.mode = "goal"
        elif name == "PathPlanner" or (name == "BLine" and self.piece):
            self.mode = "replan"
        elif name in ("DriveToPoseChain", "ChoreoHybrid"):
            self.mode = "retarget"
        else:
            self.mode = "none" if self.piece else "handoff"
        self.plant = None

    def geometry(self, native):
        return self.f.geometry(native)

    def adapt(self, course):
        return self.f.adapt(course)

    def reset(self, native, start):
        self.f.reset(native, start)
        self.goal = None
        self.t_reveal = float("nan")
        self.dtp = None
        self.build_ms = getattr(self.f, "build_ms", None)

    def _reveal(self, t, pose, vel):
        p = self.plant
        rel = _relative((p.x, p.y, p.th_unwrapped), self.true_end)
        g = _compose(pose, rel)
        if self.piece:
            g = (g[0], g[1], self.map_end[2])  # a piece has no heading: the robot keeps the planned one
        self.goal = g
        self.t_reveal = t
        inner = self.inner
        if self.mode == "goal":
            inner.srv.call(op="goal", pose=[float(v) for v in g])
        elif self.mode == "replan":
            start = Start(t, pose, vel)
            if type(inner).__name__ == "PathPlanner":
                inner.reset(pp_on_the_fly(pose, vel, g, self.approach, self.end_speed), start)
            else:
                inner.reset(bline_two_point(inner, pose, g, self.end_speed), start)
        elif self.mode == "retarget":
            if type(inner).__name__ == "DriveToPoseChain":
                inner.goals[-1] = g
            else:
                inner.goal = g
                inner.dtp.goals = [g]

    def step(self, t, pose, vel):
        if self.goal is None and math.hypot(pose[0] - self.map_end[0], pose[1] - self.map_end[1]) < self.reveal:
            self._reveal(t, pose, vel)
        if self.dtp is not None:
            return self.dtp.step(t, pose, vel)
        cmd = self.f.step(t, pose, vel)
        if self.f.done() and self.mode == "handoff" and self.goal is not None:
            from followers.drive_to_pose_chain import DriveToPoseChain

            self.dtp = DriveToPoseChain(limits=getattr(self.inner, "lim", Limits()))
            self.dtp.reset({"goals": [self.goal]}, Start(t, pose, vel))
            self.end_output = self.dtp.end_output
            return self.dtp.step(t, pose, vel)
        return cmd

    def done(self):
        return self.dtp.done() if self.dtp is not None else self.f.done()


def course_metrics(cname, r, follower):
    """Extra numbers for the harder courses, from the run's true traces."""
    ex = extras(cname)
    out = {}
    xs, ys, ths, sp = r["x"], r["y"], r["th"], r["speed"]
    if ex.get("walls"):
        reach = BUMPER_HALF * (np.abs(np.cos(ths)) + np.abs(np.sin(ths)))  # half the bumper square's extent
        clear = np.full(len(xs), np.inf)
        for axis, at in ex["walls"]:
            clear = np.minimum(clear, at - ((ys if axis == "y" else xs) + reach))
        out["wall_clear_cm"] = 100 * float(clear.min())
    if isinstance(follower, MovingEnd):
        out["t_reveal"] = follower.t_reveal
        if follower.piece:
            e = follower.true_end
            d = np.hypot(xs - e[0], ys - e[1])
            i = int(np.argmin(d))
            out["miss_cm"] = 100 * float(d[i])
            out["piece_speed"] = float(sp[i])
            out["keeps_going"] = 1.0 if sp[i] >= KEEPS_GOING else 0.0
    return out
