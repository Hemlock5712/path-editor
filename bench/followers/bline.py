"""Port of the BLine-Lib follower (edanliahovetsky/BLine-Lib, branch wpilib-2027, commit 4402250):
following/Follower.java (execute, isFinished, end), TranslationProgress, RotationProgress,
TranslationGuidance, HolonomicController, HolonomicRateLimiter; path/PreparedPath + Path's
waypoint expansion (`getPathElementsWithConstraintsNoWaypoints`).

Swerve only. Not ported: events, rotation override, flipping/mirroring, tank drive, rolling ends
(every course ends at rest, so min velocity is 0), per-element constraint ranges (every element gets
the same shared limits), BLine-Web's automatic velocity constraints and auto handoff radii.
PID controllers are P-only (as in the README), so WPILib PIDController reduces to kP * error, with
continuous input on the heading controller.
"""

import math

import numpy as np

from common import Limits, clamp, wrap
from followers.base import Follower

EPS = 1e-9
SEG_EPS = 1e-6
PROJ_EPS = 1e-6


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _project(start, end, p):
    dx, dy = end[0] - start[0], end[1] - start[1]
    l2 = dx * dx + dy * dy
    if l2 < PROJ_EPS:
        return 0.0
    return clamp(((p[0] - start[0]) * dx + (p[1] - start[1]) * dy) / l2, 0.0, 1.0)


class TranslationProgress:
    def __init__(self, els, start_xy):
        self.els = els
        self.origin = start_xy
        self.index = self._next(0)

    def _next(self, start):
        for i in range(max(0, start), len(self.els)):
            if self.els[i]["kind"] == "T":
                return i
        return -1

    def _prev(self, start):
        for i in range(start, -1, -1):
            if self.els[i]["kind"] == "T":
                return i
        return -1

    def _xy(self, i):
        return self.origin if i < 0 else self.els[i]["xy"]

    def is_last(self):
        return self._next(self.index + 1) < 0

    def target(self):
        return self._xy(self.index)

    def segment(self, p):
        a, b = self._xy(self._prev(self.index - 1)), self.target()
        L = _dist(a, b)
        return L, (1.0 if L < SEG_EPS else _project(a, b, p))

    def advance(self, p):
        while not self.is_last():
            tgt = self.els[self.index]
            L, prog = self.segment(p)
            d = _dist(p, tgt["xy"])
            r = tgt["radius"]
            # HandoffMode.RADIUS (BLine's default mode); PROGRESS mode is not used by the bench
            reached = L < SEG_EPS or d <= r
            if not reached:
                return
            self.index = self._next(self.index + 1)

    def remaining(self, p):
        prev, rem = p, 0.0
        i = self.index
        while i >= 0:
            t = self._xy(i)
            rem += _dist(prev, t)
            prev = t
            i = self._next(i + 1)
        return rem

    def cross_track(self, p):
        a = self._xy(self._prev(self.index - 1))
        b = self.target()
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        if L < SEG_EPS:
            return 0.0, (0.0, 0.0)
        n = (-dy / L, dx / L)
        err = (p[0] - a[0]) * n[0] + (p[1] - a[1]) * n[1]
        return err, n


class _Seg:
    __slots__ = ("target_index", "start", "end", "start_s", "length")

    def __init__(self, ti, a, b, s0, L):
        self.target_index, self.start, self.end, self.start_s, self.length = ti, a, b, s0, L

    def ratio(self, p):
        if self.length < EPS:
            return 1.0
        dx, dy = self.end[0] - self.start[0], self.end[1] - self.start[1]
        return clamp(((p[0] - self.start[0]) * dx + (p[1] - self.start[1]) * dy) / (self.length ** 2), 0.0, 1.0)

    def project(self, p):
        return self.start_s + self.length * self.ratio(p)

    def distance(self, p):
        r = self.ratio(p)
        q = (self.start[0] + r * (self.end[0] - self.start[0]), self.start[1] + r * (self.end[1] - self.start[1]))
        return _dist(q, p)

    def along(self, m):
        return m[0] * (self.end[0] - self.start[0]) + m[1] * (self.end[1] - self.start[1])


class RotationProgress:
    def __init__(self, els, start_pose):
        self.initial = start_pose[2]
        self.prev_pos = start_pose[:2]
        self.segs = []
        prev, s = start_pose[:2], 0.0
        for i, e in enumerate(els):
            if e["kind"] == "T":
                L = _dist(prev, e["xy"])
                self.segs.append(_Seg(i, prev, e["xy"], s, L))
                s += L
                prev = e["xy"]
        frames = []
        for i, e in enumerate(els):
            if e["kind"] != "R":
                continue
            owning = next((sg for sg in self.segs if sg.target_index > i), None)
            if owning is not None:
                frames.append([i, owning.start_s + clamp(e["t_ratio"], 0, 1) * owning.length, e["rot"], e["profiled"]])
        frames.sort(key=lambda f: f[1])  # stable: equal s keeps path order
        un = self.initial
        for f in frames:
            un += wrap(f[2] - un)
            f[2] = un
        self.frames = frames
        self.seg_index = 0
        self.accepted_s = 0.0
        self.join = None
        self.tail = None

    def final_heading(self):
        return self.frames[-1][2] if self.frames else self.initial

    def _next_frame(self, s):
        for f in self.frames:
            if f[1] > s + EPS:
                return f
        return None

    def _prev_frame(self, s):
        prev = None
        for f in self.frames:
            if f[1] > s + EPS:
                break
            prev = f
        return prev

    def heading(self, s):
        nxt = self._next_frame(s)
        if nxt is not None and not nxt[3]:
            return nxt[2]
        j = self.join
        if j is not None and j[0] <= s < j[2]:
            return j[1] + (s - j[0]) / (j[2] - j[0]) * (j[3] - j[1])
        if nxt is None:
            return self.final_heading() if self.tail is None else self.tail
        prev = self._prev_frame(s)
        s0 = 0.0 if prev is None else prev[1]
        h0 = self.initial if prev is None else prev[2]
        return h0 + clamp((s - s0) / (nxt[1] - s0), 0.0, 1.0) * (nxt[2] - h0)

    def update(self, p, authorized):
        segs = self.segs
        if not segs:
            return self.initial
        while (self.seg_index + 1 < len(segs) and segs[self.seg_index].length < EPS
               and segs[self.seg_index + 1].target_index <= authorized):
            self.seg_index += 1
        old = segs[self.seg_index]
        old_s = max(self.accepted_s, old.project(p))
        old_heading = self.heading(old_s)
        mv = (p[0] - self.prev_pos[0], p[1] - self.prev_pos[1])
        switched = False
        while self.seg_index + 1 < len(segs):
            cur, nxt = segs[self.seg_index], segs[self.seg_index + 1]
            if nxt.target_index > authorized:
                break
            retracing = (abs(cur.distance(p) - nxt.distance(p)) <= EPS and cur.along(mv) < -EPS
                         and nxt.along(mv) > EPS)
            if cur.length < EPS or cur.ratio(p) >= 1 - EPS or nxt.distance(p) + EPS < cur.distance(p) or retracing:
                self.seg_index += 1
                switched = True
            else:
                break
        self.accepted_s = max(old_s, segs[self.seg_index].project(p))
        if switched and self.accepted_s > old_s + EPS:
            fut = self._next_frame(self.accepted_s)
            if fut is not None and fut[3] and abs(self.heading(self.accepted_s) - old_heading) > EPS:
                self.join = (self.accepted_s, old_heading, fut[1], fut[2])
                self.tail = None
            elif fut is None and abs(self.heading(self.accepted_s) - old_heading) > EPS:
                sg = segs[self.seg_index]
                end_s = sg.start_s + sg.length
                if end_s > self.accepted_s + EPS:
                    self.join = (self.accepted_s, old_heading, end_s, self.final_heading())
                    self.tail = None
                else:
                    self.tail = old_heading
        self.prev_pos = p
        return self.heading(self.accepted_s)


def rate_limit(want, last, dt, amax, alpha_max, vmax, wmax):
    """HolonomicRateLimiter.limit."""
    vx, vy, om = want
    if vmax > 0 and wmax > 0:
        sp = math.hypot(vx, vy)
        if sp > vmax:
            vx, vy = vx * vmax / sp, vy * vmax / sp
        om = clamp(om, -wmax, wmax)
    if dt <= 0:
        return (vx, vy, om)
    acc = clamp(math.hypot(vx - last[0], vy - last[1]) / dt, 0.0, amax)
    th = math.atan2(vy - last[1], vx - last[0])
    oa = clamp((om - last[2]) / dt, -alpha_max, alpha_max)
    return (last[0] + math.cos(th) * acc * dt, last[1] + math.sin(th) * acc * dt, last[2] + oa * dt)


class BLine(Follower):
    """README gains by default: distance P 5, heading P 3, cross-track P 2; end tolerances 3 cm / 2 deg
    (BLine-Web project defaults)."""

    def __init__(self, kp=5.0, kp_rot=3.0, kp_ct=2.0, tol=0.03, tol_deg=2.0, radius=None, curve=False,
                 limits: Limits = Limits()):
        self.kp, self.kp_rot, self.kp_ct = kp, kp_rot, kp_ct
        self.curve = curve
        self.tol, self.tol_rad = tol, math.radians(tol_deg)
        self.radius = radius
        self.lim = limits

    def adapt(self, course):
        """curve=False: a polyline through the course waypoints; curve=True: BLine-Web's curve-to-targets."""
        from courses import to_bline, to_bline_curve

        return to_bline_curve(course) if self.curve else to_bline(course)

    @staticmethod
    def geometry(native):
        return np.array([e["xy"] for e in native["elements"] if e["kind"] == "T"], float)

    def reset(self, native, start):
        els = native["elements"]
        if self.radius is not None:
            els = [dict(e, radius=self.radius) if e["kind"] == "T" else e for e in els]
        self.els = els
        pose = start.pose
        self.last_t = start.t
        # HolonomicController(initialMeasured robot-relative, heading) -> field-relative command
        self.cmd = tuple(start.vel)
        self.tp = TranslationProgress(els, pose[:2])
        self.rp = RotationProgress(els, pose)
        self.finished = False

    def step(self, t, pose, vel):
        if self.finished:
            return (0.0, 0.0, 0.0)
        dt = t - self.last_t
        self.last_t = t
        p = pose[:2]
        tp = self.tp
        tp.advance(p)
        heading_sample = self.rp.update(p, tp.index)
        target = tp.target()
        remaining = tp.remaining(p)
        final_reached = tp.is_last() and remaining <= self.tol
        lim = self.lim
        # TranslationGuidance.calculate
        bearing = math.atan2(target[1] - p[1], target[0] - p[0])
        raw = self.kp * remaining
        speed = clamp(raw, -lim.max_speed, lim.max_speed)  # min velocity 0: minimumMagnitude is a no-op
        err, n = tp.cross_track(p)
        corr = self.kp_ct * err if (n[0] or n[1]) else 0.0
        vx = speed * math.cos(bearing) - corr * n[0]
        vy = speed * math.sin(bearing) - corr * n[1]
        if final_reached:
            vx = vy = 0.0
        target_rot = wrap(self.rp.final_heading() if final_reached else heading_sample)
        rot_err = wrap(target_rot - pose[2])
        omega = clamp(self.kp_rot * rot_err, -lim.max_omega, lim.max_omega)
        at_heading = abs(rot_err) <= self.tol_rad
        if final_reached and at_heading:
            omega = 0.0
        # HolonomicController.limit
        out = rate_limit((vx, vy, omega), self.cmd, dt, lim.max_accel, lim.max_alpha, lim.max_speed, lim.max_omega)
        if final_reached:
            out = (0.0, 0.0, 0.0 if at_heading else out[2])
        self.cmd = out
        # isFinished: at position (last target within tolerance) and at the final heading
        at_pos = tp.is_last() and tp.remaining(p) <= self.tol
        at_rot = abs(wrap(self.rp.final_heading() - pose[2])) <= self.tol_rad
        if at_pos and at_rot:
            self.finished = True  # end(): stopCommandedMotion -> zero speeds
        return out

    def done(self):
        return self.finished

