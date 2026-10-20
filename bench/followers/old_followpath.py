"""Port of the old path-editor follower (Hemlock5712/path-editor, master, 29d7ada):
public/java/commands/FollowPath.java + utils/path/SplinePath.java, CubicSegment.java,
VelocityProfile.java, VelocityConstraints.java, RotationSupplier.java.

Where the shipped code does not compile or calls code that isn't in the repo:
- FollowPath calls `rotationSupplier.getOmega(...)`, but RotationSupplier defines `getTargetHeading(...)`.
  Here the supplier returns a target heading (RotationSupplier.interpolateAlongPath over the course's
  heading targets) and a P controller turns it into omega: KP_HEADING * wrap(target - heading),
  clamped to the shared rotation limit. KP_HEADING = 5 (PathPlanner's documented rotation kP); the
  repo has no heading gain.
- `AccelerationLimiter.integrateVelocity` -> the reference race's AccelerationLimiter.limit (MU 1.0, as
  shipped); `MAX_FRICTION_ACCEL` = MU * g = 9.81; `DRIVE_BASE_RADIUS` = the sim's module radius.
  The real AccelerationLimiter was removed from that repo (kept private), so it is not ported.
- `AccelerationLimiter.normalizeSpeeds` (also not in the repo) -> wheel-speed desaturation: if the fastest
  module would exceed the shared max speed (4.0 m/s), scale vx, vy and omega together.
- `Motor.getMaxAcceleration` (not in the repo) -> the limiter's Kraken X60 FOC model at 150 A.
- VelocityProfile's curvature-window maximum is computed with scipy's maximum_filter1d (identical
  result to the Java double loop, much faster in Python).
"""

import bisect
import math

import numpy as np
from scipy.ndimage import maximum_filter1d

from common import BASE_R, RP, Limits, clamp, rot, wrap
from followers.base import AccelerationLimiter, Follower

GRAVITY = 9.81
KP_HEADING = 5.0

GL_NODES = (-0.9061798459386640, -0.5384693101056831, 0.0, 0.5384693101056831, 0.9061798459386640)
GL_WEIGHTS = (0.2369268850561891, 0.4786286704993665, 0.5688888888888889, 0.4786286704993665,
              0.2369268850561891)


class CubicSegment:
    def __init__(self, p0, p1, m0, m1):
        self.ax = 2 * p0[0] - 2 * p1[0] + m0[0] + m1[0]
        self.bx = -3 * p0[0] + 3 * p1[0] - 2 * m0[0] - m1[0]
        self.cx, self.dx = m0[0], p0[0]
        self.ay = 2 * p0[1] - 2 * p1[1] + m0[1] + m1[1]
        self.by = -3 * p0[1] + 3 * p1[1] - 2 * m0[1] - m1[1]
        self.cy, self.dy = m0[1], p0[1]

    def point(self, t):
        t2, t3 = t * t, t * t * t
        return (self.ax * t3 + self.bx * t2 + self.cx * t + self.dx, self.ay * t3 + self.by * t2 + self.cy * t + self.dy)

    def deriv(self, t):
        t2 = t * t
        return (3 * self.ax * t2 + 2 * self.bx * t + self.cx, 3 * self.ay * t2 + 2 * self.by * t + self.cy)

    def deriv2(self, t):
        return (6 * self.ax * t + 2 * self.bx, 6 * self.ay * t + 2 * self.by)

    def curvature(self, t):
        d1, d2 = self.deriv(t), self.deriv2(t)
        cross = d1[0] * d2[1] - d1[1] * d2[0]
        sq = d1[0] * d1[0] + d1[1] * d1[1]
        cubed = sq * math.sqrt(sq)
        return 0.0 if cubed < 1e-12 else cross / cubed

    def speed(self, t):
        d = self.deriv(t)
        return math.hypot(d[0], d[1])


class SplinePath:
    SAMPLES_PER_METER = 1000
    MIN_SAMPLES = 100
    COARSE_SEARCH_STEP = 0.05
    MAX_NEWTON_ITERATIONS = 8

    def __init__(self, pts):
        n = len(pts)
        self.segments = []
        for i in range(n - 1):
            p0, p1 = pts[i], pts[i + 1]
            m0 = (p1[0] - p0[0], p1[1] - p0[1]) if i == 0 else (
                0.5 * (pts[i + 1][0] - pts[i - 1][0]), 0.5 * (pts[i + 1][1] - pts[i - 1][1]))
            m1 = (p1[0] - p0[0], p1[1] - p0[1]) if i == n - 2 else (
                0.5 * (pts[i + 2][0] - pts[i][0]), 0.5 * (pts[i + 2][1] - pts[i][1]))
            self.segments.append(CubicSegment(p0, p1, m0, m1))
        rough = sum(self._integrate(seg, 0.0, 1.0) for seg in self.segments)
        num = max(self.MIN_SAMPLES, int(rough * self.SAMPLES_PER_METER))
        self.s_table = [0.0] * (num + 1)
        self.seg_table = [0] * (num + 1)
        self.t_table = [0.0] * (num + 1)
        self._build(num)
        self.total = self.s_table[num]

    @staticmethod
    def _integrate(seg, t0, t1):
        if abs(t1 - t0) < 1e-12:
            return 0.0
        half, mid = (t1 - t0) / 2.0, (t0 + t1) / 2.0
        return sum(w * seg.speed(mid + half * x) for x, w in zip(GL_NODES, GL_WEIGHTS)) * half

    def _build(self, num):
        segs = self.segments
        total_t = len(segs)
        dt = total_t / num
        cum = 0.0
        for i in range(1, num + 1):
            gt = i * dt
            si = min(int(gt), len(segs) - 1)
            lt = gt - si
            pgt = (i - 1) * dt
            psi = min(int(pgt), len(segs) - 1)
            plt = pgt - psi
            if psi == si:
                cum += self._integrate(segs[si], plt, lt)
            else:
                cum += self._integrate(segs[psi], plt, 1.0)
                cum += self._integrate(segs[si], 0.0, lt)
            self.s_table[i] = cum
            self.seg_table[i] = si
            self.t_table[i] = lt

    def _lookup(self, s):
        if s <= 0:
            return 0, 0.0
        if s >= self.total:
            return len(self.segments) - 1, 1.0
        hi = bisect.bisect_right(self.s_table, s)
        lo = hi - 1
        rng = self.s_table[hi] - self.s_table[lo]
        frac = (s - self.s_table[lo]) / rng if rng > 1e-12 else 0.0
        sl, tl, sh, th = self.seg_table[lo], self.t_table[lo], self.seg_table[hi], self.t_table[hi]
        if sl == sh:
            return sl, tl + frac * (th - tl)
        if frac < 0.5:
            return sl, min(tl + frac * 2 * (1.0 - tl), 1.0)
        return sh, max((frac - 0.5) * 2 * th, 0.0)

    def clamp_s(self, s):
        return max(0.0, min(s, self.total))

    def point(self, s):
        i, t = self._lookup(self.clamp_s(s))
        return self.segments[i].point(t)

    def tangent(self, s):
        i, t = self._lookup(self.clamp_s(s))
        d = self.segments[i].deriv(t)
        m = math.hypot(d[0], d[1])
        return (1.0, 0.0) if m < 1e-12 else (d[0] / m, d[1] / m)

    def curvature(self, s):
        i, t = self._lookup(self.clamp_s(s))
        return self.segments[i].curvature(t)

    def arc_length_at_waypoint(self, idx):
        if idx <= 0:
            return 0.0
        if idx >= len(self.segments):
            return self.total
        num = len(self.s_table) - 1
        k = int(round(idx * num / len(self.segments)))
        return self.s_table[min(k, num)]

    def closest(self, xy):
        best = self._search(xy, 0.0, self.total)
        best = self._refine(best, xy, 0.0, self.total)
        return self._result(best, xy)

    def closest_in_range(self, xy, smin, smax):
        smin, smax = max(0.0, smin), min(self.total, smax)
        if smin >= smax:
            return self._result(smin, xy)
        best = self._search(xy, smin, smax)
        best = self._refine(best, xy, smin, smax)
        return self._result(best, xy)

    def _result(self, s, xy):
        p, tg = self.point(s), self.tangent(s)
        dx, dy = xy[0] - p[0], xy[1] - p[1]
        return s, p, tg[0] * dy - tg[1] * dx, tg

    def _search(self, q, s0, s1):
        best, bd = s0, float("inf")
        n = max(1, int((s1 - s0) / self.COARSE_SEARCH_STEP))
        step = (s1 - s0) / n
        for i in range(n + 1):
            s = s0 + i * step
            p = self.point(s)
            d = (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2
            if d < bd:
                bd, best = d, s
        return best

    def _refine(self, s, q, smin, smax):
        for _ in range(self.MAX_NEWTON_ITERATIONS):
            p, tg = self.point(s), self.tangent(s)
            dx, dy = p[0] - q[0], p[1] - q[1]
            f = dx * tg[0] + dy * tg[1]
            k = self.curvature(s)
            fp = 1.0 + k * (dx * -tg[1] + dy * tg[0])
            if abs(fp) < 1e-12:
                break
            ds = -f / fp
            s = max(smin, min(smax, s + ds))
            if abs(ds) < 1e-6:
                break
        return s


class VelocityProfile:
    MIN_VELOCITY = 0.1
    STATOR_CURRENT_LIMIT = 150.0

    def __init__(self, path: SplinePath, max_vel, max_accel, max_decel, start_vel=0.0, end_vel=0.0,
                 curvature_friction_fraction=0.9):
        L = path.total
        self.total = L
        num = max(100, int(L * 1000))
        ds = L / num
        eff_vel = [max_vel] * (num + 1)
        eff_acc = [max_accel] * (num + 1)
        curv_lim = [0.0] * (num + 1)
        kappas = [0.0] * (num + 1)
        max_fric = curvature_friction_fraction * GRAVITY
        for i in range(num + 1):
            k = abs(path.curvature(i * ds))
            kappas[i] = k
            c = math.sqrt(max_fric / k) if k > 1e-6 else eff_vel[i]
            curv_lim[i] = max(min(c, eff_vel[i]), self.MIN_VELOCITY)
        window = max(1, int(max_vel * 0.1 / ds))
        sm = maximum_filter1d(np.asarray(kappas), size=2 * window + 1, mode="nearest").tolist()

        fwd = [0.0] * (num + 1)
        fwd[0] = min(start_vel, curv_lim[0])
        for i in range(1, num + 1):
            full = min(self._motor(fwd[i - 1]), eff_acc[i])
            vt = min(math.sqrt(fwd[i - 1] ** 2 + 2 * full * ds), curv_lim[i], eff_vel[i])
            cen = vt * vt * max(sm[i - 1], sm[i])
            budget = eff_acc[i]
            avail = math.sqrt(budget * budget - cen * cen) if cen < budget else self.MIN_VELOCITY
            a = min(self._motor(vt), avail)
            fwd[i] = min(math.sqrt(fwd[i - 1] ** 2 + 2 * a * ds), curv_lim[i], eff_vel[i])
        bwd = [0.0] * (num + 1)
        bwd[num] = min(end_vel, curv_lim[num])
        for i in range(num - 1, -1, -1):
            full = min(max_decel, eff_acc[i])
            vt = min(math.sqrt(bwd[i + 1] ** 2 + 2 * full * ds), curv_lim[i], eff_vel[i])
            cen = vt * vt * max(sm[i], sm[i + 1])
            avail = math.sqrt(full * full - cen * cen) if cen < full else self.MIN_VELOCITY
            bwd[i] = min(math.sqrt(bwd[i + 1] ** 2 + 2 * avail * ds), curv_lim[i], eff_vel[i])
        self.v = [max(self.MIN_VELOCITY, min(f, b)) for f, b in zip(fwd, bwd)]

    @staticmethod
    def _motor(speed):
        return AccelerationLimiter.motor_accel(speed)

    def velocity(self, s):
        v = self.v
        if s <= 0:
            return v[0]
        if s >= self.total:
            return v[-1]
        ds = self.total / (len(v) - 1)
        f = s / ds
        lo = min(int(f), len(v) - 2)
        fr = f - lo
        return v[lo] + fr * (v[lo + 1] - v[lo])


class OldFollowPath(Follower):
    end_output = "idle"  # end(): swerve.setControl(new SwerveRequest.Idle())

    LOOKAHEAD_K, LOOKAHEAD_MIN, LOOKAHEAD_MAX, LOOKAHEAD_MAX_ARC = 0.15, 0.15, 1.0, math.pi / 6
    CT_KP, CT_KD, CURV_FF = 3.0, 0.5, 0.1
    COMPLETION_TOL = 0.05
    MAX_ROTATION_BUDGET = 0.30
    PROJECTION_MAX_DELTA = 0.5

    def __init__(self, limits: Limits = Limits(), kp_heading=KP_HEADING):
        self.limits = limits
        self.kp_heading = kp_heading
        self.max_friction = AccelerationLimiter.TRACTION

    @staticmethod
    def adapt(course):
        from courses import to_catmull

        return to_catmull(course)

    @staticmethod
    def geometry(native):
        p = SplinePath(native["points"])
        return np.array([p.point(s) for s in np.linspace(0.0, p.total, max(200, int(p.total * 100)))])

    def reset(self, native, start):
        lim = self.limits
        self.path = SplinePath(native["points"])
        self.profile = VelocityProfile(self.path, lim.max_speed, lim.max_accel, lim.max_accel)
        # RotationSupplier.interpolateAlongPath: heading waypoints -> arc length
        self.heads = []
        for idx, h in native["headings"]:
            lo = int(idx)
            frac = idx - lo
            ls, hs = self.path.arc_length_at_waypoint(lo), self.path.arc_length_at_waypoint(lo + 1)
            self.heads.append((ls + frac * (hs - ls), h))
        self.heads.sort(key=lambda e: e[0])
        self.last_cmd = tuple(start.vel)  # swerve.getFieldSpeeds()
        self.last_t = start.t
        self.last_cte = 0.0
        self.last_s = self.path.closest(start.pose[:2])[0]
        self.finished = False

    def _target_heading(self, s):
        w = self.heads
        if s <= w[0][0]:
            return w[0][1]
        if s >= w[-1][0]:
            return w[-1][1]
        for (s0, h0), (s1, h1) in zip(w, w[1:]):
            if s0 <= s <= s1:
                if s1 - s0 < 1e-9:
                    return h0
                return wrap(h0 + (s - s0) / (s1 - s0) * wrap(h1 - h0))
        return w[-1][1]

    def step(self, t, pose, vel):
        if self.finished:
            return (0.0, 0.0, 0.0)
        dt = t - self.last_t
        self.last_t = t
        path = self.path
        s, _, cte, tg = path.closest_in_range(pose[:2], self.last_s - self.PROJECTION_MAX_DELTA,
                                              self.last_s + self.PROJECTION_MAX_DELTA)
        speed_now = math.hypot(self.last_cmd[0], self.last_cmd[1])
        look = clamp(self.LOOKAHEAD_K * speed_now + self.LOOKAHEAD_MIN, self.LOOKAHEAD_MIN, self.LOOKAHEAD_MAX)
        kappa = abs(path.curvature(s))
        if kappa > 1e-6:
            look = min(look, self.LOOKAHEAD_MAX_ARC / kappa)
        look = max(look, self.LOOKAHEAD_MIN)
        s_target = min(s + look, path.total)
        tp = path.point(s_target)
        v_prof = self.profile.velocity(s)
        tx, ty = tp[0] - pose[0], tp[1] - pose[1]
        dist = math.hypot(tx, ty)
        dx, dy = (tx / dist, ty / dist) if dist > 1e-6 else tg
        rate = (cte - self.last_cte) / dt if dt > 1e-6 else 0.0
        corr = self.CT_KP * cte + self.CT_KD * rate
        nx, ny = -tg[1], tg[0]
        ff = self.CURV_FF * v_prof * v_prof * path.curvature(s)
        k = -corr + ff
        vx, vy = dx * v_prof + nx * k, dy * v_prof + ny * k
        # rotation: target heading (see module docstring) -> P
        omega = clamp(self.kp_heading * wrap(self._target_heading(s) - pose[2]),
                      -self.limits.max_omega, self.limits.max_omega)
        if omega != 0.0:
            max_ang = self.MAX_ROTATION_BUDGET * self.max_friction
            ang = abs(omega) * BASE_R
            if ang > max_ang:
                omega = math.copysign(max_ang / BASE_R, omega)
                ang = max_ang
            avail = math.sqrt(max(0.0, 1.0 - ang * ang / (self.max_friction ** 2)))
            vx *= avail
            vy *= avail
        target = self._normalize((vx, vy, omega), pose[2])
        out = AccelerationLimiter.limit(self.last_cmd, target, dt if dt > 1e-6 else 0.02)
        self.last_cmd = out
        self.last_cte = cte
        self.last_s = s
        # isFinished(): projected near the end and commanded speed < 0.1 (end velocity 0)
        if self.last_s >= path.total - self.COMPLETION_TOL and math.hypot(out[0], out[1]) < 0.1:
            self.finished = True
        return out

    def _normalize(self, v, heading):
        vr = rot((v[0], v[1]), -heading)
        top = max(math.hypot(vr[0] - v[2] * my, vr[1] + v[2] * mx) for mx, my in RP.module_positions)
        if top > self.limits.max_speed:
            k = self.limits.max_speed / top
            return (v[0] * k, v[1] * k, v[2] * k)
        return v

    def done(self):
        return self.finished
