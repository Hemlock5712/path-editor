"""Port of PathPlannerLib (mjansen4857/pathplanner, branch 2027, commit 7fbd798):
path/PathPlannerPath.java (createPath, samplePath, precalcValues), trajectory/PathPlannerTrajectory.java
(generateStates, forwardAccelPass, reverseAccelPass, timing, sample), PathPlannerTrajectoryState.interpolate,
config/RobotConfig + ModuleConfig (module force / friction limits), util/GeometryUtil,
controllers/PPHolonomicDriveController and commands/FollowPathCommand.

Line-for-line in structure. Approximations (see RESULTS.md):
- RobotConfig comes from the sim (mass 60 kg, MOI 6, wheel radius, gearing 6.75, Kraken X60 FOC from
  WPILib's DCMotor numbers, the plant's current limit, max drive speed = the sim's free speed) with wheel COF
  the sim's static mu (1.5); it is NOT changed for the low-grip course (nobody adapts).
- Drive feedforwards: with ff=True, each state's module forces as PathPlannerTrajectory computes them (the module
  speed change to the next state over the time between, times mass / modules, along the module's angle), linearly
  blended between states and sent as wheel force feedforward the way CTRE's PathPlanner template does
  (ApplyRobotSpeeds.withWheelForceFeedforwardsX/Y(feedforwards.robotRelativeForcesX/Y())). Off by default.
- Events, point-towards zones, constraint zones, flipping and Choreo paths are not ported.
- A non-zero GoalEndState velocity (a chain of paths) is ported: the end state carries it, and at the end
  the command stops with the last output standing (FollowPathCommand.end only zeroes below 0.1 m/s).
- The trajectory is always generated from the measured start speeds (FollowPathCommand's branch when the
  robot doesn't match the path's IdealStartingState; when it does, the ideal trajectory is the same
  generation with the ideal speeds).
"""

import math

import numpy as np

from common import RP, Limits, clamp, rot, wrap
from followers.base import Follower

TARGET_INCREMENT = 0.05
TARGET_SPACING = 0.2


# ============================================================================================ WPILib bits
class DCMotor:
    def __init__(self, v, stall_torque, stall_current, free_current, free_speed, n=1):
        self.v = v
        self.stall_torque = stall_torque * n
        self.stall_current = stall_current * n
        self.free_current = free_current * n
        self.free_speed = free_speed
        self.R = v / self.stall_current
        self.Kv = free_speed / (v - self.R * self.free_current)
        self.Kt = self.stall_torque / self.stall_current
        self._args = (v, stall_torque, stall_current, free_current, free_speed, n)

    @staticmethod
    def kraken_x60_foc(n=1):
        # the numbers in PathPlanner's own lib/trajectory/dc_motor.dart (WPILib's getKrakenX60Foc)
        return DCMotor(12.0, 9.37, 483.0, 2.0, 5800.0 * 2 * math.pi / 60.0, n)

    def with_reduction(self, g):
        v, st, sc, fc, fs, n = self._args
        return DCMotor(v, st * g, sc, fc, fs / g, n)

    def current(self, speed, voltage):
        return -1.0 / self.Kv / self.R * speed + voltage / self.R

    def torque(self, current):
        return current * self.Kt


def calculate_radius(a, b, c):
    """GeometryUtil.calculateRadius (signed); +-inf / nan where Java divides by zero."""
    vba = (a[0] - b[0], a[1] - b[1])
    vbc = (c[0] - b[0], c[1] - b[1])
    cross = vba[0] * vbc[1] - vba[1] * vbc[0]
    sign = 1 if cross < 0 else -1
    ab, bc, ac = math.dist(a, b), math.dist(b, c), math.dist(a, c)
    p = (ab + bc + ac) / 2
    area = math.sqrt(abs(p * (p - ab) * (p - bc) * (p - ac)))
    num = sign * ab * bc * ac
    if area == 0.0:
        return math.nan if num == 0 else math.copysign(math.inf, num)
    return num / (4 * area)


def cubic_lerp(a, b, c, d, t):
    def lerp(p, q):
        return (p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t)

    p0 = lerp(lerp(a, b), lerp(b, c))
    p1 = lerp(lerp(b, c), lerp(c, d))
    return lerp(p0, p1)


class RobotConfig:
    def __init__(self, mass=RP.mass, moi=RP.inertia, wheel_radius=RP.wheel_radius, max_drive_speed=None,
                 wheel_cof=RP.mu_static, gearing=RP.drive_gear, current_limit=RP.stator_limit,
                 module_positions=RP.module_positions):
        self.mass, self.moi = mass, moi
        self.wheel_radius = wheel_radius
        self.max_drive_speed = RP.free_speed if max_drive_speed is None else max_drive_speed
        self.wheel_cof = wheel_cof
        self.motor = DCMotor.kraken_x60_foc(1).with_reduction(gearing)
        self.current_limit = current_limit * 1
        mx = self.max_drive_speed / wheel_radius
        self.torque_loss = max(self.motor.torque(min(self.motor.current(mx, 12.0), self.current_limit)), 0.0)
        self.mods = [tuple(m) for m in module_positions]
        self.n = len(self.mods)
        self.pivot = [math.hypot(*m) for m in self.mods]
        self.wheel_friction_force = wheel_cof * (mass / self.n * 9.8)
        self.max_torque_friction = self.wheel_friction_force * wheel_radius
        rows = []
        for x, y in self.mods:
            rows.append([1.0, 0.0, -y])
            rows.append([0.0, 1.0, x])
        self.pinv = np.linalg.pinv(np.array(rows))

    def to_module_states(self, sp):
        """SwerveDriveKinematics.toSwerveModuleVelocities: [(speed, angle)]."""
        vx, vy, w = sp
        out = []
        for x, y in self.mods:
            cx, cy = vx - w * y, vy + w * x
            out.append((math.hypot(cx, cy), math.atan2(cy, cx) if (cx or cy) else 0.0))
        return out

    def to_chassis(self, states):
        """Forward kinematics (least squares) from [(speed, angle)]."""
        v = []
        for s, a in states:
            v.append(s * math.cos(a))
            v.append(s * math.sin(a))
        r = self.pinv @ np.array(v)
        return (float(r[0]), float(r[1]), float(r[2]))


# ============================================================================================ path
class PathPoint:
    __slots__ = ("position", "rotation_target", "pos", "dist")

    def __init__(self, position, rotation_target=None):
        self.position = position
        self.rotation_target = rotation_target  # (waypoint-relative pos, rotation) or None
        self.pos = 0.0
        self.dist = 0.0


class PathPlannerPath:
    def __init__(self, native):
        self.wps = native["waypoints"]
        self.rotation_targets = sorted(native["rotation_targets"], key=lambda r: r[0])
        self.goal_rotation = native["goal_rotation"]
        self.points = self._create()
        self._precalc()

    def _sample(self, pos):
        n = len(self.wps)
        pos = clamp(pos, 0.0, n - 1.0)
        i = int(math.floor(pos))
        if i == n - 1:
            i -= 1
        t = pos - i
        return cubic_lerp(self.wps[i]["anchor"], self.wps[i]["next"], self.wps[i + 1]["prev"], self.wps[i + 1]["anchor"], t)

    def _create(self):
        unadded = list(self.rotation_targets)
        pts = []
        nseg = len(self.wps) - 1
        p0 = PathPoint(self._sample(0.0))
        p0.pos = 0.0
        pts.append(p0)
        pos = TARGET_INCREMENT
        while pos < nseg:
            position = self._sample(pos)
            distance = math.dist(pts[-1].position, position)
            if distance <= 0.01:
                pos = min(pos + TARGET_INCREMENT, nseg)
                continue
            prev_wp = pos - TARGET_INCREMENT
            delta = distance - TARGET_SPACING
            if delta > TARGET_SPACING * 0.25:
                ci = (TARGET_SPACING * TARGET_INCREMENT) / distance
                pos = pos - TARGET_INCREMENT + ci
                position = self._sample(pos)
                if math.dist(pts[-1].position, position) - TARGET_SPACING > TARGET_SPACING * 0.25:
                    pos = pos - ci * 0.5
                    position = self._sample(pos)
            elif delta < -TARGET_SPACING * 0.25:
                ci = (TARGET_SPACING * TARGET_INCREMENT) / distance
                pos = pos - TARGET_INCREMENT + ci
                position = self._sample(pos)
                if math.dist(pts[-1].position, position) - TARGET_SPACING < -TARGET_SPACING * 0.25:
                    pos = pos + ci * 0.5
                    position = self._sample(pos)
            target = None
            prev_point = pts[-1]
            while unadded and prev_wp <= unadded[0][0] <= pos:
                if abs(unadded[0][0] - prev_wp) < 0.001:
                    prev_point.rotation_target = unadded.pop(0)
                elif abs(unadded[0][0] - pos) < 0.001:
                    target = unadded.pop(0)
                else:
                    t = unadded.pop(0)
                    pp = PathPoint(self._sample(t[0]), t)
                    pp.pos = t[0]
                    pts.append(pp)
            pp = PathPoint(position, target)
            pp.pos = pos
            pts.append(pp)
            pos = min(pos + TARGET_INCREMENT, nseg)
        true_inc = nseg - (pos - TARGET_INCREMENT)
        pos = nseg
        invalid = True
        while invalid:
            position = self._sample(pos)
            distance = math.dist(pts[-1].position, position)
            if distance <= 0.01:
                if len(pts) < 2:
                    pp = PathPoint(position)
                    pp.pos = pos
                    pts.append(pp)
                break
            prev_pos = pos - true_inc
            delta = distance - TARGET_SPACING
            if delta > TARGET_SPACING * 0.25:
                ci = (TARGET_SPACING * true_inc) / distance
                pos = pos - true_inc + ci
                true_inc = ci
                position = self._sample(pos)
                if math.dist(pts[-1].position, position) - TARGET_SPACING > TARGET_SPACING * 0.25:
                    pos = pos - ci * 0.5
                    true_inc = ci * 0.5
                    position = self._sample(pos)
            else:
                invalid = False
            if unadded and abs(unadded[0][0] - prev_pos) <= abs(unadded[0][0] - pos):
                pts[-1].rotation_target = unadded.pop(0)
            pp = PathPoint(position)
            pp.pos = pos
            pts.append(pp)
            pos = nseg
        i = 1
        while i < len(pts) - 1:
            r = calculate_radius(pts[i - 1].position, pts[i].position, pts[i + 1].position)
            if not math.isfinite(r):
                i += 1
                continue
            if abs(r) < 0.25:
                a, b, c = pts[i - 1].pos, pts[i].pos, pts[i + 1].pos
                new = []
                for w in (a + (b - a) * 0.33, a + (b - a) * 0.67, b + (c - b) * 0.33, b + (c - b) * 0.67):
                    q = PathPoint(self._sample(w))
                    q.pos = w
                    new.append(q)
                pts[i:i] = new[:2]
                pts[i + 3:i + 3] = new[2:]
                i += 4
            elif abs(r) < 0.5:
                a, b, c = pts[i - 1].pos, pts[i].pos, pts[i + 1].pos
                q0, q1 = PathPoint(self._sample(a + (b - a) * 0.5)), PathPoint(self._sample(b + (c - b) * 0.5))
                q0.pos, q1.pos = a + (b - a) * 0.5, b + (c - b) * 0.5
                pts.insert(i, q0)
                pts.insert(i + 2, q1)
                i += 2
            i += 1
        return pts

    def _precalc(self):
        pts = self.points
        for i in range(1, len(pts)):
            pts[i].dist = pts[i - 1].dist + math.dist(pts[i - 1].position, pts[i].position)
        pts[-1].rotation_target = (-1.0, self.goal_rotation)


# ============================================================================================ trajectory
class State:
    __slots__ = ("t", "vel", "pose", "lin", "heading", "dpos", "mod_v", "mod_fpos", "mod_dpos", "mod_fangle",
                 "mod_angle", "ff")

    def __init__(self):
        self.t = 0.0
        self.vel = (0.0, 0.0, 0.0)  # field speeds
        self.lin = 0.0
        self.heading = 0.0
        self.dpos = 0.0


def _rot_interp(a, b, t):
    """Rotation2d.interpolate: shortest way, t clamped."""
    return wrap(a + wrap(b - a) * clamp(t, 0.0, 1.0))


def _next_rot_idx(pts, start):
    for i in range(start, len(pts) - 1):
        if pts[i].rotation_target is not None:
            return i
    return len(pts) - 1


def generate_trajectory(path: PathPlannerPath, start_field_speeds, start_rotation, cfg: RobotConfig, lim: Limits,
                        goal_velocity=0.0):
    pts = path.points
    n = len(pts)
    states = []
    prev_idx, prev_rot = 0, start_rotation
    nxt = _next_rot_idx(pts, 0)
    nxt_rot = pts[nxt].rotation_target[1]
    for i in range(n):
        p = pts[i]
        if i > nxt:
            prev_idx, prev_rot = nxt, nxt_rot
            nxt = _next_rot_idx(pts, i)
            nxt_rot = pts[nxt].rotation_target[1]
        den = pts[nxt].dist - pts[prev_idx].dist
        t = (p.dist - pts[prev_idx].dist) / den if den != 0 else math.nan
        if math.isnan(t):
            t = 0.0
        t2 = (1.0 - math.cos(t * math.pi)) / 2.0
        hol = _rot_interp(prev_rot, nxt_rot, t2)
        st = State()
        st.pose = (p.position[0], p.position[1], hol)
        if i != n - 1:
            d = (pts[i + 1].position[0] - p.position[0], pts[i + 1].position[1] - p.position[1])
            st.heading = 0.0 if math.hypot(*d) <= 1e-6 else math.atan2(d[1], d[0])
        else:
            st.heading = states[i - 1].heading
        if i != 0:
            st.dpos = math.dist(st.pose[:2], states[i - 1].pose[:2])
        st.mod_fpos, st.mod_dpos, st.mod_v = [], [], [0.0] * cfg.n
        for m, (mx, my) in enumerate(cfg.mods):
            off = rot((mx, my), hol)
            fp = (st.pose[0] + off[0], st.pose[1] + off[1])
            st.mod_fpos.append(fp)
            st.mod_dpos.append(math.dist(fp, states[i - 1].mod_fpos[m]) if i != 0 else 0.0)
        states.append(st)
    for i in range(n):
        st = states[i]
        st.mod_fangle, st.mod_angle = [0.0] * cfg.n, [0.0] * cfg.n
        for m in range(cfg.n):
            if i != n - 1:
                a, b = st.mod_fpos[m], states[i + 1].mod_fpos[m]
                d = (b[0] - a[0], b[1] - a[1])
                st.mod_fangle[m] = 0.0 if math.hypot(*d) <= 1e-6 else math.atan2(d[1], d[0])
            else:
                st.mod_fangle[m] = states[i - 1].mod_fangle[m]
            st.mod_angle[m] = wrap(st.mod_fangle[m] - st.pose[2])

    # initial module velocities (toSwerveModuleStates(fieldStartingSpeeds), as the Java does)
    init = cfg.to_module_states(start_field_speeds)
    s0 = states[0]
    for m in range(cfg.n):
        s0.mod_v[m] = init[m][0]
    s0.t = 0.0
    s0.vel = tuple(start_field_speeds)
    s0.lin = math.hypot(start_field_speeds[0], start_field_speeds[1])

    _forward(states, cfg, lim)
    end = states[-1]
    # the GoalEndState's velocity along the end heading
    end_field = (goal_velocity * math.cos(end.heading), goal_velocity * math.sin(end.heading))
    end_robot = rot(end_field, -end.pose[2])
    end_states = cfg.to_module_states((end_robot[0], end_robot[1], 0.0))
    for m in range(cfg.n):
        end.mod_v[m] = end_states[m][0]
    end.vel = (end_field[0], end_field[1], 0.0)
    end.lin = goal_velocity
    _reverse(states, cfg, lim)

    for i in range(1, n):
        prev, st = states[i - 1], states[i]
        sv = st.lin + prev.lin
        if abs(sv) < 1e-6 or abs(st.dpos) < 1e-6:
            st.t = prev.t
        else:
            st.t = prev.t + 2 * st.dpos / sv
    _feedforwards(states, cfg)
    return states


def _feedforwards(states, cfg):
    """DriveFeedforwards per state: robot-relative (fx, fy) per module; zero at the last state."""
    for i, st in enumerate(states):
        if i == len(states) - 1:
            st.ff = [(0.0, 0.0)] * cfg.n
            continue
        nxt = states[i + 1]
        dt = nxt.t - st.t
        out = []
        for m in range(cfg.n):
            f = (nxt.mod_v[m] - st.mod_v[m]) / dt * cfg.mass / cfg.n if dt > 1e-9 else 0.0
            out.append((f * math.cos(st.mod_angle[m]), f * math.sin(st.mod_angle[m])))
        st.ff = out


def feedforward_at(states, time):
    """The sampled state's feedforwards (PathPlannerTrajectoryState.interpolate blends them linearly)."""
    if time <= states[0].t:
        return states[0].ff
    if time >= states[-1].t:
        return states[-1].ff
    lo, hi = 1, len(states) - 1
    while lo != hi:
        mid = (lo + hi) // 2
        if states[mid].t < time:
            lo = mid + 1
        else:
            hi = mid
    b, a = states[lo], states[lo - 1]
    u = 0.0 if b.t - a.t < 1e-3 else (time - a.t) / (b.t - a.t)
    return [(p[0] + (q[0] - p[0]) * u, p[1] + (q[1] - p[1]) * u) for p, q in zip(a.ff, b.ff)]


def _pass_accel(st, cfg, lim, mod_vel_ref, reverse):
    lfx = lfy = 0.0
    torque = 0.0
    for m in range(cfg.n):
        last = mod_vel_ref[m]
        cur = min(cfg.motor.current(last / cfg.wheel_radius, 12.0), cfg.current_limit)
        avail = cfg.motor.torque(cur) - (0.0 if reverse else cfg.torque_loss)
        avail = min(avail, cfg.max_torque_friction)
        f = avail / cfg.wheel_radius
        ang = st.mod_fangle[m] + (math.pi if reverse else 0.0)
        fx, fy = f * math.cos(ang), f * math.sin(ang)
        lfx += fx
        lfy += fy
        d = (st.mod_fpos[m][0] - st.pose[0], st.mod_fpos[m][1] - st.pose[1])
        to_mod = math.atan2(d[1], d[0]) if (d[0] or d[1]) else 0.0
        theta = (0.0 if math.hypot(fx, fy) <= 1e-6 else math.atan2(fy, fx)) - to_mod
        torque += f * cfg.pivot[m] * math.sin(theta)
    ang_acc = clamp(torque / cfg.moi, -lim.max_alpha, lim.max_alpha)
    ax, ay = lfx / cfg.mass, lfy / cfg.mass
    a = math.hypot(ax, ay)
    if a > lim.max_accel:
        ax, ay = ax * lim.max_accel / a, ay * lim.max_accel / a
    ar = rot((ax, ay), -st.pose[2])
    return cfg.to_module_states((ar[0], ar[1], ang_acc))


def _equalize_and_desaturate(st, prev_angle, nxt, cfg, max_v, max_w):
    max_dt = real_max_dt = 0.0
    for m in range(cfg.n):
        drot = math.degrees(wrap(st.mod_angle[m] - prev_angle[m]))
        v = st.mod_v[m]
        dt = nxt.mod_dpos[m] / v if v != 0 else (math.inf if nxt.mod_dpos[m] > 0 else math.nan)
        if math.isfinite(dt):
            real_max_dt = max(dt, real_max_dt)
            if abs(drot) < 60:
                max_dt = max(dt, max_dt)
    if max_dt == 0.0:
        max_dt = real_max_dt
    if max_dt > 0:
        for m in range(cfg.n):
            if abs(math.degrees(wrap(st.mod_angle[m] - prev_angle[m]))) >= 60:
                continue
            st.mod_v[m] = nxt.mod_dpos[m] / max_dt
    desired = cfg.to_chassis(list(zip(st.mod_v, st.mod_angle)))
    real_max = max(abs(v) for v in st.mod_v)
    if real_max != 0:
        tp = math.hypot(desired[0], desired[1]) / max_v if abs(max_v) > 1e-8 else 0.0
        rp = abs(desired[2]) / abs(max_w) if abs(max_w) > 1e-8 else 0.0
        mp = max(tp, rp)
        scale = min(1.0, cfg.max_drive_speed / real_max)
        if mp > 0:
            scale = min(scale, 1.0 / mp)
        st.mod_v = [v * scale for v in st.mod_v]
    rs = cfg.to_chassis(list(zip(st.mod_v, st.mod_angle)))
    f = rot((rs[0], rs[1]), st.pose[2])
    st.vel = (f[0], f[1], rs[2])
    st.lin = math.hypot(f[0], f[1])


def _forward(states, cfg, lim):
    for i in range(1, len(states) - 1):
        prev, st, nxt = states[i - 1], states[i], states[i + 1]
        acc = _pass_accel(st, cfg, lim, prev.mod_v, False)
        for m in range(cfg.n):
            st.mod_v[m] = math.sqrt(abs(prev.mod_v[m] ** 2 + 2 * acc[m][0] * st.mod_dpos[m]))
            r = calculate_radius(prev.mod_fpos[m], st.mod_fpos[m], nxt.mod_fpos[m])
            if math.isfinite(r):
                safe = math.sqrt(cfg.wheel_friction_force * abs(r) / (cfg.mass / cfg.n))
                st.mod_v[m] = min(st.mod_v[m], safe)
        _equalize_and_desaturate(st, prev.mod_angle, nxt, cfg, lim.max_speed, lim.max_omega)


def _reverse(states, cfg, lim):
    for i in range(len(states) - 2, 0, -1):
        st, nxt = states[i], states[i + 1]
        acc = _pass_accel(st, cfg, lim, nxt.mod_v, True)
        for m in range(cfg.n):
            mv = math.sqrt(abs(nxt.mod_v[m] ** 2 + 2 * acc[m][0] * nxt.mod_dpos[m]))
            st.mod_v[m] = min(mv, st.mod_v[m])
        max_v = min(lim.max_speed, st.lin)
        max_w = min(lim.max_omega, abs(st.vel[2]))
        _equalize_and_desaturate(st, states[i - 1].mod_angle, nxt, cfg, max_v, max_w)


def _interpolate(a: State, b: State, t):
    """PathPlannerTrajectoryState.interpolate -> (time, pose, field speeds)."""
    tt = a.t + (b.t - a.t) * t
    if tt - a.t < 0:
        return _interpolate(b, a, 1 - t)
    vel = tuple(p + (q - p) * t for p, q in zip(a.vel, b.vel))
    lin = a.lin + (b.lin - a.lin) * t
    x, y = a.pose[0], a.pose[1]
    ch, sh = math.cos(a.heading), math.sin(a.heading)
    it = a.t + 0.01
    while True:
        iu = (it - a.t) / (tt - a.t)
        iv = a.lin + (lin - a.lin) * iu
        if it >= tt - 0.01:
            d = tt - it
            x += iv * ch * d
            y += iv * sh * d
            break
        x += iv * ch * 0.01
        y += iv * sh * 0.01
        it += 0.01
    return tt, (x, y, _rot_interp(a.pose[2], b.pose[2], t)), vel


def sample(states, time):
    if time <= states[0].t:
        s = states[0]
        return s.pose, s.vel
    if time >= states[-1].t:
        s = states[-1]
        return s.pose, s.vel
    lo, hi = 1, len(states) - 1
    while lo != hi:
        mid = (lo + hi) // 2
        if states[mid].t < time:
            lo = mid + 1
        else:
            hi = mid
    b, a = states[lo], states[lo - 1]
    if abs(b.t - a.t) < 1e-3:
        return b.pose, b.vel
    _, pose, vel = _interpolate(a, b, (time - a.t) / (b.t - a.t))
    return pose, vel


class PathPlanner(Follower):
    """FollowPathCommand + PPHolonomicDriveController (kP 5 / 5 by default). hold=True keeps running the
    controller on the final state (feedforward 0) until its own pose is within 2 cm / hold_deg (2) and the
    measured speed is under 5 cm/s, as the drive-to-pose race's '+ final hold' variant."""

    ff_forces = None

    def __init__(self, kp=5.0, kp_rot=5.0, hold=False, cof=None, ff=False, hold_deg=2.0, limits: Limits = Limits()):
        self.kp, self.kp_rot, self.hold, self.ff = kp, kp_rot, hold, ff
        self.hold_rad = math.radians(hold_deg)  # the hold's heading tolerance
        self.lim = limits
        self.cfg = RobotConfig() if cof is None else RobotConfig(wheel_cof=cof)

    @staticmethod
    def adapt(course):
        from courses import to_bezier

        return to_bezier(course)

    @staticmethod
    def geometry(native):
        return np.array([p.position for p in PathPlannerPath(native).points], float)

    def reset(self, native, start):
        path = PathPlannerPath(native)
        self.goal_velocity = native.get("goal_velocity", 0.0)
        self.states = generate_trajectory(path, start.vel, start.pose[2], self.cfg, self.lim, self.goal_velocity)
        self.total = self.states[-1].t
        self.t0 = start.t
        self.goal = self.states[-1].pose
        self.finished = False
        self.last = tuple(start.vel)

    def step(self, t, pose, vel):
        self.ff_forces = None
        if self.finished:
            return (0.0, 0.0, 0.0)
        now = t - self.t0
        if now >= self.total:  # isFinished(): timer.hasElapsed(totalTime)
            if self.goal_velocity >= 0.1:  # a rolling end: end() keeps the last output for the next command
                self.finished = True
                return self.last
            if not self.hold:
                self.finished = True
                return (0.0, 0.0, 0.0)
            g = self.goal
            ex, ey, eth = g[0] - pose[0], g[1] - pose[1], wrap(g[2] - pose[2])
            if math.hypot(ex, ey) < 0.02 and abs(eth) < self.hold_rad and math.hypot(vel[0], vel[1]) < 0.05:
                self.finished = True
                return (0.0, 0.0, 0.0)
            return (self.kp * ex, self.kp * ey, self.kp_rot * eth)
        tp, ff = sample(self.states, now)
        if self.ff:  # robot-relative to the field frame by the robot's measured heading, as CTRE turns it back
            self.ff_forces = [rot(f, pose[2]) for f in feedforward_at(self.states, now)]
        self.last = (ff[0] + self.kp * (tp[0] - pose[0]), ff[1] + self.kp * (tp[1] - pose[1]),
                     ff[2] + self.kp_rot * wrap(tp[2] - pose[2]))
        return self.last

    def done(self):
        return self.finished
