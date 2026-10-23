"""Port of PathPlanner's new path system, "path2" (mjansen4857/pathplanner, branch new-path, commit
af86ef2b, 2026-09-09). A PRE-RELEASE branch: the GUI has it, no PathPlannerLib release ships it yet, so
the robot side (FollowDynamicPath) is the GUI's own port of it.

- lib/path2/simulation/path2_path_follower.dart: Path2PathFollower, "the controller portion of the supplied
  FollowDynamicPath command". No trajectory: the robot heads for the current waypoint at P 4 x the distance
  left along the remaining waypoints (capped at the waypoint's max velocity), with a cross-track P 2 to the
  segment and a profiled heading P 5 (the next pose waypoint's rotation, else the heading held at the
  handoff). It moves on to the next waypoint within the handoff distance (default 0.25 m) and finishes
  within the end tolerance (default 10 cm and 2 deg).
- lib/path2/simulation/path2_simulator.dart: the GUI's path time. The follower's request goes through a
  swerve setpoint generator and the pose is integrated at 50 Hz, from rest at the first waypoint, until
  the follower finishes.
- The setpoint generator is PathPlannerLib's Java SwerveSetpointGenerator (branch 2027, 7fbd798), which
  the GUI's swerve_setpoint_generator.dart trims (no PathConstraints, 12 V). With constraints=None the two
  are the same.

Path2 waypoints have no Bezier handles and no linear acceleration limit: acceleration comes from the
robot (motor torque, current limit, wheel friction) through the setpoint generator. `accel_cap` passes
the bench's shared 6 m/s^2 as a PathConstraints to the generator, which PathPlannerLib supports.

Not ported: point-towards waypoints, condition transitions, events, flipping.
"""

import math

from common import RP, Limits, wrap
from followers.base import Follower
from followers.pathplanner import RobotConfig

PERIOD = 0.02
EPS = 1e-6
GUI_STEER_RATE = 10.0 * 2.0 * math.pi  # swerve_setpoint_generator.dart: 10 rotations per second
DEFAULT_HANDOFF = 0.25  # DistanceTransition.defaultDistanceMeters
DEFAULT_END_DISTANCE = 0.1  # EndTolerance defaults
DEFAULT_END_DEGREES = 2.0


# ============================================================================================ WPILib bits
def _eps_eq(a, b, e=EPS):
    return a - e <= b <= a + e


def _speeds_eq(a, b):
    return all(_eps_eq(x, y) for x, y in zip(a, b))


def _flip(rot):
    return abs(wrap(rot)) > math.pi / 2.0


def _discretize(sp, dt):
    """ChassisSpeeds.discretize: the constant twist that makes the requested transform over dt."""
    vx, vy, w = sp
    dx, dy, dth = vx * dt, vy * dt, w * dt
    if abs(dth) < 1e-9:
        s, c = 1.0 - dth * dth / 6.0, dth / 2.0 - dth ** 3 / 24.0
    else:
        s, c = math.sin(dth) / dth, (1.0 - math.cos(dth)) / dth
    det = s * s + c * c
    return ((s * dx + c * dy) / det / dt, (-c * dx + s * dy) / det / dt, dth / dt)


def _integrate(pose, sp, dt):
    """Pose2d.exp of the robot-relative twist sp * dt."""
    dx, dy, dth = sp[0] * dt, sp[1] * dt, sp[2] * dt
    if abs(dth) < 1e-9:
        s, c = 1.0 - dth * dth / 6.0, dth / 2.0 - dth ** 3 / 24.0
    else:
        s, c = math.sin(dth) / dth, (1.0 - math.cos(dth)) / dth
    tx, ty = dx * s - dy * c, dx * c + dy * s
    ch, sh = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + tx * ch - ty * sh, pose[1] + tx * sh + ty * ch, wrap(pose[2] + dth))


def _to_robot(v, heading):
    c, s = math.cos(heading), math.sin(heading)
    return (v[0] * c + v[1] * s, -v[0] * s + v[1] * c, v[2])


def _to_field(v, heading):
    c, s = math.cos(heading), math.sin(heading)
    return (v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2])


# ============================================================================================ setpoints
class Setpoint:
    def __init__(self, speeds, states):
        self.speeds = tuple(speeds)  # robot-relative
        self.states = [list(s) for s in states]  # [speed, angle]


class SetpointGenerator:
    """PathPlannerLib SwerveSetpointGenerator.generateSetpoint (12 V, no brownout), feedforwards left out.
    constraints = (max speed, max accel, max omega, max alpha) or None."""

    def __init__(self, cfg: RobotConfig, steer_rate, constraints=None):
        self.cfg, self.steer_rate, self.constraints = cfg, steer_rate, constraints

    def at(self, robot_speeds):
        """A setpoint already moving at robot_speeds (the measured state at a command's start)."""
        return Setpoint(robot_speeds, self.cfg.to_module_states(robot_speeds))

    def generate(self, prev: Setpoint, desired, dt=PERIOD):
        cfg, n = self.cfg, self.cfg.n
        max_speed = cfg.max_drive_speed
        lim = self.constraints
        if lim is not None:
            vx, vy, w = desired
            v = math.hypot(vx, vy)
            if v > lim[0]:
                vx, vy = vx * lim[0] / v, vy * lim[0] / v
            desired = (vx, vy, max(-lim[2], min(lim[2], w)))
        des = [list(s) for s in cfg.to_module_states(desired)]
        real_max = max(abs(s[0]) for s in des)
        if real_max > max_speed:
            for s in des:
                s[0] *= max_speed / real_max
        desired = cfg.to_chassis(des)

        need_to_steer = True
        if _speeds_eq(desired, (0.0, 0.0, 0.0)):
            need_to_steer = False
            for m in range(n):
                des[m] = [0.0, prev.states[m][1]]

        prev_vx, prev_vy, prev_h = [0.0] * n, [0.0] * n, [0.0] * n
        des_vx, des_vy, des_h = [0.0] * n, [0.0] * n, [0.0] * n
        all_flip = True
        for m in range(n):
            ps, pa = prev.states[m]
            ds, da = des[m]
            prev_vx[m], prev_vy[m] = math.cos(pa) * ps, math.sin(pa) * ps
            prev_h[m] = wrap(pa + math.pi) if ps < 0.0 else pa
            des_vx[m], des_vy[m] = math.cos(da) * ds, math.sin(da) * ds
            des_h[m] = wrap(da + math.pi) if ds < 0.0 else da
            if all_flip and abs(wrap(des_h[m] - prev_h[m])) < math.pi / 2.0:
                all_flip = False
        if all_flip and not _speeds_eq(prev.speeds, (0.0, 0.0, 0.0)) and not _speeds_eq(desired, (0.0, 0.0, 0.0)):
            return self.generate(prev, (0.0, 0.0, 0.0), dt)

        dx = desired[0] - prev.speeds[0]
        dy = desired[1] - prev.speeds[1]
        dth = desired[2] - prev.speeds[2]
        min_s = 1.0
        override = [None] * n
        for m in range(n):
            ps, pa = prev.states[m]
            if not need_to_steer:
                override[m] = pa
                continue
            max_theta_step = dt * self.steer_rate
            if _eps_eq(ps, 0.0):
                if _eps_eq(des[m][0], 0.0):
                    override[m] = pa
                    continue
                need = wrap(des[m][1] - pa)
                if _flip(need):
                    need = wrap(need + math.pi)
                if abs(need) / max_theta_step <= 1.0:
                    override[m] = des[m][1]
                else:
                    override[m] = wrap(pa + math.copysign(max_theta_step, need))
                    min_s = 0.0
                continue
            if min_s == 0.0:
                continue
            max_heading_change = (dt * cfg.wheel_friction_force) / ((cfg.mass / n) * abs(ps))
            max_theta_step = min(max_theta_step, max_heading_change)
            min_s = min(min_s, _steering_max_s(prev_vx[m], prev_vy[m], prev_h[m], des_vx[m], des_vy[m], des_h[m],
                                               max_theta_step))

        fx = fy = 0.0
        torque = 0.0
        for m in range(n):
            ps, pa = prev.states[m]
            last = ps / cfg.wheel_radius
            cur = max(min(cfg.motor.current(abs(last), 12.0), cfg.current_limit), 0.0)
            rev = max(min(abs(cfg.motor.current(abs(last), -12.0)), cfg.current_limit), 0.0)
            fwd_t, rev_t = cfg.motor.torque(cur), cfg.motor.torque(rev)
            # SwerveModuleVelocity.optimize against the previous angle
            if abs(wrap(des[m][1] - pa)) > math.pi / 2.0:
                des[m] = [-des[m][0], wrap(des[m][1] + math.pi)]
            ds = des[m][0]
            angle = pa
            if _eps_eq(ps, 0.0) or (ps > 0 and ds >= ps) or (ps < 0 and ds <= ps):
                mt = fwd_t - cfg.torque_loss
                sign = 1
                if ps < 0:
                    angle = wrap(angle + math.pi)
            else:
                mt = rev_t + cfg.torque_loss
                sign = -1
                if ps > 0:
                    angle = wrap(angle + math.pi)
            mt = min(mt, cfg.max_torque_friction)
            f = mt / cfg.wheel_radius
            mfx, mfy = f * sign * math.cos(angle), f * sign * math.sin(angle)
            fx += mfx
            fy += mfy
            if not _eps_eq(0.0, math.hypot(mfx, mfy)):
                mx, my = cfg.mods[m]
                to_mod = math.atan2(my, mx) if (mx or my) else 0.0
                theta = wrap(math.atan2(mfy, mfx) - to_mod)
                torque += f * cfg.pivot[m] * math.sin(theta)
        ax, ay = fx / cfg.mass, fy / cfg.mass
        alpha = torque / cfg.moi
        if lim is not None:
            a = math.hypot(ax, ay)
            if a > lim[1]:
                ax, ay = ax * lim[1] / a, ay * lim[1] / a
            alpha = max(-lim[3], min(lim[3], alpha))
        acc = cfg.to_module_states((ax, ay, alpha))
        for m in range(n):
            if min_s == 0.0:
                break
            step = abs(acc[m][0] * dt)
            vx_s = des_vx[m] if min_s == 1.0 else (des_vx[m] - prev_vx[m]) * min_s + prev_vx[m]
            vy_s = des_vy[m] if min_s == 1.0 else (des_vy[m] - prev_vy[m]) * min_s + prev_vy[m]
            min_s = min(min_s, _drive_max_s(prev_vx[m], prev_vy[m], vx_s, vy_s, step))

        ret = (prev.speeds[0] + min_s * dx, prev.speeds[1] + min_s * dy, prev.speeds[2] + min_s * dth)
        ret = _discretize(ret, dt)
        states = [list(s) for s in cfg.to_module_states(ret)]
        for m in range(n):
            if override[m] is not None:
                if _flip(override[m] - states[m][1]):
                    states[m][0] *= -1.0
                states[m][1] = override[m]
            if _flip(states[m][1] - prev.states[m][1]):
                states[m] = [-states[m][0], wrap(states[m][1] + math.pi)]
        return Setpoint(ret, states)


def _steering_max_s(x0, y0, th0, x1, y1, th1, max_dev):
    d = th1 - th0
    if d > math.pi:
        th1 -= 2.0 * math.pi
    elif d < -math.pi:
        th1 += 2.0 * math.pi
    diff = th1 - th0
    if abs(diff) <= max_dev:
        return 1.0
    target = th0 + math.copysign(max_dev, diff)
    s, c = math.sin(-target), math.cos(-target)
    h0 = s * x0 + c * y0
    h1 = s * x1 + c * y1
    return h0 / (h0 - h1)


def _valid_s(s):
    return math.isfinite(s) and 0.0 <= s <= 1.0


def _drive_max_s(x0, y0, x1, y1, step):
    l0, l1 = x0 * x0 + y0 * y0, x1 * x1 + y1 * y1
    r0 = math.sqrt(l0)
    diff = math.sqrt(l1) - r0
    if abs(diff) <= step:
        return 1.0
    target = r0 + math.copysign(step, diff)
    p = x0 * x1 + y0 * y1
    a, b, c = l0 + l1 - 2 * p, 2 * (p - l0), l0 - target * target
    root = math.sqrt(max(0.0, b * b - 4 * a * c))
    s1 = (-b + root) / (2 * a)
    if _valid_s(s1):
        return s1
    s2 = (-b - root) / (2 * a)
    if _valid_s(s2):
        return s2
    return 1.0


# ============================================================================================ follower
def _modulus(x, lo, hi):
    m = hi - lo
    x -= math.trunc((x - lo) / m) * m
    x -= math.trunc((x - hi) / m) * m
    return x


class _Pid:
    """WPILib PIDController, P only (as Path2PathFollower uses it)."""

    def __init__(self, kp, continuous=False):
        self.kp, self.continuous = kp, continuous
        self.tol, self.err, self.prev, self.vel_err, self.has = 0.05, 0.0, 0.0, 0.0, False

    def reset(self):
        self.err = self.prev = self.vel_err = 0.0
        self.has = False

    def calculate(self, measurement, setpoint):
        self.err = setpoint - measurement
        if self.continuous:
            self.err = _modulus(self.err, -math.pi, math.pi)
        self.vel_err = (self.err - self.prev) / PERIOD if self.has else 0.0
        self.prev = self.err
        self.has = True
        return self.kp * self.err

    @property
    def at_setpoint(self):
        return self.has and abs(self.err) < self.tol


def _trapezoid(t, cur, goal, vmax, amax):
    """WPILib TrapezoidProfile.calculate(t, current, goal) -> (position, velocity)."""
    direction = -1.0 if cur[0] > goal[0] else 1.0
    cur = (cur[0] * direction, cur[1] * direction)
    goal = (goal[0] * direction, goal[1] * direction)
    cur = (cur[0], max(-vmax, min(vmax, cur[1])))
    cb = cur[1] / amax
    cdb = cb * cb * amax / 2.0
    ce = goal[1] / amax
    cde = ce * ce * amax / 2.0
    full = cdb + (goal[0] - cur[0]) + cde
    acc_t = vmax / amax
    full_speed = full - acc_t * acc_t * amax
    if full_speed < 0.0:
        acc_t = math.sqrt(max(0.0, full) / amax)
        full_speed = 0.0
    end_acc = acc_t - cb
    end_full = end_acc + full_speed / vmax
    end_dec = end_full + acc_t - ce
    if t < end_acc:
        r = (cur[0] + (cur[1] + t * amax / 2.0) * t, cur[1] + t * amax)
    elif t < end_full:
        r = (cur[0] + (cur[1] + vmax) / 2.0 * end_acc + vmax * (t - end_acc), vmax)
    elif t <= end_dec:
        left = end_dec - t
        v = goal[1] + left * amax
        r = (goal[0] - (goal[1] + v) / 2.0 * left, v)
    else:
        r = goal
    return (r[0] * direction, r[1] * direction)


class _ProfiledPid:
    def __init__(self, kp, vmax, amax):
        self.pid = _Pid(kp)
        self.vmax, self.amax = vmax, amax
        self.setpoint = (0.0, 0.0)

    def reset(self, measurement, velocity):
        self.setpoint = (measurement, velocity)
        self.pid.reset()

    def calculate(self, measurement, goal):
        goal = (_modulus(goal - measurement, -math.pi, math.pi) + measurement, 0.0)
        self.setpoint = (_modulus(self.setpoint[0] - measurement, -math.pi, math.pi) + measurement, self.setpoint[1])
        self.setpoint = _trapezoid(PERIOD, self.setpoint, goal, self.vmax, self.amax)
        return self.pid.calculate(measurement, self.setpoint[0])

    @property
    def at_setpoint(self):
        return self.pid.at_setpoint


def _progress(a, b, p):
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    if l2 < 1e-6:
        return 0.0
    return max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))


class Path2Follower:
    """Path2PathFollower. waypoints: dicts {xy, rot (or None), vmax, handoff, wmax, alpha_max}."""

    def __init__(self, waypoints, end_distance, end_angle, pose, robot_speeds, target_first=False, gains=(4.0, 2.0, 5.0)):
        self.wps = waypoints
        self.idx = 0 if target_first or len(waypoints) == 1 else 1
        self.seg_start = pose[:2] if target_first else waypoints[0]["xy"]
        self.seg_end = waypoints[self.idx]["xy"]
        w = waypoints[self.idx]
        kp, kp_ct, kp_rot = gains
        self.rot = _ProfiledPid(kp_rot, w["wmax"], w["alpha_max"])
        self.rot.pid.continuous = True
        self.rot.pid.tol = end_angle
        self.rot.reset(pose[2], robot_speeds[2])
        self.trans = _Pid(kp)
        self.trans.tol = end_distance
        self.ct = _Pid(kp_ct)
        self.ct.tol = end_distance
        self.held = None
        self._update_held(pose[2])

    @property
    def finished(self):
        return self.idx == len(self.wps) - 1 and self.trans.at_setpoint and self.rot.at_setpoint

    def calculate(self, pose):
        """-> robot-relative request."""
        self._advance(pose)
        w = self.wps[self.idx]
        self.rot.vmax, self.rot.amax = w["wmax"], w["alpha_max"]
        remaining = 0.0
        cur = pose[:2]
        for q in self.wps[self.idx:]:
            remaining += math.dist(cur, q["xy"])
            cur = q["xy"]
        to = (self.seg_end[0] - pose[0], self.seg_end[1] - pose[1])
        ang = math.atan2(to[1], to[0]) if (to[0] or to[1]) else 0.0
        out = -self.trans.calculate(remaining, 0.0)
        out = max(-w["vmax"], min(w["vmax"], out))
        vx, vy = out * math.cos(ang), out * math.sin(ang)
        ct = -self.ct.calculate(self._cross_track(pose), 0.0)
        perp = ang - math.pi / 2.0
        vx += ct * math.cos(perp)
        vy += ct * math.sin(perp)
        omega = self.rot.calculate(pose[2], self._heading_target(pose))
        return _to_robot((vx, vy, omega), pose[2])

    def _advance(self, pose):
        if self.idx >= len(self.wps) - 1:
            return
        target = self.wps[self.idx]
        seg = math.dist(self.seg_start, target["xy"])
        prog = _progress(self.seg_start, target["xy"], pose[:2])
        thresh = max(0.0, min(1.0, 1.0 - target["handoff"] / seg)) if seg >= 1e-6 else 1.0
        close = math.dist(target["xy"], pose[:2]) <= target["handoff"]
        if (seg >= 1e-6 and prog > thresh) or close:
            self.idx += 1
            self.seg_start = self.seg_end
            self.seg_end = self.wps[self.idx]["xy"]
            self._update_held(pose[2])

    def _heading_target(self, pose):
        for q in self.wps[self.idx:]:
            if q["rot"] is not None:
                return q["rot"]
        if self.held is None:
            self.held = pose[2]
        return self.held

    def _update_held(self, heading):
        future = any(q["rot"] is not None for q in self.wps[self.idx:])
        self.held = None if future else heading

    def _cross_track(self, pose):
        a, b = self.seg_start, self.seg_end
        t = _progress(a, b, pose[:2])
        c = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        cross = (b[0] - a[0]) * (pose[1] - a[1]) - (b[1] - a[1]) * (pose[0] - a[0])
        e = math.dist(pose[:2], c)
        return -e if cross < 0 else e


# ============================================================================================ authoring
def to_path2(course_or_legs, limits: Limits = Limits(), handoff=DEFAULT_HANDOFF):
    """The neutral course (or chain of legs) as path2 paths. Each waypoint is a node: a pose waypoint where the
    course sets a heading, else a translation waypoint; the shared max speed and spin limits on every one; the
    default distance transitions. A chain becomes one path through all its legs' waypoints, split only where a
    leg ends at rest (path2 has no rolling ends: inside a path every handoff rolls)."""
    legs = course_or_legs if isinstance(course_or_legs, (list, tuple)) else [course_or_legs]
    paths, cur = [], []
    for k, leg in enumerate(legs):
        wps = leg.waypoints if not cur else leg.waypoints[1:]
        for w in wps:
            cur.append({"xy": (w.x, w.y), "rot": None if w.heading_deg is None else wrap(math.radians(w.heading_deg)),
                        "vmax": limits.max_speed, "handoff": handoff, "wmax": limits.max_omega,
                        "alpha_max": limits.max_alpha})
        if leg.end_speed == 0.0 or k == len(legs) - 1:
            cur[-1]["handoff"] = 0.0
            paths.append(cur)
            cur = []
    if fractional_targets(legs):
        raise ValueError("path2 can't hold a heading target between waypoints")
    return {"paths": paths, "end_distance": DEFAULT_END_DISTANCE, "end_angle": math.radians(DEFAULT_END_DEGREES)}


def path2_corner_caps(wps, max_accel):
    """Caps each interior waypoint's max velocity at the corner's: the robot hands off within h of it and turns
    by theta, an arc of radius max(h, h / tan(theta / 2)), so sqrt(a r) (BLine-Web's corner rule, safety 1).
    The cap goes on the corner and on the waypoint after it (the approach and the way out)."""
    caps = [w["vmax"] for w in wps]
    for k in range(1, len(wps) - 1):
        (x0, y0), (x1, y1), (x2, y2) = wps[k - 1]["xy"], wps[k]["xy"], wps[k + 1]["xy"]
        l0, l1 = math.hypot(x1 - x0, y1 - y0), math.hypot(x2 - x1, y2 - y1)
        if l0 <= 1e-9 or l1 <= 1e-9:
            continue
        dot = max(-1.0, min(1.0, ((x1 - x0) * (x2 - x1) + (y1 - y0) * (y2 - y1)) / (l0 * l1)))
        turn = math.acos(dot)
        if turn < 1e-4:
            continue
        h = max(wps[k]["handoff"], 0.05)
        r = max(h, h / math.tan(turn / 2)) if turn < math.pi - 1e-6 else h
        cap = min(wps[k]["vmax"], max(0.2, math.sqrt(max_accel * r)))
        caps[k] = min(caps[k], cap)
        caps[k + 1] = min(caps[k + 1], cap)
    for w, c in zip(wps, caps):
        w["vmax"] = c


def fractional_targets(legs):
    return [h for leg in legs for h in leg.heading_targets if h.pos != int(h.pos)]


def robot_config(cof=None):
    return RobotConfig() if cof is None else RobotConfig(wheel_cof=cof)


def plan_time(native, accel_cap=None, limits: Limits = Limits(), cfg=None, gains=(4.0, 2.0, 5.0)):
    """The GUI's time for the path(s): Path2Simulator from rest at each path's first waypoint (its heading),
    the setpoint generator at the GUI's 10 rotations per second steering, summed over the paths. None if a
    path doesn't finish within 120 s."""
    cfg = cfg or robot_config()
    cons = None if accel_cap is None else (limits.max_speed, accel_cap, limits.max_omega, limits.max_alpha)
    gen = SetpointGenerator(cfg, GUI_STEER_RATE, cons)
    total = 0.0
    for wps in native["paths"]:
        first = wps[0]
        pose = (first["xy"][0], first["xy"][1], first["rot"] or 0.0)
        f = Path2Follower(wps, native["end_distance"], native["end_angle"], pose, (0.0, 0.0, 0.0), gains=gains)
        sp = gen.at((0.0, 0.0, 0.0))
        for step in range(1, 6001):
            sp = gen.generate(sp, f.calculate(pose))
            pose = _integrate(pose, sp.speeds, PERIOD)
            if f.finished:
                total += step * PERIOD
                break
        else:
            return None
    return total


class PathPlanner2(Follower):
    """The path2 follower on the robot: Path2PathFollower -> PathPlannerLib's SwerveSetpointGenerator (the
    plant's 25 rad/s steering; `accel_cap` m/s^2 as PathConstraints, None = the robot's own limits) -> field
    speeds. A chain of paths: the next starts when one finishes, targeting its second waypoint."""

    def __init__(self, accel_cap=None, handoff=DEFAULT_HANDOFF, end_distance=DEFAULT_END_DISTANCE,
                 end_degrees=DEFAULT_END_DEGREES, kp=4.0, kp_ct=2.0, kp_rot=5.0, limits: Limits = Limits(), cof=None,
                 corner_caps=False):
        """corner_caps: each waypoint's max velocity set to what its corner allows, as a team drawing a path2
        path would (path2 waypoints have no curves, so the robot must slow to turn through a sharp one)."""
        self.accel_cap, self.handoff, self.lim = accel_cap, handoff, limits
        self.corner_caps = corner_caps
        self.gains = (kp, kp_ct, kp_rot)
        self.end_distance, self.end_angle = end_distance, math.radians(end_degrees)
        self.cfg = robot_config(cof)
        cons = None if accel_cap is None else (limits.max_speed, accel_cap, limits.max_omega, limits.max_alpha)
        self.gen = SetpointGenerator(self.cfg, RP.steer_max_rate, cons)

    def adapt(self, course):
        native = to_path2(course, self.lim, self.handoff)
        native["end_distance"], native["end_angle"] = self.end_distance, self.end_angle
        if self.corner_caps:
            for wps in native["paths"]:
                path2_corner_caps(wps, self.accel_cap or self.lim.max_accel)
        return native

    @staticmethod
    def geometry(native):
        import numpy as np

        return np.array([w["xy"] for wps in native["paths"] for w in wps], float)

    def reset(self, native, start):
        self.native = native
        self.k = 0
        self.finished = False
        self._start_path(start.pose, start.vel)

    def _start_path(self, pose, field_vel):
        robot = _to_robot(field_vel, pose[2])
        self.f = Path2Follower(self.native["paths"][self.k], self.native["end_distance"], self.native["end_angle"],
                               pose, robot, gains=self.gains)
        self.sp = self.gen.at(robot)

    def step(self, t, pose, vel):
        if self.finished:
            return (0.0, 0.0, 0.0)
        if self.f.finished:  # the last command ended last loop; the next one starts now
            self.k += 1
            self._start_path(pose, vel)
        self.sp = self.gen.generate(self.sp, self.f.calculate(pose))
        if self.f.finished and self.k == len(self.native["paths"]) - 1:
            self.finished = True  # the command group ends this loop
        return _to_field(self.sp.speeds, pose[2])

    def done(self):
        return self.finished
