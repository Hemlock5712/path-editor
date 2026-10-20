"""Race four ways to drive a swerve robot to one pose in the alignsim plant.

    python race.py            # everything: results.json + results.md   (~ a few minutes)
    python race.py --quick    # 2 seeds, fewer scenarios (smoke test)

Every controller has the same interface as the sim's bench: reset(pose, meas_vel) and
step(pose, goal, meas_vel, dt) -> field-frame (vx, vy, w) command, called at 50 Hz.
All methods drive the SAME plant (alignsim.plant.SwervePlant: 500 Hz physics, Kraken FOC curve,
stick/slip tires, steering lag, 250 Hz odometry thread, 50 Hz loop with jitter, 2 ms actuation delay).
Ports are listed with their source in results.md.
"""

import json
import math
import multiprocessing as mp
import os
import sys
from dataclasses import replace
from functools import partial

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from alignsim.controllers import TrapezoidProfile  # noqa: E402
from alignsim.geom import rot, wrap  # noqa: E402
from alignsim.odometry import SwerveOdometry  # noqa: E402
from alignsim.params import LoopParams, RobotParams  # noqa: E402
from alignsim.plant import SwervePlant  # noqa: E402

RP = RobotParams()
BASE_R = math.hypot(*RP.module_positions[0])
T_END = 8.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# =============================================================================================
# 1. OURS: DriveToPose.java + AccelerationLimiter.java (record/v2-code, record/v1-code), line for line
# =============================================================================================
class AccelerationLimiter:
    """Port of AccelerationLimiter.limit(). Motor numbers from the Java file; gear ratio and wheel
    radius come from the sim's RobotParams (the real TunerConstants are not available here)."""

    MU, G, MASS = 1.0, 9.81, 60.0
    TRACTION = MU * G
    MODEL_CURRENT = 150.0
    ANGULAR_ACCEL = 8.0
    # DCMotor(12 V, 9.362 N m stall, 476.1 A stall, 0 A free, 5785 rpm, 1 motor)
    KT = 9.362 / 476.1
    R = 12.0 / 476.1
    KV = (5785 * 2 * math.pi / 60) / 12.0

    @classmethod
    def motor_accel(cls, wheel_speed):
        motor_speed = wheel_speed / RP.wheel_radius * RP.drive_gear
        amps = clamp(-motor_speed / (cls.KV * cls.R) + 12.0 / cls.R, 0.0, cls.MODEL_CURRENT)
        force = cls.KT * amps * RP.drive_gear / RP.wheel_radius
        return 4 * force / cls.MASS

    @classmethod
    def limit(cls, cur, want, dt, traction=None):
        traction = cls.TRACTION if traction is None else traction
        ax, ay, al = (want[0] - cur[0]) / dt, (want[1] - cur[1]) / dt, (want[2] - cur[2]) / dt
        al = clamp(al, -cls.ANGULAR_ACCEL, cls.ANGULAR_ACCEL)
        if ax * cur[0] + ay * cur[1] > 0:  # speeding up
            ws = math.hypot(cur[0], cur[1]) + abs(cur[2]) * BASE_R
            lim = cls.motor_accel(ws)
            drive = math.hypot(ax, ay)
            if drive > lim:
                ax *= lim / drive
                ay *= lim / drive
        total = math.hypot(math.hypot(ax, ay), al * BASE_R)
        if total > traction:
            k = traction / total
            ax, ay, al = ax * k, ay * k, al * k
        return (cur[0] + ax * dt, cur[1] + ay * dt, cur[2] + al * dt)


class Ours:
    """DriveToPose.execute(): wantedSpeed/wantedSpin, then AccelerationLimiter from last command."""

    MAX_SPEED, DECEL, REACTION, EASE_IN, DEADBAND = 4.0, 6.0, 0.050, 6.0, 0.003
    MAX_OMEGA, ANG_DECEL, EASE_IN_H, H_REACTION, H_DEADBAND = 2 * math.pi, 2 * math.pi, 5.0, 0.050, math.radians(0.3)

    def __init__(self, traction=None, decel=None):
        self.traction = traction  # None = AccelerationLimiter.TRACTION (MU 1.0 -> 9.81)
        if decel is not None:
            self.DECEL = decel

    def reset(self, pose, vel):
        self.cmd = tuple(vel)

    def step(self, pose, goal, vel, dt):
        ex, ey = goal[0] - pose[0], goal[1] - pose[1]
        d = math.hypot(ex, ey)
        eth = wrap(goal[2] - pose[2])
        now = math.hypot(self.cmd[0], self.cmd[1])
        if d < self.DEADBAND:
            speed = 0.0
        else:
            brake = max(0.0, d - now * self.REACTION)
            speed = min(self.MAX_SPEED, math.sqrt(2 * self.DECEL * brake), self.EASE_IN * d)
        vx, vy = (ex / d * speed, ey / d * speed) if d > 1e-6 else (0.0, 0.0)
        a = abs(eth)
        if a < self.H_DEADBAND:
            om = 0.0
        else:
            braking = max(0.0, a - abs(self.cmd[2]) * self.H_REACTION)
            om = math.copysign(min(self.MAX_OMEGA, math.sqrt(2 * self.ANG_DECEL * braking), self.EASE_IN_H * a), eth)
        self.cmd = AccelerationLimiter.limit(self.cmd, (vx, vy, om), dt, self.traction)
        return self.cmd


class WithLimiter:
    """Variant 'same limiter': feed any method's output through OUR AccelerationLimiter, seeded from
    the measured velocity and then from its own last output (exactly how DriveToPose uses it)."""

    def __init__(self, inner):
        self.inner = inner

    def reset(self, pose, vel):
        self.inner.reset(pose, vel)
        self.cmd = tuple(vel)

    def step(self, pose, goal, vel, dt):
        want = self.inner.step(pose, goal, vel, dt)
        self.cmd = AccelerationLimiter.limit(self.cmd, want, dt)
        return self.cmd


# =============================================================================================
# 2. CTRE LinearPath (Phoenix 6 26.70.0-alpha-2, swerve/utility/LinearPath.java, from the sources jar)
# =============================================================================================
class CtreLinearPath:
    """Port of LinearPath.calculate(t, current, goal): setState() + calculate(t). No feedback inside.

    mode 'meas': 'current' is the MEASURED pose/velocity each loop (re-plans from reality, 50 Hz).
    mode 'sp'  : 'current' is the previous returned setpoint (the Javadoc's
                 `current = path.calculate(dt, current, target)` loop). kp>0 adds P on pose error
                 (what you must add yourself for any feedback; CTRE ships none).
    """

    def __init__(self, mode="meas", vmax=4.0, amax=6.0, wmax=2 * math.pi, walpha=2 * math.pi, kp=0.0, kp_w=0.0):
        self.mode, self.kp, self.kp_w = mode, kp, kp_w
        self.lin = TrapezoidProfile(vmax, amax)
        self.ang = TrapezoidProfile(wmax, walpha)

    def _calc(self, t, cur_pose, cur_vel, goal):
        tx, ty = goal[0] - cur_pose[0], goal[1] - cur_pose[1]
        dist = math.hypot(tx, ty)
        hd = math.atan2(ty, tx) if dist > 0 else 0.0  # Translation2d.getAngle().orElse(ZERO)
        ch, sh = math.cos(hd), math.sin(hd)
        v = cur_vel[0] * ch + cur_vel[1] * sh
        lp, lv = self.lin.calculate(t, (0.0, v), (dist, 0.0))
        th0 = cur_pose[2]
        a_goal = th0 + wrap(goal[2] - th0)
        ap, av = self.ang.calculate(t, (th0, cur_vel[2]), (a_goal, 0.0))
        return ((cur_pose[0] + lp * ch, cur_pose[1] + lp * sh, ap), (lv * ch, lv * sh, av))

    def reset(self, pose, vel):
        self.sp = (tuple(pose), tuple(vel))

    def step(self, pose, goal, vel, dt):
        if self.mode == "meas":
            _, v = self._calc(dt, pose, vel, goal)
            return v
        # note: the setpoint heading is unwrapped; compare via wrap()
        p, v = self._calc(dt, self.sp[0], self.sp[1], goal)
        self.sp = (p, v)
        if self.kp == 0 and self.kp_w == 0:
            return v
        return (v[0] + self.kp * (p[0] - pose[0]), v[1] + self.kp * (p[1] - pose[1]),
                v[2] + self.kp_w * wrap(p[2] - pose[2]))


# =============================================================================================
# 3. BLine-Lib (edanliahovetsky/BLine-Lib, branch wpilib-2027, commit 4402250, 2027 beta.3)
#    Follower.execute + TranslationGuidance + TranslationProgress + RotationProgress +
#    HolonomicController + HolonomicRateLimiter, for ONE straight segment to one goal pose.
# =============================================================================================
class BLine:
    def __init__(self, kp=5.0, kp_ct=2.0, kp_w=3.0, vmax=4.0, amax=6.0, wmax_deg=720.0, walpha_deg=1500.0,
                 tol=0.03, tol_deg=2.0, vmin=0.0):
        self.kp, self.kp_ct, self.kp_w = kp, kp_ct, kp_w
        self.vmax, self.amax = vmax, amax
        self.wmax, self.walpha = math.radians(wmax_deg), math.radians(walpha_deg)
        self.tol, self.tol_rad, self.vmin = tol, math.radians(tol_deg), vmin

    def reset(self, pose, vel):
        self.start = (pose[0], pose[1])
        self.th0 = pose[2]
        self.cmd = tuple(vel)  # HolonomicController(measured field velocity)
        self.s_acc = 0.0
        self.goal = None
        self.finished = False

    @staticmethod
    def _limit(want, last, dt, vmax, wmax, amax, walpha):
        vx, vy, om = want
        sp = math.hypot(vx, vy)
        if sp > vmax:
            vx, vy = vx * vmax / sp, vy * vmax / sp
        om = clamp(om, -wmax, wmax)
        da = math.hypot(vx - last[0], vy - last[1]) / dt
        acc = clamp(da, 0, amax)
        th = math.atan2(vy - last[1], vx - last[0])
        oa = clamp((om - last[2]) / dt, -walpha, walpha)
        return (last[0] + math.cos(th) * acc * dt, last[1] + math.sin(th) * acc * dt, last[2] + oa * dt)

    def step(self, pose, goal, vel, dt):
        if self.goal is None:
            self.goal = goal  # path is fixed at command creation: start -> goal
        if self.finished:
            return (0.0, 0.0, 0.0)
        g = self.goal
        sx, sy = self.start
        L = math.hypot(g[0] - sx, g[1] - sy)
        remaining = math.hypot(g[0] - pose[0], g[1] - pose[1])
        # RotationProgress: heading target ramps linearly with progress along the segment (profiled)
        th_goal = self.th0 + wrap(g[2] - self.th0)
        if L > 1e-9:
            u = clamp(((pose[0] - sx) * (g[0] - sx) + (pose[1] - sy) * (g[1] - sy)) / (L * L), 0, 1)
            self.s_acc = max(self.s_acc, u)
            frac = 1.0 if self.s_acc >= 1 - 1e-9 else self.s_acc
        else:
            frac = 1.0
        at_pos = remaining <= self.tol
        target_rot = th_goal if at_pos else self.th0 + frac * (th_goal - self.th0)
        # TranslationGuidance
        bearing = math.atan2(g[1] - pose[1], g[0] - pose[0])
        speed = clamp(self.kp * remaining, -self.vmax, self.vmax)
        if self.vmin > 0 and not at_pos and abs(speed) < self.vmin:
            speed = self.vmin
        if L > 1e-6:
            nx, ny = -(g[1] - sy) / L, (g[0] - sx) / L
            err = (pose[0] - sx) * nx + (pose[1] - sy) * ny
            corr = self.kp_ct * err
        else:
            nx = ny = corr = 0.0
        vx = speed * math.cos(bearing) - corr * nx
        vy = speed * math.sin(bearing) - corr * ny
        if at_pos:
            vx = vy = 0.0
        err_rot = wrap(wrap(target_rot) - pose[2])
        om = clamp(self.kp_w * err_rot, -self.wmax, self.wmax)
        at_head = abs(wrap(th_goal - pose[2])) <= self.tol_rad
        if at_pos and abs(err_rot) <= self.tol_rad:
            om = 0.0
        out = self._limit((vx, vy, om), self.cmd, dt, self.vmax, self.wmax, self.amax, self.walpha)
        if at_pos:  # HolonomicController.limit: zero AFTER the limiter
            out = (0.0, 0.0, 0.0 if at_head else out[2])
        self.cmd = out
        if at_pos and at_head:  # Follower.isFinished -> command ends, motors told zero
            self.finished = True
            return (0.0, 0.0, 0.0)
        return out


# =============================================================================================
# 4. PathPlanner (mjansen4857/pathplanner, branch 2027, commit 7fbd798): FollowPathCommand +
#    PPHolonomicDriveController (P on x/y/theta + trajectory feedforward). Trajectory generator
#    re-implemented for one straight, constraint-limited segment (see results.md for approximations).
# =============================================================================================
class PathPlanner:
    DS = 0.05  # PathPlannerPath resolution

    def __init__(self, kp=5.0, kp_w=5.0, vmax=4.0, amax=6.0, wmax_deg=540.0, walpha_deg=720.0, hold=False):
        self.kp, self.kp_w, self.hold = kp, kp_w, hold
        self.vmax, self.amax = vmax, amax
        self.wmax = math.radians(wmax_deg)

    def reset(self, pose, vel):
        self.pose0, self.vel0 = tuple(pose), tuple(vel)
        self.traj = None
        self.t = 0.0
        self.done = False

    def _generate(self, goal):
        sx, sy, th0 = self.pose0
        L = math.hypot(goal[0] - sx, goal[1] - sy)
        n = max(2, int(math.ceil(L / self.DS - 1e-9)) + 1)
        ds = L / (n - 1)
        hd = math.atan2(goal[1] - sy, goal[0] - sx) if L > 1e-9 else 0.0
        dth = wrap(goal[2] - th0)
        theta = [th0 + dth * (1 - math.cos(i / (n - 1) * math.pi)) / 2 for i in range(n)]  # cosine over distance
        dth_ds = [(theta[min(i + 1, n - 1)] - theta[max(i - 1, 0)]) / (ds * (min(i + 1, n - 1) - max(i - 1, 0)))
                  for i in range(n)]
        v0 = math.hypot(self.vel0[0], self.vel0[1])
        v = [0.0] * n
        v[0] = v0

        def cap(i, val):
            val = min(val, self.vmax)
            if abs(dth_ds[i]) > 1e-9:  # desaturate: omega = v * dtheta/ds <= wmax
                val = min(val, self.wmax / abs(dth_ds[i]))
            return val

        for i in range(1, n - 1):  # forward pass
            v[i] = cap(i, math.sqrt(v[i - 1] ** 2 + 2 * self.amax * ds))
        v[n - 1] = 0.0
        for i in range(n - 2, 0, -1):  # reverse pass
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * self.amax * ds))
        t = [0.0] * n
        for i in range(1, n):
            sv = v[i] + v[i - 1]
            t[i] = t[i - 1] + (2 * ds / sv if sv > 1e-6 else 0.0)
        states = []
        for i in range(n):
            x, y = sx + math.cos(hd) * ds * i, sy + math.sin(hd) * ds * i
            vel = (v[i] * math.cos(hd), v[i] * math.sin(hd), v[i] * dth_ds[i])
            if i == 0:
                vel = self.vel0  # state 0 carries the robot's actual field speeds
            states.append((t[i], x, y, theta[i], vel))
        return states

    def _sample(self, t):
        st = self.traj
        if t <= st[0][0]:
            s = st[0]
            return s[1], s[2], s[3], s[4]
        if t >= st[-1][0]:
            s = st[-1]
            return s[1], s[2], s[3], s[4]
        lo, hi = 1, len(st) - 1
        while lo != hi:
            mid = (lo + hi) // 2
            if st[mid][0] < t:
                lo = mid + 1
            else:
                hi = mid
        a, b = st[lo - 1], st[lo]
        if abs(b[0] - a[0]) < 1e-3:
            return b[1], b[2], b[3], b[4]
        u = (t - a[0]) / (b[0] - a[0])
        li = lambda p, q: p + (q - p) * u  # noqa: E731
        return (li(a[1], b[1]), li(a[2], b[2]), li(a[3], b[3]), tuple(li(p, q) for p, q in zip(a[4], b[4])))

    def step(self, pose, goal, vel, dt):
        if self.traj is None:
            self.traj = self._generate(goal)
            self.total = self.traj[-1][0]
        t = self.t
        self.t += dt
        if t >= self.total:
            if not self.hold or self.done:  # FollowPathCommand.end(): output zero speeds, command over
                return (0.0, 0.0, 0.0)
            # 'final hold': keep running PPHolonomicDriveController on the final state (FF 0, same P gains)
            # until the controller's own pose is within 2 cm / 2 deg and the measured speed is < 5 cm/s
            ex, ey, eth = goal[0] - pose[0], goal[1] - pose[1], wrap(goal[2] - pose[2])
            if math.hypot(ex, ey) < 0.02 and abs(eth) < math.radians(2) and math.hypot(vel[0], vel[1]) < 0.05:
                self.done = True
                return (0.0, 0.0, 0.0)
            return (self.kp * ex, self.kp * ey, self.kp_w * eth)
        x, y, th, ff = self._sample(t)
        return (ff[0] + self.kp * (x - pose[0]), ff[1] + self.kp * (y - pose[1]),
                ff[2] + self.kp_w * wrap(th - pose[2]))


# =============================================================================================
# Methods and scenarios
# =============================================================================================
METHODS = {  # name -> factory building the 'as shipped' controller
    "Ours (DriveToPose)": Ours,
    "CTRE LinearPath (re-plan from measured)": partial(CtreLinearPath, mode="meas"),
    "CTRE LinearPath (setpoint loop, FF only)": partial(CtreLinearPath, mode="sp"),
    "CTRE LinearPath (setpoint loop + P5)": partial(CtreLinearPath, mode="sp", kp=5.0, kp_w=5.0),
    "BLine (README gains)": BLine,
    "PathPlanner (kP 5)": PathPlanner,
}
OURS = "Ours (DriveToPose)"

GRIP = {  # (mu static, mu kinetic, wheel-radius error sigma, gyro scale error sigma)
    "normal": (RP.mu_static, RP.mu_kinetic, 0.01, 0.002),
    "low": (0.70, 0.61, 0.01, 0.002),
    "normal_cal": (RP.mu_static, RP.mu_kinetic, 0.0, 0.0),  # perfectly calibrated wheels: only slip corrupts odometry
    "low_cal": (0.70, 0.61, 0.0, 0.0),
}
S2 = math.sqrt(0.5)
SCENARIOS = {  # name -> (goal pose, start field velocity); start pose is (0, 0, 0)
    "straight 1 m": ((1.0, 0.0, 0.0), (0, 0, 0)),
    "straight 2 m": ((2.0, 0.0, 0.0), (0, 0, 0)),
    "straight 3 m": ((3.0, 0.0, 0.0), (0, 0, 0)),
    "straight 4.5 m": ((4.5, 0.0, 0.0), (0, 0, 0)),
    "straight 6 m": ((6.0, 0.0, 0.0), (0, 0, 0)),
    "sideways 3 m": ((0.0, 3.0, 0.0), (0, 0, 0)),
    "diagonal 3 m + 90 deg turn": ((3 * S2, 3 * S2, math.pi / 2), (0, 0, 0)),
    "2 m/s sideways start, goal 2 m ahead": ((2.0, 0.0, 0.0), (0.0, 2.0, 0.0)),
    "small correction 0.3 m": ((0.3, 0.0, 0.0), (0, 0, 0)),
}


def build(method, limiter, params=None):
    ctrl = METHODS[method]() if params is None else params()
    return WithLimiter(ctrl) if (limiter and method != OURS) else ctrl


# =============================================================================================
# One run (same loop timing as alignsim studies/controllers/bench.run_move, plus our own metrics)
# =============================================================================================
def run_one(job):
    scen, method, limiter, seed, pose_src, grip, override = job
    goal, v0 = SCENARIOS[scen]
    rng = np.random.default_rng(seed)
    mu = GRIP[grip]
    rp = replace(RP, wheel_radius_error=rng.normal(0, mu[2]), gyro_scale_error=rng.normal(0, mu[3]),
                 mu_static=mu[0], mu_kinetic=mu[1])
    lp = LoopParams()
    start = (0.0, 0.0, 0.0)
    plant = SwervePlant(rp, start, (v0[0], v0[1], 0.0), rng)
    odo = SwerveOdometry(rp.module_positions, plant.read_gyro(), plant.read_modules(), start)

    def snap():
        mods = plant.module_velocities()
        vxr = sum(s * math.cos(a) for s, a in mods) / 4.0
        vyr = sum(s * math.sin(a) for s, a in mods) / 4.0
        wr = 0.0
        for (s, a), (mx, my) in zip(mods, plant.mods):
            wr += (-my * s * math.cos(a) + mx * s * math.sin(a)) / (mx * mx + my * my)
        v = rot((vxr, vyr), odo.pose[2])
        return plant.pose(), odo.pose, (v[0], v[1], wr / 4.0)

    ctrl = build(method, limiter, override)
    true_p, odo_p, vel = snap()
    ctrl.reset(true_p if pose_src == "truth" else odo_p, vel)
    dt = lp.sim_dt
    t, next_odom, next_main = 0.0, 0.0, lp.main_period
    requests, cur_req = [], (v0[0], v0[1], 0.0)
    last_cmd = (v0[0], v0[1])
    pk_acc = 0.0
    ts, xs, ys, ths, sp = [], [], [], [], []
    wheel_slip_s = 0.0
    for _ in range(int(T_END / dt)):
        if t >= next_odom - 1e-9:
            next_odom += lp.odom_period
            while requests and requests[0][0] <= t:
                cur_req = requests.pop(0)[1]
            vr = rot((cur_req[0], cur_req[1]), -odo.pose[2])
            plant.set_chassis_robot(vr[0], vr[1], cur_req[2])
            odo.update(plant.read_gyro(), plant.read_modules())
            true_p, odo_p, vel = snap()
            ts.append(t)
            xs.append(true_p[0])
            ys.append(true_p[1])
            ths.append(plant.th_unwrapped)
            sp.append(math.hypot(plant.vx, plant.vy))
            wheel_slip_s += sum(1 for x in plant.slipping if x) * lp.odom_period
        if t >= next_main - 1e-9:
            next_main += lp.main_period + rng.normal(0, lp.main_jitter)
            pose = true_p if pose_src == "truth" else odo_p
            cmd = ctrl.step(pose, goal, vel, lp.main_period)
            pk_acc = max(pk_acc, math.hypot(cmd[0] - last_cmd[0], cmd[1] - last_cmd[1]) / lp.main_period)
            last_cmd = (cmd[0], cmd[1])
            requests.append((t + lp.actuation_delay, tuple(cmd)))
        plant.step(dt)
        t += dt

    ts, xs, ys, ths, sp = map(np.array, (ts, xs, ys, ths, sp))
    ex, ey = goal[0] - xs, goal[1] - ys
    dist = np.hypot(ex, ey)
    eth = np.abs((ths[0] + wrap(goal[2] - ths[0]) if False else goal[2]) - ths)
    eth = np.abs(np.arctan2(np.sin(goal[2] - ths), np.cos(goal[2] - ths)))
    ok = (dist < 0.02) & (eth < math.radians(2)) & (sp < 0.05)
    hold = int(0.25 / lp.odom_period)  # must stay settled 0.25 s
    t_settle = float("nan")
    run = 0
    for i, o in enumerate(ok):
        run = run + 1 if o else 0
        if run >= hold:
            t_settle = float(ts[i - hold + 1])
            break
    in5 = np.flatnonzero(dist < 0.05)
    L = math.hypot(goal[0], goal[1])
    ux, uy = goal[0] / L, goal[1] / L
    over = float(max(0.0, ((xs - goal[0]) * ux + (ys - goal[1]) * uy).max()))
    return {
        "t_settle": t_settle,
        "t_5cm": float(ts[in5[0]]) if len(in5) else float("nan"),
        "t_2cm": float(ts[np.flatnonzero(dist < 0.02)[0]]) if (dist < 0.02).any() else float("nan"),
        "overshoot_cm": 100 * over,
        "err_end_cm": 100 * float(dist[-1]),
        "yaw_end_deg": math.degrees(float(eth[-1])),
        "slip_ms": 1000 * plant.slip_time,
        "wheel_slip_ms": 1000 * wheel_slip_s,
        "err_vs_odo_cm": 100 * math.hypot(odo.pose[0] - goal[0], odo.pose[1] - goal[1]),
        "yaw_vs_odo_deg": abs(math.degrees(wrap(odo.pose[2] - goal[2]))),
        "odo_err_cm": 100 * math.hypot(odo.pose[0] - plant.x, odo.pose[1] - plant.y),
        "pk_acc": pk_acc,
    }


# =============================================================================================
# Driver
# =============================================================================================
def agg(rows):
    out = {"n": len(rows)}
    for k in rows[0]:
        a = np.array([r[k] for r in rows], dtype=float)
        good = a[~np.isnan(a)]
        out[k + "_mean"] = float(good.mean()) if len(good) else None
        out[k + "_worst"] = (float(good.max()) if len(good) else None)
        out[k + "_nan"] = int(np.isnan(a).sum())
    return out


def run_group(pool, jobs, seeds):
    """jobs: list of (key, (scen, method, limiter, pose_src, grip, override)); returns {key: agg}"""
    flat = [(key, (j[0], j[1], j[2], s, j[3], j[4], j[5])) for key, j in jobs for s in seeds]
    res = pool.map(run_one, [f[1] for f in flat], chunksize=4)
    by = {}
    for (key, _), r in zip(flat, res):
        by.setdefault(key, []).append(r)
    return {k: agg(v) for k, v in by.items()}


def main():
    quick = "--quick" in sys.argv
    if "--followup" in sys.argv:  # re-run only the follow-up block into the existing results.json
        out = json.load(open(os.path.join(HERE, "results.json")))
        with mp.get_context("spawn").Pool(max(1, (os.cpu_count() or 2) - 1)) as pool:
            followup(pool, out, list(range(10)))
        json.dump(out, open(os.path.join(HERE, "results.json"), "w"), indent=1)
        write_md(out)
        return
    if "--matched" in sys.argv:  # re-run only the matched-acceleration block into the existing results.json
        out = json.load(open(os.path.join(HERE, "results.json")))
        with mp.get_context("spawn").Pool(max(1, (os.cpu_count() or 2) - 1)) as pool:
            matched(pool, out, list(range(10)))
        json.dump(out, open(os.path.join(HERE, "results.json"), "w"), indent=1)
        write_md(out)
        return
    if "--md-only" in sys.argv:  # regenerate the .md files from results.json (no simulation)
        write_md(json.load(open(os.path.join(HERE, "results.json"))))
        return
    seeds = list(range(2 if quick else 10))
    scens = list(SCENARIOS) if not quick else ["straight 3 m", "small correction 0.3 m"]
    out = {"meta": {"seeds": seeds, "t_end": T_END, "grips": GRIP,
                    "settled": "TRUE pose within 2 cm and 2 deg and true speed < 5 cm/s, held 0.25 s"},
           "runs": {}, "sweeps": {}}
    procs = max(1, (os.cpu_count() or 2) - 1)
    with mp.get_context("spawn").Pool(procs) as pool:
        # main grid: controller sees wheel odometry (what slip corrupts) or the true pose (law only)
        for pose_src, grips in (("odom", ("normal", "low", "normal_cal", "low_cal")), ("truth", ("normal", "low"))):
            for grip in grips:
                for limiter in (False, True):
                    key = f"{pose_src}/{grip}/{'limiter' if limiter else 'shipped'}"
                    out["runs"][key] = {}
                    for scen in scens:
                        jobs = [((scen, m), (scen, m, limiter, pose_src, grip, None)) for m in METHODS
                                if not (limiter and m == OURS)]
                        r = run_group(pool, jobs, seeds)
                        out["runs"][key][scen] = {m: v for (_, m), v in r.items()}
                    print(key, "done", flush=True)

        def sweep(name, variants, grip="normal", pose_src="odom"):
            out["sweeps"][name] = {}
            for scen in scens:
                jobs = [((scen, n), (scen, n, False, pose_src, grip, fac)) for n, fac in variants.items()]
                r = run_group(pool, jobs, seeds)
                out["sweeps"][name][scen] = {m: v for (_, m), v in r.items()}
            print("sweep", name, "done", flush=True)

        # BLine: how much of its result is the README example gain? sweep kP and end tolerance.
        sweep("bline", {f"BLine kP={kp:g} tol={int(tol * 100)}cm": partial(BLine, kp=kp, tol=tol)
                        for kp in (1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0) for tol in (0.03, 0.01)})
        sweep("pp", {"PathPlanner kP=10": partial(PathPlanner, kp=10.0, kp_w=10.0)})
        A = 9.8  # ours really speeds up at ~9.8 m/s^2 (traction) and brakes at 6; the others got 6 above
        sweep("accel_9.8", {
            "CTRE re-plan, accel 9.8": partial(CtreLinearPath, mode="meas", amax=A),
            "CTRE setpoint + P5, accel 9.8": partial(CtreLinearPath, mode="sp", kp=5.0, kp_w=5.0, amax=A),
            "BLine kP=2 tol=1cm, accel 9.8": partial(BLine, kp=2.0, tol=0.01, amax=A),
            "PathPlanner kP=5, accel 9.8": partial(PathPlanner, amax=A)})
        # OURS with the limiter's traction derated (MU 1.0 -> 9.81 is right at the plant's friction limit)
        for g in ("normal", "low", "normal_cal", "low_cal"):
            sweep("ours_" + g, {f"Ours, limiter traction {t:g}": partial(Ours, traction=t) for t in (9.81, 8.0, 7.0, 6.0, 5.0)}, grip=g)
        # tuned BLine / stiffer PP at low grip too
        sweep("bline_low", {f"BLine kP={kp:g} tol=1cm": partial(BLine, kp=kp, tol=0.01) for kp in (1.5, 2.0, 2.5, 3.0)},
              grip="low")
        sweep("pp_low", {"PathPlanner kP=10": partial(PathPlanner, kp=10.0, kp_w=10.0)}, grip="low")
        followup(pool, out, seeds)
        matched(pool, out, seeds)
    with open(os.path.join(HERE, "results.json"), "w") as f:
        json.dump(out, f, indent=1)
    write_md(out)
    print("wrote results.json / results.md / results_tables.md")


FU_SCENS = ["straight 1 m", "straight 3 m", "straight 6 m", "diagonal 3 m + 90 deg turn",
            "2 m/s sideways start, goal 2 m ahead"]


def followup(pool, out, seeds):
    """PathPlanner + final hold (kP 5 and 10), BLine tuned and CTRE setpoint+P5 in perfect-pose AND odometry
    normal-grip runs, as shipped (no extra limiter)."""
    fu = {}
    scens = [x for x in FU_SCENS if x in SCENARIOS]
    bl = {f"BLine kP={kp:g} tol={int(t * 100)}cm": partial(BLine, kp=kp, tol=t)
          for kp in (1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0) for t in (0.03, 0.01)}
    fixed = {"Ours (DriveToPose)": Ours,
             "CTRE LinearPath (setpoint loop + P5)": partial(CtreLinearPath, mode="sp", kp=5.0, kp_w=5.0),
             "PathPlanner (kP 5), no hold (as before)": PathPlanner,
             "PathPlanner (kP 5) + final hold": partial(PathPlanner, hold=True),
             "PathPlanner (kP 10) + final hold": partial(PathPlanner, kp=10.0, kp_w=10.0, hold=True)}
    for pose_src in ("truth", "odom"):
        fu[pose_src] = {}
        for scen in scens:
            variants = dict(fixed)
            variants.update(bl)
            jobs = [((scen, n), (scen, n if n in METHODS else n, False, pose_src, "normal", fac)) for n, fac in variants.items()]
            r = run_group(pool, jobs, seeds)
            rows = {m: v for (_, m), v in r.items()}
            bm = best_of({k: v for k, v in rows.items() if k.startswith("BLine kP")})
            fu[pose_src][scen] = {k: v for k, v in rows.items() if not k.startswith("BLine kP")}
            fu[pose_src][scen][f"BLine tuned ({bm[6:]})"] = rows[bm]
    out["followup"] = fu


def followup_md(out):
    nl = chr(10)
    L = ["## Follow-up: PathPlanner with a final hold, plus BLine tuned and CTRE setpoint+P5 (normal grip, as shipped)", "",
         "'+ final hold' = after the trajectory ends keep running the PathPlanner P controllers (x, y, theta) on the final state with "
         "feedforward 0 until the controller's own pose is within 2 cm / 2 deg and measured speed < 5 cm/s, then output zero. "
         "Accuracy and settled time are on the TRUE pose. 'perfect pose' = controller sees the true pose; 'odometry' = wheel odometry + gyro.", ""]
    for pose, title in (("truth", "Perfect pose (control law only)"), ("odom", "Wheel odometry")):
        L.append(f"### {title}" + nl)
        for scen, rows in out["followup"][pose].items():
            L.append(f"**{scen}**" + nl)
            L.append(HDR)
            for m, a in rows.items():
                L.append(row(m, a))
            L.append("")
    return nl.join(L)


MATCH_SCENS = ["straight 1 m", "straight 3 m", "straight 6 m", "diagonal 3 m + 90 deg turn"]
W8 = 8.0  # rad/s^2 turning cap = AccelerationLimiter ANGULAR_ACCEL; all methods get 1 rot/s max


def matched(pool, out, seeds):
    """Perfect pose, normal grip, as shipped: every method at 4 m/s, accel A, turning 8 rad/s^2 / 2 pi rad/s."""
    res = {}
    wdeg = math.degrees(2 * math.pi)
    for A, label in ((6.0, "accel 6"), (9.8, "accel 9.8")):
        fixed = {
            "Ours (limiter traction %g, DECEL %g)" % (A if A < 9 else 9.81, A):
                partial(Ours, traction=(A if A < 9 else 9.81), decel=A),
            "CTRE setpoint loop + P5": partial(CtreLinearPath, mode="sp", kp=5.0, kp_w=5.0, amax=A, wmax=2 * math.pi, walpha=W8),
            "CTRE re-plan from measured": partial(CtreLinearPath, mode="meas", amax=A, wmax=2 * math.pi, walpha=W8),
            "PathPlanner kP5 + hold": partial(PathPlanner, hold=True, amax=A, wmax_deg=wdeg),
            "PathPlanner kP10 + hold": partial(PathPlanner, kp=10.0, kp_w=10.0, hold=True, amax=A, wmax_deg=wdeg),
        }
        bl = {f"BLine kP={kp:g} tol=1cm": partial(BLine, kp=kp, tol=0.01, amax=A, wmax_deg=wdeg, walpha_deg=math.degrees(W8))
              for kp in (1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0)}
        res[label] = {}
        for scen in MATCH_SCENS:
            variants = dict(fixed)
            variants.update(bl)
            jobs = [((scen, n), (scen, n, False, "truth", "normal", fac)) for n, fac in variants.items()]
            r = run_group(pool, jobs, seeds)
            rows = {m: v for (_, m), v in r.items()}
            bm = best_of({k: v for k, v in rows.items() if k.startswith("BLine kP")})
            res[label][scen] = {k: v for k, v in rows.items() if not k.startswith("BLine kP")}
            res[label][scen][f"BLine tuned ({bm[6:]})"] = rows[bm]
    out["matched"] = res


def matched_md(out):
    nl = chr(10)
    hdr = ("| method | t first within 2 cm | t first within 5 cm | t really settled (n/N) | worst overshoot cm | "
           "final true err cm mean / worst | wheel slip ms |" + nl + "|---|---|---|---|---|---|---|")
    L = ["## Matched acceleration (perfect pose, normal grip, as shipped)", "",
         "Every method: 4.0 m/s, the same acceleration cap, turning 8 rad/s^2 and 2 pi rad/s. Ours: limiter traction set to the cap "
         "(it then speeds up at the cap) and DECEL = cap. CTRE/PathPlanner/BLine get the cap as their acceleration/deceleration. "
         "'first within' = first moment the true position is inside the radius (position only); 'really settled' = within 2 cm and 2 deg, stopped, held 0.25 s.", ""]
    for label, scens in out["matched"].items():
        L.append(f"### {label} m/s^2" + nl)
        for scen, rows in scens.items():
            L.append(f"**{scen}**" + nl)
            L.append(hdr)
            for m, a in rows.items():
                n, ns = a["n"], a["t_settle_nan"]
                ts = "never" if ns == n else fmt(a["t_settle_mean"]) + f" ({n - ns}/{n})"
                L.append(f"| {m} | {fmt(a['t_2cm_mean'])} | {fmt(a['t_5cm_mean'])} | {ts} | {fmt(a['overshoot_cm_worst'], '{:.1f}')} | "
                         f"{fmt(a['err_end_cm_mean'], '{:.1f}')} / {fmt(a['err_end_cm_worst'], '{:.1f}')} | {fmt(a['wheel_slip_ms_mean'], '{:.0f}')} |")
            L.append("")
    return nl.join(L)


def fmt(v, f="{:.2f}"):
    return "-" if v is None else f.format(v)


HDR = ("| method | final err vs TRUE cm mean / worst | true yaw err worst deg | final err vs odometry cm mean | "
       "odometry error at finish cm mean / worst | wheel slip ms (4 wheels summed) | really settled t s (n settled/N) | "
       "t 5 cm | worst overshoot cm |" + chr(10) + "|---|---|---|---|---|---|---|---|---|")


def row(name, a):
    n, ns = a["n"], a["t_settle_nan"]
    ts = "never" if ns == n else fmt(a["t_settle_mean"]) + f" ({n - ns}/{n})"
    return (f"| {name} | {fmt(a['err_end_cm_mean'], '{:.1f}')} / {fmt(a['err_end_cm_worst'], '{:.1f}')} | "
            f"{fmt(a['yaw_end_deg_worst'], '{:.1f}')} | {fmt(a['err_vs_odo_cm_mean'], '{:.1f}')} | "
            f"{fmt(a['odo_err_cm_mean'], '{:.1f}')} / {fmt(a['odo_err_cm_worst'], '{:.1f}')} | "
            f"{fmt(a['wheel_slip_ms_mean'], '{:.0f}')} | {ts} | {fmt(a['t_5cm_mean'])} | "
            f"{fmt(a['overshoot_cm_worst'], '{:.1f}')} |")


def best_of(rows):
    """best sweep variant: settles every seed with the lowest mean settle time, else lowest true final error."""
    ok = [(a["t_settle_mean"], m) for m, a in rows.items() if a["t_settle_nan"] == 0]
    if ok:
        return min(ok)[1]
    return min(rows, key=lambda m: rows[m]["err_end_cm_mean"])


def table(out, cond, scen, with_tuned=True):
    """rows for one scenario under condition cond = 'pose/grip'."""
    sh, li = out["runs"][f"{cond}/shipped"][scen], out["runs"][f"{cond}/limiter"][scen]
    pose, grip = cond.split("/")
    L = [HDR]
    for m, a in sh.items():
        L.append(row(m + (" [same in both]" if m == OURS else ""), a))
    for m, a in li.items():
        L.append(row(m + " + OUR LIMITER", a))
    if with_tuned and pose == "odom":
        osw = out["sweeps"]["ours_" + grip][scen]
        L.append(row("OURS with limiter traction 7 m/s^2 (derated, MU 0.7)", osw["Ours, limiter traction 7"]))
    if with_tuned and pose == "odom" and not grip.endswith("_cal"):
        sw = out["sweeps"]["bline" if grip == "normal" else "bline_low"][scen]
        bm = best_of(sw)
        L.append(row(f"BLine tuned (best of sweep: {bm[6:]})", sw[bm]))
        L.append(row("PathPlanner kP 10", out["sweeps"]["pp" if grip == "normal" else "pp_low"][scen]["PathPlanner kP=10"]))
    return chr(10).join(L)


def write_md(out):
    nl = chr(10)
    full = ["# Full tables (generated by race.py)", ""]
    for cond in ("odom/normal", "odom/low", "odom/normal_cal", "odom/low_cal", "truth/normal", "truth/low"):
        pose, grip = cond.split("/")
        desc = ("controller sees WHEEL ODOMETRY + gyro (no vision), " if pose == "odom" else
                "controller sees the TRUE pose (isolates the control law), ")
        full.append(f"## {desc}{grip} grip (mu static/kinetic {out['meta']['grips'][grip]})" + nl)
        for scen in out["runs"][f"{cond}/shipped"]:
            full.append(f"**{scen}**" + nl)
            full.append(table(out, cond, scen))
            full.append("")
    full.append("## Accel 9.8 variants (odometry, normal grip): ours accelerates ~9.8, others were given 6" + nl)
    for scen, rows in out["sweeps"]["accel_9.8"].items():
        full.append(f"**{scen}**" + nl)
        full.append(HDR)
        full.append(row(OURS, out["runs"]["odom/normal/shipped"][scen][OURS]))
        for m, a in rows.items():
            full.append(row(m, a))
        full.append("")
    full.append("## BLine full sweep (odometry, normal grip)" + nl)
    for scen, rows in out["sweeps"]["bline"].items():
        full.append(f"**{scen}**" + nl)
        full.append(HDR)
        for m, a in rows.items():
            full.append(row(m, a))
        full.append("")
    open(os.path.join(HERE, "results_tables.md"), "w").write(nl.join(full))
    body = []
    hard = ["straight 4.5 m", "straight 6 m", "diagonal 3 m + 90 deg turn", "2 m/s sideways start, goal 2 m ahead"]
    body.append("## Normal grip, controller sees wheel odometry (a robot without vision), per scenario" + nl)
    for scen in out["runs"]["odom/normal/shipped"]:
        body.append(f"**{scen}**" + nl)
        body.append(table(out, "odom/normal", scen))
        body.append("")
    for cond, title in (("odom/low", "## Reduced grip (mu 0.70 static / 0.61 kinetic), wheel odometry, hard cases"),
                        ("odom/normal_cal", "## Normal grip, PERFECTLY CALIBRATED wheels (only slip corrupts odometry), hard cases"),
                        ("odom/low_cal", "## Reduced grip and perfectly calibrated wheels, hard cases")):
        body.append(title + nl)
        for scen in hard + ["straight 3 m"]:
            if scen in out["runs"][cond + "/shipped"]:
                body.append(f"**{scen}**" + nl)
                body.append(table(out, cond, scen))
                body.append("")
    if "followup" in out:
        body.append(followup_md(out))
    if "matched" in out:
        body.append(matched_md(out))
    npath = os.path.join(HERE, "notes.md")
    notes = open(npath).read() if os.path.exists(npath) else ""
    open(os.path.join(HERE, "results.md"), "w").write(
        notes + nl + nl.join(body) + nl + "Full tables, truth-pose runs and sweeps: results_tables.md; raw: results.json." + nl)


if __name__ == "__main__":
    main()
