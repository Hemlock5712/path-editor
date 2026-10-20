"""One bench run: a follower drives the alignsim plant along a course; metrics against the TRUE pose.

Loop timing is the same as bench/reference/race_drive_to_pose.py `run_one`:
- physics at 500 Hz;
- an odometry thread at 250 Hz that applies the newest request (field speeds -> robot speeds with
  the odometry heading at apply time, like CTRE's ApplyFieldSpeeds) and updates wheel odometry;
- a 50 Hz robot loop with 1 ms jitter; each request reaches the motors 2 ms later.
"""

import math
import time
from dataclasses import replace

import numpy as np

from alignsim.odometry import SwerveOdometry
from alignsim.plant import SwervePlant
from common import LP, RP, rot
from courses import COURSES, NeutralPath, polyline_distance, t_end
from followers import build
from followers.base import Start

SETTLE_POS, SETTLE_YAW, SETTLE_SPEED, SETTLE_HOLD = 0.02, math.radians(1.0), 0.05, 0.25
END_RADIUS = 0.05


class DisturbedPlant(SwervePlant):
    """External-force hook for the bench. The core plant already applies a field-frame force and torque
    for a set time (`SwervePlant.push`, added to the chassis force in `step`); this subclass only decides
    WHEN: `arm(impulse, path)` and then `poll(true_xy)` every odometry tick fires the shove once the TRUE
    position has passed `impulse.at_fraction` of the neutral path, perpendicular to the path there
    (positive force = to the left of travel). Physics is untouched."""

    armed = None
    fired_at = None
    grip_armed = None
    grip_dropped_at = None

    def arm(self, impulse, npath: NeutralPath):
        self.armed = (impulse, npath)

    def arm_grip_drop(self, drop, npath: NeutralPath):
        """drop = (fraction of the path, mu static, mu kinetic): the floor changes once the TRUE position
        passes that fraction of the neutral path (a worn carpet seam)."""
        self.grip_armed = (drop, npath)

    def poll(self, xy):
        if self.grip_armed is not None and self.grip_dropped_at is None:
            (frac, mu_s, mu_k), npath = self.grip_armed
            s, _ = npath.project(np.array([xy]))
            if s[0] >= frac * npath.length:
                self.rp = replace(self.rp, mu_static=mu_s, mu_kinetic=mu_k)
                self.grip_dropped_at = (self.t, float(s[0]))
        if self.armed is None or self.fired_at is not None:
            return
        imp, npath = self.armed
        s, _ = npath.project(np.array([xy]))
        if s[0] >= imp.at_fraction * npath.length:
            tx, ty = npath.tangent_at_s(float(s[0]))
            self.push(-ty * imp.force_n, tx * imp.force_n, 0.0, imp.duration)
            self.fired_at = (self.t, float(s[0]), xy)


def make_robot(course, seed):
    rng = np.random.default_rng(seed)
    rp = replace(RP, wheel_radius_error=rng.normal(0, 0.01), gyro_scale_error=rng.normal(0, 0.002))
    if course.grip is not None:
        rp = replace(rp, mu_static=course.grip[0], mu_kinetic=course.grip[1])
    return rp, rng


def simulate(follower, native, start, v0, rp, rng, T, pose_src, plant=None, imu_rng=None):
    """Run one follower on its native path from `start` (pose) with field velocity v0 for T seconds
    (reset is called here and timed). Returns a dict of 250 Hz traces
    (t, x, y, unwrapped heading, true speed) and run facts.

    pose_src: 'truth', 'odom' (WPILib wheel odometry + gyro) or 'slipodom' (lib's SlipAwareOdometry with
    the bench IMU, see slipsense.py). Slip sensing also runs for any follower with `wants_sensing`."""
    if plant is None:
        plant = DisturbedPlant(rp, start, v0, rng)
    gyro0, modules0 = plant.read_gyro(), plant.read_modules()
    odo = SwerveOdometry(rp.module_positions, gyro0, modules0, start)
    sensing = None

    def measure():
        mods = plant.module_velocities()
        vxr = sum(s * math.cos(a) for s, a in mods) / 4.0
        vyr = sum(s * math.sin(a) for s, a in mods) / 4.0
        wr = 0.0
        for (s, a), (mx, my) in zip(mods, plant.mods):
            wr += (-my * s * math.cos(a) + mx * s * math.sin(a)) / (mx * mx + my * my)
        if pose_src == "truth":
            pose = plant.pose()
        elif pose_src == "slipodom" and sensing is not None:
            pose = sensing.pose
        else:
            pose = odo.pose
        v = rot((vxr, vyr), pose[2])
        return pose, (v[0], v[1], wr / 4.0)

    pose, vel = measure()
    t0 = time.perf_counter()
    follower.reset(native, Start(0.0, pose, vel))
    gen_ms = 1000 * (time.perf_counter() - t0)
    if pose_src == "slipodom" or getattr(follower, "wants_sensing", False):
        from imu import Imu
        from slipsense import Sensing

        sensing = Sensing(plant, Imu(plant, imu_rng or np.random.default_rng(0)), start, gyro0, modules0)

    dt = LP.sim_dt
    t, next_odom, next_main = 0.0, 0.0, LP.main_period
    requests, cur_req = [], (v0[0], v0[1], v0[2])
    ts, xs, ys, ths, sp = [], [], [], [], []
    wheel_slip = 0.0
    t_done = None
    max_step_ms = 0.0
    for _ in range(int(round(T / dt))):
        if t >= next_odom - 1e-9:
            next_odom += LP.odom_period
            while requests and requests[0][0] <= t:
                cur_req = requests.pop(0)[1]
            if cur_req == "idle":
                plant.set_neutral()
            else:
                vr = rot((cur_req[0], cur_req[1]), -odo.pose[2])
                plant.set_chassis_robot(vr[0], vr[1], cur_req[2])
            gyro, modules = plant.read_gyro(), plant.read_modules()
            odo.update(gyro, modules)
            if sensing is not None:
                sensing.update(t, gyro, modules, cur_req, odo.pose[2])
            plant.poll((plant.x, plant.y))
            ts.append(t)
            xs.append(plant.x)
            ys.append(plant.y)
            ths.append(plant.th_unwrapped)
            sp.append(math.hypot(plant.vx, plant.vy))
            wheel_slip += sum(1 for x in plant.slipping if x) * LP.odom_period
        if t >= next_main - 1e-9:
            next_main += LP.main_period + rng.normal(0, LP.main_jitter)
            if t_done is None:
                pose, vel = measure()
                c0 = time.perf_counter()
                cmd = follower.step(t, pose, vel)
                max_step_ms = max(max_step_ms, 1000 * (time.perf_counter() - c0))
                if follower.done():
                    t_done = t
                    cmd = (0.0, 0.0, 0.0) if follower.end_output == "zero" else "idle"
                requests.append((t + LP.actuation_delay, cmd if cmd == "idle" else tuple(cmd)))
        plant.step(dt)
        t += dt
    ts, xs, ys, ths, sp = map(np.array, (ts, xs, ys, ths, sp))
    return {"t": ts, "x": xs, "y": ys, "th": ths, "speed": sp, "t_done": t_done, "wheel_slip": wheel_slip,
            "max_step_ms": max_step_ms, "gen_ms": gen_ms, "plant": plant,
            "slip_metrics": sensing.metrics() if sensing is not None else (float("nan"), float("nan")),
            "odo_err_cm": 100 * math.hypot(odo.pose[0] - plant.x, odo.pose[1] - plant.y)}


def end_metrics(r, end):
    """Time to within 5 cm of `end`, time settled (2 cm, 1 deg, < 5 cm/s, held 0.25 s), and distances."""
    ts, xs, ys, ths, sp = r["t"], r["x"], r["y"], r["th"], r["speed"]
    dist = np.hypot(end[0] - xs, end[1] - ys)
    eth = np.abs(np.arctan2(np.sin(end[2] - ths), np.cos(end[2] - ths)))
    in5 = np.flatnonzero(dist < END_RADIUS)
    t_5cm = float(ts[in5[0]]) if len(in5) else float("nan")
    ok = (dist < SETTLE_POS) & (eth < SETTLE_YAW) & (sp < SETTLE_SPEED)
    hold = int(round(SETTLE_HOLD / LP.odom_period))
    t_settle, run = float("nan"), 0
    for i, o in enumerate(ok):
        run = run + 1 if o else 0
        if run >= hold:
            t_settle = float(ts[i - hold + 1])
            break
    return dist, eth, t_5cm, t_settle


def run_one(job, trace=False):
    """job = (course name, method key, options dict or None, seed, pose source 'odom' | 'truth').
    trace=True adds the true path every 40 ms as [[t, x, y, heading], ...]."""
    cname, mkey, opts, seed, pose_src = job
    course = COURSES[cname]
    npath = NeutralPath(course)
    rp, rng = make_robot(course, seed)
    start = course.start_pose
    v0 = course.start_velocity
    plant = DisturbedPlant(rp, start, v0, rng)
    if course.impulse is not None:
        plant.arm(course.impulse, npath)
    if course.grip_drop is not None:
        plant.arm_grip_drop(course.grip_drop, npath)
    cls, kw = build(mkey, opts)
    follower = cls(**kw)
    native = follower.adapt(course)
    imu_rng = np.random.default_rng([seed, 7919])  # its own stream: the IMU changes no other number
    r = simulate(follower, native, start, v0, rp, rng, t_end(course), pose_src, plant, imu_rng)
    ts, xs, ys, ths = r["t"], r["x"], r["y"], r["th"]
    end = course.end_pose
    dist, eth, t_5cm, t_settle = end_metrics(r, end)
    t_done, wheel_slip, gen_ms, max_step_ms = r["t_done"], r["wheel_slip"], r["gen_ms"], r["max_step_ms"]
    T = t_end(course)
    # cross-track: from the start until the robot first reaches the end of the path (within 5 cm of the
    # end point, or its closest point on the neutral path within 5 cm of the end); the whole run if never
    s_all, d_all = npath.project(np.column_stack([xs, ys]))
    reached = np.flatnonzero((dist < END_RADIUS) | (s_all >= npath.length - END_RADIUS))
    n_ct = (reached[0] + 1) if len(reached) else len(ts)
    xy = np.column_stack([xs[:n_ct], ys[:n_ct]])
    d_neutral = d_all[:n_ct]
    d_own = polyline_distance(cls.geometry(native), xy)
    tx, ty = npath.tangent_at_s(npath.length)
    over = float(max(0.0, ((xs - end[0]) * tx + (ys - end[1]) * ty).max()))
    out = {
        "ct_rms_cm": 100 * float(np.sqrt(np.mean(d_neutral ** 2))),
        "ct_max_cm": 100 * float(d_neutral.max()),
        "ct_own_rms_cm": 100 * float(np.sqrt(np.mean(d_own ** 2))),
        "ct_own_max_cm": 100 * float(d_own.max()),
        "err_end_cm": 100 * float(dist[-1]),
        "yaw_end_deg": math.degrees(float(eth[-1])),
        "overshoot_cm": 100 * over,
        "t_5cm": t_5cm,
        "t_settle": t_settle,
        "t_done": float("nan") if t_done is None else t_done,
        "wheel_slip_ms": 1000 * wheel_slip,
        "odo_err_cm": r["odo_err_cm"],
        "gen_ms": gen_ms,
        "max_step_ms": max_step_ms,
        "hit_s": float("nan") if plant.fired_at is None else plant.fired_at[1],
        "t_sim": T,
        "slip_fp": r["slip_metrics"][0],
        "slip_recall": r["slip_metrics"][1],
    }
    if trace:
        k = max(1, int(round(0.04 / LP.odom_period)))
        out["trace"] = [[round(float(a), 3), round(float(b), 4), round(float(c), 4), round(float(d), 4)]
                        for a, b, c, d in zip(ts[::k], xs[::k], ys[::k], ths[::k])]
    return out

