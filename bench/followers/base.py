"""The interface every follower implements, and the shared AccelerationLimiter port.

    f = Method(**options)
    native = f.adapt(course)               # what the method authors (untimed: it's the path file)
    f.reset(native, start)                 # timed: the method's own path preparation
    cmd = f.step(t, pose, field_velocity)  # 50 Hz; returns field-relative (vx, vy, omega)
    f.done()                               # True once the method's command would end

After done(), the run loop applies `f.end_output`: 'zero' (zero speeds) or 'idle' (motors neutral).
"""

import math

from common import BASE_R, RP, clamp


class Start:
    """Start state handed to reset(): time, measured pose and measured field velocity."""

    def __init__(self, t, pose, vel, handed=None):
        self.t, self.pose, self.vel = t, tuple(pose), tuple(vel)
        self.handed = None if handed is None else tuple(handed)  # the previous command's last output, in a chain


class Follower:
    end_output = "zero"

    @staticmethod
    def adapt(course):
        raise NotImplementedError

    @staticmethod
    def geometry(native):
        """(N, 2) points of the path this method authored, for the 'own geometry' cross-track metric."""
        raise NotImplementedError

    def reset(self, native, start: Start):
        raise NotImplementedError

    def step(self, t, pose, vel):
        raise NotImplementedError

    def done(self):
        return False


class AccelerationLimiter:
    """Port of AccelerationLimiter.limit() (bench/reference/AccelerationLimiter.java, as ported in
    the first drive-to-pose race). Gear ratio and wheel radius come from the sim's RobotParams."""

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
