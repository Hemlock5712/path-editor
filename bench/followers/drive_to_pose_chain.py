"""Naive baseline: our DriveToPose (bench/reference/DriveToPose.java, the race's `Ours` port) run as a
sequence of commands, one per waypoint. Each leg runs until DriveToPose.isFinished() (measured pose within
2 cm and 2 deg, measured speed under 5 cm/s), so the robot stops at every waypoint; the next leg then
initializes from the measured velocity and runs in the same loop (its request replaces the leg's stop()).

Limits set to the shared ones: MAX_SPEED 4.0, DECEL 6, MAX_OMEGA 540 deg/s, ANGULAR_DECEL 720 deg/s^2,
and AccelerationLimiter traction 6 m/s^2 (DriveToPose's only acceleration setting) instead of the file's
MU 1.0 (9.81). The limiter's ANGULAR_ACCEL (8 rad/s^2) is kept as shipped.
"""

import math

import numpy as np

from common import Limits, clamp, wrap
from followers.base import AccelerationLimiter, Follower


class DriveToPoseChain(Follower):
    REACTION, EASE_IN, DEADBAND = 0.050, 6.0, 0.003
    EASE_IN_H, H_REACTION, H_DEADBAND = 5.0, 0.050, math.radians(0.3)
    SETTLED_SPEED, POS_TOL, HEAD_TOL = 0.05, 0.02, math.radians(2.0)

    def __init__(self, limits: Limits = Limits(), traction=None):
        self.lim = limits
        self.traction = limits.max_accel if traction is None else traction

    @staticmethod
    def adapt(course):
        from courses import to_goal_chain

        return to_goal_chain(course)

    @staticmethod
    def geometry(native):
        return np.array([native["start"][:2]] + [g[:2] for g in native["goals"]], float)

    def reset(self, native, start):
        self.goals = list(native["goals"])
        self.i = 0
        self._init_leg(start.t, start.vel)
        self.finished = False

    def _init_leg(self, t, vel):
        self.cmd = tuple(vel)  # commanded = drivetrain.getFieldVelocity()
        self.last_t = t

    def _execute(self, t, pose):
        lim = self.lim
        dt = clamp(t - self.last_t, 0.001, 0.1)
        self.last_t = t
        goal = self.goals[self.i]
        ex, ey = goal[0] - pose[0], goal[1] - pose[1]
        d = math.hypot(ex, ey)
        eth = wrap(goal[2] - pose[2])
        now = math.hypot(self.cmd[0], self.cmd[1])
        if d < self.DEADBAND:
            speed = 0.0
        else:
            brake = max(0.0, d - now * self.REACTION)
            speed = min(lim.max_speed, math.sqrt(2 * lim.max_accel * brake), self.EASE_IN * d)
        vx, vy = (ex / d * speed, ey / d * speed) if d > 1e-6 else (0.0, 0.0)
        a = abs(eth)
        if a < self.H_DEADBAND:
            om = 0.0
        else:
            braking = max(0.0, a - abs(self.cmd[2]) * self.H_REACTION)
            om = math.copysign(min(lim.max_omega, math.sqrt(2 * lim.max_alpha * braking), self.EASE_IN_H * a), eth)
        self.cmd = AccelerationLimiter.limit(self.cmd, (vx, vy, om), dt, self.traction)
        return d, eth

    def step(self, t, pose, vel):
        if self.finished:
            return (0.0, 0.0, 0.0)
        d, eth = self._execute(t, pose)
        while d < self.POS_TOL and abs(eth) < self.HEAD_TOL and math.hypot(vel[0], vel[1]) < self.SETTLED_SPEED:
            if self.i == len(self.goals) - 1:
                self.finished = True  # stop(): zero
                return (0.0, 0.0, 0.0)
            self.i += 1
            self._init_leg(t - 0.02, vel)  # the next leg's first execute runs in this loop
            d, eth = self._execute(t, pose)
        return self.cmd

    def done(self):
        return self.finished
