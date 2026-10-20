"""Slip sensing for the bench: lib's SlipAwareOdometry and SlipDetector (Java) fed every odometry tick.

It runs in the worker's BenchServer (the same JVM as ours v2's follower, so ours' adaptive grip reads the
same detector). It serves two things:
- the pose source "slipodom": slip-aware odometry, used by EVERY method in that condition;
- ours v2's adaptive grip, in every pose condition: the follower needs the slip signal whatever pose it
  is given.

It also counts how well the detector agrees with the plant's true slip flags, tick by tick.
"""

import math

from common import RP
from followers.ours_v2 import call_fresh, server


OPTIONS = {}  # SlipConfig overrides by name (for exploring thresholds on the tuning seeds); empty = lib defaults


class Sensing:
    def __init__(self, plant, imu, start_pose, gyro, modules):
        self.plant = plant
        self.imu = imu
        self.pose = tuple(start_pose)
        self.conf = 0.0
        self.ticks_clean = self.fp = self.ticks_slip = self.tp = 0
        call_fresh(op="sense_reset", modules=[list(m) for m in RP.module_positions], pose=list(start_pose),
                   gyro=float(gyro), dist=[float(m[0]) for m in modules], options=dict(OPTIONS))
        self.srv = server()

    def update(self, t, gyro, modules, request, odo_heading):
        """One odometry tick with the readings the plain odometry just used."""
        vel = self.plant.module_velocities()
        imu = self.imu.read()
        cmd = None
        if request != "idle":
            c, s = math.cos(odo_heading), math.sin(odo_heading)
            cmd = [c * request[0] + s * request[1], -s * request[0] + c * request[1], request[2]]
        r = self.srv.call(op="sense", t=float(t), gyro=float(gyro), dist=[float(m[0]) for m in modules],
                          ang=[float(m[1]) for m in modules], spd=[float(v[0]) for v in vel], imu=list(imu), cmd=cmd)
        self.pose = tuple(r["pose"])
        self.conf = r["conf"]
        self.last = r
        truth = any(self.plant.slipping)
        said = self.conf >= 0.5
        if truth:
            self.ticks_slip += 1
            self.tp += said
        else:
            self.ticks_clean += 1
            self.fp += said

    def metrics(self):
        """False-positive share of the clean ticks, and the share of truly slipping ticks detected."""
        fp = self.fp / self.ticks_clean if self.ticks_clean else float("nan")
        recall = self.tp / self.ticks_slip if self.ticks_slip else float("nan")
        return fp, recall
