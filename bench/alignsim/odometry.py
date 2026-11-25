"""Swerve odometry (WPILib forward kinematics + gyro)."""

import math

import numpy as np

from .geom import compose, twist_exp, wrap


class SwerveOdometry:
    """WPILib SwerveDriveOdometry: least-squares module twist, heading from the gyro."""

    def __init__(self, module_positions, gyro, modules, pose=(0.0, 0.0, 0.0)):
        rows = []
        for mx, my in module_positions:
            rows.append([1.0, 0.0, -my])
            rows.append([0.0, 1.0, mx])
        self.pinv = np.linalg.pinv(np.array(rows))
        self.prev_dist = [m[0] for m in modules]
        self.gyro_offset = wrap(pose[2] - gyro)
        self.prev_angle = pose[2]
        self.pose = pose

    def update(self, gyro, modules):
        d = []
        for i, (dist, ang) in enumerate(modules):
            delta = dist - self.prev_dist[i]
            d.append(delta * math.cos(ang))
            d.append(delta * math.sin(ang))
            self.prev_dist[i] = dist
        tw = self.pinv @ np.array(d)
        angle = wrap(gyro + self.gyro_offset)
        dth = wrap(angle - self.prev_angle)
        new = compose(self.pose, twist_exp(float(tw[0]), float(tw[1]), dth))
        self.pose = (new[0], new[1], angle)
        self.prev_angle = angle
        return self.pose

    def reset(self, gyro, pose):
        self.gyro_offset = wrap(pose[2] - gyro)
        self.prev_angle = pose[2]
        self.pose = pose
