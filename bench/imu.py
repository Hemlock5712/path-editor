"""An accelerometer for the bench, as a wrapper around the plant (the alignsim plant itself has none).

It reads the TRUE chassis acceleration (the change in the plant's field velocity since the last read,
turned into the robot frame) and adds the errors of a real FRC IMU. The numbers and where they come from:

| term | value | source |
|---|---|---|
| output rate | 400 Hz | Systemcore technical specifications (Limelight, June 2025 alpha): "IMU Output 400hz ... 3 axis acceleration" |
| range / resolution | +/-8 g, 16 bit (0.0024 m/s^2 per count) | CTRE Pigeon 2.0 User's Guide, accelerometer specifications |
| white noise | 70 ug/sqrt(Hz) | TDK ICM-42688-P product page (a current MEMS IMU of the same class; neither Systemcore nor the Pigeon 2 names its part or gives a noise density) |
| bias | 10 mg (0.098 m/s^2) 1-sigma per axis, fixed per run | ASSUMED: the ICM-42688-P lists a zero-g offset of +/-20 (unit not stated, mg assumed); the Pigeon 2 is factory-calibrated |
| vibration | 0.5 m/s^2 RMS per axis, white | ASSUMED, not from a datasheet: drivetrain and carpet vibration on a robot dwarfs the sensor's own noise |

The bench reads it on the 250 Hz odometry tick; the white noise is the 400 Hz part's (bandwidth 200 Hz).
The IMU sits at the robot's centre (no lever-arm terms). It draws from its own random generator, so adding
it changes no other number in the bench.
"""

import math

import numpy as np

G = 9.81
RATE_HZ = 400.0
NOISE_DENSITY = 70e-6 * G  # m/s^2 per sqrt(Hz)
LSB = 16.0 * G / 65536.0  # +/-8 g over 16 bits
BIAS_SIGMA = 0.010 * G
VIBRATION_RMS = 0.5


class Imu:
    def __init__(self, plant, rng: np.random.Generator):
        self.plant = plant
        self.rng = rng
        self.bias = rng.normal(0.0, BIAS_SIGMA, 2)
        self.white = NOISE_DENSITY * math.sqrt(RATE_HZ / 2.0)
        self.last = (plant.t, plant.vx, plant.vy)

    def read(self):
        """(ax, ay) in the robot frame, m/s^2, as the IMU reports them now."""
        p = self.plant
        t0, vx0, vy0 = self.last
        dt = p.t - t0
        if dt <= 1e-9:
            ax_f = ay_f = 0.0
        else:
            ax_f, ay_f = (p.vx - vx0) / dt, (p.vy - vy0) / dt
        self.last = (p.t, p.vx, p.vy)
        c, s = math.cos(p.th), math.sin(p.th)
        ax, ay = c * ax_f + s * ay_f, -s * ax_f + c * ay_f
        noise = self.rng.normal(0.0, 1.0, 2) * math.hypot(self.white, VIBRATION_RMS)
        ax = ax + self.bias[0] + noise[0]
        ay = ay + self.bias[1] + noise[1]
        return (round(ax / LSB) * LSB, round(ay / LSB) * LSB)
