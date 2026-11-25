"""Every tunable number of the simulated robot, cameras and field, with its source or rationale.

Numbers marked ASSUMED are guesses to be replaced from robot logs (see docs/alignment test plan).
"""

import math
from dataclasses import dataclass, field, replace


@dataclass
class RobotParams:
    # Chassis
    mass: float = 60.0  # kg incl. battery + bumpers (2026 code used 60)
    inertia: float = 6.0  # kg m^2 (~ m * (0.8^2 + 0.8^2) / 12)
    module_xy: float = 0.29  # m, module centers at (+-x, +-y)
    # Traction - ASSUMED, tread on carpet
    mu_static: float = 1.15
    mu_kinetic: float = 1.0
    lateral_stiffness: float = 4000.0  # N per m/s of sideways tread creep, before sliding (brush model)
    # Drive: Kraken X60 FOC
    wheel_radius: float = 0.0508  # m (2 in)
    drive_gear: float = 6.75
    motor_kt: float = 0.0194  # Nm/A  (9.37 Nm / 483 A)
    motor_r: float = 0.0248  # ohm   (12 V / 483 A)
    motor_kv: float = 50.6  # rad/s per V (5800 rpm / 12 V)
    stator_limit: float = 120.0  # A (template "slip current")
    reflected_wheel_mass: float = 1.2  # kg, rotor + wheel inertia seen at the tread
    drive_kp: float = 3.0  # V per m/s of wheel speed error (onboard velocity loop)
    drive_ks: float = 0.15  # V static friction feedforward
    cosine_compensation: bool = True
    # Optional physics / onboard control (0 = off, the default for every earlier study)
    cg_height: float = 0.0  # m; > 0 adds weight transfer (load moves off the wheels you accelerate away from)
    drive_ramp: float = 0.0  # m/s^2; > 0 = onboard setpoint ramp + accel feedforward (Motion Magic velocity)
    # Steering
    steer_gain: float = 60.0  # 1/s
    steer_max_rate: float = 25.0  # rad/s
    steer_noise: float = math.radians(0.15)  # encoder noise
    # Sensors - ASSUMED
    wheel_radius_error: float = 0.0  # fractional, set per trial (wear, tread compression)
    gyro_noise: float = math.radians(0.02)
    gyro_drift_rate: float = math.radians(0.5) / 60.0  # rad/s
    gyro_scale_error: float = 0.0  # fractional, set per trial

    @property
    def free_speed(self):
        return self.motor_kv * 12.0 / self.drive_gear * self.wheel_radius

    @property
    def module_positions(self):
        a = self.module_xy
        return ((a, a), (a, -a), (-a, a), (-a, -a))


@dataclass
class LoopParams:
    sim_dt: float = 0.002  # 500 Hz physics
    odom_period: float = 0.004  # 250 Hz CTRE odometry thread (CAN FD)
    main_period: float = 0.020  # 50 Hz robot loop
    main_jitter: float = 0.001
    actuation_delay: float = 0.002  # request -> motors (applied on the next odometry tick anyway)


__all__ = ["RobotParams", "LoopParams", "replace", "field"]
