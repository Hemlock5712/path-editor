package frc.robot.utils;

import frc.robot.generated.TunerConstants;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;

/** The biggest step toward the wanted velocity the robot can take this loop without slipping. */
public final class AccelerationLimiter {
  // Grip (ch09): a wheel pushes at most mu * N, so the robot speeds up at most mu * g
  private static final double MU = 1.0; // coefficient of friction, good tread on carpet
  private static final double G = 9.81; // gravity, m/s^2
  private static final double TRACTION_ACCEL = MU * G; // m/s^2, about 10
  private static final double ROBOT_MASS = 60.0; // kg, TODO: weigh the real robot

  // Motors (ch10)
  private static final double MODEL_CURRENT = 150.0; // amps per drive motor, for planning
  private static final double GEAR_RATIO = TunerConstants.FrontLeft.DriveMotorGearRatio;
  private static final double WHEEL_RADIUS = TunerConstants.FrontLeft.WheelRadius; // m

  // Turning (ch13)
  private static final double ANGULAR_ACCEL = 8.0; // rad/s^2, the turning cap
  private static final double BASE_RADIUS = // m, robot center to a wheel
      Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);

  // Kraken X60 FOC, from CTRE's dyno data (ch15). Free current is 0 so friction isn't counted as push.
  private static final DCMotor DRIVE_MOTOR =
      new DCMotor(12.0, 9.362, 476.1, 0.0, Units.rotationsPerMinuteToRadiansPerSecond(5785), 1);

  public static ChassisVelocities limit(
      ChassisVelocities current, ChassisVelocities wanted, double dt) {
    // the acceleration we're asking for: (wanted - current) / dt
    double ax = (wanted.vx - current.vx) / dt;
    double ay = (wanted.vy - current.vy) / dt;
    double alpha = (wanted.omega - current.omega) / dt;

    // Check 1: the turning cap
    alpha = Math.clamp(alpha, -ANGULAR_ACCEL, ANGULAR_ACCEL);

    // Check 2: the motors, only when speeding up (ch15)
    boolean speedingUp = ax * current.vx + ay * current.vy > 0;
    if (speedingUp) {
      double wheelSpeed =
          Math.hypot(current.vx, current.vy) + Math.abs(current.omega) * BASE_RADIUS;
      double motorLimit = motorAccel(wheelSpeed);
      double drive = Math.hypot(ax, ay);
      if (drive > motorLimit) {
        ax *= motorLimit / drive;
        ay *= motorLimit / drive;
      }
    }

    // Check 3: traction. Driving and turning are two sides of a right triangle;
    // the long side must fit inside the grip
    double total = Math.hypot(Math.hypot(ax, ay), alpha * BASE_RADIUS);
    if (total > TRACTION_ACCEL) {
      double scale = TRACTION_ACCEL / total;
      ax *= scale;
      ay *= scale;
      alpha *= scale;
    }

    // the answer: current + acceleration * dt
    return new ChassisVelocities(
        current.vx + ax * dt, current.vy + ay * dt, current.omega + alpha * dt);
  }

  /** The most acceleration the four drive motors can still add at this wheel speed (m/s^2). */
  public static double motorAccel(double wheelSpeed) {
    double motorSpeed = wheelSpeed / WHEEL_RADIUS * GEAR_RATIO; // rad/s
    double amps = Math.clamp(DRIVE_MOTOR.getCurrent(motorSpeed, 12.0), 0.0, MODEL_CURRENT);
    double torque = DRIVE_MOTOR.getTorque(amps); // N·m at the motor
    double wheelForce = torque * GEAR_RATIO / WHEEL_RADIUS; // N at each wheel
    return 4 * wheelForce / ROBOT_MASS; // F = m a, turned around
  }
}
