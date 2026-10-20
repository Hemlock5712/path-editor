package frc.team5712.path.core;

/**
 * The robot numbers the velocity profile plans with. Defaults match the bench plant ({@code
 * alignsim}): 60 kg, 6 kg·m², modules at (±0.29, ±0.29) m, 2 in wheels, 6.75:1, Kraken X60 FOC,
 * 120 A stator limit, and grip μ = 0.7 (a measured value belongs here; 1.0 spins the wheels).
 *
 * @param massKg robot mass with battery and bumpers
 * @param momentOfInertiaKgM2 yaw moment of inertia
 * @param moduleRadiusMeters robot centre to a swerve module
 * @param wheelRadiusMeters drive wheel radius
 * @param driveGearRatio motor turns per wheel turn
 * @param driveMotor drive motor model
 * @param driveMotorCount number of drive motors
 * @param statorCurrentLimitAmps drive stator current limit
 * @param frictionCoefficient grip: the robot can accelerate at most μ·g, all directions combined
 * @param maxWheelMetersPerSecond fastest any wheel may be asked to roll; NaN = the motor's free speed
 * @param maxAngularRadPerSec spin cap for the planned heading (+∞ = only the wheel-speed cap)
 * @param maxAngularRadPerSecSquared spin acceleration cap for the planned heading (+∞ = only grip and
 *     motors). With a cap, a fast turn over a short distance slows the plan down instead of asking
 *     for more spin than the follower will give.
 */
public record RobotConfig(
    double massKg,
    double momentOfInertiaKgM2,
    double moduleRadiusMeters,
    double wheelRadiusMeters,
    double driveGearRatio,
    Motor driveMotor,
    int driveMotorCount,
    double statorCurrentLimitAmps,
    double frictionCoefficient,
    double maxWheelMetersPerSecond,
    double maxAngularRadPerSec,
    double maxAngularRadPerSecSquared) {

  public static final double GRAVITY = 9.81; // m/s²

  public static final RobotConfig DEFAULT =
      new RobotConfig(
          60.0,
          6.0,
          Math.sqrt(0.29 * 0.29 + 0.29 * 0.29),
          0.0508,
          6.75,
          Motor.KRAKEN_X60_FOC,
          4,
          120.0,
          0.7,
          Double.NaN,
          Double.POSITIVE_INFINITY,
          Double.POSITIVE_INFINITY);

  public RobotConfig {
    if (!(massKg > 0.0) || !(momentOfInertiaKgM2 > 0.0) || !(moduleRadiusMeters > 0.0)
        || !(wheelRadiusMeters > 0.0) || !(driveGearRatio > 0.0) || driveMotorCount < 1
        || !(statorCurrentLimitAmps > 0.0) || !(frictionCoefficient > 0.0)
        || !(maxAngularRadPerSec > 0.0) || !(maxAngularRadPerSecSquared > 0.0)) {
      throw new IllegalArgumentException("robot parameters must be positive");
    }
    if (!Double.isNaN(maxWheelMetersPerSecond) && !(maxWheelMetersPerSecond > 0.0)) {
      throw new IllegalArgumentException("maxWheelMetersPerSecond must be positive or NaN");
    }
  }

  /** Without a spin acceleration cap (the record before it had one). */
  public RobotConfig(double massKg, double momentOfInertiaKgM2, double moduleRadiusMeters,
      double wheelRadiusMeters, double driveGearRatio, Motor driveMotor, int driveMotorCount,
      double statorCurrentLimitAmps, double frictionCoefficient, double maxWheelMetersPerSecond,
      double maxAngularRadPerSec) {
    this(massKg, momentOfInertiaKgM2, moduleRadiusMeters, wheelRadiusMeters, driveGearRatio, driveMotor,
        driveMotorCount, statorCurrentLimitAmps, frictionCoefficient, maxWheelMetersPerSecond,
        maxAngularRadPerSec, Double.POSITIVE_INFINITY);
  }

  public RobotConfig withFrictionCoefficient(double mu) {
    return new RobotConfig(massKg, momentOfInertiaKgM2, moduleRadiusMeters, wheelRadiusMeters,
        driveGearRatio, driveMotor, driveMotorCount, statorCurrentLimitAmps, mu,
        maxWheelMetersPerSecond, maxAngularRadPerSec, maxAngularRadPerSecSquared);
  }

  public RobotConfig withMaxWheelMetersPerSecond(double speed) {
    return new RobotConfig(massKg, momentOfInertiaKgM2, moduleRadiusMeters, wheelRadiusMeters,
        driveGearRatio, driveMotor, driveMotorCount, statorCurrentLimitAmps, frictionCoefficient,
        speed, maxAngularRadPerSec, maxAngularRadPerSecSquared);
  }

  public RobotConfig withMaxAngularRadPerSec(double omega) {
    return new RobotConfig(massKg, momentOfInertiaKgM2, moduleRadiusMeters, wheelRadiusMeters,
        driveGearRatio, driveMotor, driveMotorCount, statorCurrentLimitAmps, frictionCoefficient,
        maxWheelMetersPerSecond, omega, maxAngularRadPerSecSquared);
  }

  public RobotConfig withMaxAngularRadPerSecSquared(double alpha) {
    return new RobotConfig(massKg, momentOfInertiaKgM2, moduleRadiusMeters, wheelRadiusMeters,
        driveGearRatio, driveMotor, driveMotorCount, statorCurrentLimitAmps, frictionCoefficient,
        maxWheelMetersPerSecond, maxAngularRadPerSec, alpha);
  }

  /** Grip budget μ·g, m/s². */
  public double gripMetersPerSecondSquared() {
    return frictionCoefficient * GRAVITY;
  }

  /** Wheel speed at the motor's free speed, m/s. */
  public double freeSpeedMetersPerSecond() {
    return driveMotor.freeSpeedRadPerSec() / driveGearRatio * wheelRadiusMeters;
  }

  /** The wheel-speed cap in use: the given one, or the free speed. */
  public double wheelSpeedCapMetersPerSecond() {
    return Double.isNaN(maxWheelMetersPerSecond) ? freeSpeedMetersPerSecond() : maxWheelMetersPerSecond;
  }

  /**
   * The {@code I / (m·r)} lever, metres: spinning up at α needs each wheel to push like a straight
   * acceleration of {@code lever × α}.
   */
  public double spinLeverMeters() {
    return momentOfInertiaKgM2 / (massKg * moduleRadiusMeters);
  }

  /**
   * The most acceleration the drive motors can add with the wheels rolling at {@code wheelSpeed}:
   * full voltage minus back-EMF, capped by the stator limit (the AccelerationLimiter model).
   */
  public double driveAccelMetersPerSecondSquared(double wheelMetersPerSecond) {
    double motorRadPerSec = wheelMetersPerSecond / wheelRadiusMeters * driveGearRatio;
    double amps = driveMotor.currentAt(motorRadPerSec, driveMotor.nominalVolts());
    amps = amps < 0.0 ? 0.0 : (amps > statorCurrentLimitAmps ? statorCurrentLimitAmps : amps);
    return forcePerAmp() * amps / massKg;
  }

  /** The most braking the drive motors can give: back-EMF helps, so only the stator limit binds. */
  public double brakeAccelMetersPerSecondSquared() {
    return forcePerAmp() * statorCurrentLimitAmps / massKg;
  }

  /** Total push at the treads per amp in every motor, N/A. */
  private double forcePerAmp() {
    return driveMotorCount * driveMotor.torquePerAmp() * driveGearRatio / wheelRadiusMeters;
  }
}
