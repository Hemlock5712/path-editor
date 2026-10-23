package frc.team5712.path.core;

/**
 * The robot numbers the velocity profile plans with. Defaults match the bench plant ({@code
 * alignsim}): 60 kg, 6 kg·m², modules at (±0.29, ±0.29) m, 2 in wheels, 6.75:1, Kraken X60 FOC,
 * 120 A stator limit, SDS MK5 Spike Grip tread (μ = 1.5) and the centre of mass 0.2 m up (to be
 * measured).
 *
 * <p>Acceleration and cornering are limited to {@link #tractionMetersPerSecondSquared}, {@code
 * tractionShare · min(μ·g, g·x/h)}: a share (0.9 by default) of whichever comes first, the tread's grip or
 * tipping over, as Precision-Drive's AccelerationLimiter works it out. x is the tip distance and h the
 * centre of mass's height. The planner's grip circle and the follower's output limiter both come from it.
 *
 * @param massKg robot mass with battery and bumpers
 * @param momentOfInertiaKgM2 yaw moment of inertia
 * @param moduleRadiusMeters robot centre to a swerve module
 * @param wheelRadiusMeters drive wheel radius
 * @param driveGearRatio motor turns per wheel turn
 * @param driveMotor drive motor model
 * @param driveMotorCount number of drive motors
 * @param statorCurrentLimitAmps drive stator current limit
 * @param frictionCoefficient the tread's friction coefficient μ (1.5 for Spike Grip on carpet)
 * @param maxWheelMetersPerSecond fastest any wheel may be asked to roll; NaN = the motor's free speed
 * @param maxAngularRadPerSec spin cap for the planned heading (+∞ = only the wheel-speed cap)
 * @param maxAngularRadPerSecSquared spin acceleration cap for the planned heading (+∞ = only grip and
 *     motors). With a cap, a fast turn over a short distance slows the plan down instead of asking
 *     for more spin than the follower will give.
 * @param centerOfMassHeightMeters height of the centre of mass above the floor (0 = no load transfer)
 * @param tipDistanceMeters how far the centre is from the nearest tipping edge, {@code min(|module x|,
 *     |module y|)}: one value for every direction (for a square base the wheelbase and the track are
 *     the same); NaN = a square base, {@code moduleRadiusMeters / √2}
 * @param tractionShare the share of grip-or-tipping the robot plans and drives to (0.9: a 10 % margin, as
 *     AccelerationLimiter's {@code 0.9 *}), in (0, 1]
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
    double maxAngularRadPerSecSquared,
    double centerOfMassHeightMeters,
    double tipDistanceMeters,
    double tractionShare) {

  /** What sets the traction limit. */
  public enum TractionLimit {
    /** The tread's grip, μ·g (also with the centre of mass at 0). */
    GRIP,
    /** Tipping over, g·x/h (a tall robot). */
    TIPPING
  }

  public static final double GRAVITY = 9.81; // m/s²

  /** The default share of grip-or-tipping the robot uses (the rest is margin): {@link #tractionShare}. */
  public static final double TRACTION_MARGIN = 0.9;

  /** Spike Grip on carpet. */
  public static final double DEFAULT_FRICTION_COEFFICIENT = 1.5;

  /** A guess until it's measured. */
  public static final double DEFAULT_CENTER_OF_MASS_HEIGHT_METERS = 0.2;

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
          DEFAULT_FRICTION_COEFFICIENT,
          Double.NaN,
          Double.POSITIVE_INFINITY,
          Double.POSITIVE_INFINITY,
          DEFAULT_CENTER_OF_MASS_HEIGHT_METERS,
          Double.NaN,
          TRACTION_MARGIN);

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
    if (!(centerOfMassHeightMeters >= 0.0) || centerOfMassHeightMeters == Double.POSITIVE_INFINITY) {
      throw new IllegalArgumentException("centerOfMassHeightMeters must be 0 or more");
    }
    if (!Double.isNaN(tipDistanceMeters) && !(tipDistanceMeters > 0.0)) {
      throw new IllegalArgumentException("tipDistanceMeters must be positive or NaN");
    }
    if (!(tractionShare > 0.0 && tractionShare <= 1.0)) {
      throw new IllegalArgumentException("tractionShare must be in (0, 1]");
    }
  }

  /** With the default centre-of-mass height, a square base and the default 0.9 share. */
  public RobotConfig(double massKg, double momentOfInertiaKgM2, double moduleRadiusMeters,
      double wheelRadiusMeters, double driveGearRatio, Motor driveMotor, int driveMotorCount,
      double statorCurrentLimitAmps, double frictionCoefficient, double maxWheelMetersPerSecond,
      double maxAngularRadPerSec, double maxAngularRadPerSecSquared) {
    this(massKg, momentOfInertiaKgM2, moduleRadiusMeters, wheelRadiusMeters, driveGearRatio, driveMotor,
        driveMotorCount, statorCurrentLimitAmps, frictionCoefficient, maxWheelMetersPerSecond,
        maxAngularRadPerSec, maxAngularRadPerSecSquared, DEFAULT_CENTER_OF_MASS_HEIGHT_METERS, Double.NaN,
        TRACTION_MARGIN);
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
        maxWheelMetersPerSecond, maxAngularRadPerSec, maxAngularRadPerSecSquared, centerOfMassHeightMeters,
        tipDistanceMeters, tractionShare);
  }

  public RobotConfig withMaxWheelMetersPerSecond(double speed) {
    return new RobotConfig(massKg, momentOfInertiaKgM2, moduleRadiusMeters, wheelRadiusMeters,
        driveGearRatio, driveMotor, driveMotorCount, statorCurrentLimitAmps, frictionCoefficient,
        speed, maxAngularRadPerSec, maxAngularRadPerSecSquared, centerOfMassHeightMeters, tipDistanceMeters,
        tractionShare);
  }

  public RobotConfig withMaxAngularRadPerSec(double omega) {
    return new RobotConfig(massKg, momentOfInertiaKgM2, moduleRadiusMeters, wheelRadiusMeters,
        driveGearRatio, driveMotor, driveMotorCount, statorCurrentLimitAmps, frictionCoefficient,
        maxWheelMetersPerSecond, omega, maxAngularRadPerSecSquared, centerOfMassHeightMeters, tipDistanceMeters,
        tractionShare);
  }

  public RobotConfig withMaxAngularRadPerSecSquared(double alpha) {
    return new RobotConfig(massKg, momentOfInertiaKgM2, moduleRadiusMeters, wheelRadiusMeters,
        driveGearRatio, driveMotor, driveMotorCount, statorCurrentLimitAmps, frictionCoefficient,
        maxWheelMetersPerSecond, maxAngularRadPerSec, alpha, centerOfMassHeightMeters, tipDistanceMeters,
        tractionShare);
  }

  public RobotConfig withCenterOfMassHeightMeters(double h) {
    return new RobotConfig(massKg, momentOfInertiaKgM2, moduleRadiusMeters, wheelRadiusMeters,
        driveGearRatio, driveMotor, driveMotorCount, statorCurrentLimitAmps, frictionCoefficient,
        maxWheelMetersPerSecond, maxAngularRadPerSec, maxAngularRadPerSecSquared, h, tipDistanceMeters,
        tractionShare);
  }

  public RobotConfig withTipDistanceMeters(double d) {
    return new RobotConfig(massKg, momentOfInertiaKgM2, moduleRadiusMeters, wheelRadiusMeters,
        driveGearRatio, driveMotor, driveMotorCount, statorCurrentLimitAmps, frictionCoefficient,
        maxWheelMetersPerSecond, maxAngularRadPerSec, maxAngularRadPerSecSquared, centerOfMassHeightMeters, d,
        tractionShare);
  }

  public RobotConfig withTractionShare(double share) {
    return new RobotConfig(massKg, momentOfInertiaKgM2, moduleRadiusMeters, wheelRadiusMeters,
        driveGearRatio, driveMotor, driveMotorCount, statorCurrentLimitAmps, frictionCoefficient,
        maxWheelMetersPerSecond, maxAngularRadPerSec, maxAngularRadPerSecSquared, centerOfMassHeightMeters,
        tipDistanceMeters, share);
  }

  /** The tread's grip, μ·g, m/s². */
  public double gripMetersPerSecondSquared() {
    return frictionCoefficient * GRAVITY;
  }

  /** The tip distance in use: the given one, or a square base's {@code moduleRadiusMeters / √2}. */
  public double tipDistanceInUseMeters() {
    return Double.isNaN(tipDistanceMeters) ? moduleRadiusMeters / Math.sqrt(2.0) : tipDistanceMeters;
  }

  /** The acceleration that would tip the robot over, {@code g · tip / h}, m/s² (+∞ with the centre of mass at 0). */
  public double tipMetersPerSecondSquared() {
    return centerOfMassHeightMeters > 0.0 ? GRAVITY * tipDistanceInUseMeters() / centerOfMassHeightMeters
        : Double.POSITIVE_INFINITY;
  }

  /**
   * The most the robot accelerates or corners, all directions combined, m/s²: {@code tractionShare ·
   * min(μ·g, g·x/h)}, AccelerationLimiter's TRACTION_ACCEL. Both the planner's grip circle and the
   * follower's output limiter use it.
   */
  public double tractionMetersPerSecondSquared() {
    return tractionShare * Math.min(gripMetersPerSecondSquared(), tipMetersPerSecondSquared());
  }

  /**
   * Where the wheels that weight transfer unloads start to slip, {@code μ·g / (1 + μ·h/x)}, m/s², with no
   * margin. Velocity control pushes every wheel about the same, so asking for more than this, some wheels slip
   * while the rest grip: the adaptive grip takes that slip as the tread's own, not a slick floor.
   */
  public double wheelSlipOnsetMetersPerSecondSquared() {
    return gripMetersPerSecondSquared() / (1.0 + frictionCoefficient * centerOfMassHeightMeters / tipDistanceInUseMeters());
  }

  /** What sets {@link #tractionMetersPerSecondSquared} (with the centre of mass at 0, grip). */
  public TractionLimit tractionLimit() {
    return tipMetersPerSecondSquared() < gripMetersPerSecondSquared() ? TractionLimit.TIPPING : TractionLimit.GRIP;
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
