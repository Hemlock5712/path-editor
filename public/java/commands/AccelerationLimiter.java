package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.Motor;

/**
 * Physics-based acceleration limiter for swerve drive.
 *
 * <p>Uses real motor dyno data instead of arbitrary tuning constants. Applies two limits:
 *
 * <ul>
 *   <li>Motor Limit: Torque decreases with speed (limits acceleration, not braking)
 *   <li>Friction Limit: Combined acceleration cannot exceed coefficient of friction times gravity
 * </ul>
 *
 * <p>This prevents wheel slip during aggressive maneuvers while maximizing performance.
 */
public final class AccelerationLimiter {

  // Physical constants
  public static final double GRAVITY = 9.81; // m/s^2

  // Distance from robot center to wheel (for converting angular to linear)
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);

  // Maximum robot velocity from TunerX configuration
  public static final double MAX_VELOCITY = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  // Friction coefficient of 1.0 assumes good tread on carpet. Max acceleration = coefficient * g
  public static final double MAX_FRICTION_ACCEL = 1.1 * GRAVITY;

  // Robot parameters for motor torque calculations
  private static final Motor MOTOR = Motor.KRAKEN_X60_FOC;
  private static final double GEAR_RATIO = TunerConstants.FrontLeft.DriveMotorGearRatio;
  private static final double WHEEL_RADIUS = TunerConstants.FrontLeft.WheelRadius;
  private static final double ROBOT_MASS = 60; // kg, including bumpers and battery
  private static final int NUM_DRIVE_MOTORS = 4;

  // Stator current limit for torque model (150A per motor, 600A total max)
  // More conservative than the 200A hardware limit in TunerConstants
  private static final double STATOR_CURRENT_LIMIT = 150.0;

  // Minimum time step to prevent division by zero
  private static final double MIN_DT = 1e-9;

  // Reusable array for limited acceleration results (FRC is single-threaded)
  // [0] = vx, [1] = vy, [2] = omega
  private static final double[] ACCEL_RESULT = new double[3];

  // Last limited acceleration from integrateVelocity(), in caller's frame (field-relative).
  // Stored separately from ACCEL_RESULT because that array is a scratch buffer.
  private static double lastAccelVx = 0;
  private static double lastAccelVy = 0;
  private static double lastAccelOmega = 0;

  private AccelerationLimiter() {}

  /**
   * Returns the last limited acceleration computed by {@link #integrateVelocity}.
   *
   * <p>The returned ChassisSpeeds represents acceleration (m/s^2 and rad/s^2), not velocity. It is
   * in the same frame as the inputs to integrateVelocity (field-relative for all current callers).
   *
   * @return Last limited acceleration as ChassisSpeeds (fields are m/s^2 and rad/s^2)
   */
  public static ChassisSpeeds getLastAcceleration() {
    return new ChassisSpeeds(lastAccelVx, lastAccelVy, lastAccelOmega);
  }

  /**
   * Applies motor torque and friction limits to acceleration using primitives.
   *
   * <p>Uses a double array to avoid object allocations in the hot path: result[0] = limited accel
   * X, result[1] = limited accel Y, result[2] = limited omega
   *
   * @param accelX Desired X acceleration
   * @param accelY Desired Y acceleration
   * @param accelOmega Desired angular acceleration
   * @param velX Current X velocity
   * @param velY Current Y velocity
   * @param velOmega Current angular velocity
   * @param result Output array for limited acceleration [vx, vy, omega]
   */
  private static void applyLimits(
      double accelX,
      double accelY,
      double accelOmega,
      double velX,
      double velY,
      double velOmega,
      double maxAccel,
      double[] result) {

    // First apply motor torque limit (only affects acceleration, not braking)
    applyMotorLimit(accelX, accelY, accelOmega, velX, velY, velOmega, result);

    // Then apply friction limit (affects both acceleration and braking)
    applyFrictionLimit(result[0], result[1], result[2], maxAccel, result);
  }

  /**
   * Applies motor torque limit based on motor speed-torque curve.
   *
   * <p>Motor torque decreases as speed increases (back-EMF effect). This limits how fast we can
   * accelerate at high speeds. Braking is not limited because it uses regenerative braking and
   * mechanical friction, not motor torque.
   */
  private static void applyMotorLimit(
      double accelX,
      double accelY,
      double accelOmega,
      double velX,
      double velY,
      double velOmega,
      double[] result) {

    // Check if we're braking (acceleration opposes velocity)
    boolean linearBraking = isDecelerating(accelX, accelY, velX, velY);
    boolean angularBraking = accelOmega * velOmega < 0;

    // Braking doesn't use motor torque, so no limit applies
    if (linearBraking && angularBraking) {
      result[0] = accelX;
      result[1] = accelY;
      result[2] = accelOmega;
      return;
    }

    // Calculate acceleration contributions (only count accelerating components)
    double linearAccelMag = Math.hypot(accelX, accelY);
    double linearContrib = linearBraking ? 0 : linearAccelMag;
    double angularContrib = angularBraking ? 0 : Math.abs(accelOmega) * DRIVE_BASE_RADIUS;
    double combinedAccel = Math.hypot(linearContrib, angularContrib);

    // Estimate worst-case module speed for torque lookup
    double linearVelMag = Math.hypot(velX, velY);
    double moduleSpeed = linearVelMag + Math.abs(velOmega) * DRIVE_BASE_RADIUS;
    double maxMotorAccel =
        MOTOR.getMaxAcceleration(
            moduleSpeed,
            GEAR_RATIO,
            WHEEL_RADIUS,
            ROBOT_MASS,
            NUM_DRIVE_MOTORS,
            STATOR_CURRENT_LIMIT);

    // If under the limit, no scaling needed
    if (combinedAccel <= maxMotorAccel) {
      result[0] = accelX;
      result[1] = accelY;
      result[2] = accelOmega;
      return;
    }

    // Scale down accelerating components proportionally
    double scale = maxMotorAccel / combinedAccel;
    double linearScale = linearBraking ? 1.0 : scale;
    double angularScale = angularBraking ? 1.0 : scale;
    result[0] = accelX * linearScale;
    result[1] = accelY * linearScale;
    result[2] = accelOmega * angularScale;
  }

  /**
   * Checks if acceleration is opposing velocity (braking).
   *
   * @return true if the dot product of accel and velocity is negative
   */
  private static boolean isDecelerating(double accelX, double accelY, double velX, double velY) {
    return accelX * velX + accelY * velY < 0;
  }

  /**
   * Applies friction limit using circular constraint.
   *
   * <p>The total acceleration at each wheel cannot exceed friction coefficient times gravity.
   * Linear and angular acceleration combine as vectors, so we use Pythagorean theorem:
   * sqrt(linear^2 + angular^2) must be less than max friction acceleration.
   */
  private static void applyFrictionLimit(
      double accelX, double accelY, double accelOmega, double maxAccel, double[] result) {
    double effectiveLimit = Math.min(maxAccel, MAX_FRICTION_ACCEL);
    double linearMag = Math.hypot(accelX, accelY);
    // Convert angular acceleration to equivalent linear at wheel radius
    double angularContribution = Math.abs(accelOmega) * DRIVE_BASE_RADIUS;

    // Combined acceleration magnitude
    double combinedAccel = Math.hypot(linearMag, angularContribution);

    // If under the limit, no scaling needed
    if (combinedAccel <= effectiveLimit) {
      result[0] = accelX;
      result[1] = accelY;
      result[2] = accelOmega;
      return;
    }

    // Scale all components proportionally to stay within friction circle
    double scale = effectiveLimit / combinedAccel;
    result[0] = accelX * scale;
    result[1] = accelY * scale;
    result[2] = accelOmega * scale;
  }

  /**
   * Applies jerk limit using a combined vector for linear (vx, vy) and independent for omega.
   *
   * <p>Limits the rate of change of acceleration. Linear jerk is constrained as a single vector
   * magnitude (sqrt(jerkX^2 + jerkY^2) <= maxLinearJerk), ensuring direction-independent behavior.
   * Angular jerk is clamped independently.
   */
  private static void applyJerkLimit(
      double accelX,
      double accelY,
      double accelOmega,
      double prevAccelX,
      double prevAccelY,
      double prevAccelOmega,
      double dt,
      double maxLinearJerk,
      double maxOmegaJerk,
      double[] result) {

    // Linear jerk as combined vector
    double jerkX = (accelX - prevAccelX) / dt;
    double jerkY = (accelY - prevAccelY) / dt;
    double jerkMag = Math.hypot(jerkX, jerkY);

    if (jerkMag > maxLinearJerk) {
      double scale = maxLinearJerk / jerkMag;
      result[0] = prevAccelX + jerkX * scale * dt;
      result[1] = prevAccelY + jerkY * scale * dt;
    } else {
      result[0] = accelX;
      result[1] = accelY;
    }

    // Angular jerk independently
    double jerkOmega = (accelOmega - prevAccelOmega) / dt;
    if (Math.abs(jerkOmega) > maxOmegaJerk) {
      result[2] = prevAccelOmega + Math.copySign(maxOmegaJerk, jerkOmega) * dt;
    } else {
      result[2] = accelOmega;
    }
  }

  /**
   * Normalizes speeds so no swerve module exceeds max velocity.
   *
   * <p>Each module's speed is the vector sum of translation and rotation. The worst case is when
   * they add constructively, so we check: translation + abs(omega) * radius
   *
   * @param speeds The chassis speeds to normalize
   * @return Normalized speeds where no module exceeds max velocity
   */
  public static ChassisSpeeds normalizeSpeeds(ChassisSpeeds speeds) {
    double translationSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double maxModuleSpeed =
        translationSpeed + Math.abs(speeds.omegaRadiansPerSecond) * DRIVE_BASE_RADIUS;

    if (maxModuleSpeed > MAX_VELOCITY) {
      return speeds.times(MAX_VELOCITY / maxModuleSpeed);
    }
    return speeds;
  }

  /**
   * Integrates velocity with physics-based acceleration limits.
   *
   * <p>This is the main entry point for the acceleration limiter. It calculates the acceleration
   * needed to reach the desired velocity, applies motor and friction limits, then integrates to get
   * the next velocity.
   *
   * <p>Optimized to minimize object allocations by using primitive operations internally.
   *
   * @param current Current velocity (field-centric)
   * @param desired Desired velocity (field-centric)
   * @param dt Time step in seconds
   * @return Limited velocity after integration
   */
  public static ChassisSpeeds integrateVelocity(
      ChassisSpeeds current, ChassisSpeeds desired, double dt) {
    return integrateVelocity(current, desired, dt, MAX_FRICTION_ACCEL);
  }

  /**
   * Integrates velocity with physics-based acceleration limits and an external acceleration cap.
   *
   * <p>Same as {@link #integrateVelocity(ChassisSpeeds, ChassisSpeeds, double)} but applies an
   * additional acceleration limit (e.g., for shoot-mode driving). The effective limit is the
   * minimum of the external cap and the physics-based friction limit.
   *
   * @param current Current velocity (field-centric)
   * @param desired Desired velocity (field-centric)
   * @param dt Time step in seconds
   * @param maxAccel Maximum allowed acceleration in m/s^2 (clamped to friction limit)
   * @return Limited velocity after integration
   */
  public static ChassisSpeeds integrateVelocity(
      ChassisSpeeds current, ChassisSpeeds desired, double dt, double maxAccel) {
    return integrateVelocity(current, desired, dt, maxAccel, Double.MAX_VALUE, Double.MAX_VALUE);
  }

  /**
   * Integrates velocity with physics-based acceleration and jerk limits.
   *
   * <p>Same as {@link #integrateVelocity(ChassisSpeeds, ChassisSpeeds, double, double)} but also
   * applies jerk limiting (rate of change of acceleration). Linear jerk (vx, vy) is limited as a
   * combined vector magnitude for direction-independent behavior. Angular jerk is limited
   * independently.
   *
   * @param current Current velocity (field-centric)
   * @param desired Desired velocity (field-centric)
   * @param dt Time step in seconds
   * @param maxAccel Maximum allowed acceleration in m/s^2 (clamped to friction limit)
   * @param maxLinearJerk Maximum linear jerk in m/s^3 (combined vx/vy vector magnitude)
   * @param maxOmegaJerk Maximum angular jerk in rad/s^3
   * @return Limited velocity after integration
   */
  public static ChassisSpeeds integrateVelocity(
      ChassisSpeeds current,
      ChassisSpeeds desired,
      double dt,
      double maxAccel,
      double maxLinearJerk,
      double maxOmegaJerk) {

    // Guard against zero or negative time step (can happen on first frame)
    if (dt < MIN_DT) {
      dt = MIN_DT;
    }

    // Extract primitives to avoid repeated field access
    double curVx = current.vxMetersPerSecond;
    double curVy = current.vyMetersPerSecond;
    double curOmega = current.omegaRadiansPerSecond;

    // Calculate wanted acceleration: (desired - current) / dt
    double accelX = (desired.vxMetersPerSecond - curVx) / dt;
    double accelY = (desired.vyMetersPerSecond - curVy) / dt;
    double accelOmega = (desired.omegaRadiansPerSecond - curOmega) / dt;

    // Apply motor torque and friction limits (result stored in ACCEL_RESULT)
    applyLimits(accelX, accelY, accelOmega, curVx, curVy, curOmega, maxAccel, ACCEL_RESULT);

    // Apply jerk limit (uses previous frame's acceleration as baseline)
    applyJerkLimit(
        ACCEL_RESULT[0],
        ACCEL_RESULT[1],
        ACCEL_RESULT[2],
        lastAccelVx,
        lastAccelVy,
        lastAccelOmega,
        dt,
        maxLinearJerk,
        maxOmegaJerk,
        ACCEL_RESULT);

    // Store jerk-limited acceleration for external consumers (e.g., SWM velocity prediction)
    lastAccelVx = ACCEL_RESULT[0];
    lastAccelVy = ACCEL_RESULT[1];
    lastAccelOmega = ACCEL_RESULT[2];

    // Integrate to get next velocity: current + limitedAccel * dt
    double nextVx = curVx + ACCEL_RESULT[0] * dt;
    double nextVy = curVy + ACCEL_RESULT[1] * dt;
    double nextOmega = curOmega + ACCEL_RESULT[2] * dt;

    // Final check: ensure no module exceeds max velocity
    return normalizeSpeeds(new ChassisSpeeds(nextVx, nextVy, nextOmega));
  }
}
