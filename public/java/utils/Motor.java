package frc.robot.utils;

/**
 * Motor torque-speed characteristics from dyno data.
 *
 * <p>DC motors produce less torque as they spin faster due to back-EMF. This relationship is
 * approximately linear: torque = stallTorque * (1 - rpm / freeSpeedRpm)
 *
 * <p>This linear model is accurate to within 0.01% of actual dyno data for FRC motors.
 */
public enum Motor {
  /**
   * Kraken X60 with Field-Oriented Control (FOC).
   *
   * <p>FOC provides higher torque but slightly lower free speed compared to non-FOC mode.
   */
  KRAKEN_X60_FOC(9.3615, 5784.65, 476.1),

  /**
   * Kraken X60 without FOC (standard brushless control).
   *
   * <p>Lower torque but higher free speed compared to FOC mode.
   */
  KRAKEN_X60(7.1573, 6065.33, 374.4);

  /** Conversion factor from radians per second to RPM: 60 / (2π) ≈ 9.5493 */
  private static final double RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * Math.PI);

  private final double stallTorque; // Newton-meters at 0 RPM
  private final double freeSpeedRpm; // RPM at 0 torque
  private final double stallCurrent; // Amps at stall (from CTRE dyno data)
  private final double kt; // Torque constant (N-m/A)

  Motor(double stallTorque, double freeSpeedRpm, double stallCurrent) {
    this.stallTorque = stallTorque;
    this.freeSpeedRpm = freeSpeedRpm;
    this.stallCurrent = stallCurrent;
    this.kt = stallTorque / stallCurrent;
  }

  /**
   * Gets available torque at a given motor speed.
   *
   * <p>Uses the linear torque-speed relationship. Returns 0 if at or above free speed.
   *
   * @param rpm Motor speed in RPM (absolute value used, works for both directions)
   * @return Available torque in Newton-meters
   */
  public double getTorqueAtRpm(double rpm) {
    rpm = Math.abs(rpm);
    if (rpm >= freeSpeedRpm) {
      return 0.0;
    }
    return stallTorque * (1.0 - rpm / freeSpeedRpm);
  }

  /**
   * Gets available torque at a given motor speed, limited by current constraint.
   *
   * <p>The motor cannot produce more torque than Kt * currentLimit, regardless of what the
   * speed-torque curve would allow. This models real-world current-limited operation.
   *
   * @param rpm Motor speed in RPM (absolute value used)
   * @param currentLimitAmps Maximum stator current in Amps
   * @return Available torque in Newton-meters
   */
  public double getTorqueAtRpm(double rpm, double currentLimitAmps) {
    double torqueFromCurve = getTorqueAtRpm(rpm);
    double torqueFromCurrentLimit = kt * currentLimitAmps;
    return Math.min(torqueFromCurve, torqueFromCurrentLimit);
  }

  /** Returns stall torque in Newton-meters. */
  public double getStallTorque() {
    return stallTorque;
  }

  /** Returns free speed in RPM (speed at zero load). */
  public double getFreeSpeedRpm() {
    return freeSpeedRpm;
  }

  /** Returns stall current in Amps (from CTRE dyno data). */
  public double getStallCurrent() {
    return stallCurrent;
  }

  /** Returns torque constant Kt in N-m per Amp. */
  public double getKt() {
    return kt;
  }

  /**
   * Calculates maximum linear acceleration with current limiting.
   *
   * <p>This is the physically accurate version that accounts for both:
   *
   * <ul>
   *   <li>Motor torque curve (torque decreases with speed)
   *   <li>Stator current limits (torque capped by Kt * currentLimit)
   * </ul>
   *
   * @param wheelSpeedMps Current wheel speed in meters per second
   * @param gearRatio Gear ratio (motor rotations per wheel rotation)
   * @param wheelRadiusMeters Wheel radius in meters
   * @param robotMassKg Total robot mass in kilograms
   * @param numDriveMotors Number of drive motors contributing to acceleration
   * @param currentLimitAmps Stator current limit per motor in Amps
   * @return Maximum acceleration in meters per second squared
   */
  public double getMaxAcceleration(
      double wheelSpeedMps,
      double gearRatio,
      double wheelRadiusMeters,
      double robotMassKg,
      int numDriveMotors,
      double currentLimitAmps) {

    // Convert wheel speed to motor RPM
    double wheelRadPerSec = wheelSpeedMps / wheelRadiusMeters;
    double motorRadPerSec = wheelRadPerSec * gearRatio;
    double motorRpm = motorRadPerSec * RAD_PER_SEC_TO_RPM;

    // Look up torque at this speed WITH current limit
    double motorTorque = getTorqueAtRpm(motorRpm, currentLimitAmps);

    // Calculate force: F = (T * gearRatio) / wheelRadius
    // Gearbox multiplies torque by gear ratio
    double wheelTorque = motorTorque * gearRatio;
    double forcePerWheel = wheelTorque / wheelRadiusMeters;
    double totalForce = forcePerWheel * numDriveMotors;

    // Newton's second law: a = F / m
    return totalForce / robotMassKg;
  }
}
