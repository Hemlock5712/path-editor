package frc.robot.utils.path;

import frc.robot.commands.AccelerationLimiter;

/**
 * Velocity and acceleration constraints for path velocity profiling.
 *
 * <p>Uses a builder pattern for configuration. Defaults are derived from the robot's physical
 * limits (AccelerationLimiter constants).
 */
public final class VelocityConstraints {

  private final double maxVelocity;
  private final double maxAcceleration;
  private final double maxDeceleration;
  private final double startVelocity;
  private final double endVelocity;
  private final double curvatureFrictionFraction;

  private VelocityConstraints(
      double maxVelocity,
      double maxAcceleration,
      double maxDeceleration,
      double startVelocity,
      double endVelocity,
      double curvatureFrictionFraction) {
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    this.maxDeceleration = maxDeceleration;
    this.startVelocity = startVelocity;
    this.endVelocity = endVelocity;
    this.curvatureFrictionFraction = curvatureFrictionFraction;
  }

  /** Creates default constraints from the robot's physical limits. */
  public static VelocityConstraints defaults() {
    return new VelocityConstraints(
        AccelerationLimiter.MAX_VELOCITY,
        AccelerationLimiter.MAX_FRICTION_ACCEL,
        AccelerationLimiter.MAX_FRICTION_ACCEL,
        0,
        0,
        0.9);
  }

  public VelocityConstraints withMaxVelocity(double maxVelocity) {
    return new VelocityConstraints(
        maxVelocity,
        this.maxAcceleration,
        this.maxDeceleration,
        this.startVelocity,
        this.endVelocity,
        this.curvatureFrictionFraction);
  }

  public VelocityConstraints withMaxAcceleration(double maxAcceleration) {
    return new VelocityConstraints(
        this.maxVelocity,
        maxAcceleration,
        this.maxDeceleration,
        this.startVelocity,
        this.endVelocity,
        this.curvatureFrictionFraction);
  }

  public VelocityConstraints withMaxDeceleration(double maxDeceleration) {
    return new VelocityConstraints(
        this.maxVelocity,
        this.maxAcceleration,
        maxDeceleration,
        this.startVelocity,
        this.endVelocity,
        this.curvatureFrictionFraction);
  }

  public VelocityConstraints withStartVelocity(double startVelocity) {
    return new VelocityConstraints(
        this.maxVelocity,
        this.maxAcceleration,
        this.maxDeceleration,
        startVelocity,
        this.endVelocity,
        this.curvatureFrictionFraction);
  }

  public VelocityConstraints withEndVelocity(double endVelocity) {
    return new VelocityConstraints(
        this.maxVelocity,
        this.maxAcceleration,
        this.maxDeceleration,
        this.startVelocity,
        endVelocity,
        this.curvatureFrictionFraction);
  }

  public VelocityConstraints withCurvatureFrictionFraction(double fraction) {
    return new VelocityConstraints(
        this.maxVelocity,
        this.maxAcceleration,
        this.maxDeceleration,
        this.startVelocity,
        this.endVelocity,
        fraction);
  }

  public double getMaxVelocity() {
    return maxVelocity;
  }

  public double getMaxAcceleration() {
    return maxAcceleration;
  }

  public double getMaxDeceleration() {
    return maxDeceleration;
  }

  public double getStartVelocity() {
    return startVelocity;
  }

  public double getEndVelocity() {
    return endVelocity;
  }

  public double getCurvatureFrictionFraction() {
    return curvatureFrictionFraction;
  }
}
