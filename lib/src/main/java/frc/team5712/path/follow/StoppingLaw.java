package frc.team5712.path.follow;

/**
 * DriveToPose's stopping law (bench/reference/DriveToPose.java): the smallest of the top speed, the
 * speed that can still stop in the distance left after a reaction-time margin, and an ease-in line.
 */
public final class StoppingLaw {
  private StoppingLaw() {}

  /**
   * Speed toward a goal {@code distance} away: {@code min(max, sqrt(2·a·(d − v·τ)), easeIn·d)}, zero
   * inside the deadband.
   */
  public static double speed(
      double distance, double speedNow, double decel, double reactionSeconds, double maxSpeed,
      double easeIn, double deadband) {
    if (distance < deadband) {
      return 0.0;
    }
    double braking = Math.max(0.0, distance - speedNow * reactionSeconds);
    return Math.min(maxSpeed, Math.min(Math.sqrt(2.0 * decel * braking), easeIn * distance));
  }

  /**
   * Speed that still reaches {@code endSpeed} within {@code distance}, after the reaction margin:
   * {@code sqrt(v_end² + 2·a·(d − v·τ))}. With end speed 0 it is the brake part of {@link #speed}.
   */
  public static double brake(double distance, double speedNow, double decel, double reactionSeconds, double endSpeed) {
    double braking = Math.max(0.0, distance - speedNow * reactionSeconds);
    return Math.sqrt(endSpeed * endSpeed + 2.0 * decel * braking);
  }

  /** The same law for turning; returns a signed spin rate toward {@code error}. */
  public static double spin(
      double error, double spinNow, double alpha, double reactionSeconds, double maxOmega,
      double easeIn, double deadband) {
    double angle = Math.abs(error);
    if (angle < deadband) {
      return 0.0;
    }
    double braking = Math.max(0.0, angle - Math.abs(spinNow) * reactionSeconds);
    double rate = Math.min(maxOmega, Math.min(Math.sqrt(2.0 * alpha * braking), easeIn * angle));
    return Math.copySign(rate, error);
  }
}
