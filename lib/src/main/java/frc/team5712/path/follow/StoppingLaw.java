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

  /**
   * How hard the law is slowing the robot (a positive number, or 0): the acceleration to feed forward while it
   * governs. On the ease-in line {@code v = k·d} the distance shrinks at v, so the speed falls at {@code k·v};
   * further out, on the braking curve, it falls at the braking rate, which is the smaller of the two there. So
   * it is {@code min(a, k·v)} of the speed last commanded, a number that fades smoothly to nothing as the robot
   * stops: a pose that wobbles while the robot stands on its goal asks for next to nothing. At the top speed,
   * or while the command is still coming up to the law's speed, there is nothing to slow.
   *
   * @param lawSpeed what the law asks for now
   * @param commandSpeed what was last commanded
   * @param topSpeed the law's top speed
   */
  public static double slowing(
      double lawSpeed, double commandSpeed, double decel, double easeIn, double topSpeed) {
    if (commandSpeed < lawSpeed - 1e-9 || lawSpeed >= topSpeed - 1e-9) {
      return 0.0;
    }
    return Math.min(decel, easeIn * commandSpeed);
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
