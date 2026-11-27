package frc.team5712.path.follow;

/**
 * A line on the field the robot is held to, like a train on a rail: under a trench, along a scoring platform, or
 * through a game piece. {@link RailLaw} drives it: across the rail the robot is pulled back onto the line; along
 * it the driver drives.
 *
 * @param xMeters any point on the rail, x
 * @param yMeters and y
 * @param directionRadians which way the rail runs; "along" distances are measured from the point this way
 * @param headingRadians the heading to hold, or NaN to let the driver turn
 * @param startMeters the rail's near end, as an along-distance (−∞ for none)
 * @param endMeters the rail's far end, as an along-distance (+∞ for none)
 * @param pullIn how fast the rail pulls the robot onto the line, as a share of the driver's speed along it
 *     (+∞ for as fast as the robot can). At 0.5 a driver at 4 m/s along the rail is pulled in at 2 m/s at most:
 *     the robot slides on at a fixed angle, {@code atan(pullIn)}, and joins the line {@code how far off ÷
 *     pullIn} further along, so the driver chooses where it joins by when they press the button. See {@link
 *     RailLaw} for what the driver's own push toward the line, and a released stick, do
 */
public record Rail(
    double xMeters, double yMeters, double directionRadians, double headingRadians, double startMeters,
    double endMeters, double pullIn) {

  /** A rail that pulls the robot on as fast as it can. */
  public Rail(double xMeters, double yMeters, double directionRadians, double headingRadians, double startMeters,
      double endMeters) {
    this(xMeters, yMeters, directionRadians, headingRadians, startMeters, endMeters, Double.POSITIVE_INFINITY);
  }

  /** An endless rail through a point, running {@code directionRadians}, heading free. */
  public static Rail through(double xMeters, double yMeters, double directionRadians) {
    return new Rail(xMeters, yMeters, directionRadians, Double.NaN, Double.NEGATIVE_INFINITY,
        Double.POSITIVE_INFINITY);
  }

  /** Also hold this heading. */
  public Rail withHeading(double newHeadingRadians) {
    return new Rail(xMeters, yMeters, directionRadians, newHeadingRadians, startMeters, endMeters, pullIn);
  }

  /** Stop at these along-distances from the point, like the ends of a platform. */
  public Rail between(double newStartMeters, double newEndMeters) {
    return new Rail(xMeters, yMeters, directionRadians, headingRadians, newStartMeters, newEndMeters, pullIn);
  }

  /**
   * Pulls the robot onto the line at no more than this share of the driver's speed along the rail (0.5: at
   * half their speed, an angle of 27°). A feel number: set it with a driver on the robot.
   */
  public Rail withPullIn(double shareOfDriverSpeed) {
    if (!(shareOfDriverSpeed > 0.0)) {
      throw new IllegalArgumentException("the pull-in share must be more than 0");
    }
    return new Rail(xMeters, yMeters, directionRadians, headingRadians, startMeters, endMeters,
        shareOfDriverSpeed);
  }

  /** True when the rail holds a heading. */
  public boolean holdsHeading() {
    return !Double.isNaN(headingRadians);
  }
}
