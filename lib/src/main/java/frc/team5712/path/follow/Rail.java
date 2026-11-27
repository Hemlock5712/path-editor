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
 */
public record Rail(
    double xMeters, double yMeters, double directionRadians, double headingRadians, double startMeters,
    double endMeters) {

  /** An endless rail through a point, running {@code directionRadians}, heading free. */
  public static Rail through(double xMeters, double yMeters, double directionRadians) {
    return new Rail(xMeters, yMeters, directionRadians, Double.NaN, Double.NEGATIVE_INFINITY,
        Double.POSITIVE_INFINITY);
  }

  /** Also hold this heading. */
  public Rail withHeading(double newHeadingRadians) {
    return new Rail(xMeters, yMeters, directionRadians, newHeadingRadians, startMeters, endMeters);
  }

  /** Stop at these along-distances from the point, like the ends of a platform. */
  public Rail between(double newStartMeters, double newEndMeters) {
    return new Rail(xMeters, yMeters, directionRadians, headingRadians, newStartMeters, newEndMeters);
  }

  /** True when the rail holds a heading. */
  public boolean holdsHeading() {
    return !Double.isNaN(headingRadians);
  }
}
