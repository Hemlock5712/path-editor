package frc.team5712.path.core;

/**
 * A point the path passes through.
 *
 * @param xMeters field x
 * @param yMeters field y
 * @param tangentRadians direction of travel through this point, or NaN to choose it automatically
 *     (interior points: the parabola through the neighbours; ends: mirrored from the next point)
 * @param tangentScale how hard the curve leans on the tangent: the end speed |p'| of each
 *     neighbouring spline piece is {@code tangentScale ×} that piece's estimated arc length. 1 is the
 *     default; above 1 holds the direction longer, below 1 turns sooner.
 * @param headingRadians the robot's heading here (a swerve's heading is its own, not the direction of
 *     travel), or NaN for a pass-through point. {@link Path#build} turns it into a rotation target at
 *     this waypoint.
 */
public record Waypoint(double xMeters, double yMeters, double tangentRadians, double tangentScale, double headingRadians) {
  public Waypoint {
    if (!Double.isFinite(xMeters) || !Double.isFinite(yMeters)) {
      throw new IllegalArgumentException("waypoint position must be finite");
    }
    if (!(tangentScale > 0.0) || !Double.isFinite(tangentScale)) {
      throw new IllegalArgumentException("tangentScale must be positive, got " + tangentScale);
    }
    if (Double.isInfinite(tangentRadians) || Double.isInfinite(headingRadians)) {
      throw new IllegalArgumentException("tangent and heading must be finite or NaN");
    }
  }

  /** A pass-through waypoint (no heading). */
  public Waypoint(double xMeters, double yMeters, double tangentRadians, double tangentScale) {
    this(xMeters, yMeters, tangentRadians, tangentScale, Double.NaN);
  }

  /** A waypoint with an automatic tangent. */
  public static Waypoint at(double xMeters, double yMeters) {
    return new Waypoint(xMeters, yMeters, Double.NaN, 1.0);
  }

  /** A waypoint with a fixed direction of travel. */
  public static Waypoint at(double xMeters, double yMeters, double tangentRadians) {
    return new Waypoint(xMeters, yMeters, tangentRadians, 1.0);
  }

  public Waypoint withTangentScale(double scale) {
    return new Waypoint(xMeters, yMeters, tangentRadians, scale, headingRadians);
  }

  /** The same point with the robot's heading set here (NaN = pass-through). */
  public Waypoint withHeading(double radians) {
    return new Waypoint(xMeters, yMeters, tangentRadians, tangentScale, radians);
  }

  public boolean hasTangent() {
    return !Double.isNaN(tangentRadians);
  }

  public boolean hasHeading() {
    return !Double.isNaN(headingRadians);
  }

  public Vec2 position() {
    return new Vec2(xMeters, yMeters);
  }
}
