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
 */
public record Waypoint(double xMeters, double yMeters, double tangentRadians, double tangentScale) {
  public Waypoint {
    if (!Double.isFinite(xMeters) || !Double.isFinite(yMeters)) {
      throw new IllegalArgumentException("waypoint position must be finite");
    }
    if (!(tangentScale > 0.0) || !Double.isFinite(tangentScale)) {
      throw new IllegalArgumentException("tangentScale must be positive, got " + tangentScale);
    }
    if (Double.isInfinite(tangentRadians)) {
      throw new IllegalArgumentException("tangent must be finite or NaN");
    }
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
    return new Waypoint(xMeters, yMeters, tangentRadians, scale);
  }

  public boolean hasTangent() {
    return !Double.isNaN(tangentRadians);
  }

  public Vec2 position() {
    return new Vec2(xMeters, yMeters);
  }
}
