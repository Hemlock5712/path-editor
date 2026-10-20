package frc.team5712.path.core;

/**
 * A robot heading to reach at a place on the path. Headings between targets follow a minimum-jerk
 * blend; before the first target the first heading is held, and after the last the last.
 *
 * <p>The place is either a waypoint-relative position ({@code 1.5} = halfway along the arc between
 * waypoints 1 and 2), which survives geometry edits, or a distance along the path in metres. Exactly
 * one of the two is set; the other is NaN.
 */
public record RotationTarget(double waypoint, double meters, double headingRadians) {
  public RotationTarget {
    if (Double.isNaN(waypoint) == Double.isNaN(meters)) {
      throw new IllegalArgumentException("set exactly one of waypoint or meters");
    }
    if (!Double.isFinite(headingRadians)) {
      throw new IllegalArgumentException("heading must be finite");
    }
  }

  public static RotationTarget atWaypoint(double waypoint, double headingRadians) {
    return new RotationTarget(waypoint, Double.NaN, headingRadians);
  }

  public static RotationTarget atMeters(double meters, double headingRadians) {
    return new RotationTarget(Double.NaN, meters, headingRadians);
  }

  public boolean byWaypoint() {
    return !Double.isNaN(waypoint);
  }
}
