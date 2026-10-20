package frc.team5712.path.core;

/**
 * A named event at a place on the path, authored in the editor and saved in the path file. The
 * follower fires it once as the robot passes; robot code binds the name to an action or a command.
 *
 * <p>The place is either a waypoint-relative position ({@code 1.5} = halfway along the arc between
 * waypoints 1 and 2), which survives geometry edits, or a distance along the path in metres. Exactly
 * one of the two is set; the other is NaN.
 */
public record PathEvent(String name, double waypoint, double meters) {
  public PathEvent {
    if (name == null || name.isBlank()) {
      throw new IllegalArgumentException("an event needs a name");
    }
    if (Double.isNaN(waypoint) == Double.isNaN(meters)) {
      throw new IllegalArgumentException("set exactly one of atWaypoint or atMeters");
    }
    if (Double.isInfinite(waypoint) || Double.isInfinite(meters)) {
      throw new IllegalArgumentException("event position must be finite");
    }
  }

  public static PathEvent atWaypoint(String name, double waypoint) {
    return new PathEvent(name, waypoint, Double.NaN);
  }

  public static PathEvent atMeters(String name, double meters) {
    return new PathEvent(name, Double.NaN, meters);
  }

  public boolean byWaypoint() {
    return !Double.isNaN(waypoint);
  }

  /** Metres along the path built from {@code geometry}. Waypoint positions clamp to the path; metres don't. */
  public double sMeters(PathGeometry geometry) {
    return byWaypoint() ? geometry.sAtWaypoint(waypoint) : meters;
  }
}
