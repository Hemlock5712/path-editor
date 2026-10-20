package frc.team5712.path.core;

/**
 * A stretch of path with lower limits. Either limit may be +∞ (no extra limit).
 *
 * @param fromWaypoint waypoint-relative start
 * @param toWaypoint waypoint-relative end
 * @param maxMetersPerSecond speed cap inside the zone
 * @param maxMetersPerSecondSquared acceleration and braking cap inside the zone
 */
public record ConstraintZone(
    double fromWaypoint,
    double toWaypoint,
    double maxMetersPerSecond,
    double maxMetersPerSecondSquared) {
  public ConstraintZone {
    if (!(toWaypoint > fromWaypoint)) {
      throw new IllegalArgumentException("constraint zone must end after it starts");
    }
    if (!(maxMetersPerSecond > 0.0) || !(maxMetersPerSecondSquared > 0.0)) {
      throw new IllegalArgumentException("zone limits must be positive");
    }
  }
}
