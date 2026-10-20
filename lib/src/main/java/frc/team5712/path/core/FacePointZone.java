package frc.team5712.path.core;

/**
 * A stretch of path where the robot faces a field point (a goal, a game piece) instead of following
 * the rotation targets. The heading eases in over {@code blendMeters} after the zone starts and eases
 * back out over its last {@code blendMeters}, so it never jumps.
 *
 * @param fromWaypoint waypoint-relative start
 * @param toWaypoint waypoint-relative end
 * @param point the point to face
 * @param blendMeters length of each ease, clamped to half the zone
 */
public record FacePointZone(double fromWaypoint, double toWaypoint, Vec2 point, double blendMeters) {
  public static final double DEFAULT_BLEND_METERS = 0.5;

  public FacePointZone {
    if (!(toWaypoint > fromWaypoint)) {
      throw new IllegalArgumentException("face-point zone must end after it starts");
    }
    if (!(blendMeters >= 0.0) || !Double.isFinite(blendMeters)) {
      throw new IllegalArgumentException("blendMeters must be >= 0");
    }
  }
}
