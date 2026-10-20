package frc.team5712.path.core;

import java.util.List;

/**
 * Everything a path file says: the authored inputs, before any maths. {@link Path#build} turns it
 * into geometry, a heading profile and a velocity profile.
 *
 * @param waypoints at least two, no two consecutive ones at the same place
 * @param rotationTargets headings along the path; empty = face the direction of travel
 * @param facePointZones stretches where the robot faces a point
 * @param constraintZones stretches with lower limits
 * @param maxMetersPerSecond path-wide speed cap (+∞ = only the robot's limits)
 * @param maxMetersPerSecondSquared path-wide acceleration and braking cap (+∞ = grip and motors only)
 * @param startMetersPerSecond speed along the path at the start
 * @param endMetersPerSecond speed along the path at the end (above 0 = a rolling hand-off)
 */
public record PathSpec(
    List<Waypoint> waypoints,
    List<RotationTarget> rotationTargets,
    List<FacePointZone> facePointZones,
    List<ConstraintZone> constraintZones,
    double maxMetersPerSecond,
    double maxMetersPerSecondSquared,
    double startMetersPerSecond,
    double endMetersPerSecond) {

  public PathSpec {
    waypoints = List.copyOf(waypoints);
    rotationTargets = List.copyOf(rotationTargets);
    facePointZones = List.copyOf(facePointZones);
    constraintZones = List.copyOf(constraintZones);
    if (waypoints.size() < 2) {
      throw new IllegalArgumentException("a path needs at least two waypoints");
    }
    if (!(maxMetersPerSecond > 0.0) || !(maxMetersPerSecondSquared > 0.0)) {
      throw new IllegalArgumentException("path limits must be positive");
    }
    if (!(startMetersPerSecond >= 0.0) || !(endMetersPerSecond >= 0.0)) {
      throw new IllegalArgumentException("start and end speeds must be >= 0");
    }
  }

  /** A path through these waypoints with no targets, zones or path-wide caps, from rest to rest. */
  public static PathSpec of(List<Waypoint> waypoints) {
    return new PathSpec(
        waypoints,
        List.of(),
        List.of(),
        List.of(),
        Double.POSITIVE_INFINITY,
        Double.POSITIVE_INFINITY,
        0.0,
        0.0);
  }

  public PathSpec withRotationTargets(List<RotationTarget> targets) {
    return new PathSpec(waypoints, targets, facePointZones, constraintZones,
        maxMetersPerSecond, maxMetersPerSecondSquared, startMetersPerSecond, endMetersPerSecond);
  }

  public PathSpec withFacePointZones(List<FacePointZone> zones) {
    return new PathSpec(waypoints, rotationTargets, zones, constraintZones,
        maxMetersPerSecond, maxMetersPerSecondSquared, startMetersPerSecond, endMetersPerSecond);
  }

  public PathSpec withConstraintZones(List<ConstraintZone> zones) {
    return new PathSpec(waypoints, rotationTargets, facePointZones, zones,
        maxMetersPerSecond, maxMetersPerSecondSquared, startMetersPerSecond, endMetersPerSecond);
  }

  public PathSpec withLimits(double maxSpeed, double maxAccel) {
    return new PathSpec(waypoints, rotationTargets, facePointZones, constraintZones,
        maxSpeed, maxAccel, startMetersPerSecond, endMetersPerSecond);
  }

  public PathSpec withEndSpeeds(double startSpeed, double endSpeed) {
    return new PathSpec(waypoints, rotationTargets, facePointZones, constraintZones,
        maxMetersPerSecond, maxMetersPerSecondSquared, startSpeed, endSpeed);
  }
}
