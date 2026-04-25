package frc.robot.utils.path;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Supplies the desired target heading during path following.
 *
 * <p>Unlike a simple {@code DoubleSupplier}, this interface receives path context (robot pose,
 * arc-length position, and path tangent) so rotation strategies can be path-aware.
 *
 * <p>The returned heading is converted to angular velocity by the path follower's heading
 * controller.
 *
 * <p>Static factory methods provide common heading strategies (face forward, face a point, hold a
 * fixed heading, interpolate along path, zone-based dispatch).
 */
@FunctionalInterface
public interface RotationSupplier {

  /**
   * Returns the desired target heading for the robot.
   *
   * @param robotPose Current robot pose (position + heading)
   * @param pathS Current arc-length position on the path (meters)
   * @param pathTangent Unit tangent vector of the path at the projected point
   * @return Target heading in radians
   */
  double getTargetHeading(Pose2d robotPose, double pathS, Translation2d pathTangent);

  /**
   * Aligns the robot heading with the path tangent direction (face the direction of travel).
   *
   * @return A rotation supplier that returns the path tangent heading
   */
  static RotationSupplier faceForward() {
    return (robotPose, pathS, pathTangent) -> Math.atan2(pathTangent.getY(), pathTangent.getX());
  }

  /**
   * Aims the robot at a fixed field point (e.g., the hub/goal) while driving the path.
   *
   * @param target The field point to face toward
   * @return A rotation supplier that returns the heading toward the target point
   */
  static RotationSupplier facePoint(Translation2d target) {
    return (robotPose, pathS, pathTangent) -> {
      Translation2d toTarget = target.minus(robotPose.getTranslation());
      return Math.atan2(toTarget.getY(), toTarget.getX());
    };
  }

  /**
   * Maintains a fixed heading throughout the path.
   *
   * @param heading The heading to hold
   * @return A rotation supplier that returns the fixed heading
   */
  static RotationSupplier holdHeading(Rotation2d heading) {
    return (robotPose, pathS, pathTangent) -> heading.getRadians();
  }

  /**
   * Smoothly interpolates heading between waypoints defined along the path.
   *
   * <p>Heading waypoints use fractional control point indices (e.g., 1.5 = halfway between control
   * points 1 and 2). The heading is linearly interpolated between adjacent waypoints (using
   * shortest-angle wrapping). Before the first waypoint, the first heading is held; after the last,
   * the last heading is held.
   *
   * @param path The spline path (used to convert waypoint indices to arc-length)
   * @param waypoints Heading waypoints with fractional control point indices
   * @return A rotation supplier that interpolates between waypoints
   */
  static RotationSupplier interpolateAlongPath(
      SplinePath path, List<PathData.HeadingWaypoint> waypoints) {
    if (waypoints.isEmpty()) {
      return faceForward();
    }

    // Convert waypoint indices to arc-length positions at construction time
    List<ArcLengthHeading> resolved = resolveWaypoints(path, waypoints);

    return (robotPose, pathS, pathTangent) -> interpolateHeading(resolved, pathS);
  }

  /**
   * Creates a composed rotation supplier that dispatches to {@link #facePoint} within rotation
   * zones and falls back to the heading waypoint strategy outside them.
   *
   * <p>If {@code rotationZones} is empty, this behaves identically to calling {@link
   * #interpolateAlongPath} (or {@link #faceForward} if no heading waypoints exist).
   *
   * @param path The spline path
   * @param headingWaypoints Heading waypoints for the default strategy
   * @param rotationZones Zones where the robot should face a target point
   * @return A composed rotation supplier
   */
  static RotationSupplier fromZones(
      SplinePath path,
      List<PathData.HeadingWaypoint> headingWaypoints,
      List<PathData.RotationZone> rotationZones) {

    // Build the default supplier from heading waypoints
    RotationSupplier defaultSupplier =
        headingWaypoints.isEmpty() ? faceForward() : interpolateAlongPath(path, headingWaypoints);

    if (rotationZones.isEmpty()) {
      return defaultSupplier;
    }

    // Resolve zone boundaries to arc-length at construction time
    record ResolvedZone(double startS, double endS, RotationSupplier supplier) {}
    List<ResolvedZone> resolved = new ArrayList<>();
    for (PathData.RotationZone zone : rotationZones) {
      double startS = waypointIndexToArcLength(path, zone.startWaypointIndex());
      double endS = waypointIndexToArcLength(path, zone.endWaypointIndex());
      RotationSupplier zoneSupplier = facePoint(zone.targetPoint());
      resolved.add(new ResolvedZone(startS, endS, zoneSupplier));
    }
    resolved.sort(Comparator.comparingDouble(ResolvedZone::startS));
    List<ResolvedZone> frozenZones = List.copyOf(resolved);

    return (robotPose, pathS, pathTangent) -> {
      for (ResolvedZone rz : frozenZones) {
        if (pathS >= rz.startS && pathS <= rz.endS) {
          return rz.supplier.getTargetHeading(robotPose, pathS, pathTangent);
        }
      }
      return defaultSupplier.getTargetHeading(robotPose, pathS, pathTangent);
    };
  }

  // ---- Internal helpers ----

  private static double waypointIndexToArcLength(SplinePath path, double waypointIndex) {
    int lo = (int) waypointIndex;
    int hi = lo + 1;
    double frac = waypointIndex - lo;

    double loS = path.getArcLengthAtWaypointIndex(lo);
    double hiS = path.getArcLengthAtWaypointIndex(hi);

    return loS + frac * (hiS - loS);
  }

  /** Resolved waypoint with arc-length position instead of control point index. */
  record ArcLengthHeading(double s, double headingRadians) {}

  private static List<ArcLengthHeading> resolveWaypoints(
      SplinePath path, List<PathData.HeadingWaypoint> waypoints) {

    List<ArcLengthHeading> resolved = new ArrayList<>();
    for (PathData.HeadingWaypoint wp : waypoints) {
      double s = waypointIndexToArcLength(path, wp.waypointIndex());
      resolved.add(new ArcLengthHeading(s, wp.heading().getRadians()));
    }

    // Sort by arc-length for binary search at runtime
    resolved.sort(Comparator.comparingDouble(ArcLengthHeading::s));
    return List.copyOf(resolved);
  }

  private static double interpolateHeading(List<ArcLengthHeading> waypoints, double pathS) {
    // Before first waypoint
    if (pathS <= waypoints.get(0).s()) {
      return waypoints.get(0).headingRadians();
    }

    // After last waypoint
    if (pathS >= waypoints.get(waypoints.size() - 1).s()) {
      return waypoints.get(waypoints.size() - 1).headingRadians();
    }

    // Find bounding waypoints via linear scan (typically < 10 waypoints)
    for (int i = 0; i < waypoints.size() - 1; i++) {
      ArcLengthHeading a = waypoints.get(i);
      ArcLengthHeading b = waypoints.get(i + 1);
      if (pathS >= a.s() && pathS <= b.s()) {
        double range = b.s() - a.s();
        if (range < 1e-9) {
          return a.headingRadians();
        }
        double t = (pathS - a.s()) / range;

        // Shortest-angle interpolation
        double diff = MathUtil.angleModulus(b.headingRadians() - a.headingRadians());
        return MathUtil.angleModulus(a.headingRadians() + t * diff);
      }
    }

    // Fallback (shouldn't reach here)
    return waypoints.get(waypoints.size() - 1).headingRadians();
  }
}
