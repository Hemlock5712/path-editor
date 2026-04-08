package frc.robot.utils.path;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.AccelerationLimiter;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Factory methods for common {@link RotationSupplier} strategies.
 *
 * <p>All implementations use the stopping-omega pattern from DriveToPointUtils: the angular
 * velocity is limited to what allows stopping at the target heading using the available angular
 * deceleration.
 */
public final class RotationSuppliers {

  /** Maximum angular deceleration in rad/s^2 (friction-limited). */
  private static final double MAX_ANGULAR_DECEL =
      AccelerationLimiter.MAX_FRICTION_ACCEL / AccelerationLimiter.DRIVE_BASE_RADIUS;

  /** Hardware maximum angular velocity in rad/s. */
  private static final double HARDWARE_MAX_OMEGA =
      AccelerationLimiter.MAX_VELOCITY / AccelerationLimiter.DRIVE_BASE_RADIUS;

  /**
   * Conservative fraction of MAX_ANGULAR_DECEL for stopping-profile planning. Slightly below the
   * default 30% rotation budget to account for AccelerationLimiter lag and jerk limiting.
   */
  private static final double DECEL_BUDGET_FACTOR = 0.25;

  /** Max omega achievable under default 30% rotation budget. */
  private static final double BUDGET_MAX_OMEGA =
      0.30 * AccelerationLimiter.MAX_FRICTION_ACCEL / AccelerationLimiter.DRIVE_BASE_RADIUS;

  private RotationSuppliers() {}

  /**
   * Aligns the robot heading with the path tangent direction (face the direction of travel).
   *
   * @return A rotation supplier that tracks the path tangent heading
   */
  public static RotationSupplier faceForward() {
    return (robotPose, pathS, pathTangent) -> {
      double targetHeading = Math.atan2(pathTangent.getY(), pathTangent.getX());
      double angleError =
          MathUtil.angleModulus(targetHeading - robotPose.getRotation().getRadians());
      return angleErrorToOmega(angleError);
    };
  }

  /**
   * Aims the robot at a fixed field point (e.g., the hub/goal) while driving the path.
   *
   * @param target The field point to face toward
   * @return A rotation supplier that tracks the target point
   */
  public static RotationSupplier facePoint(Translation2d target) {
    return (robotPose, pathS, pathTangent) -> {
      Translation2d toTarget = target.minus(robotPose.getTranslation());
      double targetHeading = Math.atan2(toTarget.getY(), toTarget.getX());
      double angleError =
          MathUtil.angleModulus(targetHeading - robotPose.getRotation().getRadians());
      return angleErrorToOmega(angleError);
    };
  }

  /**
   * Maintains a fixed heading throughout the path.
   *
   * @param heading The heading to hold
   * @return A rotation supplier that holds the given heading
   */
  public static RotationSupplier holdHeading(Rotation2d heading) {
    return (robotPose, pathS, pathTangent) -> {
      double angleError =
          MathUtil.angleModulus(heading.getRadians() - robotPose.getRotation().getRadians());
      return angleErrorToOmega(angleError);
    };
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
  public static RotationSupplier interpolateAlongPath(
      SplinePath path, List<PathData.HeadingWaypoint> waypoints) {
    if (waypoints.isEmpty()) {
      return (robotPose, pathS, pathTangent) -> 0.0;
    }

    // Convert waypoint indices to arc-length positions at construction time
    List<ArcLengthHeading> resolved = resolveWaypoints(path, waypoints);

    return (robotPose, pathS, pathTangent) -> {
      double targetHeading = interpolateHeading(resolved, pathS);
      double angleError =
          MathUtil.angleModulus(targetHeading - robotPose.getRotation().getRadians());
      return angleErrorToOmega(angleError);
    };
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
  public static RotationSupplier fromZones(
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
          return rz.supplier.getOmega(robotPose, pathS, pathTangent);
        }
      }
      return defaultSupplier.getOmega(robotPose, pathS, pathTangent);
    };
  }

  /**
   * Converts a fractional waypoint index to an arc-length position on the path.
   *
   * @param path The spline path
   * @param waypointIndex Fractional index (e.g., 1.5 = halfway between control points 1 and 2)
   * @return Arc-length in meters
   */
  private static double waypointIndexToArcLength(SplinePath path, double waypointIndex) {
    int numSegments = path.getNumSegments(); // N-1 for N control points
    int numCPs = numSegments + 1;
    double frac = waypointIndex / (numCPs - 1);
    return frac * path.getTotalLength();
  }

  // ---- Internal helpers ----

  /**
   * Converts angle error to a stopping-limited angular velocity.
   *
   * <p>Uses the kinematic equation: omega = sqrt(2 * alpha_max * |error|), capped at hardware max.
   * This ensures the robot can decelerate to zero angular velocity by the time it reaches the
   * target heading.
   *
   * @param angleError Signed angle error in radians (positive = counterclockwise to target)
   * @return Signed angular velocity in rad/s
   */
  static double angleErrorToOmega(double angleError) {
    double absError = Math.abs(angleError);
    if (absError < 0.005) {
      return 0.0;
    }
    double stoppingOmega = Math.sqrt(2.0 * MAX_ANGULAR_DECEL * DECEL_BUDGET_FACTOR * absError);
    double omega = Math.min(stoppingOmega, BUDGET_MAX_OMEGA);
    return Math.copySign(omega, angleError);
  }

  /** Resolved waypoint with arc-length position instead of control point index. */
  private record ArcLengthHeading(double s, double headingRadians) {}

  /**
   * Converts heading waypoints from fractional control point indices to arc-length positions.
   *
   * <p>Uses path.getClosestPoint() to find the arc-length at each control point, then interpolates
   * for fractional indices. This is done once at construction time.
   */
  private static List<ArcLengthHeading> resolveWaypoints(
      SplinePath path, List<PathData.HeadingWaypoint> waypoints) {

    // Get arc-length at each integer control point index
    List<Translation2d> controlPoints = path.getControlPoints();
    double[] controlPointS = new double[controlPoints.size()];
    controlPointS[0] = 0;
    controlPointS[controlPoints.size() - 1] = path.getTotalLength();
    for (int i = 1; i < controlPoints.size() - 1; i++) {
      controlPointS[i] = path.getClosestPoint(controlPoints.get(i)).s();
    }

    // Convert each waypoint's fractional index to arc-length
    List<ArcLengthHeading> resolved = new ArrayList<>();
    for (PathData.HeadingWaypoint wp : waypoints) {
      double idx = wp.waypointIndex();
      int lo = Math.max(0, Math.min((int) idx, controlPoints.size() - 1));
      int hi = Math.min(lo + 1, controlPoints.size() - 1);
      double frac = idx - lo;

      double s = controlPointS[lo] + frac * (controlPointS[hi] - controlPointS[lo]);
      resolved.add(new ArcLengthHeading(s, wp.heading().getRadians()));
    }

    // Sort by arc-length for binary search at runtime
    resolved.sort(Comparator.comparingDouble(ArcLengthHeading::s));
    return List.copyOf(resolved);
  }

  /**
   * Interpolates the target heading at a given arc-length position.
   *
   * <p>Linearly interpolates between adjacent waypoints using shortest-angle wrapping. Before the
   * first waypoint, holds the first heading; after the last, holds the last heading.
   */
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
