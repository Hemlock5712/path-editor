package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Complete path data including control points, heading waypoints, and constraints.
 *
 * <p>This is the data model for paths exported from the React path editor and loaded by the robot.
 *
 * @param controlPoints Ordered list of points the path passes through
 * @param headingWaypoints Optional heading targets at specific arc-length positions
 * @param globalConstraints Velocity and acceleration limits for the entire path
 */
public record PathData(
    List<Translation2d> controlPoints,
    List<HeadingWaypoint> headingWaypoints,
    VelocityConstraints globalConstraints,
    List<ConstraintZone> constraintZones,
    List<RotationZone> rotationZones) {

  /** 2026 Reefscape field width in meters. */
  public static final double FIELD_WIDTH = 16.54;

  /** 2026 Reefscape field height in meters. */
  public static final double FIELD_HEIGHT = 8.21;

  /**
   * A heading target at a specific arc-length position.
   *
   * @param waypointIndex Fractional index into the control points (e.g., 1.5 = halfway between
   *     points 1 and 2)
   * @param heading Target heading
   */
  public record HeadingWaypoint(double waypointIndex, Rotation2d heading) {}

  /**
   * A constraint zone that overrides velocity and acceleration limits for a segment of the path.
   *
   * @param startWaypointIndex Index of the waypoint where the zone starts
   * @param endWaypointIndex Index of the waypoint where the zone ends
   * @param maxVelocity Maximum velocity in m/s within this zone
   * @param maxAcceleration Maximum acceleration in m/s² within this zone
   */
  public record ConstraintZone(int startWaypointIndex, int endWaypointIndex, double maxVelocity, double maxAcceleration) {}

  /**
   * A rotation zone that overrides the default heading strategy for a segment of the path.
   *
   * @param id Unique identifier
   * @param startWaypointIndex Fractional waypoint index where the zone starts
   * @param endWaypointIndex Fractional waypoint index where the zone ends
   * @param targetPoint Field point the robot should face within this zone
   */
  public record RotationZone(
      String id,
      double startWaypointIndex,
      double endWaypointIndex,
      Translation2d targetPoint) {}

  /**
   * Returns a new PathData with all coordinates and headings mirrored for the red alliance.
   *
   * <p>Paths are authored for the blue alliance in the editor. Since the Reefscape field is
   * rotationally symmetric, mirroring is a 180° rotation about the field center:
   *
   * <ul>
   *   <li>Control points: (FIELD_WIDTH - x, FIELD_HEIGHT - y)
   *   <li>Headings: rotated by 180°
   *   <li>Constraints: unchanged
   * </ul>
   */
  public PathData mirrorForRedAlliance() {
    List<Translation2d> mirroredPoints =
        controlPoints.stream()
            .map(pt -> new Translation2d(FIELD_WIDTH - pt.getX(), FIELD_HEIGHT - pt.getY()))
            .toList();

    List<HeadingWaypoint> mirroredHeadings =
        headingWaypoints.stream()
            .map(
                hw ->
                    new HeadingWaypoint(
                        hw.waypointIndex(), hw.heading().rotateBy(Rotation2d.fromDegrees(180))))
            .toList();

    List<RotationZone> mirroredRotationZones =
        rotationZones.stream()
            .map(
                rz ->
                    new RotationZone(
                        rz.id(),
                        rz.startWaypointIndex(),
                        rz.endWaypointIndex(),
                        new Translation2d(
                            FIELD_WIDTH - rz.targetPoint().getX(),
                            FIELD_HEIGHT - rz.targetPoint().getY())))
            .toList();

    return new PathData(
        mirroredPoints, mirroredHeadings, globalConstraints, constraintZones, mirroredRotationZones);
  }
}
