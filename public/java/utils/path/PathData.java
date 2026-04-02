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
    VelocityConstraints globalConstraints) {

  /**
   * A heading target at a specific arc-length position.
   *
   * @param waypointIndex Fractional index into the control points (e.g., 1.5 = halfway between
   *     points 1 and 2)
   * @param heading Target heading
   */
  public record HeadingWaypoint(double waypointIndex, Rotation2d heading) {}
}
