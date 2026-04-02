package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Result of projecting a point onto a spline path.
 *
 * @param s Arc length of the closest point on the path (meters)
 * @param point Position of the closest point on the path
 * @param crossTrackError Signed perpendicular distance from the robot to the path (positive = left
 *     of path direction)
 * @param tangent Unit tangent vector at the closest point
 */
public record ProjectionResult(
    double s, Translation2d point, double crossTrackError, Translation2d tangent) {}
