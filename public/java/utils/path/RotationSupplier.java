package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Supplies desired angular velocity during path following.
 *
 * <p>Unlike a simple {@code DoubleSupplier}, this interface receives path context (robot pose,
 * arc-length position, and path tangent) so rotation strategies can be path-aware.
 */
@FunctionalInterface
public interface RotationSupplier {

  /**
   * Returns the desired angular velocity for the robot.
   *
   * @param robotPose Current robot pose (position + heading)
   * @param pathS Current arc-length position on the path (meters)
   * @param pathTangent Unit tangent vector of the path at the projected point
   * @return Desired angular velocity in rad/s (positive = counterclockwise)
   */
  double getOmega(Pose2d robotPose, double pathS, Translation2d pathTangent);
}
