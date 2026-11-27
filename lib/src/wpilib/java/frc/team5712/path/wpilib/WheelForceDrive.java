package frc.team5712.path.wpilib;

import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * A drivetrain that takes wheel forces next to the velocity: the feed-forward (see {@link
 * FollowPathCommand#withWheelForces}). With CTRE's swerve it is one method on the drivetrain:
 *
 * <pre>
 * private final SwerveRequest.ApplyFieldVelocity follow =
 *     new SwerveRequest.ApplyFieldVelocity().withDriveRequestType(DriveRequestType.Velocity);
 *
 * public void driveFieldVelocity(ChassisVelocities velocity, double[] forcesX, double[] forcesY) {
 *   setControl(follow.withVelocity(velocity)
 *       .withWheelForceFeedforwardsX(forcesX).withWheelForceFeedforwardsY(forcesY));
 * }
 * </pre>
 */
@FunctionalInterface
public interface WheelForceDrive {
  /**
   * @param fieldVelocity the field-relative velocity to drive at
   * @param forcesXNewtons each wheel's push along the field's x, in the drivetrain's module order
   * @param forcesYNewtons each wheel's push along the field's y, in the same order
   */
  void drive(ChassisVelocities fieldVelocity, double[] forcesXNewtons, double[] forcesYNewtons);
}
