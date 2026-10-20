package frc.robot.utils;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Write every goal once, for the blue alliance; on red, flip it. WPILib 2027 puts (0, 0) at the center of the
 * field, so turning the field half a turn about its center lands red exactly where blue was: x and y change sign,
 * and the robot faces the other way.
 */
public final class AllianceFlip {
  private AllianceFlip() {}

  /** True when the driver station says red. Unknown counts as blue. */
  public static boolean isRed() {
    return MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
  }

  /** The blue pose, flipped to the red side when we are red. */
  public static Pose2d apply(Pose2d bluePose) {
    if (!isRed()) {
      return bluePose;
    }
    // A field that is the same after a half turn (most recent games). For a mirror-image field,
    // flip only x and mirror the heading instead.
    return new Pose2d(-bluePose.getX(), -bluePose.getY(), bluePose.getRotation().plus(Rotation2d.PI));
  }
}
