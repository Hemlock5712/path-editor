package frc.team5712.path.wpilib;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Pose;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** Between WPILib's types and the core's, and the alliance as a {@link FieldFrame}. */
public final class Conversions {
  private Conversions() {}

  public static Pose toPose(Pose2d pose) {
    return new Pose(pose.getX(), pose.getY(), pose.getRotation().getRadians());
  }

  public static Pose2d toPose2d(Pose pose) {
    return new Pose2d(pose.xMeters(), pose.yMeters(), new Rotation2d(pose.headingRadians()));
  }

  /** Field-relative velocities in, field speeds out. */
  public static FieldSpeeds toFieldSpeeds(ChassisVelocities fieldRelative) {
    return new FieldSpeeds(fieldRelative.vx, fieldRelative.vy, fieldRelative.omega);
  }

  /** Field speeds in, field-relative velocities out. */
  public static ChassisVelocities toChassisVelocities(FieldSpeeds speeds) {
    return new ChassisVelocities(speeds.vxMetersPerSecond(), speeds.vyMetersPerSecond(), speeds.omegaRadPerSec());
  }

  /**
   * The frame for the alliance the driver station reports now: blue, or {@code redFrame} on red
   * (RED_HALF_TURN for rotationally symmetric fields, RED_MIRROR for mirrored ones). Unknown counts
   * as blue. Read it when the path starts, not when the auto is built.
   */
  public static FieldFrame allianceFrame(FieldFrame redFrame) {
    return MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED ? redFrame : FieldFrame.BLUE;
  }
}
