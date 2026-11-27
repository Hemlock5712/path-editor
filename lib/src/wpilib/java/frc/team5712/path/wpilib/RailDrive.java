package frc.team5712.path.wpilib;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Vec2;
import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.follow.PieceAssist;
import frc.team5712.path.follow.Rail;
import frc.team5712.path.follow.RailFollower;
import frc.team5712.path.follow.WheelForces;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Driver-assist driving with WPILib's types: holding the robot to a {@link Rail} while the driver drives along
 * it, and nudging the intake onto a game piece ({@link PieceAssist}). The driver's sticks go in as a
 * field-relative velocity; the velocity to command comes out, already through the limiter teleop and the paths
 * use, with the wheel forces to send with it.
 *
 * <pre>
 * RailDrive assist = new RailDrive(followerConfig);                 // robot init
 * assist.reset(drivetrain.getFieldVelocity());                      // when the command starts
 *
 * // every loop, one of:
 * ChassisVelocities command = assist.onRail(rail, pose, fromTheSticks, maxSpeed, dt);
 * ChassisVelocities command = assist.towardPiece(pieces, pose, robotToIntake, fromTheSticks, maxSpeed, dt);
 *
 * double[][] forces = assist.wheelForces(drivetrain.getModuleLocations(), pose.getRotation());
 * drivetrain.driveFieldVelocity(command, forces[0], forces[1]);
 * </pre>
 *
 * Use one frame throughout (the pose's): the rail, the pieces and the sticks' velocity are all in it.
 */
public final class RailDrive {
  private final FollowerConfig config;
  private final RailFollower follower;
  private WheelForces forces;
  private Optional<Translation2d> chosenPiece = Optional.empty();

  public RailDrive(FollowerConfig config) {
    this.config = config;
    this.follower = new RailFollower(config);
  }

  /** A rail through {@code point} running {@code direction}, heading free: add {@code withHeading} or {@code between}. */
  public static Rail rail(Translation2d point, Rotation2d direction) {
    return Rail.through(point.getX(), point.getY(), direction.getRadians());
  }

  /** Starts from the measured velocity, so a robot already moving isn't asked to stop in one loop. */
  public void reset(ChassisVelocities measured) {
    follower.reset(Conversions.toFieldSpeeds(measured));
    chosenPiece = Optional.empty();
  }

  /** One loop held to {@code rail}: the velocity to command. */
  public ChassisVelocities onRail(Rail rail, Pose2d pose, ChassisVelocities driver, double maxSpeed, double dt) {
    return Conversions.toChassisVelocities(
        follower.step(rail, Conversions.toPose(pose), Conversions.toFieldSpeeds(driver), maxSpeed, dt));
  }

  /**
   * One loop of intake assist: the driver's velocity, plus a sideways nudge that lines the intake up on the
   * piece they are heading for. With no such piece it is the driver's velocity through the limiter, as teleop.
   *
   * @param pieces the game pieces in view, on the field
   * @param robotToIntake where the intake's mouth is on the robot
   */
  public ChassisVelocities towardPiece(
      List<Translation2d> pieces, Pose2d robot, Transform2d robotToIntake, ChassisVelocities driver,
      double maxSpeed, double dt) {
    return towardPiece(pieces, robot, robotToIntake, driver, maxSpeed, dt, PieceAssist.EAGERNESS);
  }

  /**
   * {@link #towardPiece(List, Pose2d, Transform2d, ChassisVelocities, double, double)} with the team's own
   * eagerness: how hard the nudge turns the driver toward the piece ({@link PieceAssist#EAGERNESS} is 1.5;
   * 1 lines the intake up only on arrival, more lines it up sooner and turns the robot's travel further).
   */
  public ChassisVelocities towardPiece(
      List<Translation2d> pieces, Pose2d robot, Transform2d robotToIntake, ChassisVelocities driver,
      double maxSpeed, double dt, double eagerness) {
    FieldSpeeds sticks = Conversions.toFieldSpeeds(driver);
    Pose2d intake = robot.transformBy(robotToIntake);
    // stay with the piece chosen last loop unless another is clearly more in line with the stick
    List<Vec2> points = new ArrayList<>(pieces.size());
    for (Translation2d p : pieces) {
      points.add(new Vec2(p.getX(), p.getY()));
    }
    Optional<Vec2> picked = PieceAssist.choose(points, new Vec2(intake.getX(), intake.getY()), sticks,
        chosenPiece.map(p -> new Vec2(p.getX(), p.getY())));
    chosenPiece = picked.map(v -> pieces.get(points.indexOf(v)));
    if (chosenPiece.isEmpty()) {
      return Conversions.toChassisVelocities(follower.step(sticks, dt));
    }
    Vec2 piece = new Vec2(chosenPiece.get().getX(), chosenPiece.get().getY());
    // turn the driver's velocity to point the intake (not the robot's centre) at the piece
    return Conversions.toChassisVelocities(
        follower.step(sticks, PieceAssist.nudge(sticks, new Vec2(intake.getX(), intake.getY()), piece,
            eagerness), dt));
  }

  /** The piece the driver is heading for, if any, with no memory of the last choice ({@link PieceAssist#choose}). */
  public static Optional<Translation2d> choosePiece(
      List<Translation2d> pieces, Translation2d intake, ChassisVelocities driver) {
    List<Vec2> points = new ArrayList<>(pieces.size());
    for (Translation2d p : pieces) {
      points.add(new Vec2(p.getX(), p.getY()));
    }
    Optional<Vec2> best =
        PieceAssist.choose(points, new Vec2(intake.getX(), intake.getY()), Conversions.toFieldSpeeds(driver));
    return best.map(v -> pieces.get(points.indexOf(v)));
  }

  /** The piece the last {@link #towardPiece} was nudging toward, if any. */
  public Optional<Translation2d> chosenPiece() {
    return chosenPiece;
  }

  /** What the rail wanted on the last loop, before the limiter (for logging and tests). */
  public ChassisVelocities wanted() {
    return Conversions.toChassisVelocities(follower.wanted());
  }

  /**
   * Each wheel's push for the last loop, for a drivetrain that takes the feed-forward ({@link WheelForceDrive}):
   * {@code [0]} the forces along x and {@code [1]} along y, in newtons.
   *
   * @param moduleLocations the modules' positions, in the drivetrain's order
   * @param heading the robot's heading
   */
  public double[][] wheelForces(Translation2d[] moduleLocations, Rotation2d heading) {
    if (forces == null || forces.moduleCount() != moduleLocations.length) {
      double[] x = new double[moduleLocations.length];
      double[] y = new double[moduleLocations.length];
      for (int i = 0; i < x.length; i++) {
        x[i] = moduleLocations[i].getX();
        y[i] = moduleLocations[i].getY();
      }
      forces = new WheelForces(config.robot(), x, y);
    }
    return forces.forAcceleration(follower.commandedAcceleration(), heading.getRadians());
  }
}
