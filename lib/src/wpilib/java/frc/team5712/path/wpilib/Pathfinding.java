package frc.team5712.path.wpilib;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.Pose;
import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.follow.PathFollower;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.wpilib.command2.Subsystem;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * On-the-fly paths around the field's obstacles, for Commands v3 and v2. Build one in robot init, next to
 * the {@link PathLibrary} (which loaded field.json and built the visibility graph):
 *
 * <pre>
 * Pathfinding otf = new Pathfinding(paths, FollowerConfig.defaults(paths.robot()),
 *     drivetrain::getPose, drivetrain::getFieldVelocity, drivetrain::driveFieldVelocity,
 *     FieldFrame.RED_HALF_TURN);
 *
 * Command toSource = otf.to(drivetrain, new Pose2d(-7.6, 3.1, Rotation2d.k180deg));  // blue frame, flipped on red
 * Command toAmp = otf.toPoint(drivetrain, "Amp", Rotation2d.kCCW_90deg);           // an editor named point
 * Command toTag = otf.toTarget(drivetrain, vision::bestScoringPose);              // real field frame, as is
 * Command thenAuto = otf.thenFollow(drivetrain, "ToReef");                       // to the path's start, then the path
 * </pre>
 *
 * Each command plans when it starts, from the robot's measured pose and velocity (a few milliseconds:
 * the only path work at go), snapping a start or goal out of any grown obstacle, follows the plan with a
 * {@link PathFollower}, and plans again if the robot is knocked more than 20 cm off it
 * ({@link PathfindCommand#replanAbove}) or when asked ({@link PathfindCommand#replan}). The alliance is
 * read when the command starts: authored goals (poses, named points, a path's start) are blue-frame and
 * flipped with {@code redFrame} on red; {@link #toTarget} goals are used as given. The {@code V2} methods
 * build the same commands for Commands v2.
 */
public final class Pathfinding {
  final PathLibrary library;
  final FollowerConfig config;
  final Supplier<Pose2d> pose;
  final Supplier<ChassisVelocities> fieldVelocity;
  final Consumer<ChassisVelocities> fieldOutput;
  final FieldFrame redFrame;
  DoubleSupplier clock;
  Supplier<FieldFrame> frame;

  public Pathfinding(PathLibrary library, FollowerConfig config, Supplier<Pose2d> pose,
      Supplier<ChassisVelocities> fieldVelocity, Consumer<ChassisVelocities> fieldOutput, FieldFrame redFrame) {
    this.library = library;
    this.config = config;
    this.pose = pose;
    this.fieldVelocity = fieldVelocity;
    this.fieldOutput = fieldOutput;
    this.redFrame = redFrame;
  }

  /** Every command built from here on reads this clock (default {@code Timer.getTimestamp()}), e.g. in tests. */
  public Pathfinding withClock(DoubleSupplier seconds) {
    this.clock = seconds;
    return this;
  }

  /** Every command built from here on reads the alliance from this (default: the driver station, at start). */
  public Pathfinding withFrame(Supplier<FieldFrame> frameSource) {
    this.frame = frameSource;
    return this;
  }

  private PathfindCommand v3(PathfindCommand c) {
    if (clock != null) {
      c.withClock(clock);
    }
    if (frame != null) {
      c.withFrame(frame);
    }
    return c;
  }

  private PathfindCommandV2 v2(PathfindCommandV2 c) {
    if (clock != null) {
      c.withClock(clock);
    }
    if (frame != null) {
      c.withFrame(frame);
    }
    return c;
  }

  /** Where to go, in the real field frame, given the frame the robot runs in. */
  interface Goal extends Function<FieldFrame, Pose> {}

  static Goal blue(Pose2d goal) {
    Pose p = Conversions.toPose(goal);
    return frame -> frame.fromBlue(p);
  }

  Goal point(String name, Rotation2d heading) {
    double[] xy = library.namedPoint(name);
    Pose p = new Pose(xy[0], xy[1], heading.getRadians());
    return frame -> frame.fromBlue(p);
  }

  static Goal real(Supplier<Pose2d> goal) {
    return frame -> Conversions.toPose(goal.get());
  }

  /** The start of a library path, in the frame it will run in, and the direction it sets off in. */
  static Pose start(Path path, FieldFrame frame) {
    PathSample p = path.sample(0.0);
    return frame.fromBlue(new Pose(p.xMeters(), p.yMeters(), p.headingRadians()));
  }

  static double startTangent(Path path, FieldFrame frame) {
    PathSample p = path.sample(0.0);
    return frame.fromBlue(new Pose(p.xMeters(), p.yMeters(), Math.atan2(p.tangentY(), p.tangentX()))).headingRadians();
  }

  // ------------------------------------------------------------------------------ Commands v3

  /** To an authored pose (blue frame; flipped on red). */
  public PathfindCommand to(Mechanism drive, Pose2d blueGoal) {
    return v3(new PathfindCommand(this, drive, "Pathfind", blue(blueGoal), null, null));
  }

  /** To an editor named point (blue frame; flipped on red), arriving at {@code heading} (blue frame). */
  public PathfindCommand toPoint(Mechanism drive, String pointName, Rotation2d heading) {
    return v3(new PathfindCommand(this, drive, "Pathfind " + pointName, point(pointName, heading), null, null));
  }

  /** To a pose read when it starts and at each re-plan, in the real field frame (not flipped). */
  public PathfindCommand toTarget(Mechanism drive, Supplier<Pose2d> realGoal) {
    return v3(new PathfindCommand(this, drive, "Pathfind", real(realGoal), null, null));
  }

  /**
   * To the start of the library path {@code pathName}, arriving along its start direction at its start speed
   * (a rolling hand-over when the path starts moving), then the path itself. The path's follower is built
   * here, in robot init; its events fire through {@code library.bindEvent}.
   */
  public PathfindCommand thenFollow(Mechanism drive, String pathName) {
    Path path = library.get(pathName);
    FollowPathCommand follow = new FollowPathCommand(pathName, drive, new PathFollower(path, config), pose,
        fieldVelocity, fieldOutput, redFrame).withEvents(library);
    return v3(new PathfindCommand(this, drive, "Pathfind to " + pathName, f -> start(path, f), path, follow));
  }

  // ------------------------------------------------------------------------------ Commands v2

  public PathfindCommandV2 toV2(Subsystem drive, Pose2d blueGoal) {
    return v2(new PathfindCommandV2(this, drive, "Pathfind", blue(blueGoal), null));
  }

  public PathfindCommandV2 toPointV2(Subsystem drive, String pointName, Rotation2d heading) {
    return v2(new PathfindCommandV2(this, drive, "Pathfind " + pointName, point(pointName, heading), null));
  }

  public PathfindCommandV2 toTargetV2(Subsystem drive, Supplier<Pose2d> realGoal) {
    return v2(new PathfindCommandV2(this, drive, "Pathfind", real(realGoal), null));
  }

  /** {@link #thenFollow} for Commands v2: the pathfinding command, then the path's, handing over the speed. */
  public PathfindThenFollowV2 thenFollowV2(Subsystem drive, String pathName) {
    Path path = library.get(pathName);
    PathfindCommandV2 first = new PathfindCommandV2(this, drive, "Pathfind to " + pathName, f -> start(path, f), path);
    FollowPathCommandV2 follow = new FollowPathCommandV2(pathName, drive, new PathFollower(path, config), pose,
        fieldVelocity, fieldOutput, redFrame).withEvents(library).continuingFrom(first::endCommand);
    first.handsOverTo(follow);
    v2(first);
    return new PathfindThenFollowV2(first, follow, "Pathfind then " + pathName);
  }

  /** The v2 sequence {@link #thenFollowV2} builds: the pathfinding command, then the path's. */
  public static final class PathfindThenFollowV2 extends org.wpilib.command2.SequentialCommandGroup {
    private final PathfindCommandV2 pathfind;
    private final FollowPathCommandV2 follow;

    PathfindThenFollowV2(PathfindCommandV2 pathfind, FollowPathCommandV2 follow, String name) {
      super(pathfind, follow);
      this.pathfind = pathfind;
      this.follow = follow;
      setName(name);
    }

    public PathfindCommandV2 pathfind() {
      return pathfind;
    }

    public FollowPathCommandV2 follow() {
      return follow;
    }
  }
}
