package frc.team5712.path.wpilib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.follow.FollowerConfig;
import java.io.File;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Driving to a pose with the library's commands, as robot code that used Precision-Drive's DriveToPose and
 * DriveToTag would: a library with no paths and no field file (an empty field), {@code Pathfinding.to} and
 * {@code toTarget}, a tolerance, a finish gate, the teleop limiter and a tag goal.
 */
class DriveToPoseTest {
  private static final double LOOP = 0.02;

  /** A robot whose velocity is the last command (one loop late), on an independent scheduler. */
  private static final class Bot {
    final Scheduler scheduler = Scheduler.createIndependentScheduler();
    final Mechanism drive = new Mechanism() {
      @Override
      public Scheduler getRegisteredScheduler() {
        return scheduler;
      }
    };
    double t;
    Pose2d pose;
    ChassisVelocities velocity = new ChassisVelocities();
    ChassisVelocities command = new ChassisVelocities();
    Pose2d sticksNear; // creeping in to this spot (under 0.5 m/s in its last 30 cm) the robot doesn't move

    Bot(Pose2d start) {
      pose = start;
    }

    Pathfinding pathfinding(File dir) {
      PathLibrary paths = PathLibrary.loadAll(dir, RobotConfig.DEFAULT);
      return new Pathfinding(paths, FollowerConfig.defaults(paths.robot()), () -> pose, () -> velocity,
          v -> command = v, FieldFrame.RED_HALF_TURN).withClock(() -> t).withFrame(() -> FieldFrame.BLUE);
    }

    /** Runs {@code command} until it ends or {@code maxSeconds} pass; returns the seconds it took. */
    double run(org.wpilib.command3.Command c, double maxSeconds, Runnable everyLoop) {
      double t0 = t;
      scheduler.schedule(c);
      do {
        t += LOOP;
        boolean stuck = sticksNear != null && distanceTo(sticksNear) < 0.3
            && Math.hypot(command.vx, command.vy) < 0.5;
        velocity = stuck ? new ChassisVelocities(0.0, 0.0, command.omega) : command;
        pose = new Pose2d(pose.getX() + velocity.vx * LOOP, pose.getY() + velocity.vy * LOOP,
            new Rotation2d(pose.getRotation().getRadians() + velocity.omega * LOOP));
        everyLoop.run();
        scheduler.run();
      } while (t - t0 < maxSeconds && scheduler.isScheduledOrRunning(c));
      return t - t0;
    }

    double distanceTo(Pose2d goal) {
      return pose.getTranslation().getDistance(goal.getTranslation());
    }
  }

  /** From across the field, from a few centimetres off, and from on the goal already: it arrives and finishes. */
  @Test
  void drivesToAPoseFromAnywhere(@TempDir File dir) {
    Pose2d goal = new Pose2d(2.0, 1.0, new Rotation2d(Math.PI / 2));
    Pose2d[] starts = {
      new Pose2d(-1.0, -0.5, new Rotation2d(0.0)),
      new Pose2d(2.04, 1.03, new Rotation2d(Math.toRadians(84))),
      new Pose2d(1.7, 1.0, new Rotation2d(Math.PI / 2)),
      goal,
    };
    for (Pose2d start : starts) {
      Bot bot = new Bot(start);
      PathfindCommand cmd = bot.pathfinding(dir).to(bot.drive, goal);
      double seconds = bot.run(cmd, 8.0, () -> {});
      assertFalse(bot.scheduler.isScheduledOrRunning(cmd), "finished from " + start + " (" + seconds + " s)");
      assertEquals(0.0, bot.distanceTo(goal), 0.02, "on the goal from " + start);
      assertEquals(90.0, bot.pose.getRotation().getDegrees(), 1.0, "facing the goal's way from " + start);
      assertEquals(0.0, Math.hypot(bot.command.vx, bot.command.vy), 0.0, "and told to stop");
    }
  }

  /**
   * A robot that sticks: creeping in at less than half a metre a second it doesn't move, so it stops a few
   * centimetres short. The usual 2 cm tolerance never finishes on it; a tolerance it can meet does.
   */
  @Test
  void theToleranceIsWhatFinishedMeans(@TempDir File dir) {
    Pose2d start = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
    Pose2d goal = new Pose2d(2.0, 0.5, new Rotation2d(0.0));
    Bot exact = new Bot(start);
    exact.sticksNear = goal;
    PathfindCommand tight = exact.pathfinding(dir).to(exact.drive, goal);
    exact.run(tight, 6.0, () -> {});
    assertTrue(exact.scheduler.isScheduledOrRunning(tight), "2 cm is never reached: " + exact.distanceTo(goal));
    Bot rough = new Bot(start);
    rough.sticksNear = goal;
    PathfindCommand loose = rough.pathfinding(dir).to(rough.drive, goal).withTolerance(0.10, Rotation2d.fromDegrees(5));
    double seconds = rough.run(loose, 6.0, () -> {});
    assertFalse(rough.scheduler.isScheduledOrRunning(loose), "10 cm is: finished in " + seconds + " s, " + rough.distanceTo(goal) + " m off");
    assertEquals(0.0, rough.distanceTo(goal), 0.10);
  }

  /** The gate holds the finish: the robot arrives, holds the goal, and the command ends once the gate opens. */
  @Test
  void theFinishWaitsForItsGate(@TempDir File dir) {
    Pose2d goal = new Pose2d(1.5, 0.0, new Rotation2d(0.0));
    Bot bot = new Bot(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
    boolean[] ready = {false};
    PathfindCommand cmd = bot.pathfinding(dir).to(bot.drive, goal).finishWhen(() -> ready[0]);
    bot.run(cmd, 4.0, () -> {});
    assertTrue(bot.scheduler.isScheduledOrRunning(cmd), "still running after 4 s: the gate is shut");
    assertEquals(0.0, bot.distanceTo(goal), 0.02, "but holding the goal");
    ready[0] = true;
    for (int i = 0; i < 10 && bot.scheduler.isScheduledOrRunning(cmd); i++) {
      bot.t += LOOP;
      bot.scheduler.run();
    }
    assertFalse(bot.scheduler.isScheduledOrRunning(cmd), "finished once the gate opened");
  }

  /**
   * DriveToTag with the library: set off for the field map's spot, then finish on the spot measured from the
   * tag, which is not where the map says. The command can't finish before the tag has been seen.
   */
  @Test
  void drivesToTheSpotMeasuredFromTheTag(@TempDir File dir) {
    Pose2d mapTag = new Pose2d(3.0, 0.0, Rotation2d.k180deg);
    Pose2d realTag = new Pose2d(3.06, 0.09, new Rotation2d(Math.toRadians(177))); // the element was put down off
    Transform2d tagToGoal = new Transform2d(new Translation2d(0.5, 0.0), Rotation2d.k180deg);
    Pose2d mapGoal = mapTag.transformBy(tagToGoal);
    Pose2d realGoal = realTag.transformBy(tagToGoal);
    Bot bot = new Bot(new Pose2d(0.0, 0.4, new Rotation2d(0.0)));
    TagGoal tag = new TagGoal(tagToGoal);
    PathfindCommand cmd = bot.pathfinding(dir).toTarget(bot.drive, () -> mapGoal)
        .withMovingGoal(() -> tag.fieldGoal(bot.pose, bot.pose)) // odometry and the field pose agree here
        .finishWhen(tag::hasLock);
    java.util.Random noise = new java.util.Random(12);
    double seconds = bot.run(cmd, 8.0, () -> {
      Pose2d seen = realTag.relativeTo(bot.pose);
      double distance = seen.getTranslation().getNorm();
      if (distance < 2.0) { // the camera sees the tag from 2 m in, a few millimetres off each frame
        double sigma = 0.005 + 0.01 * distance * distance;
        tag.add(new Pose2d(seen.getX() + sigma * noise.nextGaussian(), seen.getY() + sigma * noise.nextGaussian(),
            seen.getRotation()), bot.pose, bot.t);
      }
    });
    assertFalse(bot.scheduler.isScheduledOrRunning(cmd), "finished (" + seconds + " s)");
    assertTrue(tag.hasLock());
    assertEquals(0.0, bot.distanceTo(realGoal), 0.03, "on the spot measured from the tag");
    assertTrue(bot.distanceTo(mapGoal) > 0.05, "which is not the field map's");
    assertEquals(Optional.empty(), new TagGoal(tagToGoal).fieldGoal(bot.pose, bot.pose));
  }

  /** The teleop limiter: a full-stick step is spread over many loops, and it reports the push for each. */
  @Test
  void theTeleopLimiterStepsTowardTheSticks() {
    RobotConfig robot = RobotConfig.DEFAULT;
    DriveLimiter limiter = new DriveLimiter(robot);
    limiter.reset(new ChassisVelocities());
    ChassisVelocities wanted = new ChassisVelocities(3.0, 0.0, 0.0);
    ChassisVelocities first = limiter.limit(wanted, LOOP);
    assertTrue(first.vx > 0.0 && first.vx <= robot.tractionMetersPerSecondSquared() * LOOP + 1e-9,
        "one loop's step is within the traction limit: " + first.vx);
    Translation2d[] modules = {
      new Translation2d(0.29, 0.29), new Translation2d(0.29, -0.29),
      new Translation2d(-0.29, 0.29), new Translation2d(-0.29, -0.29)
    };
    double[][] push = limiter.wheelForces(modules, new Rotation2d(0.0));
    assertEquals(robot.massKg() * first.vx / LOOP, push[0][0] + push[0][1] + push[0][2] + push[0][3], 1e-6);
    ChassisVelocities v = first;
    int loops = 1;
    while (v.vx < 3.0 - 1e-9 && loops < 500) {
      v = limiter.limit(wanted, LOOP);
      loops++;
    }
    assertEquals(3.0, v.vx, 1e-9, "it gets there");
    assertTrue(loops > 5, "over many loops: " + loops);
    limiter.reset(new ChassisVelocities(2.0, 0.0, 0.0));
    assertTrue(limiter.limit(new ChassisVelocities(), LOOP).vx > 1.5, "and brakes within the limit too");
  }
}
