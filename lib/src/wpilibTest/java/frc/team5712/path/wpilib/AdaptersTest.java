package frc.team5712.path.wpilib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathEvent;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.follow.PathFollower;
import frc.team5712.path.tools.FixturePaths;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

class AdaptersTest {
  @Test
  void conversionsRoundTrip() {
    Pose2d p = new Pose2d(1.5, -2.25, new Rotation2d(0.7));
    Pose core = Conversions.toPose(p);
    assertEquals(new Pose(1.5, -2.25, 0.7).xMeters(), core.xMeters(), 0.0);
    assertEquals(0.7, core.headingRadians(), 1e-15);
    Pose2d back = Conversions.toPose2d(core);
    assertEquals(p.getX(), back.getX(), 0.0);
    assertEquals(p.getRotation().getRadians(), back.getRotation().getRadians(), 1e-15);
    ChassisVelocities v = new ChassisVelocities(1.0, -0.5, 2.0);
    FieldSpeeds s = Conversions.toFieldSpeeds(v);
    assertEquals(new FieldSpeeds(1.0, -0.5, 2.0), s);
    ChassisVelocities w = Conversions.toChassisVelocities(s);
    assertEquals(v.vx, w.vx, 0.0);
    assertEquals(v.omega, w.omega, 0.0);
  }

  @Test
  void libraryBuildsEveryPathFileAtOnce(@TempDir File dir) throws IOException {
    PathFile.write(FixturePaths.sCurve(), dir.toPath().resolve("SCurve.path.json"));
    PathFile.write(FixturePaths.autoRoute(), dir.toPath().resolve("Auto.path.json"));
    Files.writeString(dir.toPath().resolve("notes.txt"), "not a path");
    PathLibrary lib = PathLibrary.loadAll(dir, RobotConfig.DEFAULT);
    assertEquals(List.of("Auto", "SCurve"), List.copyOf(lib.all().keySet()));
    assertEquals(Path.build(FixturePaths.sCurve(), RobotConfig.DEFAULT).totalTimeSeconds(),
        lib.get("SCurve").totalTimeSeconds(), 1e-12);
    assertTrue(lib.buildMillis() >= 0);
    assertThrows(java.util.NoSuchElementException.class, () -> lib.get("Nope"));

    Files.writeString(dir.toPath().resolve("Bad.path.json"), "{ \"schema\": 9 }");
    assertThrows(IllegalStateException.class, () -> PathLibrary.loadAll(dir, RobotConfig.DEFAULT));
  }

  @Test
  void libraryBuildsAPathfinderPerFootprintAndChecksTheNames(@TempDir File dir) throws IOException {
    Files.writeString(dir.toPath().resolve("robot.json"),
        "{ \"footprints\": [ { \"name\": \"intakeOut\", \"frontMeters\": 0.3 } ] }");
    PathFile.write(FixturePaths.sCurve().withEvents(List.of(
        PathEvent.atWaypoint("deploy", 0.5).withFootprint("intakeOut"),
        PathEvent.atWaypoint("stow", 1.5).withFootprint("bumpers"))), dir.toPath().resolve("Pick.path.json"));
    PathLibrary lib = PathLibrary.loadAll(dir);
    assertEquals(List.of("bumpers", "intakeOut"), List.copyOf(lib.footprintNames()));
    assertSame(lib.pathfinder(), lib.pathfinder("bumpers"));
    assertSame(lib.widePathfinder(), lib.widePathfinder("bumpers"));
    assertEquals(0.425 + 0.3, lib.pathfinder("intakeOut").config().footprint().frontMeters(), 1e-12);
    assertNotNull(lib.widePathfinder("intakeOut"));
    assertThrows(java.util.NoSuchElementException.class, () -> lib.pathfinder("armUp"));

    PathFile.write(FixturePaths.sCurve().withStartFootprint("armUp"), dir.toPath().resolve("Bad.path.json"));
    IllegalStateException e = assertThrows(IllegalStateException.class, () -> PathLibrary.loadAll(dir));
    assertTrue(e.getMessage().contains("Bad starts with \"armUp\""), e.getMessage());
  }

  /** The v3 adapter under an independent scheduler: runs, forks its event, finishes on the goal. */
  @Test
  void commandV3DrivesThePathAndForksEvents() {
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    Mechanism drive = new Mechanism() {
      @Override
      public Scheduler getRegisteredScheduler() {
        return scheduler;
      }
    };
    Path path = Path.build(FixturePaths.sCurve(), RobotConfig.DEFAULT);
    PathFollower follower = new PathFollower(path, FollowerConfig.defaults(RobotConfig.DEFAULT))
        .addEvent("halfway", path.lengthMeters() / 2, () -> {});
    double[] t = {0.0};
    Pose2d[] pose = {new Pose2d(-4.0, -1.0, new Rotation2d(0.0))};
    ChassisVelocities[] vel = {new ChassisVelocities()};
    List<ChassisVelocities> outputs = new ArrayList<>();
    boolean[] eventRan = {false};
    org.wpilib.command3.Command marker = org.wpilib.command3.Command.noRequirements(co -> eventRan[0] = true)
        .named("marker");
    FollowPathCommand cmd = new FollowPathCommand("SCurve", drive, follower,
        () -> pose[0], () -> vel[0], outputs::add, FieldFrame.RED_HALF_TURN)
        .withClock(() -> t[0])
        .withFrame(() -> FieldFrame.BLUE)
        .onEvent("halfway", marker);
    scheduler.schedule(cmd);
    while (t[0] < 10 && (t[0] == 0 || scheduler.isScheduledOrRunning(cmd))) {
      t[0] += 0.02;
      ChassisVelocities last = vel[0];
      scheduler.run();
      pose[0] = new Pose2d(pose[0].getX() + last.vx * 0.02, pose[0].getY() + last.vy * 0.02,
          new Rotation2d(pose[0].getRotation().getRadians() + last.omega * 0.02));
      if (!outputs.isEmpty()) {
        vel[0] = outputs.get(outputs.size() - 1);
      }
    }
    assertTrue(follower.isFinished(), "finished at t = " + t[0]);
    assertTrue(eventRan[0], "the event's command ran");
    assertEquals(4.0, pose[0].getX(), 0.02);
    assertEquals(1.0, pose[0].getY(), 0.02);
  }

  /**
   * With wheel forces on, every loop's output goes to the three-argument drive: the velocity, and forces that add
   * up to mass x the acceleration the follower asked for. The velocity-only output is never called, and the stop
   * at the end carries no push.
   */
  @Test
  void commandSendsTheWheelForcesWithTheVelocity() {
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    Mechanism drive = new Mechanism() {
      @Override
      public Scheduler getRegisteredScheduler() {
        return scheduler;
      }
    };
    RobotConfig robot = RobotConfig.DEFAULT;
    Path path = Path.build(FixturePaths.sCurve(), robot);
    PathFollower follower = new PathFollower(path, FollowerConfig.defaults(robot));
    Translation2d[] modules = {
      new Translation2d(0.29, 0.29), new Translation2d(0.29, -0.29),
      new Translation2d(-0.29, 0.29), new Translation2d(-0.29, -0.29)
    };
    double[] t = {0.0};
    Pose2d[] pose = {new Pose2d(-4.0, -1.0, new Rotation2d(0.0))};
    ChassisVelocities[] vel = {new ChassisVelocities()};
    List<ChassisVelocities> velocityOnly = new ArrayList<>();
    double[] lastPush = {Double.NaN, Double.NaN};
    double[] worst = {0.0, 0.0}; // the largest gap from mass x acceleration, and the largest push seen
    int[] sends = {0};
    FollowPathCommand cmd = new FollowPathCommand("SCurve", drive, follower,
        () -> pose[0], () -> vel[0], velocityOnly::add, FieldFrame.RED_HALF_TURN)
        .withClock(() -> t[0])
        .withFrame(() -> FieldFrame.BLUE)
        .withWheelForces(robot, modules, (v, fx, fy) -> {
          assertEquals(4, fx.length);
          assertEquals(4, fy.length);
          vel[0] = v;
          sends[0]++;
          lastPush[0] = fx[0] + fx[1] + fx[2] + fx[3];
          lastPush[1] = fy[0] + fy[1] + fy[2] + fy[3];
          if (!follower.isFinished()) {
            FieldSpeeds acc = follower.commandedAcceleration();
            worst[0] = Math.max(worst[0], Math.abs(lastPush[0] - robot.massKg() * acc.vxMetersPerSecond()));
            worst[0] = Math.max(worst[0], Math.abs(lastPush[1] - robot.massKg() * acc.vyMetersPerSecond()));
            worst[1] = Math.max(worst[1], Math.hypot(lastPush[0], lastPush[1]));
          }
        });
    scheduler.schedule(cmd);
    while (t[0] < 10 && (t[0] == 0 || scheduler.isScheduledOrRunning(cmd))) {
      t[0] += 0.02;
      ChassisVelocities last = vel[0];
      scheduler.run();
      pose[0] = new Pose2d(pose[0].getX() + last.vx * 0.02, pose[0].getY() + last.vy * 0.02,
          new Rotation2d(pose[0].getRotation().getRadians() + last.omega * 0.02));
    }
    assertTrue(follower.isFinished(), "finished at t = " + t[0]);
    assertTrue(velocityOnly.isEmpty(), "the velocity-only output is replaced");
    assertTrue(sends[0] > 50, "sent every loop: " + sends[0]);
    assertEquals(0.0, worst[0], 1e-6, "the wheels together push with mass x the commanded acceleration");
    assertTrue(worst[1] > robot.massKg(), "and the robot was pushed: " + worst[1] + " N");
    assertEquals(0.0, vel[0].vx, 0.0, "the last send is the stop");
    assertEquals(0.0, lastPush[0], 0.0, "with no push");
    assertEquals(0.0, lastPush[1], 0.0);
  }

  /** Events from the path file run the commands bound by name in the library, in path order. */
  @Test
  void libraryBindsFileEventsToCommands(@TempDir File dir) throws IOException {
    PathFile.write(FixturePaths.sCurve().withEvents(List.of(
        PathEvent.atMeters("shoot", 5.0), PathEvent.atWaypoint("intakeDown", 0.5), PathEvent.atMeters("typo", 6.0))),
        dir.toPath().resolve("SCurve.path.json"));
    PathLibrary lib = PathLibrary.loadAll(dir, RobotConfig.DEFAULT);
    assertEquals(Set.of("intakeDown", "shoot", "typo"), lib.eventNames());

    Scheduler scheduler = Scheduler.createIndependentScheduler();
    Mechanism drive = new Mechanism() {
      @Override
      public Scheduler getRegisteredScheduler() {
        return scheduler;
      }
    };
    List<String> ran = new ArrayList<>();
    lib.bindEvent("intakeDown", org.wpilib.command3.Command.noRequirements(co -> ran.add("intakeDown")).named("i"))
        .bindEvent("shoot", org.wpilib.command3.Command.noRequirements(co -> ran.add("shoot")).named("s"));
    assertEquals(Set.of("typo"), lib.unboundEventNames(), "names with no command show up");

    Path path = lib.get("SCurve");
    PathFollower follower = new PathFollower(path, FollowerConfig.defaults(RobotConfig.DEFAULT));
    double[] t = {0.0};
    Pose2d[] pose = {new Pose2d(-4.0, -1.0, new Rotation2d(0.0))};
    ChassisVelocities[] vel = {new ChassisVelocities()};
    List<ChassisVelocities> outputs = new ArrayList<>();
    FollowPathCommand cmd = new FollowPathCommand("SCurve", drive, follower,
        () -> pose[0], () -> vel[0], outputs::add, FieldFrame.RED_HALF_TURN)
        .withClock(() -> t[0])
        .withFrame(() -> FieldFrame.BLUE)
        .withEvents(lib);
    scheduler.schedule(cmd);
    while (t[0] < 10 && (t[0] == 0 || scheduler.isScheduledOrRunning(cmd))) {
      t[0] += 0.02;
      ChassisVelocities last = vel[0];
      scheduler.run();
      pose[0] = new Pose2d(pose[0].getX() + last.vx * 0.02, pose[0].getY() + last.vy * 0.02,
          new Rotation2d(pose[0].getRotation().getRadians() + last.omega * 0.02));
      if (!outputs.isEmpty()) {
        vel[0] = outputs.get(outputs.size() - 1);
      }
    }
    assertTrue(follower.isFinished());
    assertEquals(List.of("intakeDown", "shoot"), ran);
  }
}
