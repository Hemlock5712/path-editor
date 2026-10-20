package frc.team5712.path.wpilib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Path;
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
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.command2.Subsystem;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
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

  /** The v2 adapter's lifecycle by hand (no scheduler): it drives a kinematic robot to the goal. */
  @Test
  void commandV2DrivesThePath() {
    Path path = Path.build(FixturePaths.sCurve(), RobotConfig.DEFAULT);
    PathFollower follower = new PathFollower(path, FollowerConfig.defaults(RobotConfig.DEFAULT));
    double[] t = {0.0};
    Pose2d[] pose = {new Pose2d(-4.0, -1.0, new Rotation2d(0.0))};
    ChassisVelocities[] vel = {new ChassisVelocities()};
    List<ChassisVelocities> outputs = new ArrayList<>();
    Subsystem drive = new Subsystem() {};
    FollowPathCommandV2 cmd = new FollowPathCommandV2("SCurve", drive, follower,
        () -> pose[0], () -> vel[0], outputs::add, FieldFrame.RED_HALF_TURN)
        .withClock(() -> t[0])
        .withFrame(() -> FieldFrame.BLUE);
    cmd.initialize();
    while (!cmd.isFinished() && t[0] < 10) {
      t[0] += 0.02;
      ChassisVelocities last = vel[0];
      cmd.execute();
      // the robot does what it was asked one loop late
      pose[0] = new Pose2d(pose[0].getX() + last.vx * 0.02, pose[0].getY() + last.vy * 0.02,
          new Rotation2d(pose[0].getRotation().getRadians() + last.omega * 0.02));
      vel[0] = outputs.get(outputs.size() - 1);
    }
    cmd.end(false);
    assertTrue(cmd.isFinished());
    assertEquals(4.0, pose[0].getX(), 0.02);
    assertEquals(1.0, pose[0].getY(), 0.02);
    ChassisVelocities stop = outputs.get(outputs.size() - 1);
    assertEquals(0.0, Math.hypot(stop.vx, stop.vy), 0.0, "ends by sending zero");
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
}
