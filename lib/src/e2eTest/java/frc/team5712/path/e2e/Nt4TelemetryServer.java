package frc.team5712.path.e2e;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.PathEvent;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.Pose;
import frc.team5712.path.follow.PathFollower;
import frc.team5712.path.tools.FixturePaths;
import frc.team5712.path.wpilib.Conversions;
import frc.team5712.path.wpilib.FollowPathCommand;
import frc.team5712.path.wpilib.PathLibrary;
import java.io.File;
import java.nio.file.Files;
import java.util.List;
import org.wpilib.backend.NetworkTablesTelemetryBackend;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTablesJNI;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.telemetry.TelemetryTable;

/**
 * A real NetworkTables 4 server (WPILib's ntcore, from the 2027 alpha-7 jars and desktop natives) with a
 * robot on it: the S-curve, with events, loaded through PathLibrary and followed by a real PathFollower
 * inside FollowPathCommand on {@link SimDrive}, in real time at 50 Hz, over and over. What it publishes:
 *
 * <ul>
 *   <li>the follower's telemetry, as robot code would (PathTelemetry under /Telemetry/Path/SCurve and
 *       /Telemetry/PathFollower);
 *   <li>the robot's true pose, /Telemetry/Robot/Pose (struct:Pose2d);
 *   <li>the server's own clock each loop, /Telemetry/Harness/ServerTimeNanos (ntcore's clock, in ns), so a client can check its
 *       time sync against it.
 * </ul>
 *
 * Usage (lib/build.gradle): {@code lib/gradlew -p lib nt4Server -Pnt4Port=5810 -Pnt4Seconds=60}. Prints
 * "NT4 READY port=..." once listening, and exits after the given time. The editor's live test
 * (editor/app/test/nt4/*.live.ts) starts it and connects the TypeScript client.
 */
public final class Nt4TelemetryServer {
  private Nt4TelemetryServer() {}

  public static void main(String[] args) throws Exception {
    int port = args.length > 0 ? Integer.parseInt(args[0]) : 5810;
    double seconds = args.length > 1 ? Double.parseDouble(args[1]) : 60.0;

    HAL.initialize();
    NetworkTableInstance inst = NetworkTableInstance.create();
    File persist = File.createTempFile("nt4-harness", ".json");
    persist.deleteOnExit();
    inst.startServer(persist.getAbsolutePath(), "", "", port);
    // what RobotBase does on a robot: Telemetry goes to NetworkTables under /Telemetry
    TelemetryRegistry.registerBackend("", new NetworkTablesTelemetryBackend(inst, "/Telemetry"));

    // the robot code's side: a path file with events, loaded and bound as on a robot
    File dir = Files.createTempDirectory("nt4-paths").toFile();
    dir.deleteOnExit();
    File file = new File(dir, "SCurve.path.json");
    file.deleteOnExit();
    PathFile.write(FixturePaths.sCurve().withEvents(List.of(
        PathEvent.atMeters("intakeDown", 0.0), PathEvent.atWaypoint("spinUp", 1.5), PathEvent.atWaypoint("shoot", 3.0))),
        file.toPath());
    PathLibrary library = PathLibrary.loadAll(dir);
    for (String name : library.eventNames()) {
      library.bindEvent(name, Command.noRequirements(co -> {}).named(name));
    }

    TelemetryTable robot = Telemetry.getTable("Robot");
    TelemetryTable harness = Telemetry.getTable("Harness");
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    Mechanism drivetrain = new Mechanism() {
      @Override
      public Scheduler getRegisteredScheduler() {
        return scheduler;
      }
    };

    System.out.println("NT4 READY port=" + port);
    System.out.flush();
    long start = System.nanoTime();
    long loop = 0;
    int run = 0;
    while ((System.nanoTime() - start) / 1e9 < seconds) {
      // one run of the path, in real time
      PathSample s0 = library.get("SCurve").sample(0);
      SimDrive drive = new SimDrive(new Pose(s0.xMeters(), s0.yMeters(), s0.headingRadians()), new FieldSpeeds(0, 0, 0), run++);
      double runStart = (System.nanoTime() - start) / 1e9;
      double[] clock = {0.0};
      PathFollower follower = new PathFollower(library.get("SCurve"), EndToEndRun.followerConfig(library));
      FollowPathCommand command = new FollowPathCommand("SCurve", drivetrain, follower,
          drive::measuredPose, drive::measuredVelocity, drive::command, FieldFrame.RED_HALF_TURN)
          .withClock(() -> clock[0])
          .withFrame(() -> FieldFrame.BLUE)
          .withEvents(library);
      scheduler.schedule(command);
      double finishedAt = Double.NaN;
      while ((System.nanoTime() - start) / 1e9 < seconds) {
        loop++;
        double now = (System.nanoTime() - start) / 1e9;
        clock[0] = now - runStart;
        drive.advanceTo(clock[0]);
        scheduler.run();
        robot.log("Pose", Conversions.toPose2d(drive.truePose()), Pose2d.struct);
        // NetworkTablesJNI.now() is in nanoseconds in 2027; NT4 stamps values in microseconds on the wire
        harness.log("ServerTimeNanos", NetworkTablesJNI.now());
        harness.log("Loop", loop);
        inst.flush(); // send this loop's values now, not on NT's 100 ms cycle (as low-latency robot code does)
        if (Double.isNaN(finishedAt) && !scheduler.isScheduledOrRunning(command)) {
          finishedAt = now;
        }
        if (!Double.isNaN(finishedAt) && now - finishedAt > 0.5) {
          break; // rest a moment on the goal, then run it again
        }
        long next = start + (long) (loop * Tolerances.LOOP_SECONDS * 1e9);
        long wait = next - System.nanoTime();
        if (wait > 0) {
          Thread.sleep(wait / 1_000_000, (int) (wait % 1_000_000));
        }
      }
    }
    inst.stopServer();
    inst.close();
    System.out.println("NT4 DONE loops=" + loop);
    System.exit(0);
  }
}
