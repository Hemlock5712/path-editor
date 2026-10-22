package frc.team5712.path.tools;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.Json;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathEvent;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.core.PathSpec;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.RobotFile;
import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.sim.FollowerSim;
import frc.team5712.path.sim.SimDrivetrain;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Golden closed-loop traces, {@code fixtures/follow/<name>.json}: {@link FollowerSim} running the real
 * {@link frc.team5712.path.follow.PathFollower} on the noise-free {@link SimDrivetrain}, every loop's
 * pose and command, and the loop each event fired in. The editor's TypeScript follower and simulator
 * must reproduce them to 1e-9. Written by {@code lib/gradlew -p lib fixtures} (through {@link FixtureGen}).
 *
 * <p>Each fixture holds its inputs: the path-file tree (built from exactly those numbers), the robot,
 * the follower settings that differ from the defaults, the drivetrain, the frame and any shove.
 */
public final class FollowFixtureGen {
  /** Where these fixtures go, relative to the fixtures folder. */
  public static final String DIR = "follow";

  /** Seconds simulated after the finish. */
  public static final double AFTER_SECONDS = 0.5;

  private FollowFixtureGen() {}

  /** One case: a path, its robot, the frame it runs in, an optional shove and the drivetrain. */
  public record Case(PathSpec spec, RobotConfig robot, FieldFrame frame, FollowerSim.Shove shove, FollowerSim.Drive drive) {
    Case(PathSpec spec, RobotConfig robot, FieldFrame frame, FollowerSim.Shove shove) {
      this(spec, robot, frame, shove, FollowerSim.Drive.DEFAULT);
    }
  }

  /** Every case, by name. */
  public static Map<String, Case> cases() {
    Map<String, Case> m = new LinkedHashMap<>();
    for (Map.Entry<String, PathSpec> e : FixturePaths.all().entrySet()) {
      m.put(e.getKey(), new Case(e.getValue(), RobotConfig.DEFAULT, FieldFrame.BLUE, null));
    }
    for (Map.Entry<String, PathSpec> e : FixturePaths.spinLimited().entrySet()) {
      m.put(e.getKey(), new Case(e.getValue(), FixturePaths.SPIN_LIMITED_ROBOT, FieldFrame.BLUE, null));
    }
    m.put("events", new Case(FixturePaths.sCurve().withEvents(List.of(
        PathEvent.atWaypoint("intake", 1.0), PathEvent.atMeters("arm", 3.0), PathEvent.atWaypoint("score", 3.0))),
        RobotConfig.DEFAULT, FieldFrame.BLUE, null));
    m.put("s_curve_shove", new Case(FixturePaths.sCurve(), RobotConfig.DEFAULT, FieldFrame.BLUE,
        new FollowerSim.Shove(1.0, 0.0, 1.5)));
    m.put("auto_route_red", new Case(FixturePaths.autoRoute(), RobotConfig.DEFAULT, FieldFrame.RED_HALF_TURN, null));
    m.put("slalom_mirror", new Case(FixturePaths.slalom(), RobotConfig.DEFAULT, FieldFrame.RED_MIRROR, null));
    // a slick floor under the drivetrain (half the plan's grip): the robot slides where it's asked for more
    FollowerSim.Drive d = FollowerSim.Drive.DEFAULT;
    m.put("s_curve_slick", new Case(FixturePaths.sCurve(), RobotConfig.DEFAULT, FieldFrame.BLUE, null,
        new FollowerSim.Drive(d.latencySeconds(), d.lagSeconds(), d.substepSeconds(), 0.35 * RobotConfig.GRAVITY)));
    // a path that doubles back: the follower stops at the cusp and reverses
    m.put("joe_auto_cusp", new Case(FixturePaths.joeAutoCusp(), RobotConfig.DEFAULT, FieldFrame.BLUE, null));
    return m;
  }

  /** The follower settings the fixtures use: the defaults, with the latency set to the drivetrain's. */
  public static Map<String, Object> followerOptions(FollowerSim.Drive drive) {
    Map<String, Object> m = new LinkedHashMap<>();
    m.put("latencySeconds", SimDrivetrain.responseDelaySeconds(drive.latencySeconds(), drive.lagSeconds()));
    return m;
  }

  /** The fixture text for one case. */
  public static String fixture(Case c) {
    FollowerSim.Drive drive = c.drive();
    Map<String, Object> pathTree =
        PathFile.asMap(Json.parse(Json.write(PathFile.toJson(c.spec()), Json::exact)), "path");
    Path path = Path.build(PathFile.fromJson(pathTree), c.robot());
    Map<String, Object> options = followerOptions(drive);
    FollowerConfig.Builder b = FollowerConfig.builder(c.robot());
    for (Map.Entry<String, Object> e : options.entrySet()) {
      b.set(e.getKey(), ((Number) e.getValue()).doubleValue());
    }
    double maxSeconds = path.totalTimeSeconds() + 6.0;
    FollowerSim.Trace trace = FollowerSim.run(path, b.build(), drive, c.frame(), c.shove(), AFTER_SECONDS, maxSeconds);

    Map<String, Object> inputs = new LinkedHashMap<>();
    inputs.put("path", pathTree);
    inputs.put("robot", RobotFile.toJson(c.robot()));
    inputs.put("follower", options);
    Map<String, Object> d = new LinkedHashMap<>();
    d.put("latencySeconds", drive.latencySeconds());
    d.put("lagSeconds", drive.lagSeconds());
    d.put("substepSeconds", drive.substepSeconds());
    if (Double.isFinite(drive.gripMetersPerSecondSquared())) {
      d.put("gripMetersPerSecondSquared", drive.gripMetersPerSecondSquared());
    }
    inputs.put("drive", d);
    inputs.put("frame", c.frame().name());
    if (c.shove() != null) {
      Map<String, Object> sh = new LinkedHashMap<>();
      sh.put("atSeconds", c.shove().atSeconds());
      sh.put("dvx", c.shove().dvxMetersPerSecond());
      sh.put("dvy", c.shove().dvyMetersPerSecond());
      inputs.put("shove", sh);
    }
    inputs.put("afterSeconds", AFTER_SECONDS);
    inputs.put("maxSeconds", maxSeconds);

    Map<String, Object> loops = new LinkedHashMap<>();
    loops.put("t", trace.t);
    loops.put("x", trace.x);
    loops.put("y", trace.y);
    loops.put("heading", trace.heading);
    loops.put("vx", trace.vx);
    loops.put("vy", trace.vy);
    loops.put("omega", trace.omega);
    loops.put("s", trace.s);
    loops.put("crossTrack", trace.crossTrack);
    List<Object> phases = new ArrayList<>();
    for (int p : trace.phase) {
      phases.add(p);
    }
    loops.put("phase", phases);
    List<Object> events = new ArrayList<>();
    for (FollowerSim.Fired f : trace.events) {
      Map<String, Object> e = new LinkedHashMap<>();
      e.put("name", f.name());
      e.put("loop", f.loop());
      events.add(e);
    }

    Map<String, Object> root = new LinkedHashMap<>();
    root.put("inputs", inputs);
    root.put("finishLoop", trace.finishLoop);
    root.put("events", events);
    root.put("loops", loops);
    return Json.write(root, Json::exact);
  }

  /** Writes every case under {@code out/follow}. */
  public static void writeAll(java.nio.file.Path out) throws IOException {
    java.nio.file.Path dir = out.resolve(DIR);
    Files.createDirectories(dir);
    for (Map.Entry<String, Case> e : cases().entrySet()) {
      java.nio.file.Path file = dir.resolve(e.getKey() + ".json");
      Files.writeString(file, fixture(e.getValue()), StandardCharsets.UTF_8);
      System.out.println("wrote " + file);
    }
  }

  public static void main(String[] args) throws IOException {
    writeAll(java.nio.file.Path.of(args.length > 0 ? args[0] : "fixtures"));
  }
}
