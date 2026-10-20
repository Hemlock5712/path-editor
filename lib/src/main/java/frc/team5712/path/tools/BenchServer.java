package frc.team5712.path.tools;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Json;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.RobotFile;
import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.follow.PathFollower;
import frc.team5712.path.plan.FieldMap;
import frc.team5712.path.plan.Pathfinder;
import frc.team5712.path.plan.PlanResult;
import frc.team5712.path.plan.PlannerConfig;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * The Python bench's bridge to {@link PathFollower}: one JSON object per line on stdin, one reply
 * per line on stdout. The protocol is in lib/README.md.
 *
 * <pre>
 * java -cp lib/build/libs/pathcore.jar frc.team5712.path.tools.BenchServer
 * </pre>
 */
public final class BenchServer {
  private Path path;
  private PathFollower follower;
  private Optional<Pose> goal = Optional.empty();
  private Pathfinder pathfinder;
  private String pathfinderKey;

  public static void main(String[] args) throws IOException {
    BufferedReader in = new BufferedReader(new InputStreamReader(System.in, StandardCharsets.UTF_8));
    PrintStream out = new PrintStream(System.out, false, StandardCharsets.UTF_8);
    BenchServer server = new BenchServer();
    String line;
    while ((line = in.readLine()) != null) {
      if (line.isBlank()) {
        continue;
      }
      String reply;
      boolean quit = false;
      try {
        Map<String, Object> msg = PathFile.asMap(Json.parse(line), "message");
        quit = "quit".equals(msg.get("op"));
        reply = quit ? "{\"ok\":true}" : server.handle(msg);
      } catch (RuntimeException e) {
        reply = "{\"error\":" + quote(e.getClass().getSimpleName() + ": " + e.getMessage()) + "}";
      }
      out.println(reply);
      out.flush();
      if (quit) {
        return;
      }
    }
  }

  /** Handles one message and returns the reply line (without the newline). */
  public String handle(Map<String, Object> msg) {
    Object op = msg.get("op");
    if (op == null) {
      throw new IllegalArgumentException("no op");
    }
    return switch (op.toString()) {
      case "load" -> load(msg);
      case "pathfind" -> pathfind(msg);
      case "reset" -> reset(msg);
      case "step" -> step(msg);
      case "goal" -> goal(msg);
      case "geometry" -> geometry(msg);
      default -> throw new IllegalArgumentException("unknown op " + op);
    };
  }

  private String load(Map<String, Object> msg) {
    Map<String, Object> pathTree = PathFile.asMap(msg.get("path"), "path");
    RobotConfig robot = msg.get("robot") == null
        ? RobotConfig.DEFAULT
        : RobotFile.fromJson(PathFile.asMap(msg.get("robot"), "robot"));
    FollowerConfig.Builder b = FollowerConfig.builder(robot);
    if (msg.get("options") != null) {
      for (Map.Entry<String, Object> e : PathFile.asMap(msg.get("options"), "options").entrySet()) {
        if (!(e.getValue() instanceof Number n) || !b.set(e.getKey(), n.doubleValue())) {
          throw new IllegalArgumentException("bad option " + e.getKey());
        }
      }
    }
    long t0 = System.nanoTime();
    path = Path.build(PathFile.fromJson(pathTree), robot);
    double buildMs = (System.nanoTime() - t0) / 1e6;
    follower = new PathFollower(path, b.build());
    goal = Optional.empty();
    follower.bindGoal(() -> goal);
    Map<String, Object> r = new LinkedHashMap<>();
    r.put("ok", Boolean.TRUE);
    r.put("lengthMeters", path.lengthMeters());
    r.put("totalTimeSeconds", path.totalTimeSeconds());
    r.put("buildMs", buildMs);
    return line(r);
  }

  /**
   * Plans with our pathfinder from the given start state and loads the result like {@code load}. The
   * pathfinder (grown obstacles and the visibility graph) is built once per field, robot and margin and
   * kept, as it would be at boot.
   */
  private String pathfind(Map<String, Object> msg) {
    Map<String, Object> fieldTree = PathFile.asMap(msg.get("field"), "field");
    RobotConfig robot = msg.get("robot") == null
        ? RobotConfig.DEFAULT
        : RobotFile.fromJson(PathFile.asMap(msg.get("robot"), "robot"));
    PlannerConfig pc = PlannerConfig.DEFAULT;
    if (msg.get("margin") != null) {
      pc = pc.withMargin(number(msg, "margin"));
    }
    if (msg.get("maxSpeed") != null) {
      pc = pc.withLimits(number(msg, "maxSpeed"), number(msg, "maxAccel"));
    }
    FollowerConfig.Builder b = FollowerConfig.builder(robot);
    if (msg.get("options") != null) {
      for (Map.Entry<String, Object> e : PathFile.asMap(msg.get("options"), "options").entrySet()) {
        if (!(e.getValue() instanceof Number n) || !b.set(e.getKey(), n.doubleValue())) {
          throw new IllegalArgumentException("bad option " + e.getKey());
        }
      }
    }
    String key = line(fieldTree) + "|" + robot + "|" + pc;
    Map<String, Object> r = new LinkedHashMap<>();
    r.put("ok", Boolean.TRUE);
    if (!key.equals(pathfinderKey)) {
      pathfinder = new Pathfinder(FieldMap.fromJson(fieldTree), robot, pc);
      pathfinderKey = key;
      r.put("bootMs", pathfinder.bootMicros() / 1e3);
    }
    PlanResult plan = pathfinder.plan(pose(msg.get("start")), speeds(msg.get("vel")), pose(msg.get("goal")));
    if (!plan.ok()) {
      throw new IllegalStateException(plan.failure());
    }
    path = plan.path();
    follower = new PathFollower(path, b.build());
    goal = Optional.empty();
    follower.bindGoal(() -> goal);
    describe(plan, r);
    return line(r);
  }

  /** The plan's numbers as reply fields (also used by PlanCli). */
  static void describe(PlanResult plan, Map<String, Object> r) {
    if (plan.ok()) {
      r.put("lengthMeters", plan.path().lengthMeters());
      r.put("totalTimeSeconds", plan.path().totalTimeSeconds());
    } else {
      r.put("failure", plan.failure());
    }
    r.put("planMs", plan.totalMicros() / 1e3);
    r.put("searchMs", plan.searchMicros() / 1e3);
    r.put("smoothMs", plan.smoothMicros() / 1e3);
    r.put("profileMs", plan.profileMicros() / 1e3);
    r.put("routeLengthMeters", plan.routeLengthMeters());
    r.put("straightLineMeters", plan.straightLineMeters());
    r.put("startSnapMeters", plan.startSnapMeters());
    r.put("goalSnapMeters", plan.goalSnapMeters());
    r.put("refits", (double) plan.refits());
    r.put("clear", plan.clear());
    r.put("minClearanceMeters", plan.minClearanceMeters());
    List<Object> wps = new ArrayList<>();
    for (double[] w : plan.waypoints()) {
      wps.add(List.of(w[0], w[1]));
    }
    r.put("waypoints", wps);
  }

  private String reset(Map<String, Object> msg) {
    need();
    FieldFrame frame = msg.get("frame") == null ? FieldFrame.BLUE : FieldFrame.valueOf(msg.get("frame").toString());
    long t0 = System.nanoTime();
    follower.start(number(msg, "t"), pose(msg.get("pose")), speeds(msg.get("vel")), frame);
    Map<String, Object> r = new LinkedHashMap<>();
    r.put("ok", Boolean.TRUE);
    r.put("startMicros", (System.nanoTime() - t0) / 1e3);
    return line(r);
  }

  private String step(Map<String, Object> msg) {
    need();
    long t0 = System.nanoTime();
    FieldSpeeds cmd = follower.step(number(msg, "t"), pose(msg.get("pose")), speeds(msg.get("vel")));
    double micros = (System.nanoTime() - t0) / 1e3;
    Map<String, Object> r = new LinkedHashMap<>();
    r.put("cmd", List.of(cmd.vxMetersPerSecond(), cmd.vyMetersPerSecond(), cmd.omegaRadPerSec()));
    r.put("done", follower.isFinished());
    r.put("phase", follower.phase().name());
    r.put("s", follower.progressMeters());
    r.put("crossTrack", follower.crossTrackMeters());
    r.put("micros", micros);
    if (!Double.isNaN(follower.lastTailRebuildMicros())) {
      r.put("rebuilds", (double) follower.tailRebuildCount());
      r.put("rebuildMicros", follower.lastTailRebuildMicros());
    }
    return line(r);
  }

  private String goal(Map<String, Object> msg) {
    need();
    Object p = msg.get("pose");
    goal = p == null ? Optional.empty() : Optional.of(pose(p));
    return "{\"ok\":true}";
  }

  /** The built path sampled every {@code step} metres (default 0.02): x and y lists. */
  private String geometry(Map<String, Object> msg) {
    need();
    double step = msg.get("step") == null ? 0.02 : number(msg, "step");
    List<Object> xs = new ArrayList<>();
    List<Object> ys = new ArrayList<>();
    for (double s : ProfileSamples.stations(path.lengthMeters(), step)) {
      PathSample p = path.sample(s);
      xs.add(p.xMeters());
      ys.add(p.yMeters());
    }
    Map<String, Object> r = new LinkedHashMap<>();
    r.put("x", xs);
    r.put("y", ys);
    return line(r);
  }

  private void need() {
    if (follower == null) {
      throw new IllegalStateException("load a path first");
    }
  }

  private static double number(Map<String, Object> m, String key) {
    Object v = m.get(key);
    if (!(v instanceof Number n)) {
      throw new IllegalArgumentException("'" + key + "' must be a number");
    }
    return n.doubleValue();
  }

  private static double[] triple(Object o, String what) {
    if (!(o instanceof List<?> l) || l.size() != 3) {
      throw new IllegalArgumentException(what + " must be [a, b, c]");
    }
    double[] v = new double[3];
    for (int i = 0; i < 3; i++) {
      v[i] = ((Number) l.get(i)).doubleValue();
    }
    return v;
  }

  private static Pose pose(Object o) {
    double[] v = triple(o, "pose");
    return new Pose(v[0], v[1], v[2]);
  }

  private static FieldSpeeds speeds(Object o) {
    double[] v = triple(o, "vel");
    return new FieldSpeeds(v[0], v[1], v[2]);
  }

  private static String line(Map<String, Object> r) {
    return Json.write(r, Json::exact).trim().replace("\n", "");
  }

  private static String quote(String s) {
    return Json.write(s, Json::exact).trim();
  }
}
