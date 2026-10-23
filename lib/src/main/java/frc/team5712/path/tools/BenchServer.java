package frc.team5712.path.tools;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.PathSpec;
import frc.team5712.path.core.VelocityProfile;
import frc.team5712.path.core.Json;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.RobotFile;
import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.follow.PathFollower;
import frc.team5712.path.slip.SlipAwareOdometry;
import frc.team5712.path.slip.SlipConfig;
import frc.team5712.path.slip.SlipSignal;
import frc.team5712.path.plan.FieldMap;
import frc.team5712.path.plan.OnTheFly;
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
  private SlipAwareOdometry sensing;

  /** The follower's slip signal: whatever the current sensing session's detector says. */
  private final SlipSignal slipSignal = new SlipSignal() {
    @Override
    public double confidence() {
      return sensing == null ? 0.0 : sensing.detector().confidence();
    }

    @Override
    public double achievedAccelMetersPerSecondSquared() {
      return sensing == null ? 0.0 : sensing.detector().achievedAccelMetersPerSecondSquared();
    }

    @Override
    public double imuGapMetersPerSecondSquared() {
      return sensing == null ? Double.NaN : sensing.detector().imuGapMetersPerSecondSquared();
    }
  };
  private Pathfinder pathfinder;
  private String pathfinderKey;
  private Pathfinder widePathfinder; // the on-the-fly runs' 0.15 m pathfinder, tried first as on the robot
  private OnTheFly otf; // an on-the-fly run (op "otf"); null for a plain path
  private Optional<Pose> otfGoal = Optional.empty();

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
      case "load" -> {
        otf = null;
        yield load(msg);
      }
      case "pathfind" -> {
        otf = null;
        yield pathfind(msg);
      }
      case "otf" -> otf(msg);
      case "otfgoal" -> {
        otfGoal = Optional.of(pose(msg.get("goal")));
        yield "{\"ok\":true}";
      }
      case "replan" -> {
        otf.requestReplan();
        yield "{\"ok\":true}";
      }
      case "reset" -> reset(msg);
      case "step" -> step(msg);
      case "goal" -> goal(msg);
      case "geometry" -> geometry(msg);
      case "sense_reset" -> senseReset(msg);
      case "sense" -> sense(msg);
      default -> throw new IllegalArgumentException("unknown op " + op);
    };
  }

  private String load(Map<String, Object> msg) {
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
    if (msg.get("trajectory") != null) { // a trajectory planned elsewhere (Choreo): Path.fromTrajectory
      Map<String, Object> tr = PathFile.asMap(msg.get("trajectory"), "trajectory");
      path = Path.fromTrajectory(column(tr, "t"), column(tr, "x"), column(tr, "y"), column(tr, "heading"),
          column(tr, "vx"), column(tr, "vy"), robot.tractionMetersPerSecondSquared());
    } else {
      PathSpec spec = PathFile.fromJson(PathFile.asMap(msg.get("path"), "path"));
      path = msg.get("startCurvature") == null ? Path.build(spec, robot)
          : Path.build(spec, robot, VelocityProfile.DEFAULT_GRID_METERS, number(msg, "startCurvature"));
    }
    double buildMs = (System.nanoTime() - t0) / 1e6;
    follower = new PathFollower(path, b.build());
    goal = Optional.empty();
    if (!Boolean.TRUE.equals(msg.get("keepSensing"))) {
      sensing = null; // a run's sensing session starts after its load (sense_reset)
    } // the next path of a chain keeps the run's session
    follower.bindGoal(() -> goal);
    follower.withSlipSignal(slipSignal);
    Map<String, Object> r = new LinkedHashMap<>();
    r.put("ok", Boolean.TRUE);
    r.put("lengthMeters", path.lengthMeters());
    r.put("totalTimeSeconds", path.totalTimeSeconds());
    r.put("buildMs", buildMs);
    r.put("endCurvature", path.sample(path.lengthMeters()).curvature());
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

  /**
   * An on-the-fly run as the robot's {@code Pathfinding} commands make one: {@link OnTheFly} plans from the start
   * state with the 0.15 m pathfinder when its route isn't much longer, else the 0.05 m one, starts the follower,
   * and re-plans after a shove or a moved goal ({@code otfgoal}). Then {@code step} drives it. Both pathfinders
   * are built once per field, robot and limits and kept, as at boot.
   */
  private String otf(Map<String, Object> msg) {
    Map<String, Object> fieldTree = PathFile.asMap(msg.get("field"), "field");
    RobotConfig robot = msg.get("robot") == null
        ? RobotConfig.DEFAULT
        : RobotFile.fromJson(PathFile.asMap(msg.get("robot"), "robot"));
    PlannerConfig pc = PlannerConfig.DEFAULT;
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
    String key = line(fieldTree) + "|" + robot + "|" + pc + "|otf";
    Map<String, Object> r = new LinkedHashMap<>();
    r.put("ok", Boolean.TRUE);
    if (!key.equals(pathfinderKey)) {
      pathfinder = new Pathfinder(FieldMap.fromJson(fieldTree), robot, pc);
      widePathfinder = new Pathfinder(FieldMap.fromJson(fieldTree), robot,
          pc.withMargin(PlannerConfig.WIDE_MARGIN_METERS));
      pathfinderKey = key;
      r.put("bootMs", (pathfinder.bootMicros() + widePathfinder.bootMicros()) / 1e3);
    }
    Pose g = pose(msg.get("goal"));
    otf = new OnTheFly(pathfinder, b.build(), () -> g).preferring(widePathfinder);
    otfGoal = Optional.empty();
    if (Boolean.TRUE.equals(msg.get("movingGoal"))) {
      otf.withMovingGoal(() -> otfGoal);
    }
    boolean ok = otf.start(number(msg, "t"), pose(msg.get("start")), speeds(msg.get("vel")));
    r.put("planOk", ok);
    PlanResult plan = otf.lastPlan();
    if (ok) {
      follower = otf.follower();
      path = plan.path();
    }
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
    FieldSpeeds measured = speeds(msg.get("vel"));
    follower.start(number(msg, "t"), pose(msg.get("pose")), measured, frame,
        msg.get("handed") == null ? measured : speeds(msg.get("handed")));
    Map<String, Object> r = new LinkedHashMap<>();
    r.put("ok", Boolean.TRUE);
    r.put("startMicros", (System.nanoTime() - t0) / 1e3);
    if (!Double.isNaN(follower.lastStartReplanMicros())) {
      r.put("startReplanMicros", follower.lastStartReplanMicros());
    }
    return line(r);
  }

  private String step(Map<String, Object> msg) {
    need();
    long t0 = System.nanoTime();
    Map<String, Object> r = new LinkedHashMap<>();
    FieldSpeeds cmd;
    if (otf != null) {
      cmd = otf.step(number(msg, "t"), pose(msg.get("pose")), speeds(msg.get("vel")));
      r.put("replans", (double) otf.replans());
      r.put("maxPlanMicros", otf.maxPlanMicros());
      if (otf.follower() == null) { // a re-plan found no route
        r.put("cmd", List.of(0.0, 0.0, 0.0));
        r.put("done", Boolean.TRUE);
        r.put("failure", String.valueOf(otf.failure()));
        return line(r);
      }
      follower = otf.follower();
    } else {
      cmd = follower.step(number(msg, "t"), pose(msg.get("pose")), speeds(msg.get("vel")));
    }
    double micros = (System.nanoTime() - t0) / 1e3;
    r.put("cmd", List.of(cmd.vxMetersPerSecond(), cmd.vyMetersPerSecond(), cmd.omegaRadPerSec()));
    r.put("done", follower.isFinished());
    r.put("phase", follower.phase().name());
    r.put("s", follower.progressMeters());
    r.put("crossTrack", follower.crossTrackMeters());
    r.put("micros", micros);
    if (Double.isFinite(follower.slipFrictionCoefficient())) {
      r.put("slipMu", follower.slipFrictionCoefficient());
    }
    if (!Double.isNaN(follower.lastTailRebuildMicros())) {
      r.put("rebuilds", (double) follower.tailRebuildCount());
      r.put("rebuildMicros", follower.lastTailRebuildMicros());
    }
    if (follower.rejoinCount() > 0) {
      r.put("rejoins", (double) follower.rejoinCount());
      r.put("rejoinMicros", follower.lastRejoinMicros());
    }
    return line(r);
  }

  private String goal(Map<String, Object> msg) {
    need();
    Object p = msg.get("pose");
    goal = p == null ? Optional.empty() : Optional.of(pose(p));
    return "{\"ok\":true}";
  }

  /**
   * Starts a slip-sensing session: slip-aware odometry (with its detector) for a robot whose
   * modules sit at {@code modules}. Options are {@link SlipConfig} fields by name.
   */
  private String senseReset(Map<String, Object> msg) {
    List<?> mods = (List<?>) msg.get("modules");
    double[][] positions = new double[mods.size()][];
    for (int i = 0; i < positions.length; i++) {
      List<?> m = (List<?>) mods.get(i);
      positions[i] = new double[] {((Number) m.get(0)).doubleValue(), ((Number) m.get(1)).doubleValue()};
    }
    SlipConfig c = SlipConfig.DEFAULT;
    if (msg.get("options") != null) {
      for (Map.Entry<String, Object> e : PathFile.asMap(msg.get("options"), "options").entrySet()) {
        c = c.with(e.getKey(), ((Number) e.getValue()).doubleValue());
      }
    }
    sensing = new SlipAwareOdometry(positions, c);
    sensing.reset(pose(msg.get("pose")), number(msg, "gyro"), array(msg.get("dist"), positions.length));
    return "{\"ok\":true}";
  }

  /** One odometry tick: returns the slip-aware pose and the detector's verdict. */
  private String sense(Map<String, Object> msg) {
    if (sensing == null) {
      throw new IllegalStateException("sense_reset first");
    }
    int n = sensing.detector().moduleCount();
    double[] imu = msg.get("imu") == null ? new double[] {Double.NaN, Double.NaN} : array(msg.get("imu"), 2);
    double[] cmd = msg.get("cmd") == null ? new double[] {Double.NaN, Double.NaN, Double.NaN} : array(msg.get("cmd"), 3);
    Pose p = sensing.update(number(msg, "t"), number(msg, "gyro"), array(msg.get("dist"), n),
        array(msg.get("ang"), n), array(msg.get("spd"), n), imu[0], imu[1], cmd[0], cmd[1], cmd[2]);
    StringBuilder slip = new StringBuilder("[");
    for (int i = 0; i < n; i++) {
      slip.append(i > 0 ? "," : "").append(sensing.detector().moduleSlipping(i));
    }
    slip.append(']');
    return "{\"pose\":[" + Json.exact(p.xMeters()) + "," + Json.exact(p.yMeters()) + ","
        + Json.exact(p.headingRadians()) + "],\"conf\":" + Json.exact(sensing.detector().confidence())
        + ",\"achieved\":" + Json.exact(sensing.detector().achievedAccelMetersPerSecondSquared())
        + ",\"slip\":" + slip + ",\"imuOnly\":" + sensing.usingImu()
        + ",\"gap\":" + (Double.isNaN(sensing.detector().imuGapMetersPerSecondSquared()) ? "null"
            : Json.exact(sensing.detector().imuGapMetersPerSecondSquared()))
        + ",\"residual\":" + Json.exact(maxResidual())
        + ",\"vel\":[" + Json.exact(sensing.fieldVx()) + "," + Json.exact(sensing.fieldVy()) + "]}";
  }

  private double maxResidual() {
    double m = 0.0;
    for (int i = 0; i < sensing.detector().moduleCount(); i++) {
      m = Math.max(m, sensing.detector().moduleResidual(i));
    }
    return m;
  }

  private static double[] array(Object o, int n) {
    if (!(o instanceof List<?> l) || l.size() != n) {
      throw new IllegalArgumentException("expected a list of " + n + " numbers");
    }
    double[] v = new double[n];
    for (int i = 0; i < n; i++) {
      v[i] = ((Number) l.get(i)).doubleValue();
    }
    return v;
  }

  /** The built path sampled every {@code step} metres (default 0.02): x and y lists. */
  private String geometry(Map<String, Object> msg) {
    need();
    double step = msg.get("step") == null ? 0.02 : number(msg, "step");
    boolean profile = Boolean.TRUE.equals(msg.get("profile"));
    List<Object> xs = new ArrayList<>();
    List<Object> ys = new ArrayList<>();
    List<Object> ts = new ArrayList<>();
    List<Object> vs = new ArrayList<>();
    List<Object> as = new ArrayList<>();
    List<Object> ks = new ArrayList<>();
    List<Object> hs = new ArrayList<>();
    for (double s : ProfileSamples.stations(path.lengthMeters(), step)) {
      PathSample p = path.sample(s);
      xs.add(p.xMeters());
      ys.add(p.yMeters());
      if (profile) {
        ts.add(p.seconds());
        vs.add(p.metersPerSecond());
        as.add(p.metersPerSecondSquared());
        ks.add(p.curvature());
        hs.add(p.headingRadiansPerMeter());
      }
    }
    Map<String, Object> r = new LinkedHashMap<>();
    r.put("x", xs);
    r.put("y", ys);
    if (profile) { // the plan along the path: time, speed, tangential acceleration, curvature, heading rate
      r.put("t", ts);
      r.put("v", vs);
      r.put("a", as);
      r.put("k", ks);
      r.put("dh", hs);
    }
    return line(r);
  }

  private void need() {
    if (follower == null) {
      throw new IllegalStateException("load a path first");
    }
  }

  private static double[] column(Map<String, Object> m, String key) {
    if (!(m.get(key) instanceof List<?> list)) {
      throw new IllegalArgumentException("'" + key + "' must be a list of numbers");
    }
    double[] out = new double[list.size()];
    for (int i = 0; i < out.length; i++) {
      out[i] = ((Number) list.get(i)).doubleValue();
    }
    return out;
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
