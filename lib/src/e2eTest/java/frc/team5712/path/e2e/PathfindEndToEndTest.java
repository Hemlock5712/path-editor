package frc.team5712.path.e2e;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.core.PathSpec;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RotationTarget;
import frc.team5712.path.core.Waypoint;
import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.plan.OnTheFly;
import frc.team5712.path.plan.Pathfinder;
import frc.team5712.path.wpilib.Conversions;
import frc.team5712.path.wpilib.PathLibrary;
import frc.team5712.path.wpilib.PathfindCommand;
import frc.team5712.path.wpilib.PathfindCommandV2;
import frc.team5712.path.wpilib.Pathfinding;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.Random;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Subsystem;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DriverStationSim;

/**
 * On-the-fly paths on the 2026 field, through the WPILib adapters, on {@link SimDrive} at 50 Hz.
 *
 * <ul>
 *   <li>A deploy folder like a robot's: field.json (fixtures/plan/field_2026.json), a named point in
 *       .pathproject.json and one path, loaded by {@link PathLibrary#loadAll}, which builds the graph.
 *   <li>50 seeded random start and goal poses across the field, each through Commands v3 and v2, on blue and
 *       on red (the start and goal half-turned; the goal handed in as a blue-frame pose).
 *   <li>Every run: no bumper contact against the TRUE pose (checked every 5 ms against field.json's obstacles
 *       and the walls), arrival within 3 cm and 1.5°, and the planning time.
 *   <li>A sustained shove across the path partway that must make it plan again, and still arrive untouched.
 *   <li>To a library path's start and then the path (a rolling hand-over), and to a named point on a moving
 *       goal.
 *   <li>A library path on its own whose end moves (8 cm and 3°, seen when the robot is 1.5 m out), through
 *       {@code follow(...).withMovingGoal(...)}.
 * </ul>
 *
 * The figures go to e2e/PATHFIND.md.
 */
class PathfindEndToEndTest {
  private static final Path FIELD = Path.of(System.getProperty("fixtures.dir", "../fixtures"), "plan", "field_2026.json");
  private static final Path REPORT = Path.of(System.getProperty("pathfind.report", "../e2e/PATHFIND.md"));
  private static final int PAIRS = 50;
  private static final int SHOVES = 10;
  private static final double LOOP = Tolerances.LOOP_SECONDS;

  private static PathLibrary library;
  private static Pathfinder planner;
  private static final List<Result> results = new ArrayList<>();

  enum Framework {
    V3,
    V2
  }

  record Result(String kind, Framework framework, FieldFrame frame, int pair, boolean arrived, double endError,
      double endHeadingDeg, double minClearance, double firstPlanMicros, double maxPlanMicros, int replans,
      double seconds, List<String> failures) {
    boolean pass() {
      return failures.isEmpty();
    }
  }

  @BeforeAll
  static void load() throws IOException {
    Path dir = Files.createTempDirectory("otf-deploy");
    Files.copy(FIELD, dir.resolve(PathLibrary.FIELD_FILE));
    Files.writeString(dir.resolve(PathLibrary.PROJECT_FILE),
        "{\"schema\": 1, \"paths\": [], \"namedPoints\": [{\"id\": \"p1\", \"name\": \"Depot\", \"x\": -7.3, \"y\": 2.2}],"
            + " \"settings\": {}, \"view\": {}}", StandardCharsets.UTF_8);
    // a sweep through the neutral zone that starts moving: the on-the-fly path hands over at 1.5 m/s
    PathSpec sweep = PathSpec.of(List.of(Waypoint.at(-1.4, -2.2, Math.PI / 2), Waypoint.at(-1.4, 0.0),
            Waypoint.at(-1.4, 2.2, Math.PI / 2)))
        .withRotationTargets(List.of(RotationTarget.atWaypoint(0, Math.PI / 2), RotationTarget.atWaypoint(2, Math.PI / 2)))
        .withLimits(4.0, 6.0)
        .withEndSpeeds(1.5, 0.0);
    Files.writeString(dir.resolve("Sweep" + PathLibrary.SUFFIX), PathFile.write(sweep), StandardCharsets.UTF_8);
    library = PathLibrary.loadAll(dir.toFile());
    planner = library.pathfinder();
    HAL.initialize();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  static FollowerConfig config() {
    return FollowerConfig.defaults(library.robot()).toBuilder().latencySeconds(SimDrive.RESPONSE_DELAY_SECONDS).build();
  }

  /** Seeded random blue-frame poses, free with 15 cm to spare, start and goal at least 1 m apart. */
  static List<Pose[]> pairs() {
    Random r = new Random(5712);
    List<Pose[]> out = new ArrayList<>();
    while (out.size() < PAIRS) {
      Pose s = randomFree(r);
      Pose g = randomFree(r);
      if (Math.hypot(g.xMeters() - s.xMeters(), g.yMeters() - s.yMeters()) > 1.0) {
        out.add(new Pose[] {s, g});
      }
    }
    return out;
  }

  static Pose randomFree(Random r) {
    while (true) {
      double x = (r.nextDouble() - 0.5) * 15.6;
      double y = (r.nextDouble() - 0.5) * 7.2;
      double h = (r.nextDouble() * 2 - 1) * Math.PI;
      if (planner.isFree(x, y) && planner.clearance(x, y, h) > 0.15) {
        return new Pose(x, y, h);
      }
    }
  }

  /** The run in progress's drivetrain (for goals that depend on where the robot is). */
  private static SimDrive currentDrive;

  /** A shove: from {@code at} seconds for {@code seconds}, add (dvx, dvy) m/s every loop. */
  record Push(double at, double seconds, double dvx, double dvy) {}

  interface Maker {
    /** The command for this run: a v3 {@link Command} or a v2 command. */
    Object make(Pathfinding pf, Mechanism mechanism, Subsystem subsystem);
  }

  /** One run from {@code start} (real frame, at rest); the robot must end at {@code end} (real frame). */
  static Result run(String kind, int pair, Framework fw, FieldFrame frame, Pose start, Pose end, Maker maker,
      Supplier<Push> push, long seed) {
    SimDrive drive = new SimDrive(start, new FieldSpeeds(0, 0, 0), seed);
    currentDrive = drive;
    double[] clock = {0.0};
    Pathfinding pf = new Pathfinding(library, config(), drive::measuredPose, drive::measuredVelocity, drive::command,
        FieldFrame.RED_HALF_TURN).withClock(() -> clock[0]).withFrame(() -> frame);
    Scheduler v3 = Scheduler.createIndependentScheduler();
    Mechanism mechanism = new Mechanism() {
      @Override
      public Scheduler getRegisteredScheduler() {
        return v3;
      }
    };
    Subsystem subsystem = new Subsystem() {};
    CommandScheduler v2 = CommandScheduler.getInstance();
    v2.cancelAll();
    Object made = maker.make(pf, mechanism, subsystem);
    List<String> failures = new ArrayList<>();
    Supplier<OnTheFly> otf;
    Command c3 = null;
    org.wpilib.command2.Command c2 = null;
    if (fw == Framework.V3) {
      Command c = (Command) made;
      otf = c instanceof PathfindCommand pc ? pc::onTheFly : () -> null; // a library path alone plans nothing
      c3 = c;
      v3.schedule(c3);
    } else {
      org.wpilib.command2.Command c = (org.wpilib.command2.Command) made;
      PathfindCommandV2 first = c instanceof PathfindCommandV2 pc ? pc
          : c instanceof Pathfinding.PathfindThenFollowV2 g ? g.pathfind() : null;
      otf = first == null ? () -> null : first::onTheFly;
      c2 = c;
      v2.schedule(c2);
    }
    double minClear = Double.POSITIVE_INFINITY;
    String worstAt = "";
    double endT = Double.NaN;
    double firstPlan = Double.NaN;
    Push p = null;
    double t = 0;
    for (int k = 0; ; k++) {
      t = k * LOOP;
      for (int j = 1; j <= 4; j++) { // the true pose every 5 ms
        drive.advanceTo(t - LOOP + j * LOOP / 4);
        Pose q = drive.truePose();
        double c = planner.clearance(q.xMeters(), q.yMeters(), q.headingRadians());
        if (c < minClear) {
          minClear = c;
          worstAt = String.format(Locale.ROOT, "t %.2f s at (%.2f, %.2f, %.0f°)", t - LOOP + j * LOOP / 4, q.xMeters(),
              q.yMeters(), Math.toDegrees(q.headingRadians()));
        }
      }
      if (p == null && push != null && otf.get() != null) {
        p = push.get();
      }
      if (p != null && t >= p.at && t < p.at + p.seconds) {
        drive.push(p.dvx, p.dvy);
      }
      clock[0] = t;
      if (fw == Framework.V3) {
        v3.run();
      } else {
        v2.run();
      }
      if (Double.isNaN(firstPlan) && otf.get() != null && otf.get().lastPlan() != null) {
        firstPlan = otf.get().lastPlan().totalMicros();
      }
      boolean running = fw == Framework.V3 ? v3.isScheduledOrRunning(c3) : v2.isScheduled(c2);
      if (Double.isNaN(endT) && !running && k > 0) {
        endT = t;
      }
      if (!Double.isNaN(endT) && t >= endT + 0.5) {
        break;
      }
      if (t > 30) {
        failures.add("still running after 30 s");
        break;
      }
    }
    v3.cancelAll();
    v2.cancelAll();
    OnTheFly o = otf.get();
    if (o != null && o.failure() != null) {
      failures.add("no route: " + o.failure());
    }
    Pose q = drive.truePose();
    double err = Math.hypot(q.xMeters() - end.xMeters(), q.yMeters() - end.yMeters());
    double head = Math.toDegrees(Math.abs(Math.IEEEremainder(q.headingRadians() - end.headingRadians(), 2 * Math.PI)));
    if (err > Tolerances.END_POSITION_METERS || head > Tolerances.END_HEADING_DEGREES) {
      failures.add(String.format(Locale.ROOT, "ended %.1f cm and %.2f° from the goal", err * 100, head));
    }
    if (!(minClear > 0)) {
      failures.add(String.format(Locale.ROOT, "bumper contact (clearance %.3f m, %s)", minClear, worstAt));
    }
    return new Result(kind, fw, frame, pair, !Double.isNaN(endT), err, head, minClear, firstPlan,
        o == null ? Double.NaN : o.maxPlanMicros(), o == null ? 0 : o.replans(), endT, failures);
  }

  static Result check(Result r) {
    results.add(r);
    return r;
  }

  @Test
  void fiftyRandomRoutesOnBothAlliancesThroughV3AndV2() {
    List<Pose[]> pairs = pairs();
    List<String> failed = new ArrayList<>();
    for (FieldFrame frame : new FieldFrame[] {FieldFrame.BLUE, FieldFrame.RED_HALF_TURN}) {
      for (Framework fw : Framework.values()) {
        for (int i = 0; i < pairs.size(); i++) {
          Pose s = pairs.get(i)[0];
          Pose g = pairs.get(i)[1];
          Pose2d blueGoal = Conversions.toPose2d(g);
          Result r = check(run("route", i, fw, frame, frame.fromBlue(s), frame.fromBlue(g),
              (pf, m, sub) -> fw == Framework.V3 ? pf.to(m, blueGoal) : pf.toV2(sub, blueGoal), null, 1000L * i + 7));
          if (!r.pass()) {
            failed.add(r.kind() + " " + i + " " + fw + " " + frame + ": " + r.failures());
          }
        }
      }
    }
    assertEquals(List.of(), failed);
  }

  @Test
  void aShoveMidRouteMakesItPlanAgain() {
    List<Pose[]> pairs = pairs();
    List<String> failed = new ArrayList<>();
    int done = 0;
    for (int i = 0; i < pairs.size() && done < SHOVES; i++) {
      Pose s = pairs.get(i)[0];
      Pose g = pairs.get(i)[1];
      Push shove = placeShove(s, g);
      if (Math.hypot(g.xMeters() - s.xMeters(), g.yMeters() - s.yMeters()) < 4.0 || shove == null) {
        continue; // long enough to shove partway, with room to be shoved
      }
      done++;
      Pose2d blueGoal = Conversions.toPose2d(g);
      for (Framework fw : Framework.values()) {
        // push across the path, towards the side with more room, for 0.3 s
        Result r = check(run("shove", i, fw, FieldFrame.BLUE, s, g,
            (pf, m, sub) -> fw == Framework.V3 ? pf.to(m, blueGoal) : pf.toV2(sub, blueGoal), () -> shove, 77L * i));
        List<String> f = new ArrayList<>(r.failures());
        if (r.replans() < 1) {
          f.add("the shove didn't make it plan again");
        }
        if (!f.isEmpty()) {
          failed.add("shove " + i + " " + fw + ": " + f + " " + r);
        }
      }
    }
    assertEquals(SHOVES, done);
    assertEquals(List.of(), failed);
  }

  /**
   * Where and which way to push: the first place from a quarter of the way along the plan where there is room to
   * be pushed (the robot clear by half a metre, and still by 0.4 m pushed 0.9 m sideways), towards the side with
   * more room; null if the route has no such place. A push into an obstacle is contact by the pusher, which no
   * planner can prevent, so the suite pushes where the robot can take it.
   */
  static Push placeShove(Pose s, Pose g) {
    // the plan the command will drive: the wide berth where it fits, as the commands choose it
    OnTheFly otf = new OnTheFly(planner, config(), () -> g).preferring(library.widePathfinder());
    otf.start(0.0, s, new FieldSpeeds(0, 0, 0));
    var path = otf.lastPlan().path();
    for (double d = 0.25 * path.lengthMeters(); d < 0.6 * path.lengthMeters(); d += 0.05) {
      var p = path.sample(d);
      double nx = -p.tangentY();
      double ny = p.tangentX();
      double here = planner.clearance(p.xMeters(), p.yMeters(), p.headingRadians());
      double left = planner.clearance(p.xMeters() + 0.9 * nx, p.yMeters() + 0.9 * ny, p.headingRadians());
      double right = planner.clearance(p.xMeters() - 0.9 * nx, p.yMeters() - 0.9 * ny, p.headingRadians());
      if (here > 0.5 && Math.max(left, right) > 0.4) {
        double sign = left >= right ? 1 : -1;
        return new Push(p.seconds(), 0.3, 0.7 * sign * nx, 0.7 * sign * ny);
      }
    }
    return null;
  }

  @Test
  void toALibraryPathsStartThenThePath() {
    List<String> failed = new ArrayList<>();
    var sweep = library.get("Sweep");
    var end = sweep.sample(sweep.lengthMeters());
    Pose endBlue = new Pose(end.xMeters(), end.yMeters(), end.headingRadians());
    Pose[] starts = {new Pose(-6.0, 2.5, 0.0), new Pose(-6.5, -2.8, Math.PI / 2), new Pose(1.5, -3.0, Math.PI)};
    for (FieldFrame frame : new FieldFrame[] {FieldFrame.BLUE, FieldFrame.RED_HALF_TURN}) {
      for (Framework fw : Framework.values()) {
        for (int i = 0; i < starts.length; i++) {
          Result r = check(run("thenFollow", i, fw, frame, frame.fromBlue(starts[i]), frame.fromBlue(endBlue),
              (pf, m, sub) -> fw == Framework.V3 ? pf.thenFollow(m, "Sweep") : pf.thenFollowV2(sub, "Sweep"), null,
              31L * i));
          if (!r.pass()) {
            failed.add("thenFollow " + i + " " + fw + " " + frame + ": " + r.failures());
          }
        }
      }
    }
    assertEquals(List.of(), failed);
  }

  /**
   * A library path (no pathfinding) whose end follows a goal seen on the field: nothing until the robot is 1.5 m
   * out, then the goal 8 cm along and 3° round from the authored end (a tag or piece estimate, in the real frame).
   */
  @Test
  void aLibraryPathWhoseEndMoves() {
    List<String> failed = new ArrayList<>();
    var sweep = library.get("Sweep");
    var end = sweep.sample(sweep.lengthMeters());
    var start = sweep.sample(0.0);
    Pose startBlue = new Pose(start.xMeters(), start.yMeters(), start.headingRadians());
    Pose movedBlue = new Pose(end.xMeters() + 0.08, end.yMeters(), end.headingRadians() + Math.toRadians(3.0));
    for (FieldFrame frame : new FieldFrame[] {FieldFrame.BLUE, FieldFrame.RED_HALF_TURN}) {
      Pose endReal = frame.fromBlue(new Pose(end.xMeters(), end.yMeters(), end.headingRadians()));
      Pose movedReal = frame.fromBlue(movedBlue);
      for (Framework fw : Framework.values()) {
        Supplier<Optional<Pose2d>> seen = () -> {
          Pose q = currentDrive.truePose();
          return Math.hypot(q.xMeters() - endReal.xMeters(), q.yMeters() - endReal.yMeters()) < 1.5
              ? Optional.of(Conversions.toPose2d(movedReal)) : Optional.empty();
        };
        Result r = check(run("followMovingGoal", 0, fw, frame, frame.fromBlue(startBlue), movedReal,
            (pf, m, sub) -> fw == Framework.V3 ? pf.follow(m, "Sweep").withMovingGoal(seen)
                : pf.followV2(sub, "Sweep").withMovingGoal(seen), null, 41L));
        if (!r.pass()) {
          failed.add("followMovingGoal " + fw + " " + frame + ": " + r.failures());
        }
      }
    }
    assertEquals(List.of(), failed);
  }

  @Test
  void toANamedPointThatMoves() {
    List<String> failed = new ArrayList<>();
    double[] depot = library.namedPoint("Depot");
    for (Framework fw : Framework.values()) {
      // the goal is seen 25 cm further along the wall after a second (a moving game piece, in the real frame)
      Pose moved = new Pose(depot[0], depot[1] - 0.25, Math.PI);
      Supplier<Optional<Pose2d>> seen = () -> Optional.of(Conversions.toPose2d(moved));
      Result r = check(run("movingGoal", 0, fw, FieldFrame.BLUE, new Pose(-2.0, 3.2, 0.0), moved,
          (pf, m, sub) -> fw == Framework.V3
              ? pf.toPoint(m, "Depot", Rotation2d.k180deg).withMovingGoal(seen)
              : pf.toPointV2(sub, "Depot", Rotation2d.k180deg).withMovingGoal(seen), null, 99));
      if (!r.pass()) {
        failed.add("movingGoal " + fw + ": " + r.failures());
      }
    }
    assertEquals(List.of(), failed);
  }

  @AfterAll
  static void report() throws IOException {
    if (results.isEmpty()) {
      return;
    }
    StringBuilder md = new StringBuilder();
    md.append("# On-the-fly paths: end-to-end results\n\n");
    md.append("Generated by `lib/gradlew -p lib e2eTest --tests '*PathfindEndToEndTest'`: on-the-fly paths on the 2026 "
        + "field (fixtures/plan/field_2026.json), through the WPILib adapters (Commands v3 and v2), on the e2e "
        + "SimDrive at 50 Hz. Contact is checked against the TRUE pose every 5 ms: the bumper rectangle against "
        + "field.json's obstacles and the walls. Arrival: within 3 cm and 1.5° of the goal. The followMovingGoal rows "
        + "follow a library path on its own (no planning) whose end is seen 8 cm and 3° away when the robot is 1.5 m "
        + "out; they must arrive on the moved goal.\n\n");
    md.append(String.format(Locale.ROOT, "Boot: field.json read and visibility graph built in %.1f ms, of which %.1f ms "
        + "warming the planner up (%d plans), %d graph nodes.%n%n", library.fieldMillis(), library.warmUpMillis(),
        40, planner.nodeCount()));
    md.append("| runs | pass | worst clearance cm | first plan ms median / p95 / max | re-plans | slowest plan in a run ms |\n");
    md.append("|---|---|---|---|---|---|\n");
    for (String kind : List.of("route", "shove", "thenFollow", "movingGoal", "followMovingGoal")) {
      for (Framework fw : Framework.values()) {
        for (FieldFrame frame : new FieldFrame[] {FieldFrame.BLUE, FieldFrame.RED_HALF_TURN}) {
          List<Result> rs = results.stream().filter(r -> r.kind().equals(kind) && r.framework() == fw && r.frame() == frame)
              .toList();
          if (rs.isEmpty()) {
            continue;
          }
          double[] first = rs.stream().mapToDouble(r -> r.firstPlanMicros() / 1000).filter(Double::isFinite).sorted()
              .toArray();
          // a library path followed on its own plans nothing
          String plans = first.length == 0 ? "- | - | -" : String.format(Locale.ROOT, "%.2f / %.2f / %.2f | %d | %.2f",
              pct(first, 0.5), pct(first, 0.95), first[first.length - 1], rs.stream().mapToInt(Result::replans).sum(),
              rs.stream().mapToDouble(r -> r.maxPlanMicros() / 1000).max().orElse(Double.NaN));
          md.append(String.format(Locale.ROOT, "| %s, %s, %s: %d | %d | %.1f | %s |%n", kind, fw, frame, rs.size(),
              rs.stream().filter(Result::pass).count(),
              100 * rs.stream().mapToDouble(Result::minClearance).min().orElse(Double.NaN), plans));
        }
      }
    }
    List<String> unshoved = results.stream().filter(r -> !r.kind().equals("shove") && r.replans() > 0)
        .map(r -> r.kind() + " " + r.pair() + " " + r.framework() + " " + r.frame()).toList();
    if (!unshoved.isEmpty()) {
      md.append("\nRuns that planned again without being shoved (more than 20 cm off their plan): ")
          .append(String.join(", ", unshoved)).append(".\n");
    }
    long pass = results.stream().filter(Result::pass).count();
    md.append(String.format(Locale.ROOT, "%n**%d of %d runs pass.** Worst bumper clearance %.1f cm.%n", pass, results.size(),
        100 * results.stream().mapToDouble(Result::minClearance).min().orElse(Double.NaN)));
    Files.writeString(REPORT, md.toString(), StandardCharsets.UTF_8);
    System.out.println(md);
  }

  static double pct(double[] sorted, double q) {
    if (sorted.length == 0) {
      return Double.NaN;
    }
    return sorted[Math.min(sorted.length - 1, (int) Math.floor(q * (sorted.length - 1) + 0.5))];
  }
}
