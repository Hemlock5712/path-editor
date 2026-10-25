package frc.team5712.path.plan;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.sim.SimDrivetrain;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.Test;

/** On-the-fly paths on the 2026 field, on the noise-free simulated drivetrain (the WPILib runs are in e2eTest). */
class OnTheFlyTest {
  static final double LOOP = 0.02;

  static FieldMap field2026() {
    try {
      java.nio.file.Path f = java.nio.file.Path.of(System.getProperty("fixtures.dir", "../fixtures"), "plan", "field_2026.json");
      return FieldMap.read(f);
    } catch (IOException e) {
      throw new IllegalStateException(e);
    }
  }

  static final Pathfinder PLANNER = new Pathfinder(field2026(), RobotConfig.DEFAULT, PlannerConfig.DEFAULT);
  static final FollowerConfig CONFIG = FollowerConfig.defaults(RobotConfig.DEFAULT).toBuilder()
      .latencySeconds(SimDrivetrain.responseDelaySeconds(0.02, 0.04)).build();

  /** A random pose whose robot centre is free with 10 cm to spare. */
  static Pose randomFree(Random r) {
    while (true) {
      double x = (r.nextDouble() - 0.5) * 15.5;
      double y = (r.nextDouble() - 0.5) * 7.0;
      double h = (r.nextDouble() * 2 - 1) * Math.PI;
      if (PLANNER.isFree(x, y) && PLANNER.clearance(x, y, h) > 0.15) {
        return new Pose(x, y, h);
      }
    }
  }

  record Run(boolean arrived, double endError, double endHeadingDeg, double minClearance, int replans,
      double maxPlanMicros, double seconds, double maxCrossTrack) {}

  /** Plans from start at rest and drives; from shoveAt for shoveSeconds a push adds (dvx, dvy) every loop. */
  static Run drive(Pose start, Pose goal, double shoveAt, double shoveSeconds, double dvx, double dvy) {
    SimDrivetrain sim = new SimDrivetrain(start, new FieldSpeeds(0, 0, 0), 0.02, 0.04, 0.001);
    OnTheFly otf = new OnTheFly(PLANNER, CONFIG, () -> goal);
    double minClear = Double.POSITIVE_INFINITY;
    double maxCt = 0;
    boolean ok = otf.start(0.0, start, new FieldSpeeds(0, 0, 0));
    assertTrue(ok, () -> "no route: " + otf.failure());
    double t = 0;
    for (int k = 0; k < 2000; k++) {
      t = k * LOOP;
      for (int j = 1; j <= 4; j++) { // the true pose every 5 ms
        sim.advanceTo(t - LOOP + j * LOOP / 4);
        Pose p = sim.pose();
        minClear = Math.min(minClear, PLANNER.clearance(p.xMeters(), p.yMeters(), p.headingRadians()));
      }
      if (t >= shoveAt && t < shoveAt + shoveSeconds) {
        sim.shove(dvx, dvy);
      }
      Pose p = sim.pose();
      sim.command(otf.step(t, p, sim.velocity()));
      if (otf.follower() != null) {
        maxCt = Math.max(maxCt, Math.abs(otf.follower().crossTrackMeters()));
      }
      if (otf.isFinished()) {
        sim.command(new FieldSpeeds(0, 0, 0));
        break;
      }
    }
    sim.advanceTo(t + 0.5);
    Pose end = sim.pose();
    return new Run(otf.isFinished(), Math.hypot(end.xMeters() - goal.xMeters(), end.yMeters() - goal.yMeters()),
        Math.toDegrees(Math.abs(Math.IEEEremainder(end.headingRadians() - goal.headingRadians(), 2 * Math.PI))),
        minClear, otf.replans(), otf.maxPlanMicros(), t, maxCt);
  }

  @Test
  void randomRoutesOnThe2026FieldArriveWithoutTouchingAnything() {
    Random r = new Random(2026);
    List<String> bad = new ArrayList<>();
    double worst = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 30; i++) {
      Pose s = randomFree(r);
      Pose g = randomFree(r);
      Run run = drive(s, g, Double.POSITIVE_INFINITY, 0, 0, 0);
      worst = Math.min(worst, run.minClearance);
      if (!run.arrived || run.endError > 0.03 || run.endHeadingDeg > 1.5 || run.minClearance <= 0) {
        bad.add(i + ": " + s + " -> " + g + " " + run);
      }
    }
    System.out.printf("on-the-fly, 30 routes: worst bumper clearance %.3f m%n", worst);
    assertEquals(List.of(), bad);
  }

  @Test
  void aHardShoveIsRecoveredByPlanningAgain() {
    Pose s = new Pose(-6.0, -1.5, 0.0);
    Pose g = new Pose(1.0, -1.5, Math.PI / 2); // through the lower trench
    // a defender pushes the robot sideways (away from the bump) for 0.4 s on the way to the trench
    Run run = drive(s, g, 0.8, 0.4, -0.85, -0.53);
    System.out.println("shove: " + run);
    assertTrue(run.replans >= 1, run.toString());
    assertTrue(run.arrived && run.endError < 0.03, run.toString());
  }

  @Test
  void aStartInsideAnObstacleIsSnappedOut() {
    Pose s = new Pose(-3.4, 0.3, 0.0); // on the blue hub
    PlanResult r = PLANNER.plan(s, new FieldSpeeds(0, 0, 0), new Pose(-6.0, 2.0, 0.0));
    assertTrue(r.ok() && r.startSnapped(), r.toString());
  }

  @Test
  void theWideBerthIsUsedWhereThereIsRoomAndTheTightOneThroughAGap() {
    // a wall across the middle of an empty field with a 1.4 m gap: the 0.85 m robot needs 1.30 m at the 5 cm
    // margin, 1.50 m at the wide 15 cm one
    double[][] lower = {{-0.15, -4.0}, {0.15, -4.0}, {0.15, -0.7}, {-0.15, -0.7}};
    double[][] upper = {{-0.15, 0.7}, {0.15, 0.7}, {0.15, 4.0}, {-0.15, 4.0}};
    FieldMap wall = new FieldMap(16.54, 8.07, List.of("lower", "upper"),
        List.of(ConvexPolygon.of(lower), ConvexPolygon.of(upper)));
    Pathfinder tight = new Pathfinder(wall, RobotConfig.DEFAULT, PlannerConfig.DEFAULT);
    Pathfinder wide = new Pathfinder(wall, RobotConfig.DEFAULT, PlannerConfig.DEFAULT.withMargin(PlannerConfig.WIDE_MARGIN_METERS));
    FieldSpeeds still = new FieldSpeeds(0, 0, 0);
    OnTheFly through = new OnTheFly(tight, CONFIG, () -> new Pose(3.0, 0.0, 0.0)).preferring(wide);
    assertTrue(through.start(0.0, new Pose(-3.0, 0.0, 0.0), still));
    assertTrue(!through.lastPlanWide(), "the wide berth can't use the gap: the tight plan goes through it");
    OnTheFly beside = new OnTheFly(tight, CONFIG, () -> new Pose(-1.5, 3.0, 0.0)).preferring(wide);
    assertTrue(beside.start(0.0, new Pose(-3.0, -2.0, 0.0), still));
    assertTrue(beside.lastPlanWide(), "on this side of the wall there is room for the wide berth");
  }

  @Test
  void aRouteTheSmootherCouldNotClearLosesToAClearWideOne() {
    // no refits: the spline through the route's corners is left to bow into the obstacles
    PlannerConfig d = PlannerConfig.DEFAULT;
    Pathfinder noRefits = new Pathfinder(field2026(), RobotConfig.DEFAULT, new PlannerConfig(d.bumperLengthMeters(),
        d.bumperWidthMeters(), d.marginMeters(), d.arcStepRadians(), 0, d.checkStepMeters(), d.maxMetersPerSecond(),
        d.maxMetersPerSecondSquared()));
    Pathfinder wide = new Pathfinder(field2026(), RobotConfig.DEFAULT, d.withMargin(PlannerConfig.WIDE_MARGIN_METERS));
    FieldSpeeds still = new FieldSpeeds(0, 0, 0);
    Random r = new Random(7);
    for (int i = 0; i < 200; i++) {
      Pose a = randomFree(r);
      Pose b = randomFree(r);
      PlanResult tight = noRefits.plan(a, still, b, 0, Double.NaN);
      PlanResult w = wide.plan(a, still, b, 0, Double.NaN);
      if (!tight.ok() || tight.clear() || !w.ok() || !w.clear()) {
        continue;
      }
      OnTheFly alone = new OnTheFly(noRefits, CONFIG, () -> b);
      assertTrue(alone.start(0.0, a, still), "with nothing better it still drives");
      assertTrue(!alone.lastPlan().clear());
      OnTheFly withWide = new OnTheFly(noRefits, CONFIG, () -> b).preferring(wide);
      assertTrue(withWide.start(0.0, a, still));
      assertTrue(withWide.lastPlanWide() && withWide.lastPlan().clear(), "the clear wide route wins, even the long way");
      return;
    }
    throw new AssertionError("no start and goal where the smoother needed refits");
  }

  @Test
  void aGoalThatMovesBehindAnObstacleIsPlannedRound() {
    // heading for a spot by the blue hub's left side, the goal jumps to its right side: bending the path's end
    // straight over to it would drive through the hub, so it must plan again round it
    Pose start = new Pose(-6.0, -1.5, 0.0);
    Pose first = new Pose(-5.2, 0.0, 0.0);
    Pose moved = new Pose(-2.1, 0.0, 0.0);
    SimDrivetrain sim = new SimDrivetrain(start, new FieldSpeeds(0, 0, 0), 0.02, 0.04, 0.001);
    double[] now = {0};
    OnTheFly otf = new OnTheFly(PLANNER, CONFIG, () -> first)
        .withMovingGoal(() -> java.util.Optional.of(now[0] >= 0.5 ? moved : first));
    assertTrue(otf.start(0.0, start, new FieldSpeeds(0, 0, 0)));
    double minClear = Double.POSITIVE_INFINITY;
    double t = 0;
    for (int k = 0; k < 1000 && !otf.isFinished(); k++) {
      t = now[0] = k * LOOP;
      sim.advanceTo(t);
      Pose p = sim.pose();
      minClear = Math.min(minClear, PLANNER.clearance(p.xMeters(), p.yMeters(), p.headingRadians()));
      sim.command(otf.step(t, p, sim.velocity()));
    }
    sim.command(new FieldSpeeds(0, 0, 0));
    sim.advanceTo(t + 0.5);
    Pose end = sim.pose();
    assertTrue(minClear > 0, "touched the hub: clearance " + minClear);
    assertTrue(otf.isFinished() && otf.replans() >= 1, "replans " + otf.replans());
    assertTrue(Math.hypot(end.xMeters() - moved.xMeters(), end.yMeters() - moved.yMeters()) < 0.03);
  }

  @Test
  void aHandOverArrivesAlongTheGivenDirection() {
    OnTheFly otf = new OnTheFly(PLANNER, CONFIG, () -> new Pose(1.0, 3.25, 0.0)).arriving(1.5, 0.0);
    assertTrue(otf.start(0.0, new Pose(-6.0, 2.0, 0.0), new FieldSpeeds(0, 0, 0)));
    var path = otf.lastPlan().path();
    var end = path.sample(path.lengthMeters());
    assertEquals(1.0, end.tangentX(), 1e-6);
    assertEquals(1.5, end.metersPerSecond(), 1e-9);
    assertTrue(otf.rollingEnd());
  }

  /** fixtures/plan/plans.json must be what this code writes; run `lib/gradlew -p lib fixtures` if not. */
  @Test
  void committedPlanFixturesAreUpToDate() throws IOException {
    java.nio.file.Path dir = java.nio.file.Path.of(System.getProperty("fixtures.dir", "../fixtures"));
    String committed = Files.readString(dir.resolve(frc.team5712.path.tools.PlanFixtureGen.OUT));
    assertEquals(frc.team5712.path.tools.PlanFixtureGen.fixture(dir), committed.replace("\r\n", "\n"),
        "fixtures/plan/plans.json is stale: run lib/gradlew -p lib fixtures");
  }

  @Test
  void theFieldFileIsTheEditorsFormat() throws IOException {
    String text = Files.readString(java.nio.file.Path.of(System.getProperty("fixtures.dir", "../fixtures"), "plan",
        "field_2026.json"));
    assertTrue(text.contains("\"length_m\"") && text.contains("\"obstacles\""));
    assertEquals(8, field2026().obstacles().size());
  }
}
