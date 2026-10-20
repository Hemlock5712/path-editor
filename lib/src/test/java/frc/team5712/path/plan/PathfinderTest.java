package frc.team5712.path.plan;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Json;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;
import org.junit.jupiter.api.Test;

class PathfinderTest {
  static final FieldSpeeds STOPPED = new FieldSpeeds(0, 0, 0);

  static FieldMap fieldV1() {
    try (InputStream in = PathfinderTest.class.getResourceAsStream("/field_v1.json")) {
      return FieldMap.fromJson(PathFile.asMap(Json.parse(new String(in.readAllBytes(), StandardCharsets.UTF_8)), "f"));
    } catch (IOException e) {
      throw new IllegalStateException(e);
    }
  }

  static FieldMap field(double[][]... polys) {
    List<String> names = new ArrayList<>();
    List<ConvexPolygon> ps = new ArrayList<>();
    for (double[][] p : polys) {
      names.add("o" + names.size());
      ps.add(ConvexPolygon.of(p));
    }
    return new FieldMap(16.54, 8.02, names, ps);
  }

  static double[][] square(double cx, double cy, double half) {
    return new double[][] {{cx - half, cy - half}, {cx + half, cy - half}, {cx + half, cy + half}, {cx - half, cy + half}};
  }

  static Pathfinder planner(FieldMap f) {
    return new Pathfinder(f, RobotConfig.DEFAULT, PlannerConfig.DEFAULT);
  }

  @Test
  void loadsTheBenchField() {
    FieldMap f = fieldV1();
    assertEquals(6, f.obstacles().size());
    assertEquals("pillar_a", f.names().get(0));
    assertEquals(16.54, f.lengthMeters());
  }

  @Test
  void grownPolygonContainsTheDiscSumAndNotMuchMore() {
    ConvexPolygon p = ConvexPolygon.of(new double[][] {{1.6, -2.2}, {2.8, -2.2}, {2.2, -1.0}});
    double r = 0.65;
    double step = Math.toRadians(30);
    ConvexPolygon g = p.grow(r, step);
    Random rnd = new Random(1);
    double slack = r / Math.cos(step / 2) - r;
    for (int i = 0; i < 20000; i++) {
      double x = 0.5 + rnd.nextDouble() * 3.5;
      double y = -3.5 + rnd.nextDouble() * 3.5;
      double d = p.distanceTo(x, y);
      if (d < r - 1e-6) {
        assertTrue(g.distanceTo(x, y) == 0.0, "inside the disc sum but outside the grown polygon");
      }
      if (d > r + slack + 1e-6) {
        assertTrue(g.distanceTo(x, y) > 0.0, "grown polygon reaches too far");
      }
    }
  }

  @Test
  void straightLineWhenNothingIsInTheWay() {
    Pathfinder pf = planner(field());
    PlanResult r = pf.plan(new Pose(-3, 0, 0), STOPPED, new Pose(3, 1, 0));
    assertTrue(r.ok());
    assertEquals(2, r.waypoints().size());
    assertEquals(Math.hypot(6, 1), r.routeLengthMeters(), 1e-12);
    assertEquals(Math.hypot(6, 1), r.path().lengthMeters(), 1e-3);
  }

  /** Shortest route around one convex obstacle, worked out independently by wrapping its hull. */
  @Test
  void shortestAroundOneObstacleMatchesHullWrap() {
    Random rnd = new Random(7);
    for (int trial = 0; trial < 60; trial++) {
      double cx = rnd.nextDouble() * 2 - 1;
      double cy = rnd.nextDouble() * 2 - 1;
      FieldMap f = field(square(cx, cy, 0.3 + rnd.nextDouble() * 0.5));
      Pathfinder pf = planner(f);
      ConvexPolygon g = pf.grownObstacles().get(0);
      double[] s = {-4 + rnd.nextDouble(), cy + rnd.nextDouble() - 0.5};
      double[] e = {4 - rnd.nextDouble(), cy + rnd.nextDouble() - 0.5};
      PlanResult r = pf.plan(new Pose(s[0], s[1], 0), STOPPED, new Pose(e[0], e[1], 0));
      assertTrue(r.ok());
      assertEquals(hullWrap(g, s, e), r.routeLengthMeters(), 1e-9, "trial " + trial);
    }
  }

  private static double hullWrap(ConvexPolygon g, double[] s, double[] e) {
    if (!g.blocksSegment(s[0], s[1], e[0], e[1])) {
      return Math.hypot(e[0] - s[0], e[1] - s[1]);
    }
    int n = g.size();
    double best = Double.POSITIVE_INFINITY;
    for (int i = 0; i < n; i++) {
      if (g.blocksSegment(s[0], s[1], g.x(i), g.y(i))) {
        continue;
      }
      for (int j = 0; j < n; j++) {
        if (g.blocksSegment(g.x(j), g.y(j), e[0], e[1])) {
          continue;
        }
        double ends = Math.hypot(g.x(i) - s[0], g.y(i) - s[1]) + Math.hypot(e[0] - g.x(j), e[1] - g.y(j));
        double ccw = 0;
        for (int k = i; k != j; k = (k + 1) % n) {
          ccw += Math.hypot(g.x((k + 1) % n) - g.x(k), g.y((k + 1) % n) - g.y(k));
        }
        double cw = 0;
        for (int k = i; k != j; k = (k + n - 1) % n) {
          cw += Math.hypot(g.x((k + n - 1) % n) - g.x(k), g.y((k + n - 1) % n) - g.y(k));
        }
        best = Math.min(best, ends + Math.min(ccw, cw));
      }
    }
    return best;
  }

  @Test
  void symmetricObstacleGivesEqualSides() {
    Pathfinder pf = planner(field(square(0, 0, 0.5)));
    PlanResult up = pf.plan(new Pose(-3, 0, 0), STOPPED, new Pose(3, 0, 0));
    Pathfinder mirrored = planner(field(square(0, 0, 0.5)));
    PlanResult again = mirrored.plan(new Pose(3, 0, 0), STOPPED, new Pose(-3, 0, 0));
    assertEquals(up.routeLengthMeters(), again.routeLengthMeters(), 1e-9);
  }

  @Test
  void blockedGoalFailsCleanly() {
    // a wall right across the field: nothing gets through
    Pathfinder pf = planner(field(new double[][] {{-0.2, -4.01}, {0.2, -4.01}, {0.2, 4.01}, {-0.2, 4.01}}));
    PlanResult r = pf.plan(new Pose(-3, 0, 0), STOPPED, new Pose(3, 0, 0));
    assertFalse(r.ok());
    assertTrue(r.failure().contains("no route"));
  }

  @Test
  void startAndGoalInsideGrownObstaclesAreSnappedAndReported() {
    FieldMap f = fieldV1();
    Pathfinder pf = planner(f);
    // 0.3 m from pillar_a's left edge: inside its grown shape
    Pose start = new Pose(-3.1, -0.9, 0);
    Pose goal = new Pose(3.1, 0.9, 0); // 0.3 m right of rect_e: inside its grown shape
    PlanResult r = pf.plan(start, STOPPED, goal);
    assertTrue(r.ok());
    assertTrue(r.startSnapped() && r.goalSnapped());
    assertEquals(start.xMeters(), r.waypoints().get(0)[0], 1e-12);
    double[] snapped = r.waypoints().get(1);
    assertTrue(pf.isFree(snapped[0], snapped[1]));
    PathSample end = r.path().sample(r.path().lengthMeters());
    assertEquals(goal.xMeters(), end.xMeters(), 1e-6);
  }

  /** Random start/goal pairs on the bench field: the route never enters a grown obstacle. */
  @Test
  void routesNeverEnterGrownObstacles() {
    Pathfinder pf = planner(fieldV1());
    Random rnd = new Random(42);
    int done = 0;
    while (done < 300) {
      double[] s = freePoint(pf, rnd);
      double[] g = freePoint(pf, rnd);
      PlanResult r = pf.plan(new Pose(s[0], s[1], 0), STOPPED, new Pose(g[0], g[1], 0));
      assertTrue(r.ok());
      List<double[]> w = r.waypoints();
      for (int i = 1; i < w.size(); i++) {
        double len = Math.hypot(w.get(i)[0] - w.get(i - 1)[0], w.get(i)[1] - w.get(i - 1)[1]);
        int n = (int) Math.ceil(len / 0.01);
        for (int k = 0; k <= n; k++) {
          double t = (double) k / n;
          double x = w.get(i - 1)[0] + t * (w.get(i)[0] - w.get(i - 1)[0]);
          double y = w.get(i - 1)[1] + t * (w.get(i)[1] - w.get(i - 1)[1]);
          for (ConvexPolygon p : pf.grownObstacles()) {
            assertFalse(p.containsStrictly(x, y), "route point inside a grown obstacle");
          }
        }
      }
      done++;
    }
  }

  /**
   * Swept bumpers on a few hundred random start/goal pairs and headings: the oriented rectangle along
   * the profiled path, with its planned heading, keeps the margin from every obstacle and wall.
   */
  @Test
  void sweptBumpersKeepTheMarginOnRandomPairs() {
    Pathfinder pf = planner(fieldV1());
    double margin = PlannerConfig.DEFAULT.marginMeters();
    Random rnd = new Random(2027);
    int notClear = 0;
    int refitted = 0;
    double worst = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 300; i++) {
      double[] s = freePoint(pf, rnd);
      double[] g = freePoint(pf, rnd);
      if (Math.hypot(g[0] - s[0], g[1] - s[1]) < 1.0) {
        i--; // pathfinding is for crossing the field; short hops are DriveToPose's job
        continue;
      }
      double hs = (rnd.nextDouble() * 2 - 1) * Math.PI;
      double hg = (rnd.nextDouble() * 2 - 1) * Math.PI;
      FieldSpeeds v = i % 3 == 0 ? new FieldSpeeds(rnd.nextDouble() * 4 - 2, rnd.nextDouble() * 4 - 2, 0) : STOPPED;
      PlanResult r;
      try {
        r = pf.plan(new Pose(s[0], s[1], hs), v, new Pose(g[0], g[1], hg));
      } catch (RuntimeException ex) {
        throw new AssertionError("pair " + i + ": " + java.util.Arrays.toString(s) + " " + hs + " " + v + " -> "
            + java.util.Arrays.toString(g) + " " + hg, ex);
      }
      assertTrue(r.ok());
      if (!r.clear()) {
        notClear++;
        continue;
      }
      refitted += r.refits() > 0 ? 1 : 0;
      Path p = r.path();
      for (double d = 0; d <= p.lengthMeters(); d += 0.01) {
        PathSample q = p.sample(d);
        double c = pf.clearance(q.xMeters(), q.yMeters(), q.headingRadians());
        worst = Math.min(worst, c);
        assertTrue(c > margin - 0.01, "clearance " + c + " at s=" + d + " on pair " + i);
      }
    }
    System.out.printf("swept bumpers on 300 random pairs: %d not clear after refits, %d needed refits, "
        + "closest %.3f m%n", notClear, refitted, worst);
    assertTrue(notClear <= 3, notClear + " plans could not keep the margin");
  }

  @Test
  void shortHopWithABigTurnFailsWithAReason() {
    Pathfinder pf = planner(fieldV1());
    PlanResult r = pf.plan(new Pose(-0.94, -1.33, -2.84), STOPPED, new Pose(-1.0, -1.53, 2.05));
    assertFalse(r.ok());
    assertTrue(r.failure().contains("drive to the pose directly"), r.failure());
  }

  @Test
  void deterministic() {
    FieldMap f = fieldV1();
    Pose s = new Pose(-4, -1.5, 0.3);
    Pose g = new Pose(3.8, 1.4, 1.2);
    FieldSpeeds v = new FieldSpeeds(0.5, 1.5, 0);
    PlanResult a = planner(f).plan(s, v, g);
    Pathfinder pf = planner(f);
    pf.plan(new Pose(0, -3, 0), STOPPED, new Pose(-5, 3, 0)); // an unrelated query in between
    PlanResult b = pf.plan(s, v, g);
    assertEquals(a.waypoints().size(), b.waypoints().size());
    for (int i = 0; i < a.waypoints().size(); i++) {
      assertArrayEquals(a.waypoints().get(i), b.waypoints().get(i), 0.0);
    }
    for (double d = 0; d < a.path().lengthMeters(); d += 0.1) {
      assertEquals(a.path().sample(d), b.path().sample(d));
    }
    assertEquals(a.path().totalTimeSeconds(), b.path().totalTimeSeconds(), 0.0);
  }

  @Test
  void movingStartLeavesAlongTheVelocityAtThatSpeed() {
    Pathfinder pf = planner(fieldV1());
    PlanResult r = pf.plan(new Pose(-5, 0, 0), new FieldSpeeds(0, 2, 0), new Pose(2.5, 3, 0));
    PathSample p0 = r.path().sample(0);
    assertEquals(0.0, p0.tangentX(), 1e-6);
    assertEquals(1.0, p0.tangentY(), 1e-6);
    assertEquals(2.0, p0.metersPerSecond(), 1e-9);
    PathSample end = r.path().sample(r.path().lengthMeters());
    assertEquals(0.0, end.metersPerSecond(), 1e-6);
  }

  @Test
  void endHeadingIsTheGoals() {
    Pathfinder pf = planner(fieldV1());
    PlanResult r = pf.plan(new Pose(1, -3, 0), STOPPED, new Pose(4.5, -0.5, Math.PI / 2));
    assertEquals(Math.PI / 2, r.path().sample(r.path().lengthMeters()).headingRadians(), 1e-9);
    assertEquals(0.0, r.path().sample(0).headingRadians(), 1e-9);
  }

  @Test
  void fieldFileErrorsAreClear() {
    Map<String, Object> bad = PathFile.asMap(Json.parse("{\"field\": {\"length_m\": 16}}"), "f");
    try {
      FieldMap.fromJson(bad);
      throw new AssertionError("expected a failure");
    } catch (IllegalArgumentException e) {
      assertTrue(e.getMessage().contains("width_m"));
    }
  }

  static double[] freePoint(Pathfinder pf, Random rnd) {
    while (true) {
      double x = (rnd.nextDouble() - 0.5) * 16.54;
      double y = (rnd.nextDouble() - 0.5) * 8.02;
      if (pf.isFree(x, y)) {
        return new double[] {x, y};
      }
    }
  }
}
