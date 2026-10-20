package frc.team5712.path.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.tools.FixturePaths;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.Test;

class PathGeometryTest {
  /** An independent, slow, very accurate arc-length inverse: fine cumulative table plus Newton. */
  static final class Reference {
    static final int STEPS = 4000;
    final QuinticSpline spline;
    final double[][] cumulative;
    final double[] segmentStart;
    final double length;

    Reference(QuinticSpline spline) {
      this.spline = spline;
      int n = spline.segmentCount();
      cumulative = new double[n][STEPS + 1];
      segmentStart = new double[n + 1];
      for (int k = 0; k < n; k++) {
        for (int i = 0; i < STEPS; i++) {
          cumulative[k][i + 1] = cumulative[k][i] + gauss(k, (double) i / STEPS, (double) (i + 1) / STEPS);
        }
        segmentStart[k + 1] = segmentStart[k] + cumulative[k][STEPS];
      }
      length = segmentStart[n];
    }

    double gauss(int k, double a, double b) {
      double[] x = {-0.9061798459386640, -0.5384693101056831, 0, 0.5384693101056831, 0.9061798459386640};
      double[] w = {0.2369268850561891, 0.4786286704993665, 0.5688888888888889, 0.4786286704993665, 0.2369268850561891};
      double sum = 0;
      for (int j = 0; j < 5; j++) {
        sum += w[j] * spline.speed(k, 0.5 * (a + b) + 0.5 * (b - a) * x[j]);
      }
      return 0.5 * (b - a) * sum;
    }

    /** The exact point at arc length s. */
    Vec2 point(double s) {
      int k = 0;
      while (k < spline.segmentCount() - 1 && segmentStart[k + 1] <= s) {
        k++;
      }
      double local = s - segmentStart[k];
      int i = 0;
      while (i < STEPS - 1 && cumulative[k][i + 1] <= local) {
        i++;
      }
      double u0 = (double) i / STEPS;
      double u = u0;
      for (int iter = 0; iter < 20; iter++) {
        double f = cumulative[k][i] + gauss(k, u0, u) - local;
        u -= f / spline.speed(k, u);
      }
      double[] d = new double[8];
      spline.evaluate(k, u, d);
      return new Vec2(d[0], d[1]);
    }
  }

  @Test
  void lengthMatchesAnIndependentIntegral() {
    for (PathSpec spec : FixturePaths.all().values()) {
      PathGeometry g = PathGeometry.build(spec.waypoints());
      Reference ref = new Reference(g.spline());
      assertEquals(ref.length, g.lengthMeters(), 1e-9);
      for (int i = 0; i < g.waypointCount(); i++) {
        assertEquals(ref.segmentStart[i], g.waypointS(i), 1e-9);
      }
    }
  }

  @Test
  void lookupByDistanceIsWithinHalfAMillimetre() {
    double worstFixture = 0;
    for (PathSpec spec : FixturePaths.all().values()) {
      PathGeometry g = PathGeometry.build(spec.waypoints());
      Reference ref = new Reference(g.spline());
      for (int i = 0; i <= 400; i++) {
        double s = g.lengthMeters() * i / 400.0;
        worstFixture = Math.max(worstFixture, g.pointAt(s).minus(ref.point(s)).norm());
      }
    }
    System.out.printf("lookup by s: worst position error on the fixture paths %.4f mm%n", worstFixture * 1000);
    assertTrue(worstFixture < 5e-4);

    Random rng = new Random(12);
    double worst = 0;
    for (int trial = 0; trial < 40; trial++) {
      List<Waypoint> w = QuinticSplineTest.randomWaypoints(rng);
      PathGeometry g = PathGeometry.build(w);
      Reference ref = new Reference(g.spline());
      for (int i = 0; i <= 200; i++) {
        double s = g.lengthMeters() * i / 200.0;
        Vec2 exact = ref.point(s);
        Vec2 got = g.pointAt(s);
        worst = Math.max(worst, got.minus(exact).norm());
      }
    }
    System.out.printf("lookup by s: worst position error on 40 random paths %.4f mm%n", worst * 1000);
    assertTrue(worst < 5e-4, "worst lookup error " + worst + " m");
  }

  @Test
  void distanceGrowsWithSAndPointsAreArcLengthApart() {
    PathGeometry g = PathGeometry.build(FixturePaths.hairpin().waypoints());
    double h = 0.01;
    Vec2 prev = g.pointAt(0);
    for (double s = h; s <= g.lengthMeters(); s += h) {
      Vec2 p = g.pointAt(s);
      double chord = p.minus(prev).norm();
      // a chord is never longer than its arc and only shorter by about κ²h³/24; the table's u(s)
      // interpolant may stretch or squeeze s by a few parts in 10,000
      assertTrue(chord <= h * (1 + 1e-3), "chord " + chord);
      assertTrue(chord >= h * (1 - 1e-3), "chord " + chord);
      prev = p;
    }
  }

  @Test
  void tangentAndCurvatureMatchFiniteDifferences() {
    PathGeometry g = PathGeometry.build(FixturePaths.autoRoute().waypoints());
    double h = 1e-4;
    for (double s = 0.1; s < g.lengthMeters() - 0.1; s += 0.137) {
      GeometrySample a = g.sample(s - h);
      GeometrySample b = g.sample(s);
      GeometrySample c = g.sample(s + h);
      assertEquals(1.0, Math.hypot(b.tangentX(), b.tangentY()), 1e-12);
      // dp/ds from the table matches the unit tangent to the table's stretch (parts in 10,000)
      assertEquals(b.tangentX(), (c.xMeters() - a.xMeters()) / (2 * h), 1e-3);
      assertEquals(b.tangentY(), (c.yMeters() - a.yMeters()) / (2 * h), 1e-3);
      double turn = Angles.wrap(c.tangentRadians() - a.tangentRadians()) / (2 * h);
      assertEquals(b.curvature(), turn, 1e-3);
      assertEquals(b.dCurvatureDs(), (c.curvature() - a.curvature()) / (2 * h), 1e-2);
    }
  }

  @Test
  void curvatureIsContinuousAlongTheWholePath() {
    Random rng = new Random(99);
    for (int trial = 0; trial < 50; trial++) {
      PathGeometry g = PathGeometry.build(QuinticSplineTest.randomWaypoints(rng));
      for (int i = 1; i < g.waypointCount() - 1; i++) {
        double s = g.waypointS(i);
        double before = g.curvatureAt(s - 1e-9);
        double after = g.curvatureAt(s + 1e-9);
        assertEquals(before, after, 1e-6);
      }
    }
  }

  @Test
  void waypointPositionsMapToTheirDistances() {
    PathSpec spec = FixturePaths.slalom();
    PathGeometry g = PathGeometry.build(spec.waypoints());
    for (int i = 0; i < g.waypointCount(); i++) {
      Vec2 p = g.pointAt(g.waypointS(i));
      assertEquals(spec.waypoints().get(i).xMeters(), p.x(), 1e-4);
      assertEquals(spec.waypoints().get(i).yMeters(), p.y(), 1e-4);
      assertEquals(g.waypointS(i), g.sAtWaypoint(i), 0.0);
    }
    assertEquals(0.5 * (g.waypointS(2) + g.waypointS(3)), g.sAtWaypoint(2.5), 1e-12);
    assertEquals(0.0, g.sAtWaypoint(-3), 0.0);
    assertEquals(g.lengthMeters(), g.sAtWaypoint(99), 0.0);
  }

  @Test
  void queriesClampToThePath() {
    PathGeometry g = PathGeometry.build(FixturePaths.sCurve().waypoints());
    assertEquals(0.0, g.sample(-1).s(), 0.0);
    assertEquals(g.lengthMeters(), g.sample(1e6).s(), 0.0);
    assertEquals(-4.0, g.pointAt(-1).x(), 1e-12);
    assertEquals(4.0, g.pointAt(100).x(), 1e-9);
  }

  @Test
  void tableStaysSmall() {
    PathGeometry g = PathGeometry.build(FixturePaths.autoRoute().waypoints());
    // about one sub-piece per 20 cm: nothing like the old 1,000 samples per metre
    assertTrue(g.tablePieceCount() < g.lengthMeters() * 10, "pieces " + g.tablePieceCount());
  }

  // ------------------------------------------------------------------------------ projection

  @Test
  void pointsOnThePathProjectToThemselves() {
    PathGeometry g = PathGeometry.build(FixturePaths.autoRoute().waypoints());
    for (double s = 0; s <= g.lengthMeters(); s += 0.31) {
      GeometrySample p = g.sample(s);
      Projection near = g.project(p.xMeters(), p.yMeters(), s + 0.2, 0.5);
      assertEquals(s, near.s(), 1e-7);
      assertEquals(0.0, near.distanceMeters(), 1e-7);
      Projection global = g.projectGlobal(p.xMeters(), p.yMeters());
      assertEquals(s, global.s(), 1e-7);
    }
  }

  @Test
  void sidewaysOffsetsGiveSignedCrossTrack() {
    PathGeometry g = PathGeometry.build(FixturePaths.sCurve().waypoints());
    for (double s = 0.5; s < g.lengthMeters() - 0.5; s += 0.5) {
      GeometrySample p = g.sample(s);
      for (double offset : new double[] {-0.3, 0.2}) {
        // stay inside the radius of curvature so the foot of the normal is the closest point
        double x = p.xMeters() - p.tangentY() * offset;
        double y = p.yMeters() + p.tangentX() * offset;
        Projection pr = g.project(x, y, s - 0.1, 0.6);
        assertEquals(s, pr.s(), 1e-7);
        assertEquals(offset, pr.crossTrackMeters(), 1e-7);
        assertEquals(Math.abs(offset), pr.distanceMeters(), 1e-7);
      }
    }
  }

  @Test
  void windowKeepsProgressOnTheRightLegOfAHairpin() {
    PathGeometry g = PathGeometry.build(FixturePaths.hairpin().waypoints());
    // (-1, 0) is equally near both legs; the window says which one the robot is on
    Projection early = g.project(-1.0, 0.0, 2.0, 0.75);
    Projection late = g.project(-1.0, 0.0, g.lengthMeters() - 2.0, 0.75);
    assertTrue(g.pointAt(early.s()).y() < 0, "first leg runs along y = -1");
    assertTrue(g.pointAt(late.s()).y() > 0, "last leg runs along y = +1");
  }

  @Test
  void windowClampsAndGlobalBeatsEverySample() {
    PathGeometry g = PathGeometry.build(FixturePaths.autoRoute().waypoints());
    Projection clamped = g.project(-5.44, 3.0, 1.0, 0.5); // the far end, searched near the start
    assertTrue(clamped.s() >= 0.5 - 1e-12 && clamped.s() <= 1.5 + 1e-12);

    Random rng = new Random(3);
    for (int i = 0; i < 200; i++) {
      double x = -7 + rng.nextDouble() * 9;
      double y = -4 + rng.nextDouble() * 8;
      Projection p = g.projectGlobal(x, y);
      for (double s = 0; s <= g.lengthMeters(); s += 0.01) {
        assertTrue(g.pointAt(s).minus(new Vec2(x, y)).norm() >= p.distanceMeters() - 1e-9);
      }
    }
  }

  @Test
  void guessOffThePathUsesTheNearerEnd() {
    PathGeometry g = PathGeometry.build(FixturePaths.straight().waypoints());
    assertEquals(0.0, g.project(-2, 0, -5, 1).s(), 0.0);
    assertEquals(g.lengthMeters(), g.project(2, 0, 10, 1).s(), 0.0);
  }
}
