package frc.team5712.path.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.tools.FixturePaths;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.Test;

class QuinticSplineTest {
  /** Random paths: 2-12 waypoints wandering about the field, some with fixed tangents and scales. */
  static List<Waypoint> randomWaypoints(Random rng) {
    int n = 2 + rng.nextInt(11);
    List<Waypoint> w = new ArrayList<>();
    double x = -6 + rng.nextDouble() * 2;
    double y = -3 + rng.nextDouble() * 6;
    for (int i = 0; i < n; i++) {
      double tangent = rng.nextDouble() < 0.3 ? (rng.nextDouble() * 2 - 1) * Math.PI : Double.NaN;
      double scale = rng.nextDouble() < 0.2 ? 0.6 + rng.nextDouble() * 0.8 : 1.0;
      w.add(new Waypoint(x, y, tangent, scale));
      double heading = rng.nextDouble() * 2 * Math.PI;
      double step = 0.3 + rng.nextDouble() * 2.5;
      x += step * Math.cos(heading);
      y += step * Math.sin(heading);
    }
    return w;
  }

  @Test
  void passesThroughEveryWaypointWithItsTangent() {
    List<Waypoint> w = FixturePaths.autoRoute().waypoints();
    QuinticSpline spline = QuinticSpline.through(w);
    double[] d = new double[8];
    for (int k = 0; k < spline.segmentCount(); k++) {
      spline.evaluate(k, 0.0, d);
      assertEquals(w.get(k).xMeters(), d[0], 1e-12);
      assertEquals(w.get(k).yMeters(), d[1], 1e-12);
      assertEquals(spline.knotTangentRadians(k), Math.atan2(d[3], d[2]), 1e-12);
      spline.evaluate(k, 1.0, d);
      assertEquals(w.get(k + 1).xMeters(), d[0], 1e-12);
      assertEquals(w.get(k + 1).yMeters(), d[1], 1e-12);
      assertEquals(spline.knotTangentRadians(k + 1), Math.atan2(d[3], d[2]), 1e-12);
    }
    assertEquals(0.0, spline.knotTangentRadians(0), 1e-15);
    assertEquals(Math.PI, Math.abs(spline.knotTangentRadians(9)), 1e-15);
  }

  @Test
  void curvatureIsContinuousAcrossKnots() {
    Random rng = new Random(5712);
    double[] d = new double[8];
    for (int trial = 0; trial < 500; trial++) {
      QuinticSpline spline = QuinticSpline.through(randomWaypoints(rng));
      for (int k = 1; k < spline.segmentCount(); k++) {
        spline.evaluate(k - 1, 1.0, d);
        double before = curvature(d);
        double tangentBefore = Math.atan2(d[3], d[2]);
        spline.evaluate(k, 0.0, d);
        double after = curvature(d);
        double tangentAfter = Math.atan2(d[3], d[2]);
        assertEquals(before, after, 1e-6, "κ jump at knot " + k + " of trial " + trial);
        assertEquals(0.0, Angles.wrap(tangentBefore - tangentAfter), 1e-9);
        assertEquals(spline.knotCurvature(k), after, 1e-6);
      }
    }
  }

  @Test
  void pointsOnACircleGiveItsCurvature() {
    double radius = 2.0;
    List<Waypoint> w = new ArrayList<>();
    for (int i = 0; i <= 6; i++) {
      double t = i * Math.PI / 4;
      w.add(Waypoint.at(radius * Math.cos(t), radius * Math.sin(t)));
    }
    PathGeometry g = PathGeometry.build(w);
    for (double s = 0; s <= g.lengthMeters(); s += 0.05) {
      assertEquals(1.0 / radius, g.sample(s).curvature(), 1e-3);
    }
    assertEquals(radius * 6 * Math.PI / 4, g.lengthMeters(), 1e-3);
  }

  @Test
  void collinearPointsMakeAStraightLine() {
    QuinticSpline spline =
        QuinticSpline.through(List.of(Waypoint.at(0, 0), Waypoint.at(1, 1), Waypoint.at(3, 3)));
    double[] d = new double[8];
    for (int k = 0; k < 2; k++) {
      for (double u = 0; u <= 1.0; u += 0.125) {
        spline.evaluate(k, u, d);
        assertEquals(d[0], d[1], 1e-12);
        assertEquals(0.0, curvature(d), 1e-12);
      }
    }
  }

  @Test
  void twoAutomaticWaypointsMakeTheChord() {
    QuinticSpline spline = QuinticSpline.through(List.of(Waypoint.at(1, 2), Waypoint.at(4, 6)));
    assertEquals(Math.atan2(4, 3), spline.knotTangentRadians(0), 1e-15);
    assertEquals(Math.atan2(4, 3), spline.knotTangentRadians(1), 1e-15);
    assertEquals(0.0, spline.knotCurvature(0), 1e-12);
    assertEquals(5.0, spline.chordMeters(0), 1e-15);
  }

  @Test
  void automaticEndMirrorsItsNeighbourAcrossTheChord() {
    // Start tangent fixed at +45°, end automatic: the end tangent is the mirror image, -45°
    QuinticSpline spline =
        QuinticSpline.through(List.of(Waypoint.at(0, 0, Math.PI / 4), Waypoint.at(2, 0)));
    assertEquals(-Math.PI / 4, spline.knotTangentRadians(1), 1e-12);
  }

  @Test
  void interiorTangentIsTheParabolaThroughTheNeighbours() {
    // Equal chords: the Bessel tangent is parallel to the line through the neighbours
    QuinticSpline spline =
        QuinticSpline.through(List.of(Waypoint.at(0, 0), Waypoint.at(1, 1), Waypoint.at(2, 0)));
    assertEquals(0.0, spline.knotTangentRadians(1), 1e-12);
  }

  @Test
  void tangentScaleHoldsTheDirectionLonger() {
    List<Waypoint> base = List.of(Waypoint.at(0, 0, 0), Waypoint.at(2, 2, Math.PI / 2));
    List<Waypoint> leaning =
        List.of(Waypoint.at(0, 0, 0).withTangentScale(1.5), Waypoint.at(2, 2, Math.PI / 2));
    assertTrue(
        heightAt(PathGeometry.build(leaning), 1.0) < heightAt(PathGeometry.build(base), 1.0),
        "a longer start handle keeps the curve nearer the x axis");
  }

  /** y where the curve crosses the vertical line at x (the curve crosses it once). */
  private static double heightAt(PathGeometry g, double x) {
    double lo = 0;
    double hi = g.lengthMeters();
    for (int i = 0; i < 60; i++) {
      double mid = 0.5 * (lo + hi);
      if (g.pointAt(mid).x() < x) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    return g.pointAt(lo).y();
  }

  @Test
  void derivativesMatchFiniteDifferences() {
    QuinticSpline spline = QuinticSpline.through(FixturePaths.tangentScales().waypoints());
    double[] a = new double[8];
    double[] b = new double[8];
    double[] c = new double[8];
    double h = 1e-5;
    for (int k = 0; k < spline.segmentCount(); k++) {
      for (double u = 0.1; u < 1.0; u += 0.2) {
        spline.evaluate(k, u - h, a);
        spline.evaluate(k, u, b);
        spline.evaluate(k, u + h, c);
        for (int j = 0; j < 6; j++) {
          assertEquals(b[j + 2], (c[j] - a[j]) / (2 * h), 1e-5 * (1 + Math.abs(b[j + 2])));
        }
        assertEquals(Math.hypot(b[2], b[3]), spline.speed(k, u), 1e-12);
      }
    }
  }

  @Test
  void aFixedStartCurvatureIsKeptAndTheRestStaysContinuous() {
    List<Waypoint> w = FixturePaths.slalom().waypoints();
    QuinticSpline spline = QuinticSpline.through(w, 0.37);
    double[] d = new double[8];
    spline.evaluate(0, 0.0, d);
    assertEquals(0.37, curvature(d), 1e-9);
    assertEquals(0.37, spline.knotCurvature(0), 0.0);
    for (int k = 1; k < spline.segmentCount(); k++) {
      spline.evaluate(k - 1, 1.0, d);
      double before = curvature(d);
      spline.evaluate(k, 0.0, d);
      assertEquals(before, curvature(d), 1e-6);
    }
    // NaN means free: the same as the plain build
    assertEquals(QuinticSpline.through(w).knotCurvature(2), QuinticSpline.through(w, Double.NaN).knotCurvature(2), 0.0);
  }

  @Test
  void fixedEndCurvaturesAreKept() {
    List<Waypoint> w = FixturePaths.slalom().waypoints();
    int n = w.size() - 1;
    QuinticSpline both = QuinticSpline.through(w, 0.37, -0.52);
    double[] d = new double[8];
    both.evaluate(n - 1, 1.0, d);
    assertEquals(-0.52, curvature(d), 1e-9);
    both.evaluate(0, 0.0, d);
    assertEquals(0.37, curvature(d), 1e-9);
    for (int k = 1; k < n; k++) {
      both.evaluate(k - 1, 1.0, d);
      double before = curvature(d);
      both.evaluate(k, 0.0, d);
      assertEquals(before, curvature(d), 1e-6);
    }
    // two waypoints, both ends fixed
    QuinticSpline piece = QuinticSpline.through(List.of(Waypoint.at(0, 0, 0.0), Waypoint.at(2, 1, 0.5)), 0.1, -0.2);
    assertEquals(0.1, piece.knotCurvature(0), 0.0);
    assertEquals(-0.2, piece.knotCurvature(1), 0.0);
    assertEquals(QuinticSpline.through(w, 0.37).knotCurvature(2), QuinticSpline.through(w, 0.37, Double.NaN).knotCurvature(2), 0.0);
  }

  @Test
  void rejectsRepeatedWaypointsAndTooFewPoints() {
    assertThrows(
        IllegalArgumentException.class,
        () -> QuinticSpline.through(List.of(Waypoint.at(1, 1), Waypoint.at(1, 1))));
    assertThrows(IllegalArgumentException.class, () -> QuinticSpline.through(List.of(Waypoint.at(1, 1))));
  }

  @Test
  void doublingBackStillBuilds() {
    QuinticSpline spline =
        QuinticSpline.through(List.of(Waypoint.at(0, 0), Waypoint.at(2, 0), Waypoint.at(0, 0)));
    // No parabola through a reversal: the knot leaves along the next chord
    assertEquals(Math.PI, Math.abs(spline.knotTangentRadians(1)), 1e-12);
    assertTrue(Double.isFinite(spline.knotCurvature(1)));
  }

  static double curvature(double[] d) {
    double q = d[2] * d[2] + d[3] * d[3];
    return (d[2] * d[5] - d[3] * d[4]) / (q * Math.sqrt(q));
  }
}
