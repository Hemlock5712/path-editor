package frc.team5712.path.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.sim.FollowerSim;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.Test;

/** Paths that double back: found, stopped at, and followed as a stop and a reversal. */
class CuspTest {
  static PathSpec joe() {
    try {
      return PathFile.read(Files.readString(
          java.nio.file.Path.of(System.getProperty("fixtures.dir", "../fixtures"), "cusp", "joe_auto.path.json"),
          StandardCharsets.UTF_8));
    } catch (IOException e) {
      throw new IllegalStateException(e);
    }
  }

  @Test
  void joesPathHasACuspAndThePlanStopsThere() {
    Path p = Path.build(joe(), RobotConfig.DEFAULT);
    double[] cusps = p.cusps();
    assertEquals(1, cusps.length);
    assertEquals(11.663, cusps[0], 0.005);
    assertEquals(0.0, p.profile().velocityAt(cusps[0]), 1e-9);
    // honest curvature at the cusp: effectively infinite
    assertTrue(Math.abs(p.geometry().sample(cusps[0]).curvature()) > 100);
    // either side, the tangent points the other way
    var before = p.sample(cusps[0] - 0.05);
    var after = p.sample(cusps[0] + 0.05);
    assertTrue(before.tangentX() * after.tangentX() + before.tangentY() * after.tangentY() < -0.9);
  }

  @Test
  void ordinaryPathsHaveNone() {
    for (PathSpec spec : frc.team5712.path.tools.FixturePaths.all().values()) {
      assertEquals(0, Path.build(spec, RobotConfig.DEFAULT).cusps().length);
    }
  }

  /**
   * Random paths, some with tangents and scales that make them double back: wherever a dense scan of the
   * spline finds the tangent reversing, a cusp is reported within 2 cm and the plan stops there.
   */
  @Test
  void everyReversalIsFoundAndStoppedAt() {
    Random r = new Random(11);
    int reversals = 0;
    List<String> missed = new ArrayList<>();
    for (int n = 0; n < 400; n++) {
      int count = 2 + r.nextInt(3);
      List<Waypoint> wps = new ArrayList<>();
      for (int i = 0; i < count; i++) {
        double tangent = r.nextDouble() < 0.7 ? (r.nextDouble() * 2 - 1) * Math.PI : Double.NaN;
        wps.add(new Waypoint(r.nextDouble() * 4, r.nextDouble() * 3, tangent, 0.2 + r.nextDouble() * 1.8));
      }
      PathSpec spec = PathSpec.of(wps).withLimits(4.0, 6.0);
      Path p;
      try {
        p = Path.build(spec, RobotConfig.DEFAULT);
      } catch (RuntimeException e) {
        continue; // waypoints on top of each other
      }
      double[] cusps = p.cusps();
      for (double s : reversals(p.geometry())) {
        reversals++;
        double nearest = Double.POSITIVE_INFINITY;
        for (double c : cusps) {
          nearest = Math.min(nearest, Math.abs(c - s));
        }
        if (nearest > 0.02) {
          missed.add("path " + n + ": a reversal at " + s + " m, cusps " + java.util.Arrays.toString(cusps));
          continue;
        }
        double ds = p.profile().gridMeters();
        boolean nearEnd = s < ds || s > p.lengthMeters() - ds;
        double v = p.profile().velocityAt(s);
        if (!nearEnd && v > 0.5) { // within 2 cm of a full stop
          missed.add("path " + n + ": " + v + " m/s at the reversal at " + s + " m");
        }
      }
    }
    System.out.println("cusp property test: " + reversals + " reversals in 400 random paths");
    assertTrue(reversals > 10, "the random paths should double back now and then");
    assertEquals(List.of(), missed);
  }

  /** Where the unit tangent turns by more than 150° within 2 cm of arc, scanning the spline densely in u. */
  static List<Double> reversals(PathGeometry g) {
    List<Double> out = new ArrayList<>();
    double step = 0.002;
    double lastFound = Double.NEGATIVE_INFINITY;
    GeometrySample prev = g.sample(0.0);
    for (double s = step; s <= g.lengthMeters(); s += step) {
      GeometrySample a = g.sample(Math.max(0.0, s - 0.01));
      GeometrySample b = g.sample(Math.min(g.lengthMeters(), s + 0.01));
      if (a.tangentX() * b.tangentX() + a.tangentY() * b.tangentY() < Math.cos(Math.toRadians(150)) && s - lastFound > 0.1) {
        out.add(s);
        lastFound = s;
      }
      prev = a;
    }
    assertTrue(prev != null);
    return out;
  }

  /** Joe's path followed in the noise-free sim: it stops at the cusp, comes back, and finishes on the goal. */
  @Test
  void theFollowerStopsAndReversesWithoutASnap() {
    Path p = Path.build(joe(), RobotConfig.DEFAULT);
    FollowerConfig cfg = FollowerConfig.defaults(RobotConfig.DEFAULT).toBuilder().latencySeconds(0.07).build();
    FollowerSim.Trace tr = FollowerSim.run(p, cfg, FollowerSim.Drive.DEFAULT, FieldFrame.BLUE, null, 0.5, 20);
    assertTrue(tr.finishLoop > 0, "finished");
    double c = p.cusps()[0];
    var tip = p.sample(c);
    var approach = p.sample(c - 0.05);
    double furthest = Double.NEGATIVE_INFINITY;
    double fastestNearTip = 0;
    for (int i = 1; i < tr.t.size(); i++) {
      double x = tr.x.get(i);
      double y = tr.y.get(i);
      // how far past the tip along the approach direction
      furthest = Math.max(furthest, (x - tip.xMeters()) * approach.tangentX() + (y - tip.yMeters()) * approach.tangentY());
      if (Math.hypot(x - tip.xMeters(), y - tip.yMeters()) < 0.03) {
        double v = Math.hypot(tr.x.get(i) - tr.x.get(i - 1), tr.y.get(i) - tr.y.get(i - 1)) / 0.02;
        fastestNearTip = Math.max(fastestNearTip, v);
      }
    }
    var end = p.sample(p.lengthMeters());
    int last = tr.t.size() - 1;
    System.out.printf("Joe's path: past the tip %.3f m, fastest within 3 cm of it %.2f m/s, end error %.3f m%n",
        furthest, fastestNearTip, Math.hypot(tr.x.get(last) - end.xMeters(), tr.y.get(last) - end.yMeters()));
    assertTrue(furthest < 0.03, "went " + furthest + " m past the cusp");
    // stopping at the traction limit, the robot is still doing sqrt(2·a·3 cm) 3 cm out
    double atThreeCm = Math.sqrt(2 * RobotConfig.DEFAULT.tractionMetersPerSecondSquared() * 0.03);
    assertTrue(fastestNearTip < atThreeCm, "moving " + fastestNearTip + " m/s at the cusp");
    assertEquals(end.xMeters(), tr.x.get(last), 0.02);
    assertEquals(end.yMeters(), tr.y.get(last), 0.02);
  }

  /** A deliberate reversal in the middle of a path: out past B, stop, back through C to D. */
  @Test
  void aDeliberateReversalMidPathIsFollowedAsStopAndBack() {
    PathSpec spec = PathSpec.of(List.of(Waypoint.at(0.0, 0.0, 0.0), Waypoint.at(2.0, 0.0, 0.0),
            Waypoint.at(1.2, 0.05, Math.PI), Waypoint.at(0.0, 0.6)))
        .withRotationTargets(List.of(RotationTarget.atWaypoint(0, 0.0), RotationTarget.atWaypoint(3, 0.0)))
        .withLimits(4.0, 6.0);
    Path p = Path.build(spec, RobotConfig.DEFAULT);
    assertEquals(1, p.cusps().length, java.util.Arrays.toString(p.cusps()));
    double c = p.cusps()[0];
    assertTrue(c > 1.0 && c < p.lengthMeters() - 1.0, "mid-path: " + c);
    assertEquals(0.0, p.profile().velocityAt(c), 1e-9);
    FollowerConfig cfg = FollowerConfig.defaults(RobotConfig.DEFAULT).toBuilder().latencySeconds(0.07).build();
    FollowerSim.Trace tr = FollowerSim.run(p, cfg, FollowerSim.Drive.DEFAULT, FieldFrame.BLUE, null, 0.5, 20);
    assertTrue(tr.finishLoop > 0, "finished");
    var tip = p.sample(c);
    var approach = p.sample(c - 0.05);
    double furthest = Double.NEGATIVE_INFINITY;
    for (int i = 0; i < tr.t.size(); i++) {
      furthest = Math.max(furthest, (tr.x.get(i) - tip.xMeters()) * approach.tangentX()
          + (tr.y.get(i) - tip.yMeters()) * approach.tangentY());
    }
    var end = p.sample(p.lengthMeters());
    int last = tr.t.size() - 1;
    System.out.printf("mid-path reversal at %.3f m: past the tip %.3f m, end error %.3f m, %.2f s (plan %.2f s)%n", c,
        furthest, Math.hypot(tr.x.get(last) - end.xMeters(), tr.y.get(last) - end.yMeters()),
        tr.finishLoop * 0.02, p.totalTimeSeconds());
    assertTrue(furthest < 0.03, "went " + furthest + " m past the cusp");
    assertEquals(end.xMeters(), tr.x.get(last), 0.02);
    assertEquals(end.yMeters(), tr.y.get(last), 0.02);
  }
}
