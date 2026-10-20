package frc.team5712.path.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

/**
 * Short hops with big turns: the turn, not the distance, sets the time. The heading is tied to s, so
 * the profile slows the translation until the spin stays within its velocity and acceleration caps.
 */
class ShortHopTest {
  static final double OMEGA = Math.toRadians(540);
  static final double ALPHA = Math.toRadians(720);
  static final RobotConfig SPIN_LIMITED =
      RobotConfig.DEFAULT.withMaxAngularRadPerSec(OMEGA).withMaxAngularRadPerSecSquared(ALPHA);

  static PathSpec hop(double meters, double turnDegrees) {
    return PathSpec.of(List.of(Waypoint.at(0.5, 0.5), Waypoint.at(0.5 + meters, 0.5)))
        .withRotationTargets(List.of(RotationTarget.atWaypoint(0, 0.0),
            RotationTarget.atWaypoint(1, Math.toRadians(turnDegrees))))
        .withLimits(4.0, 6.0);
  }

  @ParameterizedTest
  @CsvSource({"0.2, 80", "0.05, 180", "0.0, 90"})
  void plansWithinTheSpinLimits(double meters, double turnDegrees) {
    Path p = Path.build(hop(meters, turnDegrees), SPIN_LIMITED);
    double turn = Math.toRadians(turnDegrees);
    double t = p.totalTimeSeconds();
    // a minimum-jerk turn of Δθ has peak spin acceleration 10/√3·Δθ/T², so T ≥ √(5.77·Δθ / α_max)
    double floor = Math.sqrt(10 / Math.sqrt(3) * turn / ALPHA);
    assertTrue(t >= floor * 0.98, "faster than the spin limits allow: " + t + " < " + floor);
    assertTrue(t < 3.0, "too slow: " + t);
    double peakW = 0;
    double peakA = 0;
    double lastW = 0;
    int n = 4000;
    for (int i = 0; i <= n; i++) {
      double ti = t * i / n;
      PathSample s = p.sampleAtTime(ti);
      double w = s.metersPerSecond() * s.headingRadiansPerMeter();
      peakW = Math.max(peakW, Math.abs(w));
      if (i > 0) {
        peakA = Math.max(peakA, Math.abs(w - lastW) / (t / n));
      }
      lastW = w;
      if (i > 0 && i < n) {
        assertTrue(s.metersPerSecond() > 0, "v hits 0 mid-path at t = " + ti);
      }
    }
    assertTrue(peakW <= OMEGA * 1.02, "spin " + peakW);
    assertTrue(peakA <= ALPHA * 1.03, "spin acceleration " + peakA); // within 2-3 % between grid points
    PathSample end = p.sample(p.lengthMeters());
    assertEquals(turn, end.headingRadians(), 1e-9);
    assertEquals(0.5 + meters, end.xMeters(), 1e-9);
    assertEquals(0.5, end.yMeters(), 1e-9);
    System.out.printf("short hop %.2f m, %.0f deg: %.3f s (spin-limited floor %.3f s), peak %.2f rad/s, "
        + "%.2f rad/s^2%n", meters, turnDegrees, t, floor, peakW, peakA);
  }

  @Test
  void pureRotationMovesAtMostAMillimetre() {
    Path p = Path.build(hop(0.0, 90), SPIN_LIMITED);
    assertEquals(Path.PURE_ROTATION_METERS, p.lengthMeters(), 1e-6);
    PathSample start = p.sample(0);
    assertTrue(Math.hypot(start.xMeters() - 0.5, start.yMeters() - 0.5) <= Path.PURE_ROTATION_METERS + 1e-9);
  }

  @ParameterizedTest
  @CsvSource({"0.2, 80", "0.05, 180", "0.0, 90", "0.215, 80"})
  void noSpinCapsStillPlans(double meters, double turnDegrees) {
    // only grip limits the spin here; this used to stall at the first grid point
    Path p = Path.build(hop(meters, turnDegrees), RobotConfig.DEFAULT);
    assertTrue(p.totalTimeSeconds() > 0 && p.totalTimeSeconds() < 3.0, "time " + p.totalTimeSeconds());
  }

  @Test
  void spinAccelCapIsReadFromTheRobotFile() {
    RobotConfig r = RobotFile.read("{\"maxAngularRadPerSecSquared\": 12.5}");
    assertEquals(12.5, r.maxAngularRadPerSecSquared());
    assertEquals(12.5, (double) RobotFile.toJson(r).get("maxAngularRadPerSecSquared"));
    assertTrue(!RobotFile.toJson(RobotConfig.DEFAULT).containsKey("maxAngularRadPerSecSquared"));
  }
}
