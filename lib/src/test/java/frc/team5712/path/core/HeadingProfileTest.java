package frc.team5712.path.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.tools.FixturePaths;
import java.util.List;
import org.junit.jupiter.api.Test;

class HeadingProfileTest {
  private static final PathGeometry S_CURVE = PathGeometry.build(FixturePaths.sCurve().waypoints());

  private static HeadingProfile profile(List<RotationTarget> targets, List<FacePointZone> zones) {
    return HeadingProfile.build(S_CURVE, targets, zones);
  }

  @Test
  void withNoTargetsTheRobotFacesTheDirectionOfTravel() {
    HeadingProfile h = profile(List.of(), List.of());
    for (double s = 0; s <= S_CURVE.lengthMeters(); s += 0.25) {
      GeometrySample g = S_CURVE.sample(s);
      HeadingSample hs = h.sample(s);
      assertEquals(g.tangentRadians(), hs.radians(), 1e-12);
      assertEquals(g.curvature(), hs.radiansPerMeter(), 1e-12);
      assertEquals(g.dCurvatureDs(), hs.radiansPerMeterSquared(), 1e-12);
    }
  }

  @Test
  void holdsBeforeTheFirstTargetAndAfterTheLast() {
    HeadingProfile h =
        profile(
            List.of(RotationTarget.atWaypoint(1, 0.3), RotationTarget.atWaypoint(2, 1.3)), List.of());
    assertEquals(0.3, h.sample(0.0).radians(), 1e-15);
    assertEquals(0.0, h.sample(0.0).radiansPerMeter(), 0.0);
    assertEquals(0.3, h.sample(S_CURVE.waypointS(1)).radians(), 1e-15);
    assertEquals(1.3, h.sample(S_CURVE.waypointS(2)).radians(), 1e-12);
    assertEquals(1.3, h.sample(S_CURVE.lengthMeters()).radians(), 1e-15);
  }

  @Test
  void blendIsMinimumJerkBetweenTargets() {
    double a = S_CURVE.waypointS(1);
    double b = S_CURVE.waypointS(2);
    HeadingProfile h =
        profile(List.of(RotationTarget.atMeters(a, 0.0), RotationTarget.atMeters(b, 1.0)), List.of());
    assertEquals(0.5, h.sample(0.5 * (a + b)).radians(), 1e-12); // halfway by distance, halfway round
    double tau = 0.25;
    double expected = 10 * Math.pow(tau, 3) - 15 * Math.pow(tau, 4) + 6 * Math.pow(tau, 5);
    assertEquals(expected, h.sample(a + tau * (b - a)).radians(), 1e-12);
    // slope and bend vanish at both targets, so the spin starts and stops smoothly
    assertEquals(0.0, h.sample(a).radiansPerMeter(), 1e-12);
    assertEquals(0.0, h.sample(b).radiansPerMeterSquared(), 1e-9);
    assertEquals(1.875 / (b - a), h.sample(0.5 * (a + b)).radiansPerMeter(), 1e-9); // peak 30/16
  }

  @Test
  void turnsTheShortWayRound() {
    double from = Math.toRadians(170);
    double to = Math.toRadians(-170);
    HeadingProfile h =
        profile(List.of(RotationTarget.atWaypoint(0, from), RotationTarget.atWaypoint(3, to)), List.of());
    assertEquals(Math.PI, Math.abs(h.sample(0.5 * S_CURVE.lengthMeters()).radians()), 1e-12);
    assertTrue(h.sample(1.0).radiansPerMeter() > 0, "170° to -170° is +20°, not -340°");
  }

  @Test
  void targetsAreSortedByDistance() {
    HeadingProfile a =
        profile(List.of(RotationTarget.atWaypoint(2, 1.0), RotationTarget.atWaypoint(1, 0.0)), List.of());
    HeadingProfile b =
        profile(List.of(RotationTarget.atWaypoint(1, 0.0), RotationTarget.atWaypoint(2, 1.0)), List.of());
    for (double s = 0; s <= S_CURVE.lengthMeters(); s += 0.3) {
      assertEquals(b.sample(s), a.sample(s));
    }
  }

  @Test
  void facePointZoneLooksAtThePointInItsMiddle() {
    Vec2 point = new Vec2(0.0, 3.0);
    HeadingProfile h =
        profile(List.of(RotationTarget.atWaypoint(0, 0.0)), List.of(new FacePointZone(1, 2, point, 0.5)));
    double a = S_CURVE.waypointS(1);
    double b = S_CURVE.waypointS(2);
    for (double s = a + 0.5; s <= b - 0.5; s += 0.1) {
      GeometrySample g = S_CURVE.sample(s);
      double expected = Math.atan2(point.y() - g.yMeters(), point.x() - g.xMeters());
      assertEquals(expected, h.sample(s).radians(), 1e-12);
    }
    // outside the zone the targets rule
    assertEquals(0.0, h.sample(a - 0.01).radians(), 0.0);
    assertEquals(0.0, h.sample(b + 0.01).radians(), 0.0);
  }

  @Test
  void derivativesMatchFiniteDifferencesEverywhere() {
    HeadingProfile[] profiles = {
      profile(List.of(), List.of()),
      profile(
          List.of(RotationTarget.atWaypoint(0, 0.0), RotationTarget.atWaypoint(2.5, 2.0)),
          List.of(new FacePointZone(0.5, 1.8, new Vec2(1.0, 3.0), 0.7))),
      profile(List.of(), List.of(new FacePointZone(1.0, 3.0, new Vec2(-2.0, -3.0), 0.4))),
    };
    double h = 1e-5;
    for (HeadingProfile p : profiles) {
      for (double s = 0.05; s < S_CURVE.lengthMeters() - 0.05; s += 0.0731) {
        HeadingSample a = p.sample(s - h);
        HeadingSample b = p.sample(s);
        HeadingSample c = p.sample(s + h);
        // derivatives are per metre of true arc; the table's s may stretch by parts in 10,000
        assertEquals(
            b.radiansPerMeter(),
            Angles.wrap(c.radians() - a.radians()) / (2 * h),
            1e-5 + 5e-4 * Math.abs(b.radiansPerMeter()));
        assertEquals(
            b.radiansPerMeterSquared(),
            (c.radiansPerMeter() - a.radiansPerMeter()) / (2 * h),
            1e-4 + 5e-4 * Math.abs(b.radiansPerMeterSquared()));
      }
    }
  }

  @Test
  void zoneEdgesAreContinuous() {
    HeadingProfile h =
        profile(
            List.of(RotationTarget.atWaypoint(0, 0.0)),
            List.of(new FacePointZone(1, 2, new Vec2(0.0, 3.0), 0.5)));
    for (double edge : new double[] {S_CURVE.waypointS(1), S_CURVE.waypointS(2)}) {
      HeadingSample before = h.sample(edge - 1e-9);
      HeadingSample after = h.sample(edge + 1e-9);
      assertEquals(before.radians(), after.radians(), 1e-8);
      assertEquals(before.radiansPerMeter(), after.radiansPerMeter(), 1e-6);
      assertEquals(before.radiansPerMeterSquared(), after.radiansPerMeterSquared(), 1e-5);
    }
  }

  @Test
  void blendIsClampedToHalfTheZone() {
    // A 10 m blend on a short zone still reaches the facing direction at the zone's middle
    double a = S_CURVE.waypointS(1);
    double b = S_CURVE.sAtWaypoint(1.5);
    Vec2 point = new Vec2(0.0, 3.0);
    HeadingProfile h =
        profile(List.of(RotationTarget.atWaypoint(0, 0.0)), List.of(new FacePointZone(1, 1.5, point, 10)));
    GeometrySample g = S_CURVE.sample(0.5 * (a + b));
    assertEquals(
        Math.atan2(point.y() - g.yMeters(), point.x() - g.xMeters()), h.sample(0.5 * (a + b)).radians(), 1e-12);
  }

  @Test
  void sampleBySHasTheSameAnswerAsByGeometry() {
    HeadingProfile h = HeadingProfile.build(
        PathGeometry.build(FixturePaths.autoRoute().waypoints()),
        FixturePaths.autoRoute().rotationTargets(),
        FixturePaths.autoRoute().facePointZones());
    PathGeometry g = PathGeometry.build(FixturePaths.autoRoute().waypoints());
    assertEquals(h.sample(g.sample(7.3)), h.sample(7.3));
  }

  private static final PathGeometry LINE = PathGeometry.build(List.of(Waypoint.at(0, 0), Waypoint.at(4, 0)));

  @Test
  void passesATargetStillTurningWhenTheTurnCarriesOn() {
    // 0 -> 90 -> 180 deg over equal metres: through the middle target at the average rate, no stop
    HeadingProfile h = HeadingProfile.build(LINE, List.of(RotationTarget.atMeters(0.0, 0.0),
        RotationTarget.atMeters(2.0, Math.PI / 2), RotationTarget.atMeters(4.0, Math.PI)), List.of());
    assertEquals(Math.PI / 2, h.sample(2.0).radians(), 1e-12);
    assertEquals(Math.PI / 4, h.sample(2.0).radiansPerMeter(), 1e-12);
    assertEquals(0.0, h.sample(2.0).radiansPerMeterSquared(), 1e-12);
    assertEquals(0.0, h.sample(0.0).radiansPerMeter(), 1e-12);
    assertEquals(0.0, h.sample(4.0).radiansPerMeter(), 1e-12);
  }

  @Test
  void stopsAtATargetWhereTheTurnReverses() {
    HeadingProfile h = HeadingProfile.build(LINE, List.of(RotationTarget.atMeters(0.0, 0.0),
        RotationTarget.atMeters(2.0, 1.0), RotationTarget.atMeters(4.0, 0.2)), List.of());
    assertEquals(0.0, h.sample(2.0).radiansPerMeter(), 1e-12);
    assertEquals(1.0, h.sample(2.0).radians(), 1e-12);
  }

  @Test
  void neverOvershootsATarget() {
    // uneven stretches and turns: the rate never changes sign between targets, and is continuous
    double[] at = {0.0, 0.3, 1.9, 2.2, 4.0};
    double[] deg = {0, 80, 100, 260, 270};
    List<RotationTarget> t = new java.util.ArrayList<>();
    for (int i = 0; i < at.length; i++) {
      t.add(RotationTarget.atMeters(at[i], Math.toRadians(deg[i])));
    }
    HeadingProfile h = HeadingProfile.build(LINE, t, List.of());
    for (double s = 0.0; s <= 4.0; s += 0.001) {
      assertTrue(h.sample(s).radiansPerMeter() >= -1e-12, "turns one way only at s = " + s);
    }
    for (double a : at) {
      HeadingSample before = h.sample(a - 1e-9);
      HeadingSample after = h.sample(a + 1e-9);
      assertEquals(before.radiansPerMeter(), after.radiansPerMeter(), 1e-6, "rate continuous at " + a);
      assertEquals(before.radiansPerMeterSquared(), after.radiansPerMeterSquared(), 1e-3, "bend continuous at " + a);
    }
  }
}
