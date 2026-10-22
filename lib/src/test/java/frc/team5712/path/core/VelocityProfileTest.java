package frc.team5712.path.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.tools.FixturePaths;
import java.util.List;
import org.junit.jupiter.api.Test;

class VelocityProfileTest {
  private static final RobotConfig ROBOT = RobotConfig.DEFAULT;

  /** The profile's own per-node inputs, rebuilt independently for checking. */
  private record Node(double s, double v, double curvature, double spin, double spinBend) {}

  private static Node[] nodes(Path path) {
    VelocityProfile p = path.profile();
    Node[] out = new Node[p.gridPointCount()];
    for (int i = 0; i < out.length; i++) {
      double s = i == out.length - 1 ? path.lengthMeters() : i * p.gridMeters();
      GeometrySample g = path.geometry().sample(s);
      HeadingSample h = path.heading().sample(g);
      out[i] =
          new Node(
              s,
              p.gridVelocity(i),
              Math.abs(g.curvature()),
              Math.abs(h.radiansPerMeter()),
              Math.abs(h.radiansPerMeterSquared()));
    }
    return out;
  }

  @Test
  void everyGridPointFitsTheGripAndMotorBudget() {
    double grip = ROBOT.tractionMetersPerSecondSquared();
    double c = ROBOT.spinLeverMeters();
    for (var e : FixturePaths.all().entrySet()) {
      Path path = Path.build(e.getValue(), ROBOT);
      Node[] n = nodes(path);
      double ds = path.profile().gridMeters();
      double worstUse = 0;
      for (int i = 0; i < n.length; i++) {
        // the cells on either side of grid point i
        for (int cell = i - 1; cell <= i; cell++) {
          if (cell < 0 || cell + 1 >= n.length) {
            continue;
          }
          double a = (n[cell + 1].v() * n[cell + 1].v() - n[cell].v() * n[cell].v()) / (2 * ds);
          double v = n[i].v();
          double alpha = Math.abs(a) * n[i].spin() + v * v * n[i].spinBend();
          double use = Math.hypot(a, v * v * n[i].curvature()) + c * alpha;
          worstUse = Math.max(worstUse, use / grip);
          assertTrue(use <= grip * (1 + 1e-9), e.getKey() + " point " + i + " uses " + use);
          if (a > 0) {
            double wheel = v * (1 + ROBOT.moduleRadiusMeters() * n[i].spin());
            assertTrue(a + c * alpha <= ROBOT.driveAccelMetersPerSecondSquared(wheel) + 1e-9);
          }
          assertTrue(Math.abs(a) <= e.getValue().maxMetersPerSecondSquared() + 1e-9);
        }
      }
      System.out.printf("%-15s peak grip use %.3f of the traction limit%n", e.getKey(), worstUse);
    }
  }

  @Test
  void betweenGridPointsTheBudgetHoldsToTwoPercent() {
    double grip = ROBOT.tractionMetersPerSecondSquared();
    double c = ROBOT.spinLeverMeters();
    for (var e : FixturePaths.all().entrySet()) {
      Path path = Path.build(e.getValue(), ROBOT);
      for (double s = 0.0005; s < path.lengthMeters(); s += 0.0071) {
        PathSample p = path.sample(s);
        HeadingSample h = path.heading().sample(s);
        double v = p.metersPerSecond();
        double a = p.metersPerSecondSquared();
        double alpha = Math.abs(a * p.headingRadiansPerMeter()) + v * v * Math.abs(h.radiansPerMeterSquared());
        double use = Math.hypot(a, v * v * p.curvature()) + c * alpha;
        // the worst is inside a fast face-point blend, where θ'' swings by 40 rad/m² per metre
        assertTrue(use <= grip * 1.02, e.getKey() + " at s = " + s + " uses " + use);
      }
    }
  }

  @Test
  void spinTakesWheelSpeedAwayFromTranslation() {
    for (PathSpec spec : FixturePaths.all().values()) {
      Path path = Path.build(spec, ROBOT);
      for (Node n : nodes(path)) {
        double wheel = n.v() * (1 + ROBOT.moduleRadiusMeters() * n.spin());
        assertTrue(wheel <= ROBOT.wheelSpeedCapMetersPerSecond() * (1 + 1e-12));
        assertTrue(n.v() <= spec.maxMetersPerSecond() + 1e-12);
      }
    }
  }

  @Test
  void straightLineIsTheTextbookTrapezoid() {
    Path path = Path.build(FixturePaths.straight(), ROBOT);
    // 3 m at 6 m/s² up to 4 m/s: 2 × 0.667 s ramps plus 0.333 m of cruise
    assertEquals(2 * 4.0 / 6.0 + (3.0 - 2 * 16.0 / 12.0) / 4.0, path.totalTimeSeconds(), 1e-3);
    assertEquals(4.0, path.profile().velocityAt(1.5), 1e-12);
    assertEquals(6.0, path.profile().accelerationAt(0.1), 1e-9);
    assertEquals(-6.0, path.profile().accelerationAt(2.9), 1e-9);
  }

  @Test
  void motorsLimitTheTopEnd() {
    // No path caps and a huge grip: the motors' torque falls to nothing at free speed
    PathSpec spec = PathSpec.of(List.of(Waypoint.at(-8, 0, 0), Waypoint.at(8, 0, 0)));
    Path path = Path.build(spec, ROBOT.withFrictionCoefficient(5.0));
    double top = path.profile().velocityAt(8.0);
    assertTrue(top < ROBOT.freeSpeedMetersPerSecond());
    assertTrue(top > 0.9 * ROBOT.freeSpeedMetersPerSecond(), "top " + top);
  }

  @Test
  void startAndEndSpeedsAreHonoured() {
    Path rolling = Path.build(FixturePaths.autoRoute(), ROBOT);
    assertEquals(0.0, rolling.profile().velocityAt(0), 0.0);
    assertEquals(0.5, rolling.profile().velocityAt(rolling.lengthMeters()), 1e-12);
    Path moving = Path.build(FixturePaths.tangentScales(), ROBOT);
    assertEquals(1.2, moving.profile().velocityAt(0), 1e-12);
    assertEquals(0.0, moving.profile().velocityAt(moving.lengthMeters()), 0.0);
  }

  @Test
  void slowZoneIsRespected() {
    PathSpec spec = FixturePaths.slalom();
    Path path = Path.build(spec, ROBOT);
    double a = path.geometry().sAtWaypoint(2.5);
    double b = path.geometry().sAtWaypoint(3.5);
    for (double s = a; s <= b; s += 0.01) {
      assertTrue(path.profile().velocityAt(s) <= 2.0 + 1e-9);
    }
    Path open = Path.build(spec.withConstraintZones(List.of()), ROBOT);
    assertTrue(open.totalTimeSeconds() < path.totalTimeSeconds());
  }

  @Test
  void lessTractionIsSlower() {
    PathSpec spec = FixturePaths.hairpin();
    // the default: equal push, 6.5 m/s²; with the centre of mass on the floor grip sets it (13.2 m/s²)
    double low = Path.build(spec, ROBOT.withCenterOfMassHeightMeters(0)).totalTimeSeconds();
    double normal = Path.build(spec, ROBOT).totalTimeSeconds();
    double tall = Path.build(spec, ROBOT.withCenterOfMassHeightMeters(0.4)).totalTimeSeconds();
    double slippy = Path.build(spec, ROBOT.withFrictionCoefficient(0.4)).totalTimeSeconds();
    assertTrue(low < normal && normal < tall && tall < slippy, low + " " + normal + " " + tall + " " + slippy);
  }

  @Test
  void spinningCostsTime() {
    PathSpec still = FixturePaths.sCurve().withRotationTargets(List.of(RotationTarget.atWaypoint(0, 0)));
    PathSpec spinning =
        still.withRotationTargets(
            List.of(
                RotationTarget.atWaypoint(0, 0),
                RotationTarget.atWaypoint(1, 3.0),
                RotationTarget.atWaypoint(2, 0),
                RotationTarget.atWaypoint(3, 3.0)));
    assertTrue(Path.build(spinning, ROBOT).totalTimeSeconds() > Path.build(still, ROBOT).totalTimeSeconds());
  }

  @Test
  void spinCapLimitsPlannedOmega() {
    PathSpec spec =
        FixturePaths.sCurve()
            .withRotationTargets(List.of(RotationTarget.atWaypoint(0, 0), RotationTarget.atWaypoint(1, 3.0)));
    double cap = 2.0;
    Path path = Path.build(spec, ROBOT.withMaxAngularRadPerSec(cap));
    for (double s = 0; s <= path.lengthMeters(); s += 0.01) {
      PathSample p = path.sample(s);
      assertTrue(Math.abs(p.metersPerSecond() * p.headingRadiansPerMeter()) <= cap * 1.02);
    }
  }

  @Test
  void timeAndDistanceInvertEachOther() {
    Path path = Path.build(FixturePaths.autoRoute(), ROBOT);
    VelocityProfile p = path.profile();
    double last = -1;
    for (double s = 0; s <= path.lengthMeters(); s += 0.013) {
      double t = p.timeAt(s);
      assertTrue(t > last || s == 0);
      last = t;
      assertEquals(s, p.sAtTime(t), 1e-9);
    }
    assertEquals(0.0, p.sAtTime(-1), 0.0);
    assertEquals(path.lengthMeters(), p.sAtTime(1e9), 0.0);
    assertEquals(p.totalTimeSeconds(), p.timeAt(path.lengthMeters()), 1e-12);
  }

  @Test
  void timeIsTheIntegralOfOneOverSpeed() {
    Path path = Path.build(FixturePaths.sCurve(), ROBOT);
    VelocityProfile p = path.profile();
    // Midpoint rule from 0.5 m to 8.5 m, where v is well away from zero
    double t = 0;
    double h = 1e-4;
    for (double s = 0.5 + h / 2; s < 8.5; s += h) {
      t += h / p.velocityAt(s);
    }
    assertEquals(p.timeAt(8.5) - p.timeAt(0.5), t, 1e-5);
  }

  @Test
  void gridSpacingBarelyChangesTheAnswer() {
    for (PathSpec spec : FixturePaths.all().values()) {
      double fine = Path.build(spec, ROBOT, 0.002).totalTimeSeconds();
      double normal = Path.build(spec, ROBOT).totalTimeSeconds();
      double coarse = Path.build(spec, ROBOT, 0.05).totalTimeSeconds();
      // zones cap every cell they touch, so paths with zones converge only linearly
      assertEquals(fine, normal, 0.01 * fine);
      assertEquals(fine, coarse, 0.02 * fine);
    }
  }

  @Test
  void accelerationIsConstantAcrossACell() {
    Path path = Path.build(FixturePaths.hairpin(), ROBOT);
    VelocityProfile p = path.profile();
    double ds = p.gridMeters();
    double s0 = 40 * ds;
    double v0 = p.velocityAt(s0);
    double a = p.accelerationAt(s0 + 0.5 * ds);
    double v = p.velocityAt(s0 + 0.5 * ds);
    assertEquals(v0 * v0 + 2 * a * 0.5 * ds, v * v, 1e-12);
  }
}
