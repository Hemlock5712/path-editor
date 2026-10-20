package frc.team5712.path.slip;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.Pose;
import java.util.Random;
import org.junit.jupiter.api.Test;

/**
 * Slip detection and slip-aware odometry on a kinematic chassis with scripted wheel readings: a
 * rigid robot whose wheels report exactly what they should, plus whatever slip a test injects.
 */
class SlipTest {
  static final double R = 0.29;
  static final double[][] MODULES = {{R, R}, {R, -R}, {-R, R}, {-R, -R}};
  static final double DT = 0.004;

  /** The true chassis, and wheels/IMU readings derived from it. */
  static final class Chassis {
    double x;
    double y;
    double heading;
    double vx; // field frame
    double vy;
    double omega;
    final double[] distance = new double[4];
    final double[] speed = new double[4];
    final double[] angle = new double[4];
    final double[] spinExtra = new double[4]; // m/s a wheel turns faster than the ground (slip)
    double imuAx;
    double imuAy;

    /** Advances dt with field acceleration (ax, ay) and spin rate omega. */
    void step(double ax, double ay, double newOmega) {
      vx += ax * DT;
      vy += ay * DT;
      omega = newOmega;
      x += vx * DT;
      y += vy * DT;
      heading += omega * DT;
      double c = Math.cos(heading);
      double s = Math.sin(heading);
      double rvx = c * vx + s * vy;
      double rvy = -s * vx + c * vy;
      for (int i = 0; i < 4; i++) {
        double mvx = rvx - omega * MODULES[i][1];
        double mvy = rvy + omega * MODULES[i][0];
        double sp = Math.hypot(mvx, mvy);
        if (sp > 1e-9) {
          angle[i] = Math.atan2(mvy, mvx);
        }
        speed[i] = sp + spinExtra[i];
        distance[i] += speed[i] * DT;
      }
      imuAx = c * ax + s * ay; // robot frame, centre-mounted, no noise
      imuAy = -s * ax + c * ay;
    }

    Pose pose() {
      return new Pose(x, y, heading);
    }
  }

  @Test
  void cleanDrivingRaisesNoAlarm() {
    Chassis ch = new Chassis();
    SlipDetector d = new SlipDetector(MODULES, SlipConfig.DEFAULT);
    double worst = 0;
    for (int k = 0; k < 1000; k++) {
      double t = k * DT;
      ch.step(2.0 * Math.cos(t), 1.5 * Math.sin(1.3 * t), 1.5 * Math.sin(0.7 * t));
      d.update(t, ch.heading, ch.speed, ch.angle, ch.imuAx, ch.imuAy, Double.NaN, Double.NaN, Double.NaN);
      if (k > 10) {
        worst = Math.max(worst, d.confidence());
        for (int i = 0; i < 4; i++) {
          assertFalse(d.moduleSlipping(i), "module " + i + " at t = " + t);
        }
      }
    }
    assertTrue(worst < 0.2, "worst confidence " + worst);
  }

  @Test
  void oneSpinningWheelStandsOut() {
    Chassis ch = new Chassis();
    SlipDetector d = new SlipDetector(MODULES, SlipConfig.DEFAULT);
    for (int k = 0; k < 300; k++) {
      ch.spinExtra[2] = k > 150 ? 1.0 : 0.0;
      ch.step(1.0, 0.0, 0.5);
      d.update(k * DT, ch.heading, ch.speed, ch.angle, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    }
    assertTrue(d.moduleSlipping(2));
    assertFalse(d.moduleSlipping(0) || d.moduleSlipping(1) || d.moduleSlipping(3));
    assertTrue(d.confidence() >= 0.5);
    assertEquals(1.0, d.moduleResidual(2), 0.05);
  }

  @Test
  void allFourSlippingIsSeenOnlyWithAnImu() {
    for (boolean withImu : new boolean[] {true, false}) {
      Chassis ch = new Chassis();
      SlipDetector d = new SlipDetector(MODULES, SlipConfig.DEFAULT);
      boolean flagged = false;
      for (int k = 0; k < 400; k++) {
        // the wheels spin up 8 m/s² faster than the robot accelerates (2 m/s²): a launch on ice
        double extra = k > 100 ? (k - 100) * DT * 8.0 : 0.0;
        java.util.Arrays.fill(ch.spinExtra, extra);
        ch.step(2.0, 0.0, 0.0);
        d.update(k * DT, ch.heading, ch.speed, ch.angle, withImu ? ch.imuAx : Double.NaN,
            withImu ? ch.imuAy : Double.NaN, Double.NaN, Double.NaN, Double.NaN);
        flagged |= d.confidence() >= 0.5;
      }
      assertEquals(withImu, flagged, withImu ? "the IMU disagrees with the wheels" : "nothing to compare against");
      if (withImu) {
        for (int i = 0; i < 4; i++) {
          assertTrue(d.moduleSlipping(i));
        }
        assertEquals(2.0, d.achievedAccelMetersPerSecondSquared(), 0.1); // what the floor gives
      }
    }
  }

  @Test
  void aWheelFasterThanItsCommandIsSpinning() {
    Chassis ch = new Chassis();
    SlipDetector d = new SlipDetector(MODULES, SlipConfig.DEFAULT);
    for (int k = 0; k < 50; k++) {
      ch.step(0.0, 0.0, 0.0); // standing still...
      ch.speed[1] = 2.0; // ...but one wheel turning at 2 m/s
      d.update(k * DT, ch.heading, ch.speed, ch.angle, Double.NaN, Double.NaN, 0.0, 0.0, 0.0);
    }
    assertTrue(d.moduleSlipping(1));
  }

  @Test
  void confidenceDecaysAfterTheSlip() {
    Chassis ch = new Chassis();
    SlipDetector d = new SlipDetector(MODULES, SlipConfig.DEFAULT);
    for (int k = 0; k < 400; k++) {
      ch.spinExtra[0] = k < 100 ? 1.0 : 0.0;
      ch.step(0.0, 0.0, 0.0);
      d.update(k * DT, ch.heading, ch.speed, ch.angle, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
      if (k == 99) {
        assertTrue(d.confidence() >= 0.5);
      }
    }
    assertTrue(d.confidence() < 0.1);
    assertFalse(d.isSlipping());
  }

  // ------------------------------------------------------------------------------ odometry

  static SlipAwareOdometry odometry(Chassis ch) {
    SlipAwareOdometry odo = new SlipAwareOdometry(MODULES, SlipConfig.DEFAULT);
    odo.reset(ch.pose(), 0.0, ch.distance);
    return odo;
  }

  static Pose update(SlipAwareOdometry odo, Chassis ch, double t, boolean imu) {
    return odo.update(t, ch.heading, ch.distance, ch.angle, ch.speed, imu ? ch.imuAx : Double.NaN,
        imu ? ch.imuAy : Double.NaN, Double.NaN, Double.NaN, Double.NaN);
  }

  /** WPILib's SwerveDriveOdometry, written out: equal-weight least squares, gyro heading, twist exp. */
  static final class PlainOdometry {
    double x;
    double y;
    double heading;
    final double[] prev = new double[4];

    PlainOdometry(Chassis ch) {
      x = ch.x;
      y = ch.y;
      heading = ch.heading;
      System.arraycopy(ch.distance, 0, prev, 0, 4);
    }

    void update(Chassis ch) {
      double sxt = 0, syt = 0, stt = 0, bx = 0, by = 0, bt = 0;
      for (int i = 0; i < 4; i++) {
        double d = ch.distance[i] - prev[i];
        prev[i] = ch.distance[i];
        double ex = d * Math.cos(ch.angle[i]);
        double ey = d * Math.sin(ch.angle[i]);
        double mx = MODULES[i][0];
        double my = MODULES[i][1];
        sxt -= my;
        syt += mx;
        stt += mx * mx + my * my;
        bx += ex;
        by += ey;
        bt += -my * ex + mx * ey;
      }
      double dTh = (4 * bt - sxt * bx - syt * by) / (4 * stt - sxt * sxt - syt * syt);
      double tx = (bx - sxt * dTh) / 4;
      double ty = (by - syt * dTh) / 4;
      double th = ch.heading - heading;
      double so = Math.abs(th) < 1e-9 ? 1 : Math.sin(th) / th;
      double co = Math.abs(th) < 1e-9 ? 0 : (1 - Math.cos(th)) / th;
      double lx = tx * so - ty * co;
      double ly = tx * co + ty * so;
      x += Math.cos(heading) * lx - Math.sin(heading) * ly;
      y += Math.sin(heading) * lx + Math.cos(heading) * ly;
      heading = ch.heading;
    }
  }

  @Test
  void cleanRunIsExactlyPlainOdometry() {
    Chassis ch = new Chassis();
    SlipAwareOdometry odo = odometry(ch);
    PlainOdometry plain = new PlainOdometry(ch);
    for (int k = 1; k <= 1500; k++) {
      ch.step(1.5 * Math.cos(k * DT), 1.0, 2.0 * Math.sin(k * DT));
      Pose p = update(odo, ch, k * DT, true);
      plain.update(ch);
      assertEquals(plain.x, p.xMeters(), 1e-9);
      assertEquals(plain.y, p.yMeters(), 1e-9);
      assertFalse(odo.usingImu());
    }
    assertEquals(ch.x, plain.x, 0.05); // and both close to the truth
  }

  @Test
  void oneSlippingWheelIsLeftOut() {
    Chassis ch = new Chassis();
    SlipAwareOdometry odo = odometry(ch);
    Pose p = null;
    for (int k = 1; k <= 1000; k++) {
      ch.spinExtra[3] = k > 200 ? 1.5 : 0.0;
      ch.step(k < 500 ? 1.0 : -1.0, 0.0, 0.0);
      p = update(odo, ch, k * DT, false);
    }
    // plain averaging would read 1.5 m/s × 3.2 s / 4 ≈ 1.2 m too far
    assertEquals(ch.x, p.xMeters(), 0.03);
  }

  @Test
  void allFourSlippingFollowsTheImuForAShortSpell() {
    Chassis ch = new Chassis();
    SlipAwareOdometry odo = odometry(ch);
    Random rng = new Random(1);
    Pose p = null;
    // 2 s of clean driving (the bias, here zero, is learnt), then a 0.4 s spin-up on a slick patch
    for (int k = 1; k <= 600; k++) {
      double extra = k > 500 ? (k - 500) * DT * 6.0 : 0.0;
      java.util.Arrays.fill(ch.spinExtra, extra);
      ch.step(k <= 500 ? 0.8 * Math.sin(k * DT * 3) : 1.0, 0.2 * rng.nextGaussian() * 0, 0.0);
      p = update(odo, ch, k * DT, true);
    }
    assertTrue(odo.usingImu());
    double wheelsSay = 0;
    for (int i = 0; i < 100; i++) {
      wheelsSay += (i + 1) * DT * 6.0 * DT; // what the spinning wheels add on top
    }
    assertTrue(wheelsSay > 0.1);
    assertEquals(ch.x, p.xMeters(), 0.02, "the IMU carried it; the wheels would be " + wheelsSay + " m out");
  }

  @Test
  void withoutAnImuItIsPlainOdometry() {
    Chassis ch = new Chassis();
    SlipAwareOdometry odo = odometry(ch);
    Pose p = null;
    for (int k = 1; k <= 500; k++) {
      ch.step(1.0, 0.5, 1.0);
      p = update(odo, ch, k * DT, false);
    }
    assertEquals(ch.x, p.xMeters(), 0.01);
    assertEquals(ch.y, p.yMeters(), 0.01);
    assertEquals(ch.vx, odo.fieldVx(), 0.05);
  }

  @Test
  void configSetsFieldsByName() {
    SlipConfig c = SlipConfig.DEFAULT.with("accelMetersPerSecondSquared", 3.0).with("maxImuSeconds", 2.0);
    assertEquals(3.0, c.accelMetersPerSecondSquared(), 0.0);
    assertEquals(2.0, c.maxImuSeconds(), 0.0);
    assertEquals(SlipConfig.DEFAULT.residualMetersPerSecond(), c.residualMetersPerSecond(), 0.0);
    org.junit.jupiter.api.Assertions.assertThrows(IllegalArgumentException.class, () -> c.with("nope", 1));
  }
}
