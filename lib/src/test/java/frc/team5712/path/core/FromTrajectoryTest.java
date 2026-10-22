package frc.team5712.path.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class FromTrajectoryTest {
  /**
   * A trapezoid along a quarter circle of radius 2 m, sampled every 20 ms like a Choreo trajectory, turning
   * the robot 90 degrees on the way: 3 m/s^2 up to 2 m/s, cruise, 3 m/s^2 down.
   */
  private static double[][] arc() {
    double r = 2.0;
    double length = Math.PI * r / 2;
    double a = 3.0;
    double vmax = 2.0;
    double tUp = vmax / a;
    double dUp = 0.5 * a * tUp * tUp;
    double tCruise = (length - 2 * dUp) / vmax;
    double total = 2 * tUp + tCruise;
    int n = (int) Math.ceil(total / 0.02) + 1;
    double[][] c = new double[6][n];
    for (int i = 0; i < n; i++) {
      double t = Math.min(i * 0.02, total);
      double s;
      double v;
      if (t < tUp) {
        s = 0.5 * a * t * t;
        v = a * t;
      } else if (t < tUp + tCruise) {
        s = dUp + vmax * (t - tUp);
        v = vmax;
      } else {
        double td = total - t;
        s = length - 0.5 * a * td * td;
        v = a * td;
      }
      double phi = s / r;
      c[0][i] = t;
      c[1][i] = r * Math.sin(phi);
      c[2][i] = r - r * Math.cos(phi);
      c[3][i] = phi; // the heading turns with the arc
      c[4][i] = v * Math.cos(phi);
      c[5][i] = v * Math.sin(phi);
    }
    return c;
  }

  @Test
  void keepsTheTrajectorysPathTimeAndHeading() {
    double[][] c = arc();
    Path p = Path.fromTrajectory(c[0], c[1], c[2], c[3], c[4], c[5], 6.0);
    double total = c[0][c[0].length - 1];
    assertEquals(Math.PI, p.lengthMeters(), 1e-3);
    assertEquals(total, p.totalTimeSeconds(), 2e-3);
    for (double t = 0.1; t < total; t += 0.1) {
      PathSample at = p.sampleAtTime(t);
      double phi = Math.atan2(at.xMeters(), 2.0 - at.yMeters());
      assertEquals(2.0, Math.hypot(at.xMeters(), at.yMeters() - 2.0), 1e-4, "on the arc");
      assertEquals(phi, at.headingRadians(), 1e-3, "heading turns with the arc");
      assertEquals(0.5, at.headingRadiansPerMeter(), 0.02, "spin rate continuous: 1/r per metre");
      assertEquals(0.5, Math.abs(at.curvature()), 0.02);
    }
    assertEquals(0.0, p.sample(0.0).metersPerSecond(), 0.0);
    assertEquals(2.0, p.sample(p.lengthMeters() / 2).metersPerSecond(), 1e-9);
    assertEquals(0.0, p.sample(p.lengthMeters()).metersPerSecond(), 1e-6);
  }

  @Test
  void dropsSamplesOnTheSameSpot() {
    double[][] c = arc();
    int n = c[0].length;
    double[][] d = new double[6][n + 2];
    for (int k = 0; k < 6; k++) {
      // the first sample twice and the last twice (a spin on the spot at each end)
      d[k][0] = c[k][0];
      System.arraycopy(c[k], 0, d[k], 1, n);
      d[k][n + 1] = c[k][n - 1];
    }
    d[0][1] += 0.01;
    d[0][n + 1] += 0.01;
    Path p = Path.fromTrajectory(d[0], d[1], d[2], d[3], d[4], d[5], 6.0);
    assertEquals(Math.PI, p.lengthMeters(), 1e-3);
  }

  @Test
  void aTrajectoryThatDoesntMoveIsRefused() {
    double[] z = {0.0, 0.0};
    double[] t = {0.0, 0.5};
    assertThrows(IllegalArgumentException.class, () -> Path.fromTrajectory(t, z, z, z, z, z, 6.0));
  }
}
