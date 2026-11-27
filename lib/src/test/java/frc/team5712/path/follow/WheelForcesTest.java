package frc.team5712.path.follow;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.RobotConfig;
import org.junit.jupiter.api.Test;

class WheelForcesTest {
  private static final RobotConfig ROBOT = RobotConfig.DEFAULT;
  // front-left, front-right, back-left, back-right: a long robot, so the arms differ in x and y
  private static final double[] X = {0.30, 0.30, -0.30, -0.30};
  private static final double[] Y = {0.22, -0.22, 0.22, -0.22};

  /** Added up, the wheels give exactly mass x acceleration and inertia x spin acceleration, whatever the heading. */
  @Test
  void theWheelsTogetherGiveTheAccelerationAskedFor() {
    WheelForces forces = new WheelForces(ROBOT, X, Y);
    FieldSpeeds acc = new FieldSpeeds(3.0, -2.0, 5.0);
    for (double heading : new double[] {0.0, 0.7, Math.PI / 2, -2.4}) {
      double[][] f = forces.forAcceleration(acc, heading);
      double sumX = 0.0;
      double sumY = 0.0;
      double torque = 0.0;
      for (int i = 0; i < 4; i++) {
        double armX = Math.cos(heading) * X[i] - Math.sin(heading) * Y[i];
        double armY = Math.sin(heading) * X[i] + Math.cos(heading) * Y[i];
        sumX += f[0][i];
        sumY += f[1][i];
        torque += armX * f[1][i] - armY * f[0][i];
      }
      assertEquals(ROBOT.massKg() * 3.0, sumX, 1e-9);
      assertEquals(ROBOT.massKg() * -2.0, sumY, 1e-9);
      assertEquals(ROBOT.momentOfInertiaKgM2() * 5.0, torque, 1e-9);
    }
  }

  /** Driving straight with no spin, every wheel pushes the same way with a quarter of the total. */
  @Test
  void drivingStraightSharesThePushEqually() {
    double[][] f = new WheelForces(ROBOT, X, Y).forAcceleration(new FieldSpeeds(4.0, 0.0, 0.0), 1.1);
    for (int i = 0; i < 4; i++) {
      assertEquals(ROBOT.massKg() * 4.0 / 4, f[0][i], 1e-9);
      assertEquals(0.0, f[1][i], 1e-9);
    }
  }

  /** Spinning on the spot, each wheel pushes at right angles to its arm and the pushes cancel. */
  @Test
  void spinningPushesAcrossEachArm() {
    double[][] f = new WheelForces(ROBOT, X, Y).forAcceleration(new FieldSpeeds(0.0, 0.0, 6.0), 0.0);
    for (int i = 0; i < 4; i++) {
      assertEquals(0.0, f[0][i] * X[i] + f[1][i] * Y[i], 1e-9, "no push along the arm");
    }
    assertEquals(0.0, f[0][0] + f[0][1] + f[0][2] + f[0][3], 1e-9);
    assertEquals(0.0, f[1][0] + f[1][1] + f[1][2] + f[1][3], 1e-9);
    assertEquals(0.0, new WheelForces(ROBOT, X, Y).none()[0][2]);
  }

  @Test
  void theModuleListsMustMatch() {
    assertThrows(IllegalArgumentException.class, () -> new WheelForces(ROBOT, new double[] {0.3}, new double[0]));
    assertThrows(IllegalArgumentException.class, () -> new WheelForces(ROBOT, new double[] {0.0}, new double[] {0.0}));
  }
}
