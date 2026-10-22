package frc.team5712.path.follow;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.RobotConfig;
import java.util.Random;
import org.junit.jupiter.api.Test;

class LawsTest {
  @Test
  void stoppingLawIsTheSmallestOfThree() {
    // far: top speed; mid: sqrt(2 a (d - v tau)); near: ease-in; inside the deadband: zero
    assertEquals(4.0, StoppingLaw.speed(10, 0, 6, 0.05, 4, 6, 0.003), 1e-12);
    assertEquals(Math.sqrt(2 * 6 * (1.0 - 2 * 0.05)), StoppingLaw.speed(1.0, 2.0, 6, 0.05, 4, 6, 0.003), 1e-12);
    assertEquals(6 * 0.05, StoppingLaw.speed(0.05, 0, 6, 0.05, 4, 6, 0.003), 1e-12);
    assertEquals(0.0, StoppingLaw.speed(0.002, 0, 6, 0.05, 4, 6, 0.003), 0.0);
    assertEquals(Math.sqrt(1.0 + 2 * 6 * 0.9), StoppingLaw.brake(1.0, 2.0, 6, 0.05, 1.0), 1e-12);
  }

  @Test
  void spinLawIsSignedAndStops() {
    assertEquals(-StoppingLaw.spin(0.5, 0, 12, 0.05, 9, 5, 0.005), StoppingLaw.spin(-0.5, 0, 12, 0.05, 9, 5, 0.005), 0.0);
    assertEquals(0.0, StoppingLaw.spin(0.001, 0, 12, 0.05, 9, 5, 0.005), 0.0);
    assertEquals(9.0, StoppingLaw.spin(100, 0, 12, 0.05, 9, 5, 0.005), 0.0);
  }

  @Test
  void limiterSharesOneGripBudget() {
    RobotConfig r = RobotConfig.DEFAULT;
    double mu = 1.0;
    OutputLimiter lim = new OutputLimiter(r, mu, Math.toRadians(720));
    Random rng = new Random(4);
    double dt = 0.02;
    for (int i = 0; i < 2000; i++) {
      double vx = rng.nextGaussian() * 2;
      double vy = rng.nextGaussian() * 2;
      double w = rng.nextGaussian() * 3;
      lim.reset(vx, vy, w);
      lim.limit(vx + rng.nextGaussian() * 3, vy + rng.nextGaussian() * 3, w + rng.nextGaussian() * 5, dt);
      double ax = (lim.vx() - vx) / dt;
      double ay = (lim.vy() - vy) / dt;
      double alpha = (lim.omega() - w) / dt;
      assertTrue(Math.hypot(ax, ay) + r.spinLeverMeters() * Math.abs(alpha) <= mu * RobotConfig.GRAVITY + 1e-9);
      assertTrue(Math.abs(alpha) <= Math.toRadians(720) + 1e-9);
    }
  }

  @Test
  void limiterLetsSmallStepsThroughUntouched() {
    OutputLimiter lim = new OutputLimiter(RobotConfig.DEFAULT, 1.0 * RobotConfig.GRAVITY, 12);
    lim.reset(1, 0, 0);
    lim.limit(1.05, 0.02, 0.1, 0.02);
    assertEquals(1.05, lim.vx(), 1e-12);
    assertEquals(0.02, lim.vy(), 1e-12);
    assertEquals(0.1, lim.omega(), 1e-12);
  }

  @Test
  void limiterKeepsTheMotorCurveWhenSpeedingUp() {
    RobotConfig r = RobotConfig.DEFAULT;
    OutputLimiter lim = new OutputLimiter(r, 5.0 * RobotConfig.GRAVITY, 12); // grip out of the way
    double v = 4.3;
    lim.reset(v, 0, 0);
    lim.limit(10, 0, 0, 0.02);
    assertEquals(r.driveAccelMetersPerSecondSquared(v), (lim.vx() - v) / 0.02, 1e-9);
  }

  @Test
  void aLoweredGripLimitsSpeedingUpButNotBraking() {
    RobotConfig r = RobotConfig.DEFAULT;
    OutputLimiter lim = new OutputLimiter(r, 1.0 * RobotConfig.GRAVITY, 12);
    lim.setSpeedingUpBudget(0.3 * RobotConfig.GRAVITY);
    lim.setBrakeBudget(0.85 * RobotConfig.GRAVITY);
    lim.reset(2.0, 0, 0);
    lim.limit(4.0, 0, 0, 0.02); // speeding up: 0.3 g
    assertEquals(0.3 * RobotConfig.GRAVITY, (lim.vx() - 2.0) / 0.02, 1e-9);
    lim.reset(2.0, 0, 0);
    lim.limit(0.0, 0, 0, 0.02); // braking: 0.85 g
    assertEquals(-0.85 * RobotConfig.GRAVITY, (lim.vx() - 2.0) / 0.02, 1e-9);
  }
}
