package frc.team5712.path.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class RobotConfigTest {
  private static final RobotConfig R = RobotConfig.DEFAULT;

  @Test
  void freeSpeedComesFromTheMotorAndGearing() {
    assertEquals(5785.0 / 60 * 2 * Math.PI / 6.75 * 0.0508, R.freeSpeedMetersPerSecond(), 1e-12);
    assertEquals(R.freeSpeedMetersPerSecond(), R.wheelSpeedCapMetersPerSecond(), 0.0);
    assertEquals(4.0, R.withMaxWheelMetersPerSecond(4.0).wheelSpeedCapMetersPerSecond(), 0.0);
  }

  @Test
  void driveAccelIsStatorLimitedAtLowSpeedAndZeroAtFreeSpeed() {
    // 4 motors × (9.362 / 476.1 N·m/A) × 120 A × 6.75 / 0.0508 m / 60 kg
    double stalled = 4 * (9.362 / 476.1) * 120 * 6.75 / 0.0508 / 60;
    assertEquals(stalled, R.driveAccelMetersPerSecondSquared(0.0), 1e-9);
    assertEquals(stalled, R.brakeAccelMetersPerSecondSquared(), 1e-9);
    assertEquals(0.0, R.driveAccelMetersPerSecondSquared(R.freeSpeedMetersPerSecond()), 1e-9);
    assertEquals(0.0, R.driveAccelMetersPerSecondSquared(10.0), 0.0);
    // Between the knee and free speed the push falls in a straight line
    double v1 = 3.8;
    double v2 = 4.2;
    double mid = R.driveAccelMetersPerSecondSquared(4.0);
    assertEquals(
        0.5 * (R.driveAccelMetersPerSecondSquared(v1) + R.driveAccelMetersPerSecondSquared(v2)), mid, 1e-9);
    assertTrue(mid < stalled);
  }

  @Test
  void spinLeverAndGrip() {
    assertEquals(6.0 / (60.0 * Math.sqrt(2 * 0.29 * 0.29)), R.spinLeverMeters(), 1e-12);
    assertEquals(0.7 * 9.81, R.gripMetersPerSecondSquared(), 1e-12);
  }

  @Test
  void motorConstants() {
    Motor m = Motor.KRAKEN_X60_FOC;
    assertEquals(12.0 / 476.1, m.resistanceOhms(), 1e-15);
    assertEquals(9.362 / 476.1, m.torquePerAmp(), 1e-15);
    assertEquals(m.stallCurrentAmps(), m.currentAt(0, 12), 1e-9);
    assertEquals(0.0, m.currentAt(m.freeSpeedRadPerSec(), 12), 1e-9);
  }

  @Test
  void rejectsNonsense() {
    assertThrows(IllegalArgumentException.class, () -> R.withFrictionCoefficient(0));
    assertThrows(IllegalArgumentException.class, () -> R.withMaxWheelMetersPerSecond(-1));
    assertThrows(IllegalArgumentException.class, () -> R.withMaxAngularRadPerSec(0));
  }
}
