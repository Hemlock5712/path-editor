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
    assertEquals(1.5 * 9.81, R.gripMetersPerSecondSquared(), 1e-12);
  }

  @Test
  void tractionIsNinetyPercentOfGripOrTipWhicheverComesFirst() {
    // as Precision-Drive's AccelerationLimiter
    RobotConfig tg = R;
    // modules at ±0.29 m (tip 0.29 m), centre of mass 0.2 m up: tipping first
    assertEquals(0.29, tg.tipDistanceInUseMeters(), 1e-12);
    assertEquals(9.81 * 0.29 / 0.2, tg.tipMetersPerSecondSquared(), 1e-12);
    assertEquals(0.9 * 9.81 * 0.29 / 0.2, tg.tractionMetersPerSecondSquared(), 1e-12);
    assertEquals(RobotConfig.TractionLimit.TIPPING, tg.tractionLimit());
    // Joe's numbers: 0.254 m modules, 0.9 · min(14.7, 12.46) = 11.2 m/s²
    RobotConfig joe = tg.withTipDistanceMeters(0.254);
    assertEquals(0.9 * Math.min(1.5 * 9.81, 9.81 * 0.254 / 0.2), joe.tractionMetersPerSecondSquared(), 1e-12);
    assertEquals(11.21, joe.tractionMetersPerSecondSquared(), 0.01);
    // a low centre of mass: grip first; at 0 no tipping limit at all
    assertEquals(0.9 * 1.5 * 9.81, tg.withCenterOfMassHeightMeters(0.1).tractionMetersPerSecondSquared(), 1e-12);
    assertEquals(Double.POSITIVE_INFINITY, tg.withCenterOfMassHeightMeters(0).tipMetersPerSecondSquared());
    assertEquals(RobotConfig.TractionLimit.GRIP, tg.withCenterOfMassHeightMeters(0).tractionLimit());
  }

  @Test
  void theShareIsTheMargin() {
    assertEquals(0.9, R.tractionShare(), 0.0);
    RobotConfig softer = R.withTractionShare(0.75);
    assertEquals(0.75 * 9.81 * 0.29 / 0.2, softer.tractionMetersPerSecondSquared(), 1e-12);
    assertThrows(IllegalArgumentException.class, () -> R.withTractionShare(0.0));
    assertThrows(IllegalArgumentException.class, () -> R.withTractionShare(1.1));
  }

  @Test
  void theWheelSlipOnsetIsWhereTheUnloadedWheelsLetGo() {
    // every wheel pushing the same, the light ones slip at μg / (1 + μh/x): 7.2 m/s² for the defaults
    assertEquals(1.5 * 9.81 / (1 + 1.5 * 0.2 / 0.29), R.wheelSlipOnsetMetersPerSecondSquared(), 1e-12);
    // no load transfer: grip
    assertEquals(1.5 * 9.81, R.withCenterOfMassHeightMeters(0).wheelSlipOnsetMetersPerSecondSquared(), 1e-12);
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
    assertThrows(IllegalArgumentException.class, () -> R.withCenterOfMassHeightMeters(-0.1));
    assertThrows(IllegalArgumentException.class, () -> R.withTipDistanceMeters(0));
  }
}
