package frc.team5712.path.core;

/**
 * A brushless DC motor's straight-line torque-speed curve.
 *
 * @param nominalVolts voltage the curve was measured at
 * @param stallTorqueNewtonMeters torque at zero speed
 * @param stallCurrentAmps current at zero speed
 * @param freeSpeedRadPerSec speed with no load
 */
public record Motor(
    double nominalVolts,
    double stallTorqueNewtonMeters,
    double stallCurrentAmps,
    double freeSpeedRadPerSec) {

  /** Kraken X60 with FOC, from CTRE's dyno data (the numbers AccelerationLimiter uses). */
  public static final Motor KRAKEN_X60_FOC = new Motor(12.0, 9.362, 476.1, 5785.0 * 2.0 * Math.PI / 60.0);

  /** Winding resistance, ohms. */
  public double resistanceOhms() {
    return nominalVolts / stallCurrentAmps;
  }

  /** Torque per amp, N·m/A. */
  public double torquePerAmp() {
    return stallTorqueNewtonMeters / stallCurrentAmps;
  }

  /** Back-EMF constant, rad/s per volt. */
  public double radPerSecPerVolt() {
    return freeSpeedRadPerSec / nominalVolts;
  }

  /** Current drawn at full voltage while turning at {@code motorRadPerSec}. */
  public double currentAt(double motorRadPerSec, double volts) {
    return (volts - motorRadPerSec / radPerSecPerVolt()) / resistanceOhms();
  }
}
