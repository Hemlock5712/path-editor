package frc.team5712.path.slip;

/**
 * Thresholds for {@link SlipDetector} and {@link SlipAwareOdometry}. Defaults were chosen on the
 * bench's tuning seeds only (see bench/RESULTS.md).
 *
 * @param residualMetersPerSecond a module whose velocity is further than this from the best-fit
 *     rigid chassis motion is slipping...
 * @param residualPerSpeed ...plus this share of the chassis speed (encoder and steering error grow
 *     with speed)
 * @param commandMetersPerSecond a module turning this much faster than commanded is spinning free
 *     (a weak signal: the onboard velocity loop usually holds the command even while slipping)
 * @param accelMetersPerSecondSquared wheels and IMU disagreeing on the chassis acceleration by
 *     more than this means all four are slipping together
 * @param filterSeconds time constant of the low-pass on both accelerations
 * @param releaseSeconds how slowly the confidence decays once the evidence goes (it rises at once)
 * @param slippingWeight least-squares weight of a slipping module in the odometry (1 = gripping)
 * @param slipTrustSeconds while slipping, the IMU-driven velocity still drifts toward the wheels'
 *     with this time constant (it bounds the IMU's bias error; gripping, the wheels rule)
 * @param lookbackSeconds detection lags the slip's start (the filters), so a slip spell rewinds this
 *     far and replays the stored IMU samples from the velocity of then
 * @param biasSeconds time constant of the IMU bias estimate, learnt while the wheels grip
 * @param maxImuSeconds longest the IMU carries the pose in one slip spell, once its bias is learnt;
 *     after that the down-weighted wheels do again, because a bias error grows quadratically
 * @param minImuSeconds the same before any bias is learnt; the limit grows from this to the
 *     maximum as {@code 1 − exp(−gripping time / biasSeconds)}
 */
public record SlipConfig(
    double residualMetersPerSecond,
    double residualPerSpeed,
    double commandMetersPerSecond,
    double accelMetersPerSecondSquared,
    double filterSeconds,
    double releaseSeconds,
    double slippingWeight,
    double slipTrustSeconds,
    double lookbackSeconds,
    double biasSeconds,
    double maxImuSeconds,
    double minImuSeconds) {

  public static final SlipConfig DEFAULT = new SlipConfig(0.3, 0.05, 1.0, 2.5, 0.03, 0.15, 0.01, 10.0, 0.06, 1.0, 1.5, 0.1);

  /** A copy with one field changed by name (for option maps); unknown names throw. */
  public SlipConfig with(String name, double v) {
    double[] f = {residualMetersPerSecond, residualPerSpeed, commandMetersPerSecond,
        accelMetersPerSecondSquared, filterSeconds, releaseSeconds, slippingWeight, slipTrustSeconds,
        lookbackSeconds, biasSeconds, maxImuSeconds, minImuSeconds};
    String[] names = {"residualMetersPerSecond", "residualPerSpeed", "commandMetersPerSecond",
        "accelMetersPerSecondSquared", "filterSeconds", "releaseSeconds", "slippingWeight",
        "slipTrustSeconds", "lookbackSeconds", "biasSeconds", "maxImuSeconds", "minImuSeconds"};
    int k = java.util.Arrays.asList(names).indexOf(name);
    if (k < 0) {
      throw new IllegalArgumentException("unknown slip setting " + name);
    }
    f[k] = v;
    return new SlipConfig(f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11]);
  }
}
