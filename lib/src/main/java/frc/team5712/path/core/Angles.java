package frc.team5712.path.core;

/**
 * Angle helpers. Trig goes through {@link StrictMath} (fdlibm), which is what V8 uses too, so the
 * TypeScript port gets the same bits.
 */
public final class Angles {
  private static final double TWO_PI = 2.0 * Math.PI;

  private Angles() {}

  /** Wraps an angle to (-π, π]. Plain floor arithmetic, so both languages round the same way. */
  public static double wrap(double radians) {
    double r = radians - TWO_PI * Math.floor((radians + Math.PI) / TWO_PI);
    return r <= -Math.PI ? r + TWO_PI : r;
  }

  /** The direction of the vector (x, y), in radians. */
  public static double atan2(double y, double x) {
    return StrictMath.atan2(y, x);
  }

  public static double cos(double radians) {
    return StrictMath.cos(radians);
  }

  public static double sin(double radians) {
    return StrictMath.sin(radians);
  }

  /** The minimum-jerk blend 10τ³ − 15τ⁴ + 6τ⁵ on [0, 1]: zero slope and zero bend at both ends. */
  public static double smoothstep5(double tau) {
    return tau * tau * tau * (10.0 + tau * (-15.0 + tau * 6.0));
  }

  /** d/dτ of {@link #smoothstep5}: 30τ²(1 − τ)². */
  public static double smoothstep5Slope(double tau) {
    double k = tau * (1.0 - tau);
    return 30.0 * k * k;
  }

  /** d²/dτ² of {@link #smoothstep5}: 60τ(1 − τ)(1 − 2τ). */
  public static double smoothstep5Bend(double tau) {
    return 60.0 * tau * (1.0 - tau) * (1.0 - 2.0 * tau);
  }
}
