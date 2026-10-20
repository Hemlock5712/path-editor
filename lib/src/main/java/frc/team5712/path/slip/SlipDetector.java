package frc.team5712.path.slip;

import frc.team5712.path.core.Angles;

/**
 * Detects wheel slip on a swerve drive from what the robot already measures, with no WPILib.
 * Feed it one sample per odometry update.
 *
 * <p>Three kinds of evidence, each scored 0.5 at its threshold and 1 at twice it:
 *
 * <ol>
 *   <li><b>Kinematic residuals.</b> With the gyro's yaw rate ω known, every module of a rigid chassis
 *       moves at {@code v + ω × r}. The chassis velocity {@code v} is the median over the modules of
 *       {@code u − ω × r} (robust to one or two bad wheels). A module further than
 *       {@code residual + residualPerSpeed·|v|} from the fit is slipping.
 *   <li><b>Command.</b> A wheel turning faster than its commanded module speed by more than
 *       {@code commandMetersPerSecond} is spinning free. This signal is weak, because the onboard
 *       velocity loop usually holds the command while the tread slides.
 *   <li><b>IMU.</b> When all four wheels slip together their residuals agree, but the acceleration
 *       they imply (the fitted velocity's rate of change, in the field frame) no longer matches the
 *       accelerometer. Both are low-passed with {@code filterSeconds}. A gap above
 *       {@code accelMetersPerSecondSquared} flags every module.
 * </ol>
 *
 * The chassis confidence is the strongest score. It rises at once and decays with
 * {@code releaseSeconds}. The achieved acceleration is the IMU's (filtered) when there is one, else
 * the fitted wheels'. No allocation per update.
 */
public final class SlipDetector implements SlipSignal {
  private final double[] mx;
  private final double[] my;
  private final int n;
  private final SlipConfig cfg;

  private final boolean[] slipping;
  private final double[] residual;
  private final double[] qx;
  private final double[] qy;
  private final double[] sortScratch;
  private double lastT = Double.NaN;
  private double lastHeading;
  private double fitVx; // robot frame
  private double fitVy;
  private double fieldVx; // field frame, last sample
  private double fieldVy;
  private double wheelAx; // field frame, filtered
  private double wheelAy;
  private double imuAx; // field frame, filtered
  private double imuAy;
  private boolean hasImu;
  private double confidence;
  private double imuGap;
  private boolean primed; // a velocity to difference against

  /** @param modulePositions robot-frame (x, y) of each module, metres */
  public SlipDetector(double[][] modulePositions, SlipConfig config) {
    n = modulePositions.length;
    mx = new double[n];
    my = new double[n];
    for (int i = 0; i < n; i++) {
      mx[i] = modulePositions[i][0];
      my[i] = modulePositions[i][1];
    }
    cfg = config;
    slipping = new boolean[n];
    residual = new double[n];
    qx = new double[n];
    qy = new double[n];
    sortScratch = new double[n];
  }

  /** Forgets everything (start of a run). */
  public void reset() {
    lastT = Double.NaN;
    primed = false;
    confidence = 0.0;
    wheelAx = wheelAy = imuAx = imuAy = 0.0;
    java.util.Arrays.fill(slipping, false);
    java.util.Arrays.fill(residual, 0.0);
  }

  /**
   * One sample.
   *
   * @param t seconds
   * @param headingRadians the gyro's heading (field frame)
   * @param wheelSpeeds each module's wheel speed, m/s (signed)
   * @param wheelAngles each module's steering angle, robot frame, rad
   * @param imuAxRobot accelerometer x, robot frame, m/s² (NaN = no accelerometer)
   * @param imuAyRobot accelerometer y
   * @param cmdVx commanded chassis velocity, robot frame (NaN = unknown)
   * @param cmdVy commanded chassis velocity y
   * @param cmdOmega commanded spin, rad/s
   */
  public void update(
      double t, double headingRadians, double[] wheelSpeeds, double[] wheelAngles,
      double imuAxRobot, double imuAyRobot, double cmdVx, double cmdVy, double cmdOmega) {
    if (Double.isNaN(lastT)) {
      lastT = t;
      lastHeading = headingRadians;
      return;
    }
    double dt = t - lastT;
    if (!(dt > 1e-6)) {
      return;
    }
    lastT = t;
    double omega = Angles.wrap(headingRadians - lastHeading) / dt;
    lastHeading = headingRadians;

    // Rigid-body fit with the gyro's spin: q_i = u_i − ω × r_i should all be the chassis velocity
    for (int i = 0; i < n; i++) {
      double ux = wheelSpeeds[i] * Math.cos(wheelAngles[i]);
      double uy = wheelSpeeds[i] * Math.sin(wheelAngles[i]);
      qx[i] = ux + omega * my[i];
      qy[i] = uy - omega * mx[i];
    }
    fitVx = median(qx);
    fitVy = median(qy);
    double speed = Math.hypot(fitVx, fitVy);
    double threshold = cfg.residualMetersPerSecond() + cfg.residualPerSpeed() * speed;
    double score = 0.0;
    int kinematicFlags = 0;
    for (int i = 0; i < n; i++) {
      residual[i] = Math.hypot(qx[i] - fitVx, qy[i] - fitVy);
      slipping[i] = residual[i] > threshold;
      if (slipping[i]) {
        kinematicFlags++;
      }
      score = Math.max(score, scoreOf(residual[i], threshold));
      if (!Double.isNaN(cmdVx)) {
        double cx = cmdVx - cmdOmega * my[i];
        double cy = cmdVy + cmdOmega * mx[i];
        double over = Math.abs(wheelSpeeds[i]) - Math.hypot(cx, cy);
        if (over > cfg.commandMetersPerSecond()) {
          slipping[i] = true;
          score = Math.max(score, scoreOf(over, cfg.commandMetersPerSecond()));
        }
      }
    }

    // Accelerations in the field frame: what the fitted wheels imply, and what the IMU feels
    double c = Math.cos(headingRadians);
    double s = Math.sin(headingRadians);
    double vx = c * fitVx - s * fitVy;
    double vy = s * fitVx + c * fitVy;
    double k = dt / (cfg.filterSeconds() + dt);
    if (primed) {
      wheelAx += k * ((vx - fieldVx) / dt - wheelAx);
      wheelAy += k * ((vy - fieldVy) / dt - wheelAy);
    }
    fieldVx = vx;
    fieldVy = vy;
    hasImu = !Double.isNaN(imuAxRobot) && primed;
    primed = true;
    if (hasImu) {
      double ax = c * imuAxRobot - s * imuAyRobot;
      double ay = s * imuAxRobot + c * imuAyRobot;
      imuAx += k * (ax - imuAx);
      imuAy += k * (ay - imuAy);
      imuGap = Math.hypot(wheelAx - imuAx, wheelAy - imuAy);
      double imuScore = scoreOf(imuGap, cfg.accelMetersPerSecondSquared());
      if (imuScore >= 0.5 && kinematicFlags < n) {
        // the wheels agree with each other but not with the floor: all of them are slipping
        java.util.Arrays.fill(slipping, true);
      }
      score = Math.max(score, imuScore);
    } else {
      imuGap = Double.NaN;
    }

    if (score >= confidence) {
      confidence = score;
    } else {
      confidence += (score - confidence) * dt / (cfg.releaseSeconds() + dt);
    }
  }

  /** 0 at zero evidence, 0.5 at the threshold, 1 at twice it. */
  private static double scoreOf(double value, double threshold) {
    double x = value / (2.0 * threshold);
    return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x);
  }

  private double median(double[] v) {
    System.arraycopy(v, 0, sortScratch, 0, n);
    java.util.Arrays.sort(sortScratch, 0, n);
    return (n & 1) == 1 ? sortScratch[n / 2] : 0.5 * (sortScratch[n / 2 - 1] + sortScratch[n / 2]);
  }

  @Override
  public double confidence() {
    return confidence;
  }

  /** True while the chassis confidence is at least one half. */
  public boolean isSlipping() {
    return confidence >= 0.5;
  }

  @Override
  public double achievedAccelMetersPerSecondSquared() {
    return hasImu ? Math.hypot(imuAx, imuAy) : Math.hypot(wheelAx, wheelAy);
  }

  /** Whether module {@code i} is slipping (from the last update). */
  public boolean moduleSlipping(int i) {
    return slipping[i];
  }

  /** Module {@code i}'s distance from the rigid-body fit, m/s. */
  public double moduleResidual(int i) {
    return residual[i];
  }

  /** The fitted chassis velocity, robot frame, m/s. */
  public double chassisVx() {
    return fitVx;
  }

  public double chassisVy() {
    return fitVy;
  }

  /** The wheels' implied acceleration, field frame, filtered, m/s². */
  public double wheelAx() {
    return wheelAx;
  }

  public double wheelAy() {
    return wheelAy;
  }

  /** The IMU's acceleration, field frame, filtered, m/s² (0 without an IMU). */
  public double imuAx() {
    return imuAx;
  }

  public double imuAy() {
    return imuAy;
  }

  /** |wheel-implied − IMU| acceleration, m/s² (NaN without an IMU). */
  public double imuGapMetersPerSecondSquared() {
    return imuGap;
  }

  public int moduleCount() {
    return n;
  }
}
