package frc.team5712.path.slip;

import frc.team5712.path.core.Angles;
import frc.team5712.path.core.Pose;

/**
 * Swerve odometry that stops trusting wheels that slip. It is optional: use it in place of
 * WPILib's {@code SwerveDriveOdometry} on a robot that reports module velocities and a gyro, and,
 * for the full benefit, an accelerometer.
 *
 * <p>Each update:
 *
 * <ul>
 *   <li>the {@link SlipDetector} looks at the sample;
 *   <li><b>the wheels' step</b> is a weighted least-squares fit of the chassis twist to the modules'
 *       distance deltas, as in WPILib. A module flagged as slipping gets weight
 *       {@code slippingWeight} instead of 1. The heading comes from the gyro, and the step is the
 *       twist's exponential;
 *   <li><b>the velocity</b> (field frame) is predicted with the IMU's bias-corrected acceleration and
 *       corrected toward the wheels' velocity. Gripping, it follows the wheels within one sample.
 *       Slipping, it only drifts toward them with {@code slipTrustSeconds}, so the IMU carries it;
 *   <li><b>the pose</b> moves by the wheels' step while gripping, which is exactly WPILib's odometry
 *       on a clean run. While slipping (detector confidence at least 0.5) it moves by velocity × dt.
 *       Detection lags the slip by the filters' delay, so a slip spell starts by rewinding
 *       {@code lookbackSeconds} and replaying the stored IMU samples from the velocity of then.
 *       After a limit the down-weighted wheels carry the pose again: {@code minImuSeconds} before
 *       the bias is learnt, growing to {@code maxImuSeconds} as it is;
 *   <li><b>the IMU's bias</b> is learnt while gripping, as the low-passed gap between the IMU's
 *       acceleration and the wheels' ({@code biasSeconds}).
 * </ul>
 *
 * Without an accelerometer it is WPILib's odometry with slipping modules down-weighted.
 */
public final class SlipAwareOdometry {
  private static final int HISTORY = 256; // samples kept for the rewind (about 1 s at 250 Hz)

  private final SlipDetector detector;
  private final SlipConfig cfg;
  private final int n;
  private final double[] mx;
  private final double[] my;
  private final double[] prevDistance;
  private double gyroOffset;
  private double x;
  private double y;
  private double heading;
  private double lastT = Double.NaN;
  private double vx; // field-frame velocity estimate
  private double vy;
  private double biasX; // IMU bias estimate, robot frame (it turns with the robot)
  private double biasY;
  private boolean slipping;
  private boolean primed;
  private double spell; // seconds the IMU has carried the pose in this slip spell
  private double learnt; // seconds of gripping spent learning the IMU's bias

  // Ring buffer, per update: the pose and the velocity estimate before it, the bias-corrected IMU
  // acceleration (field frame) and dt
  private final double[] hx = new double[HISTORY];
  private final double[] hy = new double[HISTORY];
  private final double[] hvx = new double[HISTORY];
  private final double[] hvy = new double[HISTORY];
  private final double[] hax = new double[HISTORY];
  private final double[] hay = new double[HISTORY];
  private final double[] hdt = new double[HISTORY];
  private int head;
  private int count;

  public SlipAwareOdometry(double[][] modulePositions, SlipConfig config) {
    this.detector = new SlipDetector(modulePositions, config);
    this.cfg = config;
    this.n = modulePositions.length;
    this.mx = new double[n];
    this.my = new double[n];
    for (int i = 0; i < n; i++) {
      mx[i] = modulePositions[i][0];
      my[i] = modulePositions[i][1];
    }
    this.prevDistance = new double[n];
  }

  /** Starts at {@code pose}; the gyro reads {@code gyroRadians} now; the wheels have gone {@code distances}. */
  public void reset(Pose pose, double gyroRadians, double[] distances) {
    x = pose.xMeters();
    y = pose.yMeters();
    heading = pose.headingRadians();
    gyroOffset = Angles.wrap(pose.headingRadians() - gyroRadians);
    System.arraycopy(distances, 0, prevDistance, 0, n);
    detector.reset();
    lastT = Double.NaN;
    vx = vy = biasX = biasY = 0.0;
    slipping = false;
    primed = false;
    spell = 0.0;
    learnt = 0.0;
    head = count = 0;
  }

  /**
   * One odometry sample. Arrays are per module. The IMU acceleration is in the robot frame (NaN =
   * none). The command is the robot-frame chassis velocity last asked for (NaN = unknown).
   */
  public Pose update(
      double t, double gyroRadians, double[] distances, double[] angles, double[] wheelSpeeds,
      double imuAxRobot, double imuAyRobot, double cmdVx, double cmdVy, double cmdOmega) {
    double newHeading = Angles.wrap(gyroRadians + gyroOffset);
    detector.update(t, newHeading, wheelSpeeds, angles, imuAxRobot, imuAyRobot, cmdVx, cmdVy, cmdOmega);
    double dt = Double.isNaN(lastT) ? 0.0 : t - lastT;
    lastT = t;

    double[] step = wheelStep(distances, angles, Angles.wrap(newHeading - heading));
    System.arraycopy(distances, 0, prevDistance, 0, n);
    if (!(dt > 0.0)) {
      x += step[0];
      y += step[1];
      heading = newHeading;
      return pose();
    }
    double wheelVx = step[0] / dt;
    double wheelVy = step[1] / dt;
    boolean imu = !Double.isNaN(imuAxRobot);
    boolean nowSlipping = detector.isSlipping();
    if (!nowSlipping) {
      spell = 0.0;
    }

    if (!imu) {
      x += step[0];
      y += step[1];
      vx = wheelVx;
      vy = wheelVy;
      heading = newHeading;
      slipping = nowSlipping;
      return pose();
    }

    double c = Math.cos(newHeading);
    double s = Math.sin(newHeading);
    double bx = imuAxRobot - biasX;
    double by = imuAyRobot - biasY;
    double ax = c * bx - s * by;
    double ay = s * bx + c * by;
    if (!primed) {
      vx = wheelVx; // the first velocity is the wheels'
      vy = wheelVy;
      primed = true;
    }
    remember(x, y, vx, vy, ax, ay, dt);

    double limit = cfg.minImuSeconds()
        + (cfg.maxImuSeconds() - cfg.minImuSeconds()) * (1.0 - Math.exp(-learnt / cfg.biasSeconds()));
    boolean imuCarries = nowSlipping && spell < limit;
    if (nowSlipping && !slipping) {
      rewindAndReplay(); // brings the pose and velocity up to now on the IMU
      spell += dt;
    } else if (nowSlipping && !imuCarries) {
      // a long slide: the IMU's bias would now cost more than the wheels' error
      x += step[0];
      y += step[1];
      vx = wheelVx;
      vy = wheelVy;
    } else {
      // predict on the IMU, correct toward the wheels: at once when gripping, slowly when slipping
      vx += ax * dt;
      vy += ay * dt;
      double k = nowSlipping ? dt / (cfg.slipTrustSeconds() + dt) : 1.0;
      vx += k * (wheelVx - vx);
      vy += k * (wheelVy - vy);
      if (nowSlipping) {
        x += vx * dt;
        y += vy * dt;
        spell += dt;
      } else {
        x += step[0];
        y += step[1];
        // gripping and settled: learn the IMU's bias from its gap to the wheels' acceleration
        if (detector.confidence() < 0.25) {
          double kb = dt / (cfg.biasSeconds() + dt);
          learnt += dt;
          double gx = detector.imuAx() - detector.wheelAx(); // field frame
          double gy = detector.imuAy() - detector.wheelAy();
          biasX += kb * (c * gx + s * gy - biasX);
          biasY += kb * (-s * gx + c * gy - biasY);
        }
      }
    }
    slipping = nowSlipping;
    heading = newHeading;
    return pose();
  }

  /**
   * Field-frame displacement from the weighted least-squares fit of the robot-frame twist (dx, dy,
   * dθ) to the module deltas (module i moves (dx − dθ·y_i, dy + dθ·x_i)), then WPILib's pose
   * exponential with the gyro's turn.
   */
  private double[] wheelStep(double[] distances, double[] angles, double dTheta) {
    double sw = 0, sxt = 0, syt = 0, stt = 0, bx = 0, by = 0, bt = 0;
    for (int i = 0; i < n; i++) {
      double w = detector.moduleSlipping(i) ? cfg.slippingWeight() : 1.0;
      double d = distances[i] - prevDistance[i];
      double ex = d * Math.cos(angles[i]);
      double ey = d * Math.sin(angles[i]);
      sw += w;
      sxt += w * -my[i];
      syt += w * mx[i];
      stt += w * (my[i] * my[i] + mx[i] * mx[i]);
      bx += w * ex;
      by += w * ey;
      bt += w * (-my[i] * ex + mx[i] * ey);
    }
    // [[sw, 0, sxt], [0, sw, syt], [sxt, syt, stt]] · [dx, dy, dθ] = [bx, by, bt]
    double den = sw * stt - sxt * sxt - syt * syt;
    double tx;
    double ty;
    if (Math.abs(den) < 1e-12 || sw < 1e-12) {
      tx = sw > 1e-12 ? bx / sw : 0.0;
      ty = sw > 1e-12 ? by / sw : 0.0;
    } else {
      double dTh = (sw * bt - sxt * bx - syt * by) / den;
      tx = (bx - sxt * dTh) / sw;
      ty = (by - syt * dTh) / sw;
    }
    double sinOver;
    double cosOver;
    if (Math.abs(dTheta) < 1e-9) {
      sinOver = 1.0 - dTheta * dTheta / 6.0;
      cosOver = dTheta / 2.0;
    } else {
      sinOver = Math.sin(dTheta) / dTheta;
      cosOver = (1.0 - Math.cos(dTheta)) / dTheta;
    }
    double lx = tx * sinOver - ty * cosOver;
    double ly = tx * cosOver + ty * sinOver;
    double c = Math.cos(heading);
    double s = Math.sin(heading);
    return new double[] {c * lx - s * ly, s * lx + c * ly};
  }

  private void remember(double px, double py, double pvx, double pvy, double ax, double ay, double dt) {
    hx[head] = px;
    hy[head] = py;
    hvx[head] = pvx;
    hvy[head] = pvy;
    hax[head] = ax;
    hay[head] = ay;
    hdt[head] = dt;
    head = (head + 1) % HISTORY;
    count = Math.min(count + 1, HISTORY);
  }

  /**
   * Starts a slip spell: goes back about {@code lookbackSeconds} (the samples stored, including
   * this one), restores the pose and velocity from the start of that window, and integrates the IMU
   * samples since, up to now.
   */
  private void rewindAndReplay() {
    int back = 0;
    double span = 0.0;
    while (back < count && span < cfg.lookbackSeconds()) {
      span += hdt[(head - 1 - back + HISTORY) % HISTORY];
      back++;
    }
    int first = (head - back + HISTORY) % HISTORY;
    x = hx[first];
    y = hy[first];
    vx = hvx[first];
    vy = hvy[first];
    for (int j = back; j >= 1; j--) {
      int idx = (head - j + HISTORY) % HISTORY;
      double dt = hdt[idx];
      vx += hax[idx] * dt;
      vy += hay[idx] * dt;
      x += vx * dt;
      y += vy * dt;
    }
  }

  public Pose pose() {
    return new Pose(x, y, heading);
  }

  /** The field-frame velocity estimate, m/s. */
  public double fieldVx() {
    return vx;
  }

  public double fieldVy() {
    return vy;
  }

  /** The detector this odometry runs (its confidence is a {@link SlipSignal} for a follower). */
  public SlipDetector detector() {
    return detector;
  }

  /** True while the pose is carried by the IMU-driven velocity (a slip spell). */
  public boolean usingImu() {
    return slipping;
  }
}
