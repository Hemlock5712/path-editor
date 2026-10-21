package frc.team5712.path.sim;

import frc.team5712.path.core.Angles;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Pose;
import java.util.ArrayDeque;

/**
 * A swerve drive reduced to its chassis, with no WPILib and no noise: the end-to-end suite's
 * simulated drivetrain (lib/src/e2eTest SimDrive) minus the measurement noise, so a run repeats to
 * the last bit and the editor's TypeScript port ({@code editor/core/src/sim.ts}) can reproduce it.
 *
 * <ul>
 *   <li>A field-relative velocity command takes effect {@code latencySeconds} after it is sent.
 *   <li>The chassis velocity follows it with a first-order lag of time constant {@code lagSeconds}.
 *   <li>Everything is integrated every {@code substepSeconds} (trapezoid on the velocity).
 *   <li>{@link #shove} adds a velocity at once: a hit, which the lag then pulls back to the command.
 *   <li>Optionally the floor's grip caps how fast the translation can change ({@code grip}, m/s²;
 *       +∞ = no cap, as in the end-to-end suite): a robot asked for more slides at that limit.
 * </ul>
 *
 * With no grip cap there is no slip: the follower's limiter keeps the commands inside its budget. Exponentials go
 * through {@link StrictMath} (fdlibm, as V8's are).
 */
public final class SimDrivetrain {
  /** The end-to-end suite's numbers. */
  public static final double DEFAULT_LATENCY_SECONDS = 0.020;
  public static final double DEFAULT_LAG_SECONDS = 0.040;
  public static final double DEFAULT_SUBSTEP_SECONDS = 0.001;

  /**
   * How late the drivetrain responds, all told: latency, the lag's time constant and half a 20 ms
   * loop (a command is held for a loop). The follower's latencySeconds should be this.
   */
  public static double responseDelaySeconds(double latencySeconds, double lagSeconds) {
    return latencySeconds + lagSeconds + 0.5 * 0.020;
  }

  private final double latency;
  private final double lag;
  private final double substep;
  private final double kSubstep;
  private final double grip;
  private final ArrayDeque<double[]> inFlight = new ArrayDeque<>(); // {time it takes effect, vx, vy, omega}
  private double[] target;
  private double t;
  private double x;
  private double y;
  private double heading;
  private double vx;
  private double vy;
  private double omega;

  /** A robot at {@code pose} moving at {@code velocity} (field-relative), commanded to keep doing so. */
  public SimDrivetrain(Pose pose, FieldSpeeds velocity, double latencySeconds, double lagSeconds, double substepSeconds) {
    this(pose, velocity, latencySeconds, lagSeconds, substepSeconds, Double.POSITIVE_INFINITY);
  }

  /** The same with the floor's grip capping the translational acceleration at {@code gripMetersPerSecondSquared}. */
  public SimDrivetrain(Pose pose, FieldSpeeds velocity, double latencySeconds, double lagSeconds, double substepSeconds,
      double gripMetersPerSecondSquared) {
    grip = gripMetersPerSecondSquared;
    latency = latencySeconds;
    lag = lagSeconds;
    substep = substepSeconds;
    kSubstep = 1.0 - StrictMath.exp(-substep / lag);
    x = pose.xMeters();
    y = pose.yMeters();
    heading = pose.headingRadians();
    vx = velocity.vxMetersPerSecond();
    vy = velocity.vyMetersPerSecond();
    omega = velocity.omegaRadPerSec();
    target = new double[] {0.0, vx, vy, omega};
  }

  /** Sends a command now; it takes effect one latency later. */
  public void command(FieldSpeeds v) {
    inFlight.add(new double[] {t + latency, v.vxMetersPerSecond(), v.vyMetersPerSecond(), v.omegaRadPerSec()});
  }

  /** A hit: adds (dvx, dvy) m/s to the chassis velocity now. */
  public void shove(double dvx, double dvy) {
    vx += dvx;
    vy += dvy;
  }

  /** Integrates up to time {@code until}. */
  public void advanceTo(double until) {
    while (t < until - 1e-12) {
      while (!inFlight.isEmpty() && inFlight.peek()[0] <= t + 1e-12) {
        target = inFlight.poll();
      }
      double dt = Math.min(substep, until - t);
      double k = dt == substep ? kSubstep : 1.0 - StrictMath.exp(-dt / lag);
      double vx0 = vx;
      double vy0 = vy;
      double w0 = omega;
      double dvx = (target[1] - vx) * k;
      double dvy = (target[2] - vy) * k;
      if (grip < Double.POSITIVE_INFINITY) {
        double a = Math.sqrt(dvx * dvx + dvy * dvy) / dt;
        if (a > grip) {
          dvx *= grip / a;
          dvy *= grip / a;
        }
      }
      vx += dvx;
      vy += dvy;
      omega += (target[3] - omega) * k;
      x += 0.5 * (vx0 + vx) * dt;
      y += 0.5 * (vy0 + vy) * dt;
      heading += 0.5 * (w0 + omega) * dt;
      t += dt;
    }
  }

  public double time() {
    return t;
  }

  /** The pose, heading wrapped to (-π, π]. */
  public Pose pose() {
    return new Pose(x, y, Angles.wrap(heading));
  }

  /** The field-relative velocity. */
  public FieldSpeeds velocity() {
    return new FieldSpeeds(vx, vy, omega);
  }
}
