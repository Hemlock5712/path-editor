package frc.team5712.path.e2e;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Pose;
import java.util.ArrayDeque;
import java.util.Random;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * The simulated drivetrain for the end-to-end suite: a swerve drive reduced to its chassis.
 *
 * <ul>
 *   <li>A field-relative velocity command takes effect {@link #LATENCY_SECONDS} after it is sent
 *       (CAN, the motor controllers' loop).
 *   <li>The chassis velocity then follows it with a first-order lag of time constant
 *       {@link #LAG_SECONDS} (the drive motors' velocity loop and the modules turning).
 *   <li>Everything is integrated every {@link #SUBSTEP_SECONDS}, so the true pose between robot
 *       loops is known to well under a millimetre.
 *   <li>The robot code sees the true pose and velocity plus Gaussian noise ({@link #POSITION_NOISE_METERS},
 *       {@link #HEADING_NOISE_RADIANS}, {@link #VELOCITY_NOISE_METERS_PER_SECOND}), from a seeded
 *       generator, so every run repeats.
 * </ul>
 *
 * There is no wheel slip and no motor saturation: the follower's own limiter keeps the commands inside
 * the grip and motor budget, and the bench (bench/) is where slip and motors are modelled.
 */
final class SimDrive {
  static final double LATENCY_SECONDS = 0.020;
  static final double LAG_SECONDS = 0.040;
  static final double SUBSTEP_SECONDS = 0.001;
  static final double POSITION_NOISE_METERS = 0.002;
  static final double HEADING_NOISE_RADIANS = Math.toRadians(0.1);
  static final double VELOCITY_NOISE_METERS_PER_SECOND = 0.005; // wheel encoders: a few mm/s near a stop

  /**
   * How late the drivetrain responds, all told: the latency, the lag's time constant, and half a robot
   * loop for holding each command for a loop. A team sets the follower's latencySeconds to its own
   * robot's figure; the suite sets it to this.
   */
  static final double RESPONSE_DELAY_SECONDS = LATENCY_SECONDS + LAG_SECONDS + 0.5 * 0.020;

  private final Random noise;
  private final ArrayDeque<double[]> inFlight = new ArrayDeque<>(); // {time it takes effect, vx, vy, omega}
  private double[] target;
  private double t;
  private double x;
  private double y;
  private double heading;
  private double vx;
  private double vy;
  private double omega;

  /** A robot at {@code pose} moving at {@code velocity} (field-relative), already commanded to keep doing so. */
  SimDrive(Pose pose, FieldSpeeds velocity, long seed) {
    noise = new Random(seed);
    x = pose.xMeters();
    y = pose.yMeters();
    heading = pose.headingRadians();
    vx = velocity.vxMetersPerSecond();
    vy = velocity.vyMetersPerSecond();
    omega = velocity.omegaRadPerSec();
    target = new double[] {0.0, vx, vy, omega};
  }

  /** Sends a command now; it takes effect one latency later. */
  void command(ChassisVelocities v) {
    inFlight.add(new double[] {t + LATENCY_SECONDS, v.vx, v.vy, v.omega});
  }

  /** Integrates up to time {@code until}. */
  void advanceTo(double until) {
    double k = 1.0 - Math.exp(-SUBSTEP_SECONDS / LAG_SECONDS);
    while (t < until - 1e-12) {
      while (!inFlight.isEmpty() && inFlight.peek()[0] <= t + 1e-12) {
        target = inFlight.poll();
      }
      double dt = Math.min(SUBSTEP_SECONDS, until - t);
      double kk = dt == SUBSTEP_SECONDS ? k : 1.0 - Math.exp(-dt / LAG_SECONDS);
      // trapezoid on the velocity, so the pose integral is second-order accurate
      double vx0 = vx;
      double vy0 = vy;
      double w0 = omega;
      vx += (target[1] - vx) * kk;
      vy += (target[2] - vy) * kk;
      omega += (target[3] - omega) * kk;
      x += 0.5 * (vx0 + vx) * dt;
      y += 0.5 * (vy0 + vy) * dt;
      heading += 0.5 * (w0 + omega) * dt;
      t += dt;
    }
  }

  /** A push: adds (dvx, dvy) m/s to the chassis now (call it every loop for a sustained shove). */
  void push(double dvx, double dvy) {
    vx += dvx;
    vy += dvy;
  }

  double time() {
    return t;
  }

  /** The true pose (real field frame). */
  Pose truePose() {
    return new Pose(x, y, Math.atan2(Math.sin(heading), Math.cos(heading)));
  }

  double trueSpeed() {
    return Math.hypot(vx, vy);
  }

  /** What the robot code's pose estimate reads. */
  Pose2d measuredPose() {
    return new Pose2d(x + POSITION_NOISE_METERS * noise.nextGaussian(), y + POSITION_NOISE_METERS * noise.nextGaussian(),
        new Rotation2d(heading + HEADING_NOISE_RADIANS * noise.nextGaussian()));
  }

  /** What the robot code's field velocity reads. */
  ChassisVelocities measuredVelocity() {
    return new ChassisVelocities(vx + VELOCITY_NOISE_METERS_PER_SECOND * noise.nextGaussian(),
        vy + VELOCITY_NOISE_METERS_PER_SECOND * noise.nextGaussian(), omega);
  }
}
