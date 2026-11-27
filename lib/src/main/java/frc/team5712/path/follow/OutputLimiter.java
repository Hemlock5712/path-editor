package frc.team5712.path.follow;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.RobotConfig;

/**
 * The one limit every output goes through: the biggest step toward the wanted velocity the robot can
 * take this loop without slipping. Same shape as AccelerationLimiter, with the velocity profile's
 * grip model so translation and spin share one budget:
 *
 * <ol>
 *   <li>spin acceleration is capped at {@code maxAlpha};
 *   <li>speeding up, the push along the direction of travel is capped by the motors at the fastest wheel's
 *       speed {@code |v| + r·|ω|}. Only that part: turning the velocity at a steady speed takes the tread's
 *       grip, not motor power, so the part across the travel is left to the grip budget below;
 *   <li>{@code |a| + c·|α| ≤ μ·g} with {@code c = I/(m·r)}: if over, translation and spin scale down
 *       together, so the direction of the step is kept.
 * </ol>
 *
 * The limiter keeps its own last output (the command), as DriveToPose does: it starts from the
 * measured velocity once, then from its own previous answer.
 *
 * <p>The follower runs every output through one of these. A driver's sticks go through the same one, so teleop
 * and paths share one set of limits, the robot file's:
 *
 * <pre>
 * OutputLimiter limiter = new OutputLimiter(paths.robot());      // robot init
 * limiter.reset(measuredVelocity);                               // when the command starts
 * FieldSpeeds command = limiter.limit(fromTheSticks, dt);        // every loop
 * </pre>
 *
 * Any one frame will do (the field's or the driver's), as long as every velocity given to it is in that frame.
 */
public final class OutputLimiter {
  private final RobotConfig robot;
  private final double grip;
  private final double lever;
  private final double maxAlpha;
  private double vx;
  private double vy;
  private double omega;
  // the acceleration of the last step: what the command asks the robot to do, for feed-forward
  private double ax;
  private double ay;
  private double alpha;

  /**
   * A limiter for driving by hand, at the limits the paths are planned to: the robot file's traction limit
   * ({@link RobotConfig#tractionMetersPerSecondSquared}), its motors, and its spin acceleration cap (720°/s² if
   * the file has none).
   */
  public OutputLimiter(RobotConfig robot) {
    this(robot, robot.tractionMetersPerSecondSquared(), FollowerConfig.defaults(robot).maxAlphaRadPerSecSquared());
  }

  OutputLimiter(RobotConfig robot, double budgetMetersPerSecondSquared, double maxAlphaRadPerSecSquared) {
    this.robot = robot;
    this.grip = budgetMetersPerSecondSquared;
    this.lever = robot.spinLeverMeters();
    this.maxAlpha = maxAlphaRadPerSecSquared;
  }

  void reset(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadPerSec) {
    vx = vxMetersPerSecond;
    vy = vyMetersPerSecond;
    omega = omegaRadPerSec;
    ax = 0.0;
    ay = 0.0;
    alpha = 0.0;
  }

  /** Starts from {@code velocity}: the measured one, so a robot already moving isn't asked to stop in one loop. */
  public void reset(FieldSpeeds velocity) {
    reset(velocity.vxMetersPerSecond(), velocity.vyMetersPerSecond(), velocity.omegaRadPerSec());
  }

  /**
   * The velocity to command this loop: as close to {@code wanted} as the robot can get in {@code dtSeconds}
   * from the last one commanded.
   */
  public FieldSpeeds limit(FieldSpeeds wanted, double dtSeconds) {
    limit(wanted.vxMetersPerSecond(), wanted.vyMetersPerSecond(), wanted.omegaRadPerSec(), dtSeconds);
    return new FieldSpeeds(vx, vy, omega);
  }

  /** The acceleration of the last step (m/s² and rad/s²): what {@link WheelForces} turns into each wheel's push. */
  public FieldSpeeds acceleration() {
    return new FieldSpeeds(ax, ay, alpha);
  }

  /** Steps the command toward the wanted velocity over {@code dt} seconds. */
  void limit(double wantVx, double wantVy, double wantOmega, double dt) {
    double ax = (wantVx - vx) / dt;
    double ay = (wantVy - vy) / dt;
    double alpha = (wantOmega - omega) / dt;
    alpha = Math.max(-maxAlpha, Math.min(maxAlpha, alpha));

    boolean speedingUp = ax * vx + ay * vy > 0.0;
    if (speedingUp) { // the motors' push along the travel falls with wheel speed
      double speed = Math.sqrt(vx * vx + vy * vy);
      double wheel = speed + Math.abs(omega) * robot.moduleRadiusMeters();
      double motor = robot.driveAccelMetersPerSecondSquared(wheel);
      double ux = vx / speed;
      double uy = vy / speed;
      double along = ax * ux + ay * uy;
      if (along > motor) {
        ax -= (along - motor) * ux;
        ay -= (along - motor) * uy;
      }
    }

    double total = Math.sqrt(ax * ax + ay * ay) + lever * Math.abs(alpha);
    if (total > grip) {
      double k = grip / total;
      ax *= k;
      ay *= k;
      alpha *= k;
    }
    vx += ax * dt;
    vy += ay * dt;
    omega += alpha * dt;
    this.ax = ax;
    this.ay = ay;
    this.alpha = alpha;
  }

  /** Sets the command outright (zero at a finish). */
  void force(double newVx, double newVy, double newOmega) {
    vx = newVx;
    vy = newVy;
    omega = newOmega;
    ax = 0.0;
    ay = 0.0;
    alpha = 0.0;
  }

  double vx() {
    return vx;
  }

  double vy() {
    return vy;
  }

  double omega() {
    return omega;
  }

  double ax() {
    return ax;
  }

  double ay() {
    return ay;
  }

  double alpha() {
    return alpha;
  }

  double speed() {
    return Math.sqrt(vx * vx + vy * vy);
  }
}
