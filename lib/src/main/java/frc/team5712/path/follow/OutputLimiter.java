package frc.team5712.path.follow;

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
 * measured velocity once, then from its own previous answer. No allocation per call.
 */
final class OutputLimiter {
  private final RobotConfig robot;
  private final double nominalGrip;
  private double grip;
  private double brakeGrip;
  private final double lever;
  private final double maxAlpha;
  private double vx;
  private double vy;
  private double omega;

  OutputLimiter(RobotConfig robot, double budgetMetersPerSecondSquared, double maxAlphaRadPerSecSquared) {
    this.robot = robot;
    this.nominalGrip = budgetMetersPerSecondSquared;
    this.grip = nominalGrip;
    this.brakeGrip = nominalGrip;
    this.lever = robot.spinLeverMeters();
    this.maxAlpha = maxAlphaRadPerSecSquared;
  }

  void reset(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadPerSec) {
    vx = vxMetersPerSecond;
    vy = vyMetersPerSecond;
    omega = omegaRadPerSec;
  }

  /**
   * Lowers (or restores) the grip budget for steps that speed the robot up: μ·g, with μ capped at
   * the configured one. Braking has its own budget ({@link #setBrakeFrictionCoefficient}).
   */
  void setFrictionCoefficient(double mu) {
    grip = Math.min(nominalGrip, mu * RobotConfig.GRAVITY);
  }

  /**
   * The budget for steps that slow the robot down. It is kept above the speeding-up one on purpose.
   * A cap below what the floor still gives stops the robot using that grip exactly when it must stop:
   * on a slick floor a sliding tread still brakes at kinetic friction. The brake law plans with the
   * low μ, so the robot starts braking early instead.
   */
  void setBrakeFrictionCoefficient(double mu) {
    brakeGrip = Math.min(nominalGrip, mu * RobotConfig.GRAVITY);
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
    double budget = speedingUp ? grip : brakeGrip;
    if (total > budget) {
      double k = budget / total;
      ax *= k;
      ay *= k;
      alpha *= k;
    }
    vx += ax * dt;
    vy += ay * dt;
    omega += alpha * dt;
  }

  /** Replaces one part of the command outright (a BYPASS override). */
  void force(double newVx, double newVy, double newOmega) {
    vx = newVx;
    vy = newVy;
    omega = newOmega;
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

  double speed() {
    return Math.sqrt(vx * vx + vy * vy);
  }
}
