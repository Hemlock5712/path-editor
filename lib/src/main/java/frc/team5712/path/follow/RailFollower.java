package frc.team5712.path.follow;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Pose;

/**
 * Drives a {@link Rail}: {@link RailLaw} says what velocity to want, the {@link OutputLimiter} decides how fast
 * the robot may get there (the limiter teleop and the paths use), and the feed-forward is worked out as it is
 * for a path.
 *
 * <pre>
 * RailFollower rails = new RailFollower(FollowerConfig.defaults(robot));   // robot init
 * rails.reset(measuredVelocity);                                            // when the command starts
 * FieldSpeeds command = rails.step(rail, pose, fromTheSticks, maxSpeed, dt); // every loop
 * </pre>
 *
 * The rail is given every loop, so it can move: a game-piece assist draws a new one through the piece each loop
 * ({@link PieceAssist}).
 */
public final class RailFollower {
  private final FollowerConfig cfg;
  private final RailLaw law;
  private final OutputLimiter limiter;
  private FieldSpeeds command = new FieldSpeeds(0.0, 0.0, 0.0);
  private FieldSpeeds wanted = new FieldSpeeds(0.0, 0.0, 0.0);
  private double planAx;
  private double planAy;
  private double planAlpha;

  public RailFollower(FollowerConfig config) {
    this.cfg = config;
    this.law = new RailLaw(config);
    this.limiter = new OutputLimiter(config.robot());
  }

  public RailLaw law() {
    return law;
  }

  /** Starts from {@code velocity}: the measured one, so a robot already moving isn't asked to stop in one loop. */
  public void reset(FieldSpeeds velocity) {
    limiter.reset(velocity);
    command = velocity;
    wanted = velocity;
    planAx = 0.0;
    planAy = 0.0;
    planAlpha = 0.0;
  }

  /** One loop on {@code rail}: the velocity to command. */
  public FieldSpeeds step(Rail rail, Pose pose, FieldSpeeds driver, double maxSpeed, double dt) {
    return step(rail, pose, driver, maxSpeed, dt, Double.POSITIVE_INFINITY);
  }

  /** The same, never pulling across the rail faster than {@code maxCrossSpeed}. */
  public FieldSpeeds step(
      Rail rail, Pose pose, FieldSpeeds driver, double maxSpeed, double dt, double maxCrossSpeed) {
    wanted = law.wanted(rail, pose, command, driver, maxSpeed, dt, maxCrossSpeed);
    command = limiter.limit(wanted, dt);
    // The feed-forward, as for a path: what the plan calls for, not the difference between commands. Across
    // the rail the command follows the measured pose, so only the law's own braking counts there. Along it the
    // command is the driver's, and the limiter's step along the rail is exactly what they asked for.
    FieldSpeeds stepTaken = limiter.acceleration();
    double alongStep = stepTaken.vxMetersPerSecond() * law.alongX + stepTaken.vyMetersPerSecond() * law.alongY;
    planAx = alongStep * law.alongX + law.acrossSlowing * law.acrossX;
    planAy = alongStep * law.alongY + law.acrossSlowing * law.acrossY;
    planAlpha = Double.isNaN(law.spinSlowing) ? stepTaken.omegaRadPerSec() : law.spinSlowing;
    return command;
  }

  /** One loop with no rail: the driver's velocity through the limiter, as plain teleop. */
  public FieldSpeeds step(FieldSpeeds driver, double dt) {
    wanted = driver;
    command = limiter.limit(driver, dt);
    FieldSpeeds stepTaken = limiter.acceleration();
    planAx = stepTaken.vxMetersPerSecond();
    planAy = stepTaken.vyMetersPerSecond();
    planAlpha = stepTaken.omegaRadPerSec();
    return command;
  }

  /** What the law wanted on the last step, before the limiter. */
  public FieldSpeeds wanted() {
    return wanted;
  }

  /** The last command. */
  public FieldSpeeds command() {
    return command;
  }

  /**
   * The acceleration the last step calls for (m/s² and rad/s², field directions), for {@link WheelForces}: the
   * rail's braking across it and the driver's change along it, held to the traction limit.
   */
  public FieldSpeeds commandedAcceleration() {
    double alpha = Math.max(-cfg.maxAlphaRadPerSecSquared(), Math.min(cfg.maxAlphaRadPerSecSquared(), planAlpha));
    double total = Math.hypot(planAx, planAy) + cfg.robot().spinLeverMeters() * Math.abs(alpha);
    double budget = cfg.robot().tractionMetersPerSecondSquared();
    double k = total > budget ? budget / total : 1.0;
    return new FieldSpeeds(planAx * k, planAy * k, alpha * k);
  }
}
