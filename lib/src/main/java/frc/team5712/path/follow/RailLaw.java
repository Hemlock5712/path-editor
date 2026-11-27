package frc.team5712.path.follow;

import frc.team5712.path.core.Angles;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;

/**
 * Holds the robot to a {@link Rail} while the driver drives along it. Two directions, two different jobs:
 *
 * <ul>
 *   <li><b>Across the rail</b> is a position problem: get back onto the line. It is the finish's {@link
 *       StoppingLaw} in one dimension: the smallest of the top speed, the speed that can still stop on the line,
 *       and an ease-in near it.
 *   <li><b>Along the rail</b> is the driver's: their stick velocity, keeping only the part that points along the
 *       line. Optional ends stop the robot as a goal would.
 * </ul>
 *
 * Heading is either locked (the same law, for heading) or left to the driver.
 *
 * <p>Every limit comes from the robot file, so a rail agrees with teleop and the paths:
 *
 * <ul>
 *   <li>Braking onto the rail plans on {@link #ACROSS_SHARE} of the traction limit. Grip is one budget, so
 *       until the robot is on the rail and no longer sliding across it, the driver may only speed up or slow
 *       down along it with what is left: {@code √(traction² − across²)}, less {@link #ALONG_MARGIN}.
 *   <li>The reaction time, ease-in and heading limits are the follower's ({@link FollowerConfig}).
 * </ul>
 *
 * <p>This class only says what velocity to want. It goes through the {@link OutputLimiter} like every other
 * command ({@link RailFollower} does both).
 */
public final class RailLaw {
  /** The share of the traction limit that braking onto the rail plans on. */
  public static final double ACROSS_SHARE = 0.55;
  /** The share of the grip left over that the driver gets along the rail while the robot is still getting on. */
  public static final double ALONG_MARGIN = 0.85;
  /**
   * Near top speed the motors have almost nothing left to steer with, so a robot at full stick overshoots the
   * rail (by about 15 cm on the bench). While off the rail, this share of top speed is kept spare for steering;
   * it fades out over the last {@link #STEER_BLEND_METERS}.
   */
  public static final double STEER_MARGIN = 0.1;
  public static final double STEER_BLEND_METERS = 0.05;
  /** Sliding across the rail slower than this (m/s) counts as "on it". */
  public static final double ACROSS_SETTLED_METERS_PER_SECOND = 0.05;

  private final FollowerConfig cfg;
  private final double acrossBrake;
  private final double alongAccel;

  // What the law called for on the last call to wanted(), for the feed-forward (RailFollower reads these)
  double acrossX;
  double acrossY;
  double alongX;
  double alongY;
  double acrossSlowing; // m/s², along (acrossX, acrossY); signed
  double spinSlowing; // rad/s², signed; NaN when the heading is the driver's

  public RailLaw(FollowerConfig config) {
    this.cfg = config;
    RobotConfig robot = config.robot();
    double traction = robot.tractionMetersPerSecondSquared();
    this.acrossBrake = ACROSS_SHARE * traction;
    this.alongAccel = ALONG_MARGIN * Math.sqrt(traction * traction - acrossBrake * acrossBrake);
  }

  /** How hard the robot brakes onto the rail, m/s². */
  public double acrossBrakeMetersPerSecondSquared() {
    return acrossBrake;
  }

  /** How fast the driver's speed along the rail may change while the robot is still getting onto it, m/s². */
  public double alongAccelMetersPerSecondSquared() {
    return alongAccel;
  }

  /**
   * The velocity the robot should want this loop (field directions).
   *
   * @param pose where the robot is (or the point on it being held to the rail, e.g. its intake)
   * @param commanded last loop's command (field directions), for the reaction-time margin
   * @param driver the driver's stick velocity in field directions (m/s and rad/s)
   * @param maxSpeed top speed, m/s; getting onto the rail has first claim on it
   * @param dt seconds since the last loop
   * @param maxCrossSpeed never pull across the rail faster than this (+∞ for no cap); a game-piece assist uses
   *     it to stay a gentle nudge
   */
  public FieldSpeeds wanted(
      Rail rail, Pose pose, FieldSpeeds commanded, FieldSpeeds driver, double maxSpeed, double dt,
      double maxCrossSpeed) {
    alongX = Math.cos(rail.directionRadians());
    alongY = Math.sin(rail.directionRadians());
    // across = along turned 90° to the left
    acrossX = -alongY;
    acrossY = alongX;

    double offX = pose.xMeters() - rail.xMeters();
    double offY = pose.yMeters() - rail.yMeters();
    double along = offX * alongX + offY * alongY; // how far along the rail the robot is
    double cross = -(offX * acrossX + offY * acrossY); // signed metres back onto the rail

    // Turning first: spinning takes wheel speed (a wheel's speed is the drive speed plus spin × radius, and it
    // can't pass top speed), so driving gets what is left.
    double omega = driver.omegaRadPerSec();
    spinSlowing = Double.NaN;
    if (rail.holdsHeading()) {
      double error = Angles.wrap(rail.headingRadians() - pose.headingRadians());
      omega = StoppingLaw.spin(error, commanded.omegaRadPerSec(), cfg.maxAlphaRadPerSecSquared(),
          cfg.latencySeconds(), cfg.maxOmegaRadPerSec(), cfg.headingEaseInPerSecond(),
          PathFollower.HEADING_DEADBAND_RADIANS);
      spinSlowing = -Math.signum(error) * StoppingLaw.slowing(Math.abs(omega),
          Math.abs(commanded.omegaRadPerSec()), cfg.maxAlphaRadPerSecSquared(), cfg.headingEaseInPerSecond(),
          cfg.maxOmegaRadPerSec());
    }
    double spin = Math.max(Math.abs(omega), Math.abs(commanded.omegaRadPerSec())) * cfg.robot().moduleRadiusMeters();
    double driveSpeed = Math.max(0.0, maxSpeed - spin);

    // Across: a position problem, solved as the finish does, in one dimension
    double crossNow = commanded.vxMetersPerSecond() * acrossX + commanded.vyMetersPerSecond() * acrossY;
    double approaching = Math.max(0.0, Math.copySign(1.0, cross) * crossNow);
    double crossLaw = Math.min(maxCrossSpeed, stopSpeed(Math.abs(cross), approaching, driveSpeed));
    double crossSpeed = Math.copySign(crossLaw, cross);
    acrossSlowing = -Math.signum(cross)
        * StoppingLaw.slowing(crossLaw, approaching, acrossBrake, cfg.easeInPerSecond(),
            Math.min(maxCrossSpeed, driveSpeed));

    // Along: the driver's, but it can never push past an end
    double alongNow = commanded.vxMetersPerSecond() * alongX + commanded.vyMetersPerSecond() * alongY;
    double alongSpeed = driver.vxMetersPerSecond() * alongX + driver.vyMetersPerSecond() * alongY;
    alongSpeed = Math.min(alongSpeed, stopSpeed(rail.endMeters() - along, Math.max(0.0, alongNow), maxSpeed));
    alongSpeed = Math.max(alongSpeed, -stopSpeed(along - rail.startMeters(), Math.max(0.0, -alongNow), maxSpeed));

    // Until the robot is on the rail and stopped across it, leave grip for braking onto it: the driver's speed
    // changes gently. (Releasing at the edge of the blend instead overshot by 4.6 cm on the bench: full reverse
    // stick took all the grip while the robot still slid across at 0.3 m/s.)
    if (Math.abs(cross) > STEER_BLEND_METERS || Math.abs(crossNow) > ACROSS_SETTLED_METERS_PER_SECOND) {
      double step = alongAccel * dt;
      alongSpeed = Math.max(alongNow - step, Math.min(alongNow + step, alongSpeed));
    }

    // Getting onto the rail has first claim on the drive speed; the driver gets what is left. While still off
    // the rail, a margin of top speed is kept for steering too. Spinning's share matters as well: past top
    // wheel speed the limiter's motor rule freezes every change with any speeding up in it, and the robot crept
    // 5 cm past the rail at full stick and full spin. Last, so nothing above can undo it.
    double blend = Math.min(1.0, Math.abs(cross) / STEER_BLEND_METERS);
    double cap = Math.max(0.0, driveSpeed - blend * STEER_MARGIN * maxSpeed);
    double room = Math.sqrt(Math.max(0.0, cap * cap - crossSpeed * crossSpeed));
    alongSpeed = Math.max(-room, Math.min(room, alongSpeed));

    return new FieldSpeeds(alongX * alongSpeed + acrossX * crossSpeed, alongY * alongSpeed + acrossY * crossSpeed,
        omega);
  }

  /** The finish's speed rule along one line: the fastest speed that still stops in {@code distance}. */
  private double stopSpeed(double distance, double speedNow, double maxSpeed) {
    if (distance <= 0.0) {
      return 0.0; // at or past the line (or an end): no more speed this way
    }
    return StoppingLaw.speed(distance, speedNow, acrossBrake, cfg.latencySeconds(), maxSpeed,
        cfg.easeInPerSecond(), 0.0);
  }
}
