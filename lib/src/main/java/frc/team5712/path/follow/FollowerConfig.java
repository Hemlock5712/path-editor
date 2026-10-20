package frc.team5712.path.follow;

import frc.team5712.path.core.RobotConfig;

/**
 * Gains and limits for {@link PathFollower}. Defaults follow DriveToPose and the hybrid prototype in
 * the bench: 50 ms latency allowance, P 5 across the path and on heading, a 6 m/s² stopping law, and
 * a 1 deg / 2 cm finish.
 *
 * @param robot the robot model: the output limiter's motor curve and spin lever, and tail rebuilds
 * @param latencySeconds how far ahead (in plan time) the reference is read, and the brake's
 *     reaction-time margin (DriveToPose's REACTION_TIME)
 * @param windowMeters half-width of the projection window around the last progress
 * @param reprojectMeters cross-track error above which a global projection is also tried (a shove)
 * @param crossTrackKp sideways m/s per metre off the path
 * @param crossTrackKd sideways m/s per m/s of sideways drift
 * @param headingKp rad/s per radian of heading error
 * @param terminalRadiusMeters distance to go at which the DriveToPose law takes over
 * @param brakeMetersPerSecondSquared the stopping law's deceleration, sqrt(2·a·d)
 * @param easeInPerSecond DriveToPose's ease-in line, m/s per metre left
 * @param stopDeadbandMeters ask for zero speed inside this
 * @param maxOmegaRadPerSec spin cap
 * @param maxAlphaRadPerSecSquared spin acceleration cap, also the heading stopping law's
 * @param headingEaseInPerSecond the heading ease-in line, rad/s per radian left
 * @param headingDeadbandRadians ask for zero spin inside this
 * @param limiterFrictionCoefficient the output limiter's grip, μ·g shared by translation and spin.
 *     The plan uses the robot's μ; this is usually a little higher so feedback has room.
 * @param positionToleranceMeters finish: this close
 * @param headingToleranceRadians finish: this well aligned
 * @param settledMetersPerSecond finish: slower than this (measured)
 * @param debounceSeconds finish: all three held this long
 * @param goalMoveMeters a bound goal that moves further than this rebuilds the tail
 * @param goalMoveRadians ... or turns further than this
 */
public record FollowerConfig(
    RobotConfig robot,
    double latencySeconds,
    double windowMeters,
    double reprojectMeters,
    double crossTrackKp,
    double crossTrackKd,
    double headingKp,
    double terminalRadiusMeters,
    double brakeMetersPerSecondSquared,
    double easeInPerSecond,
    double stopDeadbandMeters,
    double maxOmegaRadPerSec,
    double maxAlphaRadPerSecSquared,
    double headingEaseInPerSecond,
    double headingDeadbandRadians,
    double limiterFrictionCoefficient,
    double positionToleranceMeters,
    double headingToleranceRadians,
    double settledMetersPerSecond,
    double debounceSeconds,
    double goalMoveMeters,
    double goalMoveRadians) {

  /** The defaults for a robot. */
  public static FollowerConfig defaults(RobotConfig robot) {
    return builder(robot).build();
  }

  public static Builder builder(RobotConfig robot) {
    return new Builder(robot);
  }

  public Builder toBuilder() {
    Builder b = new Builder(robot);
    b.latencySeconds = latencySeconds;
    b.windowMeters = windowMeters;
    b.reprojectMeters = reprojectMeters;
    b.crossTrackKp = crossTrackKp;
    b.crossTrackKd = crossTrackKd;
    b.headingKp = headingKp;
    b.terminalRadiusMeters = terminalRadiusMeters;
    b.brakeMetersPerSecondSquared = brakeMetersPerSecondSquared;
    b.easeInPerSecond = easeInPerSecond;
    b.stopDeadbandMeters = stopDeadbandMeters;
    b.maxOmegaRadPerSec = maxOmegaRadPerSec;
    b.maxAlphaRadPerSecSquared = maxAlphaRadPerSecSquared;
    b.headingEaseInPerSecond = headingEaseInPerSecond;
    b.headingDeadbandRadians = headingDeadbandRadians;
    b.limiterFrictionCoefficient = limiterFrictionCoefficient;
    b.positionToleranceMeters = positionToleranceMeters;
    b.headingToleranceRadians = headingToleranceRadians;
    b.settledMetersPerSecond = settledMetersPerSecond;
    b.debounceSeconds = debounceSeconds;
    b.goalMoveMeters = goalMoveMeters;
    b.goalMoveRadians = goalMoveRadians;
    return b;
  }

  /** Fluent builder; every field starts at its default. */
  public static final class Builder {
    private final RobotConfig robot;
    private double latencySeconds = 0.05;
    private double windowMeters = 0.5;
    private double reprojectMeters = 0.5;
    private double crossTrackKp = 5.0;
    private double crossTrackKd = 0.0;
    private double headingKp = 5.0;
    private double terminalRadiusMeters = 0.3;
    private double brakeMetersPerSecondSquared = 6.0;
    private double easeInPerSecond = 6.0;
    private double stopDeadbandMeters = 0.003;
    private double maxOmegaRadPerSec = Math.toRadians(540.0);
    private double maxAlphaRadPerSecSquared = Math.toRadians(720.0);
    private double headingEaseInPerSecond = 5.0;
    private double headingDeadbandRadians = Math.toRadians(0.3);
    private double limiterFrictionCoefficient = 1.0;
    private double positionToleranceMeters = 0.02;
    private double headingToleranceRadians = Math.toRadians(1.0);
    private double settledMetersPerSecond = 0.05;
    private double debounceSeconds = 0.1;
    private double goalMoveMeters = 0.02;
    private double goalMoveRadians = Math.toRadians(2.0);

    private Builder(RobotConfig robot) {
      this.robot = robot;
    }

    public Builder latencySeconds(double v) { latencySeconds = v; return this; }
    public Builder windowMeters(double v) { windowMeters = v; return this; }
    public Builder reprojectMeters(double v) { reprojectMeters = v; return this; }
    public Builder crossTrackKp(double v) { crossTrackKp = v; return this; }
    public Builder crossTrackKd(double v) { crossTrackKd = v; return this; }
    public Builder headingKp(double v) { headingKp = v; return this; }
    public Builder terminalRadiusMeters(double v) { terminalRadiusMeters = v; return this; }
    public Builder brakeMetersPerSecondSquared(double v) { brakeMetersPerSecondSquared = v; return this; }
    public Builder easeInPerSecond(double v) { easeInPerSecond = v; return this; }
    public Builder stopDeadbandMeters(double v) { stopDeadbandMeters = v; return this; }
    public Builder maxOmegaRadPerSec(double v) { maxOmegaRadPerSec = v; return this; }
    public Builder maxAlphaRadPerSecSquared(double v) { maxAlphaRadPerSecSquared = v; return this; }
    public Builder headingEaseInPerSecond(double v) { headingEaseInPerSecond = v; return this; }
    public Builder headingDeadbandRadians(double v) { headingDeadbandRadians = v; return this; }
    public Builder limiterFrictionCoefficient(double v) { limiterFrictionCoefficient = v; return this; }
    public Builder positionToleranceMeters(double v) { positionToleranceMeters = v; return this; }
    public Builder headingToleranceRadians(double v) { headingToleranceRadians = v; return this; }
    public Builder settledMetersPerSecond(double v) { settledMetersPerSecond = v; return this; }
    public Builder debounceSeconds(double v) { debounceSeconds = v; return this; }
    public Builder goalMoveMeters(double v) { goalMoveMeters = v; return this; }
    public Builder goalMoveRadians(double v) { goalMoveRadians = v; return this; }

    /**
     * Sets a field by its name (the record component's name), for option maps such as the bench's.
     * Returns false for an unknown name.
     */
    public boolean set(String name, double v) {
      switch (name) {
        case "latencySeconds" -> latencySeconds = v;
        case "windowMeters" -> windowMeters = v;
        case "reprojectMeters" -> reprojectMeters = v;
        case "crossTrackKp" -> crossTrackKp = v;
        case "crossTrackKd" -> crossTrackKd = v;
        case "headingKp" -> headingKp = v;
        case "terminalRadiusMeters" -> terminalRadiusMeters = v;
        case "brakeMetersPerSecondSquared" -> brakeMetersPerSecondSquared = v;
        case "easeInPerSecond" -> easeInPerSecond = v;
        case "stopDeadbandMeters" -> stopDeadbandMeters = v;
        case "maxOmegaRadPerSec" -> maxOmegaRadPerSec = v;
        case "maxAlphaRadPerSecSquared" -> maxAlphaRadPerSecSquared = v;
        case "headingEaseInPerSecond" -> headingEaseInPerSecond = v;
        case "headingDeadbandRadians" -> headingDeadbandRadians = v;
        case "limiterFrictionCoefficient" -> limiterFrictionCoefficient = v;
        case "positionToleranceMeters" -> positionToleranceMeters = v;
        case "headingToleranceRadians" -> headingToleranceRadians = v;
        case "settledMetersPerSecond" -> settledMetersPerSecond = v;
        case "debounceSeconds" -> debounceSeconds = v;
        case "goalMoveMeters" -> goalMoveMeters = v;
        case "goalMoveRadians" -> goalMoveRadians = v;
        default -> {
          return false;
        }
      }
      return true;
    }

    public FollowerConfig build() {
      return new FollowerConfig(robot, latencySeconds, windowMeters, reprojectMeters, crossTrackKp,
          crossTrackKd, headingKp, terminalRadiusMeters, brakeMetersPerSecondSquared, easeInPerSecond,
          stopDeadbandMeters, maxOmegaRadPerSec, maxAlphaRadPerSecSquared, headingEaseInPerSecond,
          headingDeadbandRadians, limiterFrictionCoefficient, positionToleranceMeters,
          headingToleranceRadians, settledMetersPerSecond, debounceSeconds, goalMoveMeters,
          goalMoveRadians);
    }
  }
}
