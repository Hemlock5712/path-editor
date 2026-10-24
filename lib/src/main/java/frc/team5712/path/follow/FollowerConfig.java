package frc.team5712.path.follow;

import frc.team5712.path.core.RobotConfig;

/**
 * The settings a team may change for {@link PathFollower}; everything else is a named constant in the
 * follower. The defaults come from DriveToPose and the bench: read the plan 50 ms ahead, P 5 across the path
 * and on heading, brake at the robot's traction limit, and finish within 2 cm and 1°.
 *
 * <p>The spin limits are the robot's: {@link RobotConfig#maxAngularRadPerSec} and {@link
 * RobotConfig#maxAngularRadPerSecSquared} (AccelerationLimiter's ANGULAR_ACCEL), or 540°/s and 720°/s² when
 * the robot file leaves them out.
 *
 * @param robot the robot model: the limiter's motor curve, spin lever, traction limit and spin limits
 * @param latencySeconds how late the drivetrain answers a command (measure it on the robot). The plan is read
 *     this far ahead, in plan time (never less than one 20 ms loop, or a robot at rest would read 0 m/s and
 *     never start), and the stopping law leaves this much reaction time (DriveToPose's REACTION_TIME)
 * @param crossTrackKp sideways m/s per metre off the path
 * @param headingKp rad/s per radian of heading error
 * @param brakeMetersPerSecondSquared the stopping law's deceleration, sqrt(2·a·d); NaN (the default) = the robot's
 *     traction limit, so the robot brakes as hard as the plan does (a fixed 6 made it brake early and arrive late
 *     on paths planned harder than that; lower it if the robot overshoots)
 * @param easeInPerSecond DriveToPose's ease-in line at the finish, m/s per metre left
 * @param headingEaseInPerSecond the same for heading, rad/s per radian left
 * @param positionToleranceMeters finished: this close to the goal ...
 * @param headingToleranceRadians ... and this well turned, and stopped
 */
public record FollowerConfig(
    RobotConfig robot,
    double latencySeconds,
    double crossTrackKp,
    double headingKp,
    double brakeMetersPerSecondSquared,
    double easeInPerSecond,
    double headingEaseInPerSecond,
    double positionToleranceMeters,
    double headingToleranceRadians) {

  /** The defaults for a robot. */
  public static FollowerConfig defaults(RobotConfig robot) {
    return builder(robot).build();
  }

  /** The spin cap: the robot's, or 540°/s (rad/s). */
  public double maxOmegaRadPerSec() {
    double cap = robot.maxAngularRadPerSec();
    return Double.isFinite(cap) ? cap : Math.toRadians(540.0);
  }

  /** The spin acceleration cap, for the limiter and the heading's stopping law: the robot's, or 720°/s² (rad/s²). */
  public double maxAlphaRadPerSecSquared() {
    double cap = robot.maxAngularRadPerSecSquared();
    return Double.isFinite(cap) ? cap : Math.toRadians(720.0);
  }

  /** The stopping law's deceleration in use: the set one, or the robot's traction limit (m/s²). */
  public double brakeDecel() {
    return Double.isNaN(brakeMetersPerSecondSquared) ? robot.tractionMetersPerSecondSquared() : brakeMetersPerSecondSquared;
  }

  public static Builder builder(RobotConfig robot) {
    return new Builder(robot);
  }

  public Builder toBuilder() {
    Builder b = new Builder(robot);
    b.latencySeconds = latencySeconds;
    b.crossTrackKp = crossTrackKp;
    b.headingKp = headingKp;
    b.brakeMetersPerSecondSquared = brakeMetersPerSecondSquared;
    b.easeInPerSecond = easeInPerSecond;
    b.headingEaseInPerSecond = headingEaseInPerSecond;
    b.positionToleranceMeters = positionToleranceMeters;
    b.headingToleranceRadians = headingToleranceRadians;
    return b;
  }

  /** Fluent builder; every field starts at its default. */
  public static final class Builder {
    private final RobotConfig robot;
    private double latencySeconds = 0.05;
    private double crossTrackKp = 5.0;
    private double headingKp = 5.0;
    private double brakeMetersPerSecondSquared = Double.NaN;
    private double easeInPerSecond = 12.0; // DriveToPose ships 6; the bench sweep's pick (tuning seeds)
    private double headingEaseInPerSecond = 5.0;
    private double positionToleranceMeters = 0.02;
    private double headingToleranceRadians = Math.toRadians(1.0);

    private Builder(RobotConfig robot) {
      this.robot = robot;
    }

    public Builder latencySeconds(double v) { latencySeconds = v; return this; }
    public Builder crossTrackKp(double v) { crossTrackKp = v; return this; }
    public Builder headingKp(double v) { headingKp = v; return this; }
    public Builder brakeMetersPerSecondSquared(double v) { brakeMetersPerSecondSquared = v; return this; }
    public Builder easeInPerSecond(double v) { easeInPerSecond = v; return this; }
    public Builder headingEaseInPerSecond(double v) { headingEaseInPerSecond = v; return this; }
    public Builder positionToleranceMeters(double v) { positionToleranceMeters = v; return this; }
    public Builder headingToleranceRadians(double v) { headingToleranceRadians = v; return this; }

    /**
     * Sets a field by its name (the record component's name), for option maps such as the bench's.
     * Returns false for an unknown name.
     */
    public boolean set(String name, double v) {
      switch (name) {
        case "latencySeconds" -> latencySeconds = v;
        case "crossTrackKp" -> crossTrackKp = v;
        case "headingKp" -> headingKp = v;
        case "brakeMetersPerSecondSquared" -> brakeMetersPerSecondSquared = v;
        case "easeInPerSecond" -> easeInPerSecond = v;
        case "headingEaseInPerSecond" -> headingEaseInPerSecond = v;
        case "positionToleranceMeters" -> positionToleranceMeters = v;
        case "headingToleranceRadians" -> headingToleranceRadians = v;
        default -> {
          return false;
        }
      }
      return true;
    }

    public FollowerConfig build() {
      return new FollowerConfig(robot, latencySeconds, crossTrackKp, headingKp, brakeMetersPerSecondSquared,
          easeInPerSecond, headingEaseInPerSecond, positionToleranceMeters, headingToleranceRadians);
    }
  }
}
