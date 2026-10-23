package frc.team5712.path.follow;

import frc.team5712.path.core.RobotConfig;

/**
 * Gains and limits for {@link PathFollower}. Defaults follow DriveToPose and the hybrid prototype in
 * the bench: 50 ms latency allowance, P 5 across the path and on heading, a 6 m/s² stopping law, and
 * a 1 deg / 2 cm finish. The ease-in line is 12/s (DriveToPose's is 6/s): the bench sweep's interior
 * optimum on its tuning seeds. The limiter's μ stays at 1.0; see lib/README.md.
 *
 * @param robot the robot model: the output limiter's motor curve and spin lever, and tail rebuilds
 * @param latencySeconds how far ahead (in plan time) the reference is read (never less than one 20 ms loop,
 *     or a robot at rest would read 0 m/s and never start), and the brake's reaction-time margin
 *     (DriveToPose's REACTION_TIME)
 * @param windowMeters half-width of the projection window around the last progress
 * @param reprojectMeters cross-track error above which a global projection is also tried (a shove)
 * @param crossTrackKp sideways m/s per metre off the path
 * @param crossTrackKd sideways m/s per m/s of sideways drift
 * @param headingKp rad/s per radian of heading error
 * @param terminalRadiusMeters distance to go at which the DriveToPose law takes over
 * @param terminalTurnRadians ... once the plan also has no more than this left to turn. A short hop
 *     with a big turn (or a turn in place) is then followed along its plan, with translation and
 *     spin sharing the grip as planned, and handed over only for its last degrees, where the plan
 *     has slowed its spin; handed over at 10° the robot was still spinning too fast to stop in the
 *     heading left and overshot by 7° (the end-to-end suite's short hop)
 * @param brakeMetersPerSecondSquared the stopping law's deceleration, sqrt(2·a·d); NaN (the default) = the robot's
 *     traction limit, so the robot brakes as hard as the plan does (a fixed 6 made it brake early and arrive late)
 * @param easeInPerSecond DriveToPose's ease-in line, m/s per metre left
 * @param stopDeadbandMeters ask for zero speed inside this
 * @param maxOmegaRadPerSec spin cap
 * @param maxAlphaRadPerSecSquared spin acceleration cap, also the heading stopping law's; NaN (the default) = the
 *     robot's maxAngularRadPerSecSquared (ANGULAR_ACCEL), or 720°/s² when the robot has none ({@link #alphaCap})
 * @param headingEaseInPerSecond the heading ease-in line, rad/s per radian left
 * @param headingDeadbandRadians ask for zero spin inside this
 * @param limiterFrictionCoefficient the output limiter's budget as a μ (μ·g shared by translation and
 *     spin); NaN, the default, = the robot's traction limit without its 10 % margin ({@link
 *     frc.team5712.path.core.RobotConfig#tractionMetersPerSecondSquared} / 0.9): the plan keeps the margin, so
 *     feedback has that room
 * @param positionToleranceMeters finish: this close
 * @param headingToleranceRadians finish: this well aligned
 * @param settledMetersPerSecond finish: slower than this (measured)
 * @param debounceSeconds finish: all three held this long
 * @param goalMoveMeters a bound goal that moves further than this rebuilds the tail
 * @param goalMoveRadians ... or turns further than this
 * @param adaptiveGrip when a {@link frc.team5712.path.slip.SlipSignal} is attached: on slip, lower
 *     the grip the limiter, the brake and the speed plan use to what the floor gave, then recover
 * @param slipConfidence the slip signal's confidence that counts as slipping
 * @param slipMargin take this share off the acceleration achieved when slip began
 * @param slipRecoveryPerSecond how fast the lowered μ climbs back, μ per second (0.065: the 0.05 tuned on a
 *     μ 1.15 floor, scaled with the tread to 1.5)
 * @param minSlipFriction never plan below this μ (0.39: 0.3 scaled the same way)
 * @param slipBrakeFriction while slip has lowered the grip, the limiter still lets braking use at
 *     least this μ: the brake law plans with the lowered μ (braking early), and the limiter does not
 *     forbid the robot from braking harder if it must (1.11: 0.85 scaled the same way)
 * @param expectedSlipShare adaptive grip ignores slip that the tread's own weight transfer explains: slip in a
 *     spell that has asked for at least this share of the wheel-slip onset {@code μ·g / (1 + μ·h/x)}. Every
 *     wheel pushes about the same, so above that the light wheels slip on a good floor too; it says nothing
 *     about the floor. Slip while asking for less does
 * @param slideGapMetersPerSecondSquared ...and so does slip while the wheels' acceleration is this far from the
 *     IMU's: the wheels spinning up together while the chassis doesn't follow, so the loaded wheels slide too
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
    double terminalTurnRadians,
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
    double goalMoveRadians,
    boolean adaptiveGrip,
    double slipConfidence,
    double slipMargin,
    double slipRecoveryPerSecond,
    double minSlipFriction,
    double slipBrakeFriction,
    double expectedSlipShare,
    double slideGapMetersPerSecondSquared) {

  /** The defaults for a robot. */
  public static FollowerConfig defaults(RobotConfig robot) {
    return builder(robot).build();
  }

  /** The spin acceleration cap in use: the set one, or the robot's, or 720°/s² (rad/s²). */
  public double alphaCap() {
    if (!Double.isNaN(maxAlphaRadPerSecSquared)) {
      return maxAlphaRadPerSecSquared;
    }
    double robotCap = robot.maxAngularRadPerSecSquared();
    return Double.isFinite(robotCap) ? robotCap : Math.toRadians(720.0);
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
    b.terminalTurnRadians = terminalTurnRadians;
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
    b.adaptiveGrip = adaptiveGrip;
    b.slipConfidence = slipConfidence;
    b.slipMargin = slipMargin;
    b.slipRecoveryPerSecond = slipRecoveryPerSecond;
    b.minSlipFriction = minSlipFriction;
    b.slipBrakeFriction = slipBrakeFriction;
    b.expectedSlipShare = expectedSlipShare;
    b.slideGapMetersPerSecondSquared = slideGapMetersPerSecondSquared;
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
    private double terminalTurnRadians = Math.toRadians(2.0);
    private double brakeMetersPerSecondSquared = Double.NaN;
    private double easeInPerSecond = 12.0; // DriveToPose ships 6; the bench sweep's pick (tuning seeds)
    private double stopDeadbandMeters = 0.003;
    private double maxOmegaRadPerSec = Math.toRadians(540.0);
    private double maxAlphaRadPerSecSquared = Double.NaN;
    private double headingEaseInPerSecond = 5.0;
    private double headingDeadbandRadians = Math.toRadians(0.3);
    private double limiterFrictionCoefficient = Double.NaN;
    private double positionToleranceMeters = 0.02;
    private double headingToleranceRadians = Math.toRadians(1.0);
    private double settledMetersPerSecond = 0.05;
    private double debounceSeconds = 0.1;
    private double goalMoveMeters = 0.02;
    private double goalMoveRadians = Math.toRadians(2.0);
    private boolean adaptiveGrip = true;
    private double slipConfidence = 0.3;
    private double slipMargin = 0.1;
    private double slipRecoveryPerSecond = 0.065;
    private double minSlipFriction = 0.39;
    private double slipBrakeFriction = 1.11;
    private double expectedSlipShare = 0.8;
    private double slideGapMetersPerSecondSquared = 3.5;

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
    public Builder terminalTurnRadians(double v) { terminalTurnRadians = v; return this; }
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
    public Builder adaptiveGrip(boolean v) { adaptiveGrip = v; return this; }
    public Builder slipConfidence(double v) { slipConfidence = v; return this; }
    public Builder slipMargin(double v) { slipMargin = v; return this; }
    public Builder slipRecoveryPerSecond(double v) { slipRecoveryPerSecond = v; return this; }
    public Builder minSlipFriction(double v) { minSlipFriction = v; return this; }
    public Builder slipBrakeFriction(double v) { slipBrakeFriction = v; return this; }
    public Builder expectedSlipShare(double v) { expectedSlipShare = v; return this; }
    public Builder slideGapMetersPerSecondSquared(double v) { slideGapMetersPerSecondSquared = v; return this; }

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
        case "terminalTurnRadians" -> terminalTurnRadians = v;
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
        case "adaptiveGrip" -> adaptiveGrip = v != 0.0;
        case "slipConfidence" -> slipConfidence = v;
        case "slipMargin" -> slipMargin = v;
        case "slipRecoveryPerSecond" -> slipRecoveryPerSecond = v;
        case "minSlipFriction" -> minSlipFriction = v;
        case "slipBrakeFriction" -> slipBrakeFriction = v;
        case "expectedSlipShare" -> expectedSlipShare = v;
        case "slideGapMetersPerSecondSquared" -> slideGapMetersPerSecondSquared = v;
        default -> {
          return false;
        }
      }
      return true;
    }

    public FollowerConfig build() {
      return new FollowerConfig(robot, latencySeconds, windowMeters, reprojectMeters, crossTrackKp,
          crossTrackKd, headingKp, terminalRadiusMeters, terminalTurnRadians, brakeMetersPerSecondSquared,
          easeInPerSecond,
          stopDeadbandMeters, maxOmegaRadPerSec, maxAlphaRadPerSecSquared, headingEaseInPerSecond,
          headingDeadbandRadians, limiterFrictionCoefficient, positionToleranceMeters,
          headingToleranceRadians, settledMetersPerSecond, debounceSeconds, goalMoveMeters,
          goalMoveRadians, adaptiveGrip, slipConfidence, slipMargin, slipRecoveryPerSecond,
          minSlipFriction, slipBrakeFriction,
          expectedSlipShare, slideGapMetersPerSecondSquared);
    }
  }
}
