package frc.team5712.path.follow;

import frc.team5712.path.core.Angles;
import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathEvent;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.PathSpec;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.Projection;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.RotationTarget;
import frc.team5712.path.core.VelocityProfile;
import frc.team5712.path.core.Waypoint;
import frc.team5712.path.slip.SlipSignal;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * One controller from path to pose. Knows nothing about WPILib or the command framework: call
 * {@link #start} when the path begins, {@link #step} every loop, and stop when {@link #isFinished}.
 *
 * <h2>Each loop (in the blue frame)</h2>
 *
 * <ol>
 *   <li><b>Frame.</b> The measured pose and velocity go into the blue frame ({@link FieldFrame}); the
 *       output comes back. The path is never rebuilt or copied for red.
 *   <li><b>Progress</b> {@code s}: the closest point within ±{@code windowMeters} of the last one. If
 *       that point is more than {@code reprojectMeters} away (a shove), the whole path is searched
 *       too and the nearer answer wins.
 *   <li><b>Along the path:</b> the plan is read where it will be one latency later, at
 *       {@code s⁺ = s(t(s) + τ)}, which also gets the robot off a standing start. The speed is
 *       {@code min(profile(s⁺), sqrt(v_end² + 2·a·(d − v·τ)))}: DriveToPose's brake on the distance
 *       left {@code d}. It points along the tangent at {@code s⁺}, which turns the command into a
 *       curve early by {@code v²κτ}: the curvature feed-forward.
 *   <li><b>Across the path:</b> PD on the cross-track error, pushing along the normal.
 *   <li><b>Heading:</b> {@code v·dθ/ds} at {@code s⁺} as feed-forward, plus P on the planned heading
 *       at {@code s}.
 *   <li><b>Near the end</b> (less than {@code terminalRadiusMeters} to go), the reference becomes the
 *       goal pose and DriveToPose's law drives straight to it. Both laws brake on the same distance
 *       with the same deceleration, so the hand-over has no seam.
 *   <li><b>Limiter:</b> every output goes through one {@link OutputLimiter} that shares the grip
 *       budget between translation and spin.
 * </ol>
 *
 * <p><b>Adaptive grip</b> (with a {@link SlipSignal} attached and {@code adaptiveGrip} on): when slip
 * starts, the follower takes the acceleration the robot achieves, less a margin, as the floor's grip
 * μ_slip (the best value seen during the spell, since the filtered IMU lags the onset). The limiter's
 * budget for speeding up becomes μ_slip·g (braking keeps at least {@code slipBrakeFriction}), the
 * brake law's deceleration at most
 * μ_slip·g, and the planned speed is scaled by sqrt(μ_slip / μ_plan), since speed on a curve goes
 * as the square root of grip. Once the slip stops, μ_slip climbs back at {@code slipRecoveryPerSecond},
 * except in the terminal phase, where the robot stops on the grip it has measured.
 * The profile is never rebuilt; this is per-loop arithmetic.
 *
 * <p>Finished: within the position and heading tolerances and measured slower than the settled
 * speed, held for the debounce time. A path whose end speed is above zero (a rolling end) never stops:
 * it is finished once the robot passes the end, and {@link #endCommand} is the velocity to hand to the
 * next follower.
 *
 * <p>Build followers and their paths in robot init. {@link #start} does no path work: no building,
 * no flipping, no copying of the path.
 */
public final class PathFollower {
  /** What the follower is doing. */
  public enum Phase {
    IDLE,
    TRACKING,
    TERMINAL,
    FINISHED,
    CANCELLED
  }

  /** The robot loop period assumed for a step that comes at the same time as the one before (or start()). */
  private static final double NOMINAL_LOOP_SECONDS = 0.02;

  /** Progress faster than this (m/s) is a jump (a shove, a re-projection), not motion. */
  private static final double MAX_PROGRESS_RATE = 8.0;

  /** Paths shorter than this to the moved goal aren't rebuilt; the terminal law handles them. */
  private static final double MIN_TAIL_METERS = 0.15;

  private final Path basePath;
  private final FollowerConfig cfg;
  private final OutputLimiter limiter;
  private final boolean rollingEnd;
  private final double endSpeed;
  private final Pose baseGoal; // blue frame
  /** A turn on the spot (two waypoints at one place): progress comes from the heading, not position. */
  private final boolean pureRotation;

  // Events: names, positions on the base path, positions on the active path, and state
  private final List<String> eventNames = new ArrayList<>();
  private final List<Runnable> eventActions = new ArrayList<>();
  private double[] eventBaseS = new double[0];
  private double[] eventS = new double[0];
  private boolean[] eventFired = new boolean[0];
  private final List<Consumer<String>> eventListeners = new ArrayList<>();

  // Overrides
  private DoubleSupplier lateralOffset = () -> 0.0;
  private DoubleSupplier alongTrackScale = () -> 1.0;
  private DoubleSupplier overrideX;
  private DoubleSupplier overrideY;
  private DoubleSupplier overrideOmega;
  private OverrideMode overrideXMode = OverrideMode.RESPECT_LIMITS;
  private OverrideMode overrideYMode = OverrideMode.RESPECT_LIMITS;
  private OverrideMode overrideOmegaMode = OverrideMode.RESPECT_LIMITS;

  // Moving goal
  private Supplier<Optional<Pose>> goalSupplier;

  // Adaptive grip
  private SlipSignal slipSignal;
  private double slipMu = Double.POSITIVE_INFINITY; // the floor's grip as measured; +∞ = no slip seen
  private boolean wasSlipping;
  private double spellStartMu = Double.POSITIVE_INFINITY; // slipMu before the current slip spell
  private double spellStart; // when it began
  private double commandedAccel; // filtered |commanded acceleration|, m/s²
  private double stepPeak; // the hardest single loop's |commanded acceleration| in the last SPELL_ONSET_SECONDS, m/s²
  private double stepPeakTime = Double.NEGATIVE_INFINITY;
  private double spellMaxCommanded; // the most the current spell has asked for
  private double spellMaxStep; // the biggest single loop's step during the current spell, m/s²
  private double spellMaxAchieved; // the most the IMU has seen the robot achieve during the current spell, m/s²
  private boolean spellLatched; // it has lowered slipMu

  // Per-run state
  private FieldFrame frame = FieldFrame.BLUE;
  private Path active;
  private Pose goal; // blue frame
  private Phase phase = Phase.IDLE;
  private double s;
  private double lastS; // progress at the previous step, for its rate
  private double lastTime;
  // The pose at the previous step, and the velocity between the two: where the body is really going (the
  // wheels' velocity says where they roll, and after a hit they slide)
  private double prevX = Double.NaN;
  private double prevY;
  private double prevPoseTime;
  private double poseVx;
  private double poseVy;
  private double startTime;
  private double headingError = Double.NaN;
  private double measuredSpeed = Double.NaN;
  private double measuredOmega = Double.NaN;
  private double settledSince = Double.NaN;
  private double crossTrack;
  private double refX;
  private double refY;
  private double refHeading;
  private int tailRebuilds;
  private double lastRebuildMicros = Double.NaN;
  // The path followed when not rejoining: the base path, or it with its speeds re-planned from the measured
  // speed at start, or a rebuilt tail
  private Path track;
  // A piece from where the robot was knocked to back onto the track at headJoinS (null when on the track),
  // replacing the stretch from headStartS (track metres)
  private Path head;
  private double headStartS;
  private double headJoinS;
  private int rejoins;
  private double lastRejoinTime = Double.NEGATIVE_INFINITY;
  private double lastRejoinMicros = Double.NaN;
  private double lastStartReplanMicros = Double.NaN;

  /** A follower for a path built in robot init. Cheap: nothing about the path is copied. */
  public PathFollower(Path path, FollowerConfig config) {
    this.basePath = path;
    this.cfg = config;
    this.limiter = new OutputLimiter(config.robot(), limiterBudget(config), config.alphaCap());
    this.endSpeed = path.spec().endMetersPerSecond();
    this.rollingEnd = endSpeed > 0.0;
    PathSample end = path.sample(path.lengthMeters());
    this.baseGoal = new Pose(end.xMeters(), end.yMeters(), end.headingRadians());
    this.active = path;
    this.track = path;
    this.goal = baseGoal;
    this.pureRotation = path.lengthMeters() <= 1.5 * Path.PURE_ROTATION_METERS;
    for (PathEvent e : path.spec().events()) {
      addEvent(e.name(), e.sMeters(path.geometry()), () -> {});
    }
  }

  // ------------------------------------------------------------------------------ set-up (robot init)

  /**
   * Runs {@code action} when the robot passes {@code sMeters} along the path. The path file's own
   * events are already registered (with no action; {@link #onEvent} sees their names), so this is for
   * events that only exist in code.
   */
  public PathFollower addEvent(String name, double sMeters, Runnable action) {
    eventNames.add(name);
    eventActions.add(action);
    int n = eventNames.size();
    double[] base = java.util.Arrays.copyOf(eventBaseS, n);
    base[n - 1] = Math.max(0.0, Math.min(basePath.lengthMeters(), sMeters));
    // keep events sorted by position (stable), so they fire in order
    for (int i = n - 1; i > 0 && base[i - 1] > base[i]; i--) {
      double t = base[i - 1];
      base[i - 1] = base[i];
      base[i] = t;
      eventNames.add(i - 1, eventNames.remove(i));
      eventActions.add(i - 1, eventActions.remove(i));
    }
    eventBaseS = base;
    eventS = base.clone();
    eventFired = new boolean[n];
    return this;
  }

  /** The same, at a waypoint-relative position (1.5 = halfway along the arc from waypoint 1 to 2). */
  public PathFollower addEventAtWaypoint(String name, double waypoint, Runnable action) {
    return addEvent(name, basePath.geometry().sAtWaypoint(waypoint), action);
  }

  /**
   * Adds a listener called with each event's name as it fires (after its own action), e.g. to start a
   * command. Listeners add up: the follow commands register theirs, and robot code can add more.
   */
  public PathFollower onEvent(Consumer<String> listener) {
    eventListeners.add(listener);
    return this;
  }

  /**
   * Binds the end of the path to a moving goal (blue or red frame, whichever the robot's pose is in).
   * When it moves more than the config's thresholds, the tail from the robot's progress to the goal
   * is rebuilt. An empty value keeps the last goal.
   */
  public PathFollower bindGoal(Supplier<Optional<Pose>> goal) {
    this.goalSupplier = goal;
    return this;
  }

  /** Attaches a slip signal (e.g. {@code SlipAwareOdometry.detector()}) for adaptive grip. */
  public PathFollower withSlipSignal(SlipSignal signal) {
    this.slipSignal = signal;
    return this;
  }

  /** Shifts the reference sideways from the path, metres to the left of travel. */
  public PathFollower lateralOffset(DoubleSupplier meters) {
    this.lateralOffset = meters;
    return this;
  }

  /** Scales the along-track speed, 0..1. */
  public PathFollower alongTrackScale(DoubleSupplier scale) {
    this.alongTrackScale = scale;
    return this;
  }

  /** Field-axis velocity override for x (robot's frame); a NaN from the supplier means no override. */
  public PathFollower overrideX(DoubleSupplier metersPerSecond, OverrideMode mode) {
    this.overrideX = metersPerSecond;
    this.overrideXMode = mode;
    return this;
  }

  public PathFollower overrideY(DoubleSupplier metersPerSecond, OverrideMode mode) {
    this.overrideY = metersPerSecond;
    this.overrideYMode = mode;
    return this;
  }

  public PathFollower overrideOmega(DoubleSupplier radPerSec, OverrideMode mode) {
    this.overrideOmega = radPerSec;
    this.overrideOmegaMode = mode;
    return this;
  }

  // ------------------------------------------------------------------------------ running

  /** Starts from the measured velocity. */
  public void start(double timeSeconds, Pose measuredPose, FieldSpeeds measuredVelocity, FieldFrame frame) {
    start(timeSeconds, measuredPose, measuredVelocity, frame, measuredVelocity);
  }

  /**
   * Starts with the limiter's command set to {@code handedCommand} (the previous follower's
   * {@link #endCommand} on a rolling chain). Allocates no path, profile or copy of either.
   */
  public void start(
      double timeSeconds, Pose measuredPose, FieldSpeeds measuredVelocity, FieldFrame frame,
      FieldSpeeds handedCommand) {
    this.frame = frame;
    this.active = basePath;
    this.goal = baseGoal;
    FieldSpeeds cmd = frame.toBlue(handedCommand);
    limiter.reset(cmd.vxMetersPerSecond(), cmd.vyMetersPerSecond(), cmd.omegaRadPerSec());
    Pose p = frame.toBlue(measuredPose);
    s = 0.0; // near the start first (a loop that ends where it began must not start at the end)
    updateProgress(p);
    lastS = s;
    lastTime = timeSeconds;
    prevX = Double.NaN;
    startTime = timeSeconds;
    headingError = Double.NaN;
    measuredSpeed = Double.NaN;
    measuredOmega = Double.NaN;
    settledSince = Double.NaN;
    phase = Phase.TRACKING;
    slipMu = Double.POSITIVE_INFINITY;
    wasSlipping = false;
    spellStartMu = Double.POSITIVE_INFINITY;
    spellLatched = false;
    commandedAccel = 0.0;
    stepPeak = 0.0;
    stepPeakTime = Double.NEGATIVE_INFINITY;
    limiter.setSpeedingUpBudget(Double.POSITIVE_INFINITY);
    limiter.setBrakeBudget(Double.POSITIVE_INFINITY);
    System.arraycopy(eventBaseS, 0, eventS, 0, eventS.length);
    java.util.Arrays.fill(eventFired, false);
    head = null;
    rejoins = 0;
    lastRejoinTime = Double.NEGATIVE_INFINITY;
    lastStartReplanMicros = Double.NaN;
    track = basePath;
    FieldSpeeds v = frame.toBlue(measuredVelocity);
    double speed = Math.hypot(v.vxMetersPerSecond(), v.vyMetersPerSecond());
    PathSample first = basePath.sample(0.0);
    if (cfg.replanStartFromMeasuredVelocity() && !pureRotation && basePath.spec().startMetersPerSecond() == 0.0
        && speed > MIN_MOVING_SPEED
        && v.vxMetersPerSecond() * first.tangentX() + v.vyMetersPerSecond() * first.tangentY() > 0.0) {
      // A path planned from rest, started while already moving (not backwards along the path): the speeds
      // are re-planned from the measured speed, all of it, as PathPlanner does. Turning the velocity onto
      // the path costs grip, not speed, and the limiter shares the grip between the two. Only the profile
      // is rebuilt; the curve stays as drawn. A path that starts moving (a rolling hand-over, an on-the-fly
      // path) already plans its start speed, so it is left alone.
      long t0 = System.nanoTime();
      track = basePath.withStartSpeed(speed, cfg.robot());
      lastStartReplanMicros = (System.nanoTime() - t0) / 1e3;
      active = track;
    }
  }

  /** Below this speed the robot has no direction of travel worth keeping. */
  private static final double MIN_MOVING_SPEED = 0.1;

  /**
   * Replaces the track up to {@code joinS} with a piece from the robot's pose, along its velocity and at its
   * speed, that meets the track at {@code joinS} with matching position, tangent and curvature (and the
   * planned speed there). A piece to a stopping end heads straight for the goal instead (its tangent there
   * is the chord's, its curvature free): off the path that close to the end, the way the path arrives isn't
   * worth a loop to match. Only a join ahead of the robot will do: in front of where it is going, with the
   * track there leading on away from it (else the piece would loop back); and the piece mustn't double
   * back. Returns whether it was built; afterwards progress is along the piece, and the follower moves back
   * onto the track when the piece ends.
   */
  private boolean rejoin(Pose pose, double vx, double vy, double joinS) {
    long t0 = System.nanoTime();
    double length = track.lengthMeters();
    joinS = Math.min(joinS, length);
    PathSample j = track.sample(joinS);
    double v = Math.hypot(vx, vy);
    double dx = j.xMeters() - pose.xMeters();
    double dy = j.yMeters() - pose.yMeters();
    boolean toEnd = joinS >= length - 1e-9;
    boolean straightIn = toEnd && !rollingEnd; // a rolling end keeps its exit direction
    if (Math.hypot(dx, dy) < MIN_TAIL_METERS || (v > MIN_MOVING_SPEED && vx * dx + vy * dy <= 0.0)
        || (!straightIn && j.tangentX() * dx + j.tangentY() * dy <= 0.0)) {
      return false;
    }
    double chord = Angles.atan2(dy, dx);
    PathSpec spec = track.spec();
    double startTangent = v > MIN_MOVING_SPEED ? Angles.atan2(vy, vx) : chord;
    double endTangent = straightIn ? chord : Angles.atan2(j.tangentY(), j.tangentX());
    double endK = straightIn ? Double.NaN : j.curvature();
    PathSpec pieceSpec = new PathSpec(
        List.of(Waypoint.at(pose.xMeters(), pose.yMeters(), startTangent),
            Waypoint.at(j.xMeters(), j.yMeters(), endTangent)),
        List.of(RotationTarget.atMeters(0.0, pose.headingRadians()), RotationTarget.atWaypoint(1.0, j.headingRadians())),
        List.of(),
        List.of(),
        spec.maxMetersPerSecond(),
        spec.maxMetersPerSecondSquared(),
        Math.min(v, spec.maxMetersPerSecond()),
        toEnd ? spec.endMetersPerSecond() : j.metersPerSecond());
    Path piece;
    try {
      piece = Path.build(pieceSpec, cfg.robot(), VelocityProfile.DEFAULT_GRID_METERS, Double.NaN, endK);
    } catch (RuntimeException e) {
      return false; // can't be planned
    }
    if (piece.geometry().hasCusps()) {
      return false;
    }
    headStartS = trackProgress();
    head = piece;
    headJoinS = joinS;
    active = piece;
    s = 0.0;
    rejoins++;
    lastRejoinMicros = (System.nanoTime() - t0) / 1e3;
    return true;
  }

  /** Metres left to the end of the path: on a rejoining piece, its rest plus the track after the join. */
  private double remainingMeters(double sNow) {
    if (head != null) {
      return head.lengthMeters() - sNow + (track.lengthMeters() - headJoinS);
    }
    return active.lengthMeters() - sNow;
  }

  /** Progress in the track's metres (events are placed in them): on a rejoining piece, its share of the stretch it replaces. */
  private double trackProgress() {
    if (head != null) {
      return headStartS + (headJoinS - headStartS) * Math.min(1.0, s / head.lengthMeters());
    }
    return s;
  }

  /** The robot has reached the end of the rejoining piece (or passed its end line, near it). */
  private boolean headDone(Pose pose) {
    double length = head.lengthMeters();
    if (s >= length - 1e-3) {
      return true;
    }
    PathSample end = head.sample(length);
    double past = (pose.xMeters() - end.xMeters()) * end.tangentX() + (pose.yMeters() - end.yMeters()) * end.tangentY();
    return past >= 0.0 && length - s <= cfg.windowMeters();
  }

  /** One loop: measured pose and field velocity in, field speeds out (both in the robot's frame). */
  public FieldSpeeds step(double timeSeconds, Pose measuredPose, FieldSpeeds measuredVelocity) {
    if (phase == Phase.IDLE) {
      throw new IllegalStateException("step() before start()");
    }
    if (phase == Phase.FINISHED || phase == Phase.CANCELLED) {
      return rollingEnd && phase == Phase.FINISHED ? endCommand() : new FieldSpeeds(0.0, 0.0, 0.0);
    }
    // The first step usually comes in the same loop as start(), at the same time: plan for a whole loop
    // then, or the limiter allows only a millisecond's acceleration and the robot starts a loop late
    double dt = timeSeconds > lastTime ? Math.max(0.001, Math.min(0.1, timeSeconds - lastTime)) : NOMINAL_LOOP_SECONDS;
    lastTime = timeSeconds;
    Pose pose = frame.toBlue(measuredPose);
    FieldSpeeds vel = frame.toBlue(measuredVelocity);
    updatePoseVelocity(timeSeconds, pose, vel);
    if (!(Double.isFinite(pose.xMeters()) && Double.isFinite(pose.yMeters())
        && Double.isFinite(pose.headingRadians()))) {
      limiter.limit(0.0, 0.0, 0.0, dt); // no pose: brake within the limits
      return output();
    }

    measuredSpeed = Math.hypot(vel.vxMetersPerSecond(), vel.vyMetersPerSecond());
    measuredOmega = vel.omegaRadPerSec();
    pollGoal(pose);
    updateGrip(timeSeconds, dt);
    double slipBudget = slipBudget();
    double planned = cfg.robot().tractionMetersPerSecondSquared();
    double brakeSetting = Double.isNaN(cfg.brakeMetersPerSecondSquared()) ? planned : cfg.brakeMetersPerSecondSquared();
    double brakeDecel = Math.min(brakeSetting, slipBudget);
    double gripScale = slipBudget < planned ? Math.sqrt(slipBudget / planned) : 1.0;
    updateProgress(pose);
    if (head != null && headJoinS < track.lengthMeters() - 1e-9 && headDone(pose)) {
      active = track; // the piece has met the track: on along it
      head = null;
      s = headJoinS;
      updateProgress(pose);
    }
    // Knocked well off: a piece from where the robot is, the way it is really going, back onto the track
    // a little ahead (or further, to the first join ahead of the robot; or to the goal, if that's near).
    // Never past a cusp: the robot must still get to where the path reverses.
    if (phase == Phase.TRACKING && !pureRotation && Math.abs(crossTrack) > cfg.rejoinAboveMeters()
        && timeSeconds - lastRejoinTime >= FollowerConfig.REJOIN_COOLDOWN_SECONDS) {
      lastRejoinTime = timeSeconds;
      double from = trackProgress();
      double cusp = track.geometry().nextCusp(from);
      double length = track.lengthMeters();
      double last = Double.isNaN(cusp) ? length : cusp - CUSP_EDGE_METERS;
      double join = from;
      do {
        join += FollowerConfig.REJOIN_METERS;
        if (join > length - cfg.terminalRadiusMeters()) {
          join = length;
        }
        join = Math.min(join, last);
      } while (!rejoin(pose, poseVx, poseVy, join) && join < last);
    }
    double remaining = remainingMeters(s);
    if (phase == Phase.TRACKING && !rollingEnd && remaining < cfg.terminalRadiusMeters()
        && Math.abs(Angles.wrap(goal.headingRadians() - active.heading().sample(s).radians())) <= cfg.terminalTurnRadians()
        && Double.isNaN(active.geometry().nextCusp(s))) { // not while the path still has to double back
      phase = Phase.TERMINAL;
    }

    double wantVx;
    double wantVy;
    double wantOmega;
    double offset = lateralOffset.getAsDouble();
    double scale = Math.max(0.0, Math.min(1.0, alongTrackScale.getAsDouble()));
    if (phase == Phase.TERMINAL) {
      // DriveToPose on the goal (shifted sideways by the offset, along the end's normal)
      PathSample end = active.sample(active.lengthMeters());
      double gx = goal.xMeters() - end.tangentY() * offset;
      double gy = goal.yMeters() + end.tangentX() * offset;
      double ex = gx - pose.xMeters();
      double ey = gy - pose.yMeters();
      double d = Math.sqrt(ex * ex + ey * ey);
      double speed = scale * StoppingLaw.speed(d, limiter.speed(), brakeDecel,
          cfg.latencySeconds(), active.spec().maxMetersPerSecond(), cfg.easeInPerSecond(),
          cfg.stopDeadbandMeters());
      wantVx = d > 1e-6 ? ex / d * speed : 0.0;
      wantVy = d > 1e-6 ? ey / d * speed : 0.0;
      double headingError = Angles.wrap(goal.headingRadians() - pose.headingRadians());
      wantOmega = StoppingLaw.spin(headingError, limiter.omega(), cfg.alphaCap(),
          cfg.latencySeconds(), cfg.maxOmegaRadPerSec(), cfg.headingEaseInPerSecond(),
          cfg.headingDeadbandRadians());
      crossTrack = 0.0;
      refX = gx;
      refY = gy;
      refHeading = goal.headingRadians();
    } else if (pureRotation && active == basePath) {
      // A turn on the spot: spin as the plan does (its speed along the 1 mm segment carries the spin),
      // and hold the point. Braking on the distance left, as a path does, would brake on a fraction of
      // a millimetre that the pose noise alone exceeds, and stall the turn.
      VelocityProfile profile = active.profile();
      PathSample ahead = active.sample(profile.sAtTime(profile.timeAt(s) + cfg.latencySeconds()));
      double headingError = Angles.wrap(active.heading().sample(s).radians() - pose.headingRadians());
      wantOmega = scale * gripScale * ahead.metersPerSecond() * ahead.headingRadiansPerMeter() + cfg.headingKp() * headingError;
      wantOmega = Math.max(-cfg.maxOmegaRadPerSec(), Math.min(cfg.maxOmegaRadPerSec(), wantOmega));
      wantVx = cfg.crossTrackKp() * (goal.xMeters() - pose.xMeters());
      wantVy = cfg.crossTrackKp() * (goal.yMeters() - pose.yMeters());
      crossTrack = Math.hypot(goal.xMeters() - pose.xMeters(), goal.yMeters() - pose.yMeters());
      refX = goal.xMeters();
      refY = goal.yMeters();
      refHeading = active.heading().sample(s).radians();
    } else {
      VelocityProfile profile = active.profile();
      double lead = cfg.latencySeconds();
      if (track.spec().startMetersPerSecond() > 0.0) {
        // A rolling start: the last path's command led into its own end, not into this path's speeding up,
        // so the lead builds up over ROLLING_LEAD_SECONDS instead of in one loop (a jump of a·τ that the
        // limiter would take at full grip, slipping the wheels at the hand-over)
        lead *= Math.min(1.0, (timeSeconds - startTime) / ROLLING_LEAD_SECONDS);
      }
      double sAhead = profile.sAtTime(profile.timeAt(s) + lead);
      double cusp = active.geometry().nextCusp(s);
      if (sAhead > cusp - CUSP_EDGE_METERS) {
        // the plan stops at a cusp and comes back: read ahead no further than the stop, on this side of it
        sAhead = Math.max(s, cusp - CUSP_EDGE_METERS);
      }
      PathSample here = active.sample(s);
      PathSample ahead = active.sample(sAhead);
      double brake = StoppingLaw.brake(remaining, limiter.speed(), brakeDecel, cfg.latencySeconds(), endSpeed);
      double speed = scale * Math.min(gripScale * ahead.metersPerSecond(), brake);
      // Across: P and D along the normal at s (left of travel is positive)
      double nx = -here.tangentY();
      double ny = here.tangentX();
      crossTrack = (pose.xMeters() - here.xMeters()) * nx + (pose.yMeters() - here.yMeters()) * ny;
      double drift = vel.vxMetersPerSecond() * nx + vel.vyMetersPerSecond() * ny;
      double sideways = -cfg.crossTrackKp() * (crossTrack - offset) - cfg.crossTrackKd() * drift;
      wantVx = speed * ahead.tangentX() + sideways * nx;
      wantVy = speed * ahead.tangentY() + sideways * ny;
      double headingError = Angles.wrap(here.headingRadians() - pose.headingRadians());
      wantOmega = speed * ahead.headingRadiansPerMeter() + cfg.headingKp() * headingError;
      wantOmega = Math.max(-cfg.maxOmegaRadPerSec(), Math.min(cfg.maxOmegaRadPerSec(), wantOmega));
      refX = here.xMeters() + nx * offset;
      refY = here.yMeters() + ny * offset;
      refHeading = here.headingRadians();
    }

    headingError = Angles.wrap(refHeading - pose.headingRadians());

    // Raw field-axis overrides, given in the robot's frame: into blue, then before or after the limiter
    double ox = overrideX == null ? Double.NaN : overrideX.getAsDouble();
    double oy = overrideY == null ? Double.NaN : overrideY.getAsDouble();
    double oo = overrideOmega == null ? Double.NaN : overrideOmega.getAsDouble();
    FieldSpeeds over = frame.toBlue(new FieldSpeeds(ox, oy, oo)); // NaN stays NaN, signs flip
    if (overrideXMode == OverrideMode.RESPECT_LIMITS && !Double.isNaN(ox)) {
      wantVx = over.vxMetersPerSecond();
    }
    if (overrideYMode == OverrideMode.RESPECT_LIMITS && !Double.isNaN(oy)) {
      wantVy = over.vyMetersPerSecond();
    }
    if (overrideOmegaMode == OverrideMode.RESPECT_LIMITS && !Double.isNaN(oo)) {
      wantOmega = over.omegaRadPerSec();
    }
    double lastVx = limiter.vx();
    double lastVy = limiter.vy();
    limiter.limit(wantVx, wantVy, wantOmega, dt);
    // the commanded acceleration as the robot will feel it: lagged by the response delay and filtered
    double cmdAccel = Math.hypot(limiter.vx() - lastVx, limiter.vy() - lastVy) / dt;
    if (cmdAccel >= stepPeak || timeSeconds - stepPeakTime > SPELL_ONSET_SECONDS) { // the detector lags a loop or two
      stepPeak = cmdAccel;
      stepPeakTime = timeSeconds;
    }
    commandedAccel += (cmdAccel - commandedAccel) * Math.min(1.0, dt / (cfg.latencySeconds() + COMMAND_FILTER_SECONDS));
    if (!Double.isNaN(ox) && overrideXMode == OverrideMode.BYPASS
        || !Double.isNaN(oy) && overrideYMode == OverrideMode.BYPASS
        || !Double.isNaN(oo) && overrideOmegaMode == OverrideMode.BYPASS) {
      limiter.force(
          !Double.isNaN(ox) && overrideXMode == OverrideMode.BYPASS ? over.vxMetersPerSecond() : limiter.vx(),
          !Double.isNaN(oy) && overrideYMode == OverrideMode.BYPASS ? over.vyMetersPerSecond() : limiter.vy(),
          !Double.isNaN(oo) && overrideOmegaMode == OverrideMode.BYPASS ? over.omegaRadPerSec() : limiter.omega());
    }

    // Events fire on the loop nearest to passing them: the progress made per loop says how far the
    // robot gets before the next one, and an event less than half of that ahead fires now. Firing on
    // the first loop past it would be late by up to a loop of travel (8 cm at 4 m/s and 50 Hz).
    double progress = trackProgress();
    double rate = Math.max(0.0, Math.min(MAX_PROGRESS_RATE, (progress - lastS) / dt));
    lastS = progress;
    double lead = 0.5 * rate * dt; // half the travel to the next loop
    fireEventsUpTo(progress + lead);
    if (phase == Phase.TERMINAL && arrived(pose)) {
      fireEventsUpTo(track.lengthMeters()); // an event at the end fires when the robot gets there, not after it settles
    }
    if (head == null || headJoinS >= track.lengthMeters() - 1e-9) { // not done while a piece is still to meet the track
      checkFinished(timeSeconds, pose, vel, remaining, lead);
    }
    return output();
  }

  /** On the goal within the finish tolerances (not yet settled). */
  private boolean arrived(Pose pose) {
    return Math.hypot(refX - pose.xMeters(), refY - pose.yMeters()) < cfg.positionToleranceMeters()
        && Math.abs(Angles.wrap(goal.headingRadians() - pose.headingRadians())) < cfg.headingToleranceRadians();
  }

  /** Interrupted: stops tracking and cancels the events that haven't fired. */
  public void cancel() {
    if (phase == Phase.TRACKING || phase == Phase.TERMINAL) {
      phase = Phase.CANCELLED;
    }
  }

  public boolean isFinished() {
    return phase == Phase.FINISHED;
  }

  /** The last command, in the robot's frame: on a rolling end, the velocity to hand on. */
  public FieldSpeeds endCommand() {
    return output();
  }

  private FieldSpeeds output() {
    return frame.fromBlue(new FieldSpeeds(limiter.vx(), limiter.vy(), limiter.omega()));
  }

  // ------------------------------------------------------------------------------ pieces

  /**
   * The output limiter's budget, m/s²: the robot's traction limit without its 10 % margin (the plan keeps
   * the margin, so feedback has that room), or the configured μ·g.
   */
  static double limiterBudget(FollowerConfig cfg) {
    return Double.isNaN(cfg.limiterFrictionCoefficient())
        ? cfg.robot().tractionMetersPerSecondSquared() / cfg.robot().tractionShare()
        : cfg.limiterFrictionCoefficient() * RobotConfig.GRAVITY;
  }

  /** The μ at which adaptive grip stops limiting anything: the tread's, or the limiter's if that is higher. */
  private double slipCeiling() {
    double mu = cfg.robot().frictionCoefficient();
    return Double.isNaN(cfg.limiterFrictionCoefficient()) ? mu : Math.max(cfg.limiterFrictionCoefficient(), mu);
  }

  /** Adaptive grip: latch the achieved acceleration when slip begins, recover slowly after. */
  private void updateGrip(double timeSeconds, double dt) {
    if (slipSignal == null || !cfg.adaptiveGrip()) {
      return;
    }
    boolean slipping = slipSignal.confidence() >= cfg.slipConfidence();
    double floorMu = Double.POSITIVE_INFINITY;
    boolean expected = false;
    if (slipping) {
      if (!wasSlipping) { // a new spell
        spellStartMu = slipMu;
        spellStart = timeSeconds;
        spellLatched = false;
        spellMaxCommanded = 0.0;
        spellMaxStep = 0.0;
        spellMaxAchieved = 0.0;
      }
      // Every wheel pushes about the same, so on the tread's own floor the wheels that weight transfer
      // unloads let go once the robot asks for μ·g / (1 + μ·h/x) (the wheel-slip onset): some wheels slip
      // while the rest grip. That slip is expected, not a slick floor, and changes nothing. A slick floor
      // shows either way: slip while asking for less than expectedSlipShare of that onset, or the wheels
      // together spinning up faster than the IMU says the chassis does (the gap at least
      // slideGapMetersPerSecondSquared: the loaded wheels are sliding too). The commanded acceleration is
      // filtered and lagged like the robot's response, and it lags the spell's start, so expected slip
      // early in a spell also undoes what the spell's first samples latched.
      floorMu = slipSignal.achievedAccelMetersPerSecondSquared() / RobotConfig.GRAVITY;
      spellMaxCommanded = Math.max(spellMaxCommanded, commandedAccel);
      spellMaxStep = Math.max(spellMaxStep, stepPeak);
      double onset = cfg.robot().wheelSlipOnsetMetersPerSecondSquared();
      boolean sliding = slipSignal.imuGapMetersPerSecondSquared() >= cfg.slideGapMetersPerSecondSquared();
      expected = spellMaxCommanded >= cfg.expectedSlipShare() * onset && !sliding;
      // A spell that starts just after a hard step (a launch, the first loop of braking) is the step's own
      // transient while it lasts no longer than BRIEF_SPELL_SECONDS: a step slips the wheels at once, and the
      // detector holds the spell a while after. A slick floor is still slipping after that.
      boolean stepTransient = spellMaxStep >= cfg.expectedSlipShare() * cfg.robot().wheelSlipOnsetMetersPerSecondSquared()
          && timeSeconds - spellStart <= BRIEF_SPELL_SECONDS;
      // ...and either way the robot still gets most of what it asks for: on its own floor the loaded wheels make
      // up for the light ones. Getting under EXPECTED_ACHIEVED_SHARE of the (lagged) commanded acceleration is the
      // floor giving out, however hard the ask.
      double asked = spellMaxCommanded;
      spellMaxAchieved = Math.max(spellMaxAchieved, slipSignal.achievedAccelMetersPerSecondSquared()); // the IMU lags
      boolean delivering = spellMaxAchieved >= EXPECTED_ACHIEVED_SHARE * asked;
      expected = (expected || stepTransient && !sliding) && delivering;
      if (expected && timeSeconds - spellStart <= BRIEF_SPELL_SECONDS) {
        slipMu = spellStartMu;
        spellLatched = false;
      }
    }
    if (slipping && !expected) {
      // what the floor is giving now, less the margin; during a spell the best of it counts, since a
      // sliding tread still delivers kinetic friction and the filtered IMU lags the onset
      double mu = Math.max(cfg.minSlipFriction(), floorMu * (1.0 - cfg.slipMargin()));
      slipMu = spellLatched ? Math.max(slipMu, mu) : Math.min(slipMu, mu);
      slipMu = Math.min(slipMu, slipCeiling());
      spellLatched = true;
    } else if (!slipping && Double.isFinite(slipMu) && phase != Phase.TERMINAL) {
      // recover, but not while stopping: a floor that slid on the way in still slides at the end
      slipMu += cfg.slipRecoveryPerSecond() * dt;
      if (slipMu >= slipCeiling()) {
        slipMu = Double.POSITIVE_INFINITY;
      }
    }
    wasSlipping = slipping;
    double budget = slipBudget();
    limiter.setSpeedingUpBudget(budget);
    limiter.setBrakeBudget(Math.max(budget, cfg.slipBrakeFriction() * RobotConfig.GRAVITY));
  }

  /** The acceleration the measured floor allows, μ_slip·g, m/s² (+∞ when no slip has lowered it). */
  private double slipBudget() {
    return slipMu * RobotConfig.GRAVITY;
  }

  private void updateProgress(Pose pose) {
    if (pureRotation && active == basePath) {
      s = turnProgress(pose.headingRadians());
      return;
    }
    var geometry = active.geometry();
    if (geometry.hasCusps()) {
      updateProgressAroundCusps(pose, geometry);
      return;
    }
    Projection near = geometry.project(pose.xMeters(), pose.yMeters(), s, cfg.windowMeters());
    if (near.distanceMeters() > cfg.reprojectMeters()) {
      Projection all = geometry.projectGlobal(pose.xMeters(), pose.yMeters());
      if (all.distanceMeters() < near.distanceMeters() - 0.05) {
        near = all;
      }
    }
    s = near.s();
  }

  /**
   * The velocity from the pose's change since the last step (the measured one on the first step, or when
   * the pose jumps faster than the robot can go: a vision correction).
   */
  private void updatePoseVelocity(double t, Pose pose, FieldSpeeds vel) {
    double dt = t - prevPoseTime;
    poseVx = vel.vxMetersPerSecond();
    poseVy = vel.vyMetersPerSecond();
    if (!Double.isNaN(prevX) && dt > MIN_POSE_DT) {
      double vx = (pose.xMeters() - prevX) / dt;
      double vy = (pose.yMeters() - prevY) / dt;
      if (Math.hypot(vx, vy) <= MAX_POSE_SPEED_RATIO * track.spec().maxMetersPerSecond()) {
        poseVx = vx;
        poseVy = vy;
      }
    }
    prevX = pose.xMeters();
    prevY = pose.yMeters();
    prevPoseTime = t;
  }

  /** The slip signal's own filter on the achieved acceleration, matched on the commanded one. */
  private static final double COMMAND_FILTER_SECONDS = 0.03;

  /** How long a slip spell's start counts as its onset, while the filtered acceleration catches up. */
  private static final double SPELL_ONSET_SECONDS = 0.1;
  /** A spell that ends within this, after a hard step, was the step's transient. */
  private static final double BRIEF_SPELL_SECONDS = 0.3;
  /** Slip the tread's own still delivers at least this share of the acceleration asked for. */
  private static final double EXPECTED_ACHIEVED_SHARE = 0.8;

  /** Shorter steps than this don't give a velocity from the pose (the measured one is used). */
  private static final double MIN_POSE_DT = 0.005;

  /** A pose moving faster than this many times the path's top speed has jumped. */
  private static final double MAX_POSE_SPEED_RATIO = 1.5;

  /** After a rolling start, the read-ahead grows to the full latency over this long. */
  static final double ROLLING_LEAD_SECONDS = 0.3;

  /** Progress reads no closer to a cusp than this (its tangent is undefined), and steps past it by as much. */
  static final double CUSP_EDGE_METERS = 1e-3;

  /**
   * Progress on a path that doubles back. Around a cusp the two sides lie on top of each other, so the
   * search stays on the robot's side of it: between the last cusp passed and the next one. The robot then
   * crosses when it gets within the position tolerance of the cusp (the plan has stopped there), and
   * progress carries on from the far side.
   */
  private void updateProgressAroundCusps(Pose pose, frc.team5712.path.core.PathGeometry geometry) {
    double next = geometry.nextCusp(s);
    double prev = geometry.previousCusp(s);
    double lo = Double.isNaN(prev) ? 0.0 : prev + CUSP_EDGE_METERS;
    double hi = Double.isNaN(next) ? geometry.lengthMeters() : next - CUSP_EDGE_METERS;
    Projection near = geometry.projectWithin(pose.xMeters(), pose.yMeters(),
        Math.max(lo, s - cfg.windowMeters()), Math.min(hi, s + cfg.windowMeters()));
    if (near.distanceMeters() > cfg.reprojectMeters()) {
      Projection all = geometry.projectGlobalWithin(pose.xMeters(), pose.yMeters(), lo, hi);
      if (all.distanceMeters() < near.distanceMeters() - 0.05) {
        near = all;
      }
    }
    s = near.s();
    if (!Double.isNaN(next) && next - s <= cfg.positionToleranceMeters()) {
      s = Math.min(geometry.lengthMeters(), next + CUSP_EDGE_METERS);
    }
  }

  /**
   * A turn on the spot is 1 mm long, so where the robot is says nothing about how far through the turn
   * it is: progress is where the planned heading reaches the measured one, never going back. That keeps
   * events, the speed read-ahead and the hand-over to the terminal law on the turn's own schedule.
   */
  private double turnProgress(double heading) {
    var profile = active.heading();
    double length = active.lengthMeters();
    int steps = 64;
    double prevS = s;
    double prevErr = Angles.wrap(profile.sample(s).radians() - heading);
    double best = s;
    double bestErr = Math.abs(prevErr);
    for (int i = 1; i <= steps; i++) {
      double si = s + (length - s) * i / steps;
      double err = Angles.wrap(profile.sample(si).radians() - heading);
      if (Math.signum(err) != Math.signum(prevErr) && prevErr != 0.0) {
        // the planned heading passes the measured one in [prevS, si]: bisect
        double lo = prevS;
        double hi = si;
        for (int k = 0; k < 30; k++) {
          double mid = 0.5 * (lo + hi);
          double e = Angles.wrap(profile.sample(mid).radians() - heading);
          if (Math.signum(e) == Math.signum(prevErr)) {
            lo = mid;
          } else {
            hi = mid;
          }
        }
        return 0.5 * (lo + hi);
      }
      if (Math.abs(err) < bestErr) {
        best = si;
        bestErr = Math.abs(err);
      }
      prevS = si;
      prevErr = err;
    }
    return best;
  }

  /** {@code lead}: half a loop's travel, so a rolling end hands over on the loop nearest its line, as events fire. */
  private void checkFinished(double t, Pose pose, FieldSpeeds vel, double remaining, double lead) {
    if (rollingEnd) {
      PathSample end = active.sample(active.lengthMeters());
      double past = (pose.xMeters() - end.xMeters()) * end.tangentX()
          + (pose.yMeters() - end.yMeters()) * end.tangentY();
      // at the end's line (nearest loop), once near the end (a path can start beyond that line, e.g. an
      // on-the-fly one). Waiting for the first loop past it started the next path up to a loop late, so its
      // start events fired up to a loop of travel past their point (8 cm at 4 m/s)
      if (remaining <= 1e-6 || (past + lead >= 0.0 && remaining <= cfg.windowMeters())) {
        finish();
      }
      return;
    }
    if (phase != Phase.TERMINAL) {
      return;
    }
    // the terminal reference is the goal moved sideways by any lateral offset
    double d = Math.hypot(refX - pose.xMeters(), refY - pose.yMeters());
    double yaw = Math.abs(Angles.wrap(goal.headingRadians() - pose.headingRadians()));
    double speed = Math.hypot(vel.vxMetersPerSecond(), vel.vyMetersPerSecond());
    boolean ok = d < cfg.positionToleranceMeters() && yaw < cfg.headingToleranceRadians()
        && speed < cfg.settledMetersPerSecond();
    if (!ok) {
      settledSince = Double.NaN;
    } else if (Double.isNaN(settledSince)) {
      settledSince = t;
    }
    if (ok && t - settledSince >= cfg.debounceSeconds() - 1e-9) {
      finish();
    }
  }

  private void finish() {
    fireEventsUpTo(Double.POSITIVE_INFINITY); // skipped events still fire on a normal finish
    phase = Phase.FINISHED;
    if (!rollingEnd) {
      limiter.force(0.0, 0.0, 0.0);
    }
  }

  private void fireEventsUpTo(double sNow) {
    for (int i = 0; i < eventS.length; i++) {
      if (!eventFired[i] && eventS[i] <= sNow) {
        eventFired[i] = true;
        eventActions.get(i).run();
        for (Consumer<String> l : eventListeners) {
          l.accept(eventNames.get(i));
        }
      }
    }
  }

  // ------------------------------------------------------------------------------ moving goal

  private void pollGoal(Pose pose) {
    if (goalSupplier == null) {
      return;
    }
    Optional<Pose> next = goalSupplier.get();
    if (next.isEmpty()) {
      return; // hold the last estimate
    }
    Pose g = frame.toBlue(next.get());
    double moved = Math.hypot(g.xMeters() - goal.xMeters(), g.yMeters() - goal.yMeters());
    double turned = Math.abs(Angles.wrap(g.headingRadians() - goal.headingRadians()));
    if (moved < cfg.goalMoveMeters() && turned < cfg.goalMoveRadians()) {
      return;
    }
    goal = g;
    if (phase == Phase.TRACKING && Math.hypot(g.xMeters() - pose.xMeters(), g.yMeters() - pose.yMeters())
        > cfg.terminalRadiusMeters() + MIN_TAIL_METERS) {
      long t0 = System.nanoTime();
      rebuildTail();
      lastRebuildMicros = (System.nanoTime() - t0) / 1e3;
    }
    // in the terminal phase the DriveToPose law simply drives to the new goal
  }

  /**
   * Replaces the rest of the path with a new piece from the current progress to the goal. The join
   * matches position, tangent and curvature (the new spline's first curvature is fixed); the heading
   * blends from the planned heading at the join to the goal's; the profile starts at the planned
   * speed there. The approach direction turns with the goal: the old end tangent plus the goal's turn.
   */
  private void rebuildTail() {
    PathSample join = active.sample(s);
    PathSample oldEnd = basePath.sample(basePath.lengthMeters());
    double tangent = Angles.wrap(Math.atan2(oldEnd.tangentY(), oldEnd.tangentX())
        + (goal.headingRadians() - baseGoal.headingRadians()));
    double cx = goal.xMeters() - join.xMeters();
    double cy = goal.yMeters() - join.yMeters();
    if (Math.cos(tangent) * cx + Math.sin(tangent) * cy <= 0.0) {
      tangent = Math.atan2(cy, cx); // the goal is behind the approach direction: head straight in
    }
    PathSpec spec = active.spec();
    PathSpec tailSpec = new PathSpec(
        List.of(
            Waypoint.at(join.xMeters(), join.yMeters(), Math.atan2(join.tangentY(), join.tangentX())),
            Waypoint.at(goal.xMeters(), goal.yMeters(), tangent)),
        List.of(RotationTarget.atMeters(0.0, join.headingRadians()), RotationTarget.atWaypoint(1.0, goal.headingRadians())),
        List.of(),
        List.of(),
        spec.maxMetersPerSecond(),
        spec.maxMetersPerSecondSquared(),
        join.metersPerSecond(),
        spec.endMetersPerSecond());
    double sJoin = trackProgress();
    double oldRemaining = track.lengthMeters() - sJoin;
    Path tail = Path.build(tailSpec, cfg.robot(), VelocityProfile.DEFAULT_GRID_METERS, join.curvature());
    // events still ahead keep their share of the distance left
    for (int i = 0; i < eventS.length; i++) {
      if (!eventFired[i]) {
        double share = oldRemaining > 1e-9 ? (eventS[i] - sJoin) / oldRemaining : 1.0;
        eventS[i] = Math.max(0.0, share) * tail.lengthMeters();
      }
    }
    track = tail;
    active = tail;
    head = null;
    s = 0.0;
    lastS = 0.0;
    tailRebuilds++;
  }

  // ------------------------------------------------------------------------------ telemetry

  public Phase phase() {
    return phase;
  }

  /** Progress along the path (the tail after a rebuild; on a rejoining piece, its share of the stretch it replaces), metres. */
  public double progressMeters() {
    return trackProgress();
  }

  /** Metres left on the active path. */
  public double remainingMeters() {
    return remainingMeters(s);
  }

  /** Signed distance from the path, metres, positive to the left of travel (0 in the terminal phase). */
  public double crossTrackMeters() {
    return crossTrack;
  }

  /**
   * How far ahead of the plan's schedule the robot is, metres: progress minus where the plan is at the
   * time since the start (negative: behind). NaN before the start.
   */
  public double alongTrackMeters() {
    if (phase == Phase.IDLE) {
      return Double.NaN;
    }
    return trackProgress() - track.profile().sAtTime(Math.max(0.0, lastTime - startTime));
  }

  /** The reference heading minus the measured one, radians (blue frame), at the last step. */
  public double headingErrorRadians() {
    return headingError;
  }

  /** The plan's speed at the robot's progress, m/s. */
  public double plannedMetersPerSecond() {
    return phase == Phase.IDLE ? Double.NaN : active.sample(s).metersPerSecond();
  }

  /** The plan's turn rate at the robot's progress, rad/s (blue frame). */
  public double plannedOmegaRadPerSec() {
    if (phase == Phase.IDLE) {
      return Double.NaN;
    }
    PathSample here = active.sample(s);
    return here.metersPerSecond() * here.headingRadiansPerMeter();
  }

  /** The measured speed over the ground at the last step, m/s. */
  public double measuredMetersPerSecond() {
    return measuredSpeed;
  }

  /** The measured turn rate at the last step, rad/s (blue frame). */
  public double measuredOmegaRadPerSec() {
    return measuredOmega;
  }

  /** Seconds since the start, at the last step. */
  public double elapsedSeconds() {
    return lastTime - startTime;
  }

  /** The pose the follower is steering for this loop, in the robot's frame. */
  public Pose reference() {
    return frame.fromBlue(new Pose(refX, refY, refHeading));
  }

  /** The goal pose, in the robot's frame. */
  public Pose goal() {
    return frame.fromBlue(goal);
  }

  /** The path being followed now: the base path, or the tail after a moving goal rebuilt it. */
  public Path activePath() {
    return active;
  }

  public int tailRebuildCount() {
    return tailRebuilds;
  }

  /** Pieces built this run to rejoin the path after the robot was knocked off it. */
  public int rejoinCount() {
    return rejoins;
  }

  /** How long the last rejoining piece took to build, microseconds (NaN if none yet). */
  public double lastRejoinMicros() {
    return lastRejoinMicros;
  }

  /**
   * How long start() took to re-plan the speeds from the measured speed, microseconds (NaN if it didn't:
   * {@link FollowerConfig#replanStartFromMeasuredVelocity} off, or the robot wasn't moving).
   */
  public double lastStartReplanMicros() {
    return lastStartReplanMicros;
  }

  /** How long the last tail rebuild took, microseconds (NaN if none yet). */
  public double lastTailRebuildMicros() {
    return lastRebuildMicros;
  }

  public FieldFrame frame() {
    return frame;
  }

  /** The floor's μ adaptive grip has measured (+∞ when no slip has lowered it). */
  public double slipFrictionCoefficient() {
    return slipMu;
  }

  /** True when the path ends moving (end speed above zero): it finishes as it passes the end. */
  public boolean rollingEnd() {
    return rollingEnd;
  }

  /** The path this follower was built for (before any tail rebuild). */
  public Path path() {
    return basePath;
  }

  /** Every event's name (the file's and code's), in the order they fire. */
  public List<String> eventNames() {
    return List.copyOf(eventNames);
  }

  /** Each event's position on the base path, metres, in the order of {@link #eventNames}. */
  public double[] eventMeters() {
    return eventBaseS.clone();
  }
}
