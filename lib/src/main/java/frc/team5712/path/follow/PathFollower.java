package frc.team5712.path.follow;

import frc.team5712.path.core.Angles;
import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Path;
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

  /** Paths shorter than this to the moved goal aren't rebuilt; the terminal law handles them. */
  private static final double MIN_TAIL_METERS = 0.15;

  private final Path basePath;
  private final FollowerConfig cfg;
  private final OutputLimiter limiter;
  private final boolean rollingEnd;
  private final double endSpeed;
  private final Pose baseGoal; // blue frame

  // Events: names, positions on the base path, positions on the active path, and state
  private final List<String> eventNames = new ArrayList<>();
  private final List<Runnable> eventActions = new ArrayList<>();
  private double[] eventBaseS = new double[0];
  private double[] eventS = new double[0];
  private boolean[] eventFired = new boolean[0];
  private Consumer<String> eventListener = name -> {};

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

  // Per-run state
  private FieldFrame frame = FieldFrame.BLUE;
  private Path active;
  private Pose goal; // blue frame
  private Phase phase = Phase.IDLE;
  private double s;
  private double lastTime;
  private double settledSince = Double.NaN;
  private double crossTrack;
  private double refX;
  private double refY;
  private double refHeading;
  private int tailRebuilds;
  private double lastRebuildMicros = Double.NaN;

  /** A follower for a path built in robot init. Cheap: nothing about the path is copied. */
  public PathFollower(Path path, FollowerConfig config) {
    this.basePath = path;
    this.cfg = config;
    this.limiter = new OutputLimiter(config.robot(), config.limiterFrictionCoefficient(), config.maxAlphaRadPerSecSquared());
    this.endSpeed = path.spec().endMetersPerSecond();
    this.rollingEnd = endSpeed > 0.0;
    PathSample end = path.sample(path.lengthMeters());
    this.baseGoal = new Pose(end.xMeters(), end.yMeters(), end.headingRadians());
    this.active = path;
    this.goal = baseGoal;
  }

  // ------------------------------------------------------------------------------ set-up (robot init)

  /** Runs {@code action} when the robot passes {@code sMeters} along the path. */
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

  /** Called with each event's name as it fires (after its own action), e.g. to start a command. */
  public PathFollower onEvent(Consumer<String> listener) {
    this.eventListener = listener;
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
    lastTime = timeSeconds;
    settledSince = Double.NaN;
    phase = Phase.TRACKING;
    slipMu = Double.POSITIVE_INFINITY;
    wasSlipping = false;
    limiter.setFrictionCoefficient(Double.POSITIVE_INFINITY);
    limiter.setBrakeFrictionCoefficient(Double.POSITIVE_INFINITY);
    System.arraycopy(eventBaseS, 0, eventS, 0, eventS.length);
    java.util.Arrays.fill(eventFired, false);
  }

  /** One loop: measured pose and field velocity in, field speeds out (both in the robot's frame). */
  public FieldSpeeds step(double timeSeconds, Pose measuredPose, FieldSpeeds measuredVelocity) {
    if (phase == Phase.IDLE) {
      throw new IllegalStateException("step() before start()");
    }
    if (phase == Phase.FINISHED || phase == Phase.CANCELLED) {
      return rollingEnd && phase == Phase.FINISHED ? endCommand() : new FieldSpeeds(0.0, 0.0, 0.0);
    }
    double dt = Math.max(0.001, Math.min(0.1, timeSeconds - lastTime));
    lastTime = timeSeconds;
    Pose pose = frame.toBlue(measuredPose);
    FieldSpeeds vel = frame.toBlue(measuredVelocity);
    if (!(Double.isFinite(pose.xMeters()) && Double.isFinite(pose.yMeters())
        && Double.isFinite(pose.headingRadians()))) {
      limiter.limit(0.0, 0.0, 0.0, dt); // no pose: brake within the limits
      return output();
    }

    pollGoal(pose);
    updateGrip(dt);
    double brakeDecel = Math.min(cfg.brakeMetersPerSecondSquared(), slipMu * RobotConfig.GRAVITY);
    double planMu = cfg.robot().frictionCoefficient();
    double gripScale = slipMu < planMu ? Math.sqrt(slipMu / planMu) : 1.0;
    updateProgress(pose);
    double remaining = active.lengthMeters() - s;
    if (phase == Phase.TRACKING && !rollingEnd && remaining < cfg.terminalRadiusMeters()) {
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
      wantOmega = StoppingLaw.spin(headingError, limiter.omega(), cfg.maxAlphaRadPerSecSquared(),
          cfg.latencySeconds(), cfg.maxOmegaRadPerSec(), cfg.headingEaseInPerSecond(),
          cfg.headingDeadbandRadians());
      crossTrack = 0.0;
      refX = gx;
      refY = gy;
      refHeading = goal.headingRadians();
    } else {
      VelocityProfile profile = active.profile();
      double sAhead = profile.sAtTime(profile.timeAt(s) + cfg.latencySeconds());
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
    limiter.limit(wantVx, wantVy, wantOmega, dt);
    if (!Double.isNaN(ox) && overrideXMode == OverrideMode.BYPASS
        || !Double.isNaN(oy) && overrideYMode == OverrideMode.BYPASS
        || !Double.isNaN(oo) && overrideOmegaMode == OverrideMode.BYPASS) {
      limiter.force(
          !Double.isNaN(ox) && overrideXMode == OverrideMode.BYPASS ? over.vxMetersPerSecond() : limiter.vx(),
          !Double.isNaN(oy) && overrideYMode == OverrideMode.BYPASS ? over.vyMetersPerSecond() : limiter.vy(),
          !Double.isNaN(oo) && overrideOmegaMode == OverrideMode.BYPASS ? over.omegaRadPerSec() : limiter.omega());
    }

    fireEventsUpTo(s);
    checkFinished(timeSeconds, pose, vel, remaining);
    return output();
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

  /** Adaptive grip: latch the achieved acceleration when slip begins, recover slowly after. */
  private void updateGrip(double dt) {
    if (slipSignal == null || !cfg.adaptiveGrip()) {
      return;
    }
    boolean slipping = slipSignal.confidence() >= cfg.slipConfidence();
    if (slipping) {
      // what the floor is giving now, less the margin; during a spell the best of it counts, since a
      // sliding tread still delivers kinetic friction and the filtered IMU lags the onset
      double mu = Math.max(cfg.minSlipFriction(),
          slipSignal.achievedAccelMetersPerSecondSquared() / RobotConfig.GRAVITY * (1.0 - cfg.slipMargin()));
      slipMu = wasSlipping ? Math.max(slipMu, mu) : Math.min(slipMu, mu);
      slipMu = Math.min(slipMu, Math.max(cfg.limiterFrictionCoefficient(), cfg.robot().frictionCoefficient()));
    } else if (Double.isFinite(slipMu) && phase != Phase.TERMINAL) {
      // recover, but not while stopping: a floor that slid on the way in still slides at the end
      slipMu += cfg.slipRecoveryPerSecond() * dt;
      if (slipMu >= Math.max(cfg.limiterFrictionCoefficient(), cfg.robot().frictionCoefficient())) {
        slipMu = Double.POSITIVE_INFINITY;
      }
    }
    wasSlipping = slipping;
    limiter.setFrictionCoefficient(slipMu);
    limiter.setBrakeFrictionCoefficient(Math.max(slipMu, cfg.slipBrakeFriction()));
  }

  private void updateProgress(Pose pose) {
    var geometry = active.geometry();
    Projection near = geometry.project(pose.xMeters(), pose.yMeters(), s, cfg.windowMeters());
    if (near.distanceMeters() > cfg.reprojectMeters()) {
      Projection all = geometry.projectGlobal(pose.xMeters(), pose.yMeters());
      if (all.distanceMeters() < near.distanceMeters() - 0.05) {
        near = all;
      }
    }
    s = near.s();
  }

  private void checkFinished(double t, Pose pose, FieldSpeeds vel, double remaining) {
    if (rollingEnd) {
      PathSample end = active.sample(active.lengthMeters());
      double past = (pose.xMeters() - end.xMeters()) * end.tangentX()
          + (pose.yMeters() - end.yMeters()) * end.tangentY();
      if (remaining <= 1e-6 || past >= 0.0) {
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
        eventListener.accept(eventNames.get(i));
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
    double oldRemaining = active.lengthMeters() - s;
    double sJoin = s;
    Path tail = Path.build(tailSpec, cfg.robot(), VelocityProfile.DEFAULT_GRID_METERS, join.curvature());
    // events still ahead keep their share of the distance left
    for (int i = 0; i < eventS.length; i++) {
      if (!eventFired[i]) {
        double share = oldRemaining > 1e-9 ? (eventS[i] - sJoin) / oldRemaining : 1.0;
        eventS[i] = Math.max(0.0, share) * tail.lengthMeters();
      }
    }
    active = tail;
    s = 0.0;
    tailRebuilds++;
  }

  // ------------------------------------------------------------------------------ telemetry

  public Phase phase() {
    return phase;
  }

  /** Progress along the active path (the tail after a rebuild), metres. */
  public double progressMeters() {
    return s;
  }

  /** Metres left on the active path. */
  public double remainingMeters() {
    return active.lengthMeters() - s;
  }

  /** Signed distance from the path, metres, positive to the left of travel (0 in the terminal phase). */
  public double crossTrackMeters() {
    return crossTrack;
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

  /** How long the last tail rebuild took, microseconds (NaN if none yet). */
  public double lastTailRebuildMicros() {
    return lastRebuildMicros;
  }

  public FieldFrame frame() {
    return frame;
  }

  /** The grip μ adaptive grip has measured (+∞ when no slip has lowered it). */
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
}
