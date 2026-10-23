package frc.team5712.path.follow;

import frc.team5712.path.core.Angles;
import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathEvent;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.Projection;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.VelocityProfile;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
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

  private final Path path;
  private final FollowerConfig cfg;
  private final OutputLimiter limiter;
  private final boolean rollingEnd;
  private final double endSpeed;
  private final Pose baseGoal; // blue frame
  /** A turn on the spot (two waypoints at one place): progress comes from the heading, not position. */
  private final boolean pureRotation;

  // Events: names, positions along the path, and whether each has fired this run
  private final List<String> eventNames = new ArrayList<>();
  private final List<Runnable> eventActions = new ArrayList<>();
  private double[] eventS = new double[0];
  private boolean[] eventFired = new boolean[0];
  private final List<Consumer<String>> eventListeners = new ArrayList<>();

  // Moving goal
  private Supplier<Optional<Pose>> goalSupplier;

  // Per-run state
  private FieldFrame frame = FieldFrame.BLUE;
  private Pose goal; // blue frame
  private Phase phase = Phase.IDLE;
  private double s;
  private double lastS; // progress at the previous step, for its rate
  private double lastTime;
  private double startTime;
  private double headingError = Double.NaN;
  private double measuredSpeed = Double.NaN;
  private double measuredOmega = Double.NaN;
  private double settledSince = Double.NaN;
  private double crossTrack;
  private double refX;
  private double refY;
  private double refHeading;

  /** A follower for a path built in robot init. Cheap: nothing about the path is copied. */
  public PathFollower(Path path, FollowerConfig config) {
    this.path = path;
    this.cfg = config;
    this.limiter = new OutputLimiter(config.robot(), limiterBudget(config), config.alphaCap());
    this.endSpeed = path.spec().endMetersPerSecond();
    this.rollingEnd = endSpeed > 0.0;
    PathSample end = path.sample(path.lengthMeters());
    this.baseGoal = new Pose(end.xMeters(), end.yMeters(), end.headingRadians());
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
    double[] base = java.util.Arrays.copyOf(eventS, n);
    base[n - 1] = Math.max(0.0, Math.min(path.lengthMeters(), sMeters));
    // keep events sorted by position (stable), so they fire in order
    for (int i = n - 1; i > 0 && base[i - 1] > base[i]; i--) {
      double t = base[i - 1];
      base[i - 1] = base[i];
      base[i] = t;
      eventNames.add(i - 1, eventNames.remove(i));
      eventActions.add(i - 1, eventActions.remove(i));
    }
    eventS = base;
    eventFired = new boolean[n];
    return this;
  }

  /** The same, at a waypoint-relative position (1.5 = halfway along the arc from waypoint 1 to 2). */
  public PathFollower addEventAtWaypoint(String name, double waypoint, Runnable action) {
    return addEvent(name, path.geometry().sAtWaypoint(waypoint), action);
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
   * Binds the end of the path to a moving goal (blue or red frame, whichever the robot's pose is in): the
   * DriveToPose finish drives to its latest value instead of the path's end. The path itself isn't changed;
   * an on-the-fly path plans again when its goal moves far ({@code OnTheFly}). An empty value keeps the last
   * goal.
   */
  public PathFollower bindGoal(Supplier<Optional<Pose>> goal) {
    this.goalSupplier = goal;
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
    this.goal = baseGoal;
    FieldSpeeds cmd = frame.toBlue(handedCommand);
    limiter.reset(cmd.vxMetersPerSecond(), cmd.vyMetersPerSecond(), cmd.omegaRadPerSec());
    Pose p = frame.toBlue(measuredPose);
    s = 0.0; // near the start first (a loop that ends where it began must not start at the end)
    updateProgress(p);
    lastS = s;
    lastTime = timeSeconds;
    startTime = timeSeconds;
    headingError = Double.NaN;
    measuredSpeed = Double.NaN;
    measuredOmega = Double.NaN;
    settledSince = Double.NaN;
    phase = Phase.TRACKING;
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
    // The first step usually comes in the same loop as start(), at the same time: plan for a whole loop
    // then, or the limiter allows only a millisecond's acceleration and the robot starts a loop late
    double dt = timeSeconds > lastTime ? Math.max(0.001, Math.min(0.1, timeSeconds - lastTime)) : NOMINAL_LOOP_SECONDS;
    lastTime = timeSeconds;
    Pose pose = frame.toBlue(measuredPose);
    FieldSpeeds vel = frame.toBlue(measuredVelocity);
    if (!(Double.isFinite(pose.xMeters()) && Double.isFinite(pose.yMeters())
        && Double.isFinite(pose.headingRadians()))) {
      limiter.limit(0.0, 0.0, 0.0, dt); // no pose: brake within the limits
      return output();
    }

    measuredSpeed = Math.hypot(vel.vxMetersPerSecond(), vel.vyMetersPerSecond());
    measuredOmega = vel.omegaRadPerSec();
    pollGoal();
    double brakeDecel = Double.isNaN(cfg.brakeMetersPerSecondSquared())
        ? cfg.robot().tractionMetersPerSecondSquared() : cfg.brakeMetersPerSecondSquared();
    updateProgress(pose);
    double remaining = path.lengthMeters() - s;
    if (phase == Phase.TRACKING && !rollingEnd && remaining < cfg.terminalRadiusMeters()
        && Math.abs(Angles.wrap(baseGoal.headingRadians() - path.heading().sample(s).radians())) <= cfg.terminalTurnRadians()
        && Double.isNaN(path.geometry().nextCusp(s))) { // not while the path still has to double back
      phase = Phase.TERMINAL;
    }

    double wantVx;
    double wantVy;
    double wantOmega;
    if (phase == Phase.TERMINAL) {
      // DriveToPose on the goal
      double gx = goal.xMeters();
      double gy = goal.yMeters();
      double ex = gx - pose.xMeters();
      double ey = gy - pose.yMeters();
      double d = Math.sqrt(ex * ex + ey * ey);
      double speed = StoppingLaw.speed(d, limiter.speed(), brakeDecel,
          cfg.latencySeconds(), path.spec().maxMetersPerSecond(), cfg.easeInPerSecond(),
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
    } else if (pureRotation) {
      // A turn on the spot: spin as the plan does (its speed along the 1 mm segment carries the spin),
      // and hold the point. Braking on the distance left, as a path does, would brake on a fraction of
      // a millimetre that the pose noise alone exceeds, and stall the turn.
      VelocityProfile profile = path.profile();
      PathSample ahead = path.sample(profile.sAtTime(profile.timeAt(s) + readAheadSeconds()));
      double headingError = Angles.wrap(path.heading().sample(s).radians() - pose.headingRadians());
      wantOmega = ahead.metersPerSecond() * ahead.headingRadiansPerMeter() + cfg.headingKp() * headingError;
      wantOmega = Math.max(-cfg.maxOmegaRadPerSec(), Math.min(cfg.maxOmegaRadPerSec(), wantOmega));
      wantVx = cfg.crossTrackKp() * (goal.xMeters() - pose.xMeters());
      wantVy = cfg.crossTrackKp() * (goal.yMeters() - pose.yMeters());
      crossTrack = Math.hypot(goal.xMeters() - pose.xMeters(), goal.yMeters() - pose.yMeters());
      refX = goal.xMeters();
      refY = goal.yMeters();
      refHeading = path.heading().sample(s).radians();
    } else {
      VelocityProfile profile = path.profile();
      double lead = readAheadSeconds();
      if (path.spec().startMetersPerSecond() > 0.0) {
        // A rolling start: the last path's command led into its own end, not into this path's speeding up,
        // so the lead builds up over ROLLING_LEAD_SECONDS instead of in one loop (a jump of a·τ that the
        // limiter would take at full grip, slipping the wheels at the hand-over)
        lead *= Math.min(1.0, (timeSeconds - startTime) / ROLLING_LEAD_SECONDS);
      }
      double sAhead = profile.sAtTime(profile.timeAt(s) + lead);
      double cusp = path.geometry().nextCusp(s);
      if (sAhead > cusp - CUSP_EDGE_METERS) {
        // the plan stops at a cusp and comes back: read ahead no further than the stop, on this side of it
        sAhead = Math.max(s, cusp - CUSP_EDGE_METERS);
      }
      PathSample here = path.sample(s);
      PathSample ahead = path.sample(sAhead);
      double brake = StoppingLaw.brake(remaining, limiter.speed(), brakeDecel, cfg.latencySeconds(), endSpeed);
      double speed = Math.min(ahead.metersPerSecond(), brake);
      // Across: P and D along the normal at s (left of travel is positive)
      double nx = -here.tangentY();
      double ny = here.tangentX();
      crossTrack = (pose.xMeters() - here.xMeters()) * nx + (pose.yMeters() - here.yMeters()) * ny;
      double drift = vel.vxMetersPerSecond() * nx + vel.vyMetersPerSecond() * ny;
      double sideways = -cfg.crossTrackKp() * crossTrack - cfg.crossTrackKd() * drift;
      wantVx = speed * ahead.tangentX() + sideways * nx;
      wantVy = speed * ahead.tangentY() + sideways * ny;
      double headingError = Angles.wrap(here.headingRadians() - pose.headingRadians());
      wantOmega = speed * ahead.headingRadiansPerMeter() + cfg.headingKp() * headingError;
      wantOmega = Math.max(-cfg.maxOmegaRadPerSec(), Math.min(cfg.maxOmegaRadPerSec(), wantOmega));
      refX = here.xMeters();
      refY = here.yMeters();
      refHeading = here.headingRadians();
    }

    headingError = Angles.wrap(refHeading - pose.headingRadians());

    limiter.limit(wantVx, wantVy, wantOmega, dt);

    // Events fire on the loop nearest to passing them: the progress made per loop says how far the
    // robot gets before the next one, and an event less than half of that ahead fires now. Firing on
    // the first loop past it would be late by up to a loop of travel (8 cm at 4 m/s and 50 Hz).
    double progress = s;
    double rate = Math.max(0.0, Math.min(MAX_PROGRESS_RATE, (progress - lastS) / dt));
    lastS = progress;
    double lead = 0.5 * rate * dt; // half the travel to the next loop
    fireEventsUpTo(progress + lead);
    if (phase == Phase.TERMINAL && arrived(pose)) {
      fireEventsUpTo(path.lengthMeters()); // an event at the end fires when the robot gets there, not after it settles
    }
    checkFinished(timeSeconds, pose, vel, remaining, lead);
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

  /**
   * How far ahead (plan time) the reference is read: the latency, but at least one loop. At rest the plan's
   * speed where the robot stands is zero, so with no lead at all the robot would never start.
   */
  private double readAheadSeconds() {
    return Math.max(cfg.latencySeconds(), NOMINAL_LOOP_SECONDS);
  }

  private void updateProgress(Pose pose) {
    if (pureRotation) {
      s = turnProgress(pose.headingRadians());
      return;
    }
    var geometry = path.geometry();
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
    var profile = path.heading();
    double length = path.lengthMeters();
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
      PathSample end = path.sample(path.lengthMeters());
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

  /** A bound goal's latest value; an empty one keeps the last. The DriveToPose finish drives to it. */
  private void pollGoal() {
    if (goalSupplier != null) {
      goalSupplier.get().ifPresent(g -> goal = frame.toBlue(g));
    }
  }

  // ------------------------------------------------------------------------------ telemetry

  public Phase phase() {
    return phase;
  }

  /** Progress along the path, metres. */
  public double progressMeters() {
    return s;
  }

  /** Metres left on the path. */
  public double remainingMeters() {
    return path.lengthMeters() - s;
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
    return s - path.profile().sAtTime(Math.max(0.0, lastTime - startTime));
  }

  /** The reference heading minus the measured one, radians (blue frame), at the last step. */
  public double headingErrorRadians() {
    return headingError;
  }

  /** The plan's speed at the robot's progress, m/s. */
  public double plannedMetersPerSecond() {
    return phase == Phase.IDLE ? Double.NaN : path.sample(s).metersPerSecond();
  }

  /** The plan's turn rate at the robot's progress, rad/s (blue frame). */
  public double plannedOmegaRadPerSec() {
    if (phase == Phase.IDLE) {
      return Double.NaN;
    }
    PathSample here = path.sample(s);
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

  public FieldFrame frame() {
    return frame;
  }

  /** True when the path ends moving (end speed above zero): it finishes as it passes the end. */
  public boolean rollingEnd() {
    return rollingEnd;
  }

  /** The path this follower follows. */
  public Path path() {
    return path;
  }

  /** Every event's name (the file's and code's), in the order they fire. */
  public List<String> eventNames() {
    return List.copyOf(eventNames);
  }

  /** Each event's position along the path, metres, in the order of {@link #eventNames}. */
  public double[] eventMeters() {
    return eventS.clone();
  }
}
