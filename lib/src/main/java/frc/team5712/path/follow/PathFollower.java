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
import java.util.ArrayList;
import java.util.Arrays;
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
 *   <li><b>Progress</b> {@code s}: the closest point within ±{@code WINDOW_METERS} of the last one. If
 *       that point is further away than that (a shove), the whole path is searched too and the nearer
 *       answer wins.
 *   <li><b>Along the path:</b> the plan is read where it will be one latency later, at
 *       {@code s⁺ = s(t(s) + τ)}, which also gets the robot off a standing start. The speed is
 *       {@code min(profile(s⁺), sqrt(v_end² + 2·a·(d − v·τ)))}: DriveToPose's brake on the distance
 *       left {@code d}. It points along the tangent at {@code s⁺}, which turns the command into a
 *       curve early by {@code v²κτ}: the curvature feed-forward.
 *   <li><b>Across the path:</b> P on the cross-track error, pushing along the normal.
 *   <li><b>Heading:</b> {@code v·dθ/ds} at {@code s⁺} as feed-forward, plus P on the planned heading
 *       at {@code s}.
 *   <li><b>Near the end</b> (less than {@code TERMINAL_RADIUS_METERS} to go), the reference becomes the
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

  /** Progress is searched for within this of the last; further off than this (a shove), on the whole path too. */
  static final double WINDOW_METERS = 0.5;

  /** The DriveToPose finish takes over this far from the end ... */
  static final double TERMINAL_RADIUS_METERS = 0.3;
  /** ... once the plan has no more than this left to turn (a short hop with a big turn follows its plan longer). */
  static final double TERMINAL_TURN_RADIANS = Math.toRadians(2.0);

  /** The finish asks for no speed this close to the goal, and no spin this close to its heading. */
  static final double STOP_DEADBAND_METERS = 0.003;
  static final double HEADING_DEADBAND_RADIANS = Math.toRadians(0.3);

  /** Finished: within the tolerances, measured slower than this, for this long. */
  static final double SETTLED_METERS_PER_SECOND = 0.05;
  static final double DEBOUNCE_SECONDS = 0.1;

  /** Progress faster than this (m/s) is a jump (a shove, a re-projection), not motion. */
  private static final double MAX_PROGRESS_RATE = 8.0;

  private final Path path; // as built
  private Path active; // the path being followed: the built one, or its end re-planned to a moved goal
  private final FollowerConfig cfg;
  private final OutputLimiter limiter;
  private final boolean rollingEnd;
  private final double endSpeed;
  private final Pose baseGoal; // blue frame
  /** A turn on the spot (two waypoints at one place): progress comes from the heading, not position. */
  private final boolean pureRotation;

  // Events: names, positions along the built path and along the one being followed, and what has fired
  private final List<String> eventNames = new ArrayList<>();
  private final List<Runnable> eventActions = new ArrayList<>();
  private double[] eventBaseS = new double[0];
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
    this.active = path;
    this.cfg = config;
    this.limiter = new OutputLimiter(config.robot(), limiterBudget(config), config.maxAlphaRadPerSecSquared());
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
    double[] base = Arrays.copyOf(eventBaseS, n);
    base[n - 1] = Math.max(0.0, Math.min(path.lengthMeters(), sMeters));
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
   * Binds the end of the path to a moving goal (blue or red frame, whichever the robot's pose is in). When it
   * moves more than 2 cm or 2°, the rest of the path is planned again, from the robot's progress to the goal,
   * joining smoothly; the DriveToPose finish drives to it. An empty value keeps the last goal. (An on-the-fly
   * path also plans again round the obstacles when its goal moves far: {@code OnTheFly}.)
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
    this.active = path;
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
    System.arraycopy(eventBaseS, 0, eventS, 0, eventS.length);
    Arrays.fill(eventFired, false);
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
    pollGoal(pose);
    updateProgress(pose);
    double remaining = active.lengthMeters() - s;
    if (phase == Phase.TRACKING && readyToFinish(remaining)) {
      phase = Phase.TERMINAL;
    }
    FieldSpeeds want;
    if (phase == Phase.TERMINAL) {
      want = driveToGoal(pose);
    } else if (pureRotation && active == path) {
      want = turnOnTheSpot(pose);
    } else {
      want = followPath(pose, remaining, timeSeconds);
    }
    headingError = Angles.wrap(refHeading - pose.headingRadians());

    limiter.limit(want.vxMetersPerSecond(), want.vyMetersPerSecond(), want.omegaRadPerSec(), dt);

    // Events fire on the loop nearest to passing them: the progress made per loop says how far the
    // robot gets before the next one, and an event less than half of that ahead fires now. Firing on
    // the first loop past it would be late by up to a loop of travel (8 cm at 4 m/s and 50 Hz).
    double progress = s;
    double rate = Math.max(0.0, Math.min(MAX_PROGRESS_RATE, (progress - lastS) / dt));
    lastS = progress;
    double lead = 0.5 * rate * dt; // half the travel to the next loop
    fireEventsUpTo(progress + lead);
    if (phase == Phase.TERMINAL && arrived(pose)) {
      fireEventsUpTo(active.lengthMeters()); // an event at the end fires when the robot gets there, not after it settles
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

  // ------------------------------------------------------------------------------ the three laws

  /**
   * The DriveToPose finish takes over once the robot is within {@link #TERMINAL_RADIUS_METERS} of a stopping end
   * and the plan has no more than {@link #TERMINAL_TURN_RADIANS} left to turn, and not while the path still has
   * to double back.
   */
  private boolean readyToFinish(double remaining) {
    double turnLeft = active.sample(active.lengthMeters()).headingRadians() - active.heading().sample(s).radians();
    return !rollingEnd && remaining < TERMINAL_RADIUS_METERS && Math.abs(Angles.wrap(turnLeft)) <= TERMINAL_TURN_RADIANS
        && Double.isNaN(active.geometry().nextCusp(s));
  }

  /**
   * Along the path: the plan read one latency ahead (s⁺), braked by DriveToPose's law on the distance left,
   * pointed along the tangent at s⁺ (which turns early by v²κτ: the curvature feed-forward); P across the
   * path; the planned turn rate at s⁺ plus P on the planned heading at s.
   */
  private FieldSpeeds followPath(Pose pose, double remaining, double timeSeconds) {
    VelocityProfile profile = active.profile();
    double lead = readAheadSeconds();
    if (active.spec().startMetersPerSecond() > 0.0) {
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
    double brake = StoppingLaw.brake(remaining, limiter.speed(), cfg.brakeDecel(), cfg.latencySeconds(), endSpeed);
    double speed = Math.min(ahead.metersPerSecond(), brake);
    // Across: P along the normal at s (left of travel is positive)
    double nx = -here.tangentY();
    double ny = here.tangentX();
    crossTrack = (pose.xMeters() - here.xMeters()) * nx + (pose.yMeters() - here.yMeters()) * ny;
    double sideways = -cfg.crossTrackKp() * crossTrack;
    double headingError = Angles.wrap(here.headingRadians() - pose.headingRadians());
    double omega = speed * ahead.headingRadiansPerMeter() + cfg.headingKp() * headingError;
    refX = here.xMeters();
    refY = here.yMeters();
    refHeading = here.headingRadians();
    return new FieldSpeeds(speed * ahead.tangentX() + sideways * nx, speed * ahead.tangentY() + sideways * ny,
        clampSpin(omega));
  }

  /**
   * A turn on the spot: spin as the plan does (its speed along the 1 mm segment carries the spin), and hold the
   * point. Braking on the distance left, as a path does, would brake on a fraction of a millimetre that the
   * pose noise alone exceeds, and stall the turn.
   */
  private FieldSpeeds turnOnTheSpot(Pose pose) {
    VelocityProfile profile = active.profile();
    PathSample ahead = active.sample(profile.sAtTime(profile.timeAt(s) + readAheadSeconds()));
    double headingError = Angles.wrap(active.heading().sample(s).radians() - pose.headingRadians());
    double omega = ahead.metersPerSecond() * ahead.headingRadiansPerMeter() + cfg.headingKp() * headingError;
    crossTrack = Math.hypot(goal.xMeters() - pose.xMeters(), goal.yMeters() - pose.yMeters());
    refX = goal.xMeters();
    refY = goal.yMeters();
    refHeading = active.heading().sample(s).radians();
    return new FieldSpeeds(cfg.crossTrackKp() * (goal.xMeters() - pose.xMeters()),
        cfg.crossTrackKp() * (goal.yMeters() - pose.yMeters()), clampSpin(omega));
  }

  /** The finish: DriveToPose's stopping law straight at the goal, and the same law for heading. */
  private FieldSpeeds driveToGoal(Pose pose) {
    double ex = goal.xMeters() - pose.xMeters();
    double ey = goal.yMeters() - pose.yMeters();
    double d = Math.sqrt(ex * ex + ey * ey);
    double speed = StoppingLaw.speed(d, limiter.speed(), cfg.brakeDecel(), cfg.latencySeconds(),
        active.spec().maxMetersPerSecond(), cfg.easeInPerSecond(), STOP_DEADBAND_METERS);
    double headingError = Angles.wrap(goal.headingRadians() - pose.headingRadians());
    double omega = StoppingLaw.spin(headingError, limiter.omega(), cfg.maxAlphaRadPerSecSquared(),
        cfg.latencySeconds(), cfg.maxOmegaRadPerSec(), cfg.headingEaseInPerSecond(), HEADING_DEADBAND_RADIANS);
    crossTrack = 0.0;
    refX = goal.xMeters();
    refY = goal.yMeters();
    refHeading = goal.headingRadians();
    return new FieldSpeeds(d > 1e-6 ? ex / d * speed : 0.0, d > 1e-6 ? ey / d * speed : 0.0, omega);
  }

  private double clampSpin(double omega) {
    return Math.max(-cfg.maxOmegaRadPerSec(), Math.min(cfg.maxOmegaRadPerSec(), omega));
  }

  // ------------------------------------------------------------------------------ pieces

  /**
   * The output limiter's budget, m/s²: the robot's traction limit without its 10 % margin (the plan keeps
   * the margin, so feedback has that room).
   */
  static double limiterBudget(FollowerConfig cfg) {
    return cfg.robot().tractionMetersPerSecondSquared() / cfg.robot().tractionShare();
  }

  /**
   * How far ahead (plan time) the reference is read: the latency, but at least one loop. At rest the plan's
   * speed where the robot stands is zero, so with no lead at all the robot would never start.
   */
  private double readAheadSeconds() {
    return Math.max(cfg.latencySeconds(), NOMINAL_LOOP_SECONDS);
  }

  private void updateProgress(Pose pose) {
    if (pureRotation && active == path) {
      s = turnProgress(pose.headingRadians());
      return;
    }
    var geometry = active.geometry();
    if (geometry.hasCusps()) {
      updateProgressAroundCusps(pose, geometry);
      return;
    }
    Projection near = geometry.project(pose.xMeters(), pose.yMeters(), s, WINDOW_METERS);
    if (near.distanceMeters() > WINDOW_METERS) {
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
        Math.max(lo, s - WINDOW_METERS), Math.min(hi, s + WINDOW_METERS));
    if (near.distanceMeters() > WINDOW_METERS) {
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
      if (remaining <= 1e-6 || (past + lead >= 0.0 && remaining <= WINDOW_METERS)) {
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
        && speed < SETTLED_METERS_PER_SECOND;
    if (!ok) {
      settledSince = Double.NaN;
    } else if (Double.isNaN(settledSince)) {
      settledSince = t;
    }
    if (ok && t - settledSince >= DEBOUNCE_SECONDS - 1e-9) {
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

  /** A bound goal that moves this far, or turns this far, re-plans the rest of the path. */
  private static final double GOAL_MOVE_METERS = 0.02;
  private static final double GOAL_MOVE_RADIANS = Math.toRadians(2.0);

  /** Closer than this past the finish's radius, a moved goal is left to the DriveToPose finish. */
  private static final double MIN_TAIL_METERS = 0.15;

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
    if (moved < GOAL_MOVE_METERS && turned < GOAL_MOVE_RADIANS) {
      return;
    }
    goal = g;
    if (phase == Phase.TRACKING && Math.hypot(g.xMeters() - pose.xMeters(), g.yMeters() - pose.yMeters())
        > TERMINAL_RADIUS_METERS + MIN_TAIL_METERS) {
      rebuildTail();
    }
    // in the terminal phase the DriveToPose law simply drives to the new goal
  }

  /**
   * Plans the rest of the path again: one quintic piece from the robot's progress to the goal. The join
   * matches position, tangent and curvature (the new spline's first curvature is fixed); the heading blends
   * from the planned heading at the join to the goal's; the speeds start at the planned speed there and end
   * at the path's end speed (so a path that drives through its goal still does). The approach direction turns
   * with the goal: the old end tangent plus the goal's turn.
   */
  private void rebuildTail() {
    PathSample join = active.sample(s);
    PathSample oldEnd = path.sample(path.lengthMeters());
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
    Path tail = Path.build(tailSpec, cfg.robot(), VelocityProfile.DEFAULT_GRID_METERS, join.curvature());
    // events still ahead keep their share of the distance left
    double left = active.lengthMeters() - s;
    for (int i = 0; i < eventS.length; i++) {
      if (!eventFired[i]) {
        double share = left > 1e-9 ? (eventS[i] - s) / left : 1.0;
        eventS[i] = Math.max(0.0, share) * tail.lengthMeters();
      }
    }
    active = tail;
    s = 0.0;
    lastS = 0.0;
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
    return active.lengthMeters() - s;
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
    return s - active.profile().sAtTime(Math.max(0.0, lastTime - startTime));
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

  public FieldFrame frame() {
    return frame;
  }

  /** True when the path ends moving (end speed above zero): it finishes as it passes the end. */
  public boolean rollingEnd() {
    return rollingEnd;
  }

  /** The path this follower was built with. */
  public Path path() {
    return path;
  }

  /** The path being followed now: the built one, or its end re-planned to a moved goal. */
  public Path activePath() {
    return active;
  }

  /** Every event's name (the file's and code's), in the order they fire. */
  public List<String> eventNames() {
    return List.copyOf(eventNames);
  }

  /** Each event's position along the path, metres, in the order of {@link #eventNames}. */
  public double[] eventMeters() {
    return eventBaseS.clone();
  }
}
