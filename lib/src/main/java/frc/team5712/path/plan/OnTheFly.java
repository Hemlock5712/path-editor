package frc.team5712.path.plan;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Pose;
import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.follow.PathFollower;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * An on-the-fly path: planned around the field's obstacles from where the robot is when it starts, then
 * followed. The WPILib commands wrap this; it has no WPILib in it, so it is tested on its own.
 *
 * <ul>
 *   <li>{@link #start}: plans from the measured pose and velocity to the goal ({@link Pathfinder#plan}; a
 *       start or goal inside a grown obstacle is snapped out) and starts a {@link PathFollower} on it. This
 *       is the one place path work happens at go: a few milliseconds.
 *   <li>{@link #step}: one follower step. If the robot is knocked further than {@link #replanAboveMeters}
 *       off the path (a hard shove), or {@link #requestReplan} was called, it plans again from where it
 *       is, at the speed it is going, and carries on with the new path.
 *   <li>A moving goal ({@link #withMovingGoal}, e.g. a tag or a game piece): the follower's DriveToPose finish
 *       always drives to its latest value, and once it is more than {@link #GOAL_REPLAN_METERS} from the goal
 *       the path was planned to, it plans again, round the obstacles, to the latest value.
 * </ul>
 *
 * Everything is in the real field frame: the robot's pose, the obstacles in field.json, the goal. The
 * caller flips an authored blue-frame goal for the red alliance before handing it in.
 */
public final class OnTheFly {
  /** Default re-plan threshold: further off the path than this. */
  public static final double DEFAULT_REPLAN_METERS = 0.20;
  /** At most one automatic re-plan per this many seconds (the robot needs a moment to get back on). */
  public static final double REPLAN_COOLDOWN_SECONDS = 0.5;
  /** The wider-margin route is used only when it is at most this much longer than the main one's. */
  public static final double WIDE_DETOUR_RATIO = 1.15;
  /** A moving goal further than this from the goal the path was planned to plans again (m). */
  public static final double GOAL_REPLAN_METERS = 0.10;
  /** ...at most once per this long (s). */
  public static final double GOAL_REPLAN_COOLDOWN_SECONDS = 0.2;

  private final Pathfinder pathfinder;
  private Pathfinder wide; // tried first when set: the same field with a wider margin
  private boolean lastPlanWide;
  private final FollowerConfig config;
  private final Supplier<Pose> goal;
  private double endSpeed;
  private double endTangent = Double.NaN;
  private double replanAboveMeters = DEFAULT_REPLAN_METERS;
  private Supplier<Optional<Pose>> movingGoal;
  private Consumer<OnTheFly> onPlan = p -> {};

  private PathFollower follower;
  private PlanResult lastPlan;
  private Pose plannedGoal; // what the current path was planned to
  private FieldSpeeds lastOutput = new FieldSpeeds(0, 0, 0);
  private boolean replanRequested;
  private double lastPlanTime = Double.NEGATIVE_INFINITY;
  private int replans;
  private String failure;
  private double maxPlanMicros;

  /** Plans to {@code goal}, read when the run starts and again at each re-plan (real field frame). */
  public OnTheFly(Pathfinder pathfinder, FollowerConfig config, Supplier<Pose> goal) {
    this.pathfinder = pathfinder;
    this.config = config;
    this.goal = goal;
  }

  /** Arrive at {@code metersPerSecond} travelling in direction {@code tangentRadians} (a hand-over to a path). */
  public OnTheFly arriving(double metersPerSecond, double tangentRadians) {
    this.endSpeed = metersPerSecond;
    this.endTangent = tangentRadians;
    return this;
  }

  /**
   * Plans with {@code widerMargin} first (the same field with a wider margin, e.g. {@link
   * PlannerConfig#WIDE_MARGIN_METERS}), and with the main pathfinder when that finds no route or its route is
   * more than {@link #WIDE_DETOUR_RATIO} times as long: a wide berth where there is room, and still through the
   * narrow gaps without a detour. Null: the main pathfinder only.
   */
  public OnTheFly preferring(Pathfinder widerMargin) {
    this.wide = widerMargin;
    return this;
  }

  /** Re-plan when further than this off the path (+∞ = never on its own). */
  public OnTheFly replanAbove(double meters) {
    this.replanAboveMeters = meters;
    return this;
  }

  /** Finishes on a moving goal: the path's end follows it (an empty value keeps the last). */
  public OnTheFly withMovingGoal(Supplier<Optional<Pose>> goal) {
    this.movingGoal = goal;
    return this;
  }

  /** Called after every plan (the first and each re-plan), e.g. to publish it. */
  public OnTheFly onPlan(Consumer<OnTheFly> listener) {
    this.onPlan = listener;
    return this;
  }

  /** Plans again on the next step, from wherever the robot is then. */
  public void requestReplan() {
    replanRequested = true;
  }

  /** Plans and starts. False (and {@link #failure}) when there is no route; the run is then finished. */
  public boolean start(double t, Pose pose, FieldSpeeds velocity) {
    replans = 0;
    failure = null;
    maxPlanMicros = 0;
    lastOutput = velocity;
    replanRequested = false;
    return plan(t, pose, velocity, velocity);
  }

  private boolean plan(double t, Pose pose, FieldSpeeds velocity, FieldSpeeds handed) {
    Pose target = follower != null && movingGoal != null ? follower.goal() : currentGoal();
    plannedGoal = target;
    // the wide berth only when it doesn't send the robot the long way round (a gap it can't use)
    boolean useWide = wide != null
        && wide.routeLengthMeters(pose, target) <= WIDE_DETOUR_RATIO * pathfinder.routeLengthMeters(pose, target);
    PlanResult r = useWide ? wide.plan(pose, velocity, target, endSpeed, endTangent) : null;
    lastPlanWide = r != null && r.ok();
    if (!lastPlanWide) {
      r = pathfinder.plan(pose, velocity, target, endSpeed, endTangent);
    }
    lastPlan = r;
    lastPlanTime = t;
    maxPlanMicros = Math.max(maxPlanMicros, r.totalMicros());
    if (!r.ok()) {
      failure = r.failure();
      follower = null;
      onPlan.accept(this);
      return false;
    }
    follower = new PathFollower(r.path(), config);
    if (movingGoal != null) {
      follower.bindGoal(movingGoal);
    }
    follower.start(t, pose, velocity, FieldFrame.BLUE, handed);
    onPlan.accept(this);
    return true;
  }

  private Pose currentGoal() {
    if (movingGoal != null) {
      Optional<Pose> g = movingGoal.get();
      if (g.isPresent()) {
        return g.get();
      }
    }
    return goal.get();
  }

  /** One loop: measured pose and field velocity in (real frame), field speeds out. */
  public FieldSpeeds step(double t, Pose pose, FieldSpeeds velocity) {
    if (follower == null) {
      return lastOutput = new FieldSpeeds(0, 0, 0);
    }
    boolean knocked = follower.phase() == PathFollower.Phase.TRACKING
        && Math.abs(follower.crossTrackMeters()) > replanAboveMeters
        && t - lastPlanTime >= REPLAN_COOLDOWN_SECONDS;
    // the follower polls the moving goal and finishes on it; once it has gone well away from where this path
    // was planned to, plan again, round the obstacles
    boolean goalMoved = movingGoal != null && follower.phase() == PathFollower.Phase.TRACKING
        && Math.hypot(follower.goal().xMeters() - plannedGoal.xMeters(), follower.goal().yMeters() - plannedGoal.yMeters())
            > GOAL_REPLAN_METERS
        && t - lastPlanTime >= GOAL_REPLAN_COOLDOWN_SECONDS;
    if ((knocked || goalMoved || replanRequested) && !follower.isFinished()) {
      replanRequested = false;
      replans++;
      if (!plan(t, pose, velocity, lastOutput)) {
        return lastOutput = new FieldSpeeds(0, 0, 0);
      }
    }
    return lastOutput = follower.step(t, pose, velocity);
  }

  /** Finished: arrived (or handed over, on a rolling end), or no route was found. */
  public boolean isFinished() {
    return follower == null || follower.isFinished();
  }

  public void cancel() {
    if (follower != null) {
      follower.cancel();
    }
  }

  /** The speed to hand to the next path on a rolling end ({@link PathFollower#endCommand}). */
  public FieldSpeeds endCommand() {
    return follower == null ? new FieldSpeeds(0, 0, 0) : follower.endCommand();
  }

  public boolean rollingEnd() {
    return follower != null && follower.rollingEnd();
  }

  /** The follower on the current plan (null before a plan, or when there is no route). */
  public PathFollower follower() {
    return follower;
  }

  public PlanResult lastPlan() {
    return lastPlan;
  }

  /** The last plan came from the wider-margin pathfinder ({@link #preferring}). */
  public boolean lastPlanWide() {
    return lastPlanWide;
  }

  /** Re-plans so far this run (automatic and requested). */
  public int replans() {
    return replans;
  }

  /** The slowest plan this run, microseconds. */
  public double maxPlanMicros() {
    return maxPlanMicros;
  }

  /** Why there is no route, or null. */
  public String failure() {
    return failure;
  }

  public Pathfinder pathfinder() {
    return pathfinder;
  }
}
