package frc.team5712.path.wpilib;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.Pose;
import frc.team5712.path.plan.OnTheFly;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

/**
 * Commands v3: an on-the-fly path around the field's obstacles (see {@link Pathfinding}, which builds these).
 * Plans when it starts, follows, re-plans when knocked off or asked, and, for {@link Pathfinding#thenFollow},
 * hands over to the library path's own follow command. Telemetry goes out as for a followed path, under
 * Path/&lt;name&gt; (the plan, each loop's state, and the planning figures).
 */
public final class PathfindCommand implements Command {
  private final Pathfinding ctx;
  private final Mechanism drive;
  private final String name;
  private final Pathfinding.Goal goal;
  private final Path handOver;
  private final FollowPathCommand follow;
  private final PathTelemetry telemetry;
  private double replanAbove = OnTheFly.DEFAULT_REPLAN_METERS;
  private Supplier<Optional<Pose2d>> movingGoal;
  private DoubleSupplier clock = Timer::getTimestamp;
  private Supplier<FieldFrame> frameSource;
  private OnTheFly run;
  private boolean following;

  PathfindCommand(Pathfinding ctx, Mechanism drive, String name, Pathfinding.Goal goal, Path handOver,
      FollowPathCommand follow) {
    this.ctx = ctx;
    this.drive = drive;
    this.name = name;
    this.goal = goal;
    this.handOver = handOver;
    this.follow = follow;
    this.telemetry = new PathTelemetry(name);
  }

  /** Re-plan when further than this off the path (default 0.2 m; +∞ = only when asked). */
  public PathfindCommand replanAbove(double meters) {
    this.replanAbove = meters;
    return this;
  }

  /** Finishes on a moving goal (real field frame, e.g. from vision); empty keeps the last. */
  public PathfindCommand withMovingGoal(Supplier<Optional<Pose2d>> realGoal) {
    this.movingGoal = realGoal;
    return this;
  }

  /** Replaces the clock (default {@code Timer.getTimestamp()}). */
  public PathfindCommand withClock(DoubleSupplier seconds) {
    this.clock = seconds;
    if (follow != null) {
      follow.withClock(seconds);
    }
    return this;
  }

  /** Replaces the alliance lookup (default: the driver station, read when it starts). */
  public PathfindCommand withFrame(Supplier<FieldFrame> frame) {
    this.frameSource = frame;
    if (follow != null) {
      follow.withFrame(frame);
    }
    return this;
  }

  /** Plans again on the next loop, from wherever the robot is. */
  public void replan() {
    if (run != null) {
      run.requestReplan();
    }
  }

  /** The current run (null before the first start). */
  public OnTheFly onTheFly() {
    return run;
  }

  /** The library path's follow command after the hand-over, or null. */
  public FollowPathCommand handOverCommand() {
    return follow;
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public Set<Mechanism> requirements() {
    return Set.of(drive);
  }

  @Override
  public void run(Coroutine coroutine) {
    FieldFrame frame = frameSource == null ? Conversions.allianceFrame(ctx.redFrame) : frameSource.get();
    following = false;
    run = new OnTheFly(ctx.library.pathfinder(), ctx.config, () -> goal.apply(frame)).replanAbove(replanAbove)
        .onPlan(r -> {
          telemetry.plan(r.lastPlan(), r.replans());
          if (r.follower() != null) {
            telemetry.begin(r.follower());
          }
        });
    if (handOver != null) {
      run.arriving(handOver.spec().startMetersPerSecond(), Pathfinding.startTangent(handOver, frame));
    }
    if (movingGoal != null) {
      run.withMovingGoal(() -> movingGoal.get().map(Conversions::toPose));
    }
    if (run.start(clock.getAsDouble(), Conversions.toPose(ctx.pose.get()), measured())) {
      while (true) {
        FieldSpeeds out = run.step(clock.getAsDouble(), Conversions.toPose(ctx.pose.get()), measured());
        ctx.fieldOutput.accept(Conversions.toChassisVelocities(out));
        if (run.follower() != null) {
          telemetry.log(run.follower());
        }
        if (run.isFinished()) {
          break;
        }
        coroutine.yield();
      }
    }
    if (follow == null || run.failure() != null) {
      if (!run.rollingEnd()) {
        ctx.fieldOutput.accept(new ChassisVelocities());
      }
      telemetry.end();
      return;
    }
    telemetry.end();
    following = true;
    OnTheFly handing = run;
    follow.continuingFrom(handing::endCommand);
    coroutine.yield(); // the path's command starts on the next loop, as the next command in a sequence would
    follow.run(coroutine);
  }

  private FieldSpeeds measured() {
    return Conversions.toFieldSpeeds(ctx.fieldVelocity.get());
  }

  @Override
  public void onCancel() {
    if (following) {
      follow.onCancel();
      return;
    }
    if (run != null) {
      run.cancel();
    }
    ctx.fieldOutput.accept(new ChassisVelocities());
    telemetry.end();
  }

  /** The pose the robot is heading for now, in the real field frame (for tests and dashboards). */
  public Pose currentGoal(FieldFrame frame) {
    return goal.apply(frame);
  }
}
