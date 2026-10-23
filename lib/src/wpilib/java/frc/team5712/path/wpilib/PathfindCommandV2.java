package frc.team5712.path.wpilib;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Path;
import frc.team5712.path.plan.OnTheFly;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

/**
 * Commands v2: the same on-the-fly path as {@link PathfindCommand} (built by {@link Pathfinding}). Plans and
 * sends its first command in {@code initialize()}, as {@link FollowPathCommandV2} does.
 */
public final class PathfindCommandV2 extends Command {
  private final Pathfinding ctx;
  private final Pathfinding.Goal goal;
  private final Path handOver;
  private final PathTelemetry telemetry;
  private FollowPathCommandV2 next;
  private double replanAbove = OnTheFly.DEFAULT_REPLAN_METERS;
  private Supplier<Optional<Pose2d>> movingGoal;
  private DoubleSupplier clock = Timer::getTimestamp;
  private Supplier<FieldFrame> frameSource;
  private OnTheFly run;
  private double lastStep = Double.NEGATIVE_INFINITY;
  private static final double SAME_LOOP_SECONDS = 0.005;

  PathfindCommandV2(Pathfinding ctx, Subsystem drive, String name, Pathfinding.Goal goal, Path handOver) {
    this.ctx = ctx;
    this.goal = goal;
    this.handOver = handOver;
    this.telemetry = new PathTelemetry(name);
    setName(name);
    addRequirements(drive);
  }

  /** The path's command that follows this one ({@link Pathfinding#thenFollowV2}): it gets the same clock and frame. */
  void handsOverTo(FollowPathCommandV2 follow) {
    this.next = follow;
  }

  public PathfindCommandV2 replanAbove(double meters) {
    this.replanAbove = meters;
    return this;
  }

  public PathfindCommandV2 withMovingGoal(Supplier<Optional<Pose2d>> realGoal) {
    this.movingGoal = realGoal;
    return this;
  }

  public PathfindCommandV2 withClock(DoubleSupplier seconds) {
    this.clock = seconds;
    if (next != null) {
      next.withClock(seconds);
    }
    return this;
  }

  public PathfindCommandV2 withFrame(Supplier<FieldFrame> frame) {
    this.frameSource = frame;
    if (next != null) {
      next.withFrame(frame);
    }
    return this;
  }

  public void replan() {
    if (run != null) {
      run.requestReplan();
    }
  }

  public OnTheFly onTheFly() {
    return run;
  }

  /** The speed to hand to the next path. */
  public FieldSpeeds endCommand() {
    return run == null ? new FieldSpeeds(0, 0, 0) : run.endCommand();
  }

  @Override
  public void initialize() {
    FieldFrame frame = frameSource == null ? Conversions.allianceFrame(ctx.redFrame) : frameSource.get();
    run = new OnTheFly(ctx.library.pathfinder(), ctx.config, () -> goal.apply(frame))
        .preferring(ctx.library.widePathfinder()).replanAbove(replanAbove)
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
    double now = clock.getAsDouble();
    if (run.start(now, Conversions.toPose(ctx.pose.get()), measured())) {
      step(now);
    }
  }

  @Override
  public void execute() {
    double now = clock.getAsDouble();
    if (now - lastStep > SAME_LOOP_SECONDS) {
      step(now);
    }
  }

  private void step(double now) {
    lastStep = now;
    FieldSpeeds out = run.step(now, Conversions.toPose(ctx.pose.get()), measured());
    ctx.fieldOutput.accept(Conversions.toChassisVelocities(out));
    if (run.follower() != null) {
      telemetry.log(run.follower());
    }
  }

  private FieldSpeeds measured() {
    return Conversions.toFieldSpeeds(ctx.fieldVelocity.get());
  }

  @Override
  public boolean isFinished() {
    return run.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      run.cancel();
    }
    if (interrupted || !run.rollingEnd()) {
      ctx.fieldOutput.accept(new ChassisVelocities());
    }
    telemetry.end();
  }
}
