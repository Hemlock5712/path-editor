package frc.team5712.path.wpilib;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.follow.PathFollower;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

/**
 * Commands v2 adapter, the same follower as {@link FollowPathCommand}. Events schedule their
 * commands on the {@link CommandScheduler}, independently of this one.
 */
public final class FollowPathCommandV2 extends Command {
  private final PathFollower follower;
  private final Supplier<Pose2d> pose;
  private final Supplier<ChassisVelocities> fieldVelocity;
  private final Consumer<ChassisVelocities> fieldOutput;
  private final FieldFrame redFrame;
  private final PathTelemetry telemetry;
  private final Map<String, Command> eventCommands = new HashMap<>();
  private final List<String> fired = new ArrayList<>();
  private Supplier<FieldSpeeds> handedCommand;
  private DoubleSupplier clock = Timer::getTimestamp;
  private Supplier<FieldFrame> frameSource;
  private PathLibrary library;
  private double lastStep = Double.NEGATIVE_INFINITY;
  /** Steps closer together than this are in the same robot loop (it runs every 20 ms). */
  private static final double SAME_LOOP_SECONDS = 0.005;

  /** The same, named by the path's generated name ({@code Paths.X}). */
  public FollowPathCommandV2(
      PathId path,
      Subsystem drive,
      PathFollower follower,
      Supplier<Pose2d> pose,
      Supplier<ChassisVelocities> fieldVelocity,
      Consumer<ChassisVelocities> fieldOutput,
      FieldFrame redFrame) {
    this(path.fileName(), drive, follower, pose, fieldVelocity, fieldOutput, redFrame);
  }

  public FollowPathCommandV2(
      String name,
      Subsystem drive,
      PathFollower follower,
      Supplier<Pose2d> pose,
      Supplier<ChassisVelocities> fieldVelocity,
      Consumer<ChassisVelocities> fieldOutput,
      FieldFrame redFrame) {
    this.follower = follower;
    this.pose = pose;
    this.fieldVelocity = fieldVelocity;
    this.fieldOutput = fieldOutput;
    this.redFrame = redFrame;
    this.telemetry = new PathTelemetry(name);
    setName(name);
    addRequirements(drive);
    follower.onEvent(fired::add);
  }

  /** {@link #onEvent(String, Command)} by the event's generated name ({@code Events.X}). */
  public FollowPathCommandV2 onEvent(EventId event, Command command) {
    return onEvent(event.eventName(), command);
  }

  /** Schedules {@code command} when the path event {@code eventName} fires. */
  public FollowPathCommandV2 onEvent(String eventName, Command command) {
    eventCommands.put(eventName, command);
    return this;
  }

  /**
   * Schedules the commands bound in {@code library} ({@link PathLibrary#bindEvent}) when their events
   * fire. A command given to {@link #onEvent} for the same name wins.
   */
  public FollowPathCommandV2 withEvents(PathLibrary library) {
    this.library = library;
    return this;
  }

  /** Replaces the clock (default {@code Timer.getTimestamp()}), e.g. for tests or replay. */
  public FollowPathCommandV2 withClock(DoubleSupplier seconds) {
    this.clock = seconds;
    return this;
  }

  /** Replaces the alliance lookup (default: the driver station, read when the path starts). */
  public FollowPathCommandV2 withFrame(Supplier<FieldFrame> frame) {
    this.frameSource = frame;
    return this;
  }

  private FieldFrame startFrame() {
    return frameSource == null ? Conversions.allianceFrame(redFrame) : frameSource.get();
  }

  /** Starts from the previous path's last command instead of the measured velocity (rolling ends). */
  public FollowPathCommandV2 continuingFrom(FollowPathCommandV2 previous) {
    this.handedCommand = previous.follower::endCommand;
    return this;
  }

  /** Starts from {@code command} (read at the start) instead of the measured velocity, e.g. an on-the-fly path's end. */
  public FollowPathCommandV2 continuingFrom(Supplier<FieldSpeeds> command) {
    this.handedCommand = command;
    return this;
  }

  public PathFollower follower() {
    return follower;
  }

  /**
   * Starts the follower and sends its first command at once, as the v3 command does. Commands v2
   * calls {@code execute()} only from the next loop, so a path chained after a rolling end (or started
   * by a trigger) would otherwise coast a loop on the old command.
   */
  @Override
  public void initialize() {
    FieldSpeeds measured = Conversions.toFieldSpeeds(fieldVelocity.get());
    double now = clock.getAsDouble();
    follower.start(now, Conversions.toPose(pose.get()), measured,
        startFrame(), handedCommand == null ? measured : handedCommand.get());
    telemetry.begin(follower);
    step(now);
  }

  @Override
  public void execute() {
    double now = clock.getAsDouble();
    if (now - lastStep > SAME_LOOP_SECONDS) { // not again in the loop initialize() already stepped
      step(now);
    }
  }

  private void step(double now) {
    lastStep = now;
    FieldSpeeds out = follower.step(now, Conversions.toPose(pose.get()),
        Conversions.toFieldSpeeds(fieldVelocity.get()));
    fieldOutput.accept(Conversions.toChassisVelocities(out));
    telemetry.log(follower);
    for (String event : fired) {
      telemetry.event(event);
      Command c = eventCommands.get(event);
      if (c == null && library != null) {
        c = library.eventCommandV2(event).orElse(null);
      }
      if (c != null) {
        CommandScheduler.getInstance().schedule(c);
      }
    }
    fired.clear();
  }

  @Override
  public boolean isFinished() {
    return follower.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      follower.cancel();
    }
    if (interrupted || !follower.rollingEnd()) {
      fieldOutput.accept(new ChassisVelocities());
    }
    telemetry.end();
  }
}
