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
    this.telemetry = new PathTelemetry("Path/" + name);
    setName(name);
    addRequirements(drive);
    follower.onEvent(fired::add);
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

  public PathFollower follower() {
    return follower;
  }

  @Override
  public void initialize() {
    FieldSpeeds measured = Conversions.toFieldSpeeds(fieldVelocity.get());
    follower.start(clock.getAsDouble(), Conversions.toPose(pose.get()), measured,
        startFrame(), handedCommand == null ? measured : handedCommand.get());
  }

  @Override
  public void execute() {
    FieldSpeeds out = follower.step(clock.getAsDouble(), Conversions.toPose(pose.get()),
        Conversions.toFieldSpeeds(fieldVelocity.get()));
    fieldOutput.accept(Conversions.toChassisVelocities(out));
    telemetry.log(follower);
    for (String event : fired) {
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
  }
}
