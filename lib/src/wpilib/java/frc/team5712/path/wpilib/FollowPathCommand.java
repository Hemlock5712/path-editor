package frc.team5712.path.wpilib;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.follow.PathFollower;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

/**
 * Commands v3 adapter: follows a path with a {@link PathFollower} built in robot init.
 *
 * <pre>
 * Path toReef = paths.get("ToReef");                      // robot init
 * Command go = new FollowPathCommand("ToReef", drivetrain,
 *     new PathFollower(toReef, FollowerConfig.defaults(paths.robot())),
 *     drivetrain::getPose, drivetrain::getFieldVelocity, drivetrain::driveFieldVelocity,
 *     FieldFrame.RED_HALF_TURN)
 *   .onEvent("intakeDown", intake.lower());
 * </pre>
 *
 * Starting it reads the alliance and the clock and does no path work. Each loop: one follower step,
 * the output to the drivetrain, telemetry, and any events that fired are forked (they run as children
 * of this command, so they end with it unless they finish first). Finishing sends zero, except on a
 * rolling end, where the last velocity is left for the next command ({@link #continuingFrom}).
 * Cancelling sends zero and cancels the events that haven't fired.
 */
public final class FollowPathCommand implements Command {
  private final String name;
  private final Mechanism drive;
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

  /**
   * @param drive the drivetrain mechanism this command requires
   * @param pose the measured field pose
   * @param fieldVelocity the measured field-relative velocity
   * @param fieldOutput takes the field-relative velocity to drive at
   * @param redFrame the flip used on the red alliance
   */
  public FollowPathCommand(
      String name,
      Mechanism drive,
      PathFollower follower,
      Supplier<Pose2d> pose,
      Supplier<ChassisVelocities> fieldVelocity,
      Consumer<ChassisVelocities> fieldOutput,
      FieldFrame redFrame) {
    this.name = name;
    this.drive = drive;
    this.follower = follower;
    this.pose = pose;
    this.fieldVelocity = fieldVelocity;
    this.fieldOutput = fieldOutput;
    this.redFrame = redFrame;
    this.telemetry = new PathTelemetry(name);
    follower.onEvent(fired::add);
  }

  /** Forks {@code command} when the path event {@code eventName} fires. */
  public FollowPathCommand onEvent(String eventName, Command command) {
    eventCommands.put(eventName, command);
    return this;
  }

  /**
   * Forks the commands bound in {@code library} ({@link PathLibrary#bindEvent}) when their events
   * fire. A command given to {@link #onEvent} for the same name wins.
   */
  public FollowPathCommand withEvents(PathLibrary library) {
    this.library = library;
    return this;
  }

  /** Replaces the clock (default {@code Timer.getTimestamp()}), e.g. for tests or replay. */
  public FollowPathCommand withClock(DoubleSupplier seconds) {
    this.clock = seconds;
    return this;
  }

  /** Replaces the alliance lookup (default: the driver station, read when the path starts). */
  public FollowPathCommand withFrame(Supplier<FieldFrame> frame) {
    this.frameSource = frame;
    return this;
  }

  private FieldFrame startFrame() {
    return frameSource == null ? Conversions.allianceFrame(redFrame) : frameSource.get();
  }

  /** Starts from the previous path's last command instead of the measured velocity (rolling ends). */
  public FollowPathCommand continuingFrom(FollowPathCommand previous) {
    this.handedCommand = previous.follower::endCommand;
    return this;
  }

  public PathFollower follower() {
    return follower;
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
    FieldSpeeds measured = Conversions.toFieldSpeeds(fieldVelocity.get());
    FieldFrame frame = startFrame();
    follower.start(clock.getAsDouble(), Conversions.toPose(pose.get()), measured, frame,
        handedCommand == null ? measured : handedCommand.get());
    telemetry.begin(follower);
    while (true) {
      execute(coroutine);
      if (follower.isFinished()) {
        break;
      }
      coroutine.yield();
    }
    if (!follower.rollingEnd()) {
      fieldOutput.accept(new ChassisVelocities());
    }
    telemetry.end();
  }

  private void execute(Coroutine coroutine) {
    FieldSpeeds out = follower.step(clock.getAsDouble(), Conversions.toPose(pose.get()),
        Conversions.toFieldSpeeds(fieldVelocity.get()));
    fieldOutput.accept(Conversions.toChassisVelocities(out));
    telemetry.log(follower);
    for (String event : fired) {
      telemetry.event(event);
      Command c = eventCommands.get(event);
      if (c == null && library != null) {
        c = library.eventCommand(event).orElse(null);
      }
      if (c != null) {
        coroutine.fork(c);
      }
    }
    fired.clear();
  }

  @Override
  public void onCancel() {
    follower.cancel();
    fieldOutput.accept(new ChassisVelocities());
    telemetry.end();
  }
}
