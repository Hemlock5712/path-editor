---
title: FollowPathCommand
description: The Commands v3 command that follows one built path and forks its event commands.
sidebar:
  order: 4
---

`FollowPathCommand` drives the robot along one path: each loop it reads the pose and velocity, steps a [`PathFollower`](/reference/follow/pathfollower/), drives the drivetrain and starts the commands bound to the path's events.

Most robot code gets one from `otf.follow(drive, Paths.TO_REEF)` on [`Pathfinding`](/reference/wpilib/pathfinding/). Build one by hand for a follower with its own settings, or events that exist only in code. [Follow a path](/guides/follow-a-path/) is the guide.

## Example

```java
import static frc.robot.PathEditorNames.*;

// Robot constructor: one follower belongs to one command
PathFollower follower = new PathFollower(
    paths.get(Paths.TO_REEF), FollowerConfig.defaults(paths.robot()));

Command toReef = new FollowPathCommand(
        Paths.TO_REEF,                                    // names the command and its telemetry
        drive,                                            // the drivetrain it requires
        follower,                                         // chooses the path
        () -> drive.getPose(),                            // where the robot is
        () -> drive.getFieldVelocity(),                   // how fast, field-relative
        velocity -> drive.driveFieldVelocity(velocity),   // how to drive
        FieldFrame.RED_HALF_TURN)                         // the flip on red
    .withEvents(paths)                                    // events bound on the library
    .onEvent(Events.INTAKE_DOWN, intake.lower());         // an event for this path only
```

## Constructors

| Constructor | What it does |
| --- | --- |
| `FollowPathCommand(PathId path, drive, follower, pose, fieldVelocity, fieldOutput, redFrame)` | The command's name is the constant's file name. |
| `FollowPathCommand(String name, drive, follower, pose, fieldVelocity, fieldOutput, redFrame)` | The same, named by a string. |

| Argument | Type | Meaning |
| --- | --- | --- |
| `path` or `name` | [`PathId`](/reference/wpilib/name-ids/) or `String` | The command's name and its telemetry table's name. It does not choose the path: the follower does. |
| `drive` | `Mechanism` | The drivetrain mechanism this command requires. |
| `follower` | [`PathFollower`](/reference/follow/pathfollower/) | The follower for the path, built in robot init. One follower belongs to one command. |
| `pose` | `Supplier<Pose2d>` | The measured pose, real field frame, meters. Read at the start and every loop. |
| `fieldVelocity` | `Supplier<ChassisVelocities>` | The measured velocity, field-relative, real field frame, m/s and rad/s. Read at the start and every loop. |
| `fieldOutput` | `Consumer<ChassisVelocities>` | Takes the velocity to drive at, field-relative, real field frame. Called every loop, and with zero at a stop. |
| `redFrame` | [`FieldFrame`](/reference/core/fieldframe/) | The flip used on red: `RED_HALF_TURN` or `RED_MIRROR`. |

:::caution
WPILib 2027 alpha-7 still has the field origin in a corner, and the library uses the center. Until that changes, convert the pose, the velocity and the output on the way in and out. [The field frame](/concepts/field-frame/) has the conversion.
:::

## Events

| Method | What it does |
| --- | --- |
| `withEvents(PathLibrary library)` | When an event fires, starts the command the library has bound to that name with `bindEvent`. Looked up when the event fires, so a later `bindEvent` still counts. |
| `onEvent(EventId event, Command command)` | Starts `command` when this event fires on this path. Wins over the library's command for the same name. |
| `onEvent(String eventName, Command command)` | The same, by the event's name. |

## Moving goals and the finish

These set the follower, not the command, so they apply to every run.

| Method | What it does |
| --- | --- |
| `withMovingGoal(Supplier<Optional<Pose2d>> realGoal)` | The end of the path follows this goal: a pose in the real field frame, not flipped on red, read every loop. An empty value keeps the last goal. Use it for a goal that is already filtered, such as a [`TagGoal`](/reference/wpilib/taggoal/). |
| `withMovingGoal(Supplier<Optional<Pose2d>> realGoal, double smoothingSeconds)` | The same, with the estimates averaged over `smoothingSeconds`, for estimates straight from a camera. The source suggests 0.5 s to start. |
| `finishWhen(BooleanSupplier ready)` | The robot drives to the end and holds it, and the command does not finish until `ready` returns `true`. A path with a rolling end ignores it. |

## Other settings

| Method | Default | What it does |
| --- | --- | --- |
| `withWheelForces(RobotConfig robot, Translation2d[] moduleLocations, WheelForceDrive drive)` | velocity only | Each loop `drive` gets the velocity and each wheel's force in newtons, field-relative, and `fieldOutput` is no longer called. `robot` is `paths.robot()`. `moduleLocations` are in meters, in the drivetrain's module order. Throws `IllegalArgumentException` if the array is empty or every module is at the same place. |
| `continuingFrom(FollowPathCommand previous)` | the measured velocity | Starts from the previous path's last command. For a chain of paths that do not stop between them. |
| `continuingFrom(Supplier<FieldSpeeds> command)` | the measured velocity | Starts from this velocity, read at the start. Field-relative, real field frame. |
| `withFrame(Supplier<FieldFrame> frame)` | the driver station's alliance | Replaces the alliance lookup. Read once at each start. |
| `withClock(DoubleSupplier seconds)` | `Timer.getTimestamp()` | Replaces the clock, in seconds. For tests and replay. |
| `follower()` | | Returns the `PathFollower` this command runs. |

## Command methods

The scheduler calls `run` and `onCancel`. Robot code does not.

| Method | What it does |
| --- | --- |
| `name()` | The name given to the constructor. |
| `requirements()` | The one `Mechanism` given to the constructor. |
| `run` | Starts the follower, then each loop reads the pose and velocity, takes one follower step, drives, publishes telemetry and forks the command for each event that fired. Each start resets the follower, so the command can run again after it finishes or is canceled. |
| `onCancel` | Stops the follower and drops events that have not fired. Sends zero velocity, with zero wheel forces if those are in use, and marks the path as no longer active in telemetry. |

## Good to know

- **The start does no path work.** It reads the velocity, the alliance, the clock and the pose, and an unknown alliance counts as blue. The path stays in the blue frame and nothing is rebuilt for red.
- **It finishes** inside the follower's tolerances, 2 cm and 1° by default, slower than 0.05 m/s, for 0.1 s, then sends zero velocity. Events that have not fired fire at the finish.
- **A rolling end sends nothing more.** A path whose end speed is above zero finishes as the robot passes the end, and the last velocity is left for the next command.
- **A moving goal knows no obstacles.** When the goal moves more than 2 cm or 2° the rest of the path is planned again. For a goal that might move around an obstacle, use an on-the-fly command with `withMovingGoal`.

:::caution
A forked event command ends when the path command ends, unless it has finished first. Start a long action near the end of a path from your own command sequence instead. Give a command that uses `finishWhen` a timeout too, in case the condition never comes true.
:::

## More

- [Events](/guides/events/), [Moving goals](/guides/moving-goals/), [Chaining paths](/guides/chaining-paths/) and [Wheel forces](/guides/wheel-forces/): the guides for each option.
- [The follower](/concepts/the-follower/) and [Path events](/concepts/path-events/): how it works.
- [`PathTelemetry`](/reference/wpilib/pathtelemetry/): the entries under `/Telemetry/Path/<name>/`.
