---
title: FollowPathCommand
description: The Commands v3 command that follows one built path and forks its event commands.
sidebar:
  order: 4
---

`FollowPathCommand` drives the robot along one path. It wraps a [`PathFollower`](/reference/follow/pathfollower/) that was built in robot init, reads the robot's pose and velocity each loop, and sends the follower's answer to the drivetrain. When the path fires an event, the command starts the command bound to it.

Most robot code gets one from `otf.follow(drive, Paths.TO_REEF)` on [`Pathfinding`](/reference/wpilib/pathfinding/), which fills in the constructor for you. Build one by hand when you want a follower with its own settings, or events that exist only in code. [Follow a path](/guides/follow-a-path/) is the guide.

## Example

```java
import static frc.robot.PathEditorNames.*;

// Robot constructor
PathFollower follower = new PathFollower(
    paths.get(Paths.TO_REEF), FollowerConfig.defaults(paths.robot()));

Command toReef = new FollowPathCommand(
        Paths.TO_REEF,
        drive,
        follower,
        () -> drive.getPose(),
        () -> drive.getFieldVelocity(),
        velocity -> drive.driveFieldVelocity(velocity),
        FieldFrame.RED_HALF_TURN)
    .withEvents(paths)
    .onEvent(Events.INTAKE_DOWN, intake.lower());
```

`getPose`, `getFieldVelocity` and `driveFieldVelocity` belong to your drivetrain class, not to the library. [The drivetrain](/start/drivetrain/) lists what is needed. `intake.lower()` is a command from your own code.

## Constructors

```java
new FollowPathCommand(PathId path, drive, follower, pose, fieldVelocity, fieldOutput, redFrame)
new FollowPathCommand(String name, drive, follower, pose, fieldVelocity, fieldOutput, redFrame)
```

The two differ only in the first argument. The `PathId` form uses the constant's file name as the command's name.

| Argument | Type | Meaning |
| --- | --- | --- |
| `path` or `name` | [`PathId`](/reference/wpilib/name-ids/) or `String` | The command's name. It is also the telemetry table's name. It does not choose the path: the follower does. |
| `drive` | `Mechanism` | The drivetrain mechanism this command requires. |
| `follower` | [`PathFollower`](/reference/follow/pathfollower/) | The follower for the path, built in robot init. One follower belongs to one command. |
| `pose` | `Supplier<Pose2d>` | The measured pose, real field frame, meters. Read at the start and every loop. |
| `fieldVelocity` | `Supplier<ChassisVelocities>` | The measured velocity, field-relative, real field frame, m/s and rad/s. Read at the start and every loop. |
| `fieldOutput` | `Consumer<ChassisVelocities>` | Takes the velocity to drive at, field-relative, real field frame. Called every loop, and with zero at a stop. |
| `redFrame` | [`FieldFrame`](/reference/core/fieldframe/) | The flip used on red: `RED_HALF_TURN` or `RED_MIRROR`. |

The path itself stays in the blue frame, with the origin at the field center and +X toward the blue alliance wall. On red, each loop converts the measured pose and velocity into the blue frame, follows the blue path, and converts the output back. Nothing is rebuilt. Your suppliers and your output always work in the real field frame, the one the robot's pose is in.

:::caution
WPILib 2027 alpha-7 still has the field origin in a corner, and the library uses the center. Until that changes, the pose, the velocity and the output must be converted on the way in and out. [The field frame](/concepts/field-frame/) has the conversion.
:::

## Events

| Method | Meaning |
| --- | --- |
| `withEvents(PathLibrary library)` | When an event fires, start the command the library has bound to that name with `bindEvent`. |
| `onEvent(EventId event, Command command)` | Start `command` when this event fires on this path. |
| `onEvent(String eventName, Command command)` | The same, by the event's name. |

For one event name, a command given to `onEvent` wins over the library's. An event with no command anywhere still fires and still shows in telemetry. It just starts nothing.

The bindings are looked up when the event fires, so a `bindEvent` made after the command was built still counts. Event commands are forked: they run as children of the path command. See [Events](/guides/events/) and [Path events](/concepts/path-events/).

:::caution
A forked event command ends when the path command ends, unless it has finished first. A long action started by an event near the end of a path is cut short when the path finishes. Start such an action from your own command sequence instead.
:::

## Moving goals and the finish

| Method | Meaning |
| --- | --- |
| `withMovingGoal(Supplier<Optional<Pose2d>> realGoal)` | The end of the path follows this goal. |
| `withMovingGoal(Supplier<Optional<Pose2d>> realGoal, double smoothingSeconds)` | The same, with the estimates averaged over `smoothingSeconds`. |
| `finishWhen(BooleanSupplier ready)` | The command does not finish until `ready` returns `true`. |

The moving goal is a pose in the real field frame, like the robot's own pose. It is not flipped on red. An empty value keeps the last goal. The supplier is read every loop.

When the goal moves more than 2 cm or 2°, the rest of the path is planned again from where the robot is on the path to the goal, joining smoothly. In the last 0.3 m the stopping law drives straight at the latest goal. The re-planned stretch knows nothing about obstacles. For a goal that might move around one, use an on-the-fly command with `withMovingGoal` instead, which plans around them.

Use the two-argument form for estimates straight from a camera. The source suggests 0.5 s to start. One estimate more than 25 cm or 20° from the average is ignored, and three in a row start a new average. Leave smoothing off for a goal that is already filtered, such as a [`TagGoal`](/reference/wpilib/taggoal/). See [Moving goals](/guides/moving-goals/).

With `finishWhen`, the robot still drives to the end and holds it, and the command waits there until `ready` is `true`. A path with a rolling end ignores it. Give the command a timeout in your own code in case the condition never comes true.

All three methods set the follower, not the command, so they apply to every run of the command.

## Other settings

| Method | Default | Meaning |
| --- | --- | --- |
| `withWheelForces(RobotConfig robot, Translation2d[] moduleLocations, WheelForceDrive drive)` | velocity only | Send the wheel-force [feed-forward](/concepts/feed-forward/) with the velocity. |
| `continuingFrom(FollowPathCommand previous)` | the measured velocity | Start from the previous path's last command. For a chain of paths that do not stop between them. |
| `continuingFrom(Supplier<FieldSpeeds> command)` | the measured velocity | Start from this velocity, read at the start. Field-relative, real field frame. |
| `withFrame(Supplier<FieldFrame> frame)` | the driver station's alliance | Replaces the alliance lookup. Read once at each start. |
| `withClock(DoubleSupplier seconds)` | `Timer.getTimestamp()` | Replaces the clock, in seconds. For tests and replay. |
| `follower()` | | Returns the `PathFollower` this command runs. |

### withWheelForces

Each loop, `drive` receives the velocity and the force every wheel needs for the acceleration the follower just asked for, in newtons, field-relative. The constructor's `fieldOutput` is no longer called. `robot` is `paths.robot()`, which supplies the mass and the moment of inertia. `moduleLocations` are the modules' positions on the robot in meters, in the drivetrain's module order. It throws `IllegalArgumentException` if the array is empty or every module is at the same place. See [`WheelForceDrive`](/reference/wpilib/wheelforcedrive/) and [Wheel forces](/guides/wheel-forces/).

### continuingFrom

A path whose end speed is above zero has a **rolling end**: it finishes as the robot passes the end, still moving. The next path should start from the velocity the first one was commanding, not from the measured velocity, which lags it. `second.continuingFrom(first)` does that. See [Chaining paths](/guides/chaining-paths/).

## What the command does

The scheduler calls `run` and `onCancel`. Robot code does not.

| Method | Value |
| --- | --- |
| `name()` | The name given to the constructor. |
| `requirements()` | The one `Mechanism` given to the constructor. |

**Starting.** The command reads the measured velocity, the alliance, the clock and the measured pose, and starts the follower. The alliance comes from the driver station at this moment, and an unknown alliance counts as blue. The start does no path work: nothing is built, flipped or copied. The planned path is then published to telemetry once.

**Each loop.** The first loop runs in the same scheduler cycle as the start. Each loop the command:

1. reads the pose and the velocity,
2. takes one follower step,
3. sends the output to the drivetrain: a field-relative velocity, with wheel forces if `withWheelForces` was called,
4. publishes telemetry,
5. forks the command for each event that fired during the step.

**Finishing.** A path that ends at rest finishes when the robot has been inside the follower's tolerances, 2 cm and 1° by default, measured slower than 0.05 m/s, for 0.1 s. Any events that have not fired by then fire at the finish. The command then sends zero velocity. A path with a rolling end finishes as it passes its end and sends nothing more, so the last velocity is left for the next command.

**Canceling.** The follower stops, and events that have not fired are dropped. The command sends zero velocity, with zero wheel forces if those are in use, and marks the path as no longer active in telemetry.

The command can run again after it finishes or is canceled. Each start resets the follower. Telemetry goes out under `/Telemetry/Path/<name>/`, and [`PathTelemetry`](/reference/wpilib/pathtelemetry/) lists the entries.
