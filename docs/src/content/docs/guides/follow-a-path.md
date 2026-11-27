---
title: Follow a path
description: Build a command that follows a path drawn in the editor, and set its options.
sidebar:
  order: 1
---

A path drawn in the editor is followed by a `FollowPathCommand`. There are two ways to get one. The factory on
`Pathfinding` is the short way and suits nearly every case. The constructor is the long way, for when a path
needs its own settings.

## The short way

```java
import static frc.robot.PathEditorNames.*;

Command toReef = otf.follow(drive, Paths.TO_REEF);
```

`otf` is the `Pathfinding` object built in robot init, as in [The first path](/start/first-path/). `follow` gives
the command everything that object was built with: the drivetrain's pose, velocity and output, the follower
settings, the alliance flip, and the wheel forces if you turned them on. It also binds the path's events to the
commands registered on the library.

The String form does the same: `otf.follow(drive, "ToReef")`. An unknown name throws `NoSuchElementException`
when the line runs, in robot init, and the message lists the paths that do exist.

## What the command does

| When | What happens |
| --- | --- |
| Robot init, at `otf.follow(...)` | The follower is built. The path itself was planned when the library loaded. |
| The command starts | It reads the alliance and the clock. No path work. |
| Every loop | One follower step, the velocity to the drivetrain, telemetry, and any event that fired is forked as a child command. |
| It finishes | It sends a zero velocity, unless the path ends moving. |
| It is canceled | It sends a zero velocity and drops the events that have not fired. |

It finishes when the robot is within the position and heading tolerance of the end, measured slower than 5 cm/s,
and has stayed that way for 0.1 s. The defaults are 2 cm and 1°.

The robot does not have to start on the path. The follower finds the closest point and steers onto the line from
wherever the robot is. A robot that starts a long way off should use `otf.thenFollow`, which plans a route to the
path's start first. See [Pathfinding](/guides/pathfinding/).

## Options

Each of these returns the command, so they chain.

| Method | What it does | Guide |
| --- | --- | --- |
| `onEvent(Events.X, command)` | Runs `command` when this path fires that event. Wins over the library's binding for the same name. | [Events](/guides/events/) |
| `withEvents(paths)` | Uses the commands bound on the library. `otf.follow` already does this. | [Events](/guides/events/) |
| `withMovingGoal(supplier)` | Lets the end of the path follow a goal seen on the field. | [Moving goals](/guides/moving-goals/) |
| `finishWhen(condition)` | Holds the end until the condition is true. | [Moving goals](/guides/moving-goals/) |
| `continuingFrom(previous)` | Starts from the last path's final command, for paths that hand over at speed. | [Chaining paths](/guides/chaining-paths/) |
| `withWheelForces(robot, modules, drive)` | Sends each wheel's push with the velocity. | [Wheel forces](/guides/wheel-forces/) |
| `withFrame(supplier)` | Replaces the alliance lookup. | [Alliances](/guides/alliances/) |
| `withClock(seconds)` | Replaces the clock, for tests and replay. | |

:::note
WPILib's compiler check refuses a command that is returned and then thrown away. Keep what a `with...` method
returns: write `Command c = otf.follow(...).finishWhen(...)`, not the two as separate statements.
:::

## The long way

Build the follower yourself when one path needs different settings from the rest, for example a looser
tolerance on a path that only has to get near its end.

```java
Path toReef = paths.get(Paths.TO_REEF);

FollowerConfig loose = FollowerConfig.defaults(paths.robot()).toBuilder()
    .positionToleranceMeters(0.05)
    .headingToleranceRadians(Math.toRadians(3))
    .build();

Command go = new FollowPathCommand(
        Paths.TO_REEF,
        drive,
        new PathFollower(toReef, loose),
        () -> drive.getPose(),
        () -> drive.getFieldVelocity(),
        velocity -> drive.driveFieldVelocity(velocity),
        FieldFrame.RED_HALF_TURN)
    .withEvents(paths);
```

The constructor's arguments, in order:

| Argument | Meaning |
| --- | --- |
| `Paths.TO_REEF` | The path's name, used for the command's name and its telemetry table. A `String` works too. |
| `drive` | The drivetrain `Mechanism` the command requires. |
| `new PathFollower(path, config)` | The follower. Build it here, in robot init, never when the command starts. |
| pose | The measured field pose. |
| field velocity | The measured field-relative velocity. |
| output | Takes the field-relative velocity to drive at. |
| `FieldFrame.RED_HALF_TURN` | The flip used on the red alliance. |

A command built this way does not pick up the library's event bindings on its own. Add `.withEvents(paths)`.

## Use one command more than once

A `FollowPathCommand` can run again and again. Each start resets the follower. Build it once in robot init and
schedule the same object each time, so the follower is not rebuilt at go.

## Related

- [The follower](/concepts/the-follower/) explains what one step does.
- [FollowPathCommand](/reference/wpilib/followpathcommand/) lists every method.
- [Tuning](/guides/tuning/) covers the settings in `FollowerConfig`.
