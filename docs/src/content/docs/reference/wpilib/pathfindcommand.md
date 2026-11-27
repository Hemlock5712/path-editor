---
title: PathfindCommand
description: A command that plans a path around the obstacles when it starts, follows it, and re-plans when needed.
sidebar:
  order: 3
---

`PathfindCommand` plans a path from where the robot is to a goal, around the obstacles in `field.json`, follows it, and plans again if the robot is knocked off it or the goal moves far.

You do not construct one: `otf.to`, `otf.toPoint`, `otf.toTarget` and `otf.thenFollow` on [`Pathfinding`](/reference/wpilib/pathfinding/) each return one. [Pathfinding](/guides/pathfinding/) is the guide.

## Example

```java
// A looser finish for a spot that does not need to be exact
Command toDepot = otf.toPoint(drive, Points.DEPOT, Rotation2d.PI)
    .withTolerance(0.05, Rotation2d.fromDegrees(2));

// vision.pieceIfSeen() is your own code. It returns an Optional<Pose2d> in the
// real field frame: the piece's pose when the camera sees it, or empty.
Command toPiece = otf.to(drive, new Pose2d(2.0, 1.5, Rotation2d.ZERO))
    .withMovingGoal(() -> vision.pieceIfSeen(), 0.5);   // averaged over 0.5 s
```

## Settings

Each returns the same command, so they chain. Set them when you build the command.

| Method | Default | What it does |
| --- | --- | --- |
| `replanAbove(double meters)` | `0.2` | Plans again when the robot is farther than this from its path, in meters, at most once every 0.5 s. Pass `Double.POSITIVE_INFINITY` to re-plan only when asked. |
| `withMovingGoal(Supplier<Optional<Pose2d>> realGoal)` | none | The end of the path follows this goal. Real field frame, not flipped on red, read every loop. An empty value keeps the last goal. For a goal that is already filtered, such as a [`TagGoal`](/reference/wpilib/taggoal/). |
| `withMovingGoal(Supplier<Optional<Pose2d>> realGoal, double smoothingSeconds)` | none | The same, with the estimates averaged over `smoothingSeconds`. For a goal that arrives raw from a camera. The source suggests 0.5 s to start. |
| `withTolerance(double meters, Rotation2d heading)` | 0.02 m and 1° | What "arrived" means: within `meters` of the goal and `heading` of its heading, slower than 0.05 m/s, for 0.1 s. |
| `finishWhen(BooleanSupplier ready)` | none | The robot drives to the goal and holds it, and the command does not finish until `ready` returns `true`. Ignored when the command ends moving, on a `thenFollow` to a path with a start speed above zero. |
| `withFootprint(Supplier<String> footprintName)` | `"bumpers"` | Plans with this [footprint](/concepts/footprints/) from `robot.json`. Read once, when the command starts. An unknown name throws `NoSuchElementException` then. |
| `withFrame(Supplier<FieldFrame> frame)` | the driver station's alliance | Replaces the alliance lookup. Read once, when the command starts. |
| `withClock(DoubleSupplier seconds)` | `Timer.getTimestamp()` | Replaces the clock, in seconds. |

## While it runs

| Method | Returns | What it does |
| --- | --- | --- |
| `setTolerance(double meters, Rotation2d heading)` | nothing | The same as `withTolerance`, for a command you already hold in a variable. |
| `replan()` | nothing | Plans again on the next loop, from wherever the robot is. Does nothing if the command has never started. |
| `onTheFly()` | [`OnTheFly`](/reference/plan/onthefly/) | The current run, with its last plan, re-plan count and failure reason. `null` before the first start. |
| `currentGoal(FieldFrame frame)` | `Pose` | The pose the command is heading for, in the real field frame, given the frame the robot runs in. For tests and dashboards. |

## Command methods

The scheduler calls these. Robot code does not.

| Method | What it does |
| --- | --- |
| `name()` | `"Pathfind"` from `to` and `toTarget`, `"Pathfind <point>"` from `toPoint`, `"Pathfind to <path>"` from `thenFollow`. Also the telemetry table's name. |
| `requirements()` | The one `Mechanism` passed to the factory method. |
| `run(Coroutine)` | Reads the alliance, the footprint, the clock, the pose and the velocity, then plans. Each loop it re-plans if a rule says to, takes one follower step, drives and publishes telemetry. On arrival it sends zero velocity and ends. |
| `onCancel()` | Stops following, sends zero velocity and marks the path as no longer active in telemetry. After a `thenFollow` hand-over, the cancel goes to the path's follow command. |

## Good to know

- **No route does not throw.** The command sends zero velocity and ends at once, and a `thenFollow` command does not go on to its path. The reason is in `onTheFly().failure()` and the `PlanFailure` telemetry entry.
- **A moving goal bends the end first.** When the goal moves more than 2 cm or 2°, the last stretch is re-planned with no obstacles. Once it is more than 0.10 m from where the path was planned to, the whole path is planned again around the obstacles, at most once every 0.2 s. A goal the supplier already has at the start is where the first plan goes.
- **`thenFollow` hands over without a zero.** It waits one loop and runs the path's [`FollowPathCommand`](/reference/wpilib/followpathcommand/) inside itself, with its last commanded velocity. The drawn path keeps the tolerance in the factory's `FollowerConfig`.
- **Planning is the only path work after robot init.** It measured about 1 ms at the median on a desktop computer and has not been timed on a roboRIO.

:::caution
If `ready` never becomes `true`, the command never finishes and the drivetrain stays held at the goal. Give the command a timeout in your own code.
:::

:::note
Two commands built by `otf.to` are both named `"Pathfind"` and share one telemetry table, which does not say which of the two is running.
:::

## More

- [On-the-fly paths](/concepts/on-the-fly/) and [Moving goals](/concepts/moving-goals/): how it works.
- [`PathTelemetry`](/reference/wpilib/pathtelemetry/): every entry, including the planning figures.
- [`Pathfinding`](/reference/wpilib/pathfinding/): the factory that builds these.
