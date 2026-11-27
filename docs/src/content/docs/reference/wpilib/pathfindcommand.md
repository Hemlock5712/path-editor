---
title: PathfindCommand
description: A command that plans a path around the obstacles when it starts, follows it, and re-plans when needed.
sidebar:
  order: 3
---

`PathfindCommand` is an on-the-fly path as a Commands v3 command. When it starts it plans a path from where the robot is to its goal, around the obstacles in `field.json`. Then it follows that path with the same follower a drawn path uses. If the robot is knocked off the path, or the goal moves far, it plans again.

You do not construct one. [`Pathfinding`](/reference/wpilib/pathfinding/) builds them: `otf.to`, `otf.toPoint`, `otf.toTarget` and `otf.thenFollow` each return a `PathfindCommand`. The methods on this page adjust the one command they are called on.

## Example

```java
Command toDepot = otf.toPoint(drive, Points.DEPOT, Rotation2d.PI)
    .withTolerance(0.05, Rotation2d.fromDegrees(2));

Command toPiece = otf.to(drive, new Pose2d(2.0, 1.5, Rotation2d.ZERO))
    .withMovingGoal(() -> vision.pieceIfSeen(), 0.5);
```

`vision.pieceIfSeen()` is your own code. It returns an `Optional<Pose2d>` in the real field frame: the piece's pose when the camera sees it, or empty when it does not. `otf` and `drive` are set up as on the [`Pathfinding`](/reference/wpilib/pathfinding/) page.

## Settings

Each returns the same command, so they chain. Set them when you build the command.

| Method | Default | Meaning |
| --- | --- | --- |
| `replanAbove(double meters)` | `0.2` | Plan again when the robot is farther than this from its path, in meters. Pass `Double.POSITIVE_INFINITY` to re-plan only when asked. |
| `withMovingGoal(Supplier<Optional<Pose2d>> realGoal)` | none | The end of the path follows this goal. Real field frame, not flipped on red. An empty value keeps the last goal. |
| `withMovingGoal(Supplier<Optional<Pose2d>> realGoal, double smoothingSeconds)` | none | The same, with the estimates averaged over `smoothingSeconds`. For a goal that arrives raw from a camera. |
| `withTolerance(double meters, Rotation2d heading)` | 0.02 m and 1° | What "arrived" means: this close to the goal, this well turned, and stopped. |
| `finishWhen(BooleanSupplier ready)` | none | The command does not finish until `ready` returns `true`. |
| `withFootprint(Supplier<String> footprintName)` | `"bumpers"` | Plan with this [footprint](/concepts/footprints/) from `robot.json`. |
| `withFrame(Supplier<FieldFrame> frame)` | the driver station's alliance | Replaces the alliance lookup. |
| `withClock(DoubleSupplier seconds)` | `Timer.getTimestamp()` | Replaces the clock, in seconds. |

### replanAbove

A shove from another robot can leave the robot beside its path with an obstacle between them. Above this distance the command plans a new path from where the robot is, at the speed it is going, at most once every 0.5 s.

### withMovingGoal

The supplier is read every loop. Small moves bend the end of the current path: when the goal moves more than 2 cm or 2°, the follower re-plans the last stretch to it. That stretch knows no obstacles. Once the goal is more than 0.10 m from where the path was planned to, the whole path is planned again around the obstacles, at most once every 0.2 s.

If the supplier already has a value when the command starts, the first plan goes to that value rather than to the command's own goal.

Use the one-argument form for a goal that is already filtered, such as a [`TagGoal`](/reference/wpilib/taggoal/). Use the two-argument form for raw camera estimates. The source suggests 0.5 s as a starting value. With averaging on, one estimate more than 25 cm or 20° from the average is ignored, and three in a row start a new average. See [Moving goals](/concepts/moving-goals/).

### withTolerance and setTolerance

The command finishes once the robot is within `meters` of the goal and within `heading` of the goal's heading, measured slower than 0.05 m/s, for 0.1 s. Loosen it for a spot that does not need to be exact, or for a robot that cannot hold 2 cm.

The tolerance applies to the on-the-fly part. On a `thenFollow` command, the drawn path that follows keeps the tolerance in the factory's `FollowerConfig`.

`setTolerance(double meters, Rotation2d heading)` does the same and returns nothing. Use it to change a command you already hold in a variable.

### finishWhen

The robot still drives to the goal and holds it. The command just does not end until `ready` is `true`. Use it for a goal that starts as a guess, such as a field-map position that a camera will refine. It is ignored when the command ends moving, which happens on a `thenFollow` to a path with a start speed above zero.

:::caution
If `ready` never becomes `true`, the command never finishes and the drivetrain stays held at the goal. Give the command a timeout in your own code.
:::

### withFootprint, withFrame, withClock

These work as the factory's methods of the same names do, for this one command. The footprint supplier and the frame supplier are each read once, when the command starts. An unknown footprint name throws `NoSuchElementException` at that point.

## While it runs

| Method | Returns | Meaning |
| --- | --- | --- |
| `replan()` | nothing | Plans again on the next loop, from wherever the robot is. Does nothing if the command has never started. |
| `onTheFly()` | [`OnTheFly`](/reference/plan/onthefly/) | The current run, with its last plan, re-plan count and failure reason. `null` before the first start. |
| `currentGoal(FieldFrame frame)` | `Pose` | The pose the command is heading for, in the real field frame, given the frame the robot runs in. For tests and dashboards. |

## Command behavior

These are the Commands v3 `Command` methods. The scheduler calls them. Robot code does not.

| Method | Value |
| --- | --- |
| `name()` | `"Pathfind"` from `to` and `toTarget`, `"Pathfind <point>"` from `toPoint`, `"Pathfind to <path>"` from `thenFollow`. |
| `requirements()` | The one `Mechanism` passed to the factory method. |
| `run(Coroutine)` | The steps below. |
| `onCancel()` | Described below. |

**Starting.** The command reads the alliance, the footprint, the clock, the measured pose and the measured velocity. It plans. Planning is the only path work that happens after robot init. It measured about 1 ms at the median on a desktop computer and has not been timed on a roboRIO. A start or a goal that lies inside a grown obstacle is routed through the nearest free point.

**Each loop.** It reads the pose and velocity, re-plans if one of the rules above says to, takes one follower step, sends the result to the drivetrain, and publishes telemetry. The output is a field-relative velocity in the real field frame, plus wheel forces if the factory was given `withWheelForces`.

**Finishing.** Once the robot has arrived, the command sends zero velocity and ends. A `thenFollow` command instead waits one loop and runs the path's [`FollowPathCommand`](/reference/wpilib/followpathcommand/) inside itself, handing over its last commanded velocity. No zero is sent between the two, so a path that starts moving is joined without a stop.

**No route.** If the planner finds no route, the command sends zero velocity and ends at once. It does not throw. The reason is in `onTheFly().failure()` and in the `PlanFailure` telemetry entry. A `thenFollow` command does not go on to its path in that case.

**Canceling.** The command stops following, sends zero velocity, and marks the path as no longer active in telemetry. If a `thenFollow` command has already handed over, the cancel goes to the path's follow command, which does the same.

## Telemetry

The command publishes under `/Telemetry/Path/<name>/`, where `<name>` is the command's name from the table above. The plan is published again at every re-plan, along with the planning figures: `PlanOk`, `PlanMicros`, `Replans`, `StartSnap`, `GoalSnap`, `MinClearance`, `PlanClear` and `PlanFailure`. [`PathTelemetry`](/reference/wpilib/pathtelemetry/) lists every entry.

:::note
Two commands built by `otf.to` are both named `"Pathfind"` and share one telemetry table. Only one drives at a time, so the values do not mix, but the table does not say which of the two is running.
:::
