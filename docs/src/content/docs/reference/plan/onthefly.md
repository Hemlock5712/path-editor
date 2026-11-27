---
title: OnTheFly
description: One on-the-fly run that plans a path at start, follows it and plans again when needed.
sidebar:
  order: 2
---

`frc.team5712.path.plan.OnTheFly` is one run of an on-the-fly path. `start` plans from the robot's measured pose to the goal with a `Pathfinder` and starts a `PathFollower` on the result. `step` runs the follower each loop and plans again when the robot is knocked off the path, the goal moves, or you ask. [On-the-fly paths](/concepts/on-the-fly/) explains the behavior.

Robot code does not usually build one. `Pathfinding` builds `PathfindCommand`s, and each command creates an `OnTheFly` when it starts and sets it up for you. See [Pathfinding](/guides/pathfinding/). You meet this class through `PathfindCommand.onTheFly()`, to read how a run went, or when you drive the library without WPILib commands, as its own tests do.

Everything is in the real field frame: the pose, the velocity, the goal. A goal authored in the blue frame has to be flipped for the red alliance before it is handed in. The `Pathfinding` commands do that.

## Example

Reading the current run from a command:

```java
PathfindCommand toDepot = otf.toPoint(drive, Points.DEPOT, Rotation2d.PI);

// later, while or after it runs
OnTheFly run = toDepot.onTheFly();
if (run != null && run.lastPlan() != null) {
  System.out.println("re-plans " + run.replans() + ", slowest plan "
      + run.maxPlanMicros() + " us, wide margin " + run.lastPlanWide());
}
```

Driving one by hand, with no commands:

```java
OnTheFly run = new OnTheFly(paths.pathfinder(), FollowerConfig.defaults(paths.robot()), () -> goal)
    .preferring(paths.widePathfinder());

if (run.start(now, pose, velocity)) {
  // every loop, until run.isFinished()
  FieldSpeeds command = run.step(now, pose, velocity);
}
```

`pose`, `velocity` and `goal` are the library's `Pose` and `FieldSpeeds`, and `now` is the time in seconds.

## Constants

| Constant | Value | Unit | Description |
| --- | --- | --- | --- |
| `DEFAULT_REPLAN_METERS` | `0.20` | meters | The default for `replanAbove`: re-plan when further than this off the path |
| `REPLAN_COOLDOWN_SECONDS` | `0.5` | seconds | At most one off-the-path re-plan per this long |
| `WIDE_DETOUR_RATIO` | `1.15` | | The wider-margin route is used only when it is at most this many times as long as the main one |
| `GOAL_REPLAN_METERS` | `0.10` | meters | A moving goal further than this from the goal the path was planned to causes a re-plan |
| `GOAL_REPLAN_COOLDOWN_SECONDS` | `0.2` | seconds | At most one moving-goal re-plan per this long |

## Constructor

| Constructor | Description |
| --- | --- |
| `OnTheFly(Pathfinder pathfinder, FollowerConfig config, Supplier<Pose> goal)` | A run that plans with `pathfinder` and follows with `config`. `goal` is read when the run starts and again at each re-plan, in the real field frame. |

## Setup

Call these before `start`. Each returns the same object, so they chain.

| Method | Description |
| --- | --- |
| `preferring(Pathfinder widerMargin)` | Tries `widerMargin` first: a pathfinder for the same field with a wider margin, such as `PlannerConfig.WIDE_MARGIN_METERS`. Its plan is used when its route is at most `WIDE_DETOUR_RATIO` times as long as the main pathfinder's. `null` means the main pathfinder only, which is the default. |
| `arriving(double metersPerSecond, double tangentRadians)` | Arrives at this speed, in m/s, traveling in this direction, in radians. Used for a hand-over into a drawn path. The default is to arrive at rest from any direction. |
| `replanAbove(double meters)` | Re-plans when the robot is further than this from the path. The default is 0.20 m. `Double.POSITIVE_INFINITY` means never on its own. |
| `withMovingGoal(Supplier<Optional<Pose>> goal)` | The path's end follows this goal, in the real field frame. An empty value keeps the last one. When it gives a value at start, that value is the goal planned to, in place of the constructor's. |
| `withMovingGoal(Supplier<Optional<Pose>> goal, double smoothingSeconds)` | The same for estimates that arrive raw from a camera: they are averaged over `smoothingSeconds`. The source suggests 0.5 s as a starting point. The average starts again at each re-plan. |
| `finishWhen(BooleanSupplier ready)` | Holds the finish until `ready` is true. The robot drives to the goal and holds it meanwhile. |
| `onPlan(Consumer<OnTheFly> listener)` | Called after every plan, the first and each re-plan, including a plan that found no route. The commands use it to publish telemetry. Setting it again replaces the earlier listener. |

## Running

### start(t, pose, velocity)

```java
public boolean start(double t, Pose pose, FieldSpeeds velocity)
```

Plans and starts following. `t` is the time in seconds, `pose` the measured pose in meters and radians, `velocity` the measured field velocity in m/s and rad/s. It also resets the re-plan count, the failure and the slowest-plan time.

Returns true when a path was found. Returns false when there is no route. Then `failure()` says why, `follower()` is null and `isFinished()` is true.

### step(t, pose, velocity)

```java
public FieldSpeeds step(double t, Pose pose, FieldSpeeds velocity)
```

One loop. Takes the measured pose and field velocity and returns the field speeds to command, in m/s and rad/s. Before stepping the follower it re-plans if any of these is true and the run has not finished:

- the follower is tracking, the cross-track error is above the `replanAbove` distance, and `REPLAN_COOLDOWN_SECONDS` have passed since the last plan;
- there is a moving goal, the follower is tracking, the goal is more than `GOAL_REPLAN_METERS` from the goal the path was planned to, and `GOAL_REPLAN_COOLDOWN_SECONDS` have passed since the last plan;
- `requestReplan()` was called.

Returns zero speeds when there is no plan, or when a re-plan finds no route.

### Other run methods

| Method | Returns | Description |
| --- | --- | --- |
| `requestReplan()` | `void` | Plans again on the next `step`, from wherever the robot is then. No cooldown applies. |
| `isFinished()` | `boolean` | True when the follower has finished, by arriving or by handing over on a rolling end, or when no route was found |
| `cancel()` | `void` | Cancels the follower, if there is one |

## Reading the run

| Method | Returns | Description |
| --- | --- | --- |
| `follower()` | `PathFollower` | The follower on the current plan. Null before a plan and when there is no route. |
| `lastPlan()` | `PlanResult` | The most recent plan. Null before `start`. |
| `lastPlanWide()` | `boolean` | True when the last plan came from the wider-margin pathfinder |
| `replans()` | `int` | Re-plans so far this run, automatic and requested |
| `maxPlanMicros()` | `double` | The slowest plan this run, in microseconds |
| `failure()` | `String` | Why there is no route, or null |
| `pathfinder()` | `Pathfinder` | The main pathfinder |
| `endCommand()` | `FieldSpeeds` | The speed to hand to the next path on a rolling end. Zero with no plan. |
| `commandedAcceleration()` | `FieldSpeeds` | The acceleration the last command asks for, for feed-forward. Zero with no plan. |
| `rollingEnd()` | `boolean` | True when the current plan ends moving. False with no plan. |

:::note
`maxPlanMicros()` counts only the plan that was used at each planning step. When both pathfinders were tried, the time of the one not used is not included.
:::

## Related

- [Pathfinder](/reference/plan/pathfinder/) and [PlanResult](/reference/plan/planresult/)
- [PathFollower](/reference/follow/pathfollower/) and [FollowerConfig](/reference/follow/followerconfig/)
- [PathfindCommand](/reference/wpilib/pathfindcommand/) and [Pathfinding](/reference/wpilib/pathfinding/)
- [Moving goals](/concepts/moving-goals/)
