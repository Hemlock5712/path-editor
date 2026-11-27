---
title: OnTheFly
description: One on-the-fly run that plans a path at start, follows it and plans again when needed.
sidebar:
  order: 2
---

`OnTheFly` is one run of an on-the-fly path: it plans from the robot's measured pose to the goal, follows the result, and plans again when needed.

Robot code does not usually build one. `Pathfinding` builds `PathfindCommand`s, and each command creates an `OnTheFly` when it starts. You meet it through `PathfindCommand.onTheFly()`, to read how a run went. See [Pathfinding](/guides/pathfinding/).

Package: `frc.team5712.path.plan`.

## Example

```java
PathfindCommand toDepot = otf.toPoint(drive, Points.DEPOT, Rotation2d.PI);

// later, while or after it runs
OnTheFly run = toDepot.onTheFly();
if (run != null && run.lastPlan() != null) {
  System.out.println("re-plans " + run.replans() + ", slowest plan "
      + run.maxPlanMicros() + " us, wide margin " + run.lastPlanWide());
}

// Driving one by hand, with no commands. pose, velocity and goal are the library's
// Pose and FieldSpeeds, and now is the time in seconds.
OnTheFly byHand = new OnTheFly(paths.pathfinder(), FollowerConfig.defaults(paths.robot()), () -> goal)
    .preferring(paths.widePathfinder());

if (byHand.start(now, pose, velocity)) {
  // every loop, until byHand.isFinished()
  FieldSpeeds command = byHand.step(now, pose, velocity);
}
```

## Constants

| Constant | What it is |
| --- | --- |
| `DEFAULT_REPLAN_METERS` | `0.20` meters. The default for `replanAbove`: re-plan when further than this off the path |
| `REPLAN_COOLDOWN_SECONDS` | `0.5` seconds. At most one off-the-path re-plan per this long |
| `WIDE_DETOUR_RATIO` | `1.15`. The wider-margin route is used only when it is at most this many times as long as the main one |
| `GOAL_REPLAN_METERS` | `0.10` meters. A moving goal further than this from the goal the path was planned to causes a re-plan |
| `GOAL_REPLAN_COOLDOWN_SECONDS` | `0.2` seconds. At most one moving-goal re-plan per this long |

## Constructors

| Constructor | What it does |
| --- | --- |
| `OnTheFly(Pathfinder pathfinder, FollowerConfig config, Supplier<Pose> goal)` | A run that plans with `pathfinder` and follows with `config`. `goal` is read when the run starts and again at each re-plan, in the real field frame. |

## Setup

Call these before `start`. Each returns the same object, so they chain.

| Method | What it does |
| --- | --- |
| `preferring(Pathfinder widerMargin)` | Tries `widerMargin` first: a pathfinder for the same field with a wider margin, such as `PlannerConfig.WIDE_MARGIN_METERS`. Its plan is used when its route is at most `WIDE_DETOUR_RATIO` times as long as the main pathfinder's. `null` means the main pathfinder only, which is the default. |
| `arriving(double metersPerSecond, double tangentRadians)` | Arrives at this speed, in m/s, traveling in this direction, in radians. Used for a hand-over into a drawn path. The default is to arrive at rest from any direction. |
| `replanAbove(double meters)` | Re-plans when the robot is further than this from the path. The default is 0.20 m. `Double.POSITIVE_INFINITY` means never on its own. |
| `withMovingGoal(Supplier<Optional<Pose>> goal)` | The path's end follows this goal, in the real field frame. An empty value keeps the last one. When it gives a value at start, that value is the goal planned to, in place of the constructor's. |
| `withMovingGoal(Supplier<Optional<Pose>> goal, double smoothingSeconds)` | The same for estimates that arrive raw from a camera: they are averaged over `smoothingSeconds`. The source suggests 0.5 s as a starting point. The average starts again at each re-plan. |
| `finishWhen(BooleanSupplier ready)` | Holds the finish until `ready` is true. The robot drives to the goal and holds it meanwhile. |
| `onPlan(Consumer<OnTheFly> listener)` | Called after every plan, the first and each re-plan, including a plan that found no route. The commands use it to publish telemetry. Setting it again replaces the earlier listener. |

## Running

`t` is the time in seconds, `pose` the measured pose in meters and radians, `velocity` the measured field velocity in m/s and rad/s.

| Method | What it does |
| --- | --- |
| `public boolean start(double t, Pose pose, FieldSpeeds velocity)` | Plans and starts following. Also resets the re-plan count, the failure and the slowest-plan time. Returns true when a path was found. Returns false when there is no route: then `failure()` says why, `follower()` is null and `isFinished()` is true. |
| `public FieldSpeeds step(double t, Pose pose, FieldSpeeds velocity)` | One loop. Re-plans if needed, steps the follower, and returns the field speeds to command, in m/s and rad/s. Returns zero speeds when there is no plan, or when a re-plan finds no route. |
| `void requestReplan()` | Plans again on the next `step`, from wherever the robot is then. No cooldown applies. |
| `boolean isFinished()` | True when the follower has finished, by arriving or by handing over on a rolling end, or when no route was found. |
| `void cancel()` | Cancels the follower, if there is one. |

## When step plans again

Before stepping the follower, `step` re-plans if the run has not finished and any row is true.

| Cause | All of these hold |
| --- | --- |
| Off the path | The follower is tracking, the cross-track error is above the `replanAbove` distance, and `REPLAN_COOLDOWN_SECONDS` have passed since the last plan |
| The goal moved | There is a moving goal, the follower is tracking, the goal is more than `GOAL_REPLAN_METERS` from the goal the path was planned to, and `GOAL_REPLAN_COOLDOWN_SECONDS` have passed since the last plan |
| You asked | `requestReplan()` was called |

## Reading the run

| Method | What it does |
| --- | --- |
| `PathFollower follower()` | The follower on the current plan. Null before a plan and when there is no route. |
| `PlanResult lastPlan()` | The most recent plan. Null before `start`. |
| `boolean lastPlanWide()` | True when the last plan came from the wider-margin pathfinder. |
| `int replans()` | Re-plans so far this run, automatic and requested. |
| `double maxPlanMicros()` | The slowest plan this run, in microseconds. |
| `String failure()` | Why there is no route, or null. |
| `Pathfinder pathfinder()` | The main pathfinder. |
| `FieldSpeeds endCommand()` | The speed to hand to the next path on a rolling end. Zero with no plan. |
| `FieldSpeeds commandedAcceleration()` | The acceleration the last command asks for, for feed-forward. Zero with no plan. |
| `boolean rollingEnd()` | True when the current plan ends moving. False with no plan. |

## Good to know

- **Everything is in the real field frame**: the pose, the velocity, the goal. A goal authored in the blue frame has to be flipped for the red alliance before it is handed in. The `Pathfinding` commands do that.
- **The commands set a run up for you.** Build one yourself only when you drive the library without WPILib commands, as its own tests do.

:::note
`maxPlanMicros()` counts only the plan that was used at each planning step. When both pathfinders were tried, the time of the one not used is not included.
:::

## More

- [On-the-fly paths](/concepts/on-the-fly/) and [Moving goals](/concepts/moving-goals/): the behavior.
- [Pathfinder](/reference/plan/pathfinder/) and [PlanResult](/reference/plan/planresult/): the planner and what it returns.
- [PathfindCommand](/reference/wpilib/pathfindcommand/) and [Pathfinding](/reference/wpilib/pathfinding/): the commands that build a run.
