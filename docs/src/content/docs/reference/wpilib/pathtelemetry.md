---
title: PathTelemetry
description: Publishes a follower's state, its plan and its events to NetworkTables each loop.
sidebar:
  order: 9
---

`PathTelemetry` publishes what a path follower is doing to NetworkTables each loop: where it is on the path, how far off, how fast, which events have fired, and the planned path itself.

[`FollowPathCommand`](/reference/wpilib/followpathcommand/) and [`PathfindCommand`](/reference/wpilib/pathfindcommand/) each own one and call it for you, so this page is mostly for reading the entries. [Telemetry](/guides/telemetry/) is the guide.

## Example

```java
// Only for code that drives a PathFollower from its own loop
PathTelemetry telemetry = new PathTelemetry("ToReef");   // robot init

telemetry.begin(follower);    // after follower.start(...)
telemetry.log(follower);      // after each follower.step(...)
telemetry.event(name);        // as each event fires, with its name
telemetry.end();              // at the finish or on a cancel
```

## Constructor and methods

None of these throw.

| Member | What it does |
| --- | --- |
| `PathTelemetry(String pathName)` | Telemetry for one path, under `Path/<pathName>`. Build it in robot init. |
| `begin(PathFollower f)` | Publishes the entries in "Once per start", then calls `begin()`. Call it after the follower has started, because the plan is published in the frame the follower was started with. |
| `begin()` | Makes this path the active one and empties both `EventsFired` lists. |
| `log(PathFollower f)` | Publishes the entries in "Every loop". |
| `event(String name)` | Records that an event fired: adds it to both `EventsFired` lists and sets `LastEvent`. |
| `end()` | Sets `Active` to an empty string. |
| `plan(PlanResult r, int replans)` | Publishes the entries in "Planning figures" from a [`PlanResult`](/reference/plan/planresult/). |

## Tables

| Table | Holds |
| --- | --- |
| `/Telemetry/Path/<name>/` | One path. `<name>` is the command's name: the path's name for a drawn path, such as `ToReef`, or `Pathfind`, `Pathfind <point>` or `Pathfind to <path>` for an on-the-fly command. |
| `/Telemetry/PathFollower/` | Shared by every follow command. Says which path is running now. |

## Every loop

Written under `/Telemetry/Path/<name>/` once per loop while the path runs.

| Entry | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `Phase` | string | | What the follower is doing: `TRACKING` along the path, `TERMINAL` in the final approach to a stop, `FINISHED`. `IDLE` and `CANCELLED` are the other phases. |
| `Progress` | double | m | Distance along the path. |
| `Remaining` | double | m | Distance left on the path. |
| `CrossTrack` | double | m | Signed distance from the path, positive to the left of travel. It is 0 in the `TERMINAL` phase. |
| `AlongTrack` | double | m | How far ahead of the plan's schedule the robot is. Negative means behind. |
| `HeadingError` | double | rad | The reference heading minus the measured heading. |
| `Speed` | double | m/s | The measured speed over the ground. |
| `PlannedSpeed` | double | m/s | The plan's speed at the robot's progress. |
| `Omega` | double | rad/s | The measured turn rate. |
| `PlannedOmega` | double | rad/s | The plan's turn rate at the robot's progress. |
| `Elapsed` | double | s | Time since the path started. |
| `Reference` | `Pose2d` struct | m, rad | The pose the follower is steering for this loop. Real field frame. |
| `Goal` | `Pose2d` struct | m, rad | The pose the path ends at. With a moving goal, the latest one. Real field frame. |

## Once per start

Written under `/Telemetry/Path/<name>/` when the path starts. An on-the-fly command writes them again at every re-plan.

| Entry | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `Plan` | double array | see below | The planned path, sampled every 0.05 m, plus its end. Sent once, not every loop. |
| `PlanEvents` | string array | | The names of the path's events, in the order they fire. |
| `PlanEventMeters` | double array | m | Each event's distance along the path, in the same order. |
| `PlanSeconds` | double | s | The planned time for the whole path. |
| `EventsFired` | string array | | Reset to an empty array. |

### The Plan array

Six numbers per sample, one sample after another. A 5 m path is 101 samples, so about 600 numbers.

| Position in each six | Value | Unit |
| --- | --- | --- |
| 0 | distance along the path | m |
| 1 | x, real field frame | m |
| 2 | y, real field frame | m |
| 3 | heading, real field frame | rad |
| 4 | planned time at this point | s |
| 5 | planned speed at this point | m/s |

## When an event fires

| Entry | Table | Type | Meaning |
| --- | --- | --- | --- |
| `EventsFired` | `/Telemetry/Path/<name>/` | string array | Every event fired on this run so far, in order. |
| `EventsFired` | `/Telemetry/PathFollower/` | string array | The same list, for the running path. |
| `LastEvent` | `/Telemetry/PathFollower/` | string | The name of the event that just fired. It is not cleared when a new path starts. |

## The running path

| Entry | Table | Type | When written | Meaning |
| --- | --- | --- | --- | --- |
| `Active` | `/Telemetry/PathFollower/` | string | At the start, and at the end | The running path's name at the start. An empty string once it finishes or is canceled. |
| `EventsFired` | `/Telemetry/PathFollower/` | string array | At the start | Reset to an empty array. |

## Planning figures

Written under `/Telemetry/Path/<name>/` by an on-the-fly command, each time it plans or re-plans.

| Entry | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `PlanOk` | boolean | | `true` when a route was found. |
| `PlanMicros` | double | µs | How long the whole plan took. |
| `Replans` | integer | | Re-plans so far on this run. 0 for the first plan. |
| `StartSnap` | double | m | How far a start inside a grown obstacle was moved to reach free space. |
| `GoalSnap` | double | m | The same for the goal. |
| `MinClearance` | double | m | The smallest gap along the plan between the robot's outline and an obstacle or wall. `NaN` when no route was found. |
| `PlanClear` | boolean | | `true` when a route was found and the outline keeps the planner's margin all the way along it. |
| `PlanFailure` | string | | Why there is no route. An empty string when it planned. |

:::note
A `PlanOk` of `true` with a `PlanClear` of `false` means the robot is driving a route that comes closer to something than the margin asks for. Look at `MinClearance` to see how close.
:::

## Good to know

- **Poses are in the real field frame**, the frame the robot's own pose is in. On red, the plan and the reference are the flipped ones, so they can be drawn straight over the robot's pose.
- **Scalar errors and rates are worked out in the blue frame**, the frame the path was drawn in. For a `RED_MIRROR` flip that reverses the sign of the turn rates and of left and right.
- **`begin(PathFollower)` samples the plan.** That is the only path work these methods do.

## More

- [Telemetry](/guides/telemetry/): viewing the entries in the editor or AdvantageScope.
- [`PathFollower`](/reference/follow/pathfollower/): the methods these entries come from.
