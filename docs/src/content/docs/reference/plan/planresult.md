---
title: PlanResult
description: What one pathfinder plan produced, with the path, the route figures and the timings.
sidebar:
  order: 3
---

`PlanResult` is the record that `Pathfinder.plan` returns: the path to follow, or the reason there is none, with figures about the plan.

You meet it when checking a plan by hand, or reading how an on-the-fly command went through `PathfindCommand.onTheFly().lastPlan()`. The `PathfindCommand` publishes the main figures as telemetry, so for most uses the dashboard is enough. See [Telemetry](/guides/telemetry/) and [Pathfinding](/guides/pathfinding/).

Package: `frc.team5712.path.plan`.

## Example

```java
PlanResult plan = paths.pathfinder().plan(start, new FieldSpeeds(0, 0, 0), goal);

if (!plan.ok()) {
  // no route: path() is null
  System.out.println("no plan: " + plan.failure());
} else if (!plan.clear()) {
  // there is a path, but the outline comes closer than the margin somewhere
  System.out.println("closest approach " + plan.minClearanceMeters() + " m is inside the margin");
} else {
  Path path = plan.path();
}
```

## Constructors

| Constructor | What it does |
| --- | --- |
| The fourteen components, in the order of the table below | The public record constructor. It exists because every record has one. Results are made by `Pathfinder.plan`, and there is no reason for robot code to build one. |

## Components

Each is read with a method of the same name, for example `plan.routeLengthMeters()`.

| Component | What it is |
| --- | --- |
| `Path path` | The path to follow. Null when there is no route. |
| `List<double[]> waypoints` | The spline's waypoints, in order, in meters and radians. See the table below. |
| `double routeLengthMeters` | The length of the corner-to-corner route, before smoothing, in meters |
| `double straightLineMeters` | Start to goal in a straight line, in meters |
| `double startSnapMeters` | How far the start was moved out of a grown obstacle or away from a wall, in meters. 0 when it was not moved. |
| `double goalSnapMeters` | The same for the goal |
| `int refits` | How many times the smoother added waypoints and fitted the spline again |
| `boolean clear` | True when the swept outline keeps the margin everywhere, outside snapped stretches |
| `double minClearanceMeters` | The smallest distance from the robot's outline to an obstacle or wall along the path, outside snapped stretches, in meters |
| `double searchMicros` | Time spent snapping, joining the graph and searching, in microseconds |
| `double smoothMicros` | Time spent fitting splines and sweeping the outline, in microseconds |
| `double profileMicros` | Time spent building the final path: geometry, heading and speed plan, in microseconds |
| `double totalMicros` | The whole plan, in microseconds |
| `String failure` | Why there is no route. Null when there is one. |

## What waypoints holds

| Entry | What it is |
| --- | --- |
| `{x, y}` | A point in meters: a corner of the route, a waypoint the smoother added, or the robot's or the goal's own position when it was snapped |
| `{x, y, heading}` | The same with a heading in radians, where the plan turns a long [footprint](/concepts/footprints/) narrow side on through a gap. The path has a rotation target at that waypoint. |

## Methods

| Method | What it does |
| --- | --- |
| `boolean ok()` | True when there is a path. |
| `boolean startSnapped()` | True when the start was moved, by more than a nanometer. |
| `boolean goalSnapped()` | True when the goal was moved, by more than a nanometer. |

## A failed plan

When there is no route, the record is filled like this.

| Component | Value |
| --- | --- |
| `path` | null |
| `failure` | `"no route between the start and the goal"` |
| `waypoints` | an empty list |
| `routeLengthMeters`, `minClearanceMeters` | `Double.NaN` |
| `clear` | false |
| `refits` | 0 |
| `smoothMicros`, `profileMicros` | 0 |
| `searchMicros`, `totalMicros` | the time the search took |
| `startSnapMeters`, `goalSnapMeters`, `straightLineMeters` | as measured |

## Good to know

- **`clear` is about the outline the pathfinder was configured with**: the bumpers, or a footprint.
- **A plan with `clear` false still has a path.** The allowed number of refits ran out and the outline comes closer than the margin somewhere. `minClearanceMeters` says how close.
- **Stretches next to a snapped start or goal are left out** of both figures, because a robot that starts inside the margin cannot keep it there.

:::caution
Check `ok()` before using `path()`. It is null on a failed plan.
:::

## More

- [Pathfinder](/reference/plan/pathfinder/): what makes a result.
- [OnTheFly](/reference/plan/onthefly/): `lastPlan()`.
- [Path](/reference/core/path/): the path inside.
