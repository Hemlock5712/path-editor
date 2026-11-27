---
title: PlanResult
description: What one pathfinder plan produced, with the path, the route figures and the timings.
sidebar:
  order: 3
---

`frc.team5712.path.plan.PlanResult` is the record that `Pathfinder.plan` returns. It holds the path to follow, or the reason there is none, together with figures about the plan: how long the route is, whether the start or goal had to be moved, how close the robot comes to anything, and how long each part of planning took.

A student meets it when checking a plan by hand or reading how an on-the-fly command went, through `PathfindCommand.onTheFly().lastPlan()`. The `PathfindCommand` publishes the main figures as telemetry, so for most uses the dashboard is enough. See [Telemetry](/guides/telemetry/) and [Pathfinding](/guides/pathfinding/).

## Example

```java
PlanResult plan = paths.pathfinder().plan(start, new FieldSpeeds(0, 0, 0), goal);

if (!plan.ok()) {
  System.out.println("no plan: " + plan.failure());
} else if (!plan.clear()) {
  System.out.println("closest approach " + plan.minClearanceMeters() + " m is inside the margin");
} else {
  Path path = plan.path();
}
```

## Components

A record's components are read with a method of the same name, for example `plan.routeLengthMeters()`.

| Component | Type | Unit | Description |
| --- | --- | --- | --- |
| `path` | `Path` | | The path to follow. Null when there is no route. |
| `waypoints` | `List<double[]>` | meters, radians | The spline's waypoints, in order. See below. |
| `routeLengthMeters` | `double` | meters | The length of the corner-to-corner route, before smoothing |
| `straightLineMeters` | `double` | meters | Start to goal in a straight line |
| `startSnapMeters` | `double` | meters | How far the start was moved out of a grown obstacle or away from a wall. 0 when it was not moved. |
| `goalSnapMeters` | `double` | meters | The same for the goal |
| `refits` | `int` | | How many times the smoother added waypoints and fitted the spline again |
| `clear` | `boolean` | | True when the swept outline keeps the margin everywhere, outside snapped stretches |
| `minClearanceMeters` | `double` | meters | The smallest distance from the robot's outline to an obstacle or wall along the path, outside snapped stretches |
| `searchMicros` | `double` | microseconds | Time spent snapping, joining the graph and searching |
| `smoothMicros` | `double` | microseconds | Time spent fitting splines and sweeping the outline |
| `profileMicros` | `double` | microseconds | Time spent building the final path: geometry, heading and speed plan |
| `totalMicros` | `double` | microseconds | The whole plan |
| `failure` | `String` | | Why there is no route. Null when there is one. |

### waypoints

Each entry is `{x, y}` in meters. The list holds the route's corners, any waypoints the smoother added, and the robot's and the goal's own positions when they were snapped.

An entry can have a third number, `{x, y, heading}`, with the heading in radians. That happens where the plan turns a long [footprint](/concepts/footprints/) narrow side on through a gap. The path has a rotation target at that waypoint.

### clear

`clear` is about the outline the pathfinder was configured with: the bumpers, or a footprint. A plan with `clear` false still has a path. It means the allowed number of refits ran out and the outline comes closer than the margin somewhere. `minClearanceMeters` says how close.

Stretches next to a snapped start or goal are left out of both figures, because a robot that starts inside the margin cannot keep it there.

## Methods

| Method | Returns | Description |
| --- | --- | --- |
| `ok()` | `boolean` | True when there is a path |
| `startSnapped()` | `boolean` | True when the start was moved, by more than a nanometer |
| `goalSnapped()` | `boolean` | True when the goal was moved, by more than a nanometer |

## A failed plan

When there is no route, the record is filled like this:

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

:::caution
Check `ok()` before using `path()`. It is null on a failed plan.
:::

## Constructor

The record has a public constructor that takes the fourteen components in the order of the table above. It exists because every record has one. Results are made by `Pathfinder.plan`, and there is no reason for robot code to build one.

## Related

- [Pathfinder](/reference/plan/pathfinder/)
- [OnTheFly](/reference/plan/onthefly/), for `lastPlan()`
- [Path](/reference/core/path/)
