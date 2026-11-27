---
title: Pathfinder
description: Plans a path around the field's obstacles from the robot's pose to a goal pose.
sidebar:
  order: 1
---

`frc.team5712.path.plan.Pathfinder` plans a collision-free path across the field. Building one grows the obstacles and builds the visibility graph. Each call to `plan` then finds the shortest route, smooths it into a spline, checks it with the robot's outline and returns an ordinary `Path` for the follower. The same inputs always give the same path. [Pathfinder](/concepts/pathfinder/) explains each step.

Most students never construct one. `PathLibrary.loadAll()` reads `field.json` and builds the pathfinders at boot, and `Pathfinding` builds the `PathfindCommand`s that use them through `OnTheFly`. Start with [Pathfinding](/guides/pathfinding/). You meet this class directly when you want to look at a plan yourself, for example in a unit test or to print how long planning takes.

The class has no WPILib in it. Poses are the library's own `Pose`, in meters and radians, in the real field frame.

## Example

```java
Pathfinder pathfinder = paths.pathfinder();

Pose start = new Pose(-4.0, -1.5, 0.0);
Pose goal = new Pose(3.8, 1.4, Math.PI);
PlanResult plan = pathfinder.plan(start, new FieldSpeeds(0, 0, 0), goal);

if (plan.ok()) {
  System.out.println("route " + plan.routeLengthMeters() + " m, planned in "
      + plan.totalMicros() + " us, clear " + plan.clear());
} else {
  System.out.println(plan.failure());
}
```

`paths` is the `PathLibrary`. To build a pathfinder without one:

```java
FieldMap field = FieldMap.read(java.nio.file.Path.of("src/main/deploy/paths/field.json"));
Pathfinder pathfinder = new Pathfinder(field, RobotConfig.DEFAULT, PlannerConfig.DEFAULT);
```

## Constructor

| Constructor | Description |
| --- | --- |
| `Pathfinder(FieldMap field, RobotConfig robot, PlannerConfig config)` | Grows every obstacle by `config.growMeters()`, moves the walls in by the same distance and builds the visibility graph. |

`robot` is used for the speed plan of every path this pathfinder returns. `config` gives the bumper size, the margin and the speed caps.

When `config` has a footprint that reaches further than the bumpers, the constructor also builds a second graph grown by `config.bumperGrowMeters()`. `plan` falls back to it. See [Footprints](/concepts/footprints/).

Construction is the slow part: 32 to 135 ms in a fresh JVM on a desktop PC, not measured on a robot controller. Do it once, at boot.

## Constants

| Constant | Value | Description |
| --- | --- | --- |
| `APPROACH_METERS` | `0.5` | Meters. The length of the straight approach that ends a route planned with an end direction, before the braking distance is added. |

## Planning

### plan(start, startVelocity, goal)

```java
public PlanResult plan(Pose start, FieldSpeeds startVelocity, Pose goal)
```

Plans from the robot's pose and field velocity to the goal pose, arriving at rest. The same as the five-argument form with an end speed of 0 and no end direction.

### plan(start, startVelocity, goal, endSpeed, endTangent)

```java
public PlanResult plan(Pose start, FieldSpeeds startVelocity, Pose goal,
    double endSpeed, double endTangent)
```

| Parameter | Unit | Description |
| --- | --- | --- |
| `start` | meters, radians | Where the robot is, real field frame |
| `startVelocity` | m/s, rad/s | The robot's field velocity. Above 0.1 m/s the path leaves along it, at that speed. |
| `goal` | meters, radians | Where to end, and the heading to end with |
| `endSpeed` | m/s | The speed to arrive at. Limited to between 0 and the config's speed cap. |
| `endTangent` | radians | The direction of travel to arrive in. `Double.NaN` means whatever the route gives. |

With an `endTangent`, the route ends with a straight approach along it:

```text
approach = APPROACH_METERS + endSpeed^2 / (2 x maxMetersPerSecondSquared)
```

The approach is used only when the goal was not snapped, the goal is more than twice the approach length from the start, and the approach is free of grown obstacles. Otherwise the direction is dropped and the plan is made without it. This is how a hand-over into a drawn path is planned.

Returns a [PlanResult](/reference/plan/planresult/). When there is no route, `ok()` is false, `path()` is null and `failure()` is `"no route between the start and the goal"`. It does not throw for a missing route.

A returned plan is not always clear. If `maxRefits` refits were not enough, the plan comes back with `clear()` false.

With a footprint's second graph: if the first graph gives no route or a plan that is not clear, the second is tried. Its plan is returned when it has a route and is either clear or the only one with a route.

Throws `IllegalStateException` with `"no free point on the field"` if a start or goal has to be snapped and no free point exists within 8 m of it.

### routeLengthMeters(start, goal)

```java
public double routeLengthMeters(Pose start, Pose goal)
```

The length in meters of the shortest corner-to-corner route from start to goal, including any snap distances. Returns `Double.POSITIVE_INFINITY` when there is no route. This runs the search alone, without the smoothing and the speed plan, so it takes about a tenth of a millisecond on a desktop PC where a full plan takes one or two. `OnTheFly` uses it to compare the two margins.

### warmUp(queries)

```java
public double warmUp(int queries)
```

Makes `queries` plans between fixed points spread over the field and throws the results away, so Java has compiled the planner before the first real plan. Returns the time it took in milliseconds. The points are the same on every call. They are kept within 8 m by 4 m of the center, so the plans are field-sized even on an open floor. Call it once at boot, after the constructor. `PathLibrary` does this with 40 queries.

## Queries about the field

| Method | Returns | Description |
| --- | --- | --- |
| `isFree(double x, double y)` | `boolean` | True when a robot center at (x, y) meters is outside every grown obstacle and inside the moved-in walls. |
| `visible(double x0, double y0, double x1, double y1)` | `boolean` | True when the straight segment between the two points, in meters, crosses no grown obstacle. It does not check the walls. |
| `clearance(double x, double y, double th)` | `double` | The distance in meters from the robot's outline, centered at (x, y) with heading `th` radians, to the nearest real obstacle or wall. 0 when it touches or overlaps an obstacle. |

`isFree` and `visible` work on the grown shapes, where the robot is a point. `clearance` works on the real shapes with the real outline, which is the config's footprint.

## Accessors

| Method | Returns | Description |
| --- | --- | --- |
| `field()` | `FieldMap` | The field it was built from |
| `config()` | `PlannerConfig` | The settings it was built with |
| `grownObstacles()` | `List<ConvexPolygon>` | The grown obstacles, what the robot's center must stay out of |
| `nodeCount()` | `int` | The number of nodes in the visibility graph: the grown corners that are in free space |
| `bootMicros()` | `double` | How long the constructor took, in microseconds |

## Related

- [PlanResult](/reference/plan/planresult/), [PlannerConfig](/reference/plan/plannerconfig/), [FieldMap](/reference/plan/fieldmap/)
- [OnTheFly](/reference/plan/onthefly/), which calls it
- [PathLibrary](/reference/wpilib/pathlibrary/) and [Pathfinding](/reference/wpilib/pathfinding/), how robot code reaches it
