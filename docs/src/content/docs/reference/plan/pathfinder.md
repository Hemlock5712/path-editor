---
title: Pathfinder
description: Plans a path around the field's obstacles from the robot's pose to a goal pose.
sidebar:
  order: 1
---

`Pathfinder` plans a collision-free path across the field and returns an ordinary `Path` for the follower.

Most students never construct one. `PathLibrary.loadAll()` reads `field.json` and builds the pathfinders at boot, and `Pathfinding` builds the `PathfindCommand`s that use them through `OnTheFly`. Start with [Pathfinding](/guides/pathfinding/).

Package: `frc.team5712.path.plan`.

## Example

```java
Pathfinder pathfinder = paths.pathfinder();     // paths is the PathLibrary

Pose start = new Pose(-4.0, -1.5, 0.0);
Pose goal = new Pose(3.8, 1.4, Math.PI);
PlanResult plan = pathfinder.plan(start, new FieldSpeeds(0, 0, 0), goal);

if (plan.ok()) {
  System.out.println("route " + plan.routeLengthMeters() + " m, planned in "
      + plan.totalMicros() + " us, clear " + plan.clear());
} else {
  System.out.println(plan.failure());
}

// To build a pathfinder without a PathLibrary
FieldMap field = FieldMap.read(java.nio.file.Path.of("src/main/deploy/paths/field.json"));
Pathfinder byHand = new Pathfinder(field, RobotConfig.DEFAULT, PlannerConfig.DEFAULT);
```

## Constructors

| Constructor | What it does |
| --- | --- |
| `Pathfinder(FieldMap field, RobotConfig robot, PlannerConfig config)` | Grows every obstacle by `config.growMeters()`, moves the walls in by the same distance and builds the visibility graph. `robot` is used for the speed plan of every path returned. `config` gives the bumper size, the margin and the speed caps. |

## Constants

| Constant | What it is |
| --- | --- |
| `APPROACH_METERS` | `0.5` meters. The length of the straight approach that ends a route planned with an end direction, before the braking distance is added. |

## Planning

| Method | What it does |
| --- | --- |
| `public PlanResult plan(Pose start, FieldSpeeds startVelocity, Pose goal)` | Plans from the robot's pose and field velocity to the goal pose, arriving at rest. The same as the five-argument form with an end speed of 0 and no end direction. |
| `public PlanResult plan(Pose start, FieldSpeeds startVelocity, Pose goal, double endSpeed, double endTangent)` | Plans to arrive at `endSpeed`, traveling along `endTangent`. Returns a [PlanResult](/reference/plan/planresult/). Throws `IllegalStateException` with `"no free point on the field"` if a start or goal has to be snapped and no free point exists within 8 m of it. |
| `public double routeLengthMeters(Pose start, Pose goal)` | The length in meters of the shortest corner-to-corner route from start to goal, including any snap distances. `Double.POSITIVE_INFINITY` when there is no route. Runs the search alone, without the smoothing and the speed plan. |
| `public double warmUp(int queries)` | Makes `queries` plans between fixed points spread over the field and throws the results away, so Java has compiled the planner before the first real plan. Returns the time it took in milliseconds. |

## Parameters of plan

| Parameter | What it is |
| --- | --- |
| `start` | Where the robot is, real field frame, in meters and radians |
| `startVelocity` | The robot's field velocity, in m/s and rad/s. Above 0.1 m/s the path leaves along it, at that speed. |
| `goal` | Where to end, and the heading to end with, in meters and radians |
| `endSpeed` | The speed to arrive at, in m/s. Limited to between 0 and the config's speed cap. |
| `endTangent` | The direction of travel to arrive in, in radians. `Double.NaN` means whatever the route gives. |

## Queries about the field

| Method | What it does |
| --- | --- |
| `boolean isFree(double x, double y)` | True when a robot center at (x, y) meters is outside every grown obstacle and inside the moved-in walls. |
| `boolean visible(double x0, double y0, double x1, double y1)` | True when the straight segment between the two points, in meters, crosses no grown obstacle. It does not check the walls. |
| `double clearance(double x, double y, double th)` | The distance in meters from the robot's outline, centered at (x, y) with heading `th` radians, to the nearest real obstacle or wall. 0 when it touches or overlaps an obstacle. |

## Accessors

| Method | What it does |
| --- | --- |
| `FieldMap field()` | The field it was built from. |
| `PlannerConfig config()` | The settings it was built with. |
| `List<ConvexPolygon> grownObstacles()` | The grown obstacles, what the robot's center must stay out of. |
| `int nodeCount()` | The number of nodes in the visibility graph: the grown corners that are in free space. |
| `double bootMicros()` | How long the constructor took, in microseconds. |

## What plan returns

| Case | Result |
| --- | --- |
| No route | `ok()` is false, `path()` is null and `failure()` is `"no route between the start and the goal"`. It does not throw for a missing route. |
| `maxRefits` refits were not enough | The plan comes back with `clear()` false. A returned plan is not always clear. |
| An `endTangent` was given | The route ends with a straight approach of `APPROACH_METERS + endSpeed^2 / (2 x maxMetersPerSecondSquared)` along it. This is how a hand-over into a drawn path is planned. |
| The goal was snapped, the goal is within twice the approach length of the start, or the approach is not free of grown obstacles | The direction is dropped and the plan is made without it. |
| The footprint reaches further than the bumpers | The constructor also builds a second graph grown by `config.bumperGrowMeters()`. If the first graph gives no route or a plan that is not clear, the second is tried. Its plan is returned when it has a route and is either clear or the only one with a route. See [Footprints](/concepts/footprints/). |

## Good to know

- **Construction is the slow part**: 32 to 135 ms in a fresh JVM on a desktop PC, not measured on a robot controller. Do it once, at boot.
- **Call `warmUp` once at boot, after the constructor.** `PathLibrary` does this with 40 queries. The points are the same on every call, and are kept within 8 m by 4 m of the center, so the plans are field-sized even on an open floor.
- **`routeLengthMeters` is fast**: about a tenth of a millisecond on a desktop PC, where a full plan takes one or two. `OnTheFly` uses it to compare the two margins.
- **Two kinds of shape.** `isFree` and `visible` work on the grown shapes, where the robot is a point. `clearance` works on the real shapes with the real outline, which is the config's footprint.

:::note
The class has no WPILib in it. Poses are the library's own `Pose`, in meters and radians, in the real field frame. The same inputs always give the same path.
:::

## More

- [Pathfinder](/concepts/pathfinder/): each step of a plan.
- [PlanResult](/reference/plan/planresult/), [PlannerConfig](/reference/plan/plannerconfig/), [FieldMap](/reference/plan/fieldmap/) and [OnTheFly](/reference/plan/onthefly/): what it takes, returns and is called by.
- [PathLibrary](/reference/wpilib/pathlibrary/) and [Pathfinding](/reference/wpilib/pathfinding/): how robot code reaches it.
