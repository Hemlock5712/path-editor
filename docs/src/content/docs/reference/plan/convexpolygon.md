---
title: ConvexPolygon
description: A convex polygon with the geometry tests the pathfinder needs.
sidebar:
  order: 6
---

`ConvexPolygon` is one obstacle: a **convex** polygon, meaning one with no dents, with the geometry tests the pathfinder is built on.

Robot code does not normally use it. Obstacles come from `field.json` through `FieldMap`, which `PathLibrary` loads at boot. You meet it when building a `FieldMap` in code for a test, or when reading `Pathfinder.grownObstacles()` to draw what the planner sees. See [Pathfinding](/guides/pathfinding/).

Package: `frc.team5712.path.plan`. All coordinates and distances are in meters.

## Example

```java
// an L shape: the dent at (1, 1) is filled in by the hull
ConvexPolygon shape = ConvexPolygon.of(new double[][] {
    {0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}});

System.out.println(shape.size());                        // 5 corners, not 6
System.out.println(shape.containsStrictly(1.4, 1.4));    // true: inside the filled dent
System.out.println(shape.distanceTo(3.0, 0.5));          // 1.0

// 0.651 m bigger on every side, round corners drawn in 30 degree steps
ConvexPolygon grown = shape.grow(0.651, Math.toRadians(30));
```

## Static factories

There is no public constructor.

| Method | What it does |
| --- | --- |
| `public static ConvexPolygon of(double[][] points)` | The **convex hull** of the points: the smallest convex polygon that contains them all, like a rubber band stretched around them. Each point is `{x, y}` in meters, in any order. Points inside the hull or along its edges are dropped. Throws `IllegalArgumentException` with `"a polygon needs at least three distinct corners"` when the hull has fewer than three corners, for example when all the points are on one line. |

## Corners

| Method | What it does |
| --- | --- |
| `int size()` | The number of corners. |
| `double x(int i)` | The X of corner `i`, in meters. Corners are numbered from 0, counter-clockwise. Throws `ArrayIndexOutOfBoundsException` for an index outside `0` to `size() - 1`. |
| `double y(int i)` | The Y of corner `i`, in meters. Throws the same as `x`. |

## Growing

| Method | What it does |
| --- | --- |
| `public ConvexPolygon grow(double r, double maxStepRadians)` | The polygon grown outward by `r` meters on every side. Each round corner is drawn as straight pieces that each turn at most `maxStepRadians`, with their points at `r / cos(step / 2)` from the corner, slightly outside the true arc. |

## Tests

| Method | What it does |
| --- | --- |
| `boolean containsStrictly(double x, double y)` | True when the point is inside the polygon and more than 0.0000001 m from every edge. A point on the boundary is not inside. |
| `boolean blocksSegment(double x0, double y0, double x1, double y1)` | True when the straight segment between the two points passes through the inside of the polygon. Touching a corner or running along an edge does not count. |

## Distances

| Method | What it does |
| --- | --- |
| `double distanceTo(double x, double y)` | The distance in meters from the point to the polygon. 0 when the point is inside or on the boundary. |
| `double[] nearestBoundaryPoint(double x, double y)` | The closest point on the boundary as `{x, y}`, whether the given point is inside or outside. |
| `double distanceToPolygon(double[] qx, double[] qy)` | The distance in meters between this polygon and another convex polygon, given as the X and the Y of its corners in counter-clockwise order. 0 when they touch or overlap. Corners given clockwise make the overlap test wrong. |

## Good to know

- **A dented shape is planned around as its hull**, which is on the safe side. The corners are stored in counter-clockwise order, and a polygon cannot be changed once made.
- **A grown shape always contains the exact one.** Every straight piece stays at least `r` from the original corner. With 30 degree steps the overshoot is at most 3.5 percent of `r`, which is 2.3 cm for the default robot's 0.651 m.
- **A route may run exactly along the edge of a grown obstacle**, because a point or line on the boundary does not count as blocked. That is what lets the shortest route hug the corners.
- **The pathfinder passes `distanceToPolygon`** the four corners of the robot's outline.

:::note
`ConvexPolygon` has no `equals` of its own. Two polygons with the same corners are different objects and do not compare equal.
:::

## More

- [Pathfinder](/concepts/pathfinder/): how the shapes are used.
- [FieldMap](/reference/plan/fieldmap/) and [Pathfinder](/reference/plan/pathfinder/): the classes that hold them.
- [Field file](/files/field-file/): where obstacles are written.
