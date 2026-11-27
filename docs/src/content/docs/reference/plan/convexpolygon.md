---
title: ConvexPolygon
description: A convex polygon with the geometry tests the pathfinder needs.
sidebar:
  order: 6
---

`frc.team5712.path.plan.ConvexPolygon` is one obstacle: a **convex** polygon, meaning one with no dents, stored with its corners in counter-clockwise order. It cannot be changed once made. It carries the geometry the pathfinder is built on: growing the shape by a distance, testing whether a point is inside, whether a line passes through, and how far away something is.

Robot code does not normally use it. Obstacles come from `field.json` through `FieldMap`, and `PathLibrary` loads that at boot. You meet this class when building a `FieldMap` in code for a test, or when reading `Pathfinder.grownObstacles()` to draw what the planner sees. See [Pathfinding](/guides/pathfinding/) for the robot code and [Pathfinder](/concepts/pathfinder/) for how the shapes are used.

All coordinates and distances are in meters.

## Example

```java
// an L shape: the dent at (1, 1) is filled in by the hull
ConvexPolygon shape = ConvexPolygon.of(new double[][] {
    {0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}});

System.out.println(shape.size());                        // 5 corners, not 6
System.out.println(shape.containsStrictly(1.4, 1.4));    // true: inside the filled dent
System.out.println(shape.distanceTo(3.0, 0.5));          // 1.0

ConvexPolygon grown = shape.grow(0.651, Math.toRadians(30));
```

## Creating one

There is no public constructor.

### of(points)

```java
public static ConvexPolygon of(double[][] points)
```

Returns the **convex hull** of the points: the smallest convex polygon that contains them all, the shape of a rubber band stretched around them. Each point is `{x, y}` in meters. The points can be in any order, and points inside the hull or along its edges are dropped. A shape with a dent is therefore planned around as its hull, which is on the safe side.

Throws `IllegalArgumentException` with `"a polygon needs at least three distinct corners"` when the hull has fewer than three corners, for example when all the points are on one line.

## Corners

| Method | Returns | Description |
| --- | --- | --- |
| `size()` | `int` | The number of corners |
| `x(int i)` | `double` | The X of corner `i`, in meters. Corners are numbered from 0, counter-clockwise. |
| `y(int i)` | `double` | The Y of corner `i`, in meters |

`x` and `y` throw `ArrayIndexOutOfBoundsException` for an index outside `0` to `size() - 1`.

## Growing

### grow(r, maxStepRadians)

```java
public ConvexPolygon grow(double r, double maxStepRadians)
```

Returns the polygon grown outward by `r` meters on every side. The exact grown shape has round corners. This method draws each round corner as straight pieces that each turn at most `maxStepRadians`, and places their points slightly outside the true arc:

```text
corner point distance = r / cos(step / 2)
```

That keeps every straight piece at least `r` from the original corner, so the result always contains the exact grown shape. With 30 degree steps the overshoot is at most 3.5 percent of `r`, which is 2.3 cm for the default robot's 0.651 m.

## Tests

| Method | Returns | Description |
| --- | --- | --- |
| `containsStrictly(double x, double y)` | `boolean` | True when the point is inside the polygon and more than 0.0000001 m from every edge. A point on the boundary is not inside. |
| `blocksSegment(double x0, double y0, double x1, double y1)` | `boolean` | True when the straight segment between the two points passes through the inside of the polygon. Touching a corner or running along an edge does not count. |

Because a point or line on the boundary does not count as blocked, a route may run exactly along the edge of a grown obstacle. That is what lets the shortest route hug the corners.

## Distances

| Method | Returns | Description |
| --- | --- | --- |
| `distanceTo(double x, double y)` | `double` | The distance in meters from the point to the polygon. 0 when the point is inside or on the boundary. |
| `nearestBoundaryPoint(double x, double y)` | `double[]` | The closest point on the boundary as `{x, y}`, whether the given point is inside or outside |
| `distanceToPolygon(double[] qx, double[] qy)` | `double` | The distance in meters between this polygon and another convex polygon. 0 when they touch or overlap. |

For `distanceToPolygon`, the other polygon is given as two arrays, the X and the Y of its corners, in counter-clockwise order. The pathfinder passes the four corners of the robot's outline. Corners given clockwise make the overlap test wrong.

:::note
`ConvexPolygon` has no `equals` of its own. Two polygons with the same corners are different objects and do not compare equal.
:::

## Related

- [FieldMap](/reference/plan/fieldmap/)
- [Pathfinder](/reference/plan/pathfinder/)
- [Field file](/files/field-file/)
