---
title: PathGeometry
description: The path's shape indexed by distance, and the closest point to the robot.
sidebar:
  order: 12
---

`PathGeometry` is the path's shape, indexed by distance along it in meters. The spline from [QuinticSpline](/reference/core/quinticspline/) does not know how long it is. This class measures it, so that the rest of the library can ask for the point 2.4 m along the path, the direction of travel there, and how sharply the path bends. It also finds the point on the path closest to the robot. [The curve](/concepts/the-curve/) explains how.

Package: `frc.team5712.path.core`.

## Where you meet it

`Path.build` makes the geometry, and `Path.geometry()` returns it. The follower uses its projection methods every loop. Robot code built on the library's commands does not call it. It is useful when you want to draw a path, or to find how far along a path a field position is.

## Example

```java
PathGeometry geometry = paths.get("ToReef").geometry();

GeometrySample middle = geometry.sample(geometry.lengthMeters() / 2.0);
double curvature = middle.curvature();                     // 1/m

Projection nearest = geometry.projectGlobal(4.0, 1.5);     // closest point to (4.0, 1.5)
double metersAlong = nearest.s();
double metersOff = nearest.crossTrackMeters();             // positive is left of the path
```

## Constants

| Constant | Value | Meaning |
| --- | --- | --- |
| `LOOKUP_TOLERANCE_METERS` | `1e-5` | The worst position error of a lookup by distance, measured at the middle of each table piece while building. This is 0.01 mm |
| `CUSP_SCAN` | `64` | The number of samples per spline piece when looking for cusps |
| `CUSP_ARC_METERS` | `0.02` | A cusp turns the tangent by more than 120 degrees between this far before it and this far after it |

## Building

All three are static.

| Method | Description |
| --- | --- |
| `PathGeometry build(List<Waypoint> waypoints)` | Builds the spline through the waypoints and its arc length table. |
| `PathGeometry build(List<Waypoint> waypoints, double startCurvature)` | The same with the curvature at the first waypoint fixed, in 1/m. `NaN` leaves it free. |
| `PathGeometry build(QuinticSpline spline)` | Builds the arc length table for a spline you already have. |

The two that take waypoints throw `IllegalArgumentException` for the same reasons as `QuinticSpline.through`: fewer than two waypoints, or two neighbors at the same place.

## Size and waypoints

| Method | Returns |
| --- | --- |
| `QuinticSpline spline()` | The spline under the table. |
| `double lengthMeters()` | The total length, in meters. |
| `int waypointCount()` | The number of waypoints. |
| `double waypointS(int i)` | The distance along the path at waypoint `i`, in meters. |
| `double sAtWaypoint(double waypoint)` | The distance along the path at a waypoint-relative position. `1.5` is halfway along the arc from waypoint 1 to waypoint 2. Clamped to the path. |
| `int tablePieceCount()` | The number of sub-pieces in the arc length table. For tests and tuning. |

## Sampling

| Method | Returns |
| --- | --- |
| `GeometrySample sample(double s)` | The path at `s` meters along it: point, unit tangent, curvature and its rate of change. |
| `Vec2 pointAt(double s)` | The point at `s`. |
| `double curvatureAt(double s)` | The signed curvature at `s`, in 1/m. Positive is turning left. |

`s` is clamped to the path in all three. [GeometrySample](/reference/core/samples/) is described on the Sample records page.

## Projection

Each method returns a [Projection](/reference/core/samples/): the closest point on a stretch of the path to the field point `(xMeters, yMeters)`. The stretches differ.

| Method | Searches | Coarse scan step |
| --- | --- | --- |
| `Projection project(double xMeters, double yMeters, double sGuess, double halfWindowMeters)` | From `sGuess - halfWindowMeters` to `sGuess + halfWindowMeters` | 0.05 m |
| `Projection projectWithin(double xMeters, double yMeters, double lo, double hi)` | From `lo` to `hi` | 0.05 m |
| `Projection projectGlobalWithin(double xMeters, double yMeters, double lo, double hi)` | From `lo` to `hi` | 0.10 m |
| `Projection projectGlobal(double xMeters, double yMeters)` | The whole path | 0.10 m |

All four clamp the stretch to the path. A coarse scan finds the nearest sample, and Newton's method with a bisection safeguard then refines it. The answer never leaves the stretch.

Use `project` to track progress from one loop to the next. A path that passes near itself cannot make the answer jump to the other branch, because the search stays near the last known distance. If `sGuess` is so far off the path that the window misses it, the nearer end of the path is returned. Use `projectGlobal` when starting up or after losing track. `projectWithin` and `projectGlobalWithin` are for a window that must stop at a cusp.

## Cusps

A cusp is a place where the path stops and doubles back. They are found once, at build.

| Method | Returns |
| --- | --- |
| `double[] cusps()` | The cusps and near-cusps, in meters along the path, ascending. Empty for most paths. The array is a copy. |
| `boolean hasCusps()` | True when the path has one or more cusps. |
| `double nextCusp(double s)` | The first cusp more than a micrometer after `s`, or `NaN` if there is none. |
| `double previousCusp(double s)` | The last cusp at or before `s`, within a micrometer, or `NaN` if there is none. |

## Diagnostics

| Method | Returns |
| --- | --- |
| `static long buildCount()` | How many geometries this JVM has built. Read it before and after starting a path to confirm that starting does no path work. |
