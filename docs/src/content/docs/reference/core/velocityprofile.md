---
title: VelocityProfile
description: The planned speed, acceleration and time at every distance along the path.
sidebar:
  order: 10
---

`VelocityProfile` is the speed plan: the fastest speed along the path that breaks none of the robot's limits, as a function of distance. It is computed with one forward pass and one backward pass over a grid of points. [The speed plan](/concepts/speed-plan/) explains the passes and every limit.

Package: `frc.team5712.path.core`.

## Where you meet it

`Path.build` makes the profile, and `Path.profile()` returns it. The follower reads it every loop. Robot code that uses the library's commands does not call it, but it is the place to look when you want a path's planned speed for a log or a test.

## Example

```java
Path path = paths.get("ToReef");
VelocityProfile profile = path.profile();

double totalSeconds = profile.totalTimeSeconds();
double speedAtTwoMeters = profile.velocityAt(2.0);      // m/s
double whereAfterOneSecond = profile.sAtTime(1.0);      // meters along the path
```

## Constants

| Constant | Value | Meaning |
| --- | --- | --- |
| `DEFAULT_GRID_METERS` | `0.03` | The default grid spacing, in meters |
| `MIN_CELLS` | `16` | The fewest grid cells on any path, so that a short hop's turn is resolved |

## Building

Both methods are static.

| Method | Description |
| --- | --- |
| `VelocityProfile build(PathGeometry geometry, HeadingProfile heading, PathSpec spec, RobotConfig robot)` | Builds with `DEFAULT_GRID_METERS`. |
| `VelocityProfile build(PathGeometry geometry, HeadingProfile heading, PathSpec spec, RobotConfig robot, double gridMeters)` | Builds with a grid of about `gridMeters`. |

The path is split into `max(MIN_CELLS, ceil(length / gridMeters))` equal cells, so the spacing used is at most `gridMeters`. A grid point is added at each cusp of the curve, where the robot must stop. A cusp in the first or last cell, or within one cell of the previous cusp, does not get its own point.

From `spec` the build reads the path-wide maxima, the start and end speeds and the constraint zones. From `robot` it reads the traction limit, the motor model, the wheel-speed cap and the two spin caps.

`build` throws `IllegalStateException` when the plan cannot move through a cell, because the speed is zero at both of its ends. The message starts `the profile stalls at s =` and gives the distance in meters.

## Queries

All distances are meters along the path. All times are seconds from the start of the path.

| Method | Returns |
| --- | --- |
| `double lengthMeters()` | The path's length, in meters. |
| `double totalTimeSeconds()` | The planned time to drive the whole path. |
| `double velocityAt(double s)` | The planned speed along the path at `s`, in m/s. |
| `double accelerationAt(double s)` | The planned acceleration along the path at `s`, in m/s². It is constant across each grid cell and negative when braking. |
| `double timeAt(double s)` | The planned time to reach `s`. |
| `double sAtTime(double t)` | Where the plan is at time `t`, in meters along the path. A time of 0 or less gives 0. A time past the end gives the path's length. |

A distance outside the path is clamped to the nearest end by `velocityAt`, `accelerationAt` and `timeAt`.

## The grid

| Method | Returns |
| --- | --- |
| `double gridMeters()` | The uniform grid spacing actually used, in meters: the length divided into equal cells. |
| `int gridPointCount()` | The number of grid points, which is the number of cells plus one. Points added at cusps are counted. |
| `double gridVelocity(int i)` | The planned speed at grid point `i`, in m/s. |

On a path with no cusps, grid point `i` is at `i * gridMeters()`. On a path with cusps the extra points break that rule, so use `velocityAt` to read the speed at a distance.

## Diagnostics

| Method | Returns |
| --- | --- |
| `static long buildCount()` | How many profiles this JVM has built. |

The library's rule is that starting a path does no path work. Reading `buildCount()` before and after a start is how its tests check that. `PathGeometry.buildCount()` does the same for curves.

:::note
Between grid points the speed follows `v² = v0² + 2 * a * d`, with `a` constant in the cell. `velocityAt`, `timeAt` and `sAtTime` are exact for that model. They are not interpolated.
:::
