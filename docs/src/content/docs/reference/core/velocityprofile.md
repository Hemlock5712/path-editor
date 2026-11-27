---
title: VelocityProfile
description: The planned speed, acceleration and time at every distance along the path.
sidebar:
  order: 10
---

`VelocityProfile` is the speed plan: the fastest speed along the path that breaks none of the robot's limits, at every distance.

Robot code that uses the library's commands does not call it. `Path.build` makes the profile and `Path.profile()` returns it, which is the place to look for a path's planned speed in a log or a test.

Package: `frc.team5712.path.core`.

## Example

```java
Path path = paths.get("ToReef");
VelocityProfile profile = path.profile();

double totalSeconds = profile.totalTimeSeconds();
double speedAtTwoMeters = profile.velocityAt(2.0);      // m/s
double whereAfterOneSecond = profile.sAtTime(1.0);      // meters along the path
```

## Constants

| Constant | What it is |
| --- | --- |
| `DEFAULT_GRID_METERS` | `0.03`. The default grid spacing, in meters |
| `MIN_CELLS` | `16`. The fewest grid cells on any path, so that a short hop's turn is resolved |

## Static builders

| Method | What it does |
| --- | --- |
| `VelocityProfile build(PathGeometry geometry, HeadingProfile heading, PathSpec spec, RobotConfig robot)` | Builds with `DEFAULT_GRID_METERS`. |
| `VelocityProfile build(PathGeometry geometry, HeadingProfile heading, PathSpec spec, RobotConfig robot, double gridMeters)` | Builds with a grid of about `gridMeters`. |

Both throw `IllegalStateException` when the plan cannot move through a cell, because the speed is zero at both of its ends. The message starts `the profile stalls at s =` and gives the distance in meters.

## What build reads

| Input | What it takes from it |
| --- | --- |
| `spec` | The path-wide maxima, the start and end speeds and the constraint zones |
| `robot` | The traction limit, the motor model, the wheel-speed cap and the two spin caps |
| `gridMeters` | The path is split into `max(MIN_CELLS, ceil(length / gridMeters))` equal cells, so the spacing used is at most `gridMeters` |
| Cusps of the curve | A grid point is added at each, where the robot must stop. A cusp in the first or last cell, or within one cell of the previous cusp, does not get its own point |

## Queries

Distances are meters along the path. Times are seconds from the start of the path.

| Method | What it does |
| --- | --- |
| `double lengthMeters()` | The path's length, in meters. |
| `double totalTimeSeconds()` | The planned time to drive the whole path. |
| `double velocityAt(double s)` | The planned speed along the path at `s`, in m/s. `s` is clamped to the nearest end. |
| `double accelerationAt(double s)` | The planned acceleration along the path at `s`, in m/s². It is constant across each grid cell and negative when braking. `s` is clamped to the nearest end. |
| `double timeAt(double s)` | The planned time to reach `s`. `s` is clamped to the nearest end. |
| `double sAtTime(double t)` | Where the plan is at time `t`, in meters along the path. A time of 0 or less gives 0. A time past the end gives the path's length. |

## The grid

| Method | What it does |
| --- | --- |
| `double gridMeters()` | The uniform grid spacing actually used, in meters: the length divided into equal cells. |
| `int gridPointCount()` | The number of grid points, which is the number of cells plus one. Points added at cusps are counted. |
| `double gridVelocity(int i)` | The planned speed at grid point `i`, in m/s. |

## Diagnostics

| Method | What it does |
| --- | --- |
| `static long buildCount()` | How many profiles this JVM has built. |

## Good to know

- **On a path with no cusps**, grid point `i` is at `i * gridMeters()`. On a path with cusps the extra points break that rule, so use `velocityAt` to read the speed at a distance.
- **Starting a path does no path work.** Reading `buildCount()` before and after a start is how the library's tests check that rule. `PathGeometry.buildCount()` does the same for curves.
- **The follower reads the profile every loop.**

:::note
Between grid points the speed follows `v² = v0² + 2 * a * d`, with `a` constant in the cell. `velocityAt`, `timeAt` and `sAtTime` are exact for that model. They are not interpolated.
:::

## More

- [The speed plan](/concepts/speed-plan/): the forward and backward passes, and every limit.
- [Path](/reference/core/path/): `Path.profile()` and `Path.sample`.
