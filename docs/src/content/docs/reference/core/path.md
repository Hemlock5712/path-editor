---
title: Path
description: A built path, with its curve, heading plan and speed plan, ready to follow.
sidebar:
  order: 1
---

`Path` is a path that is ready to follow. It holds the curve, the heading plan and the speed plan, all built once from a [PathSpec](/reference/core/pathspec/) and a [RobotConfig](/reference/core/robotconfig/). After it is built it is only read. It is immutable and safe to share between threads.

Package: `frc.team5712.path.core`.

## Where you meet it

Most robot code never builds a `Path` by hand. `PathLibrary.loadAll()` builds one for every file in `deploy/paths/` at boot, and the commands take a path by name. See [Follow a path](/guides/follow-a-path/). You meet `Path` directly when you want to read the plan, for example to log a path's length or its planned time.

## Example

```java
Path path = paths.get("ToReef");

double meters = path.lengthMeters();
double seconds = path.totalTimeSeconds();

PathSample halfway = path.sample(meters / 2.0);
Pose plannedPose = halfway.pose();
double plannedSpeed = halfway.metersPerSecond();
```

`paths` is the `PathLibrary`. To build a path yourself from a spec:

```java
PathSpec spec = PathSpec.of(List.of(Waypoint.at(6.0, 2.0), Waypoint.at(3.0, 1.0)));
Path built = Path.build(spec, RobotConfig.DEFAULT);
```

## Building

All three are static. Each one first applies `withWaypointHeadings` and then `pureRotation` to the spec, then builds the [PathGeometry](/reference/core/pathgeometry/), the [HeadingProfile](/reference/core/headingprofile/) and the [VelocityProfile](/reference/core/velocityprofile/) in that order.

| Method | Description |
| --- | --- |
| `Path build(PathSpec spec, RobotConfig robot)` | Builds with the default grid, `VelocityProfile.DEFAULT_GRID_METERS`, 0.03 m. |
| `Path build(PathSpec spec, RobotConfig robot, double gridMeters)` | Builds with a speed-plan grid of `gridMeters`. |
| `Path build(PathSpec spec, RobotConfig robot, double gridMeters, double startCurvature)` | Also fixes the curvature at the first waypoint, in 1/m. `NaN` leaves it free. Used for a tail that must join an existing path smoothly. |

Building throws `IllegalArgumentException` if two neighboring waypoints are at the same place, apart from the turn-on-the-spot case below. It throws `IllegalStateException` if the speed plan cannot move, with a message that starts `the profile stalls at s =`.

## Spec helpers

| Member | Description |
| --- | --- |
| `static PathSpec withWaypointHeadings(PathSpec spec)` | Returns a spec in which each waypoint heading has become a rotation target at that waypoint, placed ahead of the authored targets, and the waypoints carry no headings. A spec with no waypoint headings is returned unchanged. |
| `static PathSpec pureRotation(PathSpec spec)` | If the spec has exactly two waypoints at the same place, within 1e-9 m, returns a spec for a turn on the spot. Any other spec is returned unchanged. |
| `static final double PURE_ROTATION_METERS` | `0.001`. Length in meters of the segment a turn on the spot is planned on. |

A turn on the spot is planned on a 1 mm straight segment that ends exactly on the point. The segment points along the last rotation target's heading, or along 0 radians if there are no targets. Its start and end speeds are set to 0. The spin limits then set how long the turn takes, and the robot moves at most 1 mm.

`pureRotation` builds new waypoints with no headings, so call `withWaypointHeadings` first if the spec has headings on its waypoints. `build` does this for you.

## Reading the plan

| Method | Returns |
| --- | --- |
| `PathSpec spec()` | The spec the path was built from, after `withWaypointHeadings` and `pureRotation`. |
| `PathGeometry geometry()` | The curve, indexed by distance. |
| `HeadingProfile heading()` | The planned heading. |
| `VelocityProfile profile()` | The speed plan. |
| `double lengthMeters()` | The path's length in meters. |
| `double[] cusps()` | Where the path doubles back, in meters along it, ascending. Empty for most paths. The speed plan stops the robot at each. |
| `double totalTimeSeconds()` | The planned time to drive the whole path, in seconds. |
| `PathSample sample(double s)` | The whole plan at `s` meters along the path. `s` is clamped to the path. |
| `PathSample sampleAtTime(double t)` | The plan at `t` seconds after the start. A time past the end gives the end of the path. |

[PathSample](/reference/core/samples/) is described on the Sample records page.
