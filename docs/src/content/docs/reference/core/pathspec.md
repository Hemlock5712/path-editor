---
title: PathSpec
description: Everything a path file says, before any math is done.
sidebar:
  order: 2
---

`PathSpec` is a record that holds the authored inputs of a path: the waypoints, the heading inputs, the zones, the events and the path-wide limits.

Most students never construct one, because paths drawn in the editor reach robot code already built. You need one to make a path in code, or to change a loaded path before building it again.

Package: `frc.team5712.path.core`.

## Example

```java
PathSpec spec = PathSpec.of(List.of(
        Waypoint.at(6.0, 2.0),
        Waypoint.at(3.0, 1.0),
        Waypoint.at(0.5, 1.0)))
    .withLimits(3.0, 4.0)                 // 3 m/s and 4 m/s² for the whole path
    .withRotationTargets(List.of(RotationTarget.atWaypoint(2.0, Math.PI)));

Path path = Path.build(spec, RobotConfig.DEFAULT);
```

## Constructors

| Constructor | What it fills in |
| --- | --- |
| `PathSpec(waypoints, rotationTargets, facePointZones, constraintZones, maxMetersPerSecond, maxMetersPerSecondSquared, startMetersPerSecond, endMetersPerSecond, events, headingMode, startFootprint)` | Nothing. This is the full record constructor |
| The same without `startFootprint` | `startFootprint` is `Footprint.BUMPERS` |
| The same without `headingMode` and `startFootprint` | Also `headingMode` is `HeadingMode.HOLD` |
| The same without `events`, `headingMode` and `startFootprint` | Also `events` is empty |

Every constructor throws `IllegalArgumentException` when:

| Argument | Is wrong when |
| --- | --- |
| `headingMode` | It is null |
| `startFootprint` | It is null or blank |
| `waypoints` | There are fewer than two |
| `maxMetersPerSecond`, `maxMetersPerSecondSquared` | Not above 0 |
| `startMetersPerSecond`, `endMetersPerSecond` | Below 0 or `NaN` |

## Static factories

| Method | What it does |
| --- | --- |
| `static PathSpec of(List<Waypoint> waypoints)` | A path through these waypoints with no targets, zones, events or path-wide caps, from rest to rest, with `HeadingMode.HOLD` and the bumpers footprint. |

## Components

In record order. Each has an accessor of the same name.

| Component | What it is |
| --- | --- |
| `List<Waypoint> waypoints` | The points the path passes through. At least two |
| `List<RotationTarget> rotationTargets` | Headings to reach along the path |
| `List<FacePointZone> facePointZones` | Stretches where the robot faces a point |
| `List<ConstraintZone> constraintZones` | Stretches with lower limits |
| `double maxMetersPerSecond` | Path-wide speed cap, in m/s. `+∞` means only the robot's limits apply |
| `double maxMetersPerSecondSquared` | Path-wide cap on speeding up and braking, in m/s². `+∞` means only grip and motors apply |
| `double startMetersPerSecond` | Speed along the path at the start, in m/s |
| `double endMetersPerSecond` | Speed along the path at the end, in m/s. Above 0 is a rolling hand-off |
| `List<PathEvent> events` | Named events, in file order. The follower fires them in path order |
| `HeadingMode headingMode` | What the heading does where none is set |
| `String startFootprint` | The robot's outline at the start. `Footprint.BUMPERS` unless a mechanism is already out |

## Changed copies

Each returns a new spec with one part replaced and everything else kept.

| Method | What it replaces |
| --- | --- |
| `PathSpec withWaypoints(List<Waypoint> newWaypoints)` | The waypoints |
| `PathSpec withRotationTargets(List<RotationTarget> targets)` | The rotation targets |
| `PathSpec withFacePointZones(List<FacePointZone> zones)` | The face-point zones |
| `PathSpec withConstraintZones(List<ConstraintZone> zones)` | The constraint zones |
| `PathSpec withLimits(double maxSpeed, double maxAccel)` | `maxMetersPerSecond` and `maxMetersPerSecondSquared`, in m/s and m/s² |
| `PathSpec withEndSpeeds(double startSpeed, double endSpeed)` | `startMetersPerSecond` and `endMetersPerSecond`, in m/s |
| `PathSpec withEvents(List<PathEvent> newEvents)` | The events |
| `PathSpec withHeadingMode(HeadingMode mode)` | The heading mode |
| `PathSpec withStartFootprint(String footprint)` | The start footprint |

## Good to know

- **A spec is immutable.** Its `with` methods return a changed copy.
- **The lists are copied** when the record is made, so later changes to the list you passed in do not reach the spec.
- **A `with` method can throw `IllegalArgumentException`.** The same checks as the constructors apply.
- **Where specs come from and go.** [PathFile](/reference/core/pathfile/) produces one when it reads a `.path.json` file, and `Path.build` turns one into a [Path](/reference/core/path/).

## More

- [Waypoint](/reference/core/waypoint/), [RotationTarget](/reference/core/rotationtarget/), [FacePointZone](/reference/core/facepointzone/), [ConstraintZone](/reference/core/constraintzone/), [PathEvent](/reference/core/pathevent/), [HeadingMode](/reference/core/headingmode/): the parts of a spec.
- [Path file](/files/path-file/): the same data as JSON.
