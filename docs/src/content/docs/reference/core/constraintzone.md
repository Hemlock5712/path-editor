---
title: ConstraintZone
description: A stretch of path with a lower speed cap, acceleration cap, or both.
sidebar:
  order: 6
---

`ConstraintZone` is a record for a stretch of path where the robot must go slower or accelerate more gently than the path-wide limits allow.

You draw zones in the editor, which stores them under `constraintZones` in the [path file](/files/path-file/). You construct one in code only when you build a [PathSpec](/reference/core/pathspec/) by hand.

Package: `frc.team5712.path.core`.

## Example

```java
ConstraintZone slow = new ConstraintZone(
    1.5, 2.0,                          // from halfway between waypoints 1 and 2, to waypoint 2
    1.5,                               // at most 1.5 m/s
    Double.POSITIVE_INFINITY);         // no extra acceleration cap

PathSpec careful = spec.withConstraintZones(List.of(slow));
```

## Constructors

| Constructor | What it does |
| --- | --- |
| `ConstraintZone(double fromWaypoint, double toWaypoint, double maxMetersPerSecond, double maxMetersPerSecondSquared)` | The record constructor. Throws `IllegalArgumentException` when `toWaypoint` is not greater than `fromWaypoint`, or when either limit is not above 0. |

## Components

| Component | What it is |
| --- | --- |
| `double fromWaypoint` | Where the zone starts, waypoint-relative: `1.5` is halfway along the arc between waypoints 1 and 2 |
| `double toWaypoint` | Where the zone ends, waypoint-relative |
| `double maxMetersPerSecond` | Speed cap inside the zone, in m/s |
| `double maxMetersPerSecondSquared` | Cap on speeding up and on braking inside the zone, in m/s² |

## Good to know

- **No extra limit is `Double.POSITIVE_INFINITY`.** Either limit may be that. A path file that leaves a limit out reads as `+∞`.
- **A zone can only lower a limit.** Where its cap is above the path's own cap or the robot's limits, the lower one applies. Where zones overlap, the lowest cap applies.
- **The robot is already slow when the zone starts.** A zone caps every speed-plan grid point of every grid cell it touches.
- **Positions outside the path** are clamped to its ends.

## More

- [The speed plan](/concepts/speed-plan/): how zones are applied.
- [PathSpec](/reference/core/pathspec/): where the zones go.
- [Path file](/files/path-file/): the `constraintZones` key.
