---
title: ConstraintZone
description: A stretch of path with a lower speed cap, acceleration cap, or both.
sidebar:
  order: 6
---

`ConstraintZone` is a record for a stretch of path where the robot must go slower or accelerate more gently than the path-wide limits allow. A slow stretch close to an obstacle is a typical use. [The speed plan](/concepts/speed-plan/) explains how zones are applied.

Package: `frc.team5712.path.core`.

## Where you meet it

Constraint zones are drawn in the editor and stored under `constraintZones` in the [path file](/files/path-file/). You construct one in code only when you build a [PathSpec](/reference/core/pathspec/) by hand.

## Example

```java
ConstraintZone slow = new ConstraintZone(
    1.5, 2.0,                          // from halfway between waypoints 1 and 2, to waypoint 2
    1.5,                               // at most 1.5 m/s
    Double.POSITIVE_INFINITY);         // no extra acceleration cap

PathSpec careful = spec.withConstraintZones(List.of(slow));
```

## Components

| Component | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `fromWaypoint` | `double` | waypoint-relative | Where the zone starts |
| `toWaypoint` | `double` | waypoint-relative | Where the zone ends |
| `maxMetersPerSecond` | `double` | m/s | Speed cap inside the zone |
| `maxMetersPerSecondSquared` | `double` | m/s² | Cap on speeding up and on braking inside the zone |

Either limit may be `Double.POSITIVE_INFINITY`, which means no extra limit. A path file that leaves a limit out reads as `+∞`.

Both ends are waypoint-relative positions: `1.5` is halfway along the arc between waypoints 1 and 2. Positions outside the path are clamped to its ends.

## Constructor

| Constructor | Description |
| --- | --- |
| `ConstraintZone(double fromWaypoint, double toWaypoint, double maxMetersPerSecond, double maxMetersPerSecondSquared)` | The record constructor. |

Throws `IllegalArgumentException` when `toWaypoint` is not greater than `fromWaypoint`, or when either limit is not above 0.

## Behavior to know

- A zone can only lower a limit. Where a zone's cap is above the path's own cap or the robot's limits, the lower one applies.
- A zone caps every speed-plan grid point of every grid cell it touches. The robot is already down to the zone's speed when the zone starts.
- Where zones overlap, the lowest cap applies.
