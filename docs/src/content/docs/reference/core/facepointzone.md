---
title: FacePointZone
description: A stretch of path where the robot faces a fixed point on the field.
sidebar:
  order: 5
---

`FacePointZone` is a record for a stretch of path where the robot faces a field point, such as a goal or a game piece, instead of following the rotation targets. The heading eases toward the point over the first `blendMeters` of the zone and eases back over the last `blendMeters`, so it never jumps. [Heading](/concepts/heading/) explains the blend.

Package: `frc.team5712.path.core`.

## Where you meet it

Face-point zones are drawn in the editor and stored under `facePointZones` in the [path file](/files/path-file/). You construct one in code only when you build a [PathSpec](/reference/core/pathspec/) by hand.

## Example

```java
FacePointZone aim = new FacePointZone(
    1.0, 2.0,                 // from waypoint 1 to waypoint 2
    new Vec2(0.0, 0.0),       // face the center of the field
    FacePointZone.DEFAULT_BLEND_METERS);

PathSpec aiming = spec.withFacePointZones(List.of(aim));
```

## Components

| Component | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `fromWaypoint` | `double` | waypoint-relative | Where the zone starts |
| `toWaypoint` | `double` | waypoint-relative | Where the zone ends |
| `point` | `Vec2` | m | The field point to face |
| `blendMeters` | `double` | m | Length of each ease. Clamped to half the zone's length when the heading is built |

Both ends are waypoint-relative positions: `1.5` is halfway along the arc between waypoints 1 and 2. Positions outside the path are clamped to its ends.

## Constructor

| Constructor | Description |
| --- | --- |
| `FacePointZone(double fromWaypoint, double toWaypoint, Vec2 point, double blendMeters)` | The record constructor. |

Throws `IllegalArgumentException` when `toWaypoint` is not greater than `fromWaypoint`, or when `blendMeters` is negative or not finite.

## Constants

| Constant | Value | Meaning |
| --- | --- | --- |
| `DEFAULT_BLEND_METERS` | `0.5` | The blend length, in meters, used when a path file leaves `blendMeters` out |

## Behavior to know

- A `blendMeters` of 0 switches to the facing direction with no ease, which is a jump in the planned heading at each end of the zone.
- If the path passes over the point itself, there is no direction to face, and the heading from the layer below is used at that spot.
- A short blend with a large change of heading is a fast turn. The [speed plan](/concepts/speed-plan/) slows the robot down for it.
- Several zones are applied in list order, each on top of the one before.
