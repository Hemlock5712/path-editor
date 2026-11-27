---
title: FacePointZone
description: A stretch of path where the robot faces a fixed point on the field.
sidebar:
  order: 5
---

`FacePointZone` is a record for a stretch of path where the robot faces a field point, such as a goal or a game piece, instead of following the rotation targets.

You draw zones in the editor, which stores them under `facePointZones` in the [path file](/files/path-file/). You construct one in code only when you build a [PathSpec](/reference/core/pathspec/) by hand.

Package: `frc.team5712.path.core`.

## Example

```java
FacePointZone aim = new FacePointZone(
    1.0, 2.0,                 // from waypoint 1 to waypoint 2
    new Vec2(0.0, 0.0),       // face the center of the field
    FacePointZone.DEFAULT_BLEND_METERS);

PathSpec aiming = spec.withFacePointZones(List.of(aim));
```

## Constructors

| Constructor | What it does |
| --- | --- |
| `FacePointZone(double fromWaypoint, double toWaypoint, Vec2 point, double blendMeters)` | The record constructor. Throws `IllegalArgumentException` when `toWaypoint` is not greater than `fromWaypoint`, or when `blendMeters` is negative or not finite. |

## Components

| Component | What it is |
| --- | --- |
| `double fromWaypoint` | Where the zone starts, waypoint-relative: `1.5` is halfway along the arc between waypoints 1 and 2 |
| `double toWaypoint` | Where the zone ends, waypoint-relative |
| `Vec2 point` | The field point to face, in meters |
| `double blendMeters` | Length of each ease, in meters. Clamped to half the zone's length when the heading is built |

## Constants

| Constant | What it is |
| --- | --- |
| `DEFAULT_BLEND_METERS` | `0.5`. The blend length, in meters, used when a path file leaves `blendMeters` out |

## Good to know

- **The heading never jumps.** It eases toward the point over the first `blendMeters` of the zone and eases back over the last `blendMeters`. A `blendMeters` of 0 switches with no ease, which is a jump in the planned heading at each end of the zone.
- **A short blend with a large change of heading is a fast turn.** The [speed plan](/concepts/speed-plan/) slows the robot down for it.
- **If the path passes over the point itself**, there is no direction to face, and the heading from the layer below is used at that spot.
- **Several zones** are applied in list order, each on top of the one before. Positions outside the path are clamped to its ends.

## More

- [Heading](/concepts/heading/): how the blend works.
- [PathSpec](/reference/core/pathspec/): where the zones go.
- [Path file](/files/path-file/): the `facePointZones` key.
