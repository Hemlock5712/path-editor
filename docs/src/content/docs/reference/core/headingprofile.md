---
title: HeadingProfile
description: The robot's planned heading at every distance along the path.
sidebar:
  order: 9
---

`HeadingProfile` gives the robot's planned heading at every distance along the path, with its first two derivatives.

Robot code that follows paths with the library's commands never calls it. `Path.build` makes the profile and `Path.heading()` returns it, which is useful for logging or for checking a plan in a test.

Package: `frc.team5712.path.core`.

## Example

```java
Path path = paths.get("ToReef");
HeadingProfile heading = path.heading();

HeadingSample atTwoMeters = heading.sample(2.0);          // 2 meters along the path
double radians = atTwoMeters.radians();
double radiansPerMeter = atTwoMeters.radiansPerMeter();
```

## Static builders

| Method | What it does |
| --- | --- |
| `HeadingProfile build(PathGeometry geometry, List<RotationTarget> targets, List<FacePointZone> zones, HeadingMode mode)` | Builds the profile for a curve. |
| `HeadingProfile build(PathGeometry geometry, List<RotationTarget> targets, List<FacePointZone> zones)` | The same with `HeadingMode.FACE_TRAVEL`, which was the behavior before heading modes existed. |

## What build does with its inputs

| Input | What happens |
| --- | --- |
| `HeadingMode.HOLD` and no targets | One target is added at the start of the path with the direction the path starts out in. The robot holds that heading for the whole path |
| `HeadingMode.FACE_TRAVEL` and no targets | The heading is the direction of travel |
| Each target's place | Converted to meters. A waypoint-relative place or a distance outside the path is clamped to the path |
| The order of the targets | Sorted by distance. Targets at the same distance keep the order they were given in |
| Each target's heading | Unwrapped so that the turn from the previous target is the short way around |
| Each zone's `blendMeters` | Clamped to half the zone's length |
| Waypoint headings | Not turned into targets here. `Path.withWaypointHeadings` does that, and `Path.build` calls it first |

## Methods

| Method | What it does |
| --- | --- |
| `HeadingSample sample(double s)` | The heading at `s` meters along the path. `s` is clamped to the path. |
| `HeadingSample sample(GeometrySample g)` | The heading at a geometry sample you already have. This saves one lookup of the curve. |

## What a HeadingSample holds

| Value | What it is |
| --- | --- |
| `radians()` | The heading in rad, wrapped to the range above -π and up to π |
| `radiansPerMeter()` | `dθ/ds` in rad/m. The planned spin rate is `ω = v * dθ/ds` |
| `radiansPerMeterSquared()` | `d²θ/ds²` in rad/m². The planned spin acceleration is `α = a * dθ/ds + v² * d²θ/ds²` |

## Good to know

- **Two layers.** The base layer is the direction of travel with no targets, or a quintic blend between the targets, holding the first one before it and the last one after it. Face-point zones go on top, in list order.
- **The derivatives** are what let the speed plan budget for the spin.
- **For one combined lookup** of position, heading and speed, use `Path.sample` instead.

:::caution
Two targets at the same distance make the heading jump from one to the other at that point. The profile does not throw for this.
:::

## More

- [Heading](/concepts/heading/): the blend between targets and the spin rate at each one.
- [HeadingSample](/reference/core/samples/): the sample record.
