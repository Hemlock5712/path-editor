---
title: HeadingProfile
description: The robot's planned heading at every distance along the path.
sidebar:
  order: 9
---

`HeadingProfile` gives the robot's planned heading as a function of distance along the path, together with its first two derivatives. The derivatives are what let the speed plan budget for the spin. It is built from the rotation targets, the face-point zones and the heading mode. [Heading](/concepts/heading/) explains how.

Package: `frc.team5712.path.core`.

## Where you meet it

`Path.build` makes the profile, and `Path.heading()` returns it. Robot code that follows paths with the library's commands never calls it. Reading it is useful for logging or for checking a plan in a test. For one combined lookup of position, heading and speed, use `Path.sample` instead.

## Example

```java
Path path = paths.get("ToReef");
HeadingProfile heading = path.heading();

HeadingSample atTwoMeters = heading.sample(2.0);
double radians = atTwoMeters.radians();
double radiansPerMeter = atTwoMeters.radiansPerMeter();
```

## Building

Both methods are static.

| Method | Description |
| --- | --- |
| `HeadingProfile build(PathGeometry geometry, List<RotationTarget> targets, List<FacePointZone> zones, HeadingMode mode)` | Builds the profile for a curve. |
| `HeadingProfile build(PathGeometry geometry, List<RotationTarget> targets, List<FacePointZone> zones)` | The same with `HeadingMode.FACE_TRAVEL`, which was the behavior before heading modes existed. |

What `build` does with its inputs:

- With `HeadingMode.HOLD` and no targets, it adds one target at the start of the path with the direction the path starts out in. The robot holds that heading for the whole path.
- With `HeadingMode.FACE_TRAVEL` and no targets, the heading is the direction of travel.
- Each target's place is converted to meters. A waypoint-relative place or a distance outside the path is clamped to the path.
- Targets are sorted by distance. Targets at the same distance keep the order they were given in.
- Each target's heading is unwrapped so that the turn from the previous target is the short way around.
- Each zone's `blendMeters` is clamped to half the zone's length.

`build` does not turn waypoint headings into targets. `Path.withWaypointHeadings` does that, and `Path.build` calls it first.

## Sampling

| Method | Returns |
| --- | --- |
| `HeadingSample sample(double s)` | The heading at `s` meters along the path. `s` is clamped to the path. |
| `HeadingSample sample(GeometrySample g)` | The heading at a geometry sample you already have. This saves one lookup of the curve. |

A [HeadingSample](/reference/core/samples/) holds three values:

| Value | Unit | Meaning |
| --- | --- | --- |
| `radians()` | rad | The heading, wrapped to the range above -π and up to π |
| `radiansPerMeter()` | rad/m | `dθ/ds`. The planned spin rate is `ω = v * dθ/ds` |
| `radiansPerMeterSquared()` | rad/m² | `d²θ/ds²`. The planned spin acceleration is `α = a * dθ/ds + v² * d²θ/ds²` |

## The layers

A sample is built in two layers.

1. **Base.** With no targets, the direction of travel. With targets: the first target's heading before it, the last target's heading after it, and a quintic blend between each pair.
2. **Face-point zones**, in list order. Inside a zone the heading is pulled toward the direction from the path point to the zone's point, eased in and out over the zone's blend.

At the first and last target, and where the turn reverses, the spin rate `dθ/ds` is zero. Where the turn carries on the same way through a target, the rate there is the harmonic mean of the average rates on each side. `d²θ/ds²` is zero at every target, so both derivatives are continuous along the base layer.

:::caution
Two targets at the same distance make the heading jump from one to the other at that point. The profile does not throw for this.
:::
