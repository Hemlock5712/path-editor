---
title: Sample records
description: The four records that carry one lookup of a path, its heading or a projection.
sidebar:
  order: 13
---

A lookup on a path returns a small immutable record with everything known at that place: `GeometrySample`, `HeadingSample`, `PathSample` or `Projection`.

The follower reads these every loop. In robot code you get a `PathSample` from `Path.sample` to log a plan or check one in a test. The other three come from [PathGeometry](/reference/core/pathgeometry/) and [HeadingProfile](/reference/core/headingprofile/).

Package: `frc.team5712.path.core`. Each component has an accessor of the same name, such as `sample.curvature()`.

## Example

```java
Path path = paths.get("ToReef");

PathSample plan = path.sampleAtTime(1.0);          // the plan one second in
Pose pose = plan.pose();
FieldSpeeds speeds = plan.fieldSpeeds();

// The closest point on the path to that pose
Projection nearest = path.geometry().projectGlobal(pose.xMeters(), pose.yMeters());
double metersAlong = nearest.s();
```

## GeometrySample

The curve at one distance along it. Returned by `PathGeometry.sample`.

| Component | What it is |
| --- | --- |
| `double s` | Distance along the path, in m |
| `double xMeters` | Field x, in m |
| `double yMeters` | Field y, in m |
| `double tangentX` | The x part of the unit direction of travel |
| `double tangentY` | The y part of the unit direction of travel |
| `double curvature` | Signed curvature, in 1/m. Positive is turning left |
| `double dCurvatureDs` | The rate of change of curvature along the path, in 1/m² |

| Method | What it does |
| --- | --- |
| `double tangentRadians()` | The direction of travel, in radians. |

## HeadingSample

The planned robot heading at one distance along the path. Returned by `HeadingProfile.sample`. It has no methods beyond its accessors.

| Component | What it is |
| --- | --- |
| `double radians` | The heading in rad, wrapped to the range above -π and up to π |
| `double radiansPerMeter` | `dθ/ds` in rad/m. The planned spin rate is `ω = v * dθ/ds` |
| `double radiansPerMeterSquared` | `d²θ/ds²` in rad/m². The planned spin acceleration is `α = a * dθ/ds + v² * d²θ/ds²` |

## PathSample

Everything the plan says about one distance along the path. Returned by `Path.sample` and `Path.sampleAtTime`.

| Component | What it is |
| --- | --- |
| `double s` | Distance along the path, in m |
| `double xMeters` | Field x, in m |
| `double yMeters` | Field y, in m |
| `double tangentX` | The x part of the unit direction of travel |
| `double tangentY` | The y part of the unit direction of travel |
| `double curvature` | Signed curvature, in 1/m |
| `double dCurvatureDs` | The rate of change of curvature along the path, in 1/m² |
| `double headingRadians` | The planned robot heading in rad, wrapped to the range above -π and up to π |
| `double headingRadiansPerMeter` | `dθ/ds`, in rad/m |
| `double metersPerSecond` | The planned speed along the path, in m/s |
| `double metersPerSecondSquared` | The planned acceleration along the path, in m/s² |
| `double seconds` | The planned time to get here from the start, in s |

| Method | What it does |
| --- | --- |
| `FieldSpeeds fieldSpeeds()` | The planned field velocity: the speed along the tangent, and a spin of `metersPerSecond * headingRadiansPerMeter`. |
| `Pose pose()` | The planned pose: `xMeters`, `yMeters` and `headingRadians`. |

## Projection

The closest point on the path to a query point. Returned by the `project` methods of `PathGeometry`. It has no methods beyond its accessors.

| Component | What it is |
| --- | --- |
| `double s` | Distance along the path of the closest point, in m |
| `double xMeters` | The closest point, x, in m |
| `double yMeters` | The closest point, y, in m |
| `double distanceMeters` | The distance from the query point to the closest point, in m. Never negative |
| `double crossTrackMeters` | The signed distance, in m. Positive when the query point is left of the path, looking along the direction of travel |

:::note
A path is planned in the blue frame. On the red alliance, the poses and speeds in these records are still blue-frame values. See [The field frame](/concepts/field-frame/).
:::

## More

- [Pose and FieldSpeeds](/reference/core/small-types/): the library's own small types.
- [Conversions](/reference/wpilib/conversions/): turns them into WPILib types.
- [PathGeometry](/reference/core/pathgeometry/): where `GeometrySample` and `Projection` come from.
