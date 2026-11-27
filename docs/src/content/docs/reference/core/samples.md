---
title: Sample records
description: The four records that carry one lookup of a path, its heading or a projection.
sidebar:
  order: 13
---

A lookup on a path returns a small record with everything known at that place. There are four: `GeometrySample` for the curve, `HeadingSample` for the heading, `PathSample` for the whole plan, and `Projection` for the closest point to a position. All four are immutable records in `frc.team5712.path.core`. Each component has an accessor of the same name, such as `sample.curvature()`.

## Where you meet them

The follower reads these every loop. In robot code you get a `PathSample` from `Path.sample` when you want to log a plan or check one in a test. The other three come from [PathGeometry](/reference/core/pathgeometry/) and [HeadingProfile](/reference/core/headingprofile/).

## Example

```java
Path path = paths.get("ToReef");

PathSample plan = path.sampleAtTime(1.0);          // the plan one second in
Pose pose = plan.pose();
FieldSpeeds speeds = plan.fieldSpeeds();

Projection nearest = path.geometry().projectGlobal(pose.xMeters(), pose.yMeters());
double metersAlong = nearest.s();
```

## GeometrySample

The curve at one distance along it. Returned by `PathGeometry.sample`.

| Component | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `s` | `double` | m | Distance along the path |
| `xMeters` | `double` | m | Field x |
| `yMeters` | `double` | m | Field y |
| `tangentX` | `double` | | The x part of the unit direction of travel |
| `tangentY` | `double` | | The y part of the unit direction of travel |
| `curvature` | `double` | 1/m | Signed curvature. Positive is turning left |
| `dCurvatureDs` | `double` | 1/m² | The rate of change of curvature along the path |

| Method | Returns |
| --- | --- |
| `double tangentRadians()` | The direction of travel, in radians. |

## HeadingSample

The planned robot heading at one distance along the path. Returned by `HeadingProfile.sample`.

| Component | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `radians` | `double` | rad | The heading, wrapped to the range above -π and up to π |
| `radiansPerMeter` | `double` | rad/m | `dθ/ds`. The planned spin rate is `ω = v * dθ/ds` |
| `radiansPerMeterSquared` | `double` | rad/m² | `d²θ/ds²`. The planned spin acceleration is `α = a * dθ/ds + v² * d²θ/ds²` |

It has no methods beyond its accessors.

## PathSample

Everything the plan says about one distance along the path. Returned by `Path.sample` and `Path.sampleAtTime`.

| Component | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `s` | `double` | m | Distance along the path |
| `xMeters` | `double` | m | Field x |
| `yMeters` | `double` | m | Field y |
| `tangentX` | `double` | | The x part of the unit direction of travel |
| `tangentY` | `double` | | The y part of the unit direction of travel |
| `curvature` | `double` | 1/m | Signed curvature |
| `dCurvatureDs` | `double` | 1/m² | The rate of change of curvature along the path |
| `headingRadians` | `double` | rad | The planned robot heading, wrapped to the range above -π and up to π |
| `headingRadiansPerMeter` | `double` | rad/m | `dθ/ds` |
| `metersPerSecond` | `double` | m/s | The planned speed along the path |
| `metersPerSecondSquared` | `double` | m/s² | The planned acceleration along the path |
| `seconds` | `double` | s | The planned time to get here from the start |

| Method | Returns |
| --- | --- |
| `FieldSpeeds fieldSpeeds()` | The planned field velocity: the speed along the tangent, and a spin of `metersPerSecond * headingRadiansPerMeter`. |
| `Pose pose()` | The planned pose: `xMeters`, `yMeters` and `headingRadians`. |

[Pose and FieldSpeeds](/reference/core/small-types/) are the library's own small types. Convert them to WPILib types with [Conversions](/reference/wpilib/conversions/).

## Projection

The closest point on the path to a query point. Returned by the `project` methods of `PathGeometry`.

| Component | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `s` | `double` | m | Distance along the path of the closest point |
| `xMeters` | `double` | m | The closest point, x |
| `yMeters` | `double` | m | The closest point, y |
| `distanceMeters` | `double` | m | The distance from the query point to the closest point. Never negative |
| `crossTrackMeters` | `double` | m | The signed distance. Positive when the query point is left of the path, looking along the direction of travel |

It has no methods beyond its accessors.

:::note
A path is planned in the blue frame. On the red alliance, the poses and speeds in these records are still blue-frame values. See [The field frame](/concepts/field-frame/).
:::
