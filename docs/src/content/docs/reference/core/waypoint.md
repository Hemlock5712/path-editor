---
title: Waypoint
description: A point the path passes through, with an optional tangent and heading.
sidebar:
  order: 3
---

`Waypoint` is a record for one point the path passes through. It holds the position, the direction of travel through the point, how strongly the curve keeps to that direction, and optionally the robot's heading there. [The curve](/concepts/the-curve/) explains how waypoints become a path.

Package: `frc.team5712.path.core`.

## Where you meet it

Waypoints are placed in the editor and stored in the [path file](/files/path-file/). You construct them in code only when you build a [PathSpec](/reference/core/pathspec/) by hand.

## Example

```java
Waypoint start = Waypoint.at(6.0, 2.0, Math.PI);          // leaves traveling toward -X
Waypoint middle = Waypoint.at(3.0, 1.0).withTangentScale(1.2);
Waypoint end = Waypoint.at(0.5, 1.0).withHeading(-Math.PI / 2.0);
```

## Components

| Component | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `xMeters` | `double` | m | Field x |
| `yMeters` | `double` | m | Field y |
| `tangentRadians` | `double` | rad | Direction of travel through the point. `NaN` means the library chooses it |
| `tangentScale` | `double` | | How long the curve keeps to the tangent. 1 is the default. Above 1 holds the direction longer, below 1 turns sooner |
| `headingRadians` | `double` | rad | The robot's heading here. `NaN` means a pass-through point with no heading set |

An automatic tangent at an interior point is the slope of the parabola through the point and its two neighbors. At the first or last point it is mirrored from the neighbor.

The heading is the direction the robot faces, which on a swerve is separate from the direction of travel. `Path.build` turns a waypoint heading into a rotation target at that waypoint. See [Heading](/concepts/heading/).

## Constructors

| Constructor | Description |
| --- | --- |
| `Waypoint(double xMeters, double yMeters, double tangentRadians, double tangentScale, double headingRadians)` | The full record constructor. |
| `Waypoint(double xMeters, double yMeters, double tangentRadians, double tangentScale)` | A pass-through waypoint. `headingRadians` is `NaN`. |

Both throw `IllegalArgumentException` when:

- `xMeters` or `yMeters` is not finite,
- `tangentScale` is not a finite number above 0,
- `tangentRadians` or `headingRadians` is infinite. `NaN` is allowed and means not set.

## Factories

| Method | Description |
| --- | --- |
| `static Waypoint at(double xMeters, double yMeters)` | A waypoint with an automatic tangent, a scale of 1 and no heading. |
| `static Waypoint at(double xMeters, double yMeters, double tangentRadians)` | A waypoint with a fixed direction of travel, a scale of 1 and no heading. |

## Methods

| Method | Returns |
| --- | --- |
| `Waypoint withTangentScale(double scale)` | The same point with a different tangent scale. |
| `Waypoint withHeading(double radians)` | The same point with the robot's heading set. `NaN` removes the heading. |
| `boolean hasTangent()` | True when `tangentRadians` is set. |
| `boolean hasHeading()` | True when `headingRadians` is set. |
| `Vec2 position()` | The position as a [Vec2](/reference/core/small-types/). |

:::note
In a path file the same values are `tangentDegrees` and `headingDegrees`, in degrees. In code they are radians.
:::
