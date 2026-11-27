---
title: RotationTarget
description: A robot heading to reach at a place on the path.
sidebar:
  order: 4
---

`RotationTarget` is a record for a heading the robot should have at one place on the path.

You add rotation targets in the editor, which stores them under `rotationTargets` in the [path file](/files/path-file/). You construct one in code only when you build a [PathSpec](/reference/core/pathspec/) by hand.

Package: `frc.team5712.path.core`.

## Example

```java
List<RotationTarget> targets = List.of(
    RotationTarget.atWaypoint(0.0, Math.PI),       // facing -X at the first waypoint
    RotationTarget.atMeters(2.5, -Math.PI / 2.0)); // facing -Y 2.5 m along the path

PathSpec turning = spec.withRotationTargets(targets);
```

## Constructors

| Constructor | What it does |
| --- | --- |
| `RotationTarget(double waypoint, double meters, double headingRadians)` | The record constructor. Pass `NaN` for the place you are not using. Throws `IllegalArgumentException` when both places are set, when neither is set, or when `headingRadians` is not finite. |

## Static factories

| Method | What it does |
| --- | --- |
| `static RotationTarget atWaypoint(double waypoint, double headingRadians)` | A target at a waypoint-relative position. |
| `static RotationTarget atMeters(double meters, double headingRadians)` | A target at a distance along the path, in meters. |

## Components

| Component | What it is |
| --- | --- |
| `double waypoint` | The place as a waypoint-relative position, or `NaN`. `1.5` is halfway along the arc between waypoints 1 and 2 |
| `double meters` | The place as a distance along the path in meters, or `NaN` |
| `double headingRadians` | The heading to reach, in rad |

## Methods

| Method | What it does |
| --- | --- |
| `boolean byWaypoint()` | True when the place is waypoint-relative, false when it is in meters. |

## Good to know

- **Exactly one of `waypoint` and `meters` is set.** The other is `NaN`.
- **A waypoint-relative position keeps its meaning** when the waypoints are moved. A position outside the path is clamped to the nearest end, for both forms.
- **Between targets the heading blends smoothly.** Before the first target the robot holds the first heading, and after the last it holds the last.
- **A heading set on a waypoint** becomes a rotation target when the path is built.

:::caution
Two targets at the same place make the heading jump there. Do not put a rotation target on a waypoint that already has a heading.
:::

## More

- [Heading](/concepts/heading/): how the blend works.
- [PathSpec](/reference/core/pathspec/): where the targets go.
- [Path file](/files/path-file/): the `rotationTargets` key.
