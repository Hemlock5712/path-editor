---
title: FieldFrame
description: The frame a path is followed in, and the conversions between red and blue.
sidebar:
  order: 18
---

`FieldFrame` is an enum that names the frame a path is followed in. Paths are authored for blue. On red nothing is rebuilt: each loop the follower converts the measured pose into the blue frame, follows the blue path, and converts the commanded speeds back. `FieldFrame` holds that conversion. [The field frame](/concepts/field-frame/) explains the idea.

Package: `frc.team5712.path.core`.

## Where you meet it

You pass a `FieldFrame` when you set up the library's commands, to say which kind of symmetry the field has. `Conversions.allianceFrame(FieldFrame.RED_HALF_TURN)` returns `BLUE` on the blue alliance and the value you gave it on red. See [Alliances](/guides/alliances/). You seldom call the conversion methods yourself, because the follower does it.

## Example

```java
FieldFrame frame = FieldFrame.RED_HALF_TURN;

Pose measured = new Pose(-6.0, -2.0, Math.PI);
Pose inBlue = frame.toBlue(measured);            // (6.0, 2.0, 0.0)

FieldSpeeds command = new FieldSpeeds(1.0, 0.5, 0.2);
FieldSpeeds onRed = frame.fromBlue(command);     // (-1.0, -0.5, 0.2)
```

## Values

The frame is WPILib 2027's: the origin at the center of the field and +X toward the blue alliance wall, so blue's half is `x > 0`.

| Value | Use | Position | Heading | Velocity | Spin |
| --- | --- | --- | --- | --- | --- |
| `BLUE` | The blue alliance | unchanged | unchanged | unchanged | unchanged |
| `RED_HALF_TURN` | Red on a rotationally symmetric field | `(-x, -y)` | `θ + π` | `(-vx, -vy)` | `ω` |
| `RED_MIRROR` | Red on a mirror-symmetric field, mirrored across `x = 0` | `(-x, y)` | `π - θ` | `(-vx, vy)` | `-ω` |

Both red conversions keep the origin where it is, and each is its own inverse. For that reason every `fromBlue` method does the same arithmetic as the matching `toBlue` method. Use whichever name says what you mean.

Neither conversion changes distance along the path, the size of the curvature, the size of the heading's rate of change, or the speed plan. The blue plan is the red plan.

## Methods

Positions are in meters, headings in radians, velocities in m/s and spin in rad/s.

| Method | Description |
| --- | --- |
| `Vec2 toBlue(Vec2 p)` | A point from this frame into blue. |
| `Vec2 fromBlue(Vec2 p)` | A point from blue into this frame. |
| `double headingToBlue(double radians)` | A heading from this frame into blue. For the two red frames the result is wrapped to the range above -π and up to π. `BLUE` returns the value unchanged. |
| `double headingFromBlue(double radians)` | A heading from blue into this frame. |
| `Pose toBlue(Pose pose)` | A pose from this frame into blue: the point and the heading together. |
| `Pose fromBlue(Pose pose)` | A pose from blue into this frame. |
| `FieldSpeeds toBlue(FieldSpeeds v)` | A field-relative velocity from this frame into blue. |
| `FieldSpeeds fromBlue(FieldSpeeds v)` | A field-relative velocity from blue into this frame. |
| `boolean mirrors()` | True for `RED_MIRROR` only. A mirror reverses handedness: left turns become right turns and curvature changes sign. |

For `BLUE`, the `Vec2`, `Pose` and `FieldSpeeds` methods return the object they were given.

[Pose, FieldSpeeds and Vec2](/reference/core/small-types/) are the library's own types. Like every Java enum, `FieldFrame` also has `values()` and `valueOf(String)`.

:::caution
`FieldFrame` converts between alliances. It does not convert between the library's frame and the corner-origin frame that WPILib 2027 alpha-7 and alpha-8 still use. That conversion is separate. See [The field frame](/concepts/field-frame/).
:::
