---
title: The field frame
description: Where the origin is, how a blue path runs on red, and how older files are read.
sidebar:
  order: 12
---

Every position in a path file is a pair of numbers. They only mean something once you know where zero is and which way the axes point. That choice is the **field frame**. This page covers the frame Myelin uses, how one path drawn for blue runs on red, and two places where frames do not match and something has to be converted.

The class that does the converting is [FieldFrame](/reference/core/fieldframe/).

## The frame

Myelin uses the WPILib 2027 field frame:

- The origin is at the center of the field.
- +X points toward the blue alliance wall. Blue's half of the field is `x > 0`.
- +Y is to the left when you face +X.
- Headings are measured counterclockwise from +X. In code they are radians. In files they are degrees.

Paths, field.json obstacles and named points are all stored in this frame.

## One path, both alliances

Paths are drawn for blue. A team does not draw a second set for red, and the library does not build one. Instead, [FieldFrame](/reference/core/fieldframe/) has three values:

| Value | Use it for | Meaning |
| --- | --- | --- |
| `BLUE` | The blue alliance | No change |
| `RED_HALF_TURN` | Red, on a field with rotational symmetry | The field turned 180 degrees about its center |
| `RED_MIRROR` | Red, on a field with mirror symmetry | The field mirrored across the line `x = 0` |

Which red value applies depends on the game. A field that looks the same after half a turn uses `RED_HALF_TURN`. A field whose two halves are mirror images uses `RED_MIRROR`.

### The transforms

| | `RED_HALF_TURN` | `RED_MIRROR` |
| --- | --- | --- |
| Position `(x, y)` | `(-x, -y)` | `(-x, y)` |
| Heading `θ` | `θ + π` | `π - θ` |
| Velocity `(vx, vy)` | `(-vx, -vy)` | `(-vx, vy)` |
| Spin `ω` | `ω` | `-ω` |

Headings are wrapped to the range from -π to π after the change. Each transform is its own inverse: apply it twice and you are back where you started. That is why `toBlue` and `fromBlue` do the same arithmetic.

### What happens each loop

On red, the follower still follows the blue path. Each loop it does three things:

1. It converts the robot's measured pose and velocity into the blue frame with `toBlue`.
2. It follows the blue path exactly as it would on blue.
3. It converts the commanded velocity back with `fromBlue` and sends that to the drivetrain.

Nothing is rebuilt. This works because neither transform changes anything the plan depends on. A half turn or a mirror keeps every length, so the distance along the path is the same. The size of the curvature is the same. The heading's rate of change has the same size. The [speed plan](/concepts/speed-plan/) only uses those sizes, so the blue plan is the red plan.

A mirror does reverse handedness: a left turn on blue is a right turn on red, and the sign of the curvature flips. `FieldFrame.mirrors()` is true for `RED_MIRROR` for code that needs to know.

The library's tests check this two ways. They build each test path a second time from flipped inputs and compare it with the transformed blue plan: positions agree to 1e-8 m. Then they drive a simulated robot on red both ways, once converting to blue each loop and once on a path built for red, and the two traces agree to 1e-6 m.

### In robot code

You rarely call `FieldFrame` methods yourself. You tell the library which red value your field uses, and the commands read the alliance when a path starts. `Conversions.allianceFrame` returns `BLUE` on blue, and the red value you pass in on red.

```java
FieldFrame frame = Conversions.allianceFrame(FieldFrame.RED_HALF_TURN);
```

Read the alliance when the path starts, not when the auto is built. An unknown alliance counts as blue, so a value read too early can be wrong. The [Alliances](/guides/alliances/) guide shows the full setup.

Not everything is flipped. Goals that come from the real world, such as a pose from vision, are already where they physically are. The on-the-fly commands take those as given and only flip goals that were authored for blue.

## Older files

Until 2026-11-26 the editor and the library used a frame with +X toward the red wall, so blue was at `x < 0`. WPILib 2027 points +X at the blue wall. The new frame is the old one turned half a turn about the center.

Files say which frame they are in through their schema number. An older file is turned when it is read:

```text
(x, y, heading)  becomes  (-x, -y, heading + 180 degrees)
```

| File | Already in the 2027 frame | Turned when read |
| --- | --- | --- |
| `*.path.json` | `"schema": 4` | Schema 1 to 3 |
| `field.json` | `"schema": 2` | No `schema` key |
| `.pathproject.json` named points | `"schema": 2` | Schema 1 |

For a path file, the turn covers every waypoint position, every `tangentDegrees` and `headingDegrees`, every rotation target's `degrees`, and every face-point zone's point. Distances along the path and waypoint-relative places do not change. You do not need to edit an old file by hand. The next time the editor saves it, it is written as schema 4. The details are on the [path file](/files/path-file/) page.

## WPILib alpha-7 and alpha-8

:::caution
WPILib 2027.0.0-alpha-7 and alpha-8 still report poses with the old origin at a corner of the field. On either release, robot code must convert everything that crosses between WPILib and the library.
:::

The library is built against alpha-7, and its frame is the one WPILib is moving to. Until a WPILib release includes that change, the two disagree. With `L` the field's length and `W` its width in meters:

| What | Direction | Conversion |
| --- | --- | --- |
| Poses: the measured pose, moving goals | Into the library | `(L/2 - x, W/2 - y, heading + 180 degrees)` |
| Velocities | Into the library and back out | `(-vx, -vy, ω)` |

The velocity rule applies both to the measured velocity you give the library and to the commanded velocity it gives back. Spin is unchanged, because turning the frame does not change which way counterclockwise is.

Forgetting this conversion puts the robot's pose on the wrong side of the origin, and the robot will not follow its path. If a first run goes wrong in that way, check this before anything else. See [Troubleshooting](/guides/troubleshooting/).

## What this piece does not do

- It does not read the alliance. `FieldFrame` is plain arithmetic. `Conversions.allianceFrame` in the WPILib layer asks the driver station.
- It does not decide which symmetry the field has. You pass `RED_HALF_TURN` or `RED_MIRROR`.
- It does not move obstacles. field.json describes the field as it physically is, for both alliances.
