---
title: Alliances
description: How a path drawn for blue runs on red, and which poses are flipped.
sidebar:
  order: 8
---

You draw every path once, for the blue alliance. On red the library flips it. This page says what is flipped,
what is not, and when the alliance is read.

## Choose the flip

The last argument to `Pathfinding`, and to the `FollowPathCommand` constructor, is the frame to use on red.

| Frame | Use it when | What it does to a pose |
| --- | --- | --- |
| `FieldFrame.RED_HALF_TURN` | The field looks the same after a half turn about its center | `(x, y, heading)` becomes `(-x, -y, heading + 180°)` |
| `FieldFrame.RED_MIRROR` | The field is mirrored across its center line | `x` becomes `-x` and the heading becomes `180° - heading` |

Set the same choice in the editor, under Settings, Project, so its red view matches what the robot does.

## How the flip works

The path is never rebuilt for red. Each loop, the follower converts the measured pose and velocity into the blue
frame, does all its work there, and converts the velocity command back. So a red run costs nothing at the start,
and it follows exactly the plan you saw in the editor.

## What is flipped

| You give the library | Frame | Flipped on red |
| --- | --- | --- |
| A drawn path | Blue | Yes |
| `otf.to(drive, pose)` | Blue | Yes |
| `otf.toPoint(drive, Points.X, heading)` | Blue | Yes |
| The start of a path in `otf.thenFollow` | Blue | Yes |
| `otf.toTarget(drive, supplier)` | Real field | No |
| A moving goal | Real field | No |
| The measured pose and velocity | Real field | No |
| Obstacles in `field.json` | Real field | No |

The rule behind the table: **something you authored is written for blue and gets flipped. Something measured on
the field is already where it physically is and does not.** A scoring pose from a camera is the second kind. So
is an obstacle.

## When the alliance is read

The alliance is read from the driver station **when a command starts**, not when it is built. That matters
because commands are built in robot init, before the driver station has reported an alliance. An unknown alliance
counts as blue.

## Override the lookup

For tests, or to force an alliance, replace the lookup.

```java
otf.withFrame(() -> FieldFrame.RED_HALF_TURN);
```

On `otf` it applies to every command built afterward. `withFrame` on a single command applies to that command.
`Conversions.allianceFrame(redFrame)` is the default lookup, if you want to wrap it.

## Driving by hand on red

The library's output is always a field velocity. It is never turned by the driver's perspective, so the drive
method you give the library must not apply an operator perspective. Your teleop command is different: it starts
from the sticks, in the driver's directions, and has to turn them into field directions itself before limiting
them. See [Teleop limiter](/guides/teleop-limiter/).

:::caution
On WPILib 2027 alpha-7 and alpha-8 the robot's pose is still measured from the blue corner, and the library's
frame is the center of the field. That conversion is separate from the alliance flip, and both apply.
[What your drivetrain provides](/start/drivetrain/#the-field-frame) has it.
:::

## Related

- [The field frame](/concepts/field-frame/) has the exact transforms.
- [FieldFrame](/reference/core/fieldframe/) and [Conversions](/reference/wpilib/conversions/).
