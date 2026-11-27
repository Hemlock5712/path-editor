---
title: Conversions
description: Static helpers that convert between WPILib's types and the library's own, and read the alliance.
sidebar:
  order: 10
---

`Conversions` turns WPILib's geometry types into the library's own `Pose` and `FieldSpeeds` ([Small types](/reference/core/small-types/)) and back.

The commands call it for you. You need it only when you use a core class directly, such as a [`PathFollower`](/reference/follow/pathfollower/) in your own loop, or pass a `Supplier<FieldSpeeds>` to `continuingFrom` on a [`FollowPathCommand`](/reference/wpilib/followpathcommand/).

## Example

```java
// drive.getPose() and drive.getFieldVelocity() are your own drivetrain's methods
Pose robot = Conversions.toPose(drive.getPose());
FieldSpeeds moving = Conversions.toFieldSpeeds(drive.getFieldVelocity());

// Call this when a path starts, not in robot init
FieldFrame frame = Conversions.allianceFrame(FieldFrame.RED_HALF_TURN);
```

## Methods

Every method is static. The class cannot be constructed.

| Method | Returns | What it does |
| --- | --- | --- |
| `toPose(Pose2d pose)` | `Pose` | Copies x and y in meters and the heading in radians, unchanged. |
| `toPose2d(Pose pose)` | `Pose2d` | The reverse. |
| `toFieldSpeeds(ChassisVelocities fieldRelative)` | `FieldSpeeds` | Copies vx and vy in m/s and omega in rad/s, unchanged. Expects a field-relative velocity. |
| `toChassisVelocities(FieldSpeeds speeds)` | `ChassisVelocities` | The reverse. Field-relative. |
| `allianceFrame(FieldFrame redFrame)` | [`FieldFrame`](/reference/core/fieldframe/) | `FieldFrame.BLUE` on the blue alliance, `redFrame` on red. Reads the driver station at the moment of the call. An unknown alliance counts as blue. |

## Good to know

- **The four type conversions only copy numbers.** They change no frame, flip nothing and scale nothing.
- **A robot-relative velocity comes out wrong, with no error.** `ChassisVelocities` can hold either kind, and the methods cannot tell.
- **Pass the flip your field needs on red:** `RED_HALF_TURN` for a field with rotational symmetry, `RED_MIRROR` for a mirrored one.

:::caution
Call `allianceFrame` when a path starts, not when the command is built. In robot init the driver station often has not reported an alliance yet, and the answer would be blue. The library's commands read it at each start.
:::

## More

- [Alliances](/guides/alliances/): choosing the flip.
- [The field frame](/concepts/field-frame/): the frames the library uses.
- [The drivetrain](/start/drivetrain/): `getPose` and `getFieldVelocity`.
