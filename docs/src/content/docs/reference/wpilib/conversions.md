---
title: Conversions
description: Static helpers that convert between WPILib's types and the library's own, and read the alliance.
sidebar:
  order: 10
---

`Conversions` turns WPILib's geometry types into the library's own types and back. The core of the library has no WPILib in it, so it has its own small records: `Pose` for a pose and `FieldSpeeds` for a field-relative velocity. Both are described in [Small types](/reference/core/small-types/). This class is the bridge.

The commands in this package call it for you. You need it yourself only when you use a core class directly, for example when you run a [`PathFollower`](/reference/follow/pathfollower/) from your own loop, or when you pass a `Supplier<FieldSpeeds>` to `continuingFrom` on a [`FollowPathCommand`](/reference/wpilib/followpathcommand/).

## Example

```java
Pose robot = Conversions.toPose(drive.getPose());
FieldSpeeds moving = Conversions.toFieldSpeeds(drive.getFieldVelocity());

FieldFrame frame = Conversions.allianceFrame(FieldFrame.RED_HALF_TURN);
```

`getPose` and `getFieldVelocity` are methods of your own drivetrain class. See [The drivetrain](/start/drivetrain/).

## Methods

Every method is static. The class cannot be constructed.

| Method | Returns | Meaning |
| --- | --- | --- |
| `toPose(Pose2d pose)` | `Pose` | x and y in meters and the heading in radians, unchanged. |
| `toPose2d(Pose pose)` | `Pose2d` | The reverse. |
| `toFieldSpeeds(ChassisVelocities fieldRelative)` | `FieldSpeeds` | vx and vy in m/s and omega in rad/s, unchanged. |
| `toChassisVelocities(FieldSpeeds speeds)` | `ChassisVelocities` | The reverse. |
| `allianceFrame(FieldFrame redFrame)` | [`FieldFrame`](/reference/core/fieldframe/) | `FieldFrame.BLUE` on the blue alliance, `redFrame` on the red alliance. |

### The four type conversions

These copy numbers from one type to the other. They change no frame, flip nothing and scale nothing. A pose in the real field frame stays in the real field frame, and a blue-frame pose stays in the blue frame.

`toFieldSpeeds` and `toChassisVelocities` expect a field-relative velocity. `ChassisVelocities` can also hold a robot-relative velocity, and these methods cannot tell the difference, so a robot-relative one comes out wrong without any error.

### allianceFrame

Reads the alliance the driver station reports at the moment of the call. An unknown alliance counts as blue. Pass the flip your field needs on red: `RED_HALF_TURN` for a field with rotational symmetry, `RED_MIRROR` for a mirrored one. See [Alliances](/guides/alliances/) and [The field frame](/concepts/field-frame/).

:::caution
Call `allianceFrame` when a path starts, not when the command is built. In robot init the driver station often has not reported an alliance yet, and the answer would be blue. The library's commands read it at each start.
:::
