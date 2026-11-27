---
title: WheelForceDrive
description: The one-method interface for a drivetrain that takes wheel forces along with a velocity.
sidebar:
  order: 11
---

`WheelForceDrive` is a drivetrain output that takes two things each loop: the velocity to drive at, and the force each wheel should push with, which is the [feed-forward](/concepts/feed-forward/).

You meet it when you call `withWheelForces` on [`Pathfinding`](/reference/wpilib/pathfinding/) or on a [`FollowPathCommand`](/reference/wpilib/followpathcommand/). [Wheel forces](/guides/wheel-forces/) is the guide.

## Example

```java
// One method, so a lambda that calls your drivetrain is enough
otf.withWheelForces(
    drive.getModuleLocations(),   // in your drivetrain's own module order
    (velocity, forcesX, forcesY) -> drive.driveFieldVelocity(velocity, forcesX, forcesY));
```

```java
// The drivetrain's side with CTRE's swerve, from the source's own example
private final SwerveRequest.ApplyFieldVelocity follow =
    new SwerveRequest.ApplyFieldVelocity().withDriveRequestType(DriveRequestType.Velocity);

public void driveFieldVelocity(ChassisVelocities velocity, double[] forcesX, double[] forcesY) {
  setControl(follow.withVelocity(velocity)
      .withWheelForceFeedforwardsX(forcesX)
      .withWheelForceFeedforwardsY(forcesY));
}
```

## Method

| Method | What it does |
| --- | --- |
| `void drive(ChassisVelocities fieldVelocity, double[] forcesXNewtons, double[] forcesYNewtons)` | Called once every loop while a command runs. At a finish or on a cancel it is called with zero velocity and zero forces. |

| Argument | Unit | Meaning |
| --- | --- | --- |
| `fieldVelocity` | m/s, rad/s | The velocity to drive at, field-relative, in the real field frame the robot's pose is in. |
| `forcesXNewtons` | N | Each wheel's force along the field's x axis, one value per module. |
| `forcesYNewtons` | N | Each wheel's force along the field's y axis, in the same order. |

## Good to know

- **The arrays are in the order of the module locations** you gave to `withWheelForces`. The forces are field-relative, like the velocity, not robot-relative.
- **Friction is not included.** It is left to the drive motors' static gain, kS, and their velocity loops.
- **The module locations are treated as measured from the center of mass.** On a robot whose weight sits well off-center, each wheel's share will be off by that much.

:::caution
The bench simulator measured less wheel slip with the feed-forward on. It has not run on a real robot. Check the sign and the module order on blocks before driving with it.
:::

## More

- [Feed-forward](/concepts/feed-forward/): what the forces are.
- [`WheelForces`](/reference/follow/wheelforces/): the class that does the sum.
- [The drivetrain](/start/drivetrain/): `getModuleLocations` and the three-argument `driveFieldVelocity`.
