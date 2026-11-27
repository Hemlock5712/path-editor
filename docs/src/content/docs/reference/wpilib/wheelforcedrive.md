---
title: WheelForceDrive
description: The one-method interface for a drivetrain that takes wheel forces along with a velocity.
sidebar:
  order: 11
---

`WheelForceDrive` is the shape of a drivetrain output that takes two things each loop: the velocity to drive at, and the force each wheel should push with. The force is the [feed-forward](/concepts/feed-forward/). It tells the wheels about the acceleration the plan calls for, so the wheels' own velocity loops only have the error left to correct.

You meet it when you turn the feed-forward on, with `withWheelForces` on [`Pathfinding`](/reference/wpilib/pathfinding/) or on a [`FollowPathCommand`](/reference/wpilib/followpathcommand/). It is an interface with one method, so a lambda that calls your drivetrain is enough. [Wheel forces](/guides/wheel-forces/) is the guide.

## Example

```java
otf.withWheelForces(
    drive.getModuleLocations(),
    (velocity, forcesX, forcesY) -> drive.driveFieldVelocity(velocity, forcesX, forcesY));
```

`getModuleLocations` and the three-argument `driveFieldVelocity` are methods of your own drivetrain class, not of the library. See [The drivetrain](/start/drivetrain/). With CTRE's swerve, the source's own example of the drivetrain's side is one method:

```java
private final SwerveRequest.ApplyFieldVelocity follow =
    new SwerveRequest.ApplyFieldVelocity().withDriveRequestType(DriveRequestType.Velocity);

public void driveFieldVelocity(ChassisVelocities velocity, double[] forcesX, double[] forcesY) {
  setControl(follow.withVelocity(velocity)
      .withWheelForceFeedforwardsX(forcesX)
      .withWheelForceFeedforwardsY(forcesY));
}
```

## Method

```java
void drive(ChassisVelocities fieldVelocity, double[] forcesXNewtons, double[] forcesYNewtons)
```

| Argument | Unit | Meaning |
| --- | --- | --- |
| `fieldVelocity` | m/s, rad/s | The velocity to drive at, field-relative, in the real field frame the robot's pose is in. |
| `forcesXNewtons` | N | Each wheel's force along the field's x axis, one value per module. |
| `forcesYNewtons` | N | Each wheel's force along the field's y axis, in the same order. |

The arrays are in the order of the module locations you gave to `withWheelForces`, so give those in your drivetrain's own module order. The forces are field-relative, like the velocity, not robot-relative.

The commands call `drive` once every loop while they run. When a command stops the robot, at a finish or on a cancel, it calls `drive` with zero velocity and zero forces.

## What the forces are

The forces are the robot's mass times the planned acceleration, plus its moment of inertia times the planned spin acceleration, shared between the wheels. Each wheel takes the share of the push that it carries of the robot's weight, because a robot that is speeding up leans back and its front wheels can push less. [`WheelForces`](/reference/follow/wheelforces/) does the sum.

Friction is not included. It is left to the drive motors' static gain, kS, and their velocity loops.

:::note
The forces are worked out from the module locations as if they were measured from the robot's center of mass. On a robot whose weight sits well off-center, the shares will be off by that much.
:::

:::caution
The bench simulator measured less wheel slip with the feed-forward on. It has not run on a real robot. Check the sign and the module order on blocks before driving with it.
:::
