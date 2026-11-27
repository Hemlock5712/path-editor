---
title: Wheel forces
description: Send each wheel's push with the velocity, so the drive motors do not wait for an error.
sidebar:
  order: 10
---

By default the library sends the drivetrain a velocity and nothing else. **Feed-forward** also sends, for every
wheel, the force it needs to make the planned acceleration happen.

## Add a drive method that takes forces

With CTRE swerve it is the same request with two more calls.

```java
private final SwerveRequest.ApplyFieldVelocity follow =
    new SwerveRequest.ApplyFieldVelocity().withDriveRequestType(DriveRequestType.Velocity);

// velocity: field-relative
// forcesX:  each wheel's push along the field's x, newtons, in the drivetrain's module order
// forcesY:  the same along the field's y
public void driveFieldVelocity(ChassisVelocities velocity, double[] forcesX, double[] forcesY) {
  drivetrain.setControl(follow.withVelocity(velocity)
      .withWheelForceFeedforwardsX(forcesX)
      .withWheelForceFeedforwardsY(forcesY));
}
```

## Turn it on

Call this right after building `otf`, before you build any commands.

```java
otf.withWheelForces(
    drive.getModuleLocations(),   // each module from the robot's center, x forward and y left, in force order
    (velocity, forcesX, forcesY) -> drive.driveFieldVelocity(velocity, forcesX, forcesY));
```

From then on the commands send the velocity and the forces through this method, and the velocity-only output is
no longer called.

:::danger[Frame conversion]
On WPILib 2027 alpha-7 and alpha-8, the forces must be converted like the velocity: negate every force. A force
that points the wrong way pushes against the velocity it came with. See
[What your drivetrain provides](/start/drivetrain/#the-field-frame).
:::

## Check it on the robot

The forces rest on four assumptions.

| Assumption | How to check |
| --- | --- |
| The module positions are measured from the center of mass | Weigh the robot with a scale under each wheel in turn. Equal readings mean the center of mass is in the middle. |
| `massKg` and `momentOfInertiaKgM2` in `robot.json` are right | Weigh the robot with battery and bumpers. |
| The center of mass height is right | See [Traction](/concepts/traction/). |
| The drive gains use the forces at all | Log the drive motors' output with the forces on and off, on the same move. |

:::caution
The last one is open. It is not confirmed that CTRE's module code applies the forces when the drive gains are
voltage gains. Give the robot a switch that turns the forces off, and compare logs before relying on them.
:::

## Good to know

- **Not measured on a real robot.** On the team's simulation bench, sending the forces took a quarter to a third
  off the wheel slip at the same times, and removed the overshoot at the finish.
- **A hand-built `FollowPathCommand`** takes the same with `.withWheelForces(paths.robot(), modules, drive)`.
- **The acceleration is the plan's**, not the difference between this loop's command and the last one.
- **The share follows the weight.** A robot that is speeding up leans back, so its rear wheels get more of the
  force.
- **Friction is left out.** That is the job of the drive motors' static gain, kS.

## More

- [Feed-forward](/concepts/feed-forward/): the calculation.
- [WheelForceDrive](/reference/wpilib/wheelforcedrive/) and [WheelForces](/reference/follow/wheelforces/): every method.
