---
title: Wheel forces
description: Send each wheel's push with the velocity, so the drive motors do not wait for an error.
sidebar:
  order: 10
---

By default the library sends the drivetrain a velocity and nothing else. Each wheel's velocity loop then notices
it is too slow and pushes harder. That works, but the push always arrives a little late.

The library already knows how hard the robot is about to accelerate, because it planned it. **Feed-forward**
means sending that along: for every wheel, the force it needs to make the planned acceleration happen. The
velocity loop then only has the leftover error to correct.

On the team's simulation bench, sending the forces took a quarter to a third off the wheel slip at the same
times, and removed the overshoot at the finish. It has not been measured on a real robot.

## The drivetrain's side

Add a second drive method that takes the forces. With CTRE swerve it is the same request with two more calls.

```java
private final SwerveRequest.ApplyFieldVelocity follow =
    new SwerveRequest.ApplyFieldVelocity().withDriveRequestType(DriveRequestType.Velocity);

public void driveFieldVelocity(ChassisVelocities velocity, double[] forcesX, double[] forcesY) {
  drivetrain.setControl(follow.withVelocity(velocity)
      .withWheelForceFeedforwardsX(forcesX)
      .withWheelForceFeedforwardsY(forcesY));
}
```

| Argument | Meaning |
| --- | --- |
| `velocity` | The field-relative velocity to drive at |
| `forcesX` | Each wheel's push along the field's x, newtons, in the drivetrain's module order |
| `forcesY` | The same along the field's y |

## Turn it on

Tell `Pathfinding` where the modules are and where to send the forces.

```java
otf.withWheelForces(
    drive.getModuleLocations(),
    (velocity, forcesX, forcesY) -> drive.driveFieldVelocity(velocity, forcesX, forcesY));
```

Call it right after building `otf`, before you build any commands. From then on the commands send the velocity
and the forces through this method, and the velocity-only output is no longer called.

`getModuleLocations()` is each module's position from the robot's center, x forward and y left, in the same
order the drivetrain expects the forces.

A `FollowPathCommand` built with the constructor takes the same with
`.withWheelForces(paths.robot(), modules, drive)`.

:::danger[Frame conversion]
On WPILib 2027 alpha-7 and alpha-8, the forces are field-relative in the library's frame and must be converted
like the velocity: negate every force. A force that points the wrong way pushes against the velocity it came
with. See [What your drivetrain provides](/start/drivetrain/#the-field-frame).
:::

## What the forces are

Mass times the planned acceleration, plus the moment of inertia times the planned spin acceleration, shared
between the wheels.

- **The acceleration is the plan's.** It is not the difference between this loop's command and the last one.
  That difference carries the noise in the measured pose.
- **The share follows the weight.** A robot that is speeding up leans back. Its rear wheels carry more weight and
  can push harder, so they get more of the force. Equal shares would ask the front wheels for more than they can
  grip.
- **Friction is left out.** The forces cover mass and inertia only. Friction is the job of the drive motors'
  static gain, kS.

## Check it on the robot

The forces rest on four assumptions. Check the ones you can.

| Assumption | How to check |
| --- | --- |
| The module positions are measured from the center of mass | Weigh the robot with a scale under each wheel in turn. Equal readings mean the center of mass is in the middle. |
| `massKg` and `momentOfInertiaKgM2` in `robot.json` are right | Weigh the robot with battery and bumpers. |
| The center of mass height is right | See [Traction](/concepts/traction/). |
| The drive gains use the forces at all | Log the drive motors' output with the forces on and off, on the same move. |

The last one is open. It is not confirmed that CTRE's module code applies the forces when the drive gains are
voltage gains. Give the robot a switch that turns the forces off, and compare logs before relying on them.

## Related

- [Feed-forward](/concepts/feed-forward/) explains the calculation.
- [WheelForceDrive](/reference/wpilib/wheelforcedrive/) and [WheelForces](/reference/follow/wheelforces/).
