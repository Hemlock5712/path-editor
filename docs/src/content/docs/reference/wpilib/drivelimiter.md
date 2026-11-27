---
title: DriveLimiter
description: Limits a driver's requested velocity to what the robot can reach this loop without slipping.
sidebar:
  order: 5
---

`DriveLimiter` is for driving by hand. The driver's sticks ask for a velocity. The limiter returns the closest velocity the robot can reach in one loop without spinning its wheels or tipping. It is the same limiter every path goes through, the [`OutputLimiter`](/reference/follow/outputlimiter/), reading the same robot file, so teleop and autonomous share one set of limits.

You meet it when you write the teleop drive command. [The teleop limiter](/guides/teleop-limiter/) is the guide, and [The output limiter](/concepts/output-limiter/) explains the rule.

## Example

```java
// Robot constructor
DriveLimiter limiter = new DriveLimiter(paths.robot());

// when the drive command starts
limiter.reset(drive.getFieldVelocity());

// every loop
ChassisVelocities command = limiter.limit(fromTheSticks, 0.02);
drive.driveFieldVelocity(command);
```

`getFieldVelocity` and `driveFieldVelocity` are methods of your own drivetrain class. See [The drivetrain](/start/drivetrain/). `fromTheSticks` is the velocity your code worked out from the joysticks.

## Frames

The limiter does not care which frame the velocities are in. Field-relative works, and so does a driver-relative frame. What matters is that every velocity you give it, to `reset` and to `limit`, is in the same one. The result comes back in that frame.

## Constructor

| Constructor | Meaning |
| --- | --- |
| `DriveLimiter(RobotConfig robot)` | A limiter for this robot. Pass `paths.robot()`. Build it once, in robot init. |

The limits all come from the [robot file](/files/robot-file/):

- the traction limit, in m/s², which translation and spin share,
- the drive motors' push, which falls as the wheels speed up,
- the spin acceleration cap, or 720°/s² if the robot file has none.

## Methods

| Method | Returns | When to call |
| --- | --- | --- |
| `reset(ChassisVelocities measured)` | nothing | When the drive command starts. |
| `limit(ChassisVelocities wanted, double dtSeconds)` | `ChassisVelocities` | Every loop. |
| `wheelForces(Translation2d[] moduleLocations, Rotation2d heading)` | `double[][]` | Every loop, after `limit`, if the drivetrain takes wheel forces. |

### reset

Starts the limiter from the robot's measured velocity, in m/s and rad/s. The limiter remembers its own last answer and steps from there. Without a reset, a robot that is already moving when the command starts would be stepped from whatever the limiter last held, which may be zero.

### limit

Returns the velocity to command this loop: as close to `wanted` as the robot can get in `dtSeconds` from the last velocity it returned. `dtSeconds` is the loop period in seconds, 0.02 on a standard robot loop. It must be more than zero.

### wheelForces

Returns the force each wheel needs for the step `limit` just took, for a drivetrain that takes the [feed-forward](/concepts/feed-forward/).

| Argument | Meaning |
| --- | --- |
| `moduleLocations` | The swerve modules' positions on the robot, in meters, in the drivetrain's module order. |
| `heading` | The robot's heading in the frame the velocities are in. |

The result is two arrays. `[0]` holds each wheel's force along x and `[1]` each wheel's force along y, in newtons, in the frame the velocities are in, one value per module in the order given. It throws `IllegalArgumentException` if `moduleLocations` is empty or every module is at the same place. See [`WheelForceDrive`](/reference/wpilib/wheelforcedrive/) for the drivetrain's side.

:::note
The limiter holds the robot to the limits in the robot file. It does not detect slip. On a floor with less grip than the file says, lower the grip in the robot file. See [Traction](/concepts/traction/).
:::
