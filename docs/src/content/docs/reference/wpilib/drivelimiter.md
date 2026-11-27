---
title: DriveLimiter
description: Limits a driver's requested velocity to what the robot can reach this loop without slipping.
sidebar:
  order: 5
---

`DriveLimiter` returns the velocity closest to the driver's request that the robot can reach in one loop without spinning its wheels or tipping. It is the WPILib side of the [`OutputLimiter`](/reference/follow/outputlimiter/) every path goes through.

You meet it when you write the teleop drive command. [The teleop limiter](/guides/teleop-limiter/) is the guide.

## Example

```java
// Robot constructor: build it once
DriveLimiter limiter = new DriveLimiter(paths.robot());

// When the drive command starts: begin from the measured velocity
limiter.reset(drive.getFieldVelocity());

// Every loop: fromTheSticks is the velocity your code worked out from the joysticks
ChassisVelocities command = limiter.limit(fromTheSticks, 0.02);
drive.driveFieldVelocity(command);
```

## Constructor

| Constructor | What it does |
| --- | --- |
| `DriveLimiter(RobotConfig robot)` | A limiter for this robot. Pass `paths.robot()`. Build it once, in robot init. |

## Methods

| Method | Returns | What it does |
| --- | --- | --- |
| `reset(ChassisVelocities measured)` | nothing | Starts the limiter from the measured velocity, m/s and rad/s. Call it when the drive command starts. |
| `limit(ChassisVelocities wanted, double dtSeconds)` | `ChassisVelocities` | The velocity to command this loop: as close to `wanted` as the robot can get in `dtSeconds` from the last velocity it returned. `dtSeconds` is the loop period in seconds, 0.02 on a standard loop, and must be more than zero. Call it every loop. |
| `wheelForces(Translation2d[] moduleLocations, Rotation2d heading)` | `double[][]` | The force each wheel needs for the step `limit` just took. `[0]` is each wheel's force along x and `[1]` along y, in newtons, one value per module in the order given. Throws `IllegalArgumentException` if `moduleLocations` is empty or every module is at the same place. Call it after `limit`, if the drivetrain takes wheel forces. |

### wheelForces arguments

| Argument | Meaning |
| --- | --- |
| `moduleLocations` | The swerve modules' positions on the robot, in meters, in the drivetrain's module order. |
| `heading` | The robot's heading in the frame the velocities are in. |

## Limits

All come from the [robot file](/files/robot-file/).

| Limit | Value |
| --- | --- |
| Traction | The traction limit, in m/s², which translation and spin share. |
| Motor push | The drive motors' push, which falls as the wheels speed up. |
| Spin acceleration | The spin acceleration cap, or 720°/s² if the robot file has none. |

## Good to know

- **Use one frame.** Field-relative or driver-relative both work, as long as every velocity given to `reset` and `limit` is in the same frame. The result and the wheel forces come back in it.
- **Reset at every start.** The limiter steps from its own last answer. Without a reset, a robot that is already moving is stepped from whatever the limiter last held, which may be zero.
- **It does not detect slip.** On a floor with less grip than the robot file says, lower the grip in the robot file. See [Traction](/concepts/traction/).

## More

- [The output limiter](/concepts/output-limiter/): the rule.
- [`WheelForceDrive`](/reference/wpilib/wheelforcedrive/) and [Feed-forward](/concepts/feed-forward/): the drivetrain's side of wheel forces.
- [The drivetrain](/start/drivetrain/): `getFieldVelocity` and `driveFieldVelocity`.
