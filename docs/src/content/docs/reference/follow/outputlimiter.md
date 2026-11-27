---
title: OutputLimiter
description: Limits a wanted velocity to the largest step the robot can take without slipping.
sidebar:
  order: 3
---

`frc.team5712.path.follow.OutputLimiter` is the one limit every velocity command goes through: given a wanted velocity, it returns the closest velocity the robot can reach in one loop from the last one commanded.

[`PathFollower`](/reference/follow/pathfollower/) and [`RailFollower`](/reference/follow/railfollower/) each own one. Robot code meets it in teleop through [`DriveLimiter`](/reference/wpilib/drivelimiter/), the WPILib adapter, and the guide is [The teleop limiter](/guides/teleop-limiter/).

## Example

```java
// measuredVelocity, fromTheSticks and command are FieldSpeeds:
// m/s along x and y, and rad/s of spin, all in one frame
OutputLimiter limiter = new OutputLimiter(paths.robot());   // robot init

limiter.reset(measuredVelocity);                            // when the drive command starts

FieldSpeeds command = limiter.limit(fromTheSticks, dt);     // every loop
FieldSpeeds acceleration = limiter.acceleration();          // for wheel forces, if used
```

## Constructor

| Constructor | What it does |
| --- | --- |
| `OutputLimiter(RobotConfig robot)` | A limiter for driving by hand, at the limits the paths are planned to. |

## Methods

| Method | Returns | What it does |
| --- | --- | --- |
| `reset(FieldSpeeds velocity)` | `void` | Starts the limiter from `velocity`. Pass the measured velocity, so a robot that is already moving is not asked to stop in one loop. Clears the stored acceleration. |
| `limit(FieldSpeeds wanted, double dtSeconds)` | `FieldSpeeds` | The velocity to command this loop: as close to `wanted` as the robot can get in `dtSeconds` from the last one commanded. `dtSeconds` is the time since the last call, in seconds, and must be greater than 0. It is not checked. Nothing is thrown for any input. |
| `acceleration()` | `FieldSpeeds` | The acceleration of the last step: m/s² along x and y and rad/s² of spin. Zero after a `reset`. [`WheelForces`](/reference/follow/wheelforces/) turns it into each wheel's push. |

## Limits

All come from the [`RobotConfig`](/reference/core/robotconfig/). The limiter has no settings of its own: to change a limit, change [the robot file](/files/robot-file/).

| Limit | Value |
| --- | --- |
| Traction budget | `robot.tractionMetersPerSecondSquared()`, m/s² |
| Motor push | `robot.driveAccelMetersPerSecondSquared(wheelSpeed)`, m/s² |
| Spin acceleration cap | `robot.maxAngularRadPerSecSquared()`, or 720°/s² if the robot has none |
| Spin lever | `robot.spinLeverMeters()`, meters |

## Good to know

- **Use one frame.** The field's or the driver's will do, as long as every velocity given to the limiter is in it.
- **It steps from its own last output**, not from the measured velocity. It starts from the measured velocity once, at `reset`. A new limiter that has not been reset starts from zero velocity.
- **It limits acceleration only.** It does not cap the wanted speed.
- **The limiter inside `PathFollower` has a larger budget**, the traction limit divided by `tractionShare`, so feedback has room on top of the plan. Its constructor is package-private and not available to robot code.

## More

- [The output limiter](/concepts/output-limiter/): each stage of `limit`, in order.
- [Traction](/concepts/traction/): where the budget comes from.
- [Feed-forward](/concepts/feed-forward/): what `acceleration()` is used for.
