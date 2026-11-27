---
title: OutputLimiter
description: Limits a wanted velocity to the largest step the robot can take without slipping.
sidebar:
  order: 3
---

`frc.team5712.path.follow.OutputLimiter` is the one limit every velocity command goes through. Given a wanted velocity, it returns the velocity closest to it that the robot can reach in one loop from the last one commanded. It caps spin acceleration, then the motors' push when speeding up, then holds translation and spin together to the traction limit. [The output limiter](/concepts/output-limiter/) explains each stage.

## Where you meet it

[`PathFollower`](/reference/follow/pathfollower/) and [`RailFollower`](/reference/follow/railfollower/) each own one and use it without any set-up. Robot code meets it in teleop, where the driver's sticks go through the same limits as the paths. The WPILib adapter is [`DriveLimiter`](/reference/wpilib/drivelimiter/), which takes and returns `ChassisVelocities`. The guide is [The teleop limiter](/guides/teleop-limiter/).

## Example

```java
OutputLimiter limiter = new OutputLimiter(paths.robot());   // robot init

limiter.reset(measuredVelocity);                            // when the drive command starts

FieldSpeeds command = limiter.limit(fromTheSticks, dt);     // every loop
FieldSpeeds acceleration = limiter.acceleration();          // for wheel forces, if used
```

`measuredVelocity`, `fromTheSticks` and `command` are `FieldSpeeds`: m/s along x and y, and rad/s of spin. Any one frame will do, the field's or the driver's, as long as every velocity given to the limiter is in that frame.

## Constructor

| Constructor | Description |
| --- | --- |
| `OutputLimiter(RobotConfig robot)` | A limiter for driving by hand, at the limits the paths are planned to. |

The limits come from the [`RobotConfig`](/reference/core/robotconfig/):

| Limit | Value |
| --- | --- |
| Traction budget | `robot.tractionMetersPerSecondSquared()`, m/s² |
| Motor push | `robot.driveAccelMetersPerSecondSquared(wheelSpeed)`, m/s² |
| Spin acceleration cap | `robot.maxAngularRadPerSecSquared()`, or 720°/s² if the robot has none |
| Spin lever | `robot.spinLeverMeters()`, meters |

The limiter has no settings of its own. To change a limit, change [the robot file](/files/robot-file/).

:::note
The limiter inside `PathFollower` is made with a package-private constructor and a larger budget, the traction limit divided by `tractionShare`, so that feedback has room on top of the plan. That constructor is not available to robot code. A limiter made with the public constructor uses the traction limit itself.
:::

## Methods

| Method | Returns | Description |
| --- | --- | --- |
| `reset(FieldSpeeds velocity)` | `void` | Starts the limiter from `velocity`. Pass the measured velocity, so a robot that is already moving is not asked to stop in one loop. Clears the stored acceleration. |
| `limit(FieldSpeeds wanted, double dtSeconds)` | `FieldSpeeds` | The velocity to command this loop: as close to `wanted` as the robot can get in `dtSeconds` from the last one commanded. |
| `acceleration()` | `FieldSpeeds` | The acceleration of the last step: m/s² along x and y and rad/s² of spin. Zero after a `reset`. [`WheelForces`](/reference/follow/wheelforces/) turns it into each wheel's push. |

Notes on `limit`:

- `dtSeconds` is the time since the last call, in seconds. It must be greater than 0. The limiter divides by it and does not check it.
- The limiter keeps its own last output. It starts from the measured velocity once, at `reset`, and after that from its own previous answer, not from the measured velocity.
- A new limiter that has not been reset starts from zero velocity.
- It limits acceleration only. It does not cap the wanted speed.
- Nothing is thrown for any input.

## What limit does, in order

1. Works out the acceleration that would reach `wanted` in one loop.
2. Clamps the spin acceleration to the spin acceleration cap.
3. If the step speeds the robot up, cuts the part of the acceleration along the direction of travel to what the motors can give at the fastest wheel's speed, `|v| + r·|ω|`.
4. If `|a| + c·|α|` is over the traction budget, scales the translation and spin accelerations down by the same factor.
5. Adds the result, times `dtSeconds`, to the last command and returns it.

## Related pages

- [Traction](/concepts/traction/), for where the budget comes from
- [Feed-forward](/concepts/feed-forward/), for what `acceleration()` is used for
