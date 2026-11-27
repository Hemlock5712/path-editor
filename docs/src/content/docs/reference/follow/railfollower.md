---
title: RailFollower
description: Runs the rail law, the output limiter and the feed-forward together, one loop at a time.
sidebar:
  order: 9
---

`frc.team5712.path.follow.RailFollower` drives the driver assists. Each loop, [`RailLaw`](/reference/follow/raillaw/) says what velocity to want, an [`OutputLimiter`](/reference/follow/outputlimiter/) decides how fast the robot may get there, and the feed-forward is worked out as it is for a path. The same object also runs plain teleop and the intake assist's nudge through that limiter, so a drive command can switch between them without a jump. [The rail law](/concepts/rail-law/) and [The piece assist](/concepts/piece-assist/) explain what it does.

## Where you meet it

Robot code uses [`RailDrive`](/reference/wpilib/raildrive/), which wraps a `RailFollower` and takes WPILib's `Pose2d` and `ChassisVelocities`. The guides are [Rails](/guides/rails/) and [Intake assist](/guides/intake-assist/). Use `RailFollower` directly only outside WPILib.

:::caution
The rail and the intake assist are driver-feel features. Their numbers have not been reviewed with a driver on a robot.
:::

## Example

```java
RailFollower rails = new RailFollower(FollowerConfig.defaults(paths.robot()));   // robot init

rails.reset(measuredVelocity);                                                   // when the command starts

FieldSpeeds command = rails.step(rail, pose, fromTheSticks, maxSpeed, dt);       // every loop
FieldSpeeds acceleration = rails.commandedAcceleration();                        // for wheel forces, if used
```

The rail is given every loop, so it can move. Use one frame throughout: the rail, the pose and the stick velocity are all in it.

## Constructor

| Constructor | Description |
| --- | --- |
| `RailFollower(FollowerConfig config)` | Makes the `RailLaw` from `config` and an `OutputLimiter` from `config.robot()`. |

The limiter is the public one, `new OutputLimiter(robot)`. Its budget is the robot's traction limit, the same as plain teleop through [`DriveLimiter`](/reference/wpilib/drivelimiter/).

## Methods

| Method | Returns | Description |
| --- | --- | --- |
| `law()` | `RailLaw` | The law this follower uses. |
| `reset(FieldSpeeds velocity)` | `void` | Starts from `velocity`. Pass the measured velocity, so a robot already moving is not asked to stop in one loop. |
| `step(Rail rail, Pose pose, FieldSpeeds driver, double maxSpeed, double dt)` | `FieldSpeeds` | One loop on `rail`: the velocity to command. |
| `step(Rail rail, Pose pose, FieldSpeeds driver, double maxSpeed, double dt, double maxCrossSpeed)` | `FieldSpeeds` | The same, never pulling across the rail faster than `maxCrossSpeed`. |
| `step(FieldSpeeds driver, double dt)` | `FieldSpeeds` | One loop with no rail: the driver's velocity through the limiter, as plain teleop. |
| `step(FieldSpeeds driver, FieldSpeeds nudge, double dt)` | `FieldSpeeds` | One loop of the driver's velocity turned by a nudge from [`PieceAssist.nudge`](/reference/follow/pieceassist/). |
| `wanted()` | `FieldSpeeds` | What was wanted on the last step, before the limiter. |
| `command()` | `FieldSpeeds` | The last command. |
| `commandedAcceleration()` | `FieldSpeeds` | The acceleration the last step calls for, for [`WheelForces`](/reference/follow/wheelforces/). |

### Parameters of step

| Parameter | Unit | Description |
| --- | --- | --- |
| `rail` | | The [`Rail`](/reference/follow/rail/) to hold this loop. |
| `pose` | m, rad | Where the robot is, or the point on it being held to the rail. |
| `driver` | m/s, rad/s | The driver's stick velocity, in field directions. |
| `maxSpeed` | m/s | Top speed. |
| `dt` | s | Seconds since the last loop. Must be greater than 0. |
| `maxCrossSpeed` | m/s | A cap on the pull across the rail. The five-argument form passes `Double.POSITIVE_INFINITY`. |
| `nudge` | m/s | A sideways velocity to turn the driver's by. Its spin part is not used. |

Every `step` returns the velocity to command, in m/s and rad/s, already through the limiter. None throws.

### The step with a nudge

`step(driver, nudge, dt)` adds the nudge to the driver's velocity and then scales the result back to the driver's own speed. The nudge turns the driver's velocity. It does not add to it. The spin rate is the driver's. A driver speed of zero gives a wanted velocity of zero.

### commandedAcceleration

Returns m/s² along x and y and rad/s² of spin, in field directions. What it contains depends on the last step:

| Last step | Translation | Spin |
| --- | --- | --- |
| On a rail | The law's braking across the rail, plus the limiter's step along it | The heading law's slowing, or the limiter's step when the driver has the heading |
| No rail | The limiter's step | The limiter's step |
| With a nudge | The limiter's step along the way the driver is pushing | The limiter's step |

The spin part is clamped to the spin acceleration cap, and the whole is scaled down if `|a| + c·|α|` is over the robot's traction limit. As for a path, it is what the plan calls for and not the difference between commands. See [Feed-forward](/concepts/feed-forward/).

## Constants

`RailFollower` has no constants. The rail's numbers are on [`RailLaw`](/reference/follow/raillaw/).
