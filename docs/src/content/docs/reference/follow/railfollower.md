---
title: RailFollower
description: Runs the rail law, the output limiter and the feed-forward together, one loop at a time.
sidebar:
  order: 9
---

`frc.team5712.path.follow.RailFollower` drives the driver assists: each loop a [`RailLaw`](/reference/follow/raillaw/) says what velocity to want, and an [`OutputLimiter`](/reference/follow/outputlimiter/) decides how fast the robot may get there.

Robot code uses [`RailDrive`](/reference/wpilib/raildrive/), which wraps a `RailFollower` with WPILib's `Pose2d` and `ChassisVelocities`. Use `RailFollower` directly only outside WPILib. The guides are [Rails](/guides/rails/) and [Intake assist](/guides/intake-assist/).

## Example

```java
// Use one frame throughout: the rail, the pose and the stick velocity are all in it
RailFollower rails = new RailFollower(FollowerConfig.defaults(paths.robot()));   // robot init

rails.reset(measuredVelocity);                                                   // when the command starts

// The rail is given every loop, so it can move
FieldSpeeds command = rails.step(rail, pose, fromTheSticks, maxSpeed, dt);       // every loop
FieldSpeeds acceleration = rails.commandedAcceleration();                        // for wheel forces, if used
```

## Constructor

| Constructor | What it does |
| --- | --- |
| `RailFollower(FollowerConfig config)` | Makes the `RailLaw` from `config` and an `OutputLimiter` from `config.robot()`. The limiter is the public one, `new OutputLimiter(robot)`, so its budget is the robot's traction limit, the same as plain teleop through [`DriveLimiter`](/reference/wpilib/drivelimiter/). |

## Methods

Every `step` returns the velocity to command, in m/s and rad/s, already through the limiter. None throws.

| Method | Returns | What it does |
| --- | --- | --- |
| `law()` | `RailLaw` | The law this follower uses. |
| `reset(FieldSpeeds velocity)` | `void` | Starts from `velocity`. Pass the measured velocity, so a robot already moving is not asked to stop in one loop. |
| `step(Rail rail, Pose pose, FieldSpeeds driver, double maxSpeed, double dt)` | `FieldSpeeds` | One loop on `rail`: the velocity to command. |
| `step(Rail rail, Pose pose, FieldSpeeds driver, double maxSpeed, double dt, double maxCrossSpeed)` | `FieldSpeeds` | The same, never pulling across the rail faster than `maxCrossSpeed`. |
| `step(FieldSpeeds driver, double dt)` | `FieldSpeeds` | One loop with no rail: the driver's velocity through the limiter, as plain teleop. |
| `step(FieldSpeeds driver, FieldSpeeds nudge, double dt)` | `FieldSpeeds` | One loop of the driver's velocity turned by a nudge from [`PieceAssist.nudge`](/reference/follow/pieceassist/). The nudge is added, then the result is scaled back to the driver's own speed. The spin rate is the driver's. A driver speed of zero gives a wanted velocity of zero. |
| `wanted()` | `FieldSpeeds` | What was wanted on the last step, before the limiter. |
| `command()` | `FieldSpeeds` | The last command. |
| `commandedAcceleration()` | `FieldSpeeds` | The acceleration the last step calls for: m/s² along x and y and rad/s² of spin, in field directions. For [`WheelForces`](/reference/follow/wheelforces/). |

### Parameters of step

| Parameter | Unit | Meaning |
| --- | --- | --- |
| `rail` | | The [`Rail`](/reference/follow/rail/) to hold this loop. |
| `pose` | m, rad | Where the robot is, or the point on it being held to the rail. |
| `driver` | m/s, rad/s | The driver's stick velocity, in field directions. |
| `maxSpeed` | m/s | Top speed. |
| `dt` | s | Seconds since the last loop. Must be greater than 0. |
| `maxCrossSpeed` | m/s | A cap on the pull across the rail. The five-argument form passes `Double.POSITIVE_INFINITY`. |
| `nudge` | m/s | A sideways velocity to turn the driver's by. Its spin part is not used. |

### What commandedAcceleration contains

| Last step | Translation | Spin |
| --- | --- | --- |
| On a rail | The law's braking across the rail, plus the limiter's step along it | The heading law's slowing, or the limiter's step when the driver has the heading |
| No rail | The limiter's step | The limiter's step |
| With a nudge | The limiter's step along the way the driver is pushing | The limiter's step |

## Constants

`RailFollower` has no constants. The rail's numbers are on [`RailLaw`](/reference/follow/raillaw/).

## Good to know

- **One object runs the rail, plain teleop and the nudge through the same limiter**, so a drive command can switch between them without a jump.
- **The acceleration is what the plan calls for**, not the difference between commands. Its spin part is clamped to the spin acceleration cap, and the whole is scaled down if `|a| + c·|α|` is over the robot's traction limit.

:::caution
The rail and the intake assist are driver-feel features. Their numbers have not been reviewed with a driver on a robot.
:::

## More

- [The rail law](/concepts/rail-law/) and [The piece assist](/concepts/piece-assist/): what it does.
- [Feed-forward](/concepts/feed-forward/): what the acceleration is used for.
