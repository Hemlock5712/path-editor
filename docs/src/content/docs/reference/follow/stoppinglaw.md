---
title: StoppingLaw
description: Static methods that give the fastest speed that still stops on a goal.
sidebar:
  order: 4
---

`frc.team5712.path.follow.StoppingLaw` is the rule the library uses to arrive at a goal and stop on it: the smallest of a top speed, the speed that can still stop in the distance left after a reaction-time margin, and an ease-in line.

Robot code does not usually call it. [`PathFollower`](/reference/follow/pathfollower/) and [`RailLaw`](/reference/follow/raillaw/) use it, and you set its inputs through [`FollowerConfig`](/reference/follow/followerconfig/). See [Tuning](/guides/tuning/) and [Drive to a pose](/guides/drive-to-pose/).

## Example

```java
// 0.5 m from the goal, last commanded 3 m/s.
// The numbers are the library defaults for the default robot. They are not tuned values.
double speed = StoppingLaw.speed(
    0.5,      // distance left, m
    3.0,      // speed last commanded, m/s
    12.8,     // deceleration, m/s²
    0.05,     // reaction time, s
    4.0,      // top speed, m/s
    12.0,     // ease-in, m/s per meter
    0.003);   // deadband, m
// about 3.0 m/s: the braking curve governs
```

## Methods

All are static. The class cannot be constructed and it keeps no state. No method throws, and none checks its inputs.

| Method | Returns | What it does |
| --- | --- | --- |
| `speed(double distance, double speedNow, double decel, double reactionSeconds, double maxSpeed, double easeIn, double deadband)` | `double` | The speed toward a goal `distance` away, m/s, 0 or more. 0 when `distance` is less than `deadband`. |
| `brake(double distance, double speedNow, double decel, double reactionSeconds, double endSpeed)` | `double` | The speed, m/s, that still reaches `endSpeed` within `distance`, after the reaction margin. No top speed, no ease-in and no deadband. The follower uses it as a ceiling on the plan's speed along a path. |
| `slowing(double lawSpeed, double commandSpeed, double decel, double easeIn, double topSpeed)` | `double` | How hard the law is slowing the robot, m/s², 0 or more: the acceleration to feed forward while the law governs. 0 when `commandSpeed` is below `lawSpeed` or `lawSpeed` is at `topSpeed`. For heading, pass rad/s, rad/s² and the heading ease-in, and the result is in rad/s². |
| `spin(double error, double spinNow, double alpha, double reactionSeconds, double maxOmega, double easeIn, double deadband)` | `double` | The same law as `speed`, for turning. Works on the size of `error` and returns a spin rate in rad/s with the sign of `error`, so the robot turns toward the goal heading. |

## Formulas

| Method | Formula |
| --- | --- |
| `speed` | `min( maxSpeed ,  sqrt(2·decel·(distance − speedNow·reactionSeconds)) ,  easeIn·distance )` |
| `brake` | `sqrt( endSpeed² + 2·decel·(distance − speedNow·reactionSeconds) )` |
| `slowing` | `min( decel ,  easeIn·commandSpeed )` |

## Parameters

| Parameter | Used by | Unit | Meaning |
| --- | --- | --- | --- |
| `distance` | `speed`, `brake` | m | Distance left to the goal. |
| `speedNow` | `speed`, `brake` | m/s | The speed last commanded, for the reaction margin. |
| `decel` | `speed`, `brake`, `slowing` | m/s² | Braking deceleration. |
| `reactionSeconds` | `speed`, `brake`, `spin` | s | How late the drivetrain answers. |
| `maxSpeed` | `speed` | m/s | Top speed. |
| `easeIn` | `speed` | m/s per meter | Slope of the ease-in line. |
| `deadband` | `speed` | m | Closer than this, the speed is 0. |
| `endSpeed` | `brake` | m/s | The speed to arrive with. |
| `lawSpeed` | `slowing` | m/s | What the law asks for now. |
| `commandSpeed` | `slowing` | m/s | What was last commanded. |
| `easeIn` | `slowing` | 1/s | Slope of the ease-in line. |
| `topSpeed` | `slowing` | m/s | The law's top speed. |
| `error` | `spin` | rad | Goal heading minus measured heading, wrapped to the short way round. |
| `spinNow` | `spin` | rad/s | The spin rate last commanded. Its sign is ignored. |
| `alpha` | `spin` | rad/s² | Spin deceleration. |
| `maxOmega` | `spin` | rad/s | Top spin rate. |
| `easeIn` | `spin` | rad/s per radian | Slope of the ease-in line. |
| `deadband` | `spin` | rad | With less error than this, the rate is 0. |

## Constants

`StoppingLaw` has no constants. The deadbands the follower passes in are 0.003 m and 0.3°, fixed inside `PathFollower`.

## Good to know

- **In `speed`, a negative `distance − speedNow·reactionSeconds` is treated as 0.**
- **`brake` with an `endSpeed` of 0 is the braking part of `speed`.**
- **A negative `decel` gives NaN** from the square root.
- **It is the law from DriveToPose**, the drive-to-pose command the team built in its lessons.

## More

- [The stopping law](/concepts/stopping-law/): the formula explained.
- [`FollowerConfig`](/reference/follow/followerconfig/): `brakeMetersPerSecondSquared`, `latencySeconds`, `easeInPerSecond` and `headingEaseInPerSecond`.
