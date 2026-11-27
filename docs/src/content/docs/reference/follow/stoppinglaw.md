---
title: StoppingLaw
description: Static methods that give the fastest speed that still stops on a goal.
sidebar:
  order: 4
---

`frc.team5712.path.follow.StoppingLaw` is the rule the library uses to arrive at a goal and stop on it. It returns the smallest of three speeds: a top speed, the speed that can still stop in the distance left after a reaction-time margin, and an ease-in line. It is the law from DriveToPose, the drive-to-pose command the team built in its lessons. [The stopping law](/concepts/stopping-law/) explains the formula.

## Where you meet it

Robot code does not usually call it. [`PathFollower`](/reference/follow/pathfollower/) uses it for the brake along a path and for the finish, and [`RailLaw`](/reference/follow/raillaw/) uses it across a rail and at a rail's ends. Its inputs are set through [`FollowerConfig`](/reference/follow/followerconfig/): `brakeMetersPerSecondSquared`, `latencySeconds`, `easeInPerSecond` and `headingEaseInPerSecond`. See [Tuning](/guides/tuning/) and [Drive to a pose](/guides/drive-to-pose/).

The class has only static methods. It cannot be constructed and it keeps no state.

## Example

```java
// 0.5 m from the goal, last commanded 3 m/s
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

The numbers are the library defaults for the default robot, to show the units. They are not tuned values.

## Methods

### speed

```java
static double speed(double distance, double speedNow, double decel, double reactionSeconds,
    double maxSpeed, double easeIn, double deadband)
```

The speed toward a goal `distance` away:

```text
min( maxSpeed ,  sqrt(2·decel·(distance − speedNow·reactionSeconds)) ,  easeIn·distance )
```

Returns 0 when `distance` is less than `deadband`. If `distance − speedNow·reactionSeconds` is negative it is treated as 0.

| Parameter | Unit | Description |
| --- | --- | --- |
| `distance` | m | Distance left to the goal. |
| `speedNow` | m/s | The speed last commanded, for the reaction margin. |
| `decel` | m/s² | Braking deceleration. |
| `reactionSeconds` | s | How late the drivetrain answers. |
| `maxSpeed` | m/s | Top speed. |
| `easeIn` | m/s per meter | Slope of the ease-in line. |
| `deadband` | m | Closer than this, the speed is 0. |

Returns a speed in m/s, 0 or more.

### brake

```java
static double brake(double distance, double speedNow, double decel, double reactionSeconds,
    double endSpeed)
```

The speed that still reaches `endSpeed` within `distance`, after the reaction margin:

```text
sqrt( endSpeed² + 2·decel·(distance − speedNow·reactionSeconds) )
```

With `endSpeed` 0 it is the braking part of `speed`. It has no top speed, no ease-in and no deadband. The follower uses it as a ceiling on the plan's speed along a path.

| Parameter | Unit | Description |
| --- | --- | --- |
| `distance` | m | Distance left. |
| `speedNow` | m/s | The speed last commanded. |
| `decel` | m/s² | Braking deceleration. |
| `reactionSeconds` | s | How late the drivetrain answers. |
| `endSpeed` | m/s | The speed to arrive with. |

Returns a speed in m/s.

### slowing

```java
static double slowing(double lawSpeed, double commandSpeed, double decel, double easeIn,
    double topSpeed)
```

How hard the law is slowing the robot, as a positive number or 0. This is the acceleration to feed forward while the law governs.

```text
min( decel ,  easeIn·commandSpeed )
```

Returns 0 when `commandSpeed` is below `lawSpeed`, because the command is still coming up to the law's speed, or when `lawSpeed` is at `topSpeed`, because there is nothing to slow.

| Parameter | Unit | Description |
| --- | --- | --- |
| `lawSpeed` | m/s | What the law asks for now. |
| `commandSpeed` | m/s | What was last commanded. |
| `decel` | m/s² | Braking deceleration. |
| `easeIn` | 1/s | Slope of the ease-in line. |
| `topSpeed` | m/s | The law's top speed. |

Returns a deceleration in m/s², 0 or more. For heading, pass rad/s, rad/s² and the heading ease-in, and the result is in rad/s².

### spin

```java
static double spin(double error, double spinNow, double alpha, double reactionSeconds,
    double maxOmega, double easeIn, double deadband)
```

The same law as `speed`, for turning. It works on the size of `error` and returns a spin rate with the sign of `error`, so the robot turns toward the goal heading.

| Parameter | Unit | Description |
| --- | --- | --- |
| `error` | rad | Goal heading minus measured heading, wrapped to the short way round. |
| `spinNow` | rad/s | The spin rate last commanded. Its sign is ignored. |
| `alpha` | rad/s² | Spin deceleration. |
| `reactionSeconds` | s | How late the drivetrain answers. |
| `maxOmega` | rad/s | Top spin rate. |
| `easeIn` | rad/s per radian | Slope of the ease-in line. |
| `deadband` | rad | With less error than this, the rate is 0. |

Returns a signed spin rate in rad/s.

## Errors

No method throws. None checks its inputs: a negative `decel` gives NaN from the square root.

## Constants

`StoppingLaw` has no constants. The deadbands the follower passes in are 0.003 m and 0.3°, fixed inside `PathFollower`.
