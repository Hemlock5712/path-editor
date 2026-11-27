---
title: Teleop limiter
description: Limit the driver's sticks with the same traction numbers the paths use.
sidebar:
  order: 9
---

A driver who slams the stick asks the robot to change velocity faster than its tires can grip. The wheels spin,
the robot slides, and the odometry drifts. `DriveLimiter` takes the velocity the driver wants and returns the
closest one the robot can reach this loop without slipping or tipping.

It is the limiter every path goes through, reading the same `robot.json`. Teleop and autonomous then share one
set of numbers.

## Use it

Three calls: build it in robot init, reset it when the drive command starts, and limit every loop.

```java
DriveLimiter limiter = new DriveLimiter(paths.robot());      // robot init

limiter.reset(drive.getFieldVelocity());                     // when the drive command starts

ChassisVelocities commanded = limiter.limit(wanted, dt);     // every loop
drive.driveFieldVelocity(commanded);
```

| Call | Why |
| --- | --- |
| `reset(measured)` | Starts from how the robot is moving now. Coming back from a path, the robot may be rolling, and starting from zero would ask it to stop in one loop. |
| `limit(wanted, dt)` | Returns the velocity to command. `dt` is the time since the last call, in seconds. |

Measure `dt` from a clock. Clamp it, so one skipped loop does not allow a huge step.

```java
double now = Timer.getTimestamp();
double dt = Math.clamp(now - lastTime, 0.001, 0.1);
lastTime = now;
```

## One frame throughout

Use the same frame for `reset` and for `limit`. Field-relative is the simple choice, since `getFieldVelocity`
already gives it.

The sticks are in the driver's directions: forward is away from their alliance wall. Turn them into field
directions before limiting.

```java
Translation2d fieldDirection =
    new Translation2d(speed * Math.cos(direction), speed * Math.sin(direction))
        .rotateBy(drive.getOperatorForwardDirection());   // 0° on blue, 180° on red

ChassisVelocities wanted =
    new ChassisVelocities(fieldDirection.getX(), fieldDirection.getY(), turn * maxAngularRate);
```

## What it limits

In this order, each loop:

1. Spin acceleration, to the robot file's cap, or 720°/s² if the file has none.
2. The motors' push when speeding up, at the speed the fastest wheel is turning.
3. Translation and spin together, to the traction limit.

The traction limit here is the one the paths are planned to: `tractionShare`, 0.9 by default, of grip or
tipping, whichever comes first. Only the limiter inside the path follower allows the full limit, so that its
corrections have room on top of the plan. [The output limiter](/concepts/output-limiter/) explains the
difference.

## Add the feed-forward

After `limit`, the limiter can give each wheel's push for the step it just took.

```java
ChassisVelocities commanded = limiter.limit(wanted, dt);
double[][] forces = limiter.wheelForces(drive.getModuleLocations(), drive.getPose().getRotation());
drive.driveFieldVelocity(commanded, forces[0], forces[1]);
```

`forces[0]` is each wheel's push along x and `forces[1]` along y, in newtons, in the frame the velocities are in.
[Wheel forces](/guides/wheel-forces/) covers the drivetrain's side.

## If the robot slips anyway

The limiter has no slip detection. It trusts the numbers in `robot.json`. If the wheels still spin on a full
stick, the floor has less grip than the file says. Lower `frictionCoefficient` to the floor's real value, or
lower `tractionShare`. Both change the paths too, which is the idea.

## Related

- [Traction](/concepts/traction/) explains where the limit comes from.
- [DriveLimiter](/reference/wpilib/drivelimiter/) and [OutputLimiter](/reference/follow/outputlimiter/).
- [Robot file](/files/robot-file/) lists the numbers it reads.
