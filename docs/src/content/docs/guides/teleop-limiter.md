---
title: Teleop limiter
description: Limit the driver's sticks with the same traction numbers the paths use.
sidebar:
  order: 9
---

`DriveLimiter` takes the velocity the driver wants and returns the closest one the robot can reach this loop
without slipping or tipping. It reads the same `robot.json` the paths do.

## Limit the sticks

```java
DriveLimiter limiter = new DriveLimiter(paths.robot());      // robot init

// When the drive command starts. Begin from how the robot is moving now:
// after a path it may be rolling, and starting from zero would ask it to stop in one loop.
limiter.reset(drive.getFieldVelocity());

// Every loop. dt is the time since the last call, in seconds.
ChassisVelocities commanded = limiter.limit(wanted, dt);
drive.driveFieldVelocity(commanded);
```

## Measure dt

```java
double now = Timer.getTimestamp();
double dt = Math.clamp(now - lastTime, 0.001, 0.1);   // clamped, so one skipped loop does not allow a huge step
lastTime = now;
```

## Turn the sticks into field directions

The sticks are in the driver's directions: forward is away from their alliance wall.

```java
Translation2d fieldDirection =
    new Translation2d(speed * Math.cos(direction), speed * Math.sin(direction))
        .rotateBy(drive.getOperatorForwardDirection());   // 0° on blue, 180° on red

ChassisVelocities wanted =
    new ChassisVelocities(fieldDirection.getX(), fieldDirection.getY(), turn * maxAngularRate);
```

## Add the feed-forward

After `limit`, the limiter can give each wheel's push for the step it just took.

```java
ChassisVelocities commanded = limiter.limit(wanted, dt);

// forces[0] is each wheel's push along x, forces[1] along y.
// Newtons, in the frame the velocities are in.
double[][] forces = limiter.wheelForces(drive.getModuleLocations(), drive.getPose().getRotation());
drive.driveFieldVelocity(commanded, forces[0], forces[1]);
```

[Wheel forces](/guides/wheel-forces/) covers the drivetrain's side.

## Good to know

- **Use one frame** for `reset` and for `limit`. Field-relative is the simple choice, since `getFieldVelocity`
  already gives it.
- **It limits three things each loop, in order:** spin acceleration, the motors' push when speeding up, then
  translation and spin together to the traction limit.
- **The spin acceleration cap** is the robot file's, or 720°/s² if the file has none.
- **The traction limit** is the one the paths are planned to: `tractionShare`, 0.9 by default, of grip or
  tipping, whichever comes first.
- **There is no slip detection.** If the wheels still spin on a full stick, lower `frictionCoefficient` to the
  floor's real value, or lower `tractionShare`. Both change the paths too.

## More

- [The output limiter](/concepts/output-limiter/) and [Traction](/concepts/traction/): where the limit comes from.
- [DriveLimiter](/reference/wpilib/drivelimiter/) and [OutputLimiter](/reference/follow/outputlimiter/): every method.
- [Robot file](/files/robot-file/): the numbers it reads.
