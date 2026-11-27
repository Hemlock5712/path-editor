---
title: The output limiter
description: The one limit every velocity command passes through, for paths and for teleop.
sidebar:
  order: 8
---

The laws inside the follower say what velocity they want. They do not ask whether the robot can get there in one loop. A jump from 0 to 3 m/s in 20 ms would spin the wheels, and a robot with spinning wheels is not following anything.

The **output limiter** sits between the wanted velocity and the drivetrain. Each loop it takes the last velocity it commanded and returns the largest step toward the wanted velocity that the robot can take without slipping. Every output of [`PathFollower`](/reference/follow/pathfollower/) goes through one, and so does every output of the rail and the intake assist. The class is [`OutputLimiter`](/reference/follow/outputlimiter/). In robot code the driver's sticks reach it through [`DriveLimiter`](/reference/wpilib/drivelimiter/).

## It limits acceleration, not speed

The limiter keeps three numbers: the last commanded `vx`, `vy` and spin rate. On each call it works out the acceleration needed to reach the wanted velocity in one loop:

```text
a_x = (wanted_vx − vx) / dt
a_y = (wanted_vy − vy) / dt
α   = (wanted_ω  − ω)  / dt
```

It then cuts that acceleration down in three stages, and adds what is left, times `dt`, to the command.

The limiter remembers its own last output. It is reset to the measured velocity once, when a command starts, and after that each step starts from its previous answer. A robot that is already moving when the command starts is therefore not asked to stop in one loop.

## The three stages, in order

### 1. Spin acceleration

The spin acceleration `α` is clamped to the spin acceleration cap. That is `maxAngularRadPerSecSquared` in the robot file, or 720°/s² when the file has none.

### 2. The motors' push

A motor's torque falls as it spins faster. Near top speed there is very little left to speed up with. The limiter asks the robot model how much acceleration the drive motors can add at the fastest wheel's speed, which is:

```text
wheel speed = |v| + r·|ω|
```

Here `r` is the distance from the robot's center to a module. The model is full battery voltage less the motor's back-EMF, capped by the stator current limit.

This stage only applies when the step speeds the robot up, and only to the part of the acceleration that points along the direction of travel. If that part is more than the motors can give, it is cut to what they can give. The part across the direction of travel is left alone. Turning the velocity at a steady speed takes the tread's grip, not motor power, so that part belongs to the next stage.

### 3. Translation and spin together

Grip is one budget. A wheel that is using all its grip to speed the robot up has none left to spin it. The limiter uses the same model as the speed plan:

```text
|a| + c·|α| ≤ budget          with c = I / (m·r)
```

`|a|` is the size of the translation acceleration, `I` is the robot's moment of inertia, `m` its mass and `r` the module radius. `c` is a length, the **spin lever**: spinning up at `α` asks each wheel to push like a straight-line acceleration of `c·α`. For the default robot `c` is about 0.24 m.

If the total is over the budget, the limiter multiplies `a_x`, `a_y` and `α` by the same factor so the total lands exactly on it. Scaling all three together keeps the direction of the step. The robot does less of what was asked, but not something different.

## Why the follower's budget is A / tractionShare

The speed plan holds acceleration and cornering to the traction limit, `A`. It is covered in [Traction](/concepts/traction/):

```text
A = tractionShare · min(μ·g, g·x/h)
```

`tractionShare` is 0.9 by default, so the plan keeps a 10 % margin below the point where grip or tipping gives out. For the default robot `A` is 12.8 m/s².

The limiter inside `PathFollower` does not use `A`. It uses `A / tractionShare`, the limit without the margin. For the default robot that is about 14.2 m/s².

The reason is feedback. The follower's command is the plan plus corrections: the push back onto the path, the heading correction. If the limiter stopped at `A`, then on a stretch where the plan already uses all of `A` there would be nothing left for corrections, and any error would stay. With the limiter at the plan's own limit, the fixture auto route strayed 9.3 cm from the path where the plan used all of its budget. With the limiter at `A / tractionShare` it strayed 2.4 cm.

So the margin has a job. The plan stays under it, and the corrections spend it.

## Teleop goes through the same limiter

The public constructor, `new OutputLimiter(robot)`, makes a limiter for driving by hand. It uses the same robot file, the same motor model and the same spin acceleration cap as the paths. Its budget is `A` itself, with the margin kept, because a driver's sticks are the whole command and there is no plan underneath that needs room for corrections.

The point of sharing it is one set of numbers. A robot whose teleop limiter and path planner disagree drives differently in auto and teleop, and a change to the robot file would have to be made twice. Here the robot file is the only place.

```java
OutputLimiter limiter = new OutputLimiter(paths.robot());   // robot init
limiter.reset(measuredVelocity);                            // when the drive command starts
FieldSpeeds command = limiter.limit(fromTheSticks, dt);     // every loop
```

Any one frame will do, the field's or the driver's, as long as every velocity given to the limiter is in that frame. The guide is [The teleop limiter](/guides/teleop-limiter/).

:::note
The limiter also reports the acceleration of the step it just took, through `acceleration()`. In teleop that is what gets turned into wheel forces. For a path, the follower uses the plan's acceleration instead. [Feed-forward](/concepts/feed-forward/) explains the difference.
:::

## Where the numbers come from

| Number | Source |
| --- | --- |
| Spin acceleration cap | `maxAngularRadPerSecSquared` in the robot file, or 720°/s² |
| Motor push | The robot file's motor, gear ratio, wheel radius, stator current limit and mass |
| Spin lever `c` | The robot file's moment of inertia, mass and module radius |
| Budget, in the follower | `tractionMetersPerSecondSquared() / tractionShare()` |
| Budget, in teleop, the rail and the intake assist | `tractionMetersPerSecondSquared()` |

The limiter has no settings of its own. To change what it allows, change the robot file. [The robot file](/files/robot-file/) lists the keys.

## What it does not do

- It does not cap speed. A wanted velocity above the robot's top speed is passed on as wanted. The motor stage only stops the command speeding up further once the motors have nothing left.
- It does not apply the motor stage to braking.
- It does not detect slip. It trusts the friction coefficient in the robot file. On a floor with less grip, the budget is too high and the wheels slide.
- It does not know the robot's heading or where each module is. It treats the robot as one budget. Sharing the push between individual wheels is the job of [`WheelForces`](/reference/follow/wheelforces/).
