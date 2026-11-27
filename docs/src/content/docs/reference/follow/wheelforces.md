---
title: WheelForces
description: Turns an acceleration into the push each swerve wheel needs, in newtons.
sidebar:
  order: 5
---

`frc.team5712.path.follow.WheelForces` works out the push each wheel needs to give the robot an acceleration, which is the feed-forward that goes to the drivetrain next to the velocity.

Robot code usually does not make one: the follow commands make their own once you call `withWheelForces` on [`Pathfinding`](/reference/wpilib/pathfinding/) or [`FollowPathCommand`](/reference/wpilib/followpathcommand/). The guide is [Wheel forces](/guides/wheel-forces/).

## Example

```java
// Robot init: module positions from the center of mass, in the drivetrain's module order.
// These are the default robot's. Use the robot's own.
double[] moduleX = {0.29, 0.29, -0.29, -0.29};   // forward, m
double[] moduleY = {0.29, -0.29, 0.29, -0.29};   // left, m
WheelForces forces = new WheelForces(paths.robot(), moduleX, moduleY);

// Every loop, after follower.step(...)
double[][] f = forces.forAcceleration(follower.commandedAcceleration(), pose.headingRadians());
double[] forcesX = f[0];   // newtons along the field's x, one per module
double[] forcesY = f[1];   // newtons along the field's y, one per module
```

## Constructor

| Constructor | What it does |
| --- | --- |
| `WheelForces(RobotConfig robot, double[] moduleXMeters, double[] moduleYMeters)` | Copies the arrays. Throws `IllegalArgumentException` when either array is empty or the two have different lengths ("one x and one y for every module"), or when every module is at the same place ("the modules can't all be in one place"). |

| Parameter | Meaning |
| --- | --- |
| `robot` | The [`RobotConfig`](/reference/core/robotconfig/). Its mass in kg, moment of inertia in kg·m² and center of mass height in meters are used. |
| `moduleXMeters` | Each module's position forward of the center of mass, meters, in the drivetrain's module order. |
| `moduleYMeters` | Each module's position left of the center of mass, meters, in the same order. |

## Methods

| Method | Returns | What it does |
| --- | --- | --- |
| `moduleCount()` | `int` | The number of modules given to the constructor. |
| `forAcceleration(FieldSpeeds acceleration, double headingRadians)` | `double[][]` | The forces for `acceleration` with the robot facing `headingRadians`, the robot's heading in the same field frame. Two new arrays. |
| `none()` | `double[][]` | Zero push from every wheel, in the same shape. For a stopped robot. |

### forAcceleration

| Input or result | Meaning |
| --- | --- |
| `acceleration` | A `FieldSpeeds` used one derivative up: m/s² along the field's x and y, and rad/s² of spin. It is what [`PathFollower.commandedAcceleration()`](/reference/follow/pathfollower/), [`RailFollower.commandedAcceleration()`](/reference/follow/railfollower/) and [`OutputLimiter.acceleration()`](/reference/follow/outputlimiter/) return. |
| `[0]` | The force along the field's x for each module, in newtons, in the order given to the constructor. |
| `[1]` | The force along the field's y for each module, in newtons, in the same order. |

```text
sum of the forces                    = mass · acceleration
their turning effort about the center = inertia · spin acceleration
```

## What it assumes

| Assumption | Detail |
| --- | --- |
| Module positions | Measured from the center of mass, not from the middle of the frame. |
| A stiff frame | The weight on the wheels lies on a flat plane tilted by `mass · acceleration · centerOfMassHeightMeters`. A wheel's load cannot go below zero. |
| The lean | Follows the acceleration at once. |
| `centerOfMassHeightMeters` at 0 | The robot does not lean, and the shares do not change with the acceleration. |
| Every module on one line | No plane fits, and each wheel takes an equal share. |

## Constants

`WheelForces` has no public constants or fields. Gravity is `RobotConfig.GRAVITY`, 9.81 m/s².

## Good to know

- **Teleop and the assists have their own.** [`DriveLimiter`](/reference/wpilib/drivelimiter/) and [`RailDrive`](/reference/wpilib/raildrive/) each have a `wheelForces(moduleLocations, heading)` method that does the same.
- **The drivetrain takes the result** through [`WheelForceDrive`](/reference/wpilib/wheelforcedrive/).

:::caution
Friction is not included, and neither is the effort to spin up the motors' own rotors. The forces are what the floor must give the robot and nothing more. The drive motors' velocity loops and static gain, kS, cover the rest.
:::

## More

- [Feed-forward](/concepts/feed-forward/): the sums and the assumptions.
- [Wheel forces](/guides/wheel-forces/): turning it on.
