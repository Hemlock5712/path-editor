---
title: WheelForces
description: Turns an acceleration into the push each swerve wheel needs, in newtons.
sidebar:
  order: 5
---

`frc.team5712.path.follow.WheelForces` works out the push each wheel needs to give the robot an acceleration. That push is the feed-forward that goes to the drivetrain next to the velocity, so the wheels' own velocity loops only have the error left to correct. It is Newton's second law, shared between the wheels by the weight each one carries. [Feed-forward](/concepts/feed-forward/) explains the sums and the assumptions.

## Where you meet it

Robot code usually does not make one. The follow commands make their own once they are told where the modules are, with `withWheelForces` on [`Pathfinding`](/reference/wpilib/pathfinding/) or [`FollowPathCommand`](/reference/wpilib/followpathcommand/). [`DriveLimiter`](/reference/wpilib/drivelimiter/) and [`RailDrive`](/reference/wpilib/raildrive/) each have a `wheelForces(moduleLocations, heading)` method that does the same for teleop and the assists. The drivetrain takes the result through [`WheelForceDrive`](/reference/wpilib/wheelforcedrive/). The guide is [Wheel forces](/guides/wheel-forces/).

## Example

```java
// robot init: module positions from the center of mass, in the drivetrain's module order
double[] moduleX = {0.29, 0.29, -0.29, -0.29};   // forward, m
double[] moduleY = {0.29, -0.29, 0.29, -0.29};   // left, m
WheelForces forces = new WheelForces(paths.robot(), moduleX, moduleY);

// every loop, after follower.step(...)
double[][] f = forces.forAcceleration(follower.commandedAcceleration(), pose.headingRadians());
double[] forcesX = f[0];   // newtons along the field's x, one per module
double[] forcesY = f[1];   // newtons along the field's y, one per module
```

The module positions shown are the default robot's. Use the robot's own.

## Constructor

```java
WheelForces(RobotConfig robot, double[] moduleXMeters, double[] moduleYMeters)
```

| Parameter | Description |
| --- | --- |
| `robot` | The [`RobotConfig`](/reference/core/robotconfig/). Its mass in kg, moment of inertia in kg·m² and center of mass height in meters are used. |
| `moduleXMeters` | Each module's position forward of the center of mass, meters, in the drivetrain's module order. |
| `moduleYMeters` | Each module's position left of the center of mass, meters, in the same order. |

The arrays are copied.

Throws `IllegalArgumentException` when:

- either array is empty, or the two have different lengths: "one x and one y for every module",
- every module is at the same place: "the modules can't all be in one place".

## Methods

| Method | Returns | Description |
| --- | --- | --- |
| `moduleCount()` | `int` | The number of modules given to the constructor. |
| `forAcceleration(FieldSpeeds acceleration, double headingRadians)` | `double[][]` | The forces for `acceleration` with the robot facing `headingRadians`. |
| `none()` | `double[][]` | Zero push from every wheel, in the same shape. For a stopped robot. |

### forAcceleration

`acceleration` is a `FieldSpeeds` used one derivative up: m/s² along the field's x and y, and rad/s² of spin. It is what [`PathFollower.commandedAcceleration()`](/reference/follow/pathfollower/), [`RailFollower.commandedAcceleration()`](/reference/follow/railfollower/) and [`OutputLimiter.acceleration()`](/reference/follow/outputlimiter/) return.

`headingRadians` is the robot's heading in the same field frame.

The result is two new arrays:

| Index | Contents |
| --- | --- |
| `[0]` | The force along the field's x for each module, in newtons. |
| `[1]` | The force along the field's y for each module, in newtons. |

Each has one entry per module, in the order given to the constructor.

Two sums hold for every result:

```text
sum of the forces                    = mass · acceleration
their turning effort about the center = inertia · spin acceleration
```

## What it assumes

- The module positions are measured from the center of mass, not from the middle of the frame.
- The frame is stiff, so the weight on the wheels lies on a flat plane tilted by `mass · acceleration · centerOfMassHeightMeters`. A wheel's load cannot go below zero.
- The lean follows the acceleration at once.
- With `centerOfMassHeightMeters` at 0 the robot does not lean, and the shares do not change with the acceleration.
- With every module on one line, no plane fits and each wheel takes an equal share.

:::caution
Friction is not included, and neither is the effort to spin up the motors' own rotors. The forces are what the floor must give the robot and nothing more. The drive motors' velocity loops and static gain, kS, cover the rest.
:::

## Constants

`WheelForces` has no public constants or fields. Gravity is `RobotConfig.GRAVITY`, 9.81 m/s².
