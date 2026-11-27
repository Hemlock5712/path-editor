---
title: RailDrive
description: Driver assists that hold the robot to a rail or nudge the intake onto a game piece.
sidebar:
  order: 6
---

`RailDrive` holds two driver assists. The first keeps the robot on a **rail**, a straight line on the field, while the driver drives along it. The second turns the driver's direction a little so the intake lines up on the game piece they are already heading for. In both, the driver's sticks go in as a velocity and the velocity to command comes out, already through the limiter that teleop and the paths use.

You meet it when you write a teleop command with a "hold this line" button or an intake-assist button. The guides are [Rails](/guides/rails/) and [Intake assist](/guides/intake-assist/). The rules behind them are in [The rail law](/concepts/rail-law/) and [Piece assist](/concepts/piece-assist/).

## Example

```java
// Robot constructor
RailDrive assist = new RailDrive(FollowerConfig.defaults(paths.robot()));
Rail trench = RailDrive.rail(new Translation2d(0.0, 2.9), Rotation2d.ZERO)
    .withHeading(0.0);

// when the assist command starts
assist.reset(drive.getFieldVelocity());

// every loop
Pose2d pose = drive.getPose();
ChassisVelocities command = assist.onRail(trench, pose, fromTheSticks, 4.0, 0.02);
double[][] forces = assist.wheelForces(drive.getModuleLocations(), pose.getRotation());
drive.driveFieldVelocity(command, forces[0], forces[1]);
```

`getPose`, `getFieldVelocity`, `getModuleLocations` and `driveFieldVelocity` are methods of your own drivetrain class. See [The drivetrain](/start/drivetrain/). `fromTheSticks` is the field-relative velocity your code worked out from the joysticks. The numbers are examples, not values for your robot.

## Frames

Use one frame for everything: the frame the pose is in. The rail, the game pieces, the driver's velocity and the result are all in it. A rail is just a line. It knows nothing about alliances, walls or the field's center, so nothing is flipped on red. A rail drawn for blue has to be given in the real field frame for red by your own code. See [Alliances](/guides/alliances/).

## Constructor

| Constructor | Meaning |
| --- | --- |
| `RailDrive(FollowerConfig config)` | Build once, in robot init. The config supplies the robot model, the reaction time, the ease-in and the heading limits. |

## Making a rail

| Method | Returns | Meaning |
| --- | --- | --- |
| `static rail(Translation2d point, Rotation2d direction)` | [`Rail`](/reference/follow/rail/) | An endless rail through `point`, in meters, running in `direction`. The heading is left to the driver. |

Shape the rail with the methods on [`Rail`](/reference/follow/rail/), which take plain numbers:

- `withHeading(radians)` also holds that heading.
- `between(startMeters, endMeters)` gives the rail ends, as distances along it from the point.
- `withPullIn(share)` sets how hard the rail pulls the robot on, as a share of the driver's speed along it. It throws `IllegalArgumentException` unless the share is more than 0.

## Methods

| Method | Returns | When to call |
| --- | --- | --- |
| `reset(ChassisVelocities measured)` | nothing | When the assist command starts. |
| `onRail(Rail rail, Pose2d pose, ChassisVelocities driver, double maxSpeed, double dt)` | `ChassisVelocities` | Every loop, to hold a rail. |
| `towardPiece(List<Translation2d> pieces, Pose2d robot, Transform2d robotToIntake, ChassisVelocities driver, double maxSpeed, double dt)` | `ChassisVelocities` | Every loop, to assist the intake. |
| `towardPiece(..., double eagerness)` | `ChassisVelocities` | The same, with your own eagerness as a seventh argument. |
| `static choosePiece(List<Translation2d> pieces, Translation2d intake, ChassisVelocities driver)` | `Optional<Translation2d>` | Any time, to ask which piece would be chosen. |
| `chosenPiece()` | `Optional<Translation2d>` | After `towardPiece`. |
| `wanted()` | `ChassisVelocities` | After a loop, for logging. |
| `wheelForces(Translation2d[] moduleLocations, Rotation2d heading)` | `double[][]` | After a loop, if the drivetrain takes wheel forces. |

### reset

Starts from the measured velocity, in m/s and rad/s, so a robot that is already moving is not asked to stop in one loop. It also forgets the piece the intake assist was chasing.

### onRail

One loop held to `rail`.

| Argument | Meaning |
| --- | --- |
| `rail` | The line to hold. It is given every loop, so it may change between loops. |
| `pose` | The robot's measured pose, in meters. |
| `driver` | The driver's velocity, field-relative, m/s and rad/s. |
| `maxSpeed` | The robot's top speed in m/s. Getting onto the rail has first claim on it. |
| `dt` | Seconds since the last loop, 0.02 on a standard loop. |

Across the rail, the robot is pulled back onto the line and stopped there. A stick pushed straight across the rail moves a robot that is on it at 0 m/s. Releasing the button is the way off. Along the rail, the part of the driver's velocity that points along the line is kept, and the robot stops at the rail's ends if it has any. If the rail holds a heading, the robot is turned to it. If not, the driver's turn rate is used.

Braking onto the rail plans on 0.55 of the robot file's traction limit. While the robot is still getting onto the rail, the driver's speed along it may change by 0.85 of the grip that is left.

### towardPiece

One loop of intake assist. The result is the driver's velocity with its direction turned toward a piece. The speed stays the driver's, and so does the turn rate.

| Argument | Meaning |
| --- | --- |
| `pieces` | The game pieces in view, as positions on the field in meters, in the pose's frame. [`PieceMap`](/reference/wpilib/piecemap/) produces this list. |
| `robot` | The robot's measured pose. |
| `robotToIntake` | Where the intake's mouth is on the robot. |
| `driver` | The driver's velocity, field-relative, m/s and rad/s. |
| `maxSpeed` | Top speed in m/s. The current version accepts it and does not use it. |
| `dt` | Seconds since the last loop. |
| `eagerness` | How hard the nudge turns the driver toward the piece. The default is `PieceAssist.EAGERNESS`, 1.5. At 1 the intake lines up only as it arrives. More lines it up sooner and turns the robot's travel further. |

A piece is chosen only if the driver is already heading for it: within 30° of the stick's direction as seen from the intake, within 3.0 m, and with the stick asking for at least 0.3 m/s. The piece most in line wins. Once a piece is chosen the assist stays with it, and another piece takes over only when it is 10° more in line with the stick.

The sideways slide is `eagerness × side ÷ ahead` of the driver's speed, where `side` and `ahead` are the piece's distances to the side of and ahead of the intake along the stick's direction. It is never more than 0.6 of the driver's speed. With no piece chosen, the result is the driver's velocity through the limiter, the same as plain teleop.

### choosePiece and chosenPiece

`choosePiece` answers which piece the driver is heading for, given where the intake is on the field. It has no memory of an earlier choice. `chosenPiece()` returns the piece the last `towardPiece` call was nudging toward, or empty. Use it to light an indicator or to log.

### wanted

The velocity the assist asked for on the last loop, before the limiter. For logging and tests.

### wheelForces

The force each wheel needs for the last loop, for a drivetrain that takes the [feed-forward](/concepts/feed-forward/). `moduleLocations` are the modules' positions on the robot in meters, in the drivetrain's order, and `heading` is the robot's heading. The result is two arrays: `[0]` each wheel's force along x, `[1]` along y, in newtons, one value per module. It throws `IllegalArgumentException` if `moduleLocations` is empty or every module is at the same place. See [`WheelForceDrive`](/reference/wpilib/wheelforcedrive/).

:::caution
How hard a rail pulls and how eager the intake assist is are feel numbers. Nothing in the library measures what a driver prefers, and neither assist has run on a real robot. Try them with a driver, at low speed first.
:::
