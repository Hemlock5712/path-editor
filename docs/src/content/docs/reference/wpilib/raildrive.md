---
title: RailDrive
description: Driver assists that hold the robot to a rail or nudge the intake onto a game piece.
sidebar:
  order: 6
---

`RailDrive` holds two driver assists: one keeps the robot on a **rail**, a straight line on the field, and one turns the driver's direction a little so the intake lines up on a game piece.

You meet it when you write a teleop command with a "hold this line" button or an intake-assist button. The guides are [Rails](/guides/rails/) and [Intake assist](/guides/intake-assist/).

## Example

```java
// Robot constructor. The numbers are examples, not values for your robot.
RailDrive assist = new RailDrive(FollowerConfig.defaults(paths.robot()));
Rail trench = RailDrive.rail(new Translation2d(0.0, 2.9), Rotation2d.ZERO)
    .withHeading(0.0);   // also hold a heading of 0 radians

// When the assist command starts: begin from the measured velocity
assist.reset(drive.getFieldVelocity());

// Every loop: fromTheSticks is the field-relative velocity from the joysticks,
// 4.0 is the top speed in m/s and 0.02 the loop time in seconds
Pose2d pose = drive.getPose();
ChassisVelocities command = assist.onRail(trench, pose, fromTheSticks, 4.0, 0.02);
double[][] forces = assist.wheelForces(drive.getModuleLocations(), pose.getRotation());
drive.driveFieldVelocity(command, forces[0], forces[1]);
```

## Constructor

| Constructor | What it does |
| --- | --- |
| `RailDrive(FollowerConfig config)` | Build once, in robot init. The config supplies the robot model, the reaction time, the ease-in and the heading limits. |

## Making a rail

| Method | Returns | What it does |
| --- | --- | --- |
| `static rail(Translation2d point, Rotation2d direction)` | [`Rail`](/reference/follow/rail/) | An endless rail through `point`, in meters, running in `direction`. The heading is left to the driver. |
| `withHeading(radians)` on `Rail` | `Rail` | Also holds that heading. |
| `between(startMeters, endMeters)` on `Rail` | `Rail` | Gives the rail ends, as distances along it from the point. |
| `withPullIn(share)` on `Rail` | `Rail` | Sets how hard the rail pulls the robot on, as a share of the driver's speed along it. Throws `IllegalArgumentException` unless the share is more than 0. |

## Methods

| Method | Returns | What it does |
| --- | --- | --- |
| `reset(ChassisVelocities measured)` | nothing | Starts from the measured velocity, m/s and rad/s, and forgets the piece the intake assist was chasing. Call it when the assist command starts. |
| `onRail(Rail rail, Pose2d pose, ChassisVelocities driver, double maxSpeed, double dt)` | `ChassisVelocities` | One loop held to `rail`: pulled onto the line and stopped there, keeping the part of the driver's velocity along it. |
| `towardPiece(List<Translation2d> pieces, Pose2d robot, Transform2d robotToIntake, ChassisVelocities driver, double maxSpeed, double dt)` | `ChassisVelocities` | One loop of intake assist: the driver's velocity with its direction turned toward a piece. The speed and the turn rate stay the driver's. |
| `towardPiece(..., double eagerness)` | `ChassisVelocities` | The same, with your own eagerness as a seventh argument. |
| `static choosePiece(List<Translation2d> pieces, Translation2d intake, ChassisVelocities driver)` | `Optional<Translation2d>` | Which piece the driver is heading for, given where the intake is on the field. It has no memory of an earlier choice. |
| `chosenPiece()` | `Optional<Translation2d>` | The piece the last `towardPiece` call was nudging toward, or empty. For an indicator or a log. |
| `wanted()` | `ChassisVelocities` | The velocity the assist asked for on the last loop, before the limiter. For logging and tests. |
| `wheelForces(Translation2d[] moduleLocations, Rotation2d heading)` | `double[][]` | The force each wheel needs for the last loop. `[0]` is each wheel's force along x, `[1]` along y, in newtons, one value per module. `moduleLocations` are in meters, in the drivetrain's order, and `heading` is the robot's heading. Throws `IllegalArgumentException` if `moduleLocations` is empty or every module is at the same place. |

### onRail arguments

| Argument | Meaning |
| --- | --- |
| `rail` | The line to hold. It is given every loop, so it may change between loops. |
| `pose` | The robot's measured pose, in meters. |
| `driver` | The driver's velocity, field-relative, m/s and rad/s. |
| `maxSpeed` | The robot's top speed in m/s. Getting onto the rail has first claim on it. |
| `dt` | Seconds since the last loop, 0.02 on a standard loop. |

### towardPiece arguments

| Argument | Meaning |
| --- | --- |
| `pieces` | The game pieces in view, as positions on the field in meters, in the pose's frame. [`PieceMap`](/reference/wpilib/piecemap/) produces this list. |
| `robot` | The robot's measured pose. |
| `robotToIntake` | Where the intake's mouth is on the robot. |
| `driver` | The driver's velocity, field-relative, m/s and rad/s. |
| `maxSpeed` | Top speed in m/s. The current version accepts it and does not use it. |
| `dt` | Seconds since the last loop. |
| `eagerness` | How hard the nudge turns the driver toward the piece. The default is `PieceAssist.EAGERNESS`, 1.5. At 1 the intake lines up only as it arrives. More lines it up sooner and turns the robot's travel further. |

## The numbers

| Rule | Value |
| --- | --- |
| Braking onto the rail plans on | 0.55 of the robot file's traction limit |
| Driver's speed along the rail may change by, while getting on | 0.85 of the grip that is left |
| A piece is chosen within | 30° of the stick's direction as seen from the intake, 3.0 m, with the stick asking for at least 0.3 m/s |
| Another piece takes over when it is | 10° more in line with the stick |
| The sideways slide is | `eagerness × side ÷ ahead` of the driver's speed, never more than 0.6 of it |

## Good to know

- **Use one frame, the frame the pose is in.** The rail, the pieces, the driver's velocity and the result are all in it. Nothing is flipped on red, so your own code gives a rail drawn for blue in the real field frame for red. See [Alliances](/guides/alliances/).
- **Releasing the button is the way off a rail.** A stick pushed straight across the rail moves a robot that is on it at 0 m/s.
- **Along the rail** the robot stops at the rail's ends if it has any. It is turned to the rail's heading if it holds one, and otherwise the driver's turn rate is used.
- **With no piece chosen**, `towardPiece` returns the driver's velocity through the limiter, the same as plain teleop.

:::caution
How hard a rail pulls and how eager the intake assist is are feel numbers. Nothing in the library measures what a driver prefers, and neither assist has run on a real robot. Try them with a driver, at low speed first.
:::

## More

- [The rail law](/concepts/rail-law/) and [Piece assist](/concepts/piece-assist/): the rules behind the assists.
- [`WheelForceDrive`](/reference/wpilib/wheelforcedrive/) and [Feed-forward](/concepts/feed-forward/): the drivetrain's side of wheel forces.
- [The drivetrain](/start/drivetrain/): `getPose`, `getFieldVelocity`, `getModuleLocations` and `driveFieldVelocity`.
