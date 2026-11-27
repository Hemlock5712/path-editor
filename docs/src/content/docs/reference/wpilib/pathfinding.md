---
title: Pathfinding
description: The factory that builds on-the-fly path commands and library path commands for one drivetrain.
sidebar:
  order: 2
---

`Pathfinding` builds commands. You give it the drivetrain's pose, velocity and output once, in robot init, and then ask it for a command each time you need one: drive to a pose, drive to a named point, drive to the start of a drawn path and follow it, or follow a drawn path on its own.

You meet it as soon as the robot has to drive somewhere that was not drawn as a path. The guides call the one instance `otf`, short for on the fly. See [Driving to a pose](/guides/drive-to-pose/) and [Pathfinding](/guides/pathfinding/) for the tasks, and [On-the-fly paths](/concepts/on-the-fly/) for how a plan is made.

## Example

```java
import static frc.robot.PathEditorNames.*;

// Robot constructor, after PathLibrary.loadAll()
Pathfinding otf = new Pathfinding(
    paths,
    FollowerConfig.defaults(paths.robot()),
    () -> drive.getPose(),
    () -> drive.getFieldVelocity(),
    velocity -> drive.driveFieldVelocity(velocity),
    FieldFrame.RED_HALF_TURN);

Command toSpot = otf.to(drive, new Pose2d(6.0, 2.5, Rotation2d.PI));   // blue frame
Command toDepot = otf.toPoint(drive, Points.DEPOT, Rotation2d.PI);     // blue frame
Command toReef = otf.follow(drive, Paths.TO_REEF);                          // the drawn path
Command getThere = otf.thenFollow(drive, Paths.TO_REEF);                    // to its start, then the path
```

`getPose`, `getFieldVelocity` and `driveFieldVelocity` are methods of your own drivetrain class, not of this library. [The drivetrain](/start/drivetrain/) lists what the library needs from it. `drive` is also the `Mechanism` each command requires.

## Frames

Two frames appear on this page, and every method says which one it takes.

- The **blue frame** is the frame paths are drawn in: the origin at the field center, +X toward the blue alliance wall, meters and radians. A goal given in the blue frame is flipped for you when the robot is on the red alliance.
- The **real field frame** is the frame the robot's own pose is in. A goal given in this frame is used exactly as given, on either alliance. A pose from a camera is in this frame.

The alliance is read when a command starts, not when it is built. On red, blue-frame goals are flipped with the `redFrame` you passed to the constructor.

:::caution
WPILib 2027 alpha-7 still puts the field origin in a corner. The library uses the center origin. Until WPILib changes, the pose and velocity suppliers and the output have to convert between the two. [The field frame](/concepts/field-frame/) gives the conversion.
:::

## Constructor

```java
new Pathfinding(library, config, pose, fieldVelocity, fieldOutput, redFrame)
```

| Argument | Type | Meaning |
| --- | --- | --- |
| `library` | [`PathLibrary`](/reference/wpilib/pathlibrary/) | The loaded paths, pathfinders, named points and event bindings. |
| `config` | [`FollowerConfig`](/reference/follow/followerconfig/) | The follower settings every command uses. `FollowerConfig.defaults(paths.robot())` gives the library defaults. |
| `pose` | `Supplier<Pose2d>` | The robot's measured pose, real field frame, meters. Read when a command starts and every loop. |
| `fieldVelocity` | `Supplier<ChassisVelocities>` | The robot's measured velocity, field-relative, real field frame, m/s and rad/s. Read when a command starts and every loop. |
| `fieldOutput` | `Consumer<ChassisVelocities>` | Takes the velocity to drive at, field-relative, real field frame. Called every loop while a command runs, and once with zero when it stops. |
| `redFrame` | [`FieldFrame`](/reference/core/fieldframe/) | The flip used on the red alliance: `RED_HALF_TURN` for a field with rotational symmetry, `RED_MIRROR` for a mirrored one. |

The constructor does no work and throws nothing. Build one `Pathfinding` in robot init and keep it.

## Commands

Every method here builds a new command each time it is called. Build them in robot init where you can.

| Method | Returns | Goal | Goal's frame |
| --- | --- | --- | --- |
| `to(Mechanism drive, Pose2d blueGoal)` | [`PathfindCommand`](/reference/wpilib/pathfindcommand/) | A fixed pose. | Blue frame. Flipped on red. |
| `toPoint(Mechanism drive, PointId point, Rotation2d heading)` | `PathfindCommand` | A named point from the editor, arriving at `heading`. | Blue frame, point and heading both. Flipped on red. |
| `toPoint(Mechanism drive, String pointName, Rotation2d heading)` | `PathfindCommand` | The same, by the point's name. | Blue frame. Flipped on red. |
| `toTarget(Mechanism drive, Supplier<Pose2d> realGoal)` | `PathfindCommand` | A pose read from the supplier when the command starts and again at each re-plan. | Real field frame. Not flipped. |
| `thenFollow(Mechanism drive, PathId path)` | `PathfindCommand` | The start of a library path, then the path itself. | The path is blue frame. Flipped on red. |
| `thenFollow(Mechanism drive, String pathName)` | `PathfindCommand` | The same, by the path's name. | Blue frame. Flipped on red. |
| `follow(Mechanism drive, PathId path)` | [`FollowPathCommand`](/reference/wpilib/followpathcommand/) | A library path on its own. Nothing is planned. | The path is blue frame. Flipped on red. |
| `follow(Mechanism drive, String pathName)` | `FollowPathCommand` | The same, by the path's name. | Blue frame. Flipped on red. |

### to, toPoint and toTarget

Each plans a path around the field's obstacles when the command starts, from the measured pose and velocity, and follows it. `to` on a library with no field file plans on an open floor, so it is a plain drive-to-pose from anywhere.

`toPoint` looks the point up when the command is built. An unknown name throws `NoSuchElementException` at that moment, in robot init, not when the command runs.

`toTarget` is for a goal that is only known later, such as a pose worked out from vision. The supplier must return a pose every time it is asked. For a goal that keeps moving while the robot drives, add `withMovingGoal` to the command. See [Moving goals](/guides/moving-goals/).

On-the-fly paths are planned at 4 m/s and 6 m/s² unless `robot.json` sets `onTheFlyMaxMetersPerSecond` and `onTheFlyMaxMetersPerSecondSquared`.

### thenFollow

`thenFollow` plans to the start of the path, arriving along the path's start direction at the path's start speed, and then runs the path's own follow command. A path that starts at rest gets a stop at its start. A path that starts moving gets a rolling hand-over with no stop. The path's events fire through the library's `bindEvent` bindings.

The path's follower is built when you call `thenFollow`, so an unknown path name throws `NoSuchElementException` then. See [Chaining paths](/guides/chaining-paths/).

### follow

`follow` returns the same [`FollowPathCommand`](/reference/wpilib/followpathcommand/) you could build by hand, with this object's drivetrain, follower settings, clock and alliance already filled in and `withEvents(library)` already applied. It does no path work when it starts. An unknown path name throws `NoSuchElementException` when you call `follow`.

## Settings for later commands

These four methods change the factory, not a command. Each returns the same `Pathfinding`, so they chain. Call them in robot init, before you build any commands.

| Method | Applies to | Default | Read |
| --- | --- | --- | --- |
| `withWheelForces(Translation2d[] moduleLocations, WheelForceDrive drive)` | Every on-the-fly command, and `follow` commands built afterward. | Velocity only, through `fieldOutput`. | Every loop. |
| `withFootprint(Supplier<String> footprintName)` | `PathfindCommand`s built afterward. | `"bumpers"` | Once, as each command starts. |
| `withFrame(Supplier<FieldFrame> frameSource)` | Commands built afterward. | The driver station's alliance. | Once, as each command starts. |
| `withClock(DoubleSupplier seconds)` | Commands built afterward. | `Timer.getTimestamp()` | Every loop. |

### withWheelForces

Sends the [feed-forward](/concepts/feed-forward/) too. Each loop, `drive` gets the velocity and the force each wheel needs for the acceleration the follower asked for, in newtons. The constructor's `fieldOutput` is then no longer called by those commands.

```java
otf.withWheelForces(
    drive.getModuleLocations(),
    (velocity, forcesX, forcesY) -> drive.driveFieldVelocity(velocity, forcesX, forcesY));
```

`moduleLocations` are the swerve modules' positions on the robot in meters, in the drivetrain's own module order. The forces come back in that order. The array is copied. [`WheelForceDrive`](/reference/wpilib/wheelforcedrive/) describes the drivetrain's side, and [Wheel forces](/guides/wheel-forces/) is the guide. It throws `IllegalArgumentException` if the array is empty or every module is at the same place.

:::note
`withWheelForces` reaches on-the-fly commands that were already built, because they share this object's output. A `follow` command built before the call keeps sending velocity only. Calling it first avoids the difference.
:::

### withFootprint

Plans with a [footprint](/concepts/footprints/) from `robot.json`, such as the bumpers with the intake out.

```java
otf.withFootprint(() -> intake.isOut() ? "intakeOut" : "bumpers");
```

`intake.isOut()` is your own code. The supplier is read once as each command starts, and the plan and its re-plans all use that outline. A name `robot.json` does not have throws `NoSuchElementException` when the command starts. It has no effect on `follow` commands, which plan nothing.

### withFrame and withClock

`withFrame` replaces the alliance lookup. The supplier returns `FieldFrame.BLUE`, or the red flip to use. `withClock` replaces the clock, in seconds. Both exist mainly for tests and simulation.
