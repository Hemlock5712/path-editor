---
title: Pathfinding
description: The factory that builds on-the-fly path commands and library path commands for one drivetrain.
sidebar:
  order: 2
---

`Pathfinding` builds commands: you give it the drivetrain's pose, velocity and output once, in robot init, then ask it for a command each time you need one.

You meet it as soon as the robot has to drive somewhere, and the guides call the one instance `otf`, short for on the fly. See [Driving to a pose](/guides/drive-to-pose/) and [Pathfinding](/guides/pathfinding/).

## Example

```java
import static frc.robot.PathEditorNames.*;

// Robot constructor, after PathLibrary.loadAll(). Build one and keep it.
Pathfinding otf = new Pathfinding(
    paths,                                            // the loaded library
    FollowerConfig.defaults(paths.robot()),           // follower settings for every command
    () -> drive.getPose(),                            // where the robot is
    () -> drive.getFieldVelocity(),                   // how fast, field-relative
    velocity -> drive.driveFieldVelocity(velocity),   // how to drive
    FieldFrame.RED_HALF_TURN);                        // the flip on red

Command toSpot = otf.to(drive, new Pose2d(6.0, 2.5, Rotation2d.PI));   // blue frame
Command toDepot = otf.toPoint(drive, Points.DEPOT, Rotation2d.PI);     // blue frame
Command toReef = otf.follow(drive, Paths.TO_REEF);                          // the drawn path
Command getThere = otf.thenFollow(drive, Paths.TO_REEF);                    // to its start, then the path
```

## Constructor

| Constructor | What it does |
| --- | --- |
| `Pathfinding(library, config, pose, fieldVelocity, fieldOutput, redFrame)` | Does no work and throws nothing. |

| Argument | Type | Meaning |
| --- | --- | --- |
| `library` | [`PathLibrary`](/reference/wpilib/pathlibrary/) | The loaded paths, pathfinders, named points and event bindings. |
| `config` | [`FollowerConfig`](/reference/follow/followerconfig/) | The follower settings every command uses. `FollowerConfig.defaults(paths.robot())` gives the library defaults. |
| `pose` | `Supplier<Pose2d>` | The robot's measured pose, real field frame, meters. Read when a command starts and every loop. |
| `fieldVelocity` | `Supplier<ChassisVelocities>` | The robot's measured velocity, field-relative, real field frame, m/s and rad/s. Read when a command starts and every loop. |
| `fieldOutput` | `Consumer<ChassisVelocities>` | Takes the velocity to drive at, field-relative, real field frame. Called every loop while a command runs, and once with zero when it stops. |
| `redFrame` | [`FieldFrame`](/reference/core/fieldframe/) | The flip used on the red alliance: `RED_HALF_TURN` for a field with rotational symmetry, `RED_MIRROR` for a mirrored one. |

:::caution
WPILib 2027 alpha-7 still puts the field origin in a corner, and the library uses the center. Until WPILib changes, the pose and velocity suppliers and the output have to convert between the two. [The field frame](/concepts/field-frame/) gives the conversion.
:::

## Commands

Each call builds a new command. Build them in robot init where you can.

| Method | Returns | What it does |
| --- | --- | --- |
| `to(Mechanism drive, Pose2d blueGoal)` | [`PathfindCommand`](/reference/wpilib/pathfindcommand/) | Plans to a fixed pose. Blue frame, flipped on red. With no field file it plans on an open floor. |
| `toPoint(Mechanism drive, PointId point, Rotation2d heading)` | `PathfindCommand` | Plans to a named point from the editor, arriving at `heading`. Blue frame, point and heading both, flipped on red. |
| `toPoint(Mechanism drive, String pointName, Rotation2d heading)` | `PathfindCommand` | The same, by the point's name. An unknown name throws `NoSuchElementException` when the command is built. |
| `toTarget(Mechanism drive, Supplier<Pose2d> realGoal)` | `PathfindCommand` | Plans to a pose read from the supplier when the command starts and again at each re-plan. Real field frame, not flipped. The supplier must return a pose every time. |
| `thenFollow(Mechanism drive, PathId path)` | `PathfindCommand` | Plans to the start of a library path, arriving along its start direction at its start speed, then runs the path with no stop if it starts moving. The path's events fire through the library's `bindEvent` bindings. Blue frame, flipped on red. |
| `thenFollow(Mechanism drive, String pathName)` | `PathfindCommand` | The same, by the path's name. An unknown name throws `NoSuchElementException` when you call it. |
| `follow(Mechanism drive, PathId path)` | [`FollowPathCommand`](/reference/wpilib/followpathcommand/) | Follows a library path on its own, with `withEvents(library)` already applied. Nothing is planned. Blue frame, flipped on red. |
| `follow(Mechanism drive, String pathName)` | `FollowPathCommand` | The same, by the path's name. An unknown name throws `NoSuchElementException` when you call it. |

## Settings for later commands

These change the factory, not a command. Each returns the same `Pathfinding`, so they chain. Call them in robot init, before you build any commands.

| Method | Default | What it does |
| --- | --- | --- |
| `withWheelForces(Translation2d[] moduleLocations, WheelForceDrive drive)` | Velocity only, through `fieldOutput`. | Each loop `drive` gets the velocity and each wheel's force in newtons, and `fieldOutput` is no longer called. `moduleLocations` are in meters, in the drivetrain's module order, and the array is copied. Throws `IllegalArgumentException` if the array is empty or every module is at the same place. |
| `withFootprint(Supplier<String> footprintName)` | `"bumpers"` | `PathfindCommand`s built afterward plan with this [footprint](/concepts/footprints/) from `robot.json`. Read once, as each command starts. A name `robot.json` does not have throws `NoSuchElementException` then. No effect on `follow` commands. |
| `withFrame(Supplier<FieldFrame> frameSource)` | The driver station's alliance. | Commands built afterward use this in place of the alliance lookup: `FieldFrame.BLUE`, or the red flip. Read once, as each command starts. Mainly for tests and simulation. |
| `withClock(DoubleSupplier seconds)` | `Timer.getTimestamp()` | Commands built afterward use this clock, in seconds. Read every loop. Mainly for tests and simulation. |

```java
// Send each wheel's push with the velocity
otf.withWheelForces(
    drive.getModuleLocations(),
    (velocity, forcesX, forcesY) -> drive.driveFieldVelocity(velocity, forcesX, forcesY));

// Plan with the intake's outline while it is out. intake.isOut() is your own code.
otf.withFootprint(() -> intake.isOut() ? "intakeOut" : "bumpers");
```

## Good to know

- **Two frames.** The blue frame is the one paths are drawn in: origin at the field center, +X toward the blue alliance wall, meters and radians. The real field frame is the one the robot's own pose is in, and a pose from a camera is in it.
- **The alliance is read when a command starts**, not when it is built.
- **On-the-fly paths are planned at 4 m/s and 6 m/s²** unless `robot.json` sets `onTheFlyMaxMetersPerSecond` and `onTheFlyMaxMetersPerSecondSquared`.
- **Call `withWheelForces` first.** It reaches on-the-fly commands that were already built, but a `follow` command built before the call keeps sending velocity only.

## More

- [On-the-fly paths](/concepts/on-the-fly/): how a plan is made.
- [Moving goals](/guides/moving-goals/), [Chaining paths](/guides/chaining-paths/) and [Wheel forces](/guides/wheel-forces/): the guides.
- [The drivetrain](/start/drivetrain/): what the library needs from `drive`.
