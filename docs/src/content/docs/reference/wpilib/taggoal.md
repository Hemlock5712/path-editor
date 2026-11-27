---
title: TagGoal
description: Filters a camera's sightings of one AprilTag into a steady pose to drive to.
sidebar:
  order: 7
---

`TagGoal` gives a goal measured from an AprilTag itself. Field elements are not always exactly where the field map says. A robot that drives to the map's position can be a few centimeters off. A robot that drives to a spot measured from the tag it can see is not.

`TagGoal` has no camera in it. Your robot code reads the camera and hands over each sighting of the tag. `TagGoal` filters the sightings into one steady estimate and returns the pose to drive to. You meet it when you align on a scoring position. [Tag alignment](/guides/tag-alignment/) is the guide and [The tag filter](/concepts/tag-filter/) explains the filter. It wraps [`TagFilter`](/reference/follow/tagfilter/) with WPILib's types.

## Example

```java
// Robot constructor: stop 0.45 m out from the tag's face, facing the tag
TagGoal reefTag = new TagGoal(new Transform2d(0.45, 0.0, Rotation2d.PI));

// every loop, for each new camera frame that sees the tag
reefTag.add(tagInRobot, drive.getOdometryPoseAt(captureTime), now);

// the command
Command align = otf.toTarget(drive, () -> mapGoal)
    .withMovingGoal(() -> reefTag.fieldGoal(drive.getOdometryPose(), drive.getPose()))
    .finishWhen(() -> reefTag.hasLock());
```

`getPose`, `getOdometryPose` and `getOdometryPoseAt` are methods of your own drivetrain class. See [The drivetrain](/start/drivetrain/). `tagInRobot`, `captureTime` and `now` come from your vision code, and `mapGoal` is the field map's position for the same spot in the real field frame. `otf` is the [`Pathfinding`](/reference/wpilib/pathfinding/) factory.

The command sets off for the map's position, switches to the tag's own measurement once the filter has a lock, and does not finish until then.

## Two poses of the robot

`TagGoal` uses two different robot poses, and they must not be swapped.

- The **wheel-odometry pose** comes from the wheels and the gyro alone. Vision never corrects it. It drifts slowly but never jumps.
- The **field pose** is the pose the follower uses, real field frame. Vision corrects it, so it can jump.

The estimate of the tag is kept in the odometry frame. Each sighting is placed using where odometry said the robot was when the picture was taken, so the robot's motion since then does not smear it. When the goal is asked for, it is measured from the robot's odometry pose now and placed on the robot's field pose now. A vision correction then moves the robot and the goal together, and the correction cancels.

## Constructor

| Constructor | Meaning |
| --- | --- |
| `TagGoal(Transform2d tagToGoal)` | Where to stop, in the tag's own frame, in meters: +X out of the tag's face, +Y to the tag's left. A rotation of 180° faces the tag. |

One `TagGoal` holds one tag's estimate. Hand it sightings of that tag only.

## Methods

| Method | Returns | When to call |
| --- | --- | --- |
| `reset()` | nothing | When a new approach starts. |
| `add(Pose2d tagInRobot, Pose2d odometryAtCapture, double timeSeconds)` | `boolean` | For every new camera frame that sees the tag. |
| `hasLock()` | `boolean` | Any time. |
| `secondsSinceSeen(double timeSeconds)` | `double` | Any time. |
| `tagPose()` | `Optional<Pose2d>` | Any time, for logging. |
| `fieldGoal(Pose2d odometryNow, Pose2d fieldNow)` | `Optional<Pose2d>` | Every loop while aligning. |

### reset

Forgets the estimate and the lock. Call it when an approach starts, for example from the button that starts the align command. Without it, the estimate from the last approach is still held.

### add

Folds one sighting into the estimate.

| Argument | Meaning |
| --- | --- |
| `tagInRobot` | The tag as the camera saw it, in the robot's frame, in meters: x forward, y left. The rotation is the way the tag's face points. The camera's own mounting correction must already be applied. |
| `odometryAtCapture` | The robot's wheel-odometry pose at the moment the picture was taken, not now. |
| `timeSeconds` | The time now, in seconds, on the same clock you later pass to `secondsSinceSeen`. |

It returns `false` when the sighting was skipped, which happens in two cases:

- The tag was more than 3.5 m from the robot.
- The sighting put the tag more than 0.25 m from the current estimate. That is usually a wrong tag or a flipped solve. If three sightings in a row say so, the old estimate was the wrong one: the third is accepted, and the estimate and the lock start over from it.

A near sighting is sharp and moves the estimate more. A far one is noisy and moves it less.

### hasLock

`true` once three sightings agree on where the tag is. It stays `true` when the tag leaves view, because odometry carries the estimate on.

### secondsSinceSeen

Seconds since a sighting last moved the estimate. It is infinite if there has never been one. Use it to stop trusting an estimate that odometry has carried for too long.

### tagPose

The tag's pose in the wheel-odometry frame, or empty before the first sighting. This is not a field pose. It is useful to log.

### fieldGoal

The pose to drive to, in the real field frame.

| Argument | Meaning |
| --- | --- |
| `odometryNow` | The robot's wheel-odometry pose now. |
| `fieldNow` | The robot's field pose now, the one the follower is given. |

It returns empty until `hasLock()` is `true`. A command bound to it with `withMovingGoal` keeps its first goal until then.

:::tip
The goal that `fieldGoal` returns is already filtered. Bind it with the one-argument `withMovingGoal(goal)`, not the averaging form. Averaging would make the goal lag the robot's own pose corrections.
:::
