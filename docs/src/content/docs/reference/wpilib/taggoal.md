---
title: TagGoal
description: Filters a camera's sightings of one AprilTag into a steady pose to drive to.
sidebar:
  order: 7
---

`TagGoal` filters your camera's sightings of one AprilTag into a steady pose to drive to, measured from the tag itself and not from the field map.

You meet it when you align on a scoring position. [Tag alignment](/guides/tag-alignment/) is the guide.

## Example

```java
// Robot constructor: stop 0.45 m out from the tag's face, facing the tag
TagGoal reefTag = new TagGoal(new Transform2d(0.45, 0.0, Rotation2d.PI));

// Every loop, for each new camera frame that sees the tag.
// tagInRobot, captureTime and now come from your vision code.
reefTag.add(tagInRobot, drive.getOdometryPoseAt(captureTime), now);

// Set off for mapGoal, the field map's position in the real field frame.
// Switch to the tag's own measurement once locked, and do not finish until then.
Command align = otf.toTarget(drive, () -> mapGoal)
    .withMovingGoal(() -> reefTag.fieldGoal(drive.getOdometryPose(), drive.getPose()))
    .finishWhen(() -> reefTag.hasLock());
```

## Constructor

| Constructor | What it does |
| --- | --- |
| `TagGoal(Transform2d tagToGoal)` | Sets where to stop, in the tag's own frame, in meters: +X out of the tag's face, +Y to the tag's left. A rotation of 180° faces the tag. |

## Methods

| Method | Returns | What it does |
| --- | --- | --- |
| `reset()` | nothing | Forgets the estimate and the lock. Call it when a new approach starts, or the estimate from the last approach is still held. |
| `add(Pose2d tagInRobot, Pose2d odometryAtCapture, double timeSeconds)` | `boolean` | Folds one sighting into the estimate. Call it for every new camera frame that sees the tag. `false` when the sighting was skipped. |
| `hasLock()` | `boolean` | `true` once three sightings agree on where the tag is. Stays `true` when the tag leaves view, because odometry carries the estimate on. |
| `secondsSinceSeen(double timeSeconds)` | `double` | Seconds since a sighting last moved the estimate. Infinite if there has never been one. |
| `tagPose()` | `Optional<Pose2d>` | The tag's pose in the wheel-odometry frame, or empty before the first sighting. Not a field pose. For logging. |
| `fieldGoal(Pose2d odometryNow, Pose2d fieldNow)` | `Optional<Pose2d>` | The pose to drive to, in the real field frame. Empty until `hasLock()` is `true`. Call it every loop while aligning. |

### add arguments

| Argument | Meaning |
| --- | --- |
| `tagInRobot` | The tag as the camera saw it, in the robot's frame, in meters: x forward, y left. The rotation is the way the tag's face points. The camera's own mounting correction must already be applied. |
| `odometryAtCapture` | The robot's wheel-odometry pose at the moment the picture was taken, not now. |
| `timeSeconds` | The time now, in seconds, on the same clock you later pass to `secondsSinceSeen`. |

### fieldGoal arguments

| Argument | Meaning |
| --- | --- |
| `odometryNow` | The robot's wheel-odometry pose now. |
| `fieldNow` | The robot's field pose now, the one the follower is given. |

## When add skips a sighting

| Case | What happens |
| --- | --- |
| The tag was more than 3.5 m from the robot | Skipped. |
| The sighting put the tag more than 0.25 m from the current estimate | Skipped, as a wrong tag or a flipped solve. |
| Three such sightings in a row | The third is accepted, and the estimate and the lock start over from it. |

## Good to know

- **Two robot poses, never swapped.** The wheel-odometry pose comes from the wheels and gyro alone and never jumps. The field pose is the one the follower uses, real field frame, and vision corrects it.
- **One `TagGoal` holds one tag.** Hand it sightings of that tag only. It has no camera in it.
- **A command bound with `withMovingGoal` keeps its first goal until the lock.**

:::tip
The goal that `fieldGoal` returns is already filtered. Bind it with the one-argument `withMovingGoal(goal)`, not the averaging form. Averaging would make the goal lag the robot's own pose corrections.
:::

## More

- [The tag filter](/concepts/tag-filter/): the filter, and why the estimate is kept in the odometry frame.
- [`TagFilter`](/reference/follow/tagfilter/): the class this wraps with WPILib's types.
- [`Pathfinding`](/reference/wpilib/pathfinding/) and [The drivetrain](/start/drivetrain/): `otf` and the drivetrain methods in the example.
