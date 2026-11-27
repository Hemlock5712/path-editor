---
title: TagFilter
description: Filters camera sightings of one AprilTag into a steady pose to drive to.
sidebar:
  order: 6
---

`frc.team5712.path.follow.TagFilter` works out where one AprilTag really is from the camera's sightings of it, and turns that into a goal pose, for aligning on a field element that is not quite where the field map says.

Robot code uses [`TagGoal`](/reference/wpilib/taggoal/), which wraps a `TagFilter`, holds the tag-to-goal offset and takes WPILib's `Pose2d` and `Transform2d`. Use `TagFilter` directly only outside WPILib, for example in a simulator. The guide is [Tag alignment](/guides/tag-alignment/).

## Example

```java
// Every pose here is a Pose: meters and radians
TagFilter tag = new TagFilter();
Pose tagToGoal = new Pose(0.45, 0.0, Math.PI);   // 45 cm out of the tag's face, facing it (an example)

tag.reset();                                             // when an approach starts

tag.add(tagFromRobot, odometryPoseAtCapture, now);       // for every new camera frame that sees the tag

Optional<Pose> goal = tag.fieldGoal(tagToGoal, odometryPoseNow, fieldPoseNow);   // empty until locked
```

## Constructor

| Constructor | What it does |
| --- | --- |
| `TagFilter()` | A filter with no estimate. One filter tracks one tag. |

## Methods

No method throws.

| Method | Returns | What it does |
| --- | --- | --- |
| `reset()` | `void` | Forgets the estimate. Call when a new approach starts. |
| `add(Pose tagFromRobot, Pose odometryAtCapture, double timeSeconds)` | `boolean` | Folds one sighting into the estimate. False if the sighting was skipped, which leaves the estimate alone. |
| `hasLock()` | `boolean` | True once `SIGHTINGS_TO_LOCK` sightings agree on where the tag is. Stays true while the tag is out of view. |
| `tagPose()` | `Optional<Pose>` | The tag's pose in the wheel-odometry frame. Empty before the first sighting. |
| `secondsSinceSeen(double timeSeconds)` | `double` | Seconds since a sighting last moved the estimate. `Double.POSITIVE_INFINITY` if there is no estimate. |
| `fieldGoal(Pose tagToGoal, Pose odometryNow, Pose fieldNow)` | `Optional<Pose>` | The spot `tagToGoal` away from the tag, as a field pose to drive to: measured from the robot's wheel-odometry pose now, then placed on its field pose. Empty until `hasLock()`. |

### add parameters

| Parameter | Meaning |
| --- | --- |
| `tagFromRobot` | The tag as the camera saw it, in the robot's frame: x forward, y left, meters. The heading is the way the tag's face points. The camera's own correction must already be applied. |
| `odometryAtCapture` | The robot's wheel-odometry pose when the picture was taken, not the pose now. |
| `timeSeconds` | Now, on the same clock `secondsSinceSeen` is asked with. |

### fieldGoal parameters

| Parameter | Meaning |
| --- | --- |
| `tagToGoal` | Where to stop, in the tag's frame: x out of the tag's face, y to the tag's left, meters. A heading of 180° faces the tag. |
| `odometryNow` | The robot's wheel-odometry pose now. |
| `fieldNow` | The robot's field pose now, the one the follower is given. |

### What add returns

| Sighting | Returns | Effect |
| --- | --- | --- |
| The tag is farther than `MAX_DISTANCE_METERS` from the robot, or the distance is NaN | `false` | Skipped. |
| More than `JUMP_METERS` from the estimate, the first or second such sighting in a row | `false` | Skipped. |
| The third jump in a row | `true` | The estimate restarts from that sighting. The sighting count goes back to 1, so `hasLock()` is false again until two more agree. |

## Constants

All four are `public static final`. The noise model's numbers are private, and are listed in [The tag filter](/concepts/tag-filter/).

| Constant | Type | Value | Meaning |
| --- | --- | --- | --- |
| `MAX_DISTANCE_METERS` | `double` | 3.5 | Sightings from farther than this are skipped. |
| `JUMP_METERS` | `double` | 0.25 | A sighting this far from the estimate is taken as a wrong tag or a flipped solve. |
| `JUMPS_TO_RESTART` | `int` | 3 | Unless this many in a row agree. Then the estimate starts again. |
| `SIGHTINGS_TO_LOCK` | `int` | 3 | Locked once this many sightings agree. |

## Good to know

- **It knows nothing about cameras.** Robot code reads the camera and hands over each sighting.
- **The goal is already filtered.** Bind it with the plain moving goal, `bindGoal(goal)` on [`PathFollower`](/reference/follow/pathfollower/) or `withMovingGoal(goal)` on the commands, not the averaging form.
- **Pair it with `finishWhen(() -> tag.hasLock())`** so the path cannot finish on the field map's guess.
- **A vision correction cancels.** When vision corrects the field pose, the robot and the goal move together.

:::caution
`odometryAtCapture` and `odometryNow` must be the wheel-odometry pose: wheels and gyro only, never corrected by vision. Passing the vision-corrected field pose in their place removes the reason the filter exists.
:::

## More

- [The tag filter](/concepts/tag-filter/): the filter, and why the estimate is kept in the odometry frame.
- [Moving goals](/concepts/moving-goals/): binding a goal to a path.
- [Small types](/reference/core/small-types/): `Pose`.
