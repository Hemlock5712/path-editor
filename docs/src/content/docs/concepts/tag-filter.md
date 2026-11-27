---
title: The tag filter
description: How camera sightings of one AprilTag become a steady goal that pose corrections cannot move.
sidebar:
  order: 15
---

The field map says where every AprilTag is. A real field is built by hand, and a scoring element can sit a few centimeters from its drawing. A robot that aligns to the map's position aligns to the wrong place by that much.

The fix is to measure the goal from the tag itself. The camera sees the tag, the tag is bolted to the thing the robot wants to reach, and the goal is a fixed step away from the tag: 45 cm out from its face, say. [`TagFilter`](/reference/follow/tagfilter/) takes the camera's sightings of one tag, filters them into one steady estimate, and turns that into a pose to drive to. Robot code uses it through [`TagGoal`](/reference/wpilib/taggoal/). The guide is [Tag alignment](/guides/tag-alignment/).

The filter knows nothing about cameras. Robot code reads the camera, applies the camera's own corrections, and hands over each sighting.

## Two poses for one robot

A robot with vision has two ideas of where it is.

- **The wheel-odometry pose** comes from the wheels and the gyro alone. It is never corrected by vision. It drifts slowly over a match, but it is smooth: it never jumps.
- **The field pose** is odometry corrected by vision. It is right on average, but each correction moves it by a centimeter or two in one loop.

The follower drives with the field pose. If the tag's position were stored in the field frame, every vision correction would move the robot's pose and leave the goal where it was. The robot would see a sudden error and lurch to fix it, although nothing real had moved.

## The odometry frame trick

The filter stores the tag in the **odometry frame**.

**Storing a sighting.** A sighting is the tag as seen from the robot: so far forward, so far left, facing this way. The filter places it using the odometry pose at the moment the picture was taken:

```text
tag in odometry = odometry pose at capture ⊕ tag from robot
```

`⊕` means "step from this pose by that much, in this pose's own directions". Using the pose at capture, not the pose now, matters because a picture is tens of milliseconds old by the time it is processed. The robot's motion since then does not smear the estimate.

**Reading the goal.** When the follower needs a goal, the filter works out where the goal is relative to the robot right now, in odometry, and then places that on the robot's field pose:

```text
goal in odometry = tag in odometry ⊕ tagToGoal
goal from robot  = goal in odometry, as seen from the odometry pose now
goal on field    = field pose now ⊕ goal from robot
```

The last line is the trick. The goal is defined by its offset from the robot. When vision corrects the field pose by 2 cm, the goal moves by the same 2 cm in the same loop. The error the follower sees, goal minus robot, does not change. The correction cancels.

When the tag leaves the camera's view, nothing special happens. Odometry carries the estimate on, and odometry is accurate over the short distance of a final approach.

## The filter, one axis at a time

Each sighting is noisy. The filter keeps a running estimate of the tag's x, y and yaw, and for each one a number that says how unsure it is, the **variance**. This is a small **Kalman filter**, run separately on each of the three axes. In plain words it is a weighted average that trusts each new sighting by how good that sighting is.

For each axis, on each sighting:

```text
variance = variance + drift²
gain     = variance / (variance + noise²)
estimate = estimate + gain · (sighting − estimate)
variance = variance · (1 − gain)
```

`gain` is between 0 and 1. It is how far the estimate moves toward the new sighting.

- If the estimate is unsure and the sighting is sharp, the gain is near 1 and the estimate moves most of the way.
- If the estimate is already well known and the sighting is noisy, the gain is near 0 and the estimate barely moves.

After each sighting the variance shrinks, so the estimate gets steadier as sightings come in. The first sighting becomes the estimate outright, with that sighting's noise as its variance.

The `drift` line adds a little uncertainty back each time: 2 mm for position and 0.1° for yaw. Without it the variance would shrink toward zero and the filter would stop listening. Odometry does drift, so the tag does seem to move slightly, and the filter has to keep following.

## Noise that grows with distance

A camera measures a tag far better from close up. The filter models one sighting's noise as growing with the distance `d` from the robot to the tag:

```text
position noise = 0.005 m + 0.01 · d²
yaw noise      = 0.5° + 1.5° · d
```

| Distance | Position noise | Yaw noise |
| --- | --- | --- |
| 0.5 m | 0.75 cm | 1.25° |
| 1 m | 1.5 cm | 2° |
| 2 m | 4.5 cm | 3.5° |
| 3 m | 9.5 cm | 5° |

These are 1-sigma figures. A far sighting is noisy and moves the estimate a little. A near one is sharp and moves it more. As the robot closes in, the estimate tightens just when accuracy starts to matter.

Sightings from farther than 3.5 m are skipped entirely.

## The stray-sighting rule

Now and then a camera reports something badly wrong: it matched a different tag, or the pose solve flipped to its mirror answer. Averaging that in would drag the estimate off.

A sighting whose position is more than 25 cm from the estimate is a **jump**.

- The first and second jump in a row are skipped. `add` returns false and the estimate does not change.
- If a third in a row agrees, the old estimate was the wrong one. The filter starts again from that sighting.

Any sighting that is not a jump sets the count back to zero. The rule looks at position only, not yaw.

## Lock

The filter is **locked** once three sightings agree. Until then `fieldGoal` returns empty and the follower keeps the goal it had, which is normally the field map's spot.

Lock does not expire. It stays true while odometry carries the estimate with the tag out of view. Two things clear it: calling `reset()`, and a restart after three jumps, which sets the count back to one.

This is what `finishWhen(() -> tag.hasLock())` uses. The command sets off toward the map's position, switches to the measured goal when the lock comes, and cannot finish before it.

:::caution
`reset()` forgets the estimate. Call it when a new approach starts. A filter that still holds the last approach's tag will hand the follower a goal from somewhere else on the field.
:::

## Every constant

| Constant | Value | Meaning |
| --- | --- | --- |
| `MAX_DISTANCE_METERS` | 3.5 m | Sightings from farther than this are skipped. |
| `JUMP_METERS` | 0.25 m | A sighting this far from the estimate is a jump. |
| `JUMPS_TO_RESTART` | 3 | This many jumps in a row restart the estimate. |
| `SIGHTINGS_TO_LOCK` | 3 | Locked once this many sightings agree. |
| `XY_NOISE_FLOOR` | 0.005 m | Position noise at zero distance. |
| `XY_NOISE_PER_M2` | 0.01 m per m² | Position noise added per meter squared of distance. |
| `YAW_NOISE_FLOOR` | 0.5° | Yaw noise at zero distance. |
| `YAW_NOISE_PER_M` | 1.5° per meter | Yaw noise added per meter of distance. |
| `XY_DRIFT` | 0.002 m | Position uncertainty added before each sighting. |
| `YAW_DRIFT` | 0.1° | Yaw uncertainty added before each sighting. |

The first four are public. The noise and drift numbers are private to the class and cannot be changed from robot code. They are a model of a typical camera, not measurements of a particular one.

## Binding it to a path

The goal that comes out is already filtered, and it moves with the robot's pose corrections on purpose. Bind it with the plain form of the moving goal, not the averaging form. Averaging would make the goal lag the corrections it is meant to cancel. See [Moving goals](/concepts/moving-goals/).

## What it does not do

- It does not read a camera or decode tags.
- It does not correct for the camera's position on the robot or for latency. Robot code gives the tag in the robot's frame and the odometry pose at capture.
- It tracks one tag. Use one filter per tag.
- It does not correct the robot's pose. It only produces a goal.
- It does not check the sighting against the field map.
- The library has not run on a real robot, so the noise model has not been compared with a real camera.
