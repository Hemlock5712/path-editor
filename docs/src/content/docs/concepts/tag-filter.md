---
title: The tag filter
description: How camera sightings of one AprilTag become a steady goal that pose corrections cannot move.
sidebar:
  order: 15
---

A real field is built by hand, so a scoring element can sit a few centimeters from where the field map says.
[`TagFilter`](/reference/follow/tagfilter/) measures the goal from the tag itself: it filters the camera's
sightings of one tag into a steady pose to drive to.

Robot code uses it through [`TagGoal`](/reference/wpilib/taggoal/).

## Two poses for one robot

| Pose | Comes from | Behavior |
| --- | --- | --- |
| Wheel-odometry pose | The wheels and the gyro alone | Drifts slowly, never jumps |
| Field pose | Odometry corrected by vision | Right on average, moves a centimeter or two per correction |

The follower drives with the field pose. The filter stores the tag in the **odometry frame**, so a vision
correction moves the robot and the goal together.

## What it does

1. Robot code hands over each sighting: the tag as seen from the robot, and the odometry pose at capture.
2. It skips a sighting from farther than `MAX_DISTANCE_METERS`.
3. It places the tag in the odometry frame. `⊕` means "step from this pose by that much, in this pose's own
   directions".

   ```text
   tag in odometry = odometry pose at capture ⊕ tag from robot
   ```

4. It sets the sighting's noise from the distance `d` to the tag.

   ```text
   position noise = 0.005 m + 0.01 · d²
   yaw noise      = 0.5° + 1.5° · d
   ```

5. A sighting more than `JUMP_METERS` from the estimate, in position, is a **jump**. The first and second in a
   row are skipped, and `add` returns false. A third restarts the estimate from that sighting. Any other sighting
   sets the count back to zero.
6. It updates x, y and yaw, each with its own small **Kalman filter**. That is a weighted average that trusts
   each sighting by how good it is.

   ```text
   variance = variance + drift²
   gain     = variance / (variance + noise²)
   estimate = estimate + gain · (sighting − estimate)
   variance = variance · (1 − gain)
   ```

   The first sighting becomes the estimate outright.
7. It is **locked** once `SIGHTINGS_TO_LOCK` sightings agree. Until then `fieldGoal` returns empty and the
   follower keeps the goal it had.
8. To read the goal, it finds the goal relative to the robot in odometry, then places that on the field pose.

   ```text
   goal in odometry = tag in odometry ⊕ tagToGoal
   goal from robot  = goal in odometry, as seen from the odometry pose now
   goal on field    = field pose now ⊕ goal from robot
   ```

## Good to know

- **Lock does not expire.** Odometry carries the estimate while the tag is out of view. Only `reset()` and a
  restart after three jumps clear it.
- **`finishWhen(() -> tag.hasLock())`** stops the command finishing before the lock.
- **Bind it with the plain moving goal**, not the averaging form. See [Moving goals](/concepts/moving-goals/).

:::caution
`reset()` forgets the estimate. Call it when a new approach starts, or the follower gets a goal from the last
approach's tag.
:::

## The numbers

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

The first four are public. The noise and drift numbers are private, and model a typical camera.

The noise at each distance, 1 sigma:

| Distance | Position noise | Yaw noise |
| --- | --- | --- |
| 0.5 m | 0.75 cm | 1.25° |
| 1 m | 1.5 cm | 2° |
| 2 m | 4.5 cm | 3.5° |
| 3 m | 9.5 cm | 5° |

## What it does not do

- **It does not read a camera or decode tags.**
- **It does not correct for the camera's position or for latency.** Robot code gives the tag in the robot's
  frame and the odometry pose at capture.
- **It tracks one tag.** Use one filter per tag.
- **It does not correct the robot's pose** or check the sighting against the field map.

:::note
The library has not run on a real robot, so the noise model has not been compared with a real camera.
:::

## More

- [Tag alignment](/guides/tag-alignment/): the robot code.
- [`TagFilter`](/reference/follow/tagfilter/)
- [`TagGoal`](/reference/wpilib/taggoal/)
