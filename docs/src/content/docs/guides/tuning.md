---
title: Tuning
description: The numbers to measure on your robot, the eight follower settings, and the order to change them.
sidebar:
  order: 14
---

Most of what makes a path run well is not a gain. It is the robot file being true. The library plans from the
numbers in `robot.json`, and a plan made for a different robot cannot be followed well however the follower is
set. So tuning starts with measuring.

:::caution
The library has not run on a real robot. Every default below came from simulation and the team's bench. Treat
them as starting points, and run the first paths slowly, with room around the robot.
:::

## First, the robot file

Measure these before changing anything else. [Robot file](/files/robot-file/) lists every key.

| Key | How to get it |
| --- | --- |
| `massKg` | Weigh the robot with battery and bumpers. |
| `frictionCoefficient` | Brake the wheels. Pull the robot slowly across field carpet with a luggage scale held level near the floor. Grip is the peak pull divided by the weight. |
| `centerOfMassHeightMeters` | Lift the front until the robot balances on its back wheels. The height is the wheels' distance from the center divided by the tangent of the tilt angle. |
| `bumperLengthMeters`, `bumperWidthMeters` | A tape measure, outside edge to outside edge. |
| `moduleRadiusMeters`, `wheelRadiusMeters`, `driveGearRatio` | The drivetrain's constants. |

Grip and the center of mass height decide the traction limit, which decides how hard every path accelerates,
brakes and corners. [Traction](/concepts/traction/) has the rule.

## The follower's settings

`FollowerConfig` holds the eight settings a team may change.

| Setting | Default | Unit | What it is |
| --- | --- | --- | --- |
| `latencySeconds` | 0.05 | s | How late the drivetrain answers a command. The plan is read this far ahead. |
| `crossTrackKp` | 5 | m/s per m | How hard the robot is pushed back toward the path |
| `headingKp` | 5 | rad/s per rad | How hard the heading is corrected |
| `brakeMetersPerSecondSquared` | NaN | m/s² | The stopping law's braking. NaN means the robot's traction limit. |
| `easeInPerSecond` | 12 | m/s per m | The slope of the final ease-in to the goal |
| `headingEaseInPerSecond` | 5 | rad/s per rad | The same for the heading |
| `positionToleranceMeters` | 0.02 | m | How close counts as finished |
| `headingToleranceRadians` | 1° | rad | How well turned counts as finished |

Change them with the builder, and give the result to `Pathfinding`.

```java
FollowerConfig config = FollowerConfig.defaults(paths.robot()).toBuilder()
    .latencySeconds(0.06)
    .easeInPerSecond(8)
    .build();
```

The spin limits are not here. They are `maxAngularRadPerSec` and `maxAngularRadPerSecSquared` in the robot file,
or 540°/s and 720°/s² when the file leaves them out.

## An order to work in

Change one thing, run the same path, and compare the plots in [Telemetry](/guides/telemetry/).

1. **Run a straight path at low limits.** Set the path's maximum speed and acceleration low in the editor. Check
   that it goes the right way, on both alliances.
2. **Set `latencySeconds`.** On a straight path, compare `Speed` with `PlannedSpeed`. If the robot's speed trails
   the plan by a steady time, that time is the latency. Measure it on the robot.
3. **Raise the path's limits to the robot's.** Watch `CrossTrack` in the turns. If it grows in every fast turn,
   the tires are not holding what the file says. Lower `frictionCoefficient`.
4. **Look at the finish.** See the table below.
5. **Leave the P gains for last.** The bench sweeps settled on 5 for both and never needed a derivative term.

## The finish

| What you see | Try |
| --- | --- |
| The robot rolls past the goal and creeps back | Lower `brakeMetersPerSecondSquared`, or lower the path's acceleration |
| It twitches at the goal after arriving | Lower `easeInPerSecond`. 12 twitched in the CTRE simulator with example drive gains, where 8 twitched half as much. |
| It arrives slowly and takes long to settle | Raise `easeInPerSecond` |
| It never reports finished | Loosen the tolerances. A robot that cannot hold 2 cm will not finish at 2 cm. |
| A drive-to-pose overshoots | Lower `onTheFlyMaxMetersPerSecondSquared` in the robot file |

## A slippery floor

The follower has no slip detection. On a floor with less grip than the file says, it does nothing special. If
the robot slips on its paths, lower `frictionCoefficient` to the floor's real value, or lower `tractionShare`.

The size of the effect, from the bench: on a floor with a grip of 0.91 under a plan made for 1.5, wheel odometry
alone ended 31 cm off. Told the floor's real grip, 10.6 cm. A camera corrects the pose either way.

## What not to tune

Everything else in the follower is a named constant: the 0.5 m search window, the 30 cm finish hand-over, the
5 cm/s settle speed and its 0.1 s hold. They are not settings. [The follower](/concepts/the-follower/) lists
them.

## Related

- [FollowerConfig](/reference/follow/followerconfig/) documents each setting.
- [The stopping law](/concepts/stopping-law/) explains braking and the ease-in.
