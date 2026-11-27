---
title: FollowerSim
description: Runs the real path follower on a simulated drivetrain at 50 Hz and records every loop.
sidebar:
  order: 1
---

`FollowerSim` runs a `PathFollower` in a closed loop on a [SimDrivetrain](/reference/sim/simdrivetrain/), from a path's start to its finish, and records what happened in every loop.

It does not run on the robot. You use it to try a path and follower settings on a laptop, in a unit test, before the robot is available. See [Tuning](/guides/tuning/).

Package: `frc.team5712.path.sim`.

## Example

```java
Path path = paths.get(Paths.TO_REEF);
// Tell the follower how late the simulated drivetrain responds
FollowerConfig config = FollowerConfig.defaults(paths.robot()).toBuilder()
    .latencySeconds(SimDrivetrain.responseDelaySeconds(
        SimDrivetrain.DEFAULT_LATENCY_SECONDS, SimDrivetrain.DEFAULT_LAG_SECONDS))
    .build();

// Run on blue, hit the robot sideways with 1.5 m/s one second in,
// record for half a second after the finish, and give up at 20 seconds
FollowerSim.Trace trace = FollowerSim.run(path, config, FollowerSim.Drive.DEFAULT,
    FieldFrame.BLUE, new FollowerSim.Shove(1.0, 0.0, 1.5), 0.5, 20.0);

System.out.println("finished in loop " + trace.finishLoop
    + ", that is " + trace.finishLoop * FollowerSim.LOOP_SECONDS + " s");
```

## Constants

| Constant | What it is |
| --- | --- |
| `LOOP_SECONDS` | `0.020`. Seconds per loop. The run is at 50 Hz. |

## Methods

The class has no public constructor. `run` is its only method.

| Method | What it does |
| --- | --- |
| `public static Trace run(Path path, FollowerConfig config, Drive drive, FieldFrame frame, Shove shove, double afterSeconds, double maxSeconds)` | Drives the whole path once and returns the `Trace`. |

## Parameters of run

| Parameter | What it is |
| --- | --- |
| `path` | The path to follow |
| `config` | The follower's settings |
| `drive` | The simulated drivetrain's numbers |
| `frame` | The alliance frame to run in, for example `FieldFrame.BLUE` |
| `shove` | One hit during the run, or `null` for none |
| `afterSeconds` | How long to keep recording after the follower finishes, in seconds |
| `maxSeconds` | The run stops here even if the follower has not finished, in seconds |

## What a run does

| When | What happens |
| --- | --- |
| Before the first loop | The simulated robot is placed on the path's start pose, already moving at the path's start speed along its start direction. Both are converted into `frame`. |
| Every loop | The drivetrain is advanced to the loop's time, the shove is applied once its time has come, and the pose and velocity are read. |
| The first loop | The follower is started. |
| Every loop until the follower finishes | The follower takes one step and its command is sent to the drivetrain. |
| The loop the follower finishes | `finishLoop` is recorded. If the path ends at rest, a zero command is sent. On a rolling end the last command is left running. |
| The end | The run returns once the follower has finished and `afterSeconds` more have passed, or at `maxSeconds`. |

## Shove

A hit during the run, applied once, in the first loop at or after its time.

| Member | What it is |
| --- | --- |
| `public record Shove(double atSeconds, double dvxMetersPerSecond, double dvyMetersPerSecond)` | The record |
| `atSeconds` | When, in seconds from the start |
| `dvxMetersPerSecond`, `dvyMetersPerSecond` | The robot's velocity changes at once by this much, in m/s in the field frame |

## Drive

The simulated drivetrain's numbers. See [SimDrivetrain](/reference/sim/simdrivetrain/) for what each one does.

| Member | In `Drive.DEFAULT` | What it is |
| --- | --- | --- |
| `public record Drive(double latencySeconds, double lagSeconds, double substepSeconds, double gripMetersPerSecondSquared)` | | The record |
| `latencySeconds` | 0.020 | How long a command takes to take effect, in seconds |
| `lagSeconds` | 0.040 | The time constant with which the velocity follows the command, in seconds |
| `substepSeconds` | 0.001 | The integration step, in seconds |
| `gripMetersPerSecondSquared` | `Double.POSITIVE_INFINITY` | The floor's grip, in m/s²: the cap on how fast the translation can change. Infinity means no cap. |
| `Drive.DEFAULT` | | A public constant with the values in this table. They are the numbers the library's end-to-end tests use. |

## Fired

| Member | What it is |
| --- | --- |
| `public record Fired(String name, int loop)` | One path event as it fired: its name, and the number of the loop it fired in. Loops count from 0. |

## Trace

A public class with one public list per recorded quantity. Every list has one entry per loop, so entry `k` of each belongs to time `k * LOOP_SECONDS`.

| Field | What it is |
| --- | --- |
| `Trace()` | The public constructor with no arguments, which makes an empty trace. `run` fills one in and returns it. |
| `List<Double> t` | The loop's time, in seconds |
| `List<Double> x` | The robot's X, as measured at the start of the loop, in meters |
| `List<Double> y` | The robot's Y, in meters |
| `List<Double> heading` | The robot's heading, in radians |
| `List<Double> vx` | The X velocity commanded that loop, in m/s |
| `List<Double> vy` | The Y velocity commanded that loop, in m/s |
| `List<Double> omega` | The turn rate commanded that loop, in rad/s |
| `List<Double> s` | The follower's progress along the path, in meters |
| `List<Double> crossTrack` | The follower's cross-track error, the sideways distance from the path, in meters |
| `List<Integer> phase` | The follower's phase as a number: 0 `IDLE`, 1 `TRACKING`, 2 `TERMINAL`, 3 `FINISHED`, 4 `CANCELLED` |
| `List<Fired> events` | Every event that fired, in order |
| `int finishLoop` | The loop the follower finished in. -1 if it never did. |

:::note
`vx`, `vy` and `omega` are the command sent, not the velocity the simulated robot reached. After a path that ends at rest has finished, they are zero.
:::

## What the sim package is for

The `sim` package is the library's own test bench. It holds two classes, this one and `SimDrivetrain`, and it has no WPILib in it.

| Who uses it | For what |
| --- | --- |
| The library's tool `FollowFixtureGen` | Writing the **golden traces** in `fixtures/follow`: recorded runs that later versions of the follower are compared against |
| The path editor | Its Sim preview runs a TypeScript copy of the follower and of this loop. The copy is checked against the golden traces, so the preview matches what this class computes |
| The library's unit tests | `SimDrivetrain` drives on-the-fly paths |

## Good to know

- **It is not a robot simulator.** It does not model swerve modules, motors, a battery or sensor noise, and it does not connect to WPILib simulation or to your robot code.
- **A run shows the follower on an ideal drivetrain with a known delay.** It does not show how your robot will behave.

## More

- [SimDrivetrain](/reference/sim/simdrivetrain/): the drivetrain it runs on.
- [PathFollower](/reference/follow/pathfollower/) and [FollowerConfig](/reference/follow/followerconfig/): what it runs.
- [The follower](/concepts/the-follower/): what happens each loop.
