---
title: FollowerSim
description: Runs the real path follower on a simulated drivetrain at 50 Hz and records every loop.
sidebar:
  order: 1
---

`frc.team5712.path.sim.FollowerSim` runs a `PathFollower` in a closed loop on a [SimDrivetrain](/reference/sim/simdrivetrain/) and records what happened in every loop. One call drives a whole path from its start to its finish, with an optional shove on the way, and returns the recording.

## What the sim package is for

The `sim` package is the library's own test bench. It holds two classes, this one and `SimDrivetrain`, and it has no WPILib in it.

- The library's tool `FollowFixtureGen` uses it to write the **golden traces** in `fixtures/follow`: recorded runs that later versions of the follower are compared against.
- The path editor has a TypeScript copy of the follower and of this loop. Its Sim preview runs that copy, and the copy is checked against the golden traces, so the preview in the editor matches what this class computes.
- The library's unit tests use `SimDrivetrain` to drive on-the-fly paths.

It is not a robot simulator. It does not model swerve modules, motors, a battery or sensor noise, and it does not connect to WPILib simulation or to your robot code. It does not run on the robot. A student uses it to try a path and follower settings on a laptop, in a unit test, before the robot is available. A run here shows how the follower behaves on an ideal drivetrain with a known delay. It does not show how your robot will behave.

## Example

```java
Path path = paths.get(Paths.TO_REEF);
FollowerConfig config = FollowerConfig.defaults(paths.robot()).toBuilder()
    .latencySeconds(SimDrivetrain.responseDelaySeconds(
        SimDrivetrain.DEFAULT_LATENCY_SECONDS, SimDrivetrain.DEFAULT_LAG_SECONDS))
    .build();

FollowerSim.Trace trace = FollowerSim.run(path, config, FollowerSim.Drive.DEFAULT,
    FieldFrame.BLUE, new FollowerSim.Shove(1.0, 0.0, 1.5), 0.5, 20.0);

System.out.println("finished in loop " + trace.finishLoop
    + ", that is " + trace.finishLoop * FollowerSim.LOOP_SECONDS + " s");
```

This runs the path on blue, hits the robot sideways with 1.5 m/s one second in, records for half a second after the finish, and gives up at 20 seconds.

## Constants

| Constant | Value | Description |
| --- | --- | --- |
| `LOOP_SECONDS` | `0.020` | Seconds per loop. The run is at 50 Hz. |

## run

```java
public static Trace run(Path path, FollowerConfig config, Drive drive, FieldFrame frame,
    Shove shove, double afterSeconds, double maxSeconds)
```

| Parameter | Unit | Description |
| --- | --- | --- |
| `path` | | The path to follow |
| `config` | | The follower's settings |
| `drive` | | The simulated drivetrain's numbers |
| `frame` | | The alliance frame to run in, for example `FieldFrame.BLUE` |
| `shove` | | One hit during the run, or `null` for none |
| `afterSeconds` | seconds | How long to keep recording after the follower finishes |
| `maxSeconds` | seconds | The run stops here even if the follower has not finished |

What a run does:

1. The simulated robot is placed on the path's start pose, already moving at the path's start speed along its start direction. Both are converted into `frame`.
2. Each loop, the drivetrain is advanced to the loop's time, the shove is applied once its time has come, and the pose and velocity are read.
3. In the first loop the follower is started. In every loop until it finishes, the follower takes one step and its command is sent to the drivetrain.
4. In the loop the follower finishes, `finishLoop` is recorded. If the path ends at rest, a zero command is sent. On a rolling end the last command is left running.
5. The run returns once the follower has finished and `afterSeconds` more have passed, or at `maxSeconds`.

Returns the `Trace` described below. The class has no public constructor. `run` is its only method.

## Nested types

### Shove

```java
public record Shove(double atSeconds, double dvxMetersPerSecond, double dvyMetersPerSecond)
```

A hit during the run. At `atSeconds`, in seconds from the start, the robot's velocity changes at once by `dvxMetersPerSecond` and `dvyMetersPerSecond`, in m/s in the field frame. It is applied in the first loop at or after that time, once.

### Drive

```java
public record Drive(double latencySeconds, double lagSeconds, double substepSeconds,
    double gripMetersPerSecondSquared)
```

The simulated drivetrain's numbers. See [SimDrivetrain](/reference/sim/simdrivetrain/) for what each one does.

| Component | Unit | In `Drive.DEFAULT` | Description |
| --- | --- | --- | --- |
| `latencySeconds` | seconds | 0.020 | How long a command takes to take effect |
| `lagSeconds` | seconds | 0.040 | The time constant with which the velocity follows the command |
| `substepSeconds` | seconds | 0.001 | The integration step |
| `gripMetersPerSecondSquared` | m/s² | `Double.POSITIVE_INFINITY` | The floor's grip: the cap on how fast the translation can change. Infinity means no cap. |

`Drive.DEFAULT` is a public constant with the values in the table. They are the numbers the library's end-to-end tests use.

### Fired

```java
public record Fired(String name, int loop)
```

One path event as it fired: its name, and the number of the loop it fired in. Loops count from 0.

### Trace

A public class with one public list per recorded quantity. Every list has one entry per loop, so entry `k` of each belongs to time `k * LOOP_SECONDS`.

| Field | Type | Unit | Description |
| --- | --- | --- | --- |
| `t` | `List<Double>` | seconds | The loop's time |
| `x` | `List<Double>` | meters | The robot's X, as measured at the start of the loop |
| `y` | `List<Double>` | meters | The robot's Y |
| `heading` | `List<Double>` | radians | The robot's heading |
| `vx` | `List<Double>` | m/s | The X velocity commanded that loop |
| `vy` | `List<Double>` | m/s | The Y velocity commanded that loop |
| `omega` | `List<Double>` | rad/s | The turn rate commanded that loop |
| `s` | `List<Double>` | meters | The follower's progress along the path |
| `crossTrack` | `List<Double>` | meters | The follower's cross-track error, the sideways distance from the path |
| `phase` | `List<Integer>` | | The follower's phase as a number: 0 `IDLE`, 1 `TRACKING`, 2 `TERMINAL`, 3 `FINISHED`, 4 `CANCELLED` |
| `events` | `List<Fired>` | | Every event that fired, in order |
| `finishLoop` | `int` | | The loop the follower finished in. -1 if it never did. |

:::note
`vx`, `vy` and `omega` are the command sent, not the velocity the simulated robot reached. After a path that ends at rest has finished, they are zero.
:::

`Trace` has a public constructor with no arguments, which makes an empty trace. `run` fills one in and returns it.

## Related

- [SimDrivetrain](/reference/sim/simdrivetrain/)
- [PathFollower](/reference/follow/pathfollower/) and [FollowerConfig](/reference/follow/followerconfig/)
- [The follower](/concepts/the-follower/)
- [Tuning](/guides/tuning/)
