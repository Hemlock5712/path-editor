---
title: SimDrivetrain
description: A simulated swerve chassis with a command delay, a velocity lag and an optional grip cap.
sidebar:
  order: 2
---

`frc.team5712.path.sim.SimDrivetrain` is a swerve drive reduced to its chassis. You send it field-relative velocity commands, and it moves the way a drivetrain with a delay would: the command takes effect a little late, and the velocity then closes in on it gradually. It has no noise in it, so the same commands give the same motion down to the last bit.

It belongs to the `sim` package, the library's own test bench. [FollowerSim](/reference/sim/followersim/) explains what that package is for and what it is not. In short: the library's tests and golden traces run on this class, and the path editor's Sim preview runs a TypeScript copy of it. It is not WPILib simulation, it has no modules or motors, and it does not run on the robot. A student meets it when writing a unit test that drives a path or an on-the-fly run on a laptop.

## The model

- A velocity command takes effect `latencySeconds` after it is sent.
- The chassis velocity then follows the command with a **first-order lag**: in each small step it closes a fixed fraction of the gap that is left.
- Position and heading are integrated every `substepSeconds` from the average of the velocity before and after the step.
- A shove changes the velocity at once. The lag then pulls it back to the command.
- With a grip cap, the translation cannot change faster than that many m/s². A robot asked for more slides at the limit. The turn rate is not capped.

In one step of length `dt`:

```text
k        = 1 - exp(-dt / lagSeconds)
velocity = velocity + (command - velocity) x k
```

With no grip cap the wheels never slip. The follower's own limiter is what keeps its commands inside what the floor allows.

## Example

```java
Pose start = new Pose(0.0, 0.0, 0.0);
SimDrivetrain sim = new SimDrivetrain(start, new FieldSpeeds(0, 0, 0),
    SimDrivetrain.DEFAULT_LATENCY_SECONDS, SimDrivetrain.DEFAULT_LAG_SECONDS,
    SimDrivetrain.DEFAULT_SUBSTEP_SECONDS);

OnTheFly run = new OnTheFly(pathfinder, config, () -> goal);
run.start(0.0, sim.pose(), sim.velocity());

for (int k = 1; !run.isFinished() && k < 1000; k++) {
  double t = k * 0.020;
  sim.advanceTo(t);
  sim.command(run.step(t, sim.pose(), sim.velocity()));
}
```

`pathfinder`, `config` and `goal` are a `Pathfinder`, a `FollowerConfig` and a `Pose`. The loop runs at 50 Hz and gives up after 20 seconds.

## Constants

| Constant | Value | Unit | Description |
| --- | --- | --- | --- |
| `DEFAULT_LATENCY_SECONDS` | `0.020` | seconds | The command delay the library's end-to-end tests use |
| `DEFAULT_LAG_SECONDS` | `0.040` | seconds | The lag time constant those tests use |
| `DEFAULT_SUBSTEP_SECONDS` | `0.001` | seconds | The integration step those tests use |

These describe the test drivetrain. They are not measurements of a real robot.

## Constructors

| Constructor | Description |
| --- | --- |
| `SimDrivetrain(Pose pose, FieldSpeeds velocity, double latencySeconds, double lagSeconds, double substepSeconds)` | A robot at `pose` moving at `velocity`, field-relative, commanded to keep doing so. No grip cap. |
| `SimDrivetrain(Pose pose, FieldSpeeds velocity, double latencySeconds, double lagSeconds, double substepSeconds, double gripMetersPerSecondSquared)` | The same, with the floor's grip capping the translational acceleration at `gripMetersPerSecondSquared`, in m/s². `Double.POSITIVE_INFINITY` means no cap. |

| Parameter | Unit | Description |
| --- | --- | --- |
| `pose` | meters, radians | Where the robot starts |
| `velocity` | m/s, rad/s | Its starting field velocity. This is also the command in force until the first one you send takes effect. |
| `latencySeconds` | seconds | The delay before a command takes effect |
| `lagSeconds` | seconds | The lag's time constant. After one time constant the velocity has closed about 63 percent of the gap. |
| `substepSeconds` | seconds | The integration step |

The simulation's clock starts at 0. The constructors do not check their arguments. Use a `lagSeconds` and a `substepSeconds` above 0.

## Static methods

### responseDelaySeconds(latencySeconds, lagSeconds)

```java
public static double responseDelaySeconds(double latencySeconds, double lagSeconds)
```

How late this drivetrain responds, all told, in seconds:

```text
response delay = latencySeconds + lagSeconds + 0.010
```

The last term is half a 20 ms loop, because a command is held for a whole loop. With the defaults that is 0.020 + 0.040 + 0.010 = 0.070 s. A follower driving this drivetrain should have its `latencySeconds` set to this value, as the example on the [FollowerSim](/reference/sim/followersim/) page does.

## Methods

| Method | Returns | Description |
| --- | --- | --- |
| `command(FieldSpeeds v)` | `void` | Sends a field-relative velocity command now, in m/s and rad/s. It takes effect one latency later. |
| `shove(double dvx, double dvy)` | `void` | A hit: adds `dvx` and `dvy`, in m/s in the field frame, to the chassis velocity now |
| `advanceTo(double until)` | `void` | Integrates the motion up to time `until`, in seconds. Does nothing when `until` is not later than the current time. |
| `time()` | `double` | The simulation's current time, in seconds |
| `pose()` | `Pose` | The current pose, in meters and radians, with the heading wrapped to between -π and π |
| `velocity()` | `FieldSpeeds` | The current field-relative velocity, in m/s and rad/s |

Commands are applied in the order they were sent. Call `advanceTo` with the loop's time, read `pose()` and `velocity()`, then send the next command, as the example does.

:::caution
`pose()` and `velocity()` are the true values with no noise and no delay. A real robot's pose estimate has both. A path that tracks well here has only been shown to track on an ideal drivetrain.
:::

## Related

- [FollowerSim](/reference/sim/followersim/)
- [OnTheFly](/reference/plan/onthefly/) and [PathFollower](/reference/follow/pathfollower/)
- [The follower](/concepts/the-follower/)
