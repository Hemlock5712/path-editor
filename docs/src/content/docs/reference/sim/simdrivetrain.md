---
title: SimDrivetrain
description: A simulated swerve chassis with a command delay, a velocity lag and an optional grip cap.
sidebar:
  order: 2
---

`SimDrivetrain` is a swerve drive reduced to its chassis: you send it field-relative velocity commands, and it moves the way a drivetrain with a delay would.

It does not run on the robot. You meet it when writing a unit test that drives a path or an on-the-fly run on a laptop. [FollowerSim](/reference/sim/followersim/) explains what the `sim` package is for and what it is not.

Package: `frc.team5712.path.sim`.

## Example

```java
// pathfinder, config and goal are a Pathfinder, a FollowerConfig and a Pose
Pose start = new Pose(0.0, 0.0, 0.0);
SimDrivetrain sim = new SimDrivetrain(start, new FieldSpeeds(0, 0, 0),
    SimDrivetrain.DEFAULT_LATENCY_SECONDS, SimDrivetrain.DEFAULT_LAG_SECONDS,
    SimDrivetrain.DEFAULT_SUBSTEP_SECONDS);

OnTheFly run = new OnTheFly(pathfinder, config, () -> goal);
run.start(0.0, sim.pose(), sim.velocity());

// 50 Hz, giving up after 20 seconds
for (int k = 1; !run.isFinished() && k < 1000; k++) {
  double t = k * 0.020;
  sim.advanceTo(t);                                        // move up to this loop's time
  sim.command(run.step(t, sim.pose(), sim.velocity()));    // read, then send the next command
}
```

## Constants

These describe the test drivetrain. They are not measurements of a real robot.

| Constant | What it is |
| --- | --- |
| `DEFAULT_LATENCY_SECONDS` | `0.020` seconds. The command delay the library's end-to-end tests use |
| `DEFAULT_LAG_SECONDS` | `0.040` seconds. The lag time constant those tests use |
| `DEFAULT_SUBSTEP_SECONDS` | `0.001` seconds. The integration step those tests use |

## Constructors

The simulation's clock starts at 0. The constructors do not check their arguments. Use a `lagSeconds` and a `substepSeconds` above 0.

| Constructor | What it does |
| --- | --- |
| `SimDrivetrain(Pose pose, FieldSpeeds velocity, double latencySeconds, double lagSeconds, double substepSeconds)` | A robot at `pose` moving at `velocity`, field-relative, commanded to keep doing so. No grip cap. |
| `SimDrivetrain(Pose pose, FieldSpeeds velocity, double latencySeconds, double lagSeconds, double substepSeconds, double gripMetersPerSecondSquared)` | The same, with the floor's grip capping the translational acceleration at `gripMetersPerSecondSquared`, in m/s². `Double.POSITIVE_INFINITY` means no cap. |

| Parameter | What it is |
| --- | --- |
| `pose` | Where the robot starts, in meters and radians |
| `velocity` | Its starting field velocity, in m/s and rad/s. This is also the command in force until the first one you send takes effect. |
| `latencySeconds` | The delay before a command takes effect, in seconds |
| `lagSeconds` | The lag's time constant, in seconds. After one time constant the velocity has closed about 63 percent of the gap. |
| `substepSeconds` | The integration step, in seconds |

## Static methods

| Method | What it does |
| --- | --- |
| `public static double responseDelaySeconds(double latencySeconds, double lagSeconds)` | How late this drivetrain responds, all told, in seconds: `latencySeconds + lagSeconds + 0.010`. The last term is half a 20 ms loop, because a command is held for a whole loop. With the defaults that is 0.020 + 0.040 + 0.010 = 0.070 s. |

## Methods

| Method | What it does |
| --- | --- |
| `void command(FieldSpeeds v)` | Sends a field-relative velocity command now, in m/s and rad/s. It takes effect one latency later. |
| `void shove(double dvx, double dvy)` | A hit: adds `dvx` and `dvy`, in m/s in the field frame, to the chassis velocity now. |
| `void advanceTo(double until)` | Integrates the motion up to time `until`, in seconds. Does nothing when `until` is not later than the current time. |
| `double time()` | The simulation's current time, in seconds. |
| `Pose pose()` | The current pose, in meters and radians, with the heading wrapped to between -π and π. |
| `FieldSpeeds velocity()` | The current field-relative velocity, in m/s and rad/s. |

## The model

| Part | What it does |
| --- | --- |
| Delay | A velocity command takes effect `latencySeconds` after it is sent. Commands are applied in the order they were sent |
| Lag | The chassis velocity then follows the command with a **first-order lag**. In a step of length `dt` it closes the fraction `k = 1 - exp(-dt / lagSeconds)` of the gap that is left: `velocity = velocity + (command - velocity) x k` |
| Integration | Position and heading are integrated every `substepSeconds` from the average of the velocity before and after the step |
| Shove | Changes the velocity at once. The lag then pulls it back to the command |
| Grip cap | The translation cannot change faster than that many m/s². A robot asked for more slides at the limit. The turn rate is not capped |
| No grip cap | The wheels never slip. The follower's own limiter is what keeps its commands inside what the floor allows |

## Good to know

- **Set the follower's `latencySeconds` to `responseDelaySeconds`** when it drives this drivetrain, as the example on the [FollowerSim](/reference/sim/followersim/) page does.
- **Each loop**: call `advanceTo` with the loop's time, read `pose()` and `velocity()`, then send the next command.
- **There is no noise in it**, so the same commands give the same motion down to the last bit.
- **Who runs on it.** The library's tests and golden traces run on this class, and the path editor's Sim preview runs a TypeScript copy of it. It is not WPILib simulation, and it has no modules or motors.

:::caution
`pose()` and `velocity()` are the true values with no noise and no delay. A real robot's pose estimate has both. A path that tracks well here has only been shown to track on an ideal drivetrain.
:::

## More

- [FollowerSim](/reference/sim/followersim/): runs a whole path on this drivetrain.
- [OnTheFly](/reference/plan/onthefly/) and [PathFollower](/reference/follow/pathfollower/): what you drive it with.
- [The follower](/concepts/the-follower/): what happens each loop.
