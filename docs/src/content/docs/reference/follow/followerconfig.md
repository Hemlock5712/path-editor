---
title: FollowerConfig
description: The eight follower settings a team may change, with their defaults.
sidebar:
  order: 2
---

`frc.team5712.path.follow.FollowerConfig` is a record that holds the robot model and the eight settings a team may change for [`PathFollower`](/reference/follow/pathfollower/).

Robot code makes one in robot init and passes it to [`Pathfinding`](/reference/wpilib/pathfinding/), to a `PathFollower` for a [`FollowPathCommand`](/reference/wpilib/followpathcommand/), or to [`RailDrive`](/reference/wpilib/raildrive/). [Tuning](/guides/tuning/) covers which settings to change and in what order.

## Example

```java
// The defaults. paths.robot() is the robot file.
FollowerConfig config = FollowerConfig.defaults(paths.robot());

// The defaults with two settings changed.
// These numbers show the shape of the call. They are not values to copy.
FollowerConfig measured = FollowerConfig.builder(paths.robot())
    .latencySeconds(0.04)
    .positionToleranceMeters(0.03)
    .build();
```

## Settings

These are the record's components. Each has an accessor of the same name.

| Component | Type | Default | What it is |
| --- | --- | --- | --- |
| `robot` | [`RobotConfig`](/reference/core/robotconfig/) | required | The robot model: the limiter's motor curve, spin lever, traction limit and spin limits. |
| `latencySeconds` | `double` | 0.05 s | How late the drivetrain answers a command. The plan is read this far ahead in plan time, and never less than one 0.02 s loop. The stopping law leaves this much reaction time. Measure it on the robot. |
| `crossTrackKp` | `double` | 5.0 | Sideways m/s per meter off the path. |
| `headingKp` | `double` | 5.0 | rad/s per radian of heading error. |
| `brakeMetersPerSecondSquared` | `double` | NaN | The stopping law's deceleration, m/s². NaN means the robot's traction limit, so the robot brakes as hard as the plan does. Lower it if the robot overshoots. |
| `easeInPerSecond` | `double` | 12.0 | The ease-in line at the finish, m/s per meter left. |
| `headingEaseInPerSecond` | `double` | 5.0 | The same for heading, rad/s per radian left. |
| `positionToleranceMeters` | `double` | 0.02 m | Finished: this close to the goal. |
| `headingToleranceRadians` | `double` | `Math.toRadians(1.0)` | Finished: this well turned. |

## Constructor

| Constructor | What it does |
| --- | --- |
| The record's canonical constructor | Public, with all nine components in the order above. It checks nothing. The builder is the usual way to make a config. |

## Static methods

| Method | Returns | What it does |
| --- | --- | --- |
| `defaults(RobotConfig robot)` | `FollowerConfig` | The defaults for a robot. |
| `builder(RobotConfig robot)` | `FollowerConfig.Builder` | A builder with every setting at its default. |

## Instance methods

| Method | Returns | What it does |
| --- | --- | --- |
| `toBuilder()` | `FollowerConfig.Builder` | A builder that starts from this config's values. |
| `maxOmegaRadPerSec()` | `double` | The spin cap in use, rad/s: the robot's `maxAngularRadPerSec`, or 540°/s if the robot has none. |
| `maxAlphaRadPerSecSquared()` | `double` | The spin acceleration cap in use, rad/s²: the robot's `maxAngularRadPerSecSquared`, or 720°/s² if the robot has none. Used by the limiter and the heading's stopping law. |
| `brakeDecel()` | `double` | The stopping law's deceleration in use, m/s²: `brakeMetersPerSecondSquared`, or the robot's traction limit when that is NaN. |

## FollowerConfig.Builder

Every setting starts at its default. Each setter returns the builder, except `set`.

| Method | Sets |
| --- | --- |
| `latencySeconds(double v)` | `latencySeconds`, seconds |
| `crossTrackKp(double v)` | `crossTrackKp` |
| `headingKp(double v)` | `headingKp` |
| `brakeMetersPerSecondSquared(double v)` | `brakeMetersPerSecondSquared`, m/s². NaN for the traction limit. |
| `easeInPerSecond(double v)` | `easeInPerSecond` |
| `headingEaseInPerSecond(double v)` | `headingEaseInPerSecond` |
| `positionToleranceMeters(double v)` | `positionToleranceMeters`, meters |
| `headingToleranceRadians(double v)` | `headingToleranceRadians`, radians |
| `set(String name, double v)` | The setting with that component name. Returns `true`, or `false` for an unknown name, so it does not chain. For option maps such as the bench's. |
| `build()` | Returns the `FollowerConfig`. |

## Good to know

- **The defaults have not been tuned on a real robot.** They come from the team's DriveToPose command and from sweeps on the bench's simulated drivetrain.
- **The spin limits are not follower settings.** To change them, set `maxAngularRadPerSec` and `maxAngularRadPerSecSquared` in [the robot file](/files/robot-file/).
- **The builder's constructor is private.** Use `FollowerConfig.builder(robot)` or `toBuilder()`.
- **[`RailLaw`](/reference/follow/raillaw/) and [`RailFollower`](/reference/follow/railfollower/) take the same config.** They use its reaction time, ease-in slopes and spin limits.

:::note
No setter checks its value. A negative gain or a zero tolerance is accepted, and the follower will misbehave. Change one setting at a time.
:::

## More

- [The follower](/concepts/the-follower/): what each setting does in a step.
- [The stopping law](/concepts/stopping-law/): the brake and ease-in settings.
- [`PathfindCommand`](/reference/wpilib/pathfindcommand/): its `withTolerance` sets the finish tolerances for one command.
