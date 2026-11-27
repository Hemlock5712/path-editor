---
title: PathFollower
description: The controller that turns a built path and a measured pose into a velocity each loop.
sidebar:
  order: 1
---

`frc.team5712.path.follow.PathFollower` takes a [`Path`](/reference/core/path/) built in robot init and, each loop, turns the measured pose and velocity into a field-relative velocity to command, with no WPILib in it.

Most robot code never calls it: [`FollowPathCommand`](/reference/wpilib/followpathcommand/) and [`PathfindCommand`](/reference/wpilib/pathfindcommand/) wrap one. You build one for a hand-built `FollowPathCommand`, or use it directly in a simulator or a test. See [Follow a path](/guides/follow-a-path/).

## Example

```java
// Robot init
Path toReef = paths.get("ToReef");
PathFollower follower = new PathFollower(toReef, FollowerConfig.defaults(paths.robot()));

// When the path begins. now is a clock in seconds, pose is a Pose in meters and radians,
// velocity is a FieldSpeeds in m/s and rad/s, field-relative.
follower.start(now, pose, velocity, FieldFrame.BLUE);

// Every loop, until follower.isFinished()
FieldSpeeds command = follower.step(now, pose, velocity);
```

## Constructor

| Constructor | What it does |
| --- | --- |
| `PathFollower(Path path, FollowerConfig config)` | A follower for a path built in robot init. Nothing about the path is copied. The path file's events are registered, with no action. |

## Set-up methods

Call these in robot init, before the first `start()`. Each returns the follower, so they chain.

| Method | What it does |
| --- | --- |
| `addEvent(String name, double sMeters, Runnable action)` | Runs `action` when the robot passes `sMeters` along the path. A position outside the path is moved to its start or end. |
| `addEventAtWaypoint(String name, double waypoint, Runnable action)` | The same at a waypoint-relative position. 1.5 is halfway along the arc from waypoint 1 to waypoint 2. |
| `onEvent(Consumer<String> listener)` | Adds a listener called with each event's name as it fires, after the event's own action. Listeners add up. |
| `bindGoal(Supplier<Optional<Pose>> goal)` | Binds the end of the path to a moving goal, in the same frame as the robot's pose. When the goal moves more than 2 cm or 2°, the rest of the path is planned again. An empty value keeps the last goal. |
| `bindGoal(Supplier<Optional<Pose>> goal, double smoothingSeconds)` | The same, averaging the estimates first over `smoothingSeconds`. For estimates that arrive raw from a camera. 0 means no averaging. |
| `finishWhen(BooleanSupplier ready)` | Holds the finish until `ready` is true. The robot still drives to the goal and holds it. A path with a rolling end ignores it. |

:::caution
If the condition given to `finishWhen` never becomes true, the path never finishes. Give the command a timeout.
:::

## Running methods

| Method | Returns | What it does |
| --- | --- | --- |
| `start(double timeSeconds, Pose measuredPose, FieldSpeeds measuredVelocity, FieldFrame frame)` | `void` | Starts the path from the measured velocity. Does no path work. `frame` is a [`FieldFrame`](/reference/core/fieldframe/): `BLUE`, or the flip the red alliance uses. |
| `start(double timeSeconds, Pose measuredPose, FieldSpeeds measuredVelocity, FieldFrame frame, FieldSpeeds handedCommand)` | `void` | Starts with the limiter's command set to `handedCommand`: the previous follower's `endCommand()` on a rolling chain. |
| `step(double timeSeconds, Pose measuredPose, FieldSpeeds measuredVelocity)` | `FieldSpeeds` | One loop. Measured pose and field velocity in, the velocity to command out, both in the robot's frame. Throws `IllegalStateException` if it is called before `start()`. |
| `cancel()` | `void` | Stops tracking. Events that have not fired are dropped. Has no effect unless the path is running. |
| `isFinished()` | `boolean` | True once the path has finished normally. False after a cancel. |
| `endCommand()` | `FieldSpeeds` | The last command, in the robot's frame. On a rolling end, the velocity to hand to the next follower. |
| `commandedAcceleration()` | `FieldSpeeds` | The acceleration the plan calls for now: m/s² along x and y and rad/s² of spin, field-relative. Zero when the path is not running. From the plan, not from the difference between commands. For [`WheelForces`](/reference/follow/wheelforces/). |

## Telemetry methods

All distances are meters, speeds m/s, angles radians and spin rates rad/s.

| Method | Returns | What it gives |
| --- | --- | --- |
| `phase()` | `Phase` | What the follower is doing. |
| `progressMeters()` | `double` | Progress along the path being followed. |
| `remainingMeters()` | `double` | Distance left on the path being followed. |
| `crossTrackMeters()` | `double` | Signed distance from the path, positive to the left of travel. 0 in the `TERMINAL` phase. On a turn on the spot, the distance from the point. |
| `alongTrackMeters()` | `double` | How far ahead of the plan's schedule the robot is. Negative means behind. NaN before the start. |
| `headingErrorRadians()` | `double` | The reference heading minus the measured one at the last step, blue frame. NaN before the first step. |
| `plannedMetersPerSecond()` | `double` | The plan's speed at the robot's progress. NaN before the start. |
| `plannedOmegaRadPerSec()` | `double` | The plan's turn rate at the robot's progress, blue frame. NaN before the start. |
| `measuredMetersPerSecond()` | `double` | The measured speed over the ground at the last step. NaN before the first step. |
| `measuredOmegaRadPerSec()` | `double` | The measured turn rate at the last step, blue frame. NaN before the first step. |
| `elapsedSeconds()` | `double` | Seconds since the start, at the last step. |
| `reference()` | `Pose` | The pose the follower is steering for this loop, in the robot's frame. |
| `goal()` | `Pose` | The goal pose, in the robot's frame. |
| `frame()` | `FieldFrame` | The frame given to `start()`. `BLUE` before the first start. |
| `rollingEnd()` | `boolean` | True when the path's end speed is above zero. |
| `path()` | `Path` | The path this follower was built with. |
| `activePath()` | `Path` | The path being followed now: the built one, or its end re-planned to a moved goal. |
| `eventNames()` | `List<String>` | Every event's name, the file's and the code's, in firing order. A copy. |
| `eventMeters()` | `double[]` | Each event's position along the built path, in the order of `eventNames()`. A copy. |

## PathFollower.Phase

A nested enum: what the follower is doing.

| Value | Meaning |
| --- | --- |
| `IDLE` | Made, and `start()` has not been called. |
| `TRACKING` | Following the path, or turning on the spot. |
| `TERMINAL` | Within 0.3 m of a stopping end, driving straight to the goal with the stopping law. |
| `FINISHED` | Finished normally. |
| `CANCELLED` | Stopped by `cancel()`. |

## Constants

| Constants | Where the numbers are |
| --- | --- |
| None public, and no public fields | Fixed numbers such as the 0.5 m search window and the 0.3 m finish radius are in [The follower](/concepts/the-follower/). The settings are in [`FollowerConfig`](/reference/follow/followerconfig/). |

## Good to know

- **Frames are converted inside.** The pose, velocities and goal are given in the robot's own frame.
- **After a normal finish, `step()` returns zero velocity**, or `endCommand()` on a rolling end. After a cancel it returns zero.
- **The time between steps is held between 0.001 s and 0.1 s.** A step with the same time as the one before, or as `start()`, is treated as one 0.02 s loop. A pose with a NaN in it does not throw: the robot is braked within its limits for that loop.
- **`start()` may be called again** to run the same path again. It clears the record of fired events and any moved goal.

## More

- [The follower](/concepts/the-follower/), [Path events](/concepts/path-events/) and [Moving goals](/concepts/moving-goals/): how it works.
- [Feed-forward](/concepts/feed-forward/): why the acceleration comes from the plan.
- [`PathTelemetry`](/reference/wpilib/pathtelemetry/) and [Telemetry](/guides/telemetry/): where the telemetry methods are published.
