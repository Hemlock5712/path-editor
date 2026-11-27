---
title: Moving goals
description: How a path's end follows a goal that vision keeps correcting, and when to average it.
sidebar:
  order: 10
---

A drawn path ends at a fixed spot. The thing the robot is driving to often is not quite there. A game piece has rolled 10 cm. A scoring peg sits 3 cm from where the field map says. A camera can see where the thing really is, but only once the robot is close, and by then the robot is already on its way.

A **moving goal** binds the end of the path to a supplier that robot code keeps up to date. The follower reads it every loop and bends the rest of the path to end on the latest value. The method is `bindGoal` on [`PathFollower`](/reference/follow/pathfollower/). Robot code reaches it through `withMovingGoal` on [`FollowPathCommand`](/reference/wpilib/followpathcommand/) and [`PathfindCommand`](/reference/wpilib/pathfindcommand/). The task-first page is [Moving goals](/guides/moving-goals/) in the guides.

## The supplier

The goal is a `Supplier<Optional<Pose>>`. Each step the follower calls it once.

- A pose is the latest estimate of the goal. It is in the same frame as the robot's measured pose.
- An empty value means there is no new estimate. The follower holds the last goal.

Until the first estimate arrives, the goal is the end of the path as drawn.

## Re-planning the rest of the path

When the goal has moved more than 2 cm, or turned more than 2°, from the goal the follower is currently driving to, the follower plans the rest of the path again. The new piece is one quintic curve from the point on the path where the robot is now to the new goal.

The piece is built to join without a bump:

- **Position, direction and curvature** at the join match the path the robot is on. The wheels are not asked to jerk.
- **Heading** blends from the planned heading at the join to the goal's heading.
- **Speed** starts at the planned speed at the join and ends at the path's own end speed. An intake path that was drawn to drive through its piece at 1 m/s still drives through the piece at 1 m/s.
- **Approach direction** turns with the goal. It is the old path's final direction, turned by the same angle the goal's heading changed. If that would have the robot approach from past the goal, the piece heads straight in.
- **Speed and acceleration limits** are the path's own maximums.

The piece is a full path with its own speed plan, built during the match. It is small: two waypoints.

Events that have not fired yet keep their place in proportion. An event that was 40 % of the way through the distance left is 40 % of the way along the new piece.

### Near the end, no re-plan

Once the robot is within 0.45 m of the goal, a moved goal does not re-plan anything. That is the 0.3 m finish radius plus a 0.15 m minimum for the new piece. The goal is simply updated, and when the finish takes over, the [stopping law](/concepts/stopping-law/) drives straight at the latest value. In the finish itself the goal follows every estimate, with no 2 cm threshold.

## Raw estimates and the noise problem

A camera's estimate of a pose jitters by a centimeter or two from one frame to the next, even when nothing is moving. Bind that directly and the goal jumps every loop. The robot chases each jump. With 2 cm of noise on the bench, a tag alignment bound this way never settled.

The second form of `bindGoal` takes a smoothing time and averages the estimates before using them:

```java
follower.bindGoal(() -> vision.pieceIfSeen(), 0.5);
```

### How the average works

Each new estimate moves the average a fraction `k` of the way toward itself:

```text
k = max( 1/n ,  1 − e^(−dt / T) )
average = average + k · (estimate − average)
```

`n` is the number of estimates so far, `dt` is the loop time and `T` is the smoothing time in seconds.

The two parts of `k` do two different jobs. With `k = 1/n`, the result is the plain mean of every estimate so far, which is the best guess at a goal that is standing still. So at the start all estimates count equally and the average settles as fast as it can. As `n` grows, `1/n` shrinks, and the second part takes over. From then on it is a running average over about `T` seconds, so a goal that creeps is still followed.

With a 0.02 s loop and `T` of 0.5 s the changeover comes after about 25 estimates.

The heading is averaged the same way, the short way round.

### The stray-estimate rule

Sometimes one estimate is simply wrong: the camera matched the wrong tag, or a pose solve flipped. An estimate more than 25 cm or 20° from the average is treated as suspect.

- One or two in a row are ignored. The average does not move.
- The third in a row means the old average was the wrong one. The average starts again from that estimate, with `n` back to 1.

Any estimate that agrees with the average resets the count.

## Plain or averaged

| Use | When |
| --- | --- |
| `bindGoal(goal)` | The supplier already filters its estimates. [`TagGoal`](/reference/wpilib/taggoal/) does. |
| `bindGoal(goal)` | The goal is held in the odometry frame and moves with the robot's own pose corrections. Averaging would make it lag those corrections. |
| `bindGoal(goal, seconds)` | The estimates arrive raw from a camera, for a goal that stands still or creeps. The library's javadoc suggests 0.5 s as a starting value. |

On the bench, the averaged goal settled as soon as a goal with no noise at all. That is a simulator result.

:::caution
Do not average a goal that is already filtered. The two filters stack, and the goal lags. A `TagGoal` is bound with the plain form.
:::

## Holding the finish

A goal often starts as a guess. The command sets off for the field map's spot, and the cameras lock on to the real tag a second later. If the robot reaches the guess first, the path would finish in the wrong place.

`finishWhen(condition)` holds the finish until the condition is true. The robot still drives to the goal, stops on it and keeps holding it. The path is not finished until the robot has settled and the condition is true.

```java
follower.bindGoal(() -> vision.scoringPoseIfSeen())
    .finishWhen(() -> vision.hasLock());
```

Two things to know. A path with a rolling end ignores `finishWhen`, because it hands over as it crosses its end line. And if the condition never becomes true, the path never finishes, so give the command a timeout.

## What it does not do

**The new piece knows no obstacles.** It is one smooth curve from the robot to the goal, and nothing checks it against the field. For a goal that moves a few centimeters that is fine. For a goal that moves a long way it is not.

On-the-fly paths cover this. When the goal has moved more than 10 cm from where the path was planned to, [`OnTheFly`](/reference/plan/onthefly/) plans again around the obstacles. The two work together: the follower's piece handles the small moves every loop, and the pathfinder handles the large ones. On the bench, either alone fell short. See [On-the-fly paths](/concepts/on-the-fly/).

A drawn path from the library has no pathfinder behind it. If its goal can move far enough to matter, draw the path so that the last stretch is clear of obstacles.

Other limits:

- The path's constraint zones and face-point zones are not carried into the new piece.
- A turn on the spot stops being one once its goal moves and the tail is re-planned.
- The follower does not latch estimates in the odometry frame. If a goal needs that, the supplier does it. [The tag filter](/concepts/tag-filter/) is a supplier that does.
- The library has not run on a real robot. The thresholds here are defaults from the simulator and the bench.
