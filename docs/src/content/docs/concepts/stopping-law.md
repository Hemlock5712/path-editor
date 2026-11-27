---
title: The stopping law
description: How the follower chooses a speed that arrives at a goal quickly and still stops on it.
sidebar:
  order: 7
---

Driving to a spot and stopping on it sounds simple. Go too slowly and the robot wastes time. Go too fast for too long and it slides past. The **stopping law** answers one question each loop: given how far the goal is, what is the fastest speed the robot can have right now and still stop on it?

The follower uses this law for the last 0.3 m of every path that stops, for the heading at the finish, and for the brake along the whole path. The rail uses it too. The code is [`StoppingLaw`](/reference/follow/stoppinglaw/), a short class of four static methods. The law comes from DriveToPose, the drive-to-pose command the team built in its lessons, so a student who has done those lessons has seen it before.

## Three speeds, take the smallest

The law works out three speeds and returns the smallest.

```text
speed = min( v_max ,  sqrt(2·a·(d − v·τ)) ,  k·d )
```

| Symbol | Meaning | Unit |
| --- | --- | --- |
| `d` | distance left to the goal | m |
| `v` | the speed commanded last loop | m/s |
| `a` | braking deceleration | m/s² |
| `τ` | reaction time | s |
| `k` | ease-in slope | m/s per meter, which is 1/s |
| `v_max` | top speed | m/s |

Inside a small **deadband** around the goal the law returns zero. The follower uses 3 mm for position and 0.3° for heading.

### The braking curve

A robot braking at a steady `a` from speed `v` needs a distance of `v² / (2a)` to stop. Turn that around and the fastest speed that still stops in distance `d` is:

```text
v = sqrt(2·a·d)
```

That is the ideal curve. It assumes the robot starts braking the instant it is told to. A real drivetrain answers late. During that delay the robot keeps going at its current speed and covers `v·τ` meters. So the law takes that distance off before it does the sum:

```text
v = sqrt(2·a·(d − v·τ))
```

The `v` inside the bracket is the speed last commanded, not the speed being solved for. If `d − v·τ` comes out negative it is treated as zero.

With the defaults for the default robot, `a` is 12.8 m/s² and `τ` is 0.05 s. A robot commanded at 3 m/s with 0.5 m to go has a reaction margin of 0.15 m, so the braking curve allows `sqrt(2 · 12.8 · 0.35)`, which is about 3.0 m/s. It is right on the curve and must start slowing now.

### The ease-in line

The braking curve has a problem close to the goal. A square root is very steep near zero, so in the last centimeters a tiny change in measured distance is a large change in speed. Pose noise then turns into a jittery command.

The ease-in line is a straight line through the goal: `v = k·d`. Close to the goal it is lower than the braking curve, so it takes over and the robot eases in. A speed proportional to the distance left means the distance shrinks smoothly, like a ball rolling to rest, with no hard stop at the end.

With the default `k` of 12 per second and `a` of 12.8 m/s², the two curves cross where `k·d = sqrt(2·a·d)`, which is at `d = 2a / k²`, about 18 cm, ignoring the reaction margin. Outside that the braking curve governs. Inside it the ease-in line does.

### The top speed

Far from the goal both curves give large numbers, so the law is capped at a top speed. At the finish of a path the follower passes in the path's own maximum speed.

### Why the smallest

Each of the three is a limit, not a target. The top speed is the most the path allows. The braking curve is the most that still stops. The ease-in line is the most that stays calm near the goal. A speed that respects all three is the smallest of them. Which one that is changes with distance: top speed far out, braking in the middle, ease-in at the end.

## The same law for heading

Turning to a heading is the same problem in a different unit. `StoppingLaw.spin` takes the heading error in radians in place of the distance, the spin acceleration cap in place of `a`, the spin cap in place of `v_max`, and `headingEaseInPerSecond` in place of `k`. It returns a spin rate with the sign of the error, so the robot turns the short way toward the goal heading.

For the default settings on a robot file with no spin limits, those are 720°/s², 540°/s and 5 per second.

## The brake along a path

`StoppingLaw.brake` is the braking curve on its own, with an end speed:

```text
v = sqrt( v_end² + 2·a·(d − v·τ) )
```

The follower uses it along the whole path as a ceiling on the plan's speed, with `d` as the distance left on the path. For a path that stops, `v_end` is zero and this is exactly the braking part of the full law. That is why the switch to the finish has no seam: both laws brake on the same distance with the same deceleration.

For a path with a rolling end, `v_end` is the path's end speed, so the ceiling never drops below the speed the robot is meant to leave with.

## How hard the law is braking

`StoppingLaw.slowing` reports the deceleration the law is asking for right now, as a positive number. It exists for the [feed-forward](/concepts/feed-forward/). On the braking curve the answer is `a`. On the ease-in line the distance shrinks at `v`, so the speed falls at `k·v`. The method returns the smaller of the two:

```text
slowing = min( a ,  k·v )
```

It returns zero while the law is at its top speed, or while the command is still coming up to the law's speed, because nothing is being slowed then. Because `k·v` fades to nothing as the robot stops, a pose that wobbles while the robot stands on its goal asks for almost no force.

## What each setting changes

The settings live in [`FollowerConfig`](/reference/follow/followerconfig/). The values below are the library's defaults.

| Setting | Default | Raise it | Lower it |
| --- | --- | --- | --- |
| `brakeMetersPerSecondSquared` | NaN, meaning the traction limit | The robot brakes later and harder. | The robot brakes earlier and arrives later. Lower it if the robot overshoots. |
| `latencySeconds` | 0.05 s | A bigger reaction margin: braking starts earlier. | Braking starts later. Too low and the robot overshoots. |
| `easeInPerSecond` | 12 per second | A steeper line: a quicker last few centimeters, more sensitive to pose noise. | A gentler, slower arrival. |
| `headingEaseInPerSecond` | 5 per second | The same, for the last degrees of the turn. | The same. |

`brakeMetersPerSecondSquared` defaults to NaN so that the robot brakes as hard as the plan does. An earlier fixed value of 6 m/s² made the robot brake early and arrive late on paths planned harder than that.

The default ease-in of 12 per second is twice the 6 per second in the team's DriveToPose. The bench sweep picked it on its tuning seeds. It is a default that suited the simulated drivetrain, not a value measured on a robot.

:::note
In the simulator, a robot braking at the full 12.8 m/s² into a drivetrain that answers in about 70 ms rolled about 2 cm past the goal and crept back. On a real robot the latency, lag and pose noise are its own, so `latencySeconds` and `easeInPerSecond` are the two settings to tune there. See [Tuning](/guides/tuning/).
:::

## What it does not do

- It does not know about grip. It asks for a speed, and the [output limiter](/concepts/output-limiter/) decides how fast the command may change to reach it.
- It does not steer around anything. It gives a speed along a straight line to the goal.
- It has no memory. Each call is one formula on the numbers passed in.
