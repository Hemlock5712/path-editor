---
title: The rail law
description: How the robot is held to a line on the field while the driver drives along it.
sidebar:
  order: 16
---

Some parts of a field leave no room to be off-line: under a trench, along the edge of a scoring platform. A driver can hold that line by hand, but it takes attention they need for something else.

A **rail** is a line on the field the robot is held to, like a train on a track. While the driver holds a button, the robot is pulled onto the line and kept there, and the stick moves it along the line. Letting go of the button is the way off.

Three classes do it. [`Rail`](/reference/follow/rail/) describes the line. [`RailLaw`](/reference/follow/raillaw/) says what velocity to want. [`RailFollower`](/reference/follow/railfollower/) runs the law, the limiter and the feed-forward together. Robot code uses [`RailDrive`](/reference/wpilib/raildrive/), and the guide is [Rails](/guides/rails/).

:::caution
The rail is a driver-feel feature. Its numbers come from the simulator and the bench. None of them has been reviewed with a driver on a robot. Treat every value on this page as a default, not a recommendation.
:::

## What a rail is

A `Rail` is a point, a direction, and four optional extras.

| Part | Meaning | Default |
| --- | --- | --- |
| `xMeters`, `yMeters` | Any point on the line | required |
| `directionRadians` | Which way the line runs | required |
| `headingRadians` | A heading to hold | NaN: the driver turns the robot |
| `startMeters`, `endMeters` | Where the rail stops, as distances along it from the point | −∞ and +∞: endless |
| `pullIn` | How fast the rail pulls the robot on, as a share of the driver's speed | +∞: as fast as the robot can |

The rail is in whatever frame the pose is in. It knows nothing about walls or alliances.

## Two directions, two jobs

The law splits everything into two directions: **along** the rail and **across** it.

### Across: a position problem

Across the rail the job is to get onto the line and stay there. That is the same problem as the finish of a path, in one dimension, so it is solved the same way: with the [stopping law](/concepts/stopping-law/). The speed toward the line is the smallest of a top speed, the speed that can still stop on the line, and an ease-in near it.

The braking deceleration is not the full traction limit. It is `ACROSS_SHARE`, 0.55, of it. Grip is one budget, and the driver needs some of it along the rail at the same time.

The part of the driver's stick that points across the rail is not added to the command. On the line, a full stick straight across moves the robot 0 m/s. It is a rail.

### Along: the driver's

Along the rail the speed is the driver's: the part of their stick velocity that points along the line. Two things shape it.

**While the robot is still getting on,** the driver's speed along the rail may only change gently. "Getting on" means more than 5 cm off the line, or still sliding across it faster than 0.05 m/s. The rate allowed is what is left of the grip after the across braking has taken its share, times a margin:

```text
across braking = 0.55 · A
along change   = 0.85 · sqrt(A² − (0.55 · A)²)
```

`A` is the robot's traction limit. For the default robot, at 12.8 m/s², that is 7.0 m/s² across and 9.1 m/s² along. Once the robot is on the rail and no longer sliding across it, this rule is off and only the [output limiter](/concepts/output-limiter/) applies.

**Getting onto the rail has first claim on speed.** The across speed and the along speed together cannot be more than the drive speed available, so the along speed gets what is left:

```text
along speed ≤ sqrt(cap² − across speed²)
```

While the robot is off the rail, `cap` also leaves out `STEER_MARGIN`, 0.1, of top speed. Near top speed the motors have almost nothing left to steer with, and on the bench a robot at full stick overshot the rail by about 15 cm without that spare. The margin fades out over the last 5 cm, `STEER_BLEND_METERS`.

## How hard it pulls: pullIn

With no `pullIn` set, the rail pulls the robot onto the line as fast as the stopping law allows. From a long way off at speed, that is a sharp swerve.

`rail.withPullIn(share)` softens it. The pull across is then capped at a share of the driver's speed along the rail. At 0.5, a driver at 4 m/s along the rail is pulled in at 2 m/s at most. Because across speed and along speed keep the same ratio, the robot slides on at a fixed angle:

```text
angle     = atan(pullIn)
joins at  = (how far off) ÷ pullIn, farther along the rail
```

So the driver chooses where the robot joins the line by when they press the button.

Three more rules apply when a `pullIn` is set:

- **The driver's own push toward the line counts in full.** If they steer at the line harder than the share allows, they get there sooner.
- **Their push away from the line is ignored.**
- **With the stick released the rail still pulls at up to 0.5 m/s**, `SETTLE_METERS_PER_SECOND`. A robot standing off the line settles onto it, and one bumped off comes back, without waiting for the driver to move.

The library's rail test prints this table, for a robot 1.2 m off the line with the driver at 3 m/s along it. It is a simulation.

| `pullIn` | Peak speed across | Travel turned | Time to the line | Joins after | Driver's speed kept |
| --- | --- | --- | --- | --- | --- |
| none | 3.15 m/s | 61° | 0.74 s | 1.9 m | 85 % |
| 1.0 | 3.00 m/s | 48° | 0.72 s | 2.1 m | 97 % |
| 0.5 | 1.50 m/s | 27° | 0.94 s | 2.8 m | 100 % |
| 0.3 | 0.90 m/s | 17° | 1.40 s | 4.2 m | 100 % |
| 0.2 | 0.60 m/s | 11° | 2.02 s | 6.1 m | 100 % |

Which share is right is a question of feel for a driver on a robot. Nothing in the library measures that.

## The ends of a rail

`rail.between(startMeters, endMeters)` gives the rail ends, like the ends of a platform. The driver's along speed is capped by the stopping law on the distance to the end ahead, so the robot stops at an end as it would stop at a goal. At or past an end, no more speed that way is allowed. The driver can always drive back.

## Heading

A rail made with `Rail.through` leaves the heading to the driver. Their spin rate passes straight through.

`rail.withHeading(radians)` locks it. The spin rate then comes from the stopping law for heading, with the follower's spin limits, `headingEaseInPerSecond` and a 0.3° deadband.

Either way, spinning is counted before driving. A wheel's speed is the drive speed plus spin rate times the module radius, and it cannot pass top speed. So the drive speed available is the top speed less the spin's share. Without that, a robot at full stick and full spin crept 5 cm past the rail on the bench.

## Through the limiter, with feed-forward

`RailLaw` only says what velocity to want. `RailFollower` sends it through an `OutputLimiter` built from the robot file, the same limiter teleop uses, so the rail obeys the same traction limit, motor model and spin cap as everything else.

The [feed-forward](/concepts/feed-forward/) is built as it is for a path, from what the law calls for and not from the difference between commands. Across the rail, only the law's own braking counts, because the command there follows the measured pose. Along the rail the command is the driver's, so the limiter's step along the rail is used. A library test holds the rail with a 4 mm wobble in the pose and checks that the feed-forward stays quiet.

## Every constant

| Constant | Value | Meaning |
| --- | --- | --- |
| `ACROSS_SHARE` | 0.55 | The share of the traction limit that braking onto the rail plans on. |
| `ALONG_MARGIN` | 0.85 | The share of the leftover grip the driver gets along the rail while the robot is getting on. |
| `STEER_MARGIN` | 0.1 | The share of top speed kept spare for steering while off the rail. |
| `STEER_BLEND_METERS` | 0.05 m | The steering margin fades out over this distance. Also the "on the rail" distance. |
| `SETTLE_METERS_PER_SECOND` | 0.5 m/s | With a `pullIn` set, the rail still pulls at up to this with the stick released. |
| `ACROSS_SETTLED_METERS_PER_SECOND` | 0.05 m/s | Sliding across the rail slower than this counts as on it. |

The reaction time, the ease-in slopes and the spin limits are not rail constants. They are the [`FollowerConfig`](/reference/follow/followerconfig/) the rail was built with.

## What has not been tried

- No driver has driven a rail on a real robot. The library as a whole has not run on one.
- The `pullIn` share has no recommended value.
- `ACROSS_SHARE`, `ALONG_MARGIN` and `STEER_MARGIN` were set on the bench's simulated drivetrain.

## What it does not do

- It does not decide when a rail is on. Robot code does, usually from a button.
- It does not avoid obstacles or know where the walls are.
- It does not follow a curve. A rail is a straight line.
- It does not move the robot along the rail on its own. With the stick released the robot gets onto the line and stops.
