---
title: RailLaw
description: Works out the velocity that holds the robot to a rail while the driver drives.
sidebar:
  order: 8
---

`frc.team5712.path.follow.RailLaw` says what velocity the robot should want in order to stay on a [`Rail`](/reference/follow/rail/) while the driver drives along it.

Robot code uses [`RailDrive`](/reference/wpilib/raildrive/), which wraps a [`RailFollower`](/reference/follow/railfollower/), which owns a `RailLaw`. You are unlikely to call it directly, but its constants shape how a rail feels. The guide is [Rails](/guides/rails/).

## Example

```java
RailLaw law = new RailLaw(FollowerConfig.defaults(paths.robot()));

FieldSpeeds wanted = law.wanted(
    rail, pose, lastCommand, fromTheSticks,
    maxSpeed, dt, Double.POSITIVE_INFINITY);   // no cap on the pull across
// wanted still has to go through an OutputLimiter
```

## Constructor

| Constructor | What it does |
| --- | --- |
| `RailLaw(FollowerConfig config)` | A law with the limits of the robot in `config`. It uses the config's `latencySeconds`, `easeInPerSecond`, `headingEaseInPerSecond` and spin limits, and the robot's traction limit and module radius. |

## Methods

Nothing is thrown.

| Method | Returns | What it does |
| --- | --- | --- |
| `acrossBrakeMetersPerSecondSquared()` | `double` | How hard the robot brakes onto the rail, m/s². |
| `alongAccelMetersPerSecondSquared()` | `double` | How fast the driver's speed along the rail may change while the robot is still getting onto it, m/s². |
| `wanted(Rail rail, Pose pose, FieldSpeeds commanded, FieldSpeeds driver, double maxSpeed, double dt, double maxCrossSpeed)` | `FieldSpeeds` | The velocity the robot should want this loop, in field directions, m/s and rad/s. |

### Parameters of wanted

| Parameter | Unit | Meaning |
| --- | --- | --- |
| `rail` | | The rail to hold. It may be a different rail each loop. |
| `pose` | m, rad | Where the robot is, or the point on it being held to the rail, such as its intake. |
| `commanded` | m/s, rad/s | Last loop's command, in field directions. Used for the reaction-time margin. |
| `driver` | m/s, rad/s | The driver's stick velocity in field directions. |
| `maxSpeed` | m/s | Top speed. Getting onto the rail has first claim on it. |
| `dt` | s | Seconds since the last loop. |
| `maxCrossSpeed` | m/s | Never pull across the rail faster than this. `Double.POSITIVE_INFINITY` for no cap. |

### What goes into wanted

| Part | Rule |
| --- | --- |
| Spin | The driver's spin rate, or with a heading to hold, the stopping law's spin rate toward that heading with a 0.3° deadband. |
| Drive speed available | `maxSpeed` less the spin's share of wheel speed, which is the spin rate times the module radius. |
| Across | The stopping law on the distance to the line, braking at the across rate, capped by `maxCrossSpeed`, by the drive speed available, and by the rail's `pullIn` when it has one. The driver's push away from the line is not used. |
| Along | The driver's speed along the line, capped by the stopping law on the distance to each end. While the robot is more than `STEER_BLEND_METERS` off the line, or sliding across faster than `ACROSS_SETTLED_METERS_PER_SECOND`, it may change by no more than the along rate times `dt`. |
| Sharing speed | The along speed is held to what the across speed leaves of the drive speed, less the steering margin while off the rail. |
| Pull with a `pullIn` set | At most the largest of `SETTLE_METERS_PER_SECOND`, `pullIn` times the driver's speed along the rail, and the driver's own speed toward the line. |

## Constants

All are `public static final double`.

| Constant | Value | Meaning |
| --- | --- | --- |
| `ACROSS_SHARE` | 0.55 | The share of the traction limit that braking onto the rail plans on. |
| `ALONG_MARGIN` | 0.85 | The share of the grip left over that the driver gets along the rail while the robot is still getting on. |
| `STEER_MARGIN` | 0.1 | While off the rail, this share of top speed is kept spare for steering. |
| `STEER_BLEND_METERS` | 0.05 m | The steering margin fades out over this last distance to the line. |
| `SETTLE_METERS_PER_SECOND` | 0.5 m/s | With a `pullIn` share set, the rail still pulls at up to this however slowly the driver is going along it. |
| `ACROSS_SETTLED_METERS_PER_SECOND` | 0.05 m/s | Sliding across the rail slower than this counts as on it. |

## The two rates

Worked out once, in the constructor. For the default robot, with a traction limit of 12.8 m/s², they are 7.0 m/s² and 9.1 m/s².

```text
across braking = ACROSS_SHARE · traction
along change   = ALONG_MARGIN · sqrt(traction² − across braking²)
```

## Good to know

- **It only says what velocity to want.** The result still has to go through an [`OutputLimiter`](/reference/follow/outputlimiter/). `RailFollower` does both.
- **All inputs must be in one frame**, the pose's.

:::caution
The rail is a driver-feel feature. The constants were set on the bench's simulated drivetrain and have not been reviewed with a driver on a robot. They are defaults, not recommendations.
:::

## More

- [The rail law](/concepts/rail-law/) and [The stopping law](/concepts/stopping-law/): each rule explained.
- [`FollowerConfig`](/reference/follow/followerconfig/): the settings it reads.
- [`PieceAssist`](/reference/follow/pieceassist/): the other driver assist.
