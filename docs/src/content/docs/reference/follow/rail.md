---
title: Rail
description: A line on the field the robot is held to while the driver drives along it.
sidebar:
  order: 7
---

`frc.team5712.path.follow.Rail` is a record that describes a line the robot is held to, like a train on a rail: under a trench, along a scoring platform, or through a game piece.

Robot code makes a `Rail` and passes it to `onRail` on [`RailDrive`](/reference/wpilib/raildrive/) every loop. `RailDrive.rail(point, direction)` makes one from a `Translation2d` and a `Rotation2d`. The guide is [Rails](/guides/rails/).

## Example

```java
// A line through (3.0, 1.2) running along +x, holding a heading of 0,
// from 0.5 m behind the point to 4 m ahead of it, pulled on at half the driver's speed.
// The numbers are examples.
Rail trench = Rail.through(3.0, 1.2, 0.0)
    .withHeading(0.0)
    .between(-0.5, 4.0)
    .withPullIn(0.5);
```

## Components

Each component has an accessor of the same name.

| Component | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `xMeters` | `double` | m | Any point on the rail, x. |
| `yMeters` | `double` | m | The same point, y. |
| `directionRadians` | `double` | rad | Which way the rail runs. "Along" distances are measured from the point this way. |
| `headingRadians` | `double` | rad | The heading to hold, or NaN to let the driver turn. |
| `startMeters` | `double` | m | The rail's near end, as an along-distance. `Double.NEGATIVE_INFINITY` for none. |
| `endMeters` | `double` | m | The rail's far end, as an along-distance. `Double.POSITIVE_INFINITY` for none. |
| `pullIn` | `double` | none | How fast the rail pulls the robot onto the line, as a share of the driver's speed along it. `Double.POSITIVE_INFINITY` for as fast as the robot can. |

## Constructors

| Constructor | What it does |
| --- | --- |
| `Rail(double xMeters, double yMeters, double directionRadians, double headingRadians, double startMeters, double endMeters, double pullIn)` | The canonical constructor, with every component. It checks nothing. |
| `Rail(double xMeters, double yMeters, double directionRadians, double headingRadians, double startMeters, double endMeters)` | A rail that pulls the robot on as fast as it can: `pullIn` is `Double.POSITIVE_INFINITY`. |

## Static methods

| Method | Returns | What it does |
| --- | --- | --- |
| `through(double xMeters, double yMeters, double directionRadians)` | `Rail` | An endless rail through a point, running `directionRadians`, with the heading free and no `pullIn`. The usual starting point. |

## Instance methods

A `Rail` cannot be changed. Each method that returns a `Rail` returns a new one and leaves the other values as they were.

| Method | Returns | What it does |
| --- | --- | --- |
| `withHeading(double newHeadingRadians)` | `Rail` | Also hold this heading, in radians. |
| `between(double newStartMeters, double newEndMeters)` | `Rail` | Stop at these along-distances from the point, in meters, like the ends of a platform. |
| `withPullIn(double shareOfDriverSpeed)` | `Rail` | Pull the robot onto the line at no more than this share of the driver's speed along the rail. Throws `IllegalArgumentException` with "the pull-in share must be more than 0" when the share is 0, negative or NaN. |
| `holdsHeading()` | `boolean` | True when `headingRadians` is not NaN. |

## Constants

`Rail` has no constants.

## Good to know

- **A rail is in whatever frame the robot's pose is in.** It knows nothing about the field's walls or alliances.
- **With a `pullIn` of 0.5**, a driver at 4 m/s along the rail is pulled in at 2 m/s at most. The robot slides on at a fixed angle, `atan(pullIn)`, and joins the line about `how far off ÷ pullIn` farther along.
- **With a `pullIn` set, the rail still pulls at up to 0.5 m/s when the stick is released**, and the driver's own push toward the line counts in full.
- **The `pullIn` share is a feel number.** It has not been reviewed with a driver on a robot.

## More

- [The rail law](/concepts/rail-law/): what each part does to the robot.
- [`RailLaw`](/reference/follow/raillaw/) and [`RailFollower`](/reference/follow/railfollower/): the classes that drive a rail.
