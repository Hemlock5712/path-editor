---
title: PieceAssist
description: Static methods that pick the game piece a driver is heading for and nudge toward it.
sidebar:
  order: 10
---

`frc.team5712.path.follow.PieceAssist` helps a driver line the intake up on a game piece without taking over. It picks the piece the driver is already heading for, and works out a small sideways velocity that turns their direction to point the intake at it. The driver's speed is kept. [The piece assist](/concepts/piece-assist/) explains the idea and the numbers.

The class has only static methods and constants. It cannot be constructed and it keeps no state.

## Where you meet it

Robot code uses `towardPiece` on [`RailDrive`](/reference/wpilib/raildrive/), which calls `choose` and `nudge`, remembers the piece chosen last loop, and sends the result through the limiter. The pieces usually come from a [`PieceMap`](/reference/wpilib/piecemap/). The guide is [Intake assist](/guides/intake-assist/).

:::caution
The intake assist is a driver-feel feature. Its constants were set on the bench's simulated drivetrain and have not been reviewed with a driver on a robot. They are defaults, not recommendations.
:::

## Example

```java
Optional<Vec2> piece = PieceAssist.choose(pieces, intake, fromTheSticks, lastPiece);
lastPiece = piece;

FieldSpeeds command = piece.isPresent()
    ? rails.step(fromTheSticks, PieceAssist.nudge(fromTheSticks, intake, piece.get()), dt)
    : rails.step(fromTheSticks, dt);
```

`pieces` is a `List<Vec2>` of piece positions and `intake` is a `Vec2`, both in meters in the same frame as the stick velocity. `rails` is a [`RailFollower`](/reference/follow/railfollower/). `Vec2` is in [Small types](/reference/core/small-types/).

## Constants

All are `public static final double`.

| Constant | Value | Description |
| --- | --- | --- |
| `MAX_ANGLE_RADIANS` | `Math.toRadians(30)` | Pieces more than this far off the stick's direction, seen from the intake, are ignored. |
| `MAX_DISTANCE_METERS` | 3.0 m | Pieces farther than this from the intake are ignored. |
| `MIN_DRIVER_METERS_PER_SECOND` | 0.3 m/s | Below this stick speed there is no help. |
| `ASSIST_RATIO` | 0.6 | The sideways pull is at most this share of the driver's speed. |
| `EAGERNESS` | 1.5 | How much more than "just in time" the slide is. At 1 the intake comes onto the piece only as it arrives. |
| `CLOSE_METERS` | 0.20 m | Closer than this the slide stops growing. |
| `SAME_PIECE_METERS` | 0.25 m | A piece within this of the one chosen last loop is that piece again. |
| `SWITCH_RADIANS` | `Math.toRadians(10)` | Another piece takes over only when it is this much more in line with the stick. |

## Methods

### choose

```java
static Optional<Vec2> choose(List<Vec2> pieces, Vec2 intake, FieldSpeeds driver)
```

The piece the driver is heading for, if any. A piece qualifies when it is within `MAX_DISTANCE_METERS` of the intake and within `MAX_ANGLE_RADIANS` of the direction the stick is pushing. The one most in line with the stick wins.

| Parameter | Description |
| --- | --- |
| `pieces` | The pieces' positions, meters. |
| `intake` | Where the intake is on the field, meters. |
| `driver` | The driver's stick velocity in field directions, m/s. Its spin part is not used. |

Returns empty when the stick speed is below `MIN_DRIVER_METERS_PER_SECOND` or no piece qualifies. A piece at the intake's own position is skipped. The value returned is one of the objects in `pieces`.

### choose, staying with a piece

```java
static Optional<Vec2> choose(List<Vec2> pieces, Vec2 intake, FieldSpeeds driver,
    Optional<Vec2> previous)
```

The same choice, staying with the piece chosen last loop. The piece in `pieces` nearest to `previous`, and within `SAME_PIECE_METERS` of it, is taken to be that piece. While it still qualifies, another piece takes over only if it is more than `SWITCH_RADIANS` more in line with the stick.

| Parameter | Description |
| --- | --- |
| `previous` | The piece chosen last loop, or empty. |

Returns the plain choice when `previous` is empty, when no piece qualifies, or when the old piece is gone, behind, or too far off the stick.

### nudge

```java
static FieldSpeeds nudge(FieldSpeeds driver, Vec2 intake, Vec2 piece)
```

The sideways velocity to add to the driver's, to bring the intake onto `piece` as it arrives. It uses `EAGERNESS`.

```text
slide = EAGERNESS · side ÷ max(ahead, CLOSE_METERS) · speed
slide is held between −ASSIST_RATIO · speed and +ASSIST_RATIO · speed
```

`ahead` is how far the piece is from the intake along the way the driver is pushing, and `side` is how far it is to the left of that, both in meters. `speed` is the driver's stick speed in m/s.

| Parameter | Description |
| --- | --- |
| `driver` | The driver's stick velocity in field directions, m/s. |
| `intake` | Where the intake is on the field, meters. |
| `piece` | The piece chosen by `choose`, meters. |

Returns a `FieldSpeeds` in m/s at right angles to the driver's push, with a spin of 0. Returns zero when the driver's speed is zero.

### nudge, with an eagerness

```java
static FieldSpeeds nudge(FieldSpeeds driver, Vec2 intake, Vec2 piece, double eagerness)
```

The same with another eagerness in place of `EAGERNESS`, for tuning. A value of 1 lines the intake up only on arrival. More lines it up sooner and turns the robot's travel further. The value is not checked.

## Using the nudge

The nudge is meant to turn the driver's velocity, not to speed the robot up. Pass it to `step(driver, nudge, dt)` on [`RailFollower`](/reference/follow/railfollower/), which adds it and then scales the result back to the driver's speed.

`nudge` does not check that the piece qualifies. Call `choose` first.

## Errors

No method throws for any finite input. `choose` with an empty list returns empty.
