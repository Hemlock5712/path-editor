---
title: PieceAssist
description: Static methods that pick the game piece a driver is heading for and nudge toward it.
sidebar:
  order: 10
---

`frc.team5712.path.follow.PieceAssist` picks the game piece a driver is already heading for, and works out a small sideways velocity that turns their direction to point the intake at it.

Robot code uses `towardPiece` on [`RailDrive`](/reference/wpilib/raildrive/), which calls `choose` and `nudge`, remembers the piece chosen last loop, and sends the result through the limiter. The guide is [Intake assist](/guides/intake-assist/).

## Example

```java
// pieces is a List<Vec2> and intake a Vec2, in meters, in the same frame as the stick velocity.
// rails is a RailFollower.
Optional<Vec2> piece = PieceAssist.choose(pieces, intake, fromTheSticks, lastPiece);
lastPiece = piece;   // stay with this piece next loop

FieldSpeeds command = piece.isPresent()
    ? rails.step(fromTheSticks, PieceAssist.nudge(fromTheSticks, intake, piece.get()), dt)
    : rails.step(fromTheSticks, dt);   // no piece: plain teleop
```

## Methods

All are static. The class cannot be constructed and it keeps no state. No method throws for any finite input.

| Method | Returns | What it does |
| --- | --- | --- |
| `choose(List<Vec2> pieces, Vec2 intake, FieldSpeeds driver)` | `Optional<Vec2>` | The piece the driver is heading for: within `MAX_DISTANCE_METERS` of the intake and `MAX_ANGLE_RADIANS` of the stick's direction, the one most in line winning. Empty when the stick speed is below `MIN_DRIVER_METERS_PER_SECOND`, no piece qualifies, or the list is empty. A piece at the intake's own position is skipped. The value is one of the objects in `pieces`. |
| `choose(List<Vec2> pieces, Vec2 intake, FieldSpeeds driver, Optional<Vec2> previous)` | `Optional<Vec2>` | The same, staying with the piece chosen last loop: the piece nearest to `previous` and within `SAME_PIECE_METERS` of it. Another piece takes over only if it is more than `SWITCH_RADIANS` more in line with the stick. The plain choice when `previous` is empty, no piece qualifies, or the old piece is gone, behind, or too far off the stick. |
| `nudge(FieldSpeeds driver, Vec2 intake, Vec2 piece)` | `FieldSpeeds` | The sideways velocity to add to the driver's, to bring the intake onto `piece` as it arrives, using `EAGERNESS`. In m/s, at right angles to the driver's push, with a spin of 0. Zero when the driver's speed is zero. |
| `nudge(FieldSpeeds driver, Vec2 intake, Vec2 piece, double eagerness)` | `FieldSpeeds` | The same with another eagerness, for tuning. 1 lines the intake up only on arrival. More lines it up sooner and turns the robot's travel further. The value is not checked. |

### Parameters

| Parameter | Meaning |
| --- | --- |
| `pieces` | The pieces' positions, meters. |
| `intake` | Where the intake is on the field, meters. |
| `driver` | The driver's stick velocity in field directions, m/s. `choose` does not use its spin part. |
| `previous` | The piece chosen last loop, or empty. |
| `piece` | The piece chosen by `choose`, meters. |

## Constants

All are `public static final double`.

| Constant | Value | Meaning |
| --- | --- | --- |
| `MAX_ANGLE_RADIANS` | `Math.toRadians(30)` | Pieces more than this far off the stick's direction, seen from the intake, are ignored. |
| `MAX_DISTANCE_METERS` | 3.0 m | Pieces farther than this from the intake are ignored. |
| `MIN_DRIVER_METERS_PER_SECOND` | 0.3 m/s | Below this stick speed there is no help. |
| `ASSIST_RATIO` | 0.6 | The sideways pull is at most this share of the driver's speed. |
| `EAGERNESS` | 1.5 | How much more than "just in time" the slide is. At 1 the intake comes onto the piece only as it arrives. |
| `CLOSE_METERS` | 0.20 m | Closer than this the slide stops growing. |
| `SAME_PIECE_METERS` | 0.25 m | A piece within this of the one chosen last loop is that piece again. |
| `SWITCH_RADIANS` | `Math.toRadians(10)` | Another piece takes over only when it is this much more in line with the stick. |

## The nudge

`ahead` is how far the piece is from the intake along the driver's push and `side` how far to the left of that, in meters. `speed` is the stick speed in m/s.

```text
slide = EAGERNESS · side ÷ max(ahead, CLOSE_METERS) · speed
slide is held between −ASSIST_RATIO · speed and +ASSIST_RATIO · speed
```

## Good to know

- **The nudge turns the driver's velocity. It does not speed the robot up.** Pass it to `step(driver, nudge, dt)` on [`RailFollower`](/reference/follow/railfollower/), which adds it and scales the result back to the driver's speed.
- **`nudge` does not check that the piece qualifies.** Call `choose` first.

:::caution
The intake assist is a driver-feel feature. Its constants were set on the bench's simulated drivetrain and have not been reviewed with a driver on a robot. They are defaults, not recommendations.
:::

## More

- [The piece assist](/concepts/piece-assist/): the idea and the numbers.
- [`PieceMap`](/reference/wpilib/piecemap/): where the pieces usually come from.
- [Small types](/reference/core/small-types/): `Vec2` and `FieldSpeeds`.
