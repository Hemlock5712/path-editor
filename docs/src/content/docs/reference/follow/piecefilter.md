---
title: PieceFilter
description: Keeps a steady estimate of each game piece on the floor between camera photos.
sidebar:
  order: 11
---

`frc.team5712.path.follow.PieceFilter` works out where the game pieces on the floor are from a camera's sightings of them. It knows nothing about cameras. Robot code reads the camera, turns each detection into a spot on the floor in front of the robot, and hands over every photo. The filter keeps one steady estimate per piece and gives them back as field positions. [The piece assist](/concepts/piece-assist/#keeping-pieces-between-photos) explains how it weighs each sighting.

Compared with using the newest photo as it comes, the filter knows which piece is which from one photo to the next, weighs each sighting by how good it is, and does not count a detection seen only once.

## Where you meet it

Robot code uses [`PieceMap`](/reference/wpilib/piecemap/), which wraps a `PieceFilter` and takes WPILib's `Translation2d` and `Pose2d`. Its pieces go to `towardPiece` on [`RailDrive`](/reference/wpilib/raildrive/). The guide is [Intake assist](/guides/intake-assist/). Use `PieceFilter` directly only outside WPILib.

## Example

```java
PieceFilter pieces = new PieceFilter();                       // robot init

// for every photo, with the odometry pose from when it was taken
pieces.add(piecesFromRobot, cameraFromRobot, dropMeters, odometryPoseAtCapture, now);

// every loop
List<Vec2> onField = pieces.fieldPieces(odometryPoseNow, fieldPoseNow, now);
```

`piecesFromRobot` is a `List<Vec2>` and `cameraFromRobot` a `Vec2`, in meters in the robot's frame. The poses are [`Pose`](/reference/core/small-types/) values in meters and radians.

## Constants

| Constant | Type | Value | Description |
| --- | --- | --- | --- |
| `MEMORY_SECONDS` | `double` | 0.5 s | A piece not seen for this long is forgotten. |
| `SIGHTINGS_TO_COUNT` | `int` | 2 | A piece counts once it has been seen this many times. |
| `SAME_PIECE_METERS` | `double` | 0.15 m | A detection farther than this to the side of every piece, and farther than the noise allows, is a new piece. |
| `SAME_RANGE_METERS` | `double` | 0.5 m | Or farther than this nearer or farther along the line of sight, where the camera is far less sure. |

All four are `public static final`.

## Constructors

| Constructor | Description |
| --- | --- |
| `PieceFilter()` | A filter for a camera whose angles are good to about 0.3°, 1 sigma. |
| `PieceFilter(double angleNoiseRadians)` | A filter for a camera whose angles are good to `angleNoiseRadians`, 1 sigma. |

The angle noise is not checked.

## Methods

| Method | Returns | Description |
| --- | --- | --- |
| `reset()` | `void` | Forgets every piece. |
| `add(List<Vec2> piecesFromRobot, Vec2 cameraFromRobot, double dropMeters, Pose odometryAtCapture, double timeSeconds)` | `void` | Folds one photo into the estimates. |
| `pieces(double timeSeconds)` | `List<Vec2>` | The pieces that count, in the wheel-odometry frame, meters. |
| `fieldPieces(Pose odometryNow, Pose fieldNow, double timeSeconds)` | `List<Vec2>` | The pieces that count, on the field, meters. |

### add

| Parameter | Unit | Description |
| --- | --- | --- |
| `piecesFromRobot` | m | Every piece the photo shows, on the floor in the robot's frame: x forward, y left. |
| `cameraFromRobot` | m | Where the lens is on the robot: x forward, y left. |
| `dropMeters` | m | How far the lens is above the pieces' centers. |
| `odometryAtCapture` | m, rad | The robot's wheel-odometry pose when the photo was taken. |
| `timeSeconds` | s | Now. |

Call it for every photo, including one that shows no pieces, with an empty list. Each call first forgets pieces not seen for `MEMORY_SECONDS`.

Within one photo, each piece the filter holds is matched to at most one detection. A detection that matches no piece starts a new one, which does not count until it is seen again.

A detection is skipped without an error when it is at the lens's own floor position, or when `dropMeters` is 0 or less.

### pieces and fieldPieces

Both return only pieces that have been seen at least `SIGHTINGS_TO_COUNT` times and within the last `MEMORY_SECONDS` before `timeSeconds`. Each call returns a new list.

`fieldPieces` measures each piece from where the robot is now in wheel odometry, then places it on the robot's field pose. A vision correction to the field pose moves the robot and the pieces together.

| Parameter | Description |
| --- | --- |
| `odometryNow` | The robot's wheel-odometry pose now. |
| `fieldNow` | The robot's field pose now. |
| `timeSeconds` | Now, on the same clock as `add`. |

:::caution
`odometryAtCapture` and `odometryNow` must be the wheel-odometry pose: wheels and gyro only, never corrected by vision. Use the same clock for every `timeSeconds`.
:::

The list's order is the order in which the pieces were first seen. It is not sorted by distance. A piece does not keep an identity that robot code can read: to stay with one piece, compare positions, as [`PieceAssist.choose`](/reference/follow/pieceassist/) does.

## Errors

No method throws.

## Related pages

- [`TagFilter`](/reference/follow/tagfilter/), the same idea for one AprilTag
- [`PieceAssist`](/reference/follow/pieceassist/), which chooses among the pieces
