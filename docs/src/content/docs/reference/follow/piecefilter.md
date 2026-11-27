---
title: PieceFilter
description: Keeps a steady estimate of each game piece on the floor between camera photos.
sidebar:
  order: 11
---

`frc.team5712.path.follow.PieceFilter` takes a camera's sightings of the game pieces on the floor, keeps one steady estimate per piece, and gives them back as field positions.

Robot code uses [`PieceMap`](/reference/wpilib/piecemap/), which wraps a `PieceFilter` with WPILib's `Translation2d` and `Pose2d`. Use `PieceFilter` directly only outside WPILib. The guide is [Intake assist](/guides/intake-assist/).

## Example

```java
// piecesFromRobot is a List<Vec2> and cameraFromRobot a Vec2, in meters in the robot's frame.
// The poses are Pose values in meters and radians.
PieceFilter pieces = new PieceFilter();                       // robot init

// For every photo, with the odometry pose from when it was taken
pieces.add(piecesFromRobot, cameraFromRobot, dropMeters, odometryPoseAtCapture, now);

// Every loop
List<Vec2> onField = pieces.fieldPieces(odometryPoseNow, fieldPoseNow, now);
```

## Constructors

| Constructor | What it does |
| --- | --- |
| `PieceFilter()` | A filter for a camera whose angles are good to about 0.3°, 1 sigma. |
| `PieceFilter(double angleNoiseRadians)` | A filter for a camera whose angles are good to `angleNoiseRadians`, 1 sigma. The value is not checked. |

## Methods

No method throws.

| Method | Returns | What it does |
| --- | --- | --- |
| `reset()` | `void` | Forgets every piece. |
| `add(List<Vec2> piecesFromRobot, Vec2 cameraFromRobot, double dropMeters, Pose odometryAtCapture, double timeSeconds)` | `void` | Folds one photo into the estimates. Call it for every photo, including one that shows no pieces, with an empty list. Each call first forgets pieces not seen for `MEMORY_SECONDS`. |
| `pieces(double timeSeconds)` | `List<Vec2>` | The pieces that count, in the wheel-odometry frame, meters. A new list each call. |
| `fieldPieces(Pose odometryNow, Pose fieldNow, double timeSeconds)` | `List<Vec2>` | The pieces that count, on the field, meters: each measured from the robot's wheel-odometry pose now and placed on its field pose. A new list each call. |

### add parameters

| Parameter | Unit | Meaning |
| --- | --- | --- |
| `piecesFromRobot` | m | Every piece the photo shows, on the floor in the robot's frame: x forward, y left. |
| `cameraFromRobot` | m | Where the lens is on the robot: x forward, y left. |
| `dropMeters` | m | How far the lens is above the pieces' centers. |
| `odometryAtCapture` | m, rad | The robot's wheel-odometry pose when the photo was taken. |
| `timeSeconds` | s | Now. |

### fieldPieces parameters

| Parameter | Meaning |
| --- | --- |
| `odometryNow` | The robot's wheel-odometry pose now. |
| `fieldNow` | The robot's field pose now. |
| `timeSeconds` | Now, on the same clock as `add`. |

## Constants

All four are `public static final`.

| Constant | Type | Value | Meaning |
| --- | --- | --- | --- |
| `MEMORY_SECONDS` | `double` | 0.5 s | A piece not seen for this long is forgotten. |
| `SIGHTINGS_TO_COUNT` | `int` | 2 | A piece counts once it has been seen this many times. |
| `SAME_PIECE_METERS` | `double` | 0.15 m | A detection farther than this to the side of every piece, and farther than the noise allows, is a new piece. |
| `SAME_RANGE_METERS` | `double` | 0.5 m | Or farther than this nearer or farther along the line of sight, where the camera is far less sure. |

## Good to know

- **A piece counts** once it has been seen at least `SIGHTINGS_TO_COUNT` times and within the last `MEMORY_SECONDS` before `timeSeconds`.
- **Within one photo, each piece is matched to at most one detection.** A detection that matches no piece starts a new one.
- **A detection is skipped without an error** when it is at the lens's own floor position, or when `dropMeters` is 0 or less.
- **The list is in the order the pieces were first seen**, not sorted by distance. A piece has no identity robot code can read: to stay with one, compare positions, as [`PieceAssist.choose`](/reference/follow/pieceassist/) does.

:::caution
`odometryAtCapture` and `odometryNow` must be the wheel-odometry pose: wheels and gyro only, never corrected by vision. Use the same clock for every `timeSeconds`.
:::

## More

- [The piece assist](/concepts/piece-assist/): how each sighting is weighed.
- [`TagFilter`](/reference/follow/tagfilter/): the same idea for one AprilTag.
- [`RailDrive`](/reference/wpilib/raildrive/): its `towardPiece` takes the pieces.
