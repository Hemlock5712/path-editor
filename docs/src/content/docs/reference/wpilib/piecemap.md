---
title: PieceMap
description: Remembers where the game pieces on the floor are between camera photos.
sidebar:
  order: 8
---

`PieceMap` takes each camera photo's game-piece detections, works out which piece is which, and returns one steady position per piece.

You meet it when you feed game pieces to the intake assist in [`RailDrive`](/reference/wpilib/raildrive/). [Intake assist](/guides/intake-assist/) is the guide.

## Example

```java
// Robot constructor
PieceMap pieces = new PieceMap();

// For every new photo, including one that shows no pieces.
// piecesInRobot, cameraInRobot, the heights, captureTime and now come from your vision code.
pieces.add(piecesInRobot, cameraInRobot, lensHeight - pieceHeight,
    drive.getOdometryPoseAt(captureTime), now);

// Every loop, for the intake assist
List<Translation2d> onField =
    pieces.fieldPieces(drive.getOdometryPose(), drive.getPose(), now);
```

## Constructors

| Constructor | What it does |
| --- | --- |
| `PieceMap()` | For a camera whose angles are good to about 0.3°. |
| `PieceMap(double angleNoiseRadians)` | For a camera whose angle error you have measured. The value is one standard deviation, in radians. The source's advice is to measure the spread of a piece that is standing still. |

## Methods

| Method | Returns | What it does |
| --- | --- | --- |
| `reset()` | nothing | Forgets every piece at once. Most code never needs it. |
| `add(List<Translation2d> piecesInRobot, Translation2d cameraInRobot, double dropMeters, Pose2d odometryAtCapture, double timeSeconds)` | nothing | Folds one photo into the map. Call it for every new photo. |
| `fieldPieces(Pose2d odometryNow, Pose2d fieldNow, double timeSeconds)` | `List<Translation2d>` | The pieces that count, as positions on the field in meters, in the real field frame. Call it every loop the pieces are needed. The list is what `RailDrive.towardPiece` takes as `pieces`. |

### add arguments

| Argument | Meaning |
| --- | --- |
| `piecesInRobot` | Every piece the photo shows, as a spot on the floor in the robot's frame, in meters: x forward, y left. Pass an empty list for a photo that shows nothing. |
| `cameraInRobot` | Where the lens is on the robot, in meters: x forward, y left. |
| `dropMeters` | How far the lens is above the centers of the pieces, in meters. A detection is skipped if this is not more than zero. |
| `odometryAtCapture` | The robot's wheel-odometry pose when the photo was taken, not now. |
| `timeSeconds` | The time now, in seconds. |

### fieldPieces arguments

| Argument | Meaning |
| --- | --- |
| `odometryNow` | The robot's wheel-odometry pose now. |
| `fieldNow` | The robot's field pose now. |
| `timeSeconds` | The time now, in seconds, on the same clock as `add`. |

## What the map does with a detection

| Rule | Numbers |
| --- | --- |
| Matches it to a piece it already has | Same line of sight, within 0.5 m in range and 0.15 m to the side, or a little more to the side when the sighting is noisy. Anything else is a new piece. |
| Weighs it by how good it is | A far sighting moves the estimate a little, mostly sideways. A near one moves it more. |
| Waits for a second sighting | A piece counts once it has been seen twice. |
| Forgets | A piece not seen for 0.5 s is dropped. |
| Skips | A detection directly under the lens. |

## Good to know

- **It has no camera in it.** Your robot code reads each photo and turns every detection into a spot on the floor.
- **Pieces are kept in the wheel-odometry frame**, the pose from the wheels and gyro alone. When vision corrects the field pose, the robot and the pieces move together.
- **The result is in the frame of `fieldNow`.** Give `RailDrive.towardPiece` the same pose, so the pieces and the robot are in one frame.

## More

- [Piece assist](/concepts/piece-assist/): how sightings are weighed.
- [`PieceFilter`](/reference/follow/piecefilter/): the class this wraps with WPILib's types.
- [The drivetrain](/start/drivetrain/): `getPose`, `getOdometryPose` and `getOdometryPoseAt`.
