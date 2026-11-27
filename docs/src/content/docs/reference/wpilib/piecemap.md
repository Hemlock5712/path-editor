---
title: PieceMap
description: Remembers where the game pieces on the floor are between camera photos.
sidebar:
  order: 8
---

`PieceMap` keeps track of the game pieces on the floor. A camera's detections drop out, jump around, and sometimes show a piece that is not there. `PieceMap` takes each photo's detections, works out which piece is which, and returns one steady position per piece.

Like [`TagGoal`](/reference/wpilib/taggoal/), it has no camera in it. Your robot code reads each photo and turns every detection into a spot on the floor. You meet `PieceMap` when you feed game pieces to the intake assist in [`RailDrive`](/reference/wpilib/raildrive/). [Intake assist](/guides/intake-assist/) is the guide. It wraps [`PieceFilter`](/reference/follow/piecefilter/) with WPILib's types.

## Example

```java
// Robot constructor
PieceMap pieces = new PieceMap();

// for every new photo, including one that shows no pieces
pieces.add(piecesInRobot, cameraInRobot, lensHeight - pieceHeight,
    drive.getOdometryPoseAt(captureTime), now);

// every loop, for the intake assist
List<Translation2d> onField =
    pieces.fieldPieces(drive.getOdometryPose(), drive.getPose(), now);
```

`getPose`, `getOdometryPose` and `getOdometryPoseAt` are methods of your own drivetrain class. See [The drivetrain](/start/drivetrain/). `piecesInRobot`, `cameraInRobot`, the two heights, `captureTime` and `now` come from your vision code.

## Constructors

| Constructor | Meaning |
| --- | --- |
| `PieceMap()` | For a camera whose angles are good to about 0.3°. |
| `PieceMap(double angleNoiseRadians)` | For a camera whose angle error you have measured. The value is one standard deviation, in radians. The source's advice is to measure the spread of a piece that is standing still. |

## Methods

| Method | Returns | When to call |
| --- | --- | --- |
| `reset()` | nothing | To forget every piece. |
| `add(List<Translation2d> piecesInRobot, Translation2d cameraInRobot, double dropMeters, Pose2d odometryAtCapture, double timeSeconds)` | nothing | For every new photo. |
| `fieldPieces(Pose2d odometryNow, Pose2d fieldNow, double timeSeconds)` | `List<Translation2d>` | Every loop the pieces are needed. |

### add

Folds one photo into the map.

| Argument | Meaning |
| --- | --- |
| `piecesInRobot` | Every piece the photo shows, as a spot on the floor in the robot's frame, in meters: x forward, y left. Pass an empty list for a photo that shows nothing. |
| `cameraInRobot` | Where the lens is on the robot, in meters: x forward, y left. |
| `dropMeters` | How far the lens is above the centers of the pieces, in meters. |
| `odometryAtCapture` | The robot's wheel-odometry pose when the photo was taken, not now. |
| `timeSeconds` | The time now, in seconds. |

A detection is skipped if `dropMeters` is not more than zero, or if the detection sits directly under the lens.

What the map does with each detection:

- **It matches it to a piece it already has.** A detection on the same line of sight as a known piece, within 0.5 m in range and within 0.15 m to the side, or a little more to the side when the sighting is noisy, is that piece seen again. Anything else is a new piece.
- **It weighs it by how good it is.** A camera measures angles. Looking down at the floor from a low height, a small angle error is a large error in range and a small error to the side, and both grow with distance. A far sighting moves the estimate a little, mostly sideways. A near one moves it more.
- **It waits for a second sighting.** A piece counts once it has been seen twice. One stray detection is not a piece.
- **It forgets.** A piece not seen for 0.5 s is dropped.

### fieldPieces

Returns the pieces that count, as positions on the field in meters, in the real field frame.

| Argument | Meaning |
| --- | --- |
| `odometryNow` | The robot's wheel-odometry pose now. |
| `fieldNow` | The robot's field pose now. |
| `timeSeconds` | The time now, in seconds, on the same clock as `add`. |

The pieces are kept in the wheel-odometry frame, the pose from the wheels and gyro alone. Each one is measured from the robot's odometry pose now and placed on its field pose now. When vision corrects the field pose, the robot and the pieces move together. The list is what `RailDrive.towardPiece` takes as `pieces`.

### reset

Forgets every piece at once. Pieces also drop out on their own after 0.5 s without a sighting, so most code never needs it.

:::note
The frame of the result is the frame of `fieldNow`. Give `RailDrive.towardPiece` the same pose, so the pieces and the robot are in one frame.
:::
