---
title: The piece assist
description: How the intake assist picks a game piece, nudges the driver toward it, and remembers pieces.
sidebar:
  order: 17
---

Picking up a game piece at speed means lining the intake up to within a few centimeters while driving at several meters per second.

The **piece assist** helps with that last correction. It keeps the driver's speed and turns their direction a little, so the intake arrives on the piece. Done well the driver does not notice it. Two classes are involved. [`PieceAssist`](/reference/follow/pieceassist/) chooses a piece and works out the nudge. [`PieceFilter`](/reference/follow/piecefilter/) remembers where the pieces are between camera photos. Robot code uses [`RailDrive`](/reference/wpilib/raildrive/) and [`PieceMap`](/reference/wpilib/piecemap/), and the guide is [Intake assist](/guides/intake-assist/).

:::caution
The piece assist is a driver-feel feature. Its numbers come from the simulator and the bench, and none has been reviewed with a driver on a robot. The values on this page are defaults, not recommendations.
:::

## Step 1: which piece

The assist only helps with a piece the driver is already heading for. It never picks a different piece for them.

A piece is a candidate when all of these are true:

- the driver's stick speed is at least 0.3 m/s,
- the piece is within 3 m of the intake,
- seen from the intake, the piece is within 30° of the direction the stick is pushing.

Among the candidates, the one most in line with the stick wins. Everything is measured from the intake's position on the field, not from the robot's center. A piece behind the intake is 90° or more off the stick, so it is never a candidate.

### Staying with a piece

Two pieces that are about equally in line would swap places as the best choice with every wobble of the camera. The robot would weave between them and take neither.

So the choice has memory. While the piece chosen last loop is still a fair candidate, another piece takes over only if it is at least 10° more in line with the stick. A piece counts as "the one chosen last loop" if it is within 25 cm of where that one was.

## Step 2: the nudge

The assist measures the piece from the intake in two directions: how far **ahead** it is along the way the driver is pushing, and how far to the **side**.

Think about what it takes to arrive on the piece. If the piece is 2 m ahead and 25 cm to the left, the robot has to move 25 cm left in the time it takes to cover 2 m forward. That is a sideways speed of one eighth of the forward speed. In general:

```text
slide = (side ÷ ahead) · driver's speed
```

That is exactly the slide that carries the intake onto the piece as it arrives, and it is the smallest change of direction that does.

The assist asks for a little more than that, to be lined up slightly early:

```text
slide = EAGERNESS · (side ÷ max(ahead, CLOSE_METERS)) · speed
slide is held between −ASSIST_RATIO · speed and +ASSIST_RATIO · speed
```

| Part | Value | Why |
| --- | --- | --- |
| `EAGERNESS` | 1.5 | At 1 the intake comes onto the piece only as it arrives. At 1.5, two thirds of the gap is gone by halfway. |
| `CLOSE_METERS` | 0.20 m | `side ÷ ahead` would grow without limit as `ahead` goes to zero. Closer than this the slide stops growing. |
| `ASSIST_RATIO` | 0.6 | The slide is never more than this share of the driver's speed. |

The slide points at right angles to the way the driver is pushing. It has no spin part. The driver keeps the heading.

### The speed is kept

The nudge is not simply added to the driver's velocity, because that would make the robot faster. `RailFollower.step(driver, nudge, dt)` adds the two and then scales the result back to the driver's own speed. Only the direction changes.

That makes the nudge an angle. The direction turns by `atan(slide ÷ speed)`. For the piece 25 cm to the side and 2 m ahead, that is `atan(1.5 × 0.125)`, about 11°. At the cap it is `atan(0.6)`, about 31°, and it can never be more.

The result then goes through the [output limiter](/concepts/output-limiter/), the same one teleop uses.

## Why it is an assist and never takes over

Everything in the nudge is a multiple of the driver's speed.

- Push the stick harder and the robot goes faster, with the same bend.
- Let go of the stick and the speed is zero, so the nudge is zero. Below 0.3 m/s no piece is chosen at all.
- Point the stick more than 30° away from the piece and the piece is no longer a candidate.

The driver always decides whether to go and how fast. The assist only bends the direction, by an angle that depends on where the piece is.

This is different from a [rail](/concepts/rail-law/), which takes one direction over completely. An earlier approach got the intake onto the piece's line as fast as possible. For a piece 25 cm off seen 2.2 m out, it swerved the robot's travel by 25°, where the nudge bends it by about 9°.

The two main ideas come from talks by other teams. Orbit, team 1690, pulls only toward the piece the driver is already heading for, sideways to their direction. OP Robotics, team 2056, scales the push by how fast the robot is going.

## Keeping pieces between photos

A camera gives a new photo many times a second, and not every photo is good. A piece drops out for a frame. A reflection shows up as a piece for one frame.

`PieceFilter` keeps one steady estimate per piece. It knows nothing about cameras: robot code turns each detection into a spot on the floor in the robot's frame and hands over the whole photo.

Like the [tag filter](/concepts/tag-filter/), it keeps its estimates in the wheel-odometry frame and places each photo with the odometry pose at capture. A piece stays put while the robot drives, and a vision correction to the field pose moves the robot and the pieces together.

### Angle noise, range against bearing

A camera measures angles. It finds a piece's distance from how far down in the image the piece is, and a camera mounted low looks at the floor at a shallow angle. A small error in that angle is a large error in distance.

For an angle error `e`, a piece at floor distance `range` from the lens, and the lens `drop` meters above the piece's center:

```text
error to the side  = e · range
error in range     = e · (range² + drop²) ÷ drop
```

With the default `e` of 0.3°, a lens 0.33 m above the piece and a piece 2 m away, the side error is about 1 cm and the range error about 6.5 cm. Both grow with distance, and the range error grows faster.

The filter uses these two numbers as the noise of each sighting, in a Kalman filter with two dimensions. A far sighting moves the estimate a lot to the side and only a little in range. A near sighting moves it a lot both ways.

### Which piece is which

A detection is matched to a piece the filter already has when it is on nearly the same line of sight and not wildly nearer or farther:

- to the side, within 15 cm, or within three times the combined noise if that is larger,
- along the line of sight, within 0.5 m.

The range allowance is fixed and generous on purpose, because range is often wrong the same way photo after photo. A nearer sighting then corrects the estimate.

Each piece is matched to at most one detection per photo. A detection that matches nothing starts a new piece.

### Seen once, forgotten after half a second

- A piece counts only once it has been seen twice. One stray detection is not a piece.
- A piece not seen for 0.5 s is forgotten.

On the bench, with detections dropping out, a false one appearing, or two pieces nearly tied, the filter and the stay-with-a-piece rule together took the worst miss from between 20 and 45 cm down to under 5 cm. The average miss was the same. That is a simulator result.

## Every constant

`PieceAssist`:

| Constant | Value | Meaning |
| --- | --- | --- |
| `MAX_ANGLE_RADIANS` | 30° | Pieces farther than this off the stick's direction are ignored. |
| `MAX_DISTANCE_METERS` | 3.0 m | Pieces farther than this from the intake are ignored. |
| `MIN_DRIVER_METERS_PER_SECOND` | 0.3 m/s | Below this stick speed there is no help. |
| `ASSIST_RATIO` | 0.6 | The slide is at most this share of the driver's speed. |
| `EAGERNESS` | 1.5 | How much more than "just in time" the slide is. |
| `CLOSE_METERS` | 0.20 m | Closer than this the slide stops growing. |
| `SAME_PIECE_METERS` | 0.25 m | A piece within this of last loop's choice is that piece again. |
| `SWITCH_RADIANS` | 10° | Another piece must be this much more in line to take over. |

`PieceFilter`:

| Constant | Value | Meaning |
| --- | --- | --- |
| `MEMORY_SECONDS` | 0.5 s | A piece not seen for this long is forgotten. |
| `SIGHTINGS_TO_COUNT` | 2 | A piece counts once seen this many times. |
| `SAME_PIECE_METERS` | 0.15 m | The sideways allowance for matching a detection to a piece. |
| `SAME_RANGE_METERS` | 0.5 m | The allowance along the line of sight. |
| `DRIFT_METERS` | 0.004 m | Uncertainty added to each piece before each photo. Private. |
| angle noise | 0.3° | The default camera angle noise, 1 sigma. Set in the constructor. |

`EAGERNESS` is the one number a team can pass in: `PieceAssist.nudge` and `RailDrive.towardPiece` both have a form that takes it.

## What it does not do

- It does not turn the robot. The intake has to be facing roughly the right way already.
- It does not drive to a piece on its own. With no stick there is no motion.
- It does not detect pieces or read a camera.
- It does not tell pieces of different kinds apart.
- It does not avoid obstacles or other robots.
- It has not been driven by a driver on a real robot.
