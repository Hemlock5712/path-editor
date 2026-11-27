---
title: QuinticSpline
description: The smooth curve through the waypoints, one quintic piece between each pair.
sidebar:
  order: 11
---

`QuinticSpline` draws the smooth curve through a list of waypoints. It fits one quintic piece between each pair of neighboring waypoints. The pieces meet with the same position, the same direction and the same curvature, so the robot never has to jerk its wheels at a waypoint. [The curve](/concepts/the-curve/) explains the math.

Package: `frc.team5712.path.core`.

## Where you meet it

You can treat this class as a black box. `Path.build` makes the spline through [PathGeometry](/reference/core/pathgeometry/), and nearly all code reads the curve by distance through `PathGeometry` or `Path.sample`. The spline itself is parameterized by a number `u` on each piece, not by distance. Use it directly only if you need the raw pieces.

## Example

```java
QuinticSpline spline = QuinticSpline.through(List.of(
    Waypoint.at(6.0, 2.0),
    Waypoint.at(3.0, 1.0),
    Waypoint.at(0.5, 1.0)));

int pieces = spline.segmentCount();                 // 2
double curvatureAtMiddle = spline.knotCurvature(1); // 1/m

double[] out = new double[8];
spline.evaluate(0, 0.5, out);                       // halfway in u along the first piece
double x = out[0];
double y = out[1];
```

## Constants

| Constant | Value | Meaning |
| --- | --- | --- |
| `MIN_CHORD_METERS` | `1e-6` | The shortest allowed straight-line distance between neighboring waypoints, in meters |

## Building

Both methods are static.

| Method | Description |
| --- | --- |
| `QuinticSpline through(List<Waypoint> waypoints)` | Builds the spline through these waypoints. |
| `QuinticSpline through(List<Waypoint> waypoints, double startCurvature)` | The same, with the curvature at the first waypoint fixed, in 1/m. `NaN` means it is chosen like the rest. A rebuilt tail uses this to join an old path with matching position, tangent and curvature. |

Both throw `IllegalArgumentException` when:

- there are fewer than two waypoints, with the message `a spline needs at least two waypoints`,
- two neighboring waypoints are closer than `MIN_CHORD_METERS`, with a message that names the two waypoint indexes.

The spline uses each waypoint's position, `tangentRadians` and `tangentScale`. It ignores `headingRadians`.

## Methods

Pieces are numbered from 0. Piece `k` runs from waypoint `k` to waypoint `k + 1`, and its parameter `u` runs from 0 to 1. Waypoints are also numbered from 0.

| Method | Returns |
| --- | --- |
| `int segmentCount()` | The number of pieces, which is the number of waypoints minus one. |
| `double chordMeters(int k)` | The straight-line distance between waypoints `k` and `k + 1`, in meters. |
| `double knotTangentRadians(int i)` | The direction of travel at waypoint `i`, in radians, whether it was given or chosen. |
| `double knotCurvature(int i)` | The curvature at waypoint `i`, in 1/m. Positive is turning left. |
| `void evaluate(int k, double u, double[] out)` | Evaluates piece `k` at `u` and writes eight values into `out[0]` to `out[7]`. |
| `double speed(int k, double u)` | The length of the first derivative on piece `k` at `u`: meters of arc per unit of `u`. |

### What evaluate writes

`out` must have at least eight elements. The derivatives are with respect to `u`, not distance and not time.

| Index | Value |
| --- | --- |
| `out[0]`, `out[1]` | x and y, in meters |
| `out[2]`, `out[3]` | The first derivatives of x and y |
| `out[4]`, `out[5]` | The second derivatives of x and y |
| `out[6]`, `out[7]` | The third derivatives of x and y |

The methods do not check `k`, `i` or the length of `out`. An index out of range throws `ArrayIndexOutOfBoundsException`. A `u` outside 0 to 1 is not rejected and extends the piece's polynomial past its end.

## How tangents are chosen

| Waypoint | Tangent when `tangentRadians` is `NaN` |
| --- | --- |
| Interior | The slope of the parabola through the waypoint and its two neighbors, weighted by chord length |
| First or last | The neighbor's tangent mirrored across the chord between them |
| Both ends of a two-waypoint path | Straight along the chord |

If an interior waypoint's two chords point in exactly opposite directions, the path doubles straight back and the tangent follows the next chord.
