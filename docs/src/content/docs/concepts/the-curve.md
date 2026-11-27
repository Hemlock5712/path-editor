---
title: The curve
description: How Myelin turns waypoints into a smooth curve and measures it in meters.
sidebar:
  order: 2
---

A path file holds a short list of waypoints. The robot needs a point for every centimeter, the direction of
travel there, and how sharply the path bends.

[QuinticSpline](/reference/core/quinticspline/) draws the curve and
[PathGeometry](/reference/core/pathgeometry/) measures it. `PathLibrary.loadAll()` runs both at boot.

## What it does

1. **One piece per pair of waypoints.** Each piece is a **quintic**, a polynomial of degree five in a parameter
   `u` that runs from 0 to 1. A path with five waypoints has four pieces.
2. **Pick the tangents.** The **tangent** is the direction of travel through a waypoint. The editor writes it as
   `tangentDegrees`. If it is left out, the library chooses one.

   | Waypoint | Tangent chosen |
   | --- | --- |
   | Interior | The slope of the parabola through it and its two neighbors, a Bessel tangent |
   | First or last | The neighbor's tangent mirrored across the straight line between the two |
   | Two waypoints, no tangents | A straight line |

3. **Scale each piece.** `tangentScale` sets how long the curve keeps to the tangent's direction. Above 1 holds
   the direction longer. Below 1 turns sooner.

   ```text
   a = tangentScale * chord * h / sin(h)
   ```

   `chord` is the straight-line distance between the two waypoints in meters. `h` is half the total angle the two
   end tangents make with the chord, capped at 90 degrees.
4. **Match the bend.** At each waypoint both pieces get the same position, direction and **curvature** `κ`.
   Curvature is how sharply the path bends, in 1/m, and positive turns left.

   ```text
   position   P
   velocity   V = a * d          d = the unit tangent
   bend       A = a² * κ * n     n = the unit normal, d turned 90 degrees left
   ```

   A robot at speed `v` needs a sideways acceleration of `v²κ`, so a jump in curvature would make the wheels
   slip.
5. **Choose the curvatures.** You do not set them. The library picks the values that minimize the squared third
   derivative summed over the pieces.
6. **Measure in meters.** `u` is not a distance. `PathGeometry` builds an arc length table that converts `u` to
   `s`, the distance along the path in meters from the first waypoint. Everything after the curve uses `s`.
7. **Find cusps.** A **cusp** is where the curve stops and doubles back. The speed plan stops the robot at each
   one, and the editor flags them.

## Finding the closest point

**Projection** finds where the robot is on the path and returns a [Projection](/reference/core/samples/).

1. A coarse scan samples the path and keeps the nearest sample.
2. Newton's method polishes it until the line from the robot meets the path at a right angle. If a step leaves
   the bracket, the code bisects instead.

   ```text
   (p(s) - q) · T(s) = 0
   ```

   `q` is the robot's position, `p(s)` the path point and `T(s)` the unit tangent.
3. The search normally covers only a window around the last known `s`, so a path that crosses itself cannot make
   progress jump. The whole path is searched at the start and after a hard push.
4. The result carries the **cross-track error**: the signed distance from the path, positive when the robot is to
   the left.

## The numbers

| Name | Value | Unit | What it is |
| --- | --- | --- | --- |
| `tangentScale` | 1 | | The default |
| Circle of radius 2 m | 0.5 | 1/m | An example curvature. A straight line is 0. |
| Circle check | 0.001 | | Waypoints on a circle give that circle's curvature to this tolerance |
| Sub-piece start size | about 0.5 | m | Of chord. Each is measured with 5-point Gauss-Legendre quadrature. |
| Sub-piece split | 0.01 | mm | A sub-piece whose cubic misses the midpoint by more is cut in half |
| Sub-pieces, 16 m example path | 97 | | In the library's tests |
| Worst lookup error | 0.023 | mm | On the test paths, against a 4,000-step integral |
| Lookup time | under 0.5 | microseconds | On a desktop PC. The roboRIO has not been measured. |
| Window scan | 5 | cm | Sample spacing for projection |
| Follower's window | 0.5 | m | Either side of the last `s` |
| Whole-path scan | 10 | cm | Sample spacing |
| Near-cusp | more than 120 degrees within 4 cm | | A hairpin counted with the cusps |
| Waypoint-relative place | `1.5` | | Halfway from waypoint 1 to waypoint 2. `PathGeometry.sAtWaypoint` converts it to meters. |

:::note
Distances are table distances. They can differ from the true arc length by up to about one part in 10,000. Every
part of the library uses the same `s`, so the plan is consistent with itself.
:::

## What it does not do

- **It does not know about the robot.** Speed, grip and motors come later, in
  [The speed plan](/concepts/speed-plan/).
- **It does not avoid obstacles.** That is the [pathfinder](/concepts/pathfinder/).
- **It does not set the heading.** The tangent is the direction of travel. See [Heading](/concepts/heading/).

## More

- [The speed plan](/concepts/speed-plan/)
- [QuinticSpline](/reference/core/quinticspline/)
- [PathGeometry](/reference/core/pathgeometry/)
