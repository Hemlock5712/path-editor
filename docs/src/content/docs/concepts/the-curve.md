---
title: The curve
description: How Myelin turns waypoints into a smooth curve and measures it in meters.
sidebar:
  order: 2
---

A path file holds a short list of waypoints. The robot needs far more than that: a point for every centimeter, the direction of travel there, and how sharply the path bends. This page explains how the library fills in the gaps. Two classes do the work. [QuinticSpline](/reference/core/quinticspline/) draws the curve, and [PathGeometry](/reference/core/pathgeometry/) measures it.

You do not call either class in normal robot code. `PathLibrary.loadAll()` builds every path at boot, and this page describes what happens inside that call.

## Waypoints and pieces

A **waypoint** is a point the path passes through. Between each pair of neighboring waypoints the library fits one curved **piece**. A path with five waypoints has four pieces.

Each piece is a **quintic**, a polynomial of degree five in a local parameter `u` that runs from 0 at the start of the piece to 1 at its end. A quintic has six free numbers for x and six for y. That is exactly enough to pin three things at each end of the piece: the position, the direction of travel, and the bend.

## Tangents

The **tangent** at a waypoint is the direction of travel through it. You can set it in the editor, which writes `tangentDegrees` to the file. If you leave it out, the library chooses one:

- At an interior waypoint it takes the slope of the parabola through that waypoint and its two neighbors. This is called a Bessel tangent.
- At the first or last waypoint it mirrors the neighbor's tangent across the straight line between the two, which makes the end piece symmetric.
- A path with only two waypoints and no tangents is a straight line.

A tangent is only a direction. How long the curve keeps to that direction is set by `tangentScale`. The library estimates each piece's length as if it were a circular arc:

```text
a = tangentScale * chord * h / sin(h)
```

`chord` is the straight-line distance between the two waypoints in meters, and `h` is half the total angle that the two end tangents make with the chord, capped at 90 degrees. A `tangentScale` of 1 is the default. A value above 1 holds the direction longer before turning. A value below 1 turns sooner. Because each piece is scaled by its own length, a short piece next to a long one does not form a loop.

The tangent is not the robot's heading. On a swerve drive the robot can face one way and travel another. See [Heading](/concepts/heading/).

## Why curvature is continuous

**Curvature**, written `κ`, is how sharply the path bends, in units of 1/m. A circle of radius 2 m has a curvature of 0.5 1/m. A straight line has a curvature of 0. Positive curvature turns left.

Curvature matters because of what it asks of the wheels. A robot driving at speed `v` around a bend needs a sideways acceleration of `v²κ`. If the curvature jumped at a waypoint, the sideways push would have to jump with it, and so would the angle of every swerve module. Real wheels cannot do that, so the robot would slip or leave the path.

The library avoids this by construction. At each waypoint it picks one curvature `κ` and gives both pieces that meet there the same three end conditions:

```text
position   P
velocity   V = a * d          d = the unit tangent
bend       A = a² * κ * n     n = the unit normal, d turned 90 degrees left
```

The curvature of a curve is `(V × A) / |V|³`. With the values above that is `(a * a² * κ) / a³ = κ`, whatever `a` is. Both pieces meet with the same position, the same direction and the same curvature. Mathematicians call this G2 continuity.

The curvature at each waypoint is not something you set. The library chooses the set of values that makes the whole curve as smooth as possible, by minimizing the squared third derivative summed over the pieces. Each piece only involves its own two waypoints, so this is one small linear solve. A check on the result: waypoints placed on a circle give that circle's constant curvature, to a tolerance of 0.001.

## Distance along the path

The parameter `u` is not a distance. A step of 0.1 in `u` might cover 8 cm on one piece and 30 cm on another. Everything after the curve works in **distance along the path**, written `s` and measured in meters from the first waypoint. The speed plan, the heading, the events and the follower all use `s`.

`PathGeometry` builds an **arc length table** to convert between the two:

1. Each piece is cut into sub-pieces, starting at about 0.5 m of chord each.
2. The length of each sub-piece is measured with 5-point Gauss-Legendre quadrature, a standard way to integrate a smooth function from five samples.
3. Inside a sub-piece, `u` as a function of `s` is approximated by a cubic.
4. Any sub-piece where that cubic puts the midpoint more than 0.01 mm from where it belongs is cut in half, and the check repeats.

A lookup by `s` is then a binary search for the sub-piece, one cubic to get `u`, and one evaluation of the spline. The point, tangent and curvature that come back are the exact spline values at that `u`.

The 16 m example path in the library's tests needs 97 sub-pieces. Against an independent 4,000-step integral, the worst lookup error on the test paths is 0.023 mm. On a desktop PC one lookup takes under half a microsecond. The roboRIO has not been measured.

Places on a path are often given relative to waypoints instead of in meters. A value of `1.5` means halfway along the arc from waypoint 1 to waypoint 2, and `PathGeometry.sAtWaypoint` converts that to meters. A waypoint-relative place keeps its meaning when waypoints are moved.

## Finding the closest point

While the robot drives, the follower needs to know where it is on the path. **Projection** answers that: given the robot's position, it finds the closest point on the curve and returns a [Projection](/reference/core/samples/).

It works in two steps. A coarse scan samples the path every 5 cm and keeps the nearest sample. Then Newton's method polishes that sample until the line from the robot to the path meets the path at a right angle:

```text
(p(s) - q) · T(s) = 0
```

Here `q` is the robot's position, `p(s)` the path point and `T(s)` the unit tangent. If a Newton step leaves the bracket around the best sample, the code bisects instead, so it cannot run away.

The search normally covers only a window around the last known `s`. The follower uses 0.5 m either side. A path that crosses or passes near itself therefore cannot make the robot's progress jump to the other branch. A search of the whole path, scanned every 10 cm, is used at the start and after the robot is pushed far off.

The result also carries the **cross-track error**: the signed distance from the path, positive when the robot is to the left of it.

## Cusps

A curve can stop and double back on itself. That point is a **cusp**, and the curvature there is infinite. `PathGeometry` finds cusps once, at build time. It also counts near-cusps, hairpins where the direction of travel turns by more than 120 degrees within 4 cm of path. The speed plan brings the robot to a stop at each one, and the editor flags them, because a cusp is almost never drawn on purpose.

## What this piece does not do

- It does not know about the robot. Speed, grip and motors come later, in [The speed plan](/concepts/speed-plan/).
- It does not avoid obstacles. Routing around the field is the job of the [pathfinder](/concepts/pathfinder/).

:::note
Distances are table distances. The cubic inside each sub-piece can stretch `s` by up to about one part in 10,000 against the true arc length. Every part of the library uses the same `s`, so the plan is consistent with itself.
:::
