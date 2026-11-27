---
title: Heading
description: How the planned heading is built from heading modes, rotation targets and face-point zones.
sidebar:
  order: 3
---

A swerve robot can face one way while it travels another. The **heading** is the direction the robot faces, and
[HeadingProfile](/reference/core/headingprofile/) gives the planned heading at every distance `s` along the path.

It is built at boot as part of `Path.build`. You shape it in the editor.

## The inputs

| Input in the [path file](/files/path-file/) | What it says |
| --- | --- |
| `headingMode` | What the heading does when nothing else is set: `hold` or `faceTravel` |
| `headingDegrees` on a waypoint | The robot's heading at that waypoint |
| `rotationTargets` | A heading to reach at a place on the path |
| `facePointZones` | A stretch of path where the robot faces a point on the field |

## What it does

1. Each waypoint `headingDegrees` becomes a [RotationTarget](/reference/core/rotationtarget/) at that waypoint.
2. With no targets, the [HeadingMode](/reference/core/headingmode/) decides. **Hold**, the default, keeps the
   direction the path starts out in. **Face travel** makes the heading equal the tangent angle, so its rate of
   change per meter is the curvature `κ`.
3. With one target or more, the targets set the heading in both modes. They are sorted by distance. A distance
   past either end of the path is moved to that end.
4. Before the first target the robot holds the first target's heading. After the last it holds the last.
5. Between two targets it turns the short way around, spread over the whole stretch, along the minimum-jerk
   blend.

   ```text
   blend(τ) = 10τ³ - 15τ⁴ + 6τ⁵        τ runs from 0 to 1 along the stretch
   ```

6. At the first and last target, and where the turn reverses or one side does not turn, the spin stops at the
   target.
7. Where the turn carries on the same way, the robot passes the target still turning. The rate, in radians per
   meter, is the harmonic mean of the average rates on the two sides.

   ```text
   rate = 2 * δa * δb / (δa + δb)

   δa = turn before the target / length of that stretch
   δb = turn after the target  / length of that stretch
   ```

8. Inside a [FacePointZone](/reference/core/facepointzone/) the robot faces a fixed point on the field. A weight
   `w` rises from 0 to 1 over the first `blendMeters`, along the same blend, and falls back over the last.

   ```text
   heading = base + w * (direction to the point - base)
   ```

   `base` is the heading from the layer below. Several zones are applied in file order, each on top of the one
   before.
9. The [speed plan](/concepts/speed-plan/) turns heading per meter into spin. `v` is the speed and `a` the
   acceleration along the path.

   ```text
   spin rate          ω = v * dθ/ds
   spin acceleration  α = a * dθ/ds + v² * d²θ/ds²
   ```

A big turn in a short distance makes the plan drive slower. If a path crawls through a zone, lengthen the blend
or start the zone earlier.

:::caution
Two targets at the same place make the heading jump there. A hand-edited file can do this by putting a rotation
target on a waypoint that also has `headingDegrees`. The editor does not.
:::

## The numbers

| Name | Value | Unit | What it is |
| --- | --- | --- | --- |
| `headingMode` | `hold` | | The default mode |
| `blendMeters` | 0.5 | m | Ease in and ease out of a face-point zone. Cut to half the zone if longer. |
| Waypoint-relative place | `1.5` | | Halfway along the arc from waypoint 1 to waypoint 2 |
| Harmonic mean bound | 2 | times the smaller rate | Keeps each stretch turning one way only |
| Second derivative at a target | 0 | | Keeps spin rate and spin acceleration continuous |
| Example zone | about 0.9 | m/s | The planned speed in the library's 16 m example path, where a default zone turns the robot more than 100 degrees |

## What it does not do

- **It does not correct anything.** Holding the real robot to the planned heading is the job of
  [the follower](/concepts/the-follower/).
- **It does not choose headings for you.** With no targets it holds or faces travel, and nothing more.
- **It does not check that the robot fits.** That is the editor's [footprint](/concepts/footprints/) checks.

## More

- [The speed plan](/concepts/speed-plan/): how spin limits and turns on the spot are planned.
- [HeadingProfile](/reference/core/headingprofile/)
- [Sample records](/reference/core/samples/): the sample type it returns.
