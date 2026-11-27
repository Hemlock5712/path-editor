---
title: Heading
description: How the planned heading is built from heading modes, rotation targets and face-point zones.
sidebar:
  order: 3
---

A swerve robot can face one way while it travels another. The **heading** is the direction the robot faces. The direction of travel is the path's tangent. They are separate, and a path has to say what the heading does along its whole length. A robot that must point its intake at a game piece while sliding sideways past an obstacle depends on that.

[HeadingProfile](/reference/core/headingprofile/) holds the answer. It gives the planned heading at every distance `s` along the path, together with how fast the heading changes per meter. It is built at boot as part of `Path.build`. You shape it in the editor; robot code does not call it directly.

## The inputs

Four things in a [path file](/files/path-file/) feed the heading:

| Input | What it says |
| --- | --- |
| `headingMode` | What the heading does when nothing else is set: `hold` or `faceTravel` |
| `headingDegrees` on a waypoint | The robot's heading at that waypoint |
| `rotationTargets` | A heading to reach at a place on the path |
| `facePointZones` | A stretch of path where the robot faces a point on the field |

A heading on a waypoint is shorthand. When the path is built, each one becomes a rotation target at that waypoint, placed ahead of the targets you wrote yourself. From then on the profile only sees rotation targets.

## With no targets

If a path has no rotation targets and no waypoint headings, the [HeadingMode](/reference/core/headingmode/) decides:

- **Hold** is the default. The robot keeps the direction the path starts out in for the whole path. If the path leaves its first waypoint pointing at 30 degrees, the robot faces 30 degrees from start to end.
- **Face travel** turns the robot to face along the tangent everywhere. The heading equals the tangent angle, so its rate of change per meter is the curvature `κ`.

The mode only matters when there are no targets. Once a path has one target or more, the targets set the heading in both modes.

## Rotation targets

A [RotationTarget](/reference/core/rotationtarget/) is a heading and a place. The place is either waypoint-relative, where `1.5` means halfway along the arc from waypoint 1 to waypoint 2, or a distance in meters from the start. A distance past either end of the path is moved to that end.

The profile sorts the targets by distance and then walks along the path:

1. **Before the first target** the robot holds the first target's heading.
2. **Between two targets** it turns from one heading to the next, the short way around.
3. **After the last target** it holds the last target's heading.

So a path with a single target holds that heading from start to end. To make the robot start at one heading and finish at another, put a target at the first waypoint and another at the last.

### The turn between two targets

The turn is spread over the whole stretch between the two targets. The basic shape is the minimum-jerk blend:

```text
blend(τ) = 10τ³ - 15τ⁴ + 6τ⁵        τ runs from 0 to 1 along the stretch
```

It starts flat, turns fastest in the middle, and ends flat. Both its slope and its bend are zero at each end, so the spin starts and stops without a jolt.

### What happens at a target

Stopping the spin at every target would waste time when the robot is turning the same way on both sides. The profile handles each target by what its neighbors do:

- At the **first and last target** the spin stops. The heading is being held on the other side, so there is nothing to carry on into.
- Where the turn **reverses**, or where one side does not turn at all, the spin stops at the target.
- Where the turn **carries on the same way**, the robot passes the target still turning.

The rate it passes at, in radians per meter, is the harmonic mean of the average rates on the two sides:

```text
rate = 2 * δa * δb / (δa + δb)

δa = turn before the target / length of that stretch
δb = turn after the target  / length of that stretch
```

The harmonic mean is never more than twice the smaller of the two rates. That bound is what guarantees each stretch turns one way only, so the heading never swings past a target and comes back.

The second derivative of the heading is zero at every target. The spin rate and the spin acceleration are therefore continuous along the whole path, which the [speed plan](/concepts/speed-plan/) relies on.

:::caution
Two targets at the same place make the heading jump there. A rotation target placed on a waypoint that also has `headingDegrees` is the usual way this happens. The editor does not create one, but a hand-edited file can.
:::

## Face-point zones

A [FacePointZone](/reference/core/facepointzone/) covers a stretch of path, from one waypoint-relative place to another. Inside it the robot faces a fixed point on the field, such as a goal, instead of following the rotation targets.

Switching straight from the targets' heading to the facing direction would be a jump, so the zone eases in and out. Over the first `blendMeters` of the zone a weight `w` rises from 0 to 1 along the same minimum-jerk blend. Over the last `blendMeters` it falls back to 0. In between it is 1.

```text
heading = base + w * (direction to the point - base)
```

`base` is the heading from the layer below, and the difference is taken the short way around. The default `blendMeters` is 0.5 m. A blend longer than half the zone is cut to half the zone, so the ease in and the ease out never overlap.

If a path has several zones, they are applied in the order they appear in the file, each on top of the result of the one before.

## Heading is tied to distance

The profile gives the heading as a function of `s`, not of time. The speed plan turns that into motion. With the robot at speed `v` and acceleration `a` along the path:

```text
spin rate          ω = v * dθ/ds
spin acceleration  α = a * dθ/ds + v² * d²θ/ds²
```

A consequence follows. A large turn packed into a short distance means a large `dθ/ds`, and the only way to keep the spin within the robot's limits is to drive slower. The plan does exactly that. In the library's 16 m example path, a face-point zone with the default 0.5 m blend turns the robot by more than 100 degrees, and the plan slows to about 0.9 m/s for it. If a path crawls through a zone, lengthen the blend or start the zone earlier.

A path whose two waypoints are at the same place is a turn on the spot. See [The speed plan](/concepts/speed-plan/) for how that is planned.

## What this piece does not do

- It does not correct anything. It is a plan. Holding the real robot to the planned heading is the job of [the follower](/concepts/the-follower/).
- It does not choose headings for you. With no targets it holds or faces travel, and nothing more.
- It does not check that the robot fits. A heading that swings a long intake into a wall is caught by the editor's [footprint](/concepts/footprints/) checks, not here.

The sample type it returns is described on [Sample records](/reference/core/samples/).
