---
title: Traction
description: The traction limit that every plan and every command is held to, and where it comes from.
sidebar:
  order: 5
---

A robot can only push on the floor so hard. Ask for more and one of two things happens: the wheels slip, or the robot starts to tip. Myelin works out one number for how hard the robot may push, the **traction limit**, and holds every plan to it. This page explains the formula, what shares the limit, and which numbers in robot.json change it.

The limit lives in [RobotConfig](/reference/core/robotconfig/), which is read from [robot.json](/files/robot-file/) at boot.

## The formula

```text
A = tractionShare * min(μ * g, g * x / h)
```

`A` is the traction limit in m/s². It is the most the robot accelerates, in any direction, all directions combined. The two terms inside `min` are the two ways to lose control:

- **Grip.** `μ * g` is the most acceleration the tread can hold before it slides. `μ` is the friction coefficient between the tread and the carpet, and `g` is gravity, 9.81 m/s².
- **Tipping.** `g * x / h` is the acceleration that would lift the wheels on one side. `h` is the height of the center of mass above the floor. `x` is the **tip distance**, how far the center is from the nearest tipping edge. A tall robot on a narrow base tips before it slides.

Whichever is smaller comes first, so that one sets the limit. `tractionShare` then keeps a margin. The default share is 0.9, which leaves 10 % unused.

The teleop limiter in the team's Precision-Drive robot code uses the same rule. Paths are therefore planned to the same limit the driver drives to.

## The defaults

The defaults describe the bench robot the library was developed against. They are in the source, and they are not a measurement of your robot.

| Quantity | robot.json key | Default |
| --- | --- | --- |
| Friction coefficient `μ` | `frictionCoefficient` | 1.5 |
| Center of mass height `h` | `centerOfMassHeightMeters` | 0.2 m |
| Tip distance `x` | `tipDistanceMeters` | `moduleRadiusMeters / √2`, which is 0.29 m |
| Share | `tractionShare` | 0.9 |

Working through the formula with those numbers:

```text
grip     = 1.5 * 9.81         = 14.7 m/s²
tipping  = 9.81 * 0.29 / 0.2  = 14.2 m/s²
A        = 0.9 * 14.2         = 12.8 m/s²
```

Tipping comes first with the defaults, by a small amount. Move the modules in to 0.254 m from the center on each axis and the limit drops to 11.2 m/s². `RobotConfig.tractionLimit()` reports which of the two is setting the limit.

Two special cases are worth knowing. A `centerOfMassHeightMeters` of 0 turns the tipping term off, and grip alone sets the limit. And `tipDistanceMeters` is one value for every direction: the smaller of the modules' x and y distances from the center. Leave it out for a square base.

:::caution
The default center of mass height is a guess. The source says so. Measure yours, because with the defaults it is the number that sets the limit.
:::

## Sharing the limit

The limit is not only for speeding up. Four things push on the floor, and they share one budget:

```text
√(a² + (v²κ)²) + c * |α| ≤ A
```

| Term | What it is |
| --- | --- |
| `a` | Speeding up or braking along the path, m/s² |
| `v²κ` | Cornering: the sideways acceleration at speed `v` on a bend of curvature `κ` |
| `c * \|α\|` | Spinning: the extra push each wheel needs for a spin acceleration `α` |

Speeding up and cornering are at right angles, so they combine like the sides of a right triangle. A robot using all of its grip to corner has none left to brake with. The spin term is added on top, for the worst wheel, where the spin push lines up with the rest.

`c` is a length, `I / (m * r)`: the moment of inertia over the mass times the module radius. For the defaults it is about 0.24 m. `RobotConfig.spinLeverMeters()` returns it.

The [speed plan](/concepts/speed-plan/) solves this inequality for the largest `a` at every point of a path. That is why a path slows for a corner before it reaches it, and why a path that turns the robot quickly while cornering is slower than one that does either alone.

## The motors

Grip is not the only thing that limits acceleration. The drive motors have to supply the push. [Motor](/reference/core/motor/) models a brushless motor as a straight line from stall torque at zero speed to zero torque at free speed. The default is the Kraken X60 with FOC: 12 V, 9.362 N·m stall torque, 476.1 A stall current, 5,785 rpm free speed.

At a given wheel speed the model works out the current the motor can draw at full voltage, caps it at the stator current limit, and converts that to acceleration:

```text
current = (12 V - back-EMF at this speed) / winding resistance
current = clamp(current, 0, statorCurrentLimitAmps)
accel   = motors * torque per amp * gear ratio / wheel radius * current / mass
```

Working these formulas through with the default robot gives three numbers:

- At low speed the stator limit of 120 A binds, and the motors can push for 20.9 m/s². That is above the traction limit, so grip is what limits a launch.
- Above a wheel speed of about 3.9 m/s the motors can no longer supply the full 12.8 m/s², and they become the limit.
- At 4.56 m/s the motors reach their free speed and can add nothing more.

Braking is different. Back-EMF helps a motor slow down, so only the stator limit applies, and the traction limit is what binds.

## Plans and commands

The planner uses `A` directly. The follower's output limiter allows a little more, `A / tractionShare`, which is the limit without its margin. For the defaults that is 14.2 m/s². The plan keeps the margin and the feedback has the rest to correct errors with. The [output limiter](/concepts/output-limiter/) page covers this.

The same split applies to spin. If `maxAngularRadPerSecSquared` is set in robot.json, the plan turns at `tractionShare` of it and the follower is held to the full value.

## On a slippery floor

The library has no slip detection. If the wheels slip, the follower does not notice. It only sees the pose drift. When the robot slips on its paths, change robot.json:

- Lower `tractionShare`. This always lowers the limit.
- Lower `frictionCoefficient` to the floor's real value. Check which term is binding first. With the default geometry tipping sets the limit until `μ` falls below `x / h`, which is 1.45, so a small reduction from 1.5 changes nothing.

Both changes apply to every path at the next boot, and to the teleop limiter if you use [DriveLimiter](/guides/teleop-limiter/).

On the team's simulation bench, a floor with a real `μ` of 0.91 under a plan made for 1.5 left wheel odometry 31 cm off at the end of a run. With robot.json told the floor's real `μ`, the error was 10.6 cm. Those are simulator figures. The library has not run on a real robot.

## What this piece does not do

- It does not measure the floor. `μ` is whatever robot.json says.
- It does not model each wheel's load. The limit is one number for the whole robot. Weight shifting between the wheels is handled separately, in the [wheel force feed-forward](/concepts/feed-forward/).
- It does not change during a match.

See [Tuning](/guides/tuning/) for the order to adjust things in on a real robot.
