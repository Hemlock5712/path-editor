---
title: The output limiter
description: The one limit every velocity command passes through, for paths and for teleop.
sidebar:
  order: 8
---

The follower's laws say what velocity they want, not whether the robot can get there in one loop. The **output
limiter** returns the largest step toward the wanted velocity that the robot can take without slipping.

Every output of [`PathFollower`](/reference/follow/pathfollower/), the rail and the intake assist goes through
an [`OutputLimiter`](/reference/follow/outputlimiter/). The driver's sticks reach it through
[`DriveLimiter`](/reference/wpilib/drivelimiter/).

## What it does

1. It keeps the last commanded `vx`, `vy` and spin rate. It is reset to the measured velocity once, when a
   command starts.
2. Each loop it works out the acceleration needed to reach the wanted velocity.

   ```text
   a_x = (wanted_vx − vx) / dt
   a_y = (wanted_vy − vy) / dt
   α   = (wanted_ω  − ω)  / dt
   ```

3. **Spin.** It clamps `α` to the spin acceleration cap.
4. **Motors.** When the step speeds the robot up, it cuts the part along the direction of travel to what the
   drive motors can add at the fastest wheel's speed. `r` is the distance from the robot's center to a module.

   ```text
   wheel speed = |v| + r·|ω|
   ```

   The motor model is full battery voltage less the back-EMF, capped by the stator current limit.
5. **Grip.** Translation and spin share one budget. `c` is the **spin lever**, a length.

   ```text
   |a| + c·|α| ≤ budget          with c = I / (m·r)
   ```

   Over the budget, it multiplies `a_x`, `a_y` and `α` by the same factor, so the step keeps its direction.
6. It adds what is left, times `dt`, to the command.

## Use it in teleop

`new OutputLimiter(robot)` makes a limiter for driving by hand, from the same robot file as the paths.

```java
OutputLimiter limiter = new OutputLimiter(paths.robot());   // robot init
limiter.reset(measuredVelocity);                            // when the drive command starts
FieldSpeeds command = limiter.limit(fromTheSticks, dt);     // every loop
```

Every velocity given to one limiter must be in the same frame, the field's or the driver's.

## The numbers

The traction limit `A` is covered in [Traction](/concepts/traction/).

```text
A = tractionShare · min(μ·g, g·x/h)
```

| Name | Value | Unit | What it is |
| --- | --- | --- | --- |
| Spin acceleration cap | `maxAngularRadPerSecSquared`, or 720 | °/s² | From the robot file, or the default |
| `tractionShare` | 0.9 | | The plan keeps a 10 % margin |
| `A`, default robot | 12.8 | m/s² | `tractionMetersPerSecondSquared()` |
| `A / tractionShare`, default robot | about 14.2 | m/s² | The limit without the margin |
| Spin lever `c`, default robot | about 0.24 | m | From moment of inertia `I`, mass `m` and module radius `r` |
| Budget in the follower | `A / tractionShare` | m/s² | The plan stays under `A`, and the corrections spend the margin |
| Budget in teleop, the rail and the intake assist | `A` | m/s² | No plan underneath, so the margin is kept |
| Motor push | | | The robot file's motor, gear ratio, wheel radius, stator current limit and mass |

The limiter has no settings of its own. To change what it allows, change
[the robot file](/files/robot-file/).

In simulation, the fixture auto route strayed 9.3 cm with the follower's budget at `A`, and 2.4 cm at
`A / tractionShare`.

:::note
`acceleration()` reports the step the limiter just took. Teleop turns that into wheel forces. A path uses the
plan's acceleration instead. See [Feed-forward](/concepts/feed-forward/).
:::

## What it does not do

- **It does not cap speed.** A wanted velocity above top speed is passed on. The motor stage only stops the
  command speeding up further.
- **It does not apply the motor stage to braking.**
- **It does not detect slip.** On a floor with less grip than the robot file says, the wheels slide.
- **It does not know where each module is.** Sharing the push between wheels is the job of
  [`WheelForces`](/reference/follow/wheelforces/).

## More

- [The teleop limiter](/guides/teleop-limiter/): the robot code.
- [`OutputLimiter`](/reference/follow/outputlimiter/)
- [`DriveLimiter`](/reference/wpilib/drivelimiter/)
