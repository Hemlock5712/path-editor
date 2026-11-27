---
title: Feed-forward
description: How the follower tells each wheel how hard to push, using the plan's acceleration.
sidebar:
  order: 9
---

A wheel's velocity loop only pushes harder once it sees an error. **Feed-forward** tells each wheel the force the
planned acceleration needs before any error appears. It is optional.

## What it does

1. The follower reports the acceleration the **plan** calls for, as `commandedAcceleration()`. It does not
   difference the velocity commands, because the corrections in them carry pose noise.

   | While | Along the direction of travel | Sideways | Spin |
   | --- | --- | --- | --- |
   | Following the path | The plan's speeding up or slowing down at the point read ahead. Once the stopping law's brake governs, the braking deceleration. | `v²κ` toward the inside of the turn | The change in the plan's turn rate from here to the point read ahead, per second |
   | Turning on the spot | 0 | 0 | The same |
   | Driving to the goal | The [stopping law's](/concepts/stopping-law/) slowing, `min(a, k·v)`, along the line to the goal | 0 | The heading law's slowing |

2. It clamps the spin part to the spin acceleration cap.
3. If `|a| + c·|α|` is over the [limiter's](/concepts/output-limiter/) budget, it scales all three parts down
   together. The value is zero when the path is not running.
4. [`WheelForces`](/reference/follow/wheelforces/) finds the weight on each wheel. It treats the frame as stiff,
   so the loads lie on a flat plane.

   ```text
   load_i = p0 + p1·x_i + p2·y_i
   ```

   `x_i` and `y_i` are where module `i` is. The loads add up to `m·g`, their pitching moment balances
   `m · h · a_forward`, and their rolling moment balances `m · h · a_left`. `h` is the height of the center of
   mass.
5. A load below zero counts as zero. The loads are divided by their total to give shares that add up to 1.
6. Each wheel gets its share of the driving push, mass times acceleration.
7. Each wheel also pushes at right angles to its arm to turn the robot, and wheels farther out push harder. This
   spin term is the turning effort asked for, less what the driving forces already give.
8. The result is two arrays in the field frame, forces along x and along y, in newtons, one entry per module in
   the order the modules were given.

The forces always add up to mass times acceleration, and their turning effort about the center is inertia times
spin acceleration. The library's tests check both on random robots.

In simulation, sending these forces next to the velocity took a quarter to a third off the wheel slip at the same
path times. A path planned at 9 m/s² with the feed-forward slipped no more than one planned at 6 m/s² without it.

## In teleop and on the rail

| Mode | Feed-forward |
| --- | --- |
| Plain teleop | The limiter's own step, `OutputLimiter.acceleration()` |
| Across the [rail](/concepts/rail-law/) | The law's own braking only |
| Along the rail | The limiter's step along the rail |

## The numbers

| Name | Value | Unit | What it is |
| --- | --- | --- | --- |
| `centerOfMassHeightMeters` | 0.2 | m | Height of the center of mass. With 0 the shares do not change with acceleration. |

:::caution
The library marks 0.2 m as a guess until it is measured. Measure the robot's and put it in
[the robot file](/files/robot-file/).
:::

## What it does not do

- **No friction.** The force to keep the drivetrain turning, and the few percent more for the motors' rotors, are
  left to the drive motors' velocity loops and their static gain, kS.
- **No corrections.** The push back onto the path is left to the wheels' velocity loops.
- **No slip detection.**
- **No delay in the lean.** A real robot takes a moment to pitch back.
- **No off-center guess.** Module positions must be measured from the center of mass. With all modules in one
  line, the shares are equal.

:::note
The library has not run on a real robot. The slip result is from the simulator.
:::

## More

- [Wheel forces](/guides/wheel-forces/): the robot-code side.
- [`PathFollower`](/reference/follow/pathfollower/): `commandedAcceleration()`.
- [`WheelForces`](/reference/follow/wheelforces/) and [`WheelForceDrive`](/reference/wpilib/wheelforcedrive/).
