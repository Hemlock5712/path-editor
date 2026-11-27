---
title: Feed-forward
description: How the follower tells each wheel how hard to push, and why the push comes from the plan.
sidebar:
  order: 9
---

The follower's main output is a velocity. Each swerve module then runs its own velocity loop: it measures how fast its wheel is turning, compares that with the velocity asked for, and pushes harder or softer to close the gap. That is **feedback**. It only acts once there is an error to see.

**Feed-forward** is the other half. If the code already knows the robot is about to speed up at 6 m/s², it can tell the wheels how much force that takes before any error has appeared. The velocity loop then only has the leftover error to correct. The follower offers this as an acceleration, `commandedAcceleration()`, and [`WheelForces`](/reference/follow/wheelforces/) turns it into a push in newtons for each wheel.

It is optional. Without it the follow commands send the velocity alone. The robot-code side is in [Wheel forces](/guides/wheel-forces/).

## The plan's acceleration, not the command's

The obvious way to get an acceleration is to subtract last loop's velocity command from this loop's and divide by the loop time. The follower does not do that, and the reason matters.

The command is the plan plus corrections. The corrections follow the measured pose: the push back onto the path, and the ease-in over the last centimeters. The measured pose wobbles by a few millimeters every loop. A few millimeters of wobble, multiplied by a gain and divided by 0.02 s, is a large and random acceleration. A feed-forward built from that kicks the robot about when it should be standing still.

So `commandedAcceleration()` reports what the **plan** calls for:

| While | Along the direction of travel | Sideways | Spin |
| --- | --- | --- | --- |
| Following the path | The plan's speeding up or slowing down at the point read ahead. Once the stopping law's brake governs, the braking deceleration. | `v²κ` toward the inside of the turn | The change in the plan's turn rate from here to the point read ahead, per second |
| Turning on the spot | 0 | 0 | The same |
| Driving to the goal | The [stopping law's](/concepts/stopping-law/) slowing, `min(a, k·v)`, along the line to the goal | 0 | The heading law's slowing |

Corrections are left to the wheels' velocity loops. That is the job they are good at.

Two limits are applied before the value is returned. The spin part is clamped to the spin acceleration cap. Then, if `|a| + c·|α|` is over the [limiter's](/concepts/output-limiter/) budget, all three parts are scaled down together. The value is never past the traction limit, and it is zero when the path is not running.

## From acceleration to wheel forces

Newton's second law gives the total: the wheels together must push with mass times acceleration, and turn the robot with inertia times spin acceleration. The question is how to share that between the wheels.

### Driving: by the weight each wheel carries

An equal share is the obvious answer, and it is wrong for a robot that is accelerating. A robot speeding up leans back. Its back wheels carry more of its weight and its front wheels less. A wheel can only push as hard as it is pressed into the floor, so a quarter of the push is more than the light front wheels can grip, and they spin.

`WheelForces` gives each wheel the share of the push that it carries of the weight. To find the weight on each wheel it treats the frame as stiff. The loads then lie on a flat plane across the wheels:

```text
load_i = p0 + p1·x_i + p2·y_i
```

`x_i` and `y_i` are where module `i` is. The three numbers `p0`, `p1` and `p2` are fixed by three balance conditions:

- the loads add up to the robot's weight, `m·g`,
- their pitching moment balances `m · h · a_forward`,
- their rolling moment balances `m · h · a_left`.

`h` is the height of the center of mass. A load that comes out below zero is counted as zero, because a wheel can be lifted off the floor but not pulled down into it. The loads are then divided by their total to give shares that add up to 1.

### Spinning: at right angles to each arm

The turning effort is shared too. Each wheel pushes at right angles to its arm from the middle of the modules, and wheels farther out push harder.

There is one correction. Unequal driving shares push off-center, and that would turn the robot a little on its own. The spin term is the turning effort asked for, less what the driving forces already give.

### Two sums that always hold

Whatever the shares are:

- the forces add up to mass times acceleration,
- their turning effort about the robot's center is inertia times spin acceleration.

The library's tests check both on random robots.

The result comes back as two arrays in the field frame, forces along x and forces along y, in newtons, one entry per module in the order the modules were given.

## What it assumes

The numbers are only as good as these assumptions, which are listed in the class itself.

- **Module positions are measured from the center of mass.** A robot whose weight sits off-center should give the positions from where the weight is, not from the middle of the frame.
- **The frame is stiff.** The plane of loads is the stiff-frame answer. With `centerOfMassHeightMeters` at 0 the robot does not lean, and the shares do not change with the acceleration.
- **The lean follows the acceleration at once.** A real robot takes a moment to pitch back on its bumpers and treads.
- **The push is what the floor must give the robot.** Spinning up the motors' own rotors takes a few percent more.

If the modules are all in one line, no plane fits and the shares are equal.

## What it does not include

The forces are mass and inertia times the acceleration, and nothing else. **Friction is not in them.** The force needed just to keep the drivetrain turning, and the extra for the rotors, are left to the drive motors' velocity loops and their static gain, kS.

The follower's feed-forward also does not include the corrections, as described above, and it does not detect slip.

:::caution
`centerOfMassHeightMeters` defaults to 0.2 m, and the library marks that value as a guess until it is measured. The weight shares depend on it directly. Measure the robot's and put it in [the robot file](/files/robot-file/).
:::

## The bench result

On the bench, which is a simulated drivetrain, sending these forces next to the velocity took a quarter to a third off the wheel slip at the same path times, and removed the overshoot at the finish. A path planned at 9 m/s² with the feed-forward slipped no more than one planned at 6 m/s² without it.

That is a simulator result. The library has not run on a real robot, so there is no measurement of what the feed-forward does there.

## Teleop and the rail

In plain teleop there is no plan, so the feed-forward is the limiter's own step: `OutputLimiter.acceleration()`. The sticks do not carry pose noise, so the difference between commands is safe to use there.

The rail is in between. Across the rail the command follows the measured pose, so only the law's own braking counts. Along the rail the command is the driver's, and the limiter's step along the rail is used. See [The rail law](/concepts/rail-law/).

## Related pages

- [`PathFollower`](/reference/follow/pathfollower/), for `commandedAcceleration()`
- [`WheelForces`](/reference/follow/wheelforces/)
- [`WheelForceDrive`](/reference/wpilib/wheelforcedrive/), the interface a drivetrain implements to take the forces
