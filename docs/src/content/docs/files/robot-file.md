---
title: Robot file
description: Every key of robot.json, with its unit, its default and what reads it.
sidebar:
  order: 2
---

`robot.json` describes the robot to the library: how heavy it is, where its wheels are, what its motors can do, and how big its bumpers are. Every path is planned with these numbers, and the commands that drive the robot are limited by them. The file lives in `deploy/paths/` next to the path files. `PathLibrary.loadAll()` reads it once at boot.

Every key is optional. A key that is missing keeps its default, and if there is no `robot.json` at all, every default applies. The defaults describe the bench robot the library was developed against. They are a starting point, not a measurement of your robot.

## An example

This example sets every key the library reads. The values shown are the defaults, apart from the three optional caps and the footprint, which have no default value to show.

```json
{
  "massKg": 60,
  "momentOfInertiaKgM2": 6,
  "moduleRadiusMeters": 0.41012,
  "wheelRadiusMeters": 0.0508,
  "driveGearRatio": 6.75,
  "driveMotorCount": 4,
  "statorCurrentLimitAmps": 120,
  "frictionCoefficient": 1.5,
  "maxWheelMetersPerSecond": 4.5,
  "maxAngularRadPerSec": 9.4,
  "maxAngularRadPerSecSquared": 12.6,
  "centerOfMassHeightMeters": 0.2,
  "tipDistanceMeters": 0.29,
  "tractionShare": 0.9,
  "motor": {
    "nominalVolts": 12,
    "stallTorqueNewtonMeters": 9.362,
    "stallCurrentAmps": 476.1,
    "freeSpeedRadPerSec": 605.8
  },
  "bumperLengthMeters": 0.85,
  "bumperWidthMeters": 0.85,
  "onTheFlyMaxMetersPerSecond": 4,
  "onTheFlyMaxMetersPerSecondSquared": 6,
  "footprints": [
    { "name": "intakeOut", "frontMeters": 0.3, "retractSeconds": 0.4 }
  ]
}
```

## Three readers

Three parts of the library read this one file, each taking the keys it knows and ignoring the rest.

| Reader | Keys | Produces |
| --- | --- | --- |
| [RobotFile](/reference/core/robotfile/) | The mass, drivetrain, traction and `motor` keys | A [RobotConfig](/reference/core/robotconfig/) for the speed plan and the follower |
| [PlannerConfig.fromRobotFile](/reference/plan/plannerconfig/) | The bumper size and the two `onTheFly` keys | Settings for the pathfinder |
| [Footprint.fromRobotFile](/reference/core/footprint/) | The bumper size and `footprints` | The list of robot outlines |

## The robot

Read by `RobotFile`.

| Key | Unit | Default | Meaning |
| --- | --- | --- | --- |
| `massKg` | kg | 60 | Robot mass with battery and bumpers |
| `momentOfInertiaKgM2` | kg·m² | 6 | Moment of inertia about the vertical axis |
| `moduleRadiusMeters` | m | 0.41012 | Distance from the robot's center to a swerve module |
| `wheelRadiusMeters` | m | 0.0508 | Drive wheel radius. The default is a 2 inch radius |
| `driveGearRatio` | motor turns per wheel turn | 6.75 | Drive gear ratio |
| `driveMotorCount` | count | 4 | Number of drive motors |
| `statorCurrentLimitAmps` | A | 120 | Drive stator current limit |
| `maxWheelMetersPerSecond` | m/s | the motor's free speed at the wheel | The fastest any wheel may be asked to roll |
| `maxAngularRadPerSec` | rad/s | no cap | Spin rate cap for the planned heading |
| `maxAngularRadPerSecSquared` | rad/s² | no cap | Spin acceleration cap |

The default `moduleRadiusMeters` is the diagonal of modules at 0.29 m from the center on each axis. With the default motor, gearing and wheel, the free speed at the wheel is 4.56 m/s.

The two spin caps need a note. When they are not set, the speed plan has no spin cap of its own, and only the wheel-speed, traction and motor limits hold the planned spin. The follower still applies caps of 540 degrees per second and 720 degrees per second squared to what it commands. When `maxAngularRadPerSecSquared` is set, the plan turns at `tractionShare` of it and the follower is held to the full value.

## Traction

Read by `RobotFile`. These four keys set the traction limit, `tractionShare * min(μ * g, g * x / h)`. The [Traction](/concepts/traction/) page explains the formula.

| Key | Unit | Default | Meaning |
| --- | --- | --- | --- |
| `frictionCoefficient` | none | 1.5 | The tread's friction coefficient `μ` on carpet |
| `centerOfMassHeightMeters` | m | 0.2 | Height `h` of the center of mass above the floor. 0 turns the tipping limit off |
| `tipDistanceMeters` | m | `moduleRadiusMeters / √2` | Distance `x` from the center to the nearest tipping edge: the smaller of the modules' x and y distances |
| `tractionShare` | none | 0.9 | The share of grip or tipping that the robot uses. Above 0, at most 1 |

With the defaults the limit is 12.8 m/s².

An older file may contain a `tractionRule` key. It is ignored.

## motor

Read by `RobotFile`. The `motor` object describes one drive motor as a [Motor](/reference/core/motor/). Each key inside it is optional too. The defaults are the Kraken X60 with FOC.

| Key | Unit | Default | Meaning |
| --- | --- | --- | --- |
| `nominalVolts` | V | 12 | The voltage the other three numbers were measured at |
| `stallTorqueNewtonMeters` | N·m | 9.362 | Torque at zero speed |
| `stallCurrentAmps` | A | 476.1 | Current at zero speed |
| `freeSpeedRadPerSec` | rad/s | 605.8 | Speed with no load. This is 5,785 rpm |

## Bumpers and on-the-fly limits

Read by `PlannerConfig.fromRobotFile`. The bumper keys are also read by `Footprint.fromRobotFile`.

| Key | Unit | Default | Meaning |
| --- | --- | --- | --- |
| `bumperLengthMeters` | m | 0.85 | Robot length with bumpers, along its heading |
| `bumperWidthMeters` | m | 0.85 | Robot width with bumpers |
| `onTheFlyMaxMetersPerSecond` | m/s | 4 | Speed cap for paths planned during a match |
| `onTheFlyMaxMetersPerSecondSquared` | m/s² | 6 | Acceleration and braking cap for those paths |

The pathfinder grows every obstacle by the bumpers' half-diagonal plus a margin, so the bumper size decides which gaps the robot is allowed through. See [The pathfinder](/concepts/pathfinder/).

The two `onTheFly` keys apply to drive-to-pose and pathfinding commands, not to drawn paths. A drawn path carries its own maxima in its [path file](/files/path-file/). The robot's traction limit still applies to both, so an `onTheFlyMaxMetersPerSecondSquared` above the traction limit changes nothing.

## footprints

Read by `Footprint.fromRobotFile`. A [Footprint](/reference/core/footprint/) is the robot's outline seen from above, as a rectangle. The list always starts with one called `"bumpers"`, built from the two bumper keys. Each entry in `footprints` adds another: the bumpers plus how much further a mechanism reaches on each side.

| Key | Unit | Default | Meaning |
| --- | --- | --- | --- |
| `name` | text | required | What path events and commands call this footprint |
| `frontMeters` | m | 0 | Reach past the front bumper, along the heading |
| `backMeters` | m | 0 | Reach past the back bumper |
| `leftMeters` | m | 0 | Reach past the left bumper |
| `rightMeters` | m | 0 | Reach past the right bumper |
| `retractSeconds` | s | 0 | How long the mechanism takes to come back in |

Names must be different from each other, and none may be `"bumpers"`. Every extension must be zero or more. A path that names a footprint this list does not have fails at boot. See [Footprints](/concepts/footprints/).

## What to measure

The defaults will run in simulation. On a real robot, these are the numbers that differ from robot to robot:

1. **`massKg`.** Weigh the robot with its battery and bumpers on.
2. **`moduleRadiusMeters` and `tipDistanceMeters`.** Measure from the center of the robot to the center of a wheel. For a base that is not square, also set `tipDistanceMeters` to the smaller of the two axis distances.
3. **`centerOfMassHeightMeters`.** The default of 0.2 m is a guess, and with the default geometry it is the number that sets the traction limit.
4. **`wheelRadiusMeters`.** Treads wear, and the radius shrinks with them.
5. **`driveGearRatio`, `driveMotorCount` and `statorCurrentLimitAmps`.** Copy these from the swerve modules and from the drive motor configuration in your robot code.
6. **`bumperLengthMeters` and `bumperWidthMeters`.** Measure the outside of the bumpers.
7. **`frictionCoefficient`.** Use the value for your tread on carpet.

`momentOfInertiaKgM2` is hard to measure directly. The source does not give a procedure for it, so this page does not either.

:::caution
The library has not run on a real robot. Treat every default as a number to check, and follow the order in [Tuning](/guides/tuning/) when you first drive.
:::

## Errors

Unknown keys are ignored. A value of the wrong type, or a number outside its allowed range, throws an `IllegalArgumentException` when the file is read, which stops the robot code at boot. Every size, ratio, count and limit in the robot section must be above zero.
