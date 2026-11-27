---
title: Robot file
description: Every key of robot.json, with its unit, its default and what reads it.
sidebar:
  order: 2
---

`robot.json` describes the robot to the library. It lives in `deploy/paths/`, and `PathLibrary.loadAll()` reads it once at boot.

## An example

Every key is optional, and a missing key keeps its default. The values here are the defaults, apart from the three optional caps and the footprint, which have none.

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

:::caution
The library has not run on a real robot, and the defaults describe the bench robot it was developed against. Treat every default as a number to check, and follow the order in [Tuning](/guides/tuning/) when you first drive.
:::

## Who reads it

Each reader takes the keys it knows and ignores the rest.

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

- **With no spin caps set**, the plan has no spin cap of its own. The follower still applies 540 degrees per second and 720 degrees per second squared to what it commands.
- **With `maxAngularRadPerSecSquared` set**, the plan turns at `tractionShare` of it and the follower is held to the full value.
- **The default free speed at the wheel** is 4.56 m/s, with the default motor, gearing and wheel.

## Traction

Read by `RobotFile`. These four keys set the traction limit, `tractionShare * min(μ * g, g * x / h)`, which is 12.8 m/s² with the defaults. See [Traction](/concepts/traction/).

| Key | Unit | Default | Meaning |
| --- | --- | --- | --- |
| `frictionCoefficient` | none | 1.5 | The tread's friction coefficient `μ` on carpet |
| `centerOfMassHeightMeters` | m | 0.2 | Height `h` of the center of mass above the floor. 0 turns the tipping limit off |
| `tipDistanceMeters` | m | `moduleRadiusMeters / √2` | Distance `x` from the center to the nearest tipping edge: the smaller of the modules' x and y distances |
| `tractionShare` | none | 0.9 | The share of grip or tipping that the robot uses. Above 0, at most 1 |

An older file may contain a `tractionRule` key. It is ignored.

## motor

Read by `RobotFile`. The `motor` object describes one drive motor as a [Motor](/reference/core/motor/), and the defaults are the Kraken X60 with FOC.

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

- **The bumper size decides which gaps the robot is allowed through.** See [The pathfinder](/concepts/pathfinder/).
- **The `onTheFly` keys do not apply to drawn paths.** A drawn path carries its own maxima in its [path file](/files/path-file/).
- **The traction limit still applies to both.** An `onTheFlyMaxMetersPerSecondSquared` above it changes nothing.

## footprints

Read by `Footprint.fromRobotFile`. Each entry adds a [Footprint](/reference/core/footprint/) to the list, which always starts with `"bumpers"`: the bumpers plus how much further a mechanism reaches on each side. See [Footprints](/concepts/footprints/).

| Key | Unit | Default | Meaning |
| --- | --- | --- | --- |
| `name` | text | required | What path events and commands call this footprint |
| `frontMeters` | m | 0 | Reach past the front bumper, along the heading |
| `backMeters` | m | 0 | Reach past the back bumper |
| `leftMeters` | m | 0 | Reach past the left bumper |
| `rightMeters` | m | 0 | Reach past the right bumper |
| `retractSeconds` | s | 0 | How long the mechanism takes to come back in |

Names must be different from each other, and none may be `"bumpers"`. Every extension must be zero or more.

## What to measure

These numbers differ from robot to robot. [Tuning](/guides/tuning/) has ways to measure some of them.

| Key | Note |
| --- | --- |
| `massKg` | With battery and bumpers on |
| `moduleRadiusMeters`, `tipDistanceMeters` | From the center of the robot to the center of a wheel. For a base that is not square, also set `tipDistanceMeters` to the smaller of the two axis distances. |
| `centerOfMassHeightMeters` | The default of 0.2 m is a guess, and with the default geometry it is the number that sets the traction limit. |
| `wheelRadiusMeters` | Treads wear, and the radius shrinks with them. |
| `driveGearRatio`, `driveMotorCount`, `statorCurrentLimitAmps` | Copy from the swerve modules and the drive motor configuration in your robot code. |
| `bumperLengthMeters`, `bumperWidthMeters` | The outside of the bumpers |
| `frictionCoefficient` | The value for your tread on carpet |

## Errors

Unknown keys are ignored. A value of the wrong type, or a number outside its allowed range, throws an `IllegalArgumentException` when the file is read, which stops the robot code at boot.
