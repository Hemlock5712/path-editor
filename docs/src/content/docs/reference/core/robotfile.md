---
title: RobotFile
description: Reads and writes a RobotConfig as the JSON in robot.json.
sidebar:
  order: 15
---

`RobotFile` converts between JSON and a [RobotConfig](/reference/core/robotconfig/). It is how the robot's numbers get from `robot.json` into the library. Every key is optional when reading: a missing key keeps the value from `RobotConfig.DEFAULT`. The class has only static methods and cannot be constructed.

Package: `frc.team5712.path.core`.

## Where you meet it

`PathLibrary.loadAll()` calls `RobotFile.read` on `deploy/paths/robot.json` at boot, and uses `RobotConfig.DEFAULT` if the file is not there. Most students edit the file and never call this class. The [Robot file](/files/robot-file/) page lists every key.

## Example

```java
RobotConfig robot = RobotFile.read("{ \"massKg\": 52, \"frictionCoefficient\": 1.3 }");

double mass = robot.massKg();                 // 52
double gearRatio = robot.driveGearRatio();    // 6.75, the default
```

## Methods

All three are static.

| Method | Description |
| --- | --- |
| `RobotConfig read(String json)` | Parses JSON text and builds a `RobotConfig` from it. |
| `RobotConfig fromJson(Map<String, Object> m)` | Builds a `RobotConfig` from a JSON tree that is already parsed, as returned by [Json](/reference/core/json/). |
| `Map<String, Object> toJson(RobotConfig r)` | The JSON tree for a config, with keys in a fixed order. |

## Keys read

| Key | Becomes |
| --- | --- |
| `massKg` | `massKg` |
| `momentOfInertiaKgM2` | `momentOfInertiaKgM2` |
| `moduleRadiusMeters` | `moduleRadiusMeters` |
| `wheelRadiusMeters` | `wheelRadiusMeters` |
| `driveGearRatio` | `driveGearRatio` |
| `driveMotorCount` | `driveMotorCount`. The number is cut to a whole number |
| `statorCurrentLimitAmps` | `statorCurrentLimitAmps` |
| `frictionCoefficient` | `frictionCoefficient` |
| `maxWheelMetersPerSecond` | `maxWheelMetersPerSecond` |
| `maxAngularRadPerSec` | `maxAngularRadPerSec` |
| `maxAngularRadPerSecSquared` | `maxAngularRadPerSecSquared` |
| `centerOfMassHeightMeters` | `centerOfMassHeightMeters` |
| `tipDistanceMeters` | `tipDistanceMeters` |
| `tractionShare` | `tractionShare` |
| `motor` | `driveMotor`. An object with `nominalVolts`, `stallTorqueNewtonMeters`, `stallCurrentAmps` and `freeSpeedRadPerSec`, each optional |

Every other key is ignored. That includes `bumperLengthMeters`, `bumperWidthMeters`, `onTheFlyMaxMetersPerSecond`, `onTheFlyMaxMetersPerSecondSquared` and `footprints`, which other classes read from the same file, and `tractionRule` from older files.

## Errors

`read` and `fromJson` throw `IllegalArgumentException` when:

- the text is not valid JSON, or its top level is not an object,
- a key that should hold a number holds something else, with a message such as `'massKg' must be a number`,
- `motor` is present and is not an object,
- a value is outside the range `RobotConfig` allows, such as a mass of 0.

## What toJson writes

`toJson` writes the keys in this order: `massKg`, `momentOfInertiaKgM2`, `moduleRadiusMeters`, `wheelRadiusMeters`, `driveGearRatio`, `driveMotorCount`, `statorCurrentLimitAmps`, `frictionCoefficient`, `maxWheelMetersPerSecond`, `maxAngularRadPerSec`, `maxAngularRadPerSecSquared`, `centerOfMassHeightMeters`, `tipDistanceMeters`, `tractionShare`, `motor`.

The four optional limits, `maxWheelMetersPerSecond`, `maxAngularRadPerSec`, `maxAngularRadPerSecSquared` and `tipDistanceMeters`, are left out when they are not set. `toJson` returns a tree, not text. Pass it to `Json.write` to get a string.

:::note
`toJson` only knows what is in a `RobotConfig`. It does not write the bumper size, the on-the-fly limits or the footprints, so writing a config over an existing robot.json would lose those keys.
:::
