---
title: RobotFile
description: Reads and writes a RobotConfig as the JSON in robot.json.
sidebar:
  order: 15
---

`RobotFile` converts between JSON and a [RobotConfig](/reference/core/robotconfig/), which is how the robot's numbers get from `robot.json` into the library.

Most students edit the file and never call this class. `PathLibrary.loadAll()` calls `RobotFile.read` on `deploy/paths/robot.json` at boot, and uses `RobotConfig.DEFAULT` if the file is not there.

Package: `frc.team5712.path.core`.

## Example

```java
// Two keys set, every other key keeps its default
RobotConfig robot = RobotFile.read("{ \"massKg\": 52, \"frictionCoefficient\": 1.3 }");

double mass = robot.massKg();                 // 52
double gearRatio = robot.driveGearRatio();    // 6.75, the default
```

## Methods

All three are static. The class cannot be constructed.

| Method | What it does |
| --- | --- |
| `RobotConfig read(String json)` | Parses JSON text and builds a `RobotConfig` from it. |
| `RobotConfig fromJson(Map<String, Object> m)` | Builds a `RobotConfig` from a JSON tree that is already parsed, as returned by [Json](/reference/core/json/). |
| `Map<String, Object> toJson(RobotConfig r)` | The JSON tree for a config, with keys in a fixed order. It returns a tree, not text. Pass it to `Json.write` to get a string. |

## Errors

`read` and `fromJson` throw `IllegalArgumentException` when:

| What is wrong | Message, where the page gives one |
| --- | --- |
| The text is not valid JSON, or its top level is not an object | |
| A key that should hold a number holds something else | Such as `'massKg' must be a number` |
| `motor` is present and is not an object | |
| A value is outside the range `RobotConfig` allows, such as a mass of 0 | |

## Keys

Reading takes these keys, and `toJson` writes them in this order.

| Key | What it becomes |
| --- | --- |
| `massKg` | `massKg` |
| `momentOfInertiaKgM2` | `momentOfInertiaKgM2` |
| `moduleRadiusMeters` | `moduleRadiusMeters` |
| `wheelRadiusMeters` | `wheelRadiusMeters` |
| `driveGearRatio` | `driveGearRatio` |
| `driveMotorCount` | `driveMotorCount`. The number is cut to a whole number |
| `statorCurrentLimitAmps` | `statorCurrentLimitAmps` |
| `frictionCoefficient` | `frictionCoefficient` |
| `maxWheelMetersPerSecond` | `maxWheelMetersPerSecond`. Left out by `toJson` when not set |
| `maxAngularRadPerSec` | `maxAngularRadPerSec`. Left out by `toJson` when not set |
| `maxAngularRadPerSecSquared` | `maxAngularRadPerSecSquared`. Left out by `toJson` when not set |
| `centerOfMassHeightMeters` | `centerOfMassHeightMeters` |
| `tipDistanceMeters` | `tipDistanceMeters`. Left out by `toJson` when not set |
| `tractionShare` | `tractionShare` |
| `motor` | `driveMotor`. An object with `nominalVolts`, `stallTorqueNewtonMeters`, `stallCurrentAmps` and `freeSpeedRadPerSec`, each optional |

## Good to know

- **Every key is optional when reading.** A missing key keeps the value from `RobotConfig.DEFAULT`.
- **Every other key is ignored.** That includes `bumperLengthMeters`, `bumperWidthMeters`, `onTheFlyMaxMetersPerSecond`, `onTheFlyMaxMetersPerSecondSquared` and `footprints`, which other classes read from the same file, and `tractionRule` from older files.

:::note
`toJson` only knows what is in a `RobotConfig`. It does not write the bumper size, the on-the-fly limits or the footprints, so writing a config over an existing robot.json would lose those keys.
:::

## More

- [Robot file](/files/robot-file/): every key in the file.
- [RobotConfig](/reference/core/robotconfig/): what the file becomes.
- [Json](/reference/core/json/): the parser underneath.
