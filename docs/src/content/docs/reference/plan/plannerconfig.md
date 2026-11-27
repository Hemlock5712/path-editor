---
title: PlannerConfig
description: The pathfinder's settings, including bumper size, margin, speed caps and footprint.
sidebar:
  order: 4
---

`frc.team5712.path.plan.PlannerConfig` is the record of settings a `Pathfinder` is built with: how big the robot is, how much room to leave, how carefully to check, and how fast the planned paths may go.

Most teams never write one in code. `PathLibrary.loadAll()` builds it from `robot.json` with `fromRobotFile`, so you change the bumper size and the on-the-fly speed caps by editing the [robot file](/files/robot-file/) in the editor. You meet the class directly when building a `Pathfinder` yourself, for a test or a tool. See [Pathfinding](/guides/pathfinding/) for the usual route.

## Example

```java
// what PathLibrary does: the defaults with the robot file's bumpers and speed caps
Map<String, Object> robotJson = PathFile.asMap(Json.parse(text), "robot.json");
PlannerConfig config = PlannerConfig.fromRobotFile(robotJson);

// by hand: the defaults with a wider margin and gentler limits
PlannerConfig gentle = PlannerConfig.DEFAULT
    .withMargin(PlannerConfig.WIDE_MARGIN_METERS)
    .withLimits(2.5, 4.0);

Pathfinder pathfinder = new Pathfinder(field, RobotConfig.DEFAULT, gentle);
```

`text` is the contents of `robot.json` as a string. The numbers in the second half show the methods. They are not recommended settings.

## Components

| Component | Type | Unit | `DEFAULT` | Description |
| --- | --- | --- | --- | --- |
| `bumperLengthMeters` | `double` | meters | 0.85 | Robot length with bumpers, along its heading |
| `bumperWidthMeters` | `double` | meters | 0.85 | Robot width with bumpers |
| `marginMeters` | `double` | meters | 0.05 | Extra clearance added when obstacles are grown, and the clearance the sweep demands of the finished path |
| `arcStepRadians` | `double` | radians | 30 degrees, `Math.toRadians(30)` | The largest step of the short straight pieces that round the grown corners |
| `maxRefits` | `int` | | 8 | How many times the smoother may add waypoints and fit again before giving up |
| `checkStepMeters` | `double` | meters | 0.02 | The spacing of the swept-outline check along the path |
| `maxMetersPerSecond` | `double` | m/s | 4.0 | The speed cap of planned paths |
| `maxMetersPerSecondSquared` | `double` | m/s² | 6.0 | The cap on both speeding up and slowing down |
| `footprint` | `Footprint` | | the bumpers | The outline planned with: the bumpers, or the bumpers with a mechanism out |

The speed caps are caps. The speed plan still applies the robot's own traction and motor limits from its `RobotConfig`, so a value above what the robot can do changes nothing.

## Constants

| Constant | Value | Description |
| --- | --- | --- |
| `DEFAULT` | see the table above | 0.85 m square bumpers, 5 cm margin, 30 degree arcs, 8 refits, 2 cm checks, 4 m/s and 6 m/s², planning with the bumpers alone |
| `WIDE_MARGIN_METERS` | `0.15` | Meters. The margin on-the-fly paths try first. Wheel odometry can drift by more than the default 5 cm, so where the field has room the robot keeps 15 cm away. |

With the 0.85 m robot, the 0.15 m margin closes any gap under 1.5 m. The 0.05 m plan still gets through gaps down to 1.3 m. See [On-the-fly paths](/concepts/on-the-fly/) for how the two are chosen between.

## Constructors

| Constructor | Description |
| --- | --- |
| `PlannerConfig(bumperLengthMeters, bumperWidthMeters, marginMeters, arcStepRadians, maxRefits, checkStepMeters, maxMetersPerSecond, maxMetersPerSecondSquared, footprint)` | Every component, in the order of the table |
| `PlannerConfig(bumperLengthMeters, bumperWidthMeters, marginMeters, arcStepRadians, maxRefits, checkStepMeters, maxMetersPerSecond, maxMetersPerSecondSquared)` | The same without a footprint. It plans with the bumpers alone, `Footprint.bumpers(bumperLengthMeters, bumperWidthMeters)`. |

Both throw `IllegalArgumentException` with `"bad planner settings"` unless all of these hold:

- `bumperLengthMeters` and `bumperWidthMeters` are above 0;
- `marginMeters` is 0 or more;
- `arcStepRadians` is above 0 and at most `Math.PI / 2`;
- `maxRefits` is 0 or more;
- `checkStepMeters`, `maxMetersPerSecond` and `maxMetersPerSecondSquared` are above 0.

A null `footprint` throws `IllegalArgumentException` with a message that says to use `Footprint.bumpers` for the bumpers alone.

## Static methods

### fromRobotFile(robot)

```java
public static PlannerConfig fromRobotFile(Map<String, Object> robot)
```

`DEFAULT` with four values taken from a parsed `robot.json`. Every key is optional.

| Key in `robot.json` | Unit | Used when absent | Sets |
| --- | --- | --- | --- |
| `bumperLengthMeters` | meters | 0.85 | `bumperLengthMeters` |
| `bumperWidthMeters` | meters | 0.85 | `bumperWidthMeters` |
| `onTheFlyMaxMetersPerSecond` | m/s | 4 | `maxMetersPerSecond` |
| `onTheFlyMaxMetersPerSecondSquared` | m/s² | 6 | `maxMetersPerSecondSquared` |

The margin, arc step, refits and check step are always `DEFAULT`'s, and the footprint is the bumpers. Throws `IllegalArgumentException` when a key is present but is not a number, or when the values fail the constructor's checks.

It does not read the `footprints` list. `Footprint.fromRobotFile` does that, and `PathLibrary` combines the two with `withFootprint`.

## Methods

| Method | Returns | Description |
| --- | --- | --- |
| `halfDiagonalMeters()` | `double` | The robot's reach in any heading, in meters: the bumpers' half-diagonal, or the far corner of the footprint when a mechanism is out. The same as `footprint().reachMeters()`. |
| `growMeters()` | `double` | How far obstacles are grown, in meters: `halfDiagonalMeters() + marginMeters` |
| `bumperGrowMeters()` | `double` | `growMeters()` for the bumpers alone, whatever the footprint: `hypot(bumperLengthMeters, bumperWidthMeters) / 2 + marginMeters` |
| `withMargin(double margin)` | `PlannerConfig` | A copy with a different margin, in meters |
| `withLimits(double maxSpeed, double maxAccel)` | `PlannerConfig` | A copy with different caps, in m/s and m/s² |
| `withFootprint(Footprint outline)` | `PlannerConfig` | A copy that plans with `outline`, one of the footprints from `Footprint.fromRobotFile` |

The `with` methods go through the constructor, so they throw the same `IllegalArgumentException` for a bad value.

For `DEFAULT`:

```text
halfDiagonalMeters = sqrt(0.85^2 + 0.85^2) / 2 = 0.601 m
growMeters         = 0.601 + 0.05              = 0.651 m
```

:::note
`withFootprint` does not change `bumperLengthMeters` or `bumperWidthMeters`. The pathfinder uses the footprint for growing and sweeping, and the bumper size only for the fallback graph of a footprint that reaches further than the bumpers. See [Footprints](/concepts/footprints/).
:::

## Related

- [Pathfinder](/reference/plan/pathfinder/)
- [Footprint](/reference/core/footprint/)
- [Robot file](/files/robot-file/)
- [PathLibrary](/reference/wpilib/pathlibrary/)
