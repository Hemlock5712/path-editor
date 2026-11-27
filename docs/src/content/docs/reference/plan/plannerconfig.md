---
title: PlannerConfig
description: The pathfinder's settings, including bumper size, margin, speed caps and footprint.
sidebar:
  order: 4
---

`PlannerConfig` is the record of settings a `Pathfinder` is built with: how big the robot is, how much room to leave, how carefully to check, and how fast the planned paths may go.

Most teams never write one in code. `PathLibrary.loadAll()` builds it from `robot.json` with `fromRobotFile`, so you change the bumper size and the on-the-fly speed caps by editing the [robot file](/files/robot-file/) in the editor. See [Pathfinding](/guides/pathfinding/).

Package: `frc.team5712.path.plan`.

## Example

```java
// what PathLibrary does: the defaults with the robot file's bumpers and speed caps
// text is the contents of robot.json as a string
Map<String, Object> robotJson = PathFile.asMap(Json.parse(text), "robot.json");
PlannerConfig config = PlannerConfig.fromRobotFile(robotJson);

// by hand: the defaults with a wider margin and gentler limits
// these numbers show the methods. They are not recommended settings.
PlannerConfig gentle = PlannerConfig.DEFAULT
    .withMargin(PlannerConfig.WIDE_MARGIN_METERS)
    .withLimits(2.5, 4.0);

Pathfinder pathfinder = new Pathfinder(field, RobotConfig.DEFAULT, gentle);
```

## Components

| Component | `DEFAULT` | What it is |
| --- | --- | --- |
| `double bumperLengthMeters` | 0.85 | Robot length with bumpers, along its heading, in meters |
| `double bumperWidthMeters` | 0.85 | Robot width with bumpers, in meters |
| `double marginMeters` | 0.05 | Extra clearance added when obstacles are grown, and the clearance the sweep demands of the finished path, in meters |
| `double arcStepRadians` | 30 degrees, `Math.toRadians(30)` | The largest step of the short straight pieces that round the grown corners, in radians |
| `int maxRefits` | 8 | How many times the smoother may add waypoints and fit again before giving up |
| `double checkStepMeters` | 0.02 | The spacing of the swept-outline check along the path, in meters |
| `double maxMetersPerSecond` | 4.0 | The speed cap of planned paths, in m/s |
| `double maxMetersPerSecondSquared` | 6.0 | The cap on both speeding up and slowing down, in m/s² |
| `Footprint footprint` | the bumpers | The outline planned with: the bumpers, or the bumpers with a mechanism out |

## Constants

| Constant | What it is |
| --- | --- |
| `DEFAULT` | 0.85 m square bumpers, 5 cm margin, 30 degree arcs, 8 refits, 2 cm checks, 4 m/s and 6 m/s², planning with the bumpers alone |
| `WIDE_MARGIN_METERS` | `0.15` meters. The margin on-the-fly paths try first. Wheel odometry can drift by more than the default 5 cm, so where the field has room the robot keeps 15 cm away. |

## Constructors

| Constructor | What it does |
| --- | --- |
| `PlannerConfig(bumperLengthMeters, bumperWidthMeters, marginMeters, arcStepRadians, maxRefits, checkStepMeters, maxMetersPerSecond, maxMetersPerSecondSquared, footprint)` | Every component, in the order of the table. A null `footprint` throws `IllegalArgumentException` with a message that says to use `Footprint.bumpers` for the bumpers alone. |
| `PlannerConfig(bumperLengthMeters, bumperWidthMeters, marginMeters, arcStepRadians, maxRefits, checkStepMeters, maxMetersPerSecond, maxMetersPerSecondSquared)` | The same without a footprint. It plans with the bumpers alone, `Footprint.bumpers(bumperLengthMeters, bumperWidthMeters)`. |

Both throw `IllegalArgumentException` with `"bad planner settings"` unless all of these hold:

| Component | Must be |
| --- | --- |
| `bumperLengthMeters`, `bumperWidthMeters` | Above 0 |
| `marginMeters` | 0 or more |
| `arcStepRadians` | Above 0 and at most `Math.PI / 2` |
| `maxRefits` | 0 or more |
| `checkStepMeters`, `maxMetersPerSecond`, `maxMetersPerSecondSquared` | Above 0 |

## Static methods

| Method | What it does |
| --- | --- |
| `public static PlannerConfig fromRobotFile(Map<String, Object> robot)` | `DEFAULT` with four values taken from a parsed `robot.json`. The margin, arc step, refits and check step are always `DEFAULT`'s, and the footprint is the bumpers. Throws `IllegalArgumentException` when a key is present but is not a number, or when the values fail the constructor's checks. |

Every key is optional.

| Key in `robot.json` | Used when absent | What it sets |
| --- | --- | --- |
| `bumperLengthMeters` | 0.85 | `bumperLengthMeters`, in meters |
| `bumperWidthMeters` | 0.85 | `bumperWidthMeters`, in meters |
| `onTheFlyMaxMetersPerSecond` | 4 | `maxMetersPerSecond`, in m/s |
| `onTheFlyMaxMetersPerSecondSquared` | 6 | `maxMetersPerSecondSquared`, in m/s² |

## Methods

| Method | What it does |
| --- | --- |
| `double halfDiagonalMeters()` | The robot's reach in any heading, in meters: the bumpers' half-diagonal, or the far corner of the footprint when a mechanism is out. The same as `footprint().reachMeters()`. 0.601 m for `DEFAULT`. |
| `double growMeters()` | How far obstacles are grown, in meters: `halfDiagonalMeters() + marginMeters`. 0.651 m for `DEFAULT`. |
| `double bumperGrowMeters()` | `growMeters()` for the bumpers alone, whatever the footprint: `hypot(bumperLengthMeters, bumperWidthMeters) / 2 + marginMeters`. |
| `PlannerConfig withMargin(double margin)` | A copy with a different margin, in meters. |
| `PlannerConfig withLimits(double maxSpeed, double maxAccel)` | A copy with different caps, in m/s and m/s². |
| `PlannerConfig withFootprint(Footprint outline)` | A copy that plans with `outline`, one of the footprints from `Footprint.fromRobotFile`. |

## Good to know

- **The speed caps are caps.** The speed plan still applies the robot's own traction and motor limits from its `RobotConfig`, so a value above what the robot can do changes nothing.
- **A wider margin closes gaps.** With the 0.85 m robot, the 0.15 m margin closes any gap under 1.5 m. The 0.05 m plan still gets through gaps down to 1.3 m.
- **The `with` methods go through the constructor**, so they throw the same `IllegalArgumentException` for a bad value.
- **`fromRobotFile` does not read the `footprints` list.** `Footprint.fromRobotFile` does that, and `PathLibrary` combines the two with `withFootprint`.

:::note
`withFootprint` does not change `bumperLengthMeters` or `bumperWidthMeters`. The pathfinder uses the footprint for growing and sweeping, and the bumper size only for the fallback graph of a footprint that reaches further than the bumpers. See [Footprints](/concepts/footprints/).
:::

## More

- [On-the-fly paths](/concepts/on-the-fly/): how the two margins are chosen between.
- [Pathfinder](/reference/plan/pathfinder/) and [Footprint](/reference/core/footprint/): what the settings feed.
- [Robot file](/files/robot-file/) and [PathLibrary](/reference/wpilib/pathlibrary/): where the settings come from.
