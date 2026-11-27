---
title: RobotConfig
description: The robot numbers that every path is planned with and every command is limited by.
sidebar:
  order: 14
---

`RobotConfig` is a record of the robot's physical numbers: mass, inertia, where the modules are, the drive motors and gearing, the tread's grip, and the center of mass. The speed plan reads it to find how fast a path may be driven, and the follower's limiter reads it too. It also works out the traction limit, `tractionShare * min(μ * g, g * x / h)`. [Traction](/concepts/traction/) explains that formula.

Package: `frc.team5712.path.core`.

## Where you meet it

You set these numbers in [robot.json](/files/robot-file/), not in code. `PathLibrary.loadAll()` reads the file, and `paths.robot()` returns the result. You pass that to settings such as `FollowerConfig.defaults(paths.robot())`. See [Project files](/start/project-files/).

## Example

```java
RobotConfig robot = paths.robot();

double limit = robot.tractionMetersPerSecondSquared();       // m/s²
RobotConfig.TractionLimit why = robot.tractionLimit();        // GRIP or TIPPING
double topWheelSpeed = robot.wheelSpeedCapMetersPerSecond();  // m/s

RobotConfig slippery = robot.withFrictionCoefficient(1.0);
```

## Components

The default column is the value in `RobotConfig.DEFAULT`.

| Component | Type | Unit | Default | Meaning |
| --- | --- | --- | --- | --- |
| `massKg` | `double` | kg | 60 | Robot mass with battery and bumpers |
| `momentOfInertiaKgM2` | `double` | kg·m² | 6 | Moment of inertia about the vertical axis |
| `moduleRadiusMeters` | `double` | m | 0.41012 | Distance from the robot's center to a swerve module |
| `wheelRadiusMeters` | `double` | m | 0.0508 | Drive wheel radius |
| `driveGearRatio` | `double` | | 6.75 | Motor turns per wheel turn |
| `driveMotor` | `Motor` | | `Motor.KRAKEN_X60_FOC` | The drive motor model |
| `driveMotorCount` | `int` | | 4 | Number of drive motors |
| `statorCurrentLimitAmps` | `double` | A | 120 | Drive stator current limit |
| `frictionCoefficient` | `double` | | 1.5 | The tread's friction coefficient `μ` |
| `maxWheelMetersPerSecond` | `double` | m/s | `NaN` | The fastest any wheel may be asked to roll. `NaN` means the motor's free speed |
| `maxAngularRadPerSec` | `double` | rad/s | `+∞` | Spin cap for the planned heading. `+∞` means only the wheel-speed cap |
| `maxAngularRadPerSecSquared` | `double` | rad/s² | `+∞` | Spin acceleration cap. `+∞` means only grip and motors |
| `centerOfMassHeightMeters` | `double` | m | 0.2 | Height of the center of mass above the floor. 0 means no tipping limit |
| `tipDistanceMeters` | `double` | m | `NaN` | Distance from the center to the nearest tipping edge. `NaN` means a square base, `moduleRadiusMeters / √2` |
| `tractionShare` | `double` | | 0.9 | The share of grip or tipping that the robot plans and drives to |

The plan turns at `tractionShare` of `maxAngularRadPerSecSquared`, and the follower is held to the full value, so feedback has room.

## Constants

| Constant | Value | Meaning |
| --- | --- | --- |
| `GRAVITY` | `9.81` | Gravity, in m/s² |
| `TRACTION_MARGIN` | `0.9` | The default `tractionShare` |
| `DEFAULT_FRICTION_COEFFICIENT` | `1.5` | The default `μ` |
| `DEFAULT_CENTER_OF_MASS_HEIGHT_METERS` | `0.2` | The default center of mass height. The source calls it a guess until it is measured |
| `DEFAULT` | | A `RobotConfig` with every default in the table above |

## TractionLimit

A nested enum that says what sets the traction limit.

| Value | Meaning |
| --- | --- |
| `GRIP` | The tread's grip, `μ * g`. Also the answer when the center of mass height is 0 |
| `TIPPING` | Tipping over, `g * x / h` |

## Constructors

| Constructor | Fills in |
| --- | --- |
| All 15 components, in the order of the table | Nothing |
| The first 12, through `maxAngularRadPerSecSquared` | `centerOfMassHeightMeters` 0.2, `tipDistanceMeters` `NaN`, `tractionShare` 0.9 |
| The first 11, through `maxAngularRadPerSec` | Also `maxAngularRadPerSecSquared` `+∞` |

Every constructor throws `IllegalArgumentException` when:

- `massKg`, `momentOfInertiaKgM2`, `moduleRadiusMeters`, `wheelRadiusMeters`, `driveGearRatio`, `statorCurrentLimitAmps`, `frictionCoefficient`, `maxAngularRadPerSec` or `maxAngularRadPerSecSquared` is not above 0,
- `driveMotorCount` is below 1,
- `maxWheelMetersPerSecond` is neither `NaN` nor above 0,
- `centerOfMassHeightMeters` is negative, `NaN` or infinite,
- `tipDistanceMeters` is neither `NaN` nor above 0,
- `tractionShare` is not above 0 and at most 1.

## Changed copies

Each returns a new config with one component replaced. The same checks apply.

| Method | Replaces |
| --- | --- |
| `RobotConfig withFrictionCoefficient(double mu)` | `frictionCoefficient` |
| `RobotConfig withMaxWheelMetersPerSecond(double speed)` | `maxWheelMetersPerSecond`, in m/s |
| `RobotConfig withMaxAngularRadPerSec(double omega)` | `maxAngularRadPerSec`, in rad/s |
| `RobotConfig withMaxAngularRadPerSecSquared(double alpha)` | `maxAngularRadPerSecSquared`, in rad/s² |
| `RobotConfig withCenterOfMassHeightMeters(double h)` | `centerOfMassHeightMeters`, in m |
| `RobotConfig withTipDistanceMeters(double d)` | `tipDistanceMeters`, in m |
| `RobotConfig withTractionShare(double share)` | `tractionShare` |

## Traction

The value column is the result for `RobotConfig.DEFAULT`.

| Method | Returns | Default value |
| --- | --- | --- |
| `double gripMetersPerSecondSquared()` | The tread's grip, `μ * g`, in m/s² | 14.7 |
| `double tipDistanceInUseMeters()` | The tip distance in use: the given one, or `moduleRadiusMeters / √2`, in m | 0.29 |
| `double tipMetersPerSecondSquared()` | The acceleration that would tip the robot, `g * x / h`, in m/s². `+∞` when the center of mass height is 0 | 14.2 |
| `double tractionMetersPerSecondSquared()` | The traction limit: `tractionShare` times the smaller of the two above, in m/s² | 12.8 |
| `TractionLimit tractionLimit()` | Which of the two sets the limit | `TIPPING` |
| `double wheelSlipOnsetMetersPerSecondSquared()` | Where the wheels that weight transfer unloads start to slip, `μ * g / (1 + μ * h / x)`, in m/s², with no margin | 7.2 |
| `double plannedSpinAccelRadPerSecSquared()` | The spin acceleration the plan uses: `tractionShare * maxAngularRadPerSecSquared`, in rad/s² | `+∞` |
| `double spinLeverMeters()` | `I / (m * r)`, in m. Spinning up at `α` needs each wheel to push like a straight acceleration of this times `α` | 0.24 |

`tractionMetersPerSecondSquared()` is the limit the planner uses for speeding up, braking, cornering and spinning combined. The follower's output limiter is derived from it as well.

## Motors and wheel speed

| Method | Returns | Default value |
| --- | --- | --- |
| `double freeSpeedMetersPerSecond()` | The wheel speed at the motor's free speed, in m/s | 4.56 |
| `double wheelSpeedCapMetersPerSecond()` | The wheel-speed cap in use: `maxWheelMetersPerSecond`, or the free speed when that is `NaN`, in m/s | 4.56 |
| `double driveAccelMetersPerSecondSquared(double wheelMetersPerSecond)` | The most acceleration the drive motors can add with the wheels rolling at that speed, in m/s² | 20.9 at 0 m/s |
| `double brakeAccelMetersPerSecondSquared()` | The most braking the drive motors can give, in m/s² | 20.9 |

`driveAccelMetersPerSecondSquared` uses full voltage minus back-EMF, with the current clamped between 0 and the stator limit. It returns 0 at and above the free speed. When braking, back-EMF helps, so `brakeAccelMetersPerSecondSquared` uses the stator limit alone.

:::caution
The defaults describe the bench robot the library was developed against. They are not tuned values for your robot. Measure your own and put them in robot.json.
:::
