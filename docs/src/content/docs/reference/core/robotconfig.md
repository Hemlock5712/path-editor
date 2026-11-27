---
title: RobotConfig
description: The robot numbers that every path is planned with and every command is limited by.
sidebar:
  order: 14
---

`RobotConfig` is a record of the robot's physical numbers: mass, inertia, where the modules are, the drive motors and gearing, the tread's grip, and the center of mass.

You set these numbers in [robot.json](/files/robot-file/), not in code. `PathLibrary.loadAll()` reads the file, `paths.robot()` returns the result, and you pass that to settings such as `FollowerConfig.defaults(paths.robot())`. See [Project files](/start/project-files/).

Package: `frc.team5712.path.core`.

## Example

```java
RobotConfig robot = paths.robot();

double limit = robot.tractionMetersPerSecondSquared();       // m/s²
RobotConfig.TractionLimit why = robot.tractionLimit();        // GRIP or TIPPING
double topWheelSpeed = robot.wheelSpeedCapMetersPerSecond();  // m/s

RobotConfig slippery = robot.withFrictionCoefficient(1.0);    // a changed copy
```

## Components

The default is the value in `RobotConfig.DEFAULT`.

| Component | Default | What it is |
| --- | --- | --- |
| `double massKg` | 60 | Robot mass with battery and bumpers, in kg |
| `double momentOfInertiaKgM2` | 6 | Moment of inertia about the vertical axis, in kg·m² |
| `double moduleRadiusMeters` | 0.41012 | Distance from the robot's center to a swerve module, in m |
| `double wheelRadiusMeters` | 0.0508 | Drive wheel radius, in m |
| `double driveGearRatio` | 6.75 | Motor turns per wheel turn |
| `Motor driveMotor` | `Motor.KRAKEN_X60_FOC` | The drive motor model |
| `int driveMotorCount` | 4 | Number of drive motors |
| `double statorCurrentLimitAmps` | 120 | Drive stator current limit, in A |
| `double frictionCoefficient` | 1.5 | The tread's friction coefficient `μ` |
| `double maxWheelMetersPerSecond` | `NaN` | The fastest any wheel may be asked to roll, in m/s. `NaN` means the motor's free speed |
| `double maxAngularRadPerSec` | `+∞` | Spin cap for the planned heading, in rad/s. `+∞` means only the wheel-speed cap |
| `double maxAngularRadPerSecSquared` | `+∞` | Spin acceleration cap, in rad/s². `+∞` means only grip and motors |
| `double centerOfMassHeightMeters` | 0.2 | Height of the center of mass above the floor, in m. 0 means no tipping limit |
| `double tipDistanceMeters` | `NaN` | Distance from the center to the nearest tipping edge, in m. `NaN` means a square base, `moduleRadiusMeters / √2` |
| `double tractionShare` | 0.9 | The share of grip or tipping that the robot plans and drives to |

## Constructors

| Constructor | What it fills in |
| --- | --- |
| All 15 components, in the order of the table | Nothing |
| The first 12, through `maxAngularRadPerSecSquared` | `centerOfMassHeightMeters` 0.2, `tipDistanceMeters` `NaN`, `tractionShare` 0.9 |
| The first 11, through `maxAngularRadPerSec` | Also `maxAngularRadPerSecSquared` `+∞` |

Every constructor throws `IllegalArgumentException` when:

| Component | Is wrong when |
| --- | --- |
| `massKg`, `momentOfInertiaKgM2`, `moduleRadiusMeters`, `wheelRadiusMeters`, `driveGearRatio`, `statorCurrentLimitAmps`, `frictionCoefficient`, `maxAngularRadPerSec`, `maxAngularRadPerSecSquared` | Not above 0 |
| `driveMotorCount` | Below 1 |
| `maxWheelMetersPerSecond` | Neither `NaN` nor above 0 |
| `centerOfMassHeightMeters` | Negative, `NaN` or infinite |
| `tipDistanceMeters` | Neither `NaN` nor above 0 |
| `tractionShare` | Not above 0 and at most 1 |

## Constants

| Constant | What it is |
| --- | --- |
| `GRAVITY` | `9.81`. Gravity, in m/s² |
| `TRACTION_MARGIN` | `0.9`. The default `tractionShare` |
| `DEFAULT_FRICTION_COEFFICIENT` | `1.5`. The default `μ` |
| `DEFAULT_CENTER_OF_MASS_HEIGHT_METERS` | `0.2`. The default center of mass height. The source calls it a guess until it is measured |
| `DEFAULT` | A `RobotConfig` with every default in the components table |

## TractionLimit

A nested enum that says what sets the traction limit.

| Value | What it means |
| --- | --- |
| `GRIP` | The tread's grip, `μ * g`. Also the answer when the center of mass height is 0 |
| `TIPPING` | Tipping over, `g * x / h` |

## Changed copies

Each returns a new config with one component replaced. The constructor's checks apply.

| Method | What it replaces |
| --- | --- |
| `RobotConfig withFrictionCoefficient(double mu)` | `frictionCoefficient` |
| `RobotConfig withMaxWheelMetersPerSecond(double speed)` | `maxWheelMetersPerSecond`, in m/s |
| `RobotConfig withMaxAngularRadPerSec(double omega)` | `maxAngularRadPerSec`, in rad/s |
| `RobotConfig withMaxAngularRadPerSecSquared(double alpha)` | `maxAngularRadPerSecSquared`, in rad/s² |
| `RobotConfig withCenterOfMassHeightMeters(double h)` | `centerOfMassHeightMeters`, in m |
| `RobotConfig withTipDistanceMeters(double d)` | `tipDistanceMeters`, in m |
| `RobotConfig withTractionShare(double share)` | `tractionShare` |

## Traction

The last column is the result for `RobotConfig.DEFAULT`.

| Method | What it returns | Default value |
| --- | --- | --- |
| `double gripMetersPerSecondSquared()` | The tread's grip, `μ * g`, in m/s² | 14.7 |
| `double tipDistanceInUseMeters()` | The tip distance in use: the given one, or `moduleRadiusMeters / √2`, in m | 0.29 |
| `double tipMetersPerSecondSquared()` | The acceleration that would tip the robot, `g * x / h`, in m/s². `+∞` when the center of mass height is 0 | 14.2 |
| `double tractionMetersPerSecondSquared()` | The traction limit: `tractionShare` times the smaller of the two above, in m/s² | 12.8 |
| `TractionLimit tractionLimit()` | Which of the two sets the limit | `TIPPING` |
| `double wheelSlipOnsetMetersPerSecondSquared()` | Where the wheels that weight transfer unloads start to slip, `μ * g / (1 + μ * h / x)`, in m/s², with no margin | 7.2 |
| `double plannedSpinAccelRadPerSecSquared()` | The spin acceleration the plan uses: `tractionShare * maxAngularRadPerSecSquared`, in rad/s² | `+∞` |
| `double spinLeverMeters()` | `I / (m * r)`, in m. Spinning up at `α` needs each wheel to push like a straight acceleration of this times `α` | 0.24 |

## Motors and wheel speed

| Method | What it returns | Default value |
| --- | --- | --- |
| `double freeSpeedMetersPerSecond()` | The wheel speed at the motor's free speed, in m/s | 4.56 |
| `double wheelSpeedCapMetersPerSecond()` | The wheel-speed cap in use: `maxWheelMetersPerSecond`, or the free speed when that is `NaN`, in m/s | 4.56 |
| `double driveAccelMetersPerSecondSquared(double wheelMetersPerSecond)` | The most acceleration the drive motors can add with the wheels rolling at that speed, in m/s². Full voltage minus back-EMF, with the current clamped between 0 and the stator limit. 0 at and above the free speed | 20.9 at 0 m/s |
| `double brakeAccelMetersPerSecondSquared()` | The most braking the drive motors can give, in m/s². Back-EMF helps when braking, so this uses the stator limit alone | 20.9 |

## Good to know

- **Who reads it.** The speed plan reads it to find how fast a path may be driven, and the follower's limiter reads it too.
- **One limit for everything.** `tractionMetersPerSecondSquared()` is `tractionShare * min(μ * g, g * x / h)`, and the planner uses it for speeding up, braking, cornering and spinning combined. The follower's output limiter is derived from it as well.
- **Feedback has room on spin.** The plan turns at `tractionShare` of `maxAngularRadPerSecSquared`, and the follower is held to the full value.

:::caution
The defaults describe the bench robot the library was developed against. They are not tuned values for your robot. Measure your own and put them in robot.json.
:::

## More

- [Traction](/concepts/traction/): where the traction formula comes from.
- [robot.json](/files/robot-file/): the file these numbers are set in.
- [Project files](/start/project-files/): where the file goes.
