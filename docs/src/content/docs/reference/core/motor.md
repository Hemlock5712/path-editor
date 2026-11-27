---
title: Motor
description: A brushless motor's straight-line torque and speed curve.
sidebar:
  order: 16
---

`Motor` is a record that models a brushless DC motor with four numbers, which the speed plan uses to find how hard the drive motors can push at each wheel speed.

It is the `driveMotor` component of [RobotConfig](/reference/core/robotconfig/), and you change it through the `motor` object in [robot.json](/files/robot-file/). If your robot drives on Kraken X60 motors with FOC, the default already matches and there is nothing to set.

Package: `frc.team5712.path.core`.

## Example

```java
Motor motor = Motor.KRAKEN_X60_FOC;

double ohms = motor.resistanceOhms();
// Amps drawn at half the free speed, at the nominal voltage
double ampsAtHalfSpeed = motor.currentAt(motor.freeSpeedRadPerSec() / 2.0, motor.nominalVolts());
```

## Constructors

| Constructor | What it does |
| --- | --- |
| `Motor(double nominalVolts, double stallTorqueNewtonMeters, double stallCurrentAmps, double freeSpeedRadPerSec)` | The record constructor. It does not check its arguments. |

## Components

| Component | What it is |
| --- | --- |
| `double nominalVolts` | The voltage the curve was measured at, in V |
| `double stallTorqueNewtonMeters` | Torque at zero speed, in N·m |
| `double stallCurrentAmps` | Current at zero speed, in A |
| `double freeSpeedRadPerSec` | Speed with no load, in rad/s |

## Constants

| Constant | What it is |
| --- | --- |
| `KRAKEN_X60_FOC` | 12 V, 9.362 N·m stall torque, 476.1 A stall current, 5,785 rpm free speed, which is 605.8 rad/s. The source gives CTRE's dyno data as the origin of these numbers |

## Methods

| Method | What it does |
| --- | --- |
| `double resistanceOhms()` | Winding resistance, in ohms: `nominalVolts / stallCurrentAmps`. About 0.0252 for the Kraken X60 default. |
| `double torquePerAmp()` | Torque per amp, in N·m/A: `stallTorqueNewtonMeters / stallCurrentAmps`. About 0.0197 for the default. |
| `double radPerSecPerVolt()` | The back-EMF constant, in rad/s per volt: `freeSpeedRadPerSec / nominalVolts`. About 50.5 for the default. |
| `double currentAt(double motorRadPerSec, double volts)` | The current drawn at `volts` while turning at `motorRadPerSec`, in A: `(volts - motorRadPerSec / radPerSecPerVolt()) / resistanceOhms()`. |

## Good to know

- **The model is a straight line**: full torque at zero speed, falling evenly to no torque at the free speed.
- **`currentAt` takes the motor's own speed**, not the wheel's.
- **`currentAt` does not clamp its answer.** Above the free speed for that voltage the result is negative. `RobotConfig.driveAccelMetersPerSecondSquared` clamps it between 0 and the stator current limit.

## More

- [Traction](/concepts/traction/): where the motor curve fits in the speed plan.
- [RobotConfig](/reference/core/robotconfig/): the `driveMotor` component.
- [robot.json](/files/robot-file/): the `motor` object.
