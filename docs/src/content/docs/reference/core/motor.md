---
title: Motor
description: A brushless motor's straight-line torque and speed curve.
sidebar:
  order: 16
---

`Motor` is a record that models a brushless DC motor with four numbers. The model is a straight line: full torque at zero speed, falling evenly to no torque at the free speed. The speed plan uses it to find how hard the drive motors can push at each wheel speed. [Traction](/concepts/traction/) shows where that fits.

Package: `frc.team5712.path.core`.

## Where you meet it

A `Motor` is the `driveMotor` component of [RobotConfig](/reference/core/robotconfig/). You change it through the `motor` object in [robot.json](/files/robot-file/). If your robot drives on Kraken X60 motors with FOC, the default already matches and there is nothing to set.

## Example

```java
Motor motor = Motor.KRAKEN_X60_FOC;

double ohms = motor.resistanceOhms();
double ampsAtHalfSpeed = motor.currentAt(motor.freeSpeedRadPerSec() / 2.0, motor.nominalVolts());
```

## Components

| Component | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `nominalVolts` | `double` | V | The voltage the curve was measured at |
| `stallTorqueNewtonMeters` | `double` | N·m | Torque at zero speed |
| `stallCurrentAmps` | `double` | A | Current at zero speed |
| `freeSpeedRadPerSec` | `double` | rad/s | Speed with no load |

## Constructor

| Constructor | Description |
| --- | --- |
| `Motor(double nominalVolts, double stallTorqueNewtonMeters, double stallCurrentAmps, double freeSpeedRadPerSec)` | The record constructor. It does not check its arguments. |

## Constants

| Constant | Value |
| --- | --- |
| `KRAKEN_X60_FOC` | 12 V, 9.362 N·m stall torque, 476.1 A stall current, 5,785 rpm free speed, which is 605.8 rad/s |

The source gives CTRE's dyno data as the origin of these numbers.

## Methods

| Method | Returns | Formula |
| --- | --- | --- |
| `double resistanceOhms()` | Winding resistance, in ohms | `nominalVolts / stallCurrentAmps` |
| `double torquePerAmp()` | Torque per amp, in N·m/A | `stallTorqueNewtonMeters / stallCurrentAmps` |
| `double radPerSecPerVolt()` | The back-EMF constant, in rad/s per volt | `freeSpeedRadPerSec / nominalVolts` |
| `double currentAt(double motorRadPerSec, double volts)` | The current drawn at `volts` while turning at `motorRadPerSec`, in A | `(volts - motorRadPerSec / radPerSecPerVolt()) / resistanceOhms()` |

`currentAt` takes the motor's own speed, not the wheel's. It does not clamp its answer. Above the free speed for that voltage the result is negative. `RobotConfig.driveAccelMetersPerSecondSquared` clamps it between 0 and the stator current limit.

For the Kraken X60 default the methods give a resistance of about 0.0252 ohms, about 0.0197 N·m per amp, and about 50.5 rad/s per volt.
