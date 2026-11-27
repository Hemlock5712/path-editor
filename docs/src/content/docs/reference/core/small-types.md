---
title: Pose, speeds and vectors
description: The small value types of the core package, and its angle helpers.
sidebar:
  order: 21
---

`Pose`, `FieldSpeeds` and `Vec2` are the core package's own records for a pose, a velocity and a 2D vector, and `Angles` is a class of static angle helpers.

Robot code works in WPILib types, `Pose2d` and `ChassisVelocities`, and the library's commands convert for you. You meet the core types when you read a plan directly, for example `PathSample.pose()`, or when you call `FieldFrame` yourself.

Package: `frc.team5712.path.core`.

## Example

```java
Pose pose = new Pose(6.0, 2.0, Math.PI);
Pose2d forWpilib = Conversions.toPose2d(pose);

// From the robot to the center of the field
Vec2 toGoal = new Vec2(0.0, 0.0).minus(new Vec2(pose.xMeters(), pose.yMeters()));
double distance = toGoal.norm();                                    // meters
// How far the robot must turn to face it, the short way
double error = Angles.wrap(Angles.atan2(toGoal.y(), toGoal.x()) - pose.headingRadians());
```

## Pose

A field pose: where the robot is and which way it faces. There are no methods beyond the accessors.

| Member | What it is |
| --- | --- |
| `Pose(double xMeters, double yMeters, double headingRadians)` | The constructor. It does not check or wrap its arguments |
| `double xMeters` | Field x, in m |
| `double yMeters` | Field y, in m |
| `double headingRadians` | The robot's heading, in rad |

## FieldSpeeds

A field-relative velocity. There are no methods beyond the accessors.

| Member | What it is |
| --- | --- |
| `FieldSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadPerSec)` | The constructor. It does not check its arguments |
| `double vxMetersPerSecond` | Speed along field x, in m/s |
| `double vyMetersPerSecond` | Speed along field y, in m/s |
| `double omegaRadPerSec` | Spin, in rad/s. Counterclockwise is positive |

## Vec2

A point or a vector on the field, in meters. It is immutable, and every method returns a new value.

| Member | What it is |
| --- | --- |
| `Vec2(double x, double y)` | The constructor |
| `double x` | The x part, in m |
| `double y` | The y part, in m |
| `static final Vec2 ZERO` | The vector `(0, 0)`. |
| `Vec2 plus(Vec2 o)` | The sum of this vector and `o`. |
| `Vec2 minus(Vec2 o)` | This vector minus `o`. |
| `Vec2 times(double k)` | This vector with both parts multiplied by `k`. |
| `double dot(Vec2 o)` | The dot product, `x * o.x + y * o.y`. |
| `double cross(Vec2 o)` | The z part of the 3D cross product, `x * o.y - y * o.x`. Positive when `o` is to the left of this vector. |
| `double norm()` | The length, in meters. |
| `Vec2 left()` | This vector turned 90 degrees to the left: `(-y, x)`. |

## Angles

Static helpers for angles, in radians. The class cannot be constructed.

| Method | What it does |
| --- | --- |
| `static double wrap(double radians)` | The same angle in the range above -π and up to π. |
| `static double atan2(double y, double x)` | The direction of the vector `(x, y)`, in radians. Note the argument order: `y` first. |
| `static double cos(double radians)` | The cosine. |
| `static double sin(double radians)` | The sine. |
| `static double smoothstep5(double tau)` | The minimum-jerk blend `10τ³ - 15τ⁴ + 6τ⁵`. It goes from 0 at `tau = 0` to 1 at `tau = 1`, with zero slope and zero bend at both ends. |
| `static double smoothstep5Slope(double tau)` | The first derivative of `smoothstep5`: `30τ²(1 - τ)²`. |
| `static double smoothstep5Bend(double tau)` | The second derivative of `smoothstep5`: `60τ(1 - τ)(1 - 2τ)`. |

## Good to know

- **Use `wrap` when you subtract two headings.** Going from 170 degrees to -170 degrees is a turn of 20 degrees the short way. Plain subtraction gives -340 degrees, and `wrap` turns that into 20.
- **The three `smoothstep5` methods do not clamp `tau`.** They are meant for values from 0 to 1.
- **The core does not depend on WPILib**, which is why it has its own types.

:::note
`atan2`, `cos` and `sin` use Java's `StrictMath`, and `Vec2.norm` is written as a plain square root. This is so that the editor, which runs the same math in TypeScript, gets the same bits as the robot. Use these helpers for the same reason if you extend the library's math.
:::

## More

- [Conversions](/reference/wpilib/conversions/): converts between these and the WPILib types.
- [FieldFrame](/reference/core/fieldframe/): converts these between red and blue.
