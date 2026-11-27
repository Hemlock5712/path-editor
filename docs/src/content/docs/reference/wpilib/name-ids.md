---
title: PathId, EventId, PointId
description: The three small interfaces that the editor's generated name enums implement.
sidebar:
  order: 12
---

`PathId`, `EventId` and `PointId` let robot code name a path, an event or a named point with a constant instead of a string. Each is an interface with one method that returns the name.

You do not implement them. The editor writes a class called `PathEditorNames` into your robot project every time you save, and its three enums implement them. You meet the interfaces as the parameter types of the library's methods: wherever a method takes a name as a `String`, it also takes one of these. [Generated names](/files/generated-names/) describes the generated file.

## Example

```java
import static frc.robot.PathEditorNames.*;

Path toReef = paths.get(Paths.TO_REEF);                    // a PathId
paths.bindEvent(Events.INTAKE_DOWN, intake.lower());       // an EventId
Command toDepot = otf.toPoint(drive, Points.DEPOT, Rotation2d.PI);   // a PointId
```

`frc.robot` is the default package. The editor's project settings can change it. `intake.lower()` is a command from your own code. The string forms do the same thing: `paths.get("ToReef")`.

## Why use the constants

A string is checked when the robot program runs. A constant is checked when it compiles.

- If you misspell `Paths.TO_REEF`, the code does not build.
- If you rename or delete a path in the editor, the constant changes or disappears, and every line that used it stops building until you fix it.
- If you misspell `"ToReef"`, the code builds, and the robot program throws `NoSuchElementException` when it reaches that line.

There is a second check at boot. When [`PathLibrary.loadAll()`](/reference/wpilib/pathlibrary/) finds the generated class in the robot program, it compares every constant with the files in `deploy/paths`. If they do not match, because the paths were deployed without rebuilding the code or the other way around, it throws `IllegalStateException` and lists the stale constants.

## The interfaces

| Interface | Method | Returns | Generated enum |
| --- | --- | --- | --- |
| `PathId` | `fileName()` | The path file's name without `.path.json`. | `PathEditorNames.Paths` |
| `EventId` | `eventName()` | The event's name as the path files spell it. | `PathEditorNames.Events` |
| `PointId` | `pointName()` | The named point's name as the editor shows it. | `PathEditorNames.Points` |

The enum constant and the name it carries are not the same text. In the library's own test copy of the generated file, `Paths.S_CURVE` carries the file name `SCurve`, `Events.INTAKE_DOWN` carries `intakeDown`, and `Points.SCORE_L` carries `Score L`.

## Where each is accepted

| Type | Methods |
| --- | --- |
| `PathId` | `PathLibrary.get`, `Pathfinding.follow`, `Pathfinding.thenFollow`, the first argument of the `FollowPathCommand` constructor |
| `EventId` | `PathLibrary.bindEvent`, `FollowPathCommand.onEvent` |
| `PointId` | `PathLibrary.namedPoint`, `Pathfinding.toPoint` |

See [`PathLibrary`](/reference/wpilib/pathlibrary/), [`Pathfinding`](/reference/wpilib/pathfinding/) and [`FollowPathCommand`](/reference/wpilib/followpathcommand/).

## Checking your own enums

The boot check does not need the class to be the generated one. `paths.requireNames(SomeClass.class)` checks any class whose nested enums implement these interfaces: every `PathId` constant must be a loaded path file, every `EventId` constant must be an event some path uses, and every `PointId` constant must be a named point.

:::note
`PathEditorNames.java` is rewritten on every save in the editor. Do not edit it by hand. Changes are lost on the next save.
:::
