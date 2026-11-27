---
title: PathId, EventId, PointId
description: The three small interfaces that the editor's generated name enums implement.
sidebar:
  order: 12
---

`PathId`, `EventId` and `PointId` let robot code name a path, an event or a named point with a constant instead of a string.

You do not implement them: the editor writes `PathEditorNames` into your robot project on every save, and its three enums do. [Generated names](/files/generated-names/) describes that file.

## Example

```java
import static frc.robot.PathEditorNames.*;   // frc.robot is the default package

Path toReef = paths.get(Paths.TO_REEF);                    // a PathId
paths.bindEvent(Events.INTAKE_DOWN, intake.lower());       // an EventId
Command toDepot = otf.toPoint(drive, Points.DEPOT, Rotation2d.PI);   // a PointId

// The string form does the same thing, but is only checked when the robot program runs
Path same = paths.get("ToReef");
```

## The interfaces

| Interface | Method | Returns | Generated enum |
| --- | --- | --- | --- |
| `PathId` | `fileName()` | The path file's name without `.path.json`. | `PathEditorNames.Paths` |
| `EventId` | `eventName()` | The event's name as the path files spell it. | `PathEditorNames.Events` |
| `PointId` | `pointName()` | The named point's name as the editor shows it. | `PathEditorNames.Points` |

## Where each is accepted

| Type | Methods |
| --- | --- |
| `PathId` | `PathLibrary.get`, `Pathfinding.follow`, `Pathfinding.thenFollow`, the first argument of the `FollowPathCommand` constructor |
| `EventId` | `PathLibrary.bindEvent`, `FollowPathCommand.onEvent` |
| `PointId` | `PathLibrary.namedPoint`, `Pathfinding.toPoint` |

## What catches a wrong name

| Mistake | What happens |
| --- | --- |
| A misspelled constant, such as `Paths.TO_REEF` | The code does not build. |
| A path renamed or deleted in the editor | The constant changes or disappears, and every line that used it stops building. |
| A misspelled string, such as `"ToReef"` | The code builds, and the robot program throws `NoSuchElementException` when it reaches that line. |
| Paths deployed without rebuilding the code, or the other way around | [`PathLibrary.loadAll()`](/reference/wpilib/pathlibrary/) throws `IllegalStateException` at boot and lists the stale constants. |

## Good to know

- **The constant and its name are not the same text.** In the library's test copy of the generated file, `Paths.S_CURVE` carries `SCurve`, `Events.INTAKE_DOWN` carries `intakeDown`, and `Points.SCORE_L` carries `Score L`.
- **The boot check works on your own enums too.** `paths.requireNames(SomeClass.class)` checks any class whose nested enums implement these interfaces: every `PathId` must be a loaded path file, every `EventId` an event some path uses, and every `PointId` a named point.
- **The package can change.** The editor's project settings set it.

:::note
`PathEditorNames.java` is rewritten on every save in the editor. Do not edit it by hand.
:::

## More

- [`PathLibrary`](/reference/wpilib/pathlibrary/), [`Pathfinding`](/reference/wpilib/pathfinding/) and [`FollowPathCommand`](/reference/wpilib/followpathcommand/): the methods that take these types.
