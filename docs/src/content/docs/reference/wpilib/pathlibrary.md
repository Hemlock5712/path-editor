---
title: PathLibrary
description: Loads and builds every path in deploy/paths at robot init, with the field, named points and event bindings.
sidebar:
  order: 1
---

`PathLibrary` reads the `deploy/paths` folder once, when the robot program starts, and builds everything in it: every path, the robot model, the field's obstacles, the editor's named points. After that, starting a path does no path work. The command follows the object that was built here, on either alliance.

You meet it in the first lines of robot code. Every other class in this package takes a `PathLibrary`, or something that came out of one. The folder it reads is described in [Project files](/start/project-files/).

## Example

```java
import static frc.robot.PathEditorNames.*;   // written by the editor on every save

// Robot constructor
PathLibrary paths = PathLibrary.loadAll();

paths.bindEvent(Events.INTAKE_DOWN, intake.lower())
    .bindEvent(Events.SHOOT, shooter.shoot())
    .requireAllEventsBound();

Path toReef = paths.get(Paths.TO_REEF);
```

`intake.lower()` and `shooter.shoot()` are commands from your own robot code. `Paths.TO_REEF` and `Events.SHOOT` come from the generated [`PathEditorNames`](/files/generated-names/) class. Every method that takes one of those constants also takes a plain `String`, so `paths.get("ToReef")` works too, but a misspelled constant is a compile error and a misspelled string is found only at boot.

## What loadAll reads

`loadAll()` reads the `paths` folder inside the deploy directory. `loadAll(File dir)` does the same for any folder. The steps run in this order, and the first one that fails stops the load.

| Step | File | What happens | If the file is absent |
| --- | --- | --- | --- |
| 1 | `robot.json` | Read as the robot model, a [`RobotConfig`](/reference/core/robotconfig/). | `RobotConfig.DEFAULT` is used. |
| 2 | every `*.path.json` | Each is read and built into a [`Path`](/reference/core/path/). The path's name is the file name without `.path.json`. | The library has no paths. |
| 3 | `robot.json` again | The bumper size, the on-the-fly speed limits and the [footprints](/concepts/footprints/) are read. | 0.85 m square bumpers, 4 m/s and 6 m/s², bumpers only. |
| 4 | the paths | Every footprint a path starts with, or an event switches to, must be one that `robot.json` names. | |
| 5 | `field.json` | The obstacles are read and the [pathfinders](/concepts/pathfinder/) are built, then warmed up with 40 throwaway plans each. | An open floor, 1000 m across, with no walls and no obstacles. |
| 6 | `.pathproject.json` | The editor's named points and the Java package of the generated names are read. | No named points. |
| 7 | `PathEditorNames` | If that class is compiled into the robot program, every constant is checked against what was loaded. | No check. |

A named point from a project file older than schema 2 is turned half a turn, `(x, y)` to `(-x, -y)`, because older projects put +X toward the red wall. See [The field frame](/concepts/field-frame/).

:::caution
A `deploy/paths` folder that does not exist is not an error. The library loads with no paths, and the first `get` throws. If every path is "missing", check that the folder was deployed.
:::

## Boot failures

A bad file stops the robot program at startup, with a message that names the file. That is on purpose: a path that cannot be built should be found in the pit, not when autonomous starts.

| Exception | Message starts with | Meaning |
| --- | --- | --- |
| `IllegalStateException` | `path <file>:` | That path file could not be read or built. The rest of the message is the reason. |
| `IllegalStateException` | `robot.json has no such footprint:` | A path starts with a footprint, or an event switches to one, that `robot.json` does not list. The message names each path and event, and lists the footprints that exist. |
| `IllegalStateException` | `field <file>:` | `field.json` is not valid. The reason follows. |
| `IllegalStateException` | `named points in <file>:` | `.pathproject.json` could not be read. |
| `IllegalStateException` | `<class> doesn't match deploy/paths:` | The generated names are stale. Each missing constant is listed with what it wanted: a path file, an event, or a named point. Save the project in the editor, then build and deploy again. |
| `IllegalArgumentException` | varies | `robot.json` has a bad footprint (no name, a repeated name, a negative size) or a bumper size that is not positive. |
| `UncheckedIOException` | | A file exists but could not be read from disk. |

An invalid `robot.json` also fails at step 1, with the error the robot file reader gives. See [The robot file](/files/robot-file/).

## Loading

| Method | Returns | Notes |
| --- | --- | --- |
| `static loadAll()` | `PathLibrary` | Everything in the table above, from the deploy directory's `paths` folder. |
| `static loadAll(File dir)` | `PathLibrary` | The same, from `dir`. |
| `static loadAll(File dir, RobotConfig robot)` | `PathLibrary` | Step 2 only: builds the path files in `dir` with the robot you pass. It reads no `robot.json`, no `field.json`, no named points, and does no names check. |
| `loadField(File file, PlannerConfig planner)` | this library | Reads `file` as a field file, or uses the open floor if it does not exist, and builds the pathfinders for the planner's bumpers. Throws `IllegalStateException` for a bad file. |
| `loadField(File file, PlannerConfig planner, List<Footprint> footprints)` | this library | The same, with a pathfinder pair for each footprint as well. |
| `requireNames(Class<?> names)` | this library | Runs the names check by hand against any class whose nested enums implement [`PathId`, `EventId` or `PointId`](/reference/wpilib/name-ids/). Throws `IllegalStateException` listing the stale constants. |

Call all of these in robot init. `loadAll()` already calls `loadField` and `requireNames` for you, so most robots never call them directly.

## Paths and the robot

| Method | Returns | Notes |
| --- | --- | --- |
| `get(PathId id)` | [`Path`](/reference/core/path/) | The path the constant names. |
| `get(String name)` | `Path` | The path built from `<name>.path.json`. Throws `NoSuchElementException`, listing the names that exist, if there is none. |
| `all()` | `Map<String, Path>` | Every path by name, sorted by name. The map cannot be changed. |
| `robot()` | [`RobotConfig`](/reference/core/robotconfig/) | The robot model the paths were built with. Pass it to `FollowerConfig.defaults(paths.robot())`. |

Paths are stored in the blue frame, as they were drawn. Nothing is rebuilt for red.

## Events

An event is a name placed on a path in the editor. Binding a command to the name here covers every path that uses it. A follow command picks the bindings up with `withEvents(paths)`, and commands built by [`Pathfinding`](/reference/wpilib/pathfinding/) do that for you. See [Events](/guides/events/).

| Method | Returns | Notes |
| --- | --- | --- |
| `bindEvent(EventId event, Command command)` | this library | Runs `command` whenever a path fires this event. Binding the same name again replaces the earlier command. |
| `bindEvent(String eventName, Command command)` | this library | The same, by the event's name as the path files spell it. |
| `eventCommand(String eventName)` | `Optional<Command>` | The command bound to that name, or empty. |
| `eventNames()` | `Set<String>` | Every event name the loaded path files use, sorted. |
| `unboundEventNames()` | `Set<String>` | The event names with no command bound, sorted. |
| `requireAllEventsBound()` | this library | Throws `IllegalStateException` listing the unbound names, if there are any. Call it after the last `bindEvent`, in robot init. |

:::tip
End the chain of `bindEvent` calls with `requireAllEventsBound()`. Without it, an event with no command does nothing and says nothing.
:::

## Named points

Named points are spots saved in the editor, such as a scoring position. They are in the blue frame: the origin at the field center, +X toward the blue alliance wall, in meters.

| Method | Returns | Notes |
| --- | --- | --- |
| `namedPoint(PointId point)` | `double[]` | `{x, y}` in meters, blue frame. |
| `namedPoint(String name)` | `double[]` | The same, by the name the editor shows. Throws `NoSuchElementException`, listing the points that exist, if there is none. |
| `namedPoints()` | `Map<String, double[]>` | Every named point by name. Empty when there is no project file. |

`otf.toPoint(drive, Points.DEPOT, heading)` on [`Pathfinding`](/reference/wpilib/pathfinding/) drives to one.

## Pathfinders and footprints

The library builds two [`Pathfinder`](/reference/plan/pathfinder/) objects for the bumpers: one with the planner's own margin, 0.05 m by default, and one with a wider 0.15 m margin that on-the-fly paths try first. If `robot.json` lists footprints, each footprint gets its own pair. Robot code rarely calls these methods. The on-the-fly commands use them.

| Method | Returns | Notes |
| --- | --- | --- |
| `pathfinder()` | `Pathfinder` | The bumpers' pathfinder. On a library that never loaded a field, this builds one for the open floor with default settings. |
| `widePathfinder()` | `Pathfinder` | The same field with the 0.15 m margin, or `null` if the main margin is already that wide. |
| `pathfinder(String footprint)` | `Pathfinder` | The pathfinder that plans with that footprint's outline. The name is one from `robot.json`, or `"bumpers"`. Throws `NoSuchElementException`, listing the names that exist, for an unknown one. |
| `widePathfinder(String footprint)` | `Pathfinder` | The wide-margin partner of the above, or `null`. Throws the same way. |
| `footprintNames()` | `Set<String>` | The footprints there are pathfinders for, the bumpers first. |
| `hasFieldFile()` | `boolean` | `true` when the obstacles came from a `field.json`, `false` for the open floor. |

## Timing

All three are in milliseconds, measured once during the load.

| Method | What it measures |
| --- | --- |
| `buildMillis()` | Reading and building the path files. |
| `fieldMillis()` | Everything after the paths: the planner settings, the field file, building and warming up the pathfinders, and the named points. It is 0 for a library made with `loadAll(dir, robot)`. |
| `warmUpMillis()` | The warm-up plans alone. It is part of `fieldMillis()`. |

## Constants

| Constant | Value | Meaning |
| --- | --- | --- |
| `SUFFIX` | `".path.json"` | The ending that marks a path file. |
| `FIELD_FILE` | `"field.json"` | The field file's name. |
| `PROJECT_FILE` | `".pathproject.json"` | The project file's name. |
| `PROJECT_SCHEMA` | `2` | The project schema whose named points are already in the 2027 frame. |
| `NAMES_CLASS` | `"PathEditorNames"` | The name of the class the editor generates. |
| `OPEN_FLOOR_METERS` | `1000.0` | The side of the open floor used when there is no `field.json`, in meters. |
| `DEFAULT_FIELD_LENGTH` | `16.54` | The 2026 field's length in meters. The loader does not use it. |
| `DEFAULT_FIELD_WIDTH` | `8.07` | The 2026 field's width in meters. The loader does not use it. |

:::note
With no `field.json` the library knows no walls. A robot being tested on a shop floor drives straight to its goal from wherever its pose was zeroed. On a real field, deploy the field file so that on-the-fly paths stay inside the walls and around the obstacles.
:::
