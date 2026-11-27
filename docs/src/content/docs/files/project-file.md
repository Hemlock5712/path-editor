---
title: Project file
description: The keys of .pathproject.json that the library reads, and which ones belong to the editor.
sidebar:
  order: 4
---

`.pathproject.json` is the path editor's own file. It holds what the editor needs to reopen your project the way you left it. The editor at https://paths.frc5712.com writes it to `src/main/deploy/paths/.pathproject.json` on every save.

The library reads three things from it and ignores the rest. This page documents those three. You should not need to edit this file by hand: change named points and the Java package in the editor and save.

## What the library reads

`PathLibrary.loadAll()` reads the file once at boot.

| Key | Type | Unit | Default | What the library does with it |
| --- | --- | --- | --- | --- |
| `schema` | number | | 1 when absent | Decides which field frame the named points are in. `2` is the current one. |
| `namedPoints` | list | | no named points | The points you can plan to by name |
| `namedPoints[].name` | string | | | The point's name, as the editor shows it |
| `namedPoints[].x` | number | meters | | The point's X, in the blue frame |
| `namedPoints[].y` | number | meters | | The point's Y, in the blue frame |
| `settings.javaPackage` | string | | none | The Java package to look for `PathEditorNames` in |

### Named points

A **named point** is a spot on the field you gave a name in the editor, such as a scoring position. The library loads each one's name and position so robot code can plan to it:

```java
Command toDepot = otf.toPoint(drive, Points.DEPOT, Rotation2d.PI);
```

`paths.namedPoint(Points.DEPOT)` returns the position as `{x, y}` in meters, and `paths.namedPoints()` returns all of them by name.

Positions are stored in the blue frame, as you placed them in the editor. `toPoint` flips the point for the red alliance when the command starts. See [Alliances](/guides/alliances/).

The library reads only the position. The heading the robot should arrive with is the argument you pass to `toPoint`, not something taken from the file.

### The schema and the field frame

`"schema": 2` says the named points are in the WPILib 2027 field frame: origin at the field's center, +X toward the blue alliance wall. A file with `"schema": 1`, or none, was written when +X pointed toward red. The library turns those points half a turn as it loads them, so `(x, y)` becomes `(-x, -y)`. Opening and saving the project in the editor rewrites the file at schema 2. See [The field frame](/concepts/field-frame/).

### The Java package

`settings.javaPackage` is the package the editor generates `PathEditorNames.java` into. It is `frc.robot` unless you change it in the editor under Settings, Project. At boot the library looks for a class called `PathEditorNames` in that package and, if it finds one, checks its constants against the files it loaded. See [Generated names](/files/generated-names/).

## An example

This is trimmed to the keys the library reads. A real file has more in it.

```json
{
  "schema": 2,
  "namedPoints": [
    { "name": "Depot", "x": 6.9, "y": 2.4 },
    { "name": "Score L", "x": 2.3, "y": -1.1 }
  ],
  "settings": {
    "javaPackage": "frc.robot"
  }
}
```

## What belongs to the editor

Everything else in the file is for the editor only, and the library never looks at it:

- each path tab's order and color;
- which waypoints follow a named point, and which are locked;
- linked copies of a path;
- a named point's heading and its twin on the other side of the field;
- the red-alliance flip setting and the defaults for new paths;
- the sim preview's drivetrain settings;
- the editor's view of the field.

These keys can change between editor versions. Robot code should not depend on them.

## When the file is missing or wrong

| Situation | Result |
| --- | --- |
| No `.pathproject.json` | No named points, and the generated names are not checked. Paths still load. |
| No `namedPoints` key | No named points |
| No `settings.javaPackage` | The generated names are not checked |
| A named point whose `x` or `y` is not a number | Boot fails with `IllegalStateException`, naming the file |
| The file is not a JSON object | Boot fails with `IllegalStateException`, naming the file |
| Robot code asks for a point that is not in the file | `namedPoint` throws `NoSuchElementException`, listing the points that exist |

:::note
The file name starts with a dot. Some file browsers hide such files. It still has to be deployed with the rest of `src/main/deploy/paths`, or the robot has no named points.
:::

## Related

- [PathLibrary](/reference/wpilib/pathlibrary/), which reads it
- [Pathfinding](/reference/wpilib/pathfinding/), for `toPoint`
- [Name IDs](/reference/wpilib/name-ids/), for `PointId`
- [Project files](/start/project-files/)
