---
title: Field file
description: Every key of field.json, the file that gives the pathfinder the field's size and obstacles.
sidebar:
  order: 3
---

`field.json` describes the field for the [pathfinder](/concepts/pathfinder/): how big it is and where the obstacles are. The path editor at https://paths.frc5712.com writes it into the robot project at `src/main/deploy/paths/field.json`, and `PathLibrary.loadAll()` reads it once at boot. Drawn paths do not need it. Only on-the-fly paths use it.

You edit this file by hand when you want to add an obstacle the editor's default field does not have, or to check what the robot will plan around.

## A complete example

```json
{
  "schema": 2,
  "field": { "length_m": 16.54, "width_m": 8.07 },
  "obstacles": [
    {
      "name": "blue hub",
      "polygon": [
        [3.0481, -0.5969],
        [4.2419, -0.5969],
        [4.2419, 0.5969],
        [3.0481, 0.5969]
      ]
    },
    {
      "name": "cart",
      "polygon": [[-1.0, 2.0], [-0.2, 2.0], [-0.6, 2.9]]
    }
  ]
}
```

## Keys

| Key | Type | Unit | Default | Meaning |
| --- | --- | --- | --- | --- |
| `schema` | number | | 1 when absent | Which field frame the file is in. `2` is the current one. |
| `field` | object | | required | The field's size |
| `field.length_m` | number | meters | required | Size along X, wall to wall |
| `field.width_m` | number | meters | required | Size along Y, wall to wall |
| `obstacles` | list | | no obstacles | The things to drive around |
| `obstacles[].name` | string | | `obstacle 1`, `obstacle 2`, ... | A label, used in messages and by the editor |
| `obstacles[].polygon` | list of `[x, y]` | meters | required | The obstacle's corners |

All of these are read by `FieldMap.fromJson`. Any other key is ignored by the library, and the editor keeps keys it does not edit when it saves.

Both sizes must be numbers greater than zero. The walls are the rectangle of that size centered on the origin, so the 2026 field's walls are at x = ±8.27 m and y = ±4.035 m.

## The field frame

Coordinates are in meters in the WPILib 2027 field frame: the origin is the center of the field and +X points toward the blue alliance wall. `"schema": 2` says the file is in this frame. See [The field frame](/concepts/field-frame/).

A file with no `schema` key, or a `schema` below 2, was written when +X pointed toward the red wall. The library still reads it. It turns every obstacle half a turn about the center as it loads, so each corner `(x, y)` becomes `(-x, -y)`.

:::caution
If you write a `field.json` by hand, include `"schema": 2`. Without it your obstacles are loaded on the opposite side of the field.
:::

## Obstacles

Each obstacle is a polygon, a list of corners, each corner a two-number list `[x, y]`. The corners can be listed in either direction.

The library replaces every polygon with its **convex hull**: the shape a rubber band would make if stretched around the corners. A shape with a dent, such as an L, is planned around as if the dent were filled in. This is always on the safe side, but it can close a gap the real shape leaves open. To keep such a gap, split the shape into two convex obstacles.

A polygon needs at least three corners that are not all on one line.

Obstacles are where things physically are. They are not flipped for the red alliance, which is why a symmetric field lists both alliances' elements.

## With no field file

When `src/main/deploy/paths` has no `field.json`, the library plans on an open floor: no obstacles and no walls. The floor is a square 1,000 m across, `PathLibrary.OPEN_FLOOR_METERS`, centered on the origin. Every on-the-fly path is then a straight drive to its goal.

This is on purpose. A robot being tested in a shop, with its pose zeroed wherever it happens to be standing, is not stopped by walls that are not there. The field's walls arrive with its `field.json`. `paths.hasFieldFile()` tells you which case you are in.

In practice the editor always writes `field.json` when it saves, using the 2026 field by default, so a project saved from the editor has one.

## Errors

A bad file stops the robot program at boot. `PathLibrary` throws `IllegalStateException` with the file's path and one of these reasons:

| Message | Cause |
| --- | --- |
| `'field' must be a JSON object` | The `field` key is missing or is not an object |
| `field length_m must be a number` | Missing, not a number, or not finite. The same for `width_m`. |
| `field size must be positive` | A size of zero or less |
| `'obstacles' must be a list` | `obstacles` is present but is not a list |
| `an obstacle needs a 'polygon' list` | An obstacle with no `polygon` |
| `polygon points must be [x, y]` | A corner that is not a two-item list |
| `a polygon needs at least three distinct corners` | Fewer than three corners, or all of them in a line |

## The 2026 field

The editor's default `field.json` is the 2026 field, 16.54 m by 8.07 m, with eight obstacles:

| Obstacles | Size |
| --- | --- |
| The blue hub and the red hub | 47 in squares |
| The blue tower and the red tower | 45 in deep and 49.25 in wide, against each alliance wall |
| Four trench supports, two per alliance | The solid part of each trench, between the bump and the lane |

The bumps are not obstacles, because they are ramps a robot can drive over. The trench lanes are not obstacles either. They are open to robots up to 22.25 in tall. If your robot avoids the bumps, or is too tall for the trench, add those areas as obstacles in the editor.

The library's tests and the editor use this same file, so the robot, the editor and the tests plan around one set of obstacles.

## Related

- [FieldMap](/reference/plan/fieldmap/), the class this file becomes
- [ConvexPolygon](/reference/plan/convexpolygon/)
- [PathLibrary](/reference/wpilib/pathlibrary/), which loads it
- [Project files](/start/project-files/)
