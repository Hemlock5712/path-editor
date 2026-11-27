---
title: Field file
description: Every key of field.json, the file that gives the pathfinder the field's size and obstacles.
sidebar:
  order: 3
---

`field.json` gives the [pathfinder](/concepts/pathfinder/) the field's size and obstacles. The editor writes it to `src/main/deploy/paths/field.json`, and only on-the-fly paths use it.

## An example

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

Coordinates are in meters in the [field frame](/concepts/field-frame/): the origin is the center of the field and +X points toward the blue alliance wall.

:::caution
If you write a `field.json` by hand, include `"schema": 2`. Without it your obstacles are loaded on the opposite side of the field.
:::

## Keys

All of these are read by `FieldMap.fromJson`. Any other key is ignored.

| Key | Type | Unit | Default | Meaning |
| --- | --- | --- | --- | --- |
| `schema` | number | | 1 when absent | Which field frame the file is in. `2` is the current one. |
| `field` | object | | required | The field's size |
| `field.length_m` | number | meters | required | Size along X, wall to wall |
| `field.width_m` | number | meters | required | Size along Y, wall to wall |
| `obstacles` | list | | no obstacles | The things to drive around |
| `obstacles[].name` | string | | `obstacle 1`, `obstacle 2`, ... | A label, used in messages and by the editor |
| `obstacles[].polygon` | list of `[x, y]` | meters | required | The obstacle's corners |

## Good to know

- **The walls** are the rectangle of the field's size centered on the origin. Both sizes must be greater than zero.
- **A `schema` below 2, or none,** is read as the old frame: every corner `(x, y)` becomes `(-x, -y)`.
- **A polygon** needs at least three corners that are not all on one line, listed in either direction.
- **A dent is filled in.** The library replaces every polygon with its **convex hull**. To keep a gap, split the shape into two convex obstacles.
- **Obstacles are not flipped for red.** A symmetric field lists both alliances' elements.

## With no field file

The library plans on an open floor with no obstacles and no walls, so every on-the-fly path is a straight drive to its goal. `paths.hasFieldFile()` tells you which case you are in.

## Errors

A bad file stops the robot program at boot, with an `IllegalStateException` that gives the file's path and one of these reasons.

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

The editor's default `field.json` is the 2026 field, 16.54 m by 8.07 m, with eight obstacles.

| Obstacles | Size |
| --- | --- |
| The blue hub and the red hub | 47 in squares |
| The blue tower and the red tower | 45 in deep and 49.25 in wide, against each alliance wall |
| Four trench supports, two per alliance | The solid part of each trench, between the bump and the lane |

The bumps and the trench lanes are not obstacles. If your robot avoids the bumps, or is taller than the 22.25 in the trench allows, add those areas as obstacles in the editor.

## More

- [FieldMap](/reference/plan/fieldmap/) and [ConvexPolygon](/reference/plan/convexpolygon/): the classes this file becomes.
- [PathLibrary](/reference/wpilib/pathlibrary/): loads it.
- [Project files](/start/project-files/)
