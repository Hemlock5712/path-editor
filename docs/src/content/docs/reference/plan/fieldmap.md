---
title: FieldMap
description: The field's size and obstacles for pathfinding, as read from field.json.
sidebar:
  order: 5
---

`FieldMap` is the record that holds the fixed field for pathfinding: its length, its width and its obstacles. It is what `field.json` becomes once it is loaded.

Robot code rarely touches it. `PathLibrary.loadAll()` reads `field.json` into a `FieldMap` and builds the pathfinders from it at boot. You meet it through `paths.pathfinder().field()`, or when building a `Pathfinder` yourself in a test. See [Pathfinding](/guides/pathfinding/).

Package: `frc.team5712.path.plan`.

## Example

```java
// What was loaded at boot
FieldMap field = paths.pathfinder().field();
System.out.println(field.lengthMeters() + " m by " + field.widthMeters() + " m, "
    + field.obstacles().size() + " obstacles: " + field.names());

// Reading a file directly
FieldMap fromFile = FieldMap.read(java.nio.file.Path.of("src/main/deploy/paths/field.json"));

// Building one in code: a 16.54 m by 8.07 m field with one square obstacle at the center
ConvexPolygon box = ConvexPolygon.of(new double[][] {
    {-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}});
FieldMap inCode = new FieldMap(16.54, 8.07, List.of("box"), List.of(box));
```

## Constructors

| Constructor | What it does |
| --- | --- |
| `FieldMap(double lengthMeters, double widthMeters, List<String> names, List<ConvexPolygon> obstacles)` | A field of this size with these obstacles. Throws `IllegalArgumentException` with `"field size must be positive"` when either size is not above 0, or `"one name per obstacle"` when the two lists have different lengths. |

## Components

| Component | What it is |
| --- | --- |
| `double lengthMeters` | The field's size along X, in meters. The walls are at x = ± half of it. |
| `double widthMeters` | The field's size along Y, in meters. The walls are at y = ± half of it. |
| `List<String> names` | One name per obstacle, in the same order as `obstacles` |
| `List<ConvexPolygon> obstacles` | The obstacles, each a convex polygon, in meters |

## Constants

| Constant | What it is |
| --- | --- |
| `SCHEMA` | `2`. The `field.json` schema that is in the WPILib 2027 frame |

## Static methods

| Method | What it does |
| --- | --- |
| `public static FieldMap read(java.nio.file.Path file) throws IOException` | Reads a `field.json` file as UTF-8 text and returns its `FieldMap`. Throws `IOException` when the file cannot be read, and the exceptions of `fromJson` when its contents are wrong. |
| `public static FieldMap fromJson(Map<String, Object> root)` | Builds a `FieldMap` from an already parsed `field.json`. Throws `IllegalArgumentException` with one of the messages below. |

## Keys fromJson reads

Any other key is ignored.

| Key | Required | What it is |
| --- | --- | --- |
| `schema` | no | Treated as 1 when absent. Below `SCHEMA`, every obstacle corner is turned half a turn about the center, `(x, y)` to `(-x, -y)`. |
| `field.length_m` | yes | Meters |
| `field.width_m` | yes | Meters |
| `obstacles` | no | A list. Absent means no obstacles. |
| `obstacles[].polygon` | yes | A list of `[x, y]` corners in meters. Replaced by its convex hull. |
| `obstacles[].name` | no | `"obstacle N"` when absent, where N counts from 1 |

## Messages fromJson throws

| Message | Cause |
| --- | --- |
| `'field' must be a JSON object` | `field` is missing or is not an object |
| `field length_m must be a number` | Missing, not a number, or not finite. `field width_m` has the same message with its own name. |
| `'obstacles' must be a list` | `obstacles` is present and is not a list |
| `obstacle must be a JSON object` | An entry of `obstacles` is not an object |
| `an obstacle needs a 'polygon' list` | An obstacle without a `polygon` list |
| `polygon points must be [x, y]` | A corner that is not a list of exactly two items |
| `x must be a number` | A corner coordinate that is not a finite number. `y` has the same message with its own name. |
| `a polygon needs at least three distinct corners` | The polygon's convex hull has fewer than three corners |
| `field size must be positive` | A size of 0 or less |

## Good to know

- **Everything is in meters in the WPILib 2027 field frame**, with the origin at the center of the field and +X toward the blue alliance wall.
- **Both lists are copied** when the record is built and cannot be changed afterward.

:::note
`FieldMap.read` and `fromJson` throw `IllegalArgumentException`. When `PathLibrary` loads the file at boot it wraps that in an `IllegalStateException` whose message starts with the file's path.
:::

## More

- [Field file](/files/field-file/): every key of the file.
- [ConvexPolygon](/reference/plan/convexpolygon/) and [Pathfinder](/reference/plan/pathfinder/): what uses the map.
- [The field frame](/concepts/field-frame/): where the origin is.
