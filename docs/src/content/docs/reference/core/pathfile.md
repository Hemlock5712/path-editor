---
title: PathFile
description: Reads and writes path.json files as PathSpec objects.
sidebar:
  order: 19
---

`PathFile` reads a `.path.json` file into a [PathSpec](/reference/core/pathspec/) and writes a spec back out. The writer prints every number to 5 decimal places with keys in a fixed order, so a small edit makes a small git diff. Angles are in degrees in the file and in radians everywhere else. The class has only static methods and cannot be constructed.

Package: `frc.team5712.path.core`.

## Where you meet it

`PathLibrary.loadAll()` calls `PathFile.read` on every file in `deploy/paths/` at boot. Most students never call it. Use it directly to load one file in a test, or to write a path that was made in code. The [Path file](/files/path-file/) page lists every key.

## Example

```java
String json = PathFile.write(PathSpec.of(List.of(Waypoint.at(6.0, 2.0), Waypoint.at(3.0, 1.0))));

PathSpec spec = PathFile.read(json);
Path path = Path.build(spec, RobotConfig.DEFAULT);
```

## Constants

| Constant | Value | Meaning |
| --- | --- | --- |
| `SCHEMA` | `4` | The schema this code writes |
| `FRAME_2027_SCHEMA` | `4` | The first schema in the WPILib 2027 frame, with +X toward blue. Older files are turned half a turn when read |
| `OLDEST_SCHEMA` | `1` | The oldest schema this code still reads |

## Reading

| Method | Description |
| --- | --- |
| `static PathSpec read(java.nio.file.Path file)` | Reads a file as UTF-8 text and parses it. Throws `IOException` if the file cannot be read. |
| `static PathSpec read(String json)` | Parses JSON text. |
| `static PathSpec fromJson(Map<String, Object> root)` | Builds a spec from a JSON tree that is already parsed, as returned by [Json](/reference/core/json/). |

All three throw `IllegalArgumentException` when the content is not a usable path file:

- the text is not valid JSON, or its top level is not an object,
- there is no `schema` key, with the message `not a path file: no schema`,
- `schema` is not a whole number from `OLDEST_SCHEMA` to `SCHEMA`,
- a required key is missing, with a message such as `missing 'x'`,
- a value has the wrong type,
- `headingMode` is not `"hold"` or `"faceTravel"`,
- a record's own checks fail, such as fewer than two waypoints.

What reading does beyond copying values:

- Degrees become radians.
- A file with a schema below 3 gets `HeadingMode.FACE_TRAVEL`. A schema 3 or 4 file with no `headingMode` key gets `HeadingMode.HOLD`.
- A file with a schema below 4 is turned half a turn about the field center. Every x and y is negated, and every tangent, heading and rotation target angle has 180 degrees added.
- Keys the reader does not know are ignored.

## Writing

| Method | Description |
| --- | --- |
| `static String write(PathSpec spec)` | The file's text, ending in a newline. |
| `static void write(PathSpec spec, java.nio.file.Path file)` | Writes the text to a file as UTF-8. Throws `IOException` if the file cannot be written. |
| `static Map<String, Object> toJson(PathSpec spec)` | The file's JSON tree, with keys in file order. |

Writing always gives schema `SCHEMA`, whatever schema the spec was read from. Numbers go through `Json.fixed5`, which throws `IllegalArgumentException` for `NaN` or an infinite value. Values that are not set are left out of the file, so that does not happen for an automatic tangent or a limit with no cap.

Left out when writing: `tangentDegrees` when the tangent is automatic, `tangentScale` when it is exactly 1, `headingDegrees` when the waypoint has no heading, a zone's or the path's maxima when they have no cap, an event's `footprint` when it is null, and `startFootprint` when it is `"bumpers"`.

## Frame helpers

These turn one value by half a turn about the field center. The reader uses them on older files, and `PathLibrary` uses `turnedMeters` on named points from an older project file.

| Method | Description |
| --- | --- |
| `static double turnedMeters(double v)` | A coordinate turned half a turn: `-v`. A turned 0 is 0, not negative zero. |
| `static double turnedDegrees(double v)` | A direction in degrees turned half a turn, in the range above -180 and up to 180. `NaN`, meaning not set, stays `NaN`. |

## JSON helpers

Other readers in the library share these.

| Method | Description |
| --- | --- |
| `static Map<String, Object> asMap(Object o, String what)` | Casts a parsed JSON value to an object. Throws `IllegalArgumentException` with the message `<what> must be a JSON object` if it is something else. |
| `static double number(Map<String, Object> m, String key, double fallback)` | The number under `key`, or `fallback` when the key is absent or null. Throws `IllegalArgumentException` with the message `'<key>' must be a number` if the value is not a number. |

:::note
The class is called `PathFile`, and its file methods take a `java.nio.file.Path`. That is Java's file path type, not the library's [Path](/reference/core/path/). If a file imports both, write one of them with its full package name.
:::
