---
title: HeadingMode
description: What the robot's heading does on a path where no heading is set.
sidebar:
  order: 8
---

`HeadingMode` is an enum that says what the robot's heading does on a path with no rotation targets and no waypoint headings.

You choose the mode in the editor, which stores it as `headingMode` in the [path file](/files/path-file/). In code it is a component of [PathSpec](/reference/core/pathspec/).

Package: `frc.team5712.path.core`.

## Example

```java
PathSpec facing = spec.withHeadingMode(HeadingMode.FACE_TRAVEL);

String inFile = HeadingMode.HOLD.fileName();            // "hold"
HeadingMode mode = HeadingMode.fromFileName("faceTravel");
```

## Values

| Value | File name | Behavior |
| --- | --- | --- |
| `HOLD` | `"hold"` | The default. With no headings set, the robot holds the direction the path starts out in. With headings set, it holds the first one before it, blends between them, and holds the last one after it |
| `FACE_TRAVEL` | `"faceTravel"` | With no headings set, the robot faces the direction of travel. This is what path files before schema 3 did |

## Methods

| Method | What it does |
| --- | --- |
| `String fileName()` | The name used in a path file: `"hold"` or `"faceTravel"`. |
| `static HeadingMode fromFileName(String name)` | The mode for a file name. Throws `IllegalArgumentException` for any other text, with a message that starts `unknown headingMode`. |
| `values()`, `valueOf(String)` | The methods every Java enum has. `valueOf` takes the constant's name, `HOLD` or `FACE_TRAVEL`, not the file name. |

## Good to know

- **The mode only matters when a path has no headings set at all.** As soon as there is one rotation target or waypoint heading, both modes behave the same.
- **A swerve robot's heading is independent of where it travels**, so the default is to hold.

:::note
`PathSpec` and schema 3 and 4 path files default to `HOLD`. The three-argument `HeadingProfile.build` defaults to `FACE_TRAVEL`, to match older code.
:::

## More

- [Heading](/concepts/heading/): both modes explained.
- [PathSpec](/reference/core/pathspec/): `withHeadingMode`.
- [Path file](/files/path-file/): the `headingMode` key.
