---
title: HeadingMode
description: What the robot's heading does on a path where no heading is set.
sidebar:
  order: 8
---

`HeadingMode` is an enum that says what the robot's heading does on a path with no rotation targets and no waypoint headings. A swerve robot's heading is independent of where it travels, so the default is to hold. [Heading](/concepts/heading/) explains both modes.

Package: `frc.team5712.path.core`.

## Where you meet it

The mode is chosen in the editor and stored as `headingMode` in the [path file](/files/path-file/). In code it is a component of [PathSpec](/reference/core/pathspec/).

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

The mode only makes a difference when a path has no headings set at all. As soon as there is one rotation target or waypoint heading, both modes behave the same.

## Methods

| Method | Description |
| --- | --- |
| `String fileName()` | The name used in a path file: `"hold"` or `"faceTravel"`. |
| `static HeadingMode fromFileName(String name)` | The mode for a file name. Throws `IllegalArgumentException` for any other text, with a message that starts `unknown headingMode`. |

Like every Java enum, `HeadingMode` also has `values()` and `valueOf(String)`. `valueOf` takes the constant's name, `HOLD` or `FACE_TRAVEL`, not the file name.

:::note
`PathSpec` and schema 3 and 4 path files default to `HOLD`. The three-argument `HeadingProfile.build` defaults to `FACE_TRAVEL`, to match older code.
:::
