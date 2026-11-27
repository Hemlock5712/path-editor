---
title: Path file
description: Every key of a path.json file, with its type, unit and default.
sidebar:
  order: 1
---

A path file holds one drawn path: its waypoints, what the heading does, where it slows down, and the events along it. The file name ends in `.path.json`, and the name before that suffix is the path's name in robot code. The editor writes these files into `deploy/paths/`, and `PathLibrary.loadAll()` reads every one at boot.

You normally change a path in the editor. This page is for reading a file in a git diff, or editing one by hand. The reader and writer are [PathFile](/reference/core/pathfile/), and the result of reading is a [PathSpec](/reference/core/pathspec/).

## An example

```json
{
  "schema": 4,
  "headingMode": "hold",
  "waypoints": [
    { "x": 6, "y": 2, "tangentDegrees": 180, "headingDegrees": 180 },
    { "x": 3, "y": 1, "tangentScale": 1.2 },
    { "x": 0.5, "y": 1 }
  ],
  "rotationTargets": [
    { "meters": 2.5, "degrees": -90 }
  ],
  "facePointZones": [
    { "fromWaypoint": 1, "toWaypoint": 2, "x": 0, "y": 0, "blendMeters": 0.5 }
  ],
  "constraintZones": [
    { "fromWaypoint": 1.5, "toWaypoint": 2, "maxMetersPerSecond": 1.5 }
  ],
  "events": [
    { "name": "intakeDown", "atWaypoint": 0.5, "footprint": "intakeOut" },
    { "name": "shoot", "atMeters": 4.25 }
  ],
  "maxMetersPerSecond": 4,
  "maxMetersPerSecondSquared": 6,
  "startMetersPerSecond": 0,
  "endMetersPerSecond": 0
}
```

Positions are in meters in the [field frame](/concepts/field-frame/): the origin at the center of the field, +X toward the blue alliance wall. Angles are in degrees in the file. The library converts them to radians when it reads.

## Places along the path

Several keys name a place on the path. There are two ways to write one:

- **Waypoint-relative.** A number where the whole part is a waypoint index, counted from 0, and the fraction is how far along the arc to the next waypoint. `1.5` is halfway from waypoint 1 to waypoint 2. This keeps its meaning when waypoints are moved.
- **Meters.** The distance from the first waypoint along the curve.

## Top-level keys

| Key | Type | Unit | Default | Read by |
| --- | --- | --- | --- | --- |
| `schema` | integer | | required | `PathFile`: the file's version, 1 to 4 |
| `headingMode` | string | | `"hold"` | [HeadingMode](/reference/core/headingmode/): `"hold"` or `"faceTravel"` |
| `waypoints` | list | | required, at least two | The curve |
| `rotationTargets` | list | | empty | The heading |
| `facePointZones` | list | | empty | The heading |
| `constraintZones` | list | | empty | The speed plan |
| `events` | list | | empty | The follower |
| `startFootprint` | string | | `"bumpers"` | The editor's checks and `PathLibrary` |
| `maxMetersPerSecond` | number | m/s | no cap | The speed plan |
| `maxMetersPerSecondSquared` | number | m/s² | no cap | The speed plan: speeding up and braking |
| `startMetersPerSecond` | number | m/s | 0 | The speed plan |
| `endMetersPerSecond` | number | m/s | 0 | The speed plan and the follower |

The two maxima must be above zero if present. The start and end speeds must be zero or more. An `endMetersPerSecond` above zero makes a rolling end, where the path finishes with the robot still moving. See [Chaining paths](/guides/chaining-paths/).

## waypoints

Each entry is a point the path passes through. See [The curve](/concepts/the-curve/).

| Key | Type | Unit | Default | Meaning |
| --- | --- | --- | --- | --- |
| `x` | number | m | required | Field x |
| `y` | number | m | required | Field y |
| `tangentDegrees` | number | degrees | chosen automatically | Direction of travel through the point |
| `tangentScale` | number | | 1 | How long the curve keeps to the tangent. Must be above 0 |
| `headingDegrees` | number | degrees | none | The robot's heading at this waypoint |

Two neighboring waypoints may not be at the same place. The one exception is a path of exactly two waypoints at the same place, which is a turn on the spot.

## rotationTargets

Each entry is a heading to reach at a place. See [Heading](/concepts/heading/).

| Key | Type | Unit | Default | Meaning |
| --- | --- | --- | --- | --- |
| `waypoint` | number | waypoint-relative | | Where. Set this or `meters`, not both |
| `meters` | number | m | | Where, as a distance along the path |
| `degrees` | number | degrees | required | The heading to reach |

## facePointZones

Each entry is a stretch where the robot faces a field point.

| Key | Type | Unit | Default | Meaning |
| --- | --- | --- | --- | --- |
| `fromWaypoint` | number | waypoint-relative | required | Where the zone starts |
| `toWaypoint` | number | waypoint-relative | required | Where it ends. Must be after the start |
| `x` | number | m | required | The point to face, field x |
| `y` | number | m | required | The point to face, field y |
| `blendMeters` | number | m | 0.5 | Length of the ease in and of the ease out |

## constraintZones

Each entry is a stretch with lower limits. See [The speed plan](/concepts/speed-plan/).

| Key | Type | Unit | Default | Meaning |
| --- | --- | --- | --- | --- |
| `fromWaypoint` | number | waypoint-relative | required | Where the zone starts |
| `toWaypoint` | number | waypoint-relative | required | Where it ends. Must be after the start |
| `maxMetersPerSecond` | number | m/s | no cap | Speed cap inside the zone |
| `maxMetersPerSecondSquared` | number | m/s² | no cap | Acceleration and braking cap inside the zone |

## events

Each entry is a named event the follower fires once as the robot passes. Robot code binds the name to a command. See [Events](/guides/events/).

| Key | Type | Unit | Default | Meaning |
| --- | --- | --- | --- | --- |
| `name` | string | | required | The event's name. Must not be blank |
| `atWaypoint` | number | waypoint-relative | | Where. Set this or `atMeters`, not both |
| `atMeters` | number | m | | Where, as a distance along the path |
| `footprint` | string | | no change | The robot's outline from this event on |

### footprint and startFootprint

A **footprint** is the robot's outline seen from above: the bumpers, or the bumpers plus a mechanism that sticks out. Footprints are named in [robot.json](/files/robot-file/). The name `"bumpers"` always exists and means the bumpers alone.

An event's `footprint` says the outline changes at that event. In the example, `intakeDown` switches to `intakeOut`. `startFootprint` is the outline at the start of the path, for a path that begins with a mechanism already out.

These two keys do not change how the path is followed. The editor uses them to check clearance with the right outline. `PathLibrary.loadAll()` does check the names, and fails at boot if a path names a footprint that robot.json does not have. See [Footprints](/concepts/footprints/).

## Schema versions

The reader accepts schema 1 to 4. The writer always writes 4.

| Schema | What changed | How an older file is read |
| --- | --- | --- |
| 1 | The first format | |
| 2 | Added `events` | A schema 1 file has no events |
| 3 | Added `headingDegrees` on waypoints and `headingMode` | Schema 1 and 2 files read as `faceTravel`, which is what they did |
| 4 | The WPILib 2027 field frame, +X toward the blue wall | Schema 1 to 3 files are turned half a turn about the field center |

The half turn for an older file negates every `x` and `y`, and adds 180 degrees to every `tangentDegrees`, `headingDegrees` and rotation target `degrees`. The result is kept between -180 and 180 degrees.

The footprint keys were added without a new schema number, because a library that ignores them follows the path the same way.

## How it is written

The writer is deterministic, so a small edit in the editor makes a small git diff.

- Numbers are rounded to 5 decimal places, and trailing zeros are dropped. `1.50000` is written `1.5` and `3.00000` is written `3`.
- Keys are always in the order shown in the example.
- Each waypoint, target, zone and event is on one line.
- Optional values that are not set are left out: an automatic tangent, a `tangentScale` of 1, a waypoint with no heading, an event with no footprint, a `startFootprint` of `"bumpers"`, and any maximum with no cap.

## Unknown keys and errors

Keys the reader does not know are ignored. The editor can store its own data in a path file without breaking the robot.

A file that cannot be used makes `PathFile.read` throw an `IllegalArgumentException` with a message that says what is wrong. `PathLibrary.loadAll()` reads every file in robot init and rethrows the problem as an `IllegalStateException` that names the file. A bad file therefore stops the robot code at boot, not in the middle of a match.

Typical messages are `not a path file: no schema`, `missing 'x'` for a required key, `'tangentScale' must be a number` for a value of the wrong type, and `a path needs at least two waypoints`. The full list is on the [PathFile](/reference/core/pathfile/) page.

:::caution
A hand-edited file is not checked until something reads it. After editing by hand, open the path in the editor or run the robot code in simulation before you deploy.
:::
