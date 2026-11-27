---
title: PathEvent
description: A named event at a place on the path, fired once as the robot passes.
sidebar:
  order: 7
---

`PathEvent` is a record for a named event at one place on a path. It is authored in the editor and saved in the path file. The follower fires it once as the robot passes, and robot code binds the name to a command. An event can also say that the robot's outline changes at that place.

Package: `frc.team5712.path.core`.

## Where you meet it

You bind events by name with `PathLibrary.bindEvent`, and you do not handle `PathEvent` objects to do that. See [Events](/guides/events/) for the robot code and [Path events](/concepts/path-events/) for when they fire. You construct a `PathEvent` only when you build a [PathSpec](/reference/core/pathspec/) by hand.

## Example

```java
List<PathEvent> events = List.of(
    PathEvent.atWaypoint("intakeDown", 0.5).withFootprint("intakeOut"),
    PathEvent.atMeters("shoot", 4.25));

PathSpec withEvents = spec.withEvents(events);
```

## Components

| Component | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `name` | `String` | | The event's name. Robot code binds to this |
| `waypoint` | `double` | waypoint-relative | The place as a waypoint-relative position, or `NaN` |
| `meters` | `double` | m | The place as a distance along the path, or `NaN` |
| `footprint` | `String` | | The name of the robot's outline from this event on, or `null` for no change |

Exactly one of `waypoint` and `meters` is set. The other is `NaN`. A waypoint-relative position of `1.5` is halfway along the arc between waypoints 1 and 2.

`footprint` names a [Footprint](/reference/core/footprint/) from robot.json, or `"bumpers"` for the bumpers alone. An event called `deployIntake` might switch to `intakeOut`, and `stowIntake` back to `bumpers`. The editor checks the path's clearance with the outline in force. Following the path does not use it.

## Constructors

| Constructor | Description |
| --- | --- |
| `PathEvent(String name, double waypoint, double meters, String footprint)` | The full record constructor. Pass `NaN` for the place you are not using. |
| `PathEvent(String name, double waypoint, double meters)` | An event that leaves the footprint as it is. `footprint` is `null`. |

Both throw `IllegalArgumentException` when:

- `name` is null or blank,
- both places are set, or neither is,
- a place is infinite,
- `footprint` is not null but is blank.

## Factories

| Method | Description |
| --- | --- |
| `static PathEvent atWaypoint(String name, double waypoint)` | An event at a waypoint-relative position, with no footprint change. |
| `static PathEvent atMeters(String name, double meters)` | An event at a distance along the path in meters, with no footprint change. |

## Methods

| Method | Returns |
| --- | --- |
| `PathEvent withFootprint(String footprintName)` | The same event, switching the outline to `footprintName`. `null` means no change. |
| `boolean byWaypoint()` | True when the place is waypoint-relative. |
| `boolean changesFootprint()` | True when `footprint` is not null. |
| `double sMeters(PathGeometry geometry)` | The event's place in meters along the path built from `geometry`. |

`sMeters` clamps a waypoint-relative place to the path. A place given in meters is returned as it is, even if it is past the end.

:::note
`PathLibrary.loadAll()` fails at boot if an event names a footprint that robot.json does not define.
:::
