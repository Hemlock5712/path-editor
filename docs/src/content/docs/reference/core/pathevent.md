---
title: PathEvent
description: A named event at a place on the path, fired once as the robot passes.
sidebar:
  order: 7
---

`PathEvent` is a record for a named event at one place on a path, which the follower fires once as the robot passes.

You bind events by name with `PathLibrary.bindEvent` and never handle `PathEvent` objects to do that. See [Events](/guides/events/). You construct one only when you build a [PathSpec](/reference/core/pathspec/) by hand.

Package: `frc.team5712.path.core`.

## Example

```java
List<PathEvent> events = List.of(
    // halfway between waypoints 0 and 1, and the outline becomes "intakeOut"
    PathEvent.atWaypoint("intakeDown", 0.5).withFootprint("intakeOut"),
    // 4.25 meters along the path
    PathEvent.atMeters("shoot", 4.25));

PathSpec withEvents = spec.withEvents(events);
```

## Constructors

| Constructor | What it does |
| --- | --- |
| `PathEvent(String name, double waypoint, double meters, String footprint)` | The full record constructor. Pass `NaN` for the place you are not using. |
| `PathEvent(String name, double waypoint, double meters)` | An event that leaves the footprint as it is. `footprint` is `null`. |

Both throw `IllegalArgumentException` when `name` is null or blank, when both places are set or neither is, when a place is infinite, or when `footprint` is not null but is blank.

## Static factories

| Method | What it does |
| --- | --- |
| `static PathEvent atWaypoint(String name, double waypoint)` | An event at a waypoint-relative position, with no footprint change. |
| `static PathEvent atMeters(String name, double meters)` | An event at a distance along the path in meters, with no footprint change. |

## Components

| Component | What it is |
| --- | --- |
| `String name` | The event's name. Robot code binds to this |
| `double waypoint` | The place as a waypoint-relative position, or `NaN`. `1.5` is halfway along the arc between waypoints 1 and 2 |
| `double meters` | The place as a distance along the path in meters, or `NaN` |
| `String footprint` | The name of the robot's outline from this event on, or `null` for no change. A [Footprint](/reference/core/footprint/) from robot.json, or `"bumpers"` for the bumpers alone |

## Methods

| Method | What it does |
| --- | --- |
| `PathEvent withFootprint(String footprintName)` | The same event, switching the outline to `footprintName`. `null` means no change. |
| `boolean byWaypoint()` | True when the place is waypoint-relative. |
| `boolean changesFootprint()` | True when `footprint` is not null. |
| `double sMeters(PathGeometry geometry)` | The event's place in meters along the path built from `geometry`. A waypoint-relative place is clamped to the path. A place given in meters is returned as it is, even if it is past the end. |

## Good to know

- **Exactly one of `waypoint` and `meters` is set.** The other is `NaN`.
- **A footprint switch is for the clearance check.** An event called `deployIntake` might switch to `intakeOut`, and `stowIntake` back to `bumpers`. The editor checks the path's clearance with the outline in force. Following the path does not use it.
- **Events are authored in the editor** and saved in the path file.

:::note
`PathLibrary.loadAll()` fails at boot if an event names a footprint that robot.json does not define.
:::

## More

- [Events](/guides/events/): the robot code that binds a name to a command.
- [Path events](/concepts/path-events/): when they fire.
- [PathSpec](/reference/core/pathspec/): where the events go.
