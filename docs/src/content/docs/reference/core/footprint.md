---
title: Footprint
description: The robot's outline from above, as the bumpers or the bumpers plus a mechanism.
sidebar:
  order: 17
---

`Footprint` is a record for the robot's outline seen from above: a rectangle around the robot's center, given as how far it reaches to the front, the back, the left and the right.

You name footprints in the `footprints` list of [robot.json](/files/robot-file/) and refer to them by name. Robot code rarely constructs one.

Package: `frc.team5712.path.core`.

## Example

```java
Footprint bumpers = Footprint.bumpers(0.85, 0.85);    // length and width, meters
// The bumpers plus a mechanism that reaches 0.3 m further forward and takes 0.4 s to retract
Footprint intakeOut = Footprint.extended(bumpers, "intakeOut", 0.3, 0.0, 0.0, 0.0, 0.4);

double reach = intakeOut.reachMeters();     // meters from the center to the far corner
```

## Constructors

| Constructor | What it does |
| --- | --- |
| `Footprint(String name, double frontMeters, double backMeters, double leftMeters, double rightMeters, double retractSeconds)` | The record constructor. Throws `IllegalArgumentException` when `name` is null or blank, when `frontMeters + backMeters` or `leftMeters + rightMeters` is not above 0 and finite, or when `retractSeconds` is negative or not finite. |

## Static factories

| Method | What it does |
| --- | --- |
| `Footprint bumpers(double lengthMeters, double widthMeters)` | The bumpers: a rectangle of that length and width centered on the robot, named `"bumpers"`, with a `retractSeconds` of 0. |
| `Footprint extended(Footprint bumpers, String name, double front, double back, double left, double right, double retractSeconds)` | The bumpers plus a mechanism reaching `front`, `back`, `left` and `right` meters further on each side. |
| `List<Footprint> fromRobotFile(Map<String, Object> robot)` | Every footprint in a parsed robot.json: the bumpers first, then each entry of `footprints` as the bumpers plus its extensions. The returned list cannot be changed. |

## What fromRobotFile reads

| Key | Default and checks |
| --- | --- |
| `bumperLengthMeters`, `bumperWidthMeters` | 0.85 m each if absent |
| `footprints` | Throws `IllegalArgumentException` when present and not a list |
| `name`, in an entry | Throws `IllegalArgumentException` when an entry is not an object or has no `name` string, when two entries have the same name, or when an entry is called `"bumpers"` |
| `frontMeters`, `backMeters`, `leftMeters`, `rightMeters`, `retractSeconds`, in an entry | 0 each. Throws `IllegalArgumentException` when an extension or `retractSeconds` is negative or not finite |

## Components

| Component | What it is |
| --- | --- |
| `String name` | What path events and `PathSpec.startFootprint` call it |
| `double frontMeters` | Reach from the robot's center to the front, along its heading, in meters |
| `double backMeters` | Reach from the center to the back, in meters |
| `double leftMeters` | Reach from the center to the left, in meters |
| `double rightMeters` | Reach from the center to the right, in meters |
| `double retractSeconds` | How long the mechanism takes to come back in, in seconds. After an event switches away from this footprint, the checks keep it this long |

## Methods

| Method | What it does |
| --- | --- |
| `double reachMeters()` | How far the outline reaches from the robot's center in any heading, in meters: the distance to the far corner of a centered rectangle built from the larger of front and back and the larger of left and right. About 0.60 m for 0.85 m square bumpers. |

## Constants

| Constant | What it is |
| --- | --- |
| `BUMPERS` | `"bumpers"`. The name of the bumpers-only footprint. Every path starts with it unless it says otherwise |

## Good to know

- **`"bumpers"` is the bumpers alone.** Any other footprint is the bumpers plus a mechanism that sticks out, such as an intake or an arm.
- **The two places measure differently.** In a `Footprint` the four distances are from the robot's center. In robot.json they are how much further than the bumpers each side reaches.
- **Who uses it.** A path event can switch the outline, and the editor checks the path's clearance with the outline in force. The pathfinder plans with whichever footprint it is given, and grows obstacles by `reachMeters` so the outline clears them whichever way the robot faces.
- **Following a drawn path does not use footprints.**

:::note
`PathLibrary.loadAll()` fails at boot if a path file names a footprint, in an event or in `startFootprint`, that robot.json does not define.
:::

## More

- [Footprints](/concepts/footprints/): how they are used.
- [robot.json](/files/robot-file/): the `footprints` list.
