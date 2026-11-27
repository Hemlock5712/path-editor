---
title: Footprint
description: The robot's outline from above, as the bumpers or the bumpers plus a mechanism.
sidebar:
  order: 17
---

`Footprint` is a record for the robot's outline seen from above. The outline is a rectangle around the robot's center, given as how far it reaches to the front, the back, the left and the right. The footprint called `"bumpers"` is the bumpers alone. Any other footprint is the bumpers plus a mechanism that sticks out, such as an intake or an arm. [Footprints](/concepts/footprints/) explains how they are used.

Package: `frc.team5712.path.core`.

## Where you meet it

Footprints are named in the `footprints` list of [robot.json](/files/robot-file/). A path event can switch the outline, and the editor checks the path's clearance with the outline in force. On the robot, the pathfinder plans with whichever footprint it is given. Following a drawn path does not use footprints. In robot code you refer to a footprint by its name, and you rarely construct one.

## Example

```java
Footprint bumpers = Footprint.bumpers(0.85, 0.85);
Footprint intakeOut = Footprint.extended(bumpers, "intakeOut", 0.3, 0.0, 0.0, 0.0, 0.4);

double reach = intakeOut.reachMeters();     // meters from the center to the far corner
```

## Components

| Component | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `name` | `String` | | What path events and `PathSpec.startFootprint` call it |
| `frontMeters` | `double` | m | Reach from the robot's center to the front, along its heading |
| `backMeters` | `double` | m | Reach from the center to the back |
| `leftMeters` | `double` | m | Reach from the center to the left |
| `rightMeters` | `double` | m | Reach from the center to the right |
| `retractSeconds` | `double` | s | How long the mechanism takes to come back in. After an event switches away from this footprint, the checks keep it this long |

In a `Footprint` the four distances are measured from the robot's center. In robot.json they are written differently: as how much further than the bumpers each side reaches.

## Constants

| Constant | Value | Meaning |
| --- | --- | --- |
| `BUMPERS` | `"bumpers"` | The name of the bumpers-only footprint. Every path starts with it unless it says otherwise |

## Constructor

| Constructor | Description |
| --- | --- |
| `Footprint(String name, double frontMeters, double backMeters, double leftMeters, double rightMeters, double retractSeconds)` | The record constructor. |

Throws `IllegalArgumentException` when:

- `name` is null or blank,
- the rectangle does not have a positive, finite length and width, meaning `frontMeters + backMeters` and `leftMeters + rightMeters` must both be above 0,
- `retractSeconds` is negative or not finite.

## Factories

All three are static.

| Method | Description |
| --- | --- |
| `Footprint bumpers(double lengthMeters, double widthMeters)` | The bumpers: a rectangle of that length and width centered on the robot, named `"bumpers"`, with a `retractSeconds` of 0. |
| `Footprint extended(Footprint bumpers, String name, double front, double back, double left, double right, double retractSeconds)` | The bumpers plus a mechanism reaching `front`, `back`, `left` and `right` meters further on each side. |
| `List<Footprint> fromRobotFile(Map<String, Object> robot)` | Every footprint in a parsed robot.json. |

### fromRobotFile

The list starts with the bumpers, from `bumperLengthMeters` and `bumperWidthMeters`, each 0.85 m if absent. Each entry of `footprints` follows, as the bumpers plus its extensions. In an entry, `frontMeters`, `backMeters`, `leftMeters`, `rightMeters` and `retractSeconds` each default to 0.

It throws `IllegalArgumentException` when:

- an entry is not an object, or has no `name` string,
- two entries have the same name, or an entry is called `"bumpers"`,
- an extension or `retractSeconds` is negative or not finite,
- `footprints` is present and is not a list.

The returned list cannot be changed.

## Methods

| Method | Returns |
| --- | --- |
| `double reachMeters()` | How far the outline reaches from the robot's center in any heading, in meters. |

`reachMeters` is the distance to the far corner of a centered rectangle built from the larger of front and back and the larger of left and right. The pathfinder grows obstacles by this distance, so that the outline clears them whichever way the robot faces. For 0.85 m square bumpers it is about 0.60 m.

:::note
`PathLibrary.loadAll()` fails at boot if a path file names a footprint, in an event or in `startFootprint`, that robot.json does not define.
:::
