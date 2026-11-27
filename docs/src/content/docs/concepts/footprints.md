---
title: Footprints
description: How the library describes a robot with a mechanism sticking out past its bumpers.
sidebar:
  order: 18
---

Most of the time the robot's outline, seen from above, is its bumpers. With an intake down or an arm out, it is bigger. A **footprint** is a named outline: the bumpers plus how far a mechanism reaches past them on each side. You name the footprints in `robot.json`, the editor draws and checks them, and the pathfinder can plan around obstacles with one.

You need footprints when a mechanism sticks out while the robot drives near field elements. A robot whose mechanisms stay inside the bumpers while moving does not need any.

## What a footprint is

A footprint is a rectangle around the robot's center, described by four distances: how far it reaches to the front, the back, the left and the right. Front is the direction the robot is heading.

The footprint called `bumpers` always exists. It is the bumper rectangle, centered on the robot, built from `bumperLengthMeters` and `bumperWidthMeters`. Both are 0.85 m when the robot file does not set them.

Every other footprint is the bumpers plus extensions.

## Naming them in robot.json

```json
{
  "bumperLengthMeters": 0.85,
  "bumperWidthMeters": 0.85,
  "footprints": [
    { "name": "intakeOut", "frontMeters": 0.3, "retractSeconds": 0.4 },
    { "name": "armUp", "frontMeters": 0.15, "backMeters": 0.1 }
  ]
}
```

| Key | Type | Unit | Default | Meaning |
| --- | --- | --- | --- | --- |
| `name` | string | | required | What events and robot code call it |
| `frontMeters` | number | meters | 0 | How much further than the bumpers it reaches to the front |
| `backMeters` | number | meters | 0 | The same to the back |
| `leftMeters` | number | meters | 0 | The same to the left |
| `rightMeters` | number | meters | 0 | The same to the right |
| `retractSeconds` | number | seconds | 0 | How long the mechanism takes to come back in |

Each number must be 0 or more. Names must be unique, and none may be `bumpers`. A file that breaks these rules fails at boot with a message naming the footprint. The [robot file](/files/robot-file/) page lists the other keys.

In the example, `intakeOut` on 0.85 m bumpers reaches 0.425 + 0.3 = 0.725 m to the front and 0.425 m in the other three directions.

## Switching the outline along a path

In the editor, a path event can switch the robot's outline. The event carries a `footprint` key in the path file, and a path that begins with a mechanism already out carries `startFootprint`.

```json
{ "name": "intakeDown", "atWaypoint": 0.5, "footprint": "intakeOut" }
```

From that event on, the editor draws the new outline, and its clearance checks, its optimizer and its Route around tool all use it. When a later event switches back to a smaller outline, the editor keeps the larger one for its `retractSeconds`, because the mechanism is still on its way in.

At boot, `PathLibrary.loadAll()` checks every path. A path whose event or `startFootprint` names a footprint that `robot.json` does not have stops the robot program with a message listing the path, the event and the footprints that do exist. See [Path events](/concepts/path-events/) and the [path file](/files/path-file/).

:::note
Following a drawn path does not use footprints. The follower drives the path as drawn, whatever the outline. The footprint keys in a path file are there for the editor's checks, which you run before the match.
:::

## What the pathfinder does with one

The [pathfinder](/concepts/pathfinder/) grows every obstacle by the robot's reach. For the bumpers the reach is the half-diagonal. For a footprint it is the distance from the center to the outline's far corner, treating the longest side as if it stuck out both ways:

```text
reach = sqrt( max(front, back)^2 + max(left, right)^2 )

bumpers, 0.85 m square:      reach = 0.601 m
intakeOut, 0.3 m in front:   reach = sqrt(0.725^2 + 0.425^2) = 0.840 m
```

### A pathfinder pair per footprint

Growing depends on the reach, so each footprint needs its own grown obstacles and its own visibility graph. `PathLibrary.loadAll()` builds them at boot: for each footprint, one pathfinder at the 0.05 m margin and one at the 0.15 m margin. That pair is what [on-the-fly paths](/concepts/on-the-fly/) use. `paths.footprintNames()` lists the footprints that have one.

### The bumpers-only graph

Growing by the far corner keeps the footprint clear at any heading, but it can close gaps the robot fits through nose first. A 0.35 m intake closes the 1.85 m gaps between the hub and the trench supports on the 2026 field.

So a pathfinder for a footprint that reaches further than the bumpers keeps a second graph, grown by the bumpers' reach alone. When the first graph gives no route, or no clear one, it plans on the second. Either way, the rectangle swept along the finished path is the footprint itself. `PlanResult.clear()` is true only when the footprint keeps the margin.

### Pushing a waypoint off an obstacle

A route on the bumpers-only graph can run too close for the footprint, for example with the intake pointing at the hub. The normal fix, adding the route point as a waypoint, does not help when the route point itself is too close. In that case the new waypoint is pushed straight away from whatever it is nearest to, an obstacle or a wall, until the footprint clears it by the margin plus 2 cm. A bumpers-only plan never needs this and is not changed by it.

### Turning narrow side on

A footprint that is longer one way than the other may still not get through with the heading it has at that point of the path. If the first attempt is not clear, the pathfinder smooths the route a second time. This time each waypoint it adds also gets a heading that puts the footprint's long side along the direction of travel, nose first or tail first, whichever is the smaller turn. Those headings become rotation targets on the path. The second attempt is kept only if it comes out clear, so it costs nothing when the first one already is.

## Choosing the footprint in robot code

Tell the pathfinding factory which footprint is in force. The supplier is read when each command starts.

```java
Pathfinding otf = new Pathfinding(paths, FollowerConfig.defaults(paths.robot()),
    () -> drive.getPose(), () -> drive.getFieldVelocity(),
    v -> drive.driveFieldVelocity(v), FieldFrame.RED_HALF_TURN);

otf.withFootprint(() -> intake.isOut() ? "intakeOut" : "bumpers");
```

`withFootprint` on `Pathfinding` applies to every command built after it. The same method on one `PathfindCommand` applies to that command only. With neither, commands plan with `bumpers`. A name that `robot.json` does not have throws `NoSuchElementException` when the command starts, listing the names that exist.

:::caution
The footprint is read once, when the command starts. If the intake goes out halfway through an on-the-fly path, the plan in progress was made for the smaller outline. Start the command with the largest outline the robot will have during it.
:::

## What it does not do

- It does not move the mechanism. A footprint is a description of the outline. Your robot code still deploys and retracts the intake.
- It does not change how a drawn path is followed.
- It does not model shapes other than a rectangle. An arm that reaches out from one corner is covered by the rectangle that contains it.
- It has not been tried on a real robot. The behavior described here comes from the library's unit tests on the 2026 field.

## Reference

- [Footprint](/reference/core/footprint/)
- [PlannerConfig](/reference/plan/plannerconfig/), for `withFootprint`, `halfDiagonalMeters` and `bumperGrowMeters`
- [Pathfinder](/reference/plan/pathfinder/) and [PlanResult](/reference/plan/planresult/)
- [PathLibrary](/reference/wpilib/pathlibrary/), for `pathfinder(name)` and `footprintNames()`
- [Pathfinding](/reference/wpilib/pathfinding/) and [PathfindCommand](/reference/wpilib/pathfindcommand/), for `withFootprint`
