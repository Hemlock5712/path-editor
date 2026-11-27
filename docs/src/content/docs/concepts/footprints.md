---
title: Footprints
description: How the library describes a robot with a mechanism sticking out past its bumpers.
sidebar:
  order: 18
---

With an intake down or an arm out, the robot is bigger than its bumpers. A **footprint** is a named outline: the
bumpers plus how far a mechanism reaches past them to the front, back, left and right.

## Name them in robot.json

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

Each number must be 0 or more. Names must be unique, and none may be `bumpers`. A file that breaks these rules
fails at boot with a message naming the footprint.

The footprint `bumpers` always exists. Here `intakeOut` reaches 0.425 + 0.3 = 0.725 m to the front and 0.425 m
in the other three directions.

## Switch the outline along a path

```json
{ "name": "intakeDown", "atWaypoint": 0.5, "footprint": "intakeOut" }
```

A path that begins with a mechanism already out carries `startFootprint`.

1. From the event on, the editor draws the new outline. Its clearance checks, its optimizer and its Route around
   tool all use it.
2. When a later event switches to a smaller outline, the editor keeps the larger one for its `retractSeconds`.
3. At boot, `PathLibrary.loadAll()` stops the robot program if a path names a footprint that `robot.json` does
   not have.

:::note
Following a drawn path does not use footprints. The keys are there for the editor's checks.
:::

## What the pathfinder does

1. It grows every obstacle by the footprint's reach, the distance from the center to the outline's far corner.

   ```text
   reach = sqrt( max(front, back)^2 + max(left, right)^2 )

   bumpers, 0.85 m square:      reach = 0.601 m
   intakeOut, 0.3 m in front:   reach = sqrt(0.725^2 + 0.425^2) = 0.840 m
   ```

2. `PathLibrary.loadAll()` builds a [pathfinder](/concepts/pathfinder/) pair for each footprint at boot, one at
   the 0.05 m margin and one at the 0.15 m margin. `paths.footprintNames()` lists them.
3. A footprint bigger than the bumpers also keeps a second graph, grown by the bumpers' reach alone. The
   pathfinder plans on it when the first graph gives no route, or no clear one.
4. It sweeps the footprint itself along the finished path. `PlanResult.clear()` is true only when the footprint
   keeps the margin.
5. If a route point is itself too close, the new waypoint is pushed straight away from the nearest obstacle or
   wall until the footprint clears it by the margin plus 2 cm.
6. If the route is still not clear, it smooths again with headings that put the footprint's long side along the
   direction of travel. The second attempt is kept only if it is clear.

Step 3 exists because a 0.35 m intake closes the 1.85 m gaps between the hub and the trench supports on the
2026 field.

## Choose the footprint in robot code

```java
Pathfinding otf = new Pathfinding(paths, FollowerConfig.defaults(paths.robot()),
    () -> drive.getPose(), () -> drive.getFieldVelocity(),
    v -> drive.driveFieldVelocity(v), FieldFrame.RED_HALF_TURN);

// Read when each command starts
otf.withFootprint(() -> intake.isOut() ? "intakeOut" : "bumpers");
```

| Where you call `withFootprint` | Applies to |
| --- | --- |
| On `Pathfinding` | Every command built after it |
| On one `PathfindCommand` | That command only |
| Nowhere | Commands plan with `bumpers` |

A name that `robot.json` does not have throws `NoSuchElementException` when the command starts.

:::caution
The footprint is read once, when the command starts. Start the command with the largest outline the robot will
have during it.
:::

## The numbers

| Name | Value | Unit | What it is |
| --- | --- | --- | --- |
| `bumperLengthMeters` | 0.85 | m | Bumper length when the robot file does not set it |
| `bumperWidthMeters` | 0.85 | m | Bumper width when the robot file does not set it |
| Pathfinder margins | 0.05 and 0.15 | m | The pair built for each footprint |
| Push-off clearance | margin + 2 cm | | How far a pushed waypoint clears |

## What it does not do

- **It does not move the mechanism.** Your robot code still deploys and retracts the intake.
- **It does not change how a drawn path is followed.**
- **It only models a rectangle.** An arm that reaches out from one corner is covered by the rectangle that
  contains it.
- **It has not been tried on a real robot.** The behavior here comes from the library's unit tests on the 2026
  field.

## More

- [Path events](/concepts/path-events/), the [path file](/files/path-file/) and the
  [robot file](/files/robot-file/)
- [On-the-fly paths](/concepts/on-the-fly/)
- [Footprint](/reference/core/footprint/), [PlannerConfig](/reference/plan/plannerconfig/),
  [Pathfinder](/reference/plan/pathfinder/) and [PlanResult](/reference/plan/planresult/)
- [PathLibrary](/reference/wpilib/pathlibrary/), [Pathfinding](/reference/wpilib/pathfinding/) and
  [PathfindCommand](/reference/wpilib/pathfindcommand/)
