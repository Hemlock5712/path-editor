---
title: Path events
description: When the follower fires each event, and what happens to events at a finish or a cancel.
sidebar:
  order: 11
---

An **event** is a name at a place on a path. The follower runs once per loop, so it has to choose which loop
fires each event, and what to do with events at a finish or a cancel.

## Where events come from

| Source | How | Action |
| --- | --- | --- |
| [The path file](/files/path-file/) | The `events` list. Registered when a [`PathFollower`](/reference/follow/pathfollower/) is made. | None. A listener handles the name. |
| Code | `addEvent(name, sMeters, action)`, at a distance in meters | A `Runnable` |
| Code | `addEventAtWaypoint(name, waypoint, action)`, where 1.5 is halfway from waypoint 1 to waypoint 2 | A `Runnable` |

```java
PathFollower follower = new PathFollower(path, FollowerConfig.defaults(paths.robot()))
    .addEventAtWaypoint("halfway", 1.5, () -> lights.flash())
    .onEvent(name -> System.out.println("event " + name));   // called with every event's name
```

[`FollowPathCommand`](/reference/wpilib/followpathcommand/) adds a listener that starts the command bound to each
name. In Commands v3 that command is forked as a child of the path command.

## What it does

1. Events are kept sorted by position. Two at the same position fire in the order they were added, file events
   first. A position before the start or past the end is moved to that end.
2. Each loop the follower works out how far the robot progressed since the last loop.
3. It fires every event that is less than half of that ahead. That is the loop nearest to passing the event.

   ```text
   lead = 0.5 · (progress this loop − progress last loop)
   fire every event with position ≤ progress + lead
   ```

4. When an event fires, its own action runs first, then every listener in the order they were added.
5. An event the robot has already passed, for example after a shove, fires on the next step. Events are never
   skipped during a run.

## When it fires

| Case | What happens |
| --- | --- |
| An event at position 0 | Fires on the first step |
| Progress goes backward | Zero lead. An event never fires early. |
| A turn on the spot | Progress comes from the heading, so events fire as the turn reaches them |
| The end of a path that stops | Every event fires as soon as the robot is inside the finish band. It does not have to be slow or settled. |
| A normal finish, stopping or rolling | Any event that has not fired fires then |
| A rolling end | The finish is timed with the same nearest-loop rule |
| `cancel()` | Unfired events are dropped. `step()` returns zero velocity. |
| `start()` again | The record of what fired is cleared, and every event fires again |
| A [moving goal](/concepts/moving-goals/) re-plans | Events ahead keep their share of the distance left |

:::caution
A canceled path does not fire the rest of its events. `FollowPathCommand` calls `cancel()` when it is
interrupted. If an event must be undone, such as raising an intake again, do it in the code that cancels the
path.
:::

## The numbers

| Name | Value | Unit | What it is |
| --- | --- | --- | --- |
| Timing error | at most half a loop of travel | | 4 cm at 4 m/s on a 50 Hz loop, where the robot moves 8 cm between loops |
| Progress rate cap | 8 | m/s | A jump in progress is not treated as speed |
| `positionToleranceMeters` | 2 | cm | Finish band, position |
| `headingToleranceRadians` | 1 | ° | Finish band, heading |

## Good to know

- **Add events in robot init**, before the first `start()`. Adding one while a path is running clears the record
  of what has fired.
- **`eventNames()`** lists every event's name in firing order. **`eventMeters()`** gives each position in meters,
  on the path as built.

## What it does not do

- **Events do not wait.** Nothing stops the robot until a mechanism is ready.
- **They do not carry data.** A listener gets the name and nothing else.
- **They are not timed.** An event is at a place, not at a number of seconds.
- **The follower does not check that anything is listening.**
  [`PathLibrary`](/reference/wpilib/pathlibrary/) has `requireAllEventsBound()` to catch that at boot.

## More

- [Events](/guides/events/): bind events to commands.
- [`PathEvent`](/reference/core/pathevent/): the event as stored in a path.
- [The follower](/concepts/the-follower/): the rest of the step.
