---
title: Path events
description: When the follower fires each event, and what happens to events at a finish or a cancel.
sidebar:
  order: 11
---

An **event** is a name at a place on a path: lower the intake here, start the shooter there. The follower watches the robot's progress and fires each event as the robot passes it. This page is about that timing. To bind events to commands in robot code, see [Events](/guides/events/).

## Where events come from

There are two sources.

**The path file.** Events drawn in the editor are saved in the path file's `events` list, each with a name and a position, as described in [The path file](/files/path-file/). When a [`PathFollower`](/reference/follow/pathfollower/) is made, every event in its path is registered automatically. These events have a name and no action of their own.

**Code.** `addEvent(name, sMeters, action)` adds an event at a distance along the path in meters, with a `Runnable` to run. `addEventAtWaypoint(name, waypoint, action)` does the same at a waypoint-relative position, where 1.5 means halfway along the arc from waypoint 1 to waypoint 2.

All events are kept sorted by position, so they fire in order along the path. Two events at the same position fire in the order they were added, and the file's events are added first. A position before the start or past the end is moved to the start or the end.

## Listening for events

An event from the file has no action, so something has to listen for its name. `onEvent(listener)` adds a `Consumer<String>` that is called with the name of every event as it fires. Listeners add up. Each one sees every event.

When an event fires, its own action runs first, then every listener in the order they were added.

This is how the commands work. [`FollowPathCommand`](/reference/wpilib/followpathcommand/) adds a listener that looks each name up in its bindings and starts the bound command. In Commands v3 that command is forked as a child of the path command, so it ends with the path command unless it finishes first.

```java
PathFollower follower = new PathFollower(path, FollowerConfig.defaults(paths.robot()))
    .addEventAtWaypoint("halfway", 1.5, () -> lights.flash())
    .onEvent(name -> System.out.println("event " + name));
```

`eventNames()` lists every event's name in firing order, and `eventMeters()` gives each one's position in meters in the same order.

## When an event fires

The follower runs once per loop, so it can only fire an event on a loop. A robot at 4 m/s on a 50 Hz loop moves 8 cm between loops. The simple rule, fire on the first loop past the event, would be late by anything up to that 8 cm.

The follower fires on the loop **nearest** to passing the event instead. Each loop it works out how far the robot progressed since the last loop. An event that is less than half of that ahead will be passed before the next loop gets a chance, and this loop is the closer of the two. So it fires now.

```text
lead = 0.5 · (progress this loop − progress last loop)
fire every event with position ≤ progress + lead
```

The error is then at most half a loop of travel either side of the event: 4 cm at 4 m/s, less when slower.

Three details:

- Progress that goes backward counts as zero lead. An event never fires early because the robot was pushed back.
- A jump in progress is not treated as speed. The rate is capped at 8 m/s, so after a shove moves the robot's place on the path, the lead stays small.
- An event the robot has already passed, for example after that same shove, fires on the next step. Events are never skipped during a run.

An event at position 0 fires on the first step.

On a turn on the spot, progress comes from the heading, not from position, so events on such a path fire as the turn reaches them.

## Events at the end of a path that stops

An event at the very end of a path has a problem of its own. Near the end the follower drives to the goal and waits for the robot to settle before it calls the path finished. That takes a moment. An event meant to fire "on arrival" should not wait for it.

So in the finish, as soon as the robot is inside the finish band, every event up to the end of the path fires. The band is `positionToleranceMeters` and `headingToleranceRadians`, 2 cm and 1° by default. The robot does not have to be slow or settled yet.

## At a normal finish

When the path finishes, any event that has still not fired fires then. This holds for a path that stops and for one with a rolling end. An event is a promise that something happens during the path, and a normal finish keeps it.

On a rolling end the finish itself is timed with the same nearest-loop rule, so the next path starts on the loop nearest the line and its own first events are not late.

## On cancel

`cancel()` stops the follower. Events that have not fired are dropped. They do not fire, then or later. After a cancel `step()` returns zero velocity.

`FollowPathCommand` calls `cancel()` when it is interrupted.

:::caution
A canceled path does not fire the rest of its events. If an event does something that must be undone or completed, such as raising an intake again, handle that in the code that cancels the path.
:::

## Starting again

`start()` clears the record of what has fired, so the same follower can run the same path again and every event fires again.

Add events in robot init, before the first `start()`. Adding one while a path is running clears the record of what has fired.

## With a moving goal

When a [moving goal](/concepts/moving-goals/) re-plans the rest of the path, the path's length changes. Events still ahead keep their share of the distance left. An event halfway through what remained is halfway along the new piece. `eventMeters()` keeps reporting the positions on the path as built.

## What events do not do

- They do not wait. The follower fires an event and carries on. Nothing stops the robot until a mechanism is ready.
- They do not carry data. A listener gets the name and nothing else.
- They are not timed. An event is at a place on the path, not at a number of seconds.
- The follower does not check that anything is listening. A file event with no binding fires and nothing happens. [`PathLibrary`](/reference/wpilib/pathlibrary/) has `requireAllEventsBound()` to catch that at boot.

## Related pages

- [Events](/guides/events/), the guide
- [`PathEvent`](/reference/core/pathevent/), the event as stored in a path
- [The follower](/concepts/the-follower/), for the rest of the step
