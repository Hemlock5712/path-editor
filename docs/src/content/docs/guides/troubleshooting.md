---
title: Troubleshooting
description: The library's error messages and the common ways a path run goes wrong, with the cause of each.
sidebar:
  order: 15
---

Most problems show up as an exception in robot init, with a message that names the file or the name at fault.
The rest show up as a robot that drives, but wrongly.

## Errors at boot

Thrown by `PathLibrary.loadAll()` or by the lines right after it.

| Message | Cause | Fix |
| --- | --- | --- |
| `path <file>: ...` | A path file could not be read or built. The rest of the message says why. | Open the path in the editor and save it again. |
| `field <file>: ...` | `field.json` is damaged. | Save the field again from the editor's Settings page. |
| `named points in <file>: ...` | `.pathproject.json` has a named point that is not a number. | Save the project again from the editor. |
| `PathEditorNames doesn't match deploy/paths: ... Save the project in Myelin, then build and deploy again.` | The generated names class and the deployed files come from different saves. | Save in the editor, build, and deploy, in that order. |
| `robot.json has no such footprint: ... (have ...)` | A path's event switches to a footprint the robot file does not define. | Add the footprint to the robot, or fix the event, in the editor. |
| `events with no command bound: ... (bind each with bindEvent(Events.X, command))` | `requireAllEventsBound()` found an event with nothing bound. | Bind it, or remove the event from the path. |

## Errors when building commands

Thrown in robot init too, by the line that asks for the name. Each lists what does exist.

| Message | Thrown by |
| --- | --- |
| `no path named X (have [...])` | `paths.get`, `otf.follow`, `otf.thenFollow` |
| `no named point X (have [...])` | `paths.namedPoint`, `otf.toPoint` |
| `no footprint named X (have [...]; robot.json's footprints)` | `paths.pathfinder(name)`, and a command starting with that footprint |

## The robot drives the wrong way

[What your drivetrain provides](/start/drivetrain/#check-it-before-driving-a-path) has a four-step check for
these.

| What you see | Cause |
| --- | --- |
| It drives away from the path, faster and faster | The frame conversion is applied to the pose and not to the velocities, or the reverse. Convert everything or nothing. |
| It follows a mirror image of the path | Nothing is converted, and the robot's pose is in the old corner frame. |
| Correct on blue, backward on red | The drive method applies the operator perspective to the library's field velocity. |
| Correct on blue, a wrong but sensible path on red | The red frame is wrong for this field: `RED_HALF_TURN` where it should be `RED_MIRROR`, or the reverse. |
| It spins as it drives | `getFieldVelocity` returns a robot-relative velocity. |

## The robot follows the path badly

| What you see | Cause | Where to look |
| --- | --- | --- |
| Wide in every fast turn | The tires hold less than the file says | `frictionCoefficient`, [Tuning](/guides/tuning/) |
| Behind the plan the whole way | The plan is faster than the robot, or the latency is set too low | `massKg`, the motor keys, `latencySeconds` |
| Slow where you expected fast | A constraint zone, a tight turn, or a fast heading change is capping the speed | The editor's speed coloring and charts |
| Slows to a crawl in a face-point zone | The zone turns the robot a long way over a short blend | Lengthen the zone's blend |
| Overshoots the end | Braking harder than the drivetrain can answer | `brakeMetersPerSecondSquared`, the path's acceleration |
| Never finishes | The robot cannot hold the tolerance, or a `finishWhen` condition never comes true | The tolerances, a timeout |

## Events

| What you see | Cause |
| --- | --- |
| An event never runs | Nothing is bound. Call `requireAllEventsBound()` to catch it at boot. A hand-built command also needs `.withEvents(paths)`. |
| An event's command stops early | It is a child of the follow command and ended with it. |
| The path stops when an event fires | The event's command requires the drivetrain. |
| Every event fires at the end | The robot reached the end without passing them, usually because it started near the end of the path. |

## On-the-fly paths

| What you see | Cause |
| --- | --- |
| The command ends at once, robot stopped | No route was found. Read `PlanFailure`. |
| It drives straight through where an obstacle should be | No `field.json` was deployed. `paths.hasFieldFile()` is false. |
| It refuses a gap the robot fits through | The bumper size is too large, or a footprint is closing the gap. |
| It clips an obstacle | The bumper size in `robot.json` is too small, or the pose estimate is off. |
| It re-plans over and over | The robot keeps ending up more than 0.2 m off the plan. Raise `replanAbove`, or find out why. |

## Telemetry

| What you see | Cause |
| --- | --- |
| No `Path/` entries | No follow command has started. The entries appear on the first start. |
| The reference is mirrored against the robot in AdvantageScope | The library's poses are from the field's center and the robot's from the corner. See [Telemetry](/guides/telemetry/). |

## Still stuck

Read the source: the sources jar ships with the vendordep, so Ctrl-click any library class in VS Code. Start at
`PathFollower.step()`.
