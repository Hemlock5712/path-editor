# editor: the path editor

| Folder | What |
| --- | --- |
| `core/` | The TypeScript port of `lib/`'s path maths (spline, heading, profile, path files). Reproduces `fixtures/` exactly. |
| `app/` | The editor: React + TypeScript + Vite. Every number it shows comes from `core/`, imported by relative path. |

## Running it

```
cd editor/app
npm install
npm run dev          # http://localhost:5712
```

| What | Command (in `editor/app`) |
| --- | --- |
| Production build (typecheck + bundle into `dist/`) | `npm run build` |
| Typecheck only | `npm run typecheck` |
| Unit tests (vitest) | `npm test` |
| End-to-end test (Playwright) | `npm run e2e` |
| Author the phase 6 suite into `e2e/out` (Playwright) | `npm run e2e:author` |
| The whole phase 6 chain (author, then run every path on the simulated robot) | `npm run e2e:full` |
| The core's tests | `cd ../core && npm install && npm test` |

The e2e test builds the app, serves it with `vite preview` and drives the installed Microsoft Edge, so no
browser download is needed. To use Playwright's own Chromium instead: `npx playwright install chromium`, then
`PW_CHANNEL=chromium npm run e2e`.

Opening a folder in place needs the File System Access API (Chrome or Edge). In other browsers, **Upload**
loads a folder and **Save** downloads a zip to unpack over the robot project.

## What it reads and writes

Open the robot project's root folder (the one with `build.gradle`). The editor works in:

| File | What | Read by |
| --- | --- | --- |
| `src/main/deploy/paths/<name>.path.json` | one per path tab, written by `core`'s `writePath` (schema 2) | `PathLibrary.loadAll()` |
| `src/main/deploy/paths/field.json` | field size and obstacle polygons | `FieldMap.read` |
| `src/main/deploy/paths/robot.json` | the robot (`RobotConfig`'s keys) plus `bumperLengthMeters` / `bumperWidthMeters` | `RobotFile`, `PlannerConfig.fromRobotFile` |
| `src/main/deploy/paths/.pathproject.json` | editor-only: tab order and colours, named points and which waypoints follow them, the red-frame and enum settings, the view | the editor only |
| `src/main/java/<package>/PathName.java` | an enum of every path file (package set in the Project panel; `frc.robot` by default) | robot code |

- **Only changed files are written.** `field.json` and `robot.json` are only created once you edit them.
- **Unknown keys are kept.** That applies to the keys in `field.json` and `robot.json` that the editor
  doesn't edit.
- **Renamed or deleted tabs lose their old file on the next save.** Only files this session loaded or
  saved are removed.
- **What you see is what the robot plans.** Edits are rounded to what the file holds (5 decimals; angles
  to 1e-5°), so the preview is exactly the plan the robot builds from the saved file.

## Features

**Field.**
- The 2026 field image, with grid lines every metre and the origin at the centre (WPILib 2027).
- Wheel to zoom about the pointer. Drag the empty field, or right- or middle-drag anywhere, to pan.
  **Fit** (F) resets the view.
- **Blue/red view** (A). Paths are always stored for blue. The red view draws them through the
  project's red frame (a half-turn about the centre, or a mirror; set in the Project panel), and edits
  made there are converted back to blue. Obstacles are physical, so they never flip.
- The bumper footprint is drawn along the path every 0.5, 1 or 2 m (or off). The path is coloured by
  planned speed.

**Tools.**
- **Select** (V): drag waypoints, tangent handles, rotation-target heading handles, face points, named
  points, events (they slide along the path) and obstacles. Hold Shift to snap to 5 cm and 15°.
- **Waypoint** (W): click the path to insert a waypoint there; click elsewhere to append one.
- **Event** (E): click the path to place a named event at the nearest point on it.
- **Rotation** (R): click the path to add a rotation target. It starts at the planned heading there.
- **Obstacle** (O): click corners, then double-click or press Enter to finish.

**Waypoints.**
- Add, insert, drag, delete and nudge (arrow keys: 1 cm, or 10 cm with Shift).
- Tangent handles set the direction and the `tangentScale` (the handle's length). A dashed handle is
  automatic; drag it to fix it, and double-click it to make it automatic again.
- The Path panel sets the start and end speeds, the top speed and the maximum acceleration.
- Inserting or deleting a waypoint renumbers the waypoint-relative positions of targets, zones and
  events, so each stays on the same stretch of path.

**Heading.**
- Rotation targets, by waypoint position or by metres.
- Face-point zones, with a blend distance. New ones default to a 1 m blend: lib's README notes that
  0.5 m can turn the robot over 100° and slow the plan to 0.7 m/s.
- Constraint zones.

**Named events.**
- Placed on the path by clicking. Each is stored as `atWaypoint` (it stays on its stretch when the path
  changes) or `atMeters`; switch in the panel.
- The name field suggests every event name already used in the project.
- The panel shows each event's distance along the path and its fire time in the plan.
- Event markers on the field and the charts light up as the playback passes them.

**Named points (linked targets).**
- Field points shared across paths. Link any waypoint to one, from the Waypoints panel or the Named
  points panel.
- Moving the point, by dragging it, dragging a linked waypoint or typing, moves every linked waypoint
  in every path, as one undo step.

**Obstacles.**
- Edits the polygons in `field.json`: draw, drag corners, drag whole obstacles, rename and delete.
- Each polygon is used as its convex hull, as `FieldMap` does.

**Live checks** (in the Checks panel, and marked on the path). All of them run on the core's plan:
- **Tight bend.** The steady cornering speed `sqrt(μg/|κ|)` is under half the path's top speed.
- **Turn too sharp for its distance.** The planned heading slows the plan to under 75 % of the same path
  planned without turning, and by at least 0.3 m/s. The message gives the turn, the distance, and both
  speeds. On lib's example route it reports "turns 128° over 1.54 m … slows to 0.77 m/s".
- **Bumpers.** The bumper rectangle is swept along the plan every 2 cm, with the planned heading.
  Touching an obstacle or the field wall is an error. Passing closer than 5 cm (the pathfinder's default
  margin) is a warning.
- **Off the path.** An event placed off the path is an error. A rotation target or zone off the path is
  a warning.
- **Speed.** A start or end speed that the plan can't hold, because a bend or turn caps it there.

**Charts and playback.**
- Charts of v(s), a(s), κ(s), θ(s) and ω(t), with the total time.
- Hovering a distance chart marks that point on the field, and hovering the path marks the charts.
  Clicking a chart seeks the playback.
- Play, pause and step (Space, comma and full stop). The robot moves along the plan using t(s) from
  the core.

**Robot.**
- Mass, moment of inertia, module radius, wheel radius, gear ratio, motor count, stator current limit
  and grip μ.
- Maximum wheel speed, spin and spin acceleration (`maxAngularRadPerSecSquared`). Leave one blank for no
  limit.
- A drive motor preset, and the bumper size.

**Editing.**
- Undo and redo are commands (one per edit, or per drag), covering every path, the field, the robot and
  the named points.
- Path tabs: new, duplicate, rename and delete. `[` and `]` switch between them.
- Every shortcut is listed under `?`.

**Off the UI thread.** Building and checking the active path runs in a Web Worker. At most one job is in
flight; a newer request replaces the one pending, so dragging stays smooth and the preview never starves.
Hit tests and snapping use the core's geometry on the main thread, which is cheap.

## The phase 6 authoring suite

`e2e-author/author.spec.ts` builds the end-to-end paths entirely through the UI: the panels, plus Event-tool clicks
on the path. The paths are the bench courses, a short hop, a turn on the spot, zones, a named point and a rolling
pair. It saves them with the editor's save flow into a folder on disk (a directory handle whose writes go through
Playwright to the file system). It reads each path's preview through `window.pathEditorTest.preview()`, a
read-only hook that waits for the worker to plan the current document and returns the total time, the peak speed
and each event's s and t(s). `E2E_ONLY=Straight,Hairpin` authors just those paths.

## Stubbed or not done

- **NT4 live robot: minimal and untested on a robot.** The client is read-only. It subscribes to a pose
  topic (`struct:Pose2d`, or a Field2d-style `double[]` in degrees) and to `/Telemetry/Path/`: WPILib
  2027's `Telemetry` publishes under `/Telemetry`, so `PathTelemetry("Path/<name>")` lands there. It
  shows the pose, the follower's reference and its values. It ignores NT time sync and has only been
  tested with hand-built frames.
- **Not built:** these items are in PLAN.md's phase 5 but weren't in this round's brief.
  - a follower sim preview;
  - the optimiser;
  - loading the bench comparison;
  - freehand stroke to waypoints.
- **Motor presets other than the Kraken X60 FOC** use vendor datasheet figures (stall torque, stall
  current, free speed) and haven't been checked against dyno data. The FOC numbers are the core's.
- **No tab reordering, no multi-select, and no numeric editing of obstacle corners** (drag them on the
  field).
