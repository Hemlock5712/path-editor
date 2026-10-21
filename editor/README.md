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
| The NT4 client against WPILib's real ntcore server (starts the Java harness through gradle) | `npm run test:nt4` |
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
| `src/main/deploy/paths/<name>.path.json` | one per path tab, written by `core`'s `writePath` (schema 3; schemas 1 and 2 still read) | `PathLibrary.loadAll()` |
| `src/main/deploy/paths/field.json` | field size and obstacle polygons | `FieldMap.read` |
| `src/main/deploy/paths/robot.json` | the robot (`RobotConfig`'s keys) plus `bumperLengthMeters` / `bumperWidthMeters` | `RobotFile`, `PlannerConfig.fromRobotFile` |
| `src/main/deploy/paths/.pathproject.json` | editor-only: tab order and colours, named points and which waypoints follow them, pinned waypoints, the red-frame and enum settings, the view | the editor only |
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
- The path line is the robot's centre, coloured by planned speed. Every waypoint is drawn as the robot
  there (its bumper footprint). Faint footprints between waypoints are optional (every 0.5, 1 or 2 m).
- A one-line hint at the bottom of the field says what the current tool does. A quick start shows on the
  first run; **?** shows it again with the keyboard shortcuts.

**Drawing a swerve path.** A swerve's heading is its own, so a waypoint is a robot pose: a position and,
optionally, a heading.
- **Draw** (W): click to add a point at the end; click the path to insert one. Enter or Esc when done.
  Drag a robot to move it; Delete or right-click removes it.
- Drag the knob on a robot's ring to turn it. Headings snap to 15° (hold Alt to turn freely); the
  waypoint's **Heading** field takes a typed angle. Right-click a waypoint for **Set heading…**,
  pass-through, pin and delete.
- A waypoint without a heading is **pass-through** (dashed): the robot blends between the set headings.
  The first and last waypoints always have one. A new end takes the old end's heading, and the old end
  becomes pass-through.
- **Heading** (R): click the path to add a heading partway along it (a rotation target), at the planned
  heading there. Drag its robot along the path, or its knob to turn it.
- The path's **Heading between set headings** is *hold* (blend between them: the default) or *face
  travel*. Files from schemas 1 and 2 read as face travel, as they always behaved.
- Snapping: to named points (a magnet, which also links the waypoint), to a 25 cm grid (**Grid** in the
  tabs row), and to 5 cm with Shift.
- **+** starts a new path where the active one ends, facing the way the robot ends up. A named point's
  **Start a path here** starts one there, linked to it.
- Travel direction is automatic, so clicks alone make a smooth path. **Shape the curve here** (in the
  waypoint's panel) shows its tangent handle, to set the direction and `tangentScale` by hand; **Reset
  to auto** undoes that.
- **Event** (E): click the path to place a named event at the nearest point on it.
- **Obstacle** (O): click corners, then double-click or press Enter to finish.
- **Select** (V): drag robots, knobs, face points, named points, events (they slide along the path) and
  obstacles; press on the path and drag to insert a point.

**Waypoints.**
- Add, insert, drag, delete and nudge (arrow keys: 1 cm, or 10 cm with Shift). **Pin** keeps a waypoint
  where it is when the optimiser runs (a padlock marks it).
- The Path panel sets the start and end speeds, the top speed and the maximum acceleration.
- Inserting or deleting a waypoint renumbers the waypoint-relative positions of targets, zones and
  events, so each stays on the same stretch of path.

**Heading.**
- Waypoint headings (above), and headings along the path (rotation targets), by waypoint position or
  by metres.
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

## Optimiser

Shaping a path is mostly the optimiser's job: draw the poses, then press **Optimise path** in the Path panel.

- **What moves.** The pass-through points, each within a corridor (0.5 m by default), and their tangent
  scales (half to double). Tangent directions don't change.
- **What stays put.** The start and end poses, any waypoint with a set heading (a pose the robot must reach
  there), pinned waypoints (**Pin** in the waypoint panel, its right-click menu, or **Optimiser settings**),
  waypoints linked to a named point, and any that a zone starts or ends on or between. **Only between
  waypoint … and waypoint …** in the settings optimises one segment. Events, rotation targets and zones placed
  by waypoint keep their waypoint position; those placed by metres keep the same fraction of the path's
  length.
- **What it must keep.** A candidate counts only if it plans and its swept bumpers keep the clearance the path
  had (up to the 5 cm margin). It also mustn't add heading warnings or errors.
- **Budget and repeatability.** The budget (3 s by default) sets how many plans it tries: 500 per second of
  budget. So the same path, budget and seed give the same result on any machine. The clock only stops a slow
  machine at twice the budget.
- **Results.** The field shows the best path so far, dashed. The Path panel gives the before and after times.
  **Apply** is one undo step (it refuses if the path changed meanwhile); **Discard** keeps the path.
- **Method.** Nelder-Mead with adaptive coefficients, restarted around the best point as it converges, in
  `app/src/compute/optimiser.ts`. In the editor it runs in a Web Worker in chunks, so **Stop** keeps the best
  so far. Every number comes from the core's plan.

**Headless.** The same optimiser runs from the repo root under Node 22.6+ (it loads the TypeScript sources
with type stripping):

```
node tools/optimise.mjs <path.json> --robot <robot.json> --field <field.json> --budget-ms 3000 --seed 1     [--out <file>] [--corridor 0.5] [--pin 2,5] [--segment 1,4] [--no-scales]
```

It writes the optimised path file (to `--out`, or stdout) and a one-line summary to stderr. `--robot` and
`--field` are the robot project's `robot.json` and `field.json`; both default to the editor's defaults. There is
no clock here, so the result depends only on the inputs, the budget and the seed. A unit test checks it against
the in-browser optimiser byte for byte.

## Bench comparison

**Bench** in the toolbar swaps the workspace for a read-only view of the bench's results.

- **Getting the data.** Export it with `python bench/run.py --export-editor`, which reads `bench/out/results.json`
  and writes `bench/out/editor_bench.json`. Seed 0's traces come from `results.json`; add
  `--trace-seeds 0,1,2` to simulate more. Then load the export.
- **What it shows.** For the chosen course and pose source, a table of every method's mean (and worst) time to
  done, time settled, cross-track, final error and yaw, overshoot and wheel slip. The lowest mean in each
  column is bold.
- **On the field.** Each method's authored path (dashed) and its true trace (solid) for the chosen seed: ours,
  Choreo, PathPlanner, BLine, the old follower, and the variants. The course is moved to the middle of the
  field.
- **Editing.** Editing shortcuts are off while the view is open, and nothing in it touches the project.

## Live robot (NT4)

`nt4/client.ts` is an NT4 client following WPILib 2027's ntcore sources.

- **Protocol.** WebSocket to `ws://<robot>:5810/nt/<name>` with the v4.1 subprotocol. It subscribes by prefix,
  handles announce, unannounce and properties, and decodes MessagePack binary values.
- **Time sync.** It sends `[-1, 0, int, its time]` and the server answers with its own clock. The client keeps
  the shortest round trip's offset, as ntcore's client does.
- **Connection.** It reconnects when the robot goes away. It is read-only.
- **The panel.** The connect box takes a team number (5712 → 10.57.12.2) or an address (`localhost` for a
  simulation) and the pose topic. The panel shows the active path, phase, progress s, the distance left,
  cross-track, the reference, the robot's pose and its age, and the events fired.
- **On the field.** The robot (pink), the reference, and the follower's progress, drawn on the path when the
  editor shows the path the robot is running.
- **Topics.** It reads what `PathTelemetry` publishes: `/Telemetry/PathFollower/{Active, EventsFired, LastEvent}`
  and `/Telemetry/Path/<path>/{Phase, Progress, Remaining, CrossTrack, Reference, Goal}`.

**Tested against the real thing.** `npm run test:nt4` starts lib's `Nt4TelemetryServer` with `gradlew nt4Server`.
That is WPILib's ntcore as an NT4 server, from the alpha-7 jars and desktop natives, with a real `PathFollower` in
`FollowPathCommand` running the S-curve on the simulated drivetrain in real time. The client connects and checks:

- the v4.1 protocol is negotiated;
- the telemetry arrives: the active path, progress rising inside the path's length, cross-track, and the
  reference and robot pose as Pose2d structs;
- a whole run's events arrive in order;
- the time sync works. The harness publishes ntcore's own clock (ns) and flushes each loop. On arrival, the
  client's estimate of server time sits at or just after that clock: measured median 0.8 ms, never earlier than
  half a round trip, with about a 1 ms round trip.

It hasn't been on a robot yet.

## Not done

- A follower sim preview (being built separately) and freehand stroke to waypoints.
- **Motor presets other than the Kraken X60 FOC** use vendor datasheet figures (stall torque, stall
  current, free speed) and haven't been checked against dyno data. The FOC numbers are the core's.
- **No tab reordering, no multi-select, and no numeric editing of obstacle corners** (drag them on the
  field).
