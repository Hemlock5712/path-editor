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

**Deployed.** Vercel builds `editor/app` (`vercel.json`): `master` is https://paths.frc5712.com and `v2` gets
preview deploys. A clean install builds with
`npm ci --prefix editor/core && npm ci --prefix editor/app && npm --prefix editor/app run build`. On the
https site, connecting to a robot needs the browser's permission (see **Live robot**, below).

**Install it as an app.** The deployed site is a PWA. In Edge or Chrome, use **Install app** (or the install
icon in the address bar), and it opens in its own window from the Start menu or the dock. The service worker
(`app/sw-plugin.ts` writes `dist/sw.js` at build time) caches the built app, so once it has been opened it
also opens offline. A new deploy replaces the cache the next time it opens online. `e2e/pwa.spec.ts` checks
that the browser finds it installable and that it reloads with the network off. The worker isn't registered
in development or under test automation. For native file access and NT4 without the browser's rules, see
the desktop app (`editor/desktop/`).

## What it reads and writes

Open the robot project's root folder (the one with `build.gradle`). The editor works in:

| File | What | Read by |
| --- | --- | --- |
| `src/main/deploy/paths/<name>.path.json` | one per path tab, written by `core`'s `writePath` (schema 3; schemas 1 and 2 still read) | `PathLibrary.loadAll()` |
| `src/main/deploy/paths/field.json` | field size and obstacle polygons | `FieldMap.read` |
| `src/main/deploy/paths/robot.json` | the robot (`RobotConfig`'s keys) plus `bumperLengthMeters` / `bumperWidthMeters` | `RobotFile`, `PlannerConfig.fromRobotFile` |
| `src/main/deploy/paths/.pathproject.json` | editor-only: tab order and colours, named points and which waypoints follow them, pinned waypoints, the red-frame and enum settings, the view | the editor only |
| `src/main/java/<package>/PathName.java` | an enum of every path file (package set in Settings > Project; `frc.robot` by default) | robot code |

- **Only changed files are written.** `field.json` and `robot.json` are only created once you edit them.
- **Unknown keys are kept.** That applies to the keys in `field.json` and `robot.json` that the editor
  doesn't edit.
- **Renamed or deleted tabs lose their old file on the next save.** Only files this session loaded or
  saved are removed.
- **What you see is what the robot plans.** Edits are rounded to what the file holds (5 decimals; angles
  to 1e-5°), so the preview is exactly the plan the robot builds from the saved file.

## Layout

The editor is built as an app in a window: no page scroll, its own controls, menus and dialogs, laid out
after PathPlanner's.

- **Top bar.** The project and its file actions, the pages (**Editor**, **Telemetry**, **Bench**), the
  command palette (**Ctrl+K**: every command by name, with its shortcut), help (**?**) and **Settings**
  (**Ctrl+,**).
- **Editor page.**
  - **Paths rail (left).** One card per path with a small preview: New (from the active path's end),
    Duplicate, Rename (F2 or double-click), Delete (undoable), and a right-click menu.
  - **Field (centre).** The tools and Undo/Redo float top left, with the path's name and plan (length, time,
    top speed). The checks badge (its popover lists them), blue/red view, grid snap, footprints and Fit float
    top right. Playback is a slim bar along the bottom. The charts are a drawer underneath, closed until opened.
  - **Inspector (right).** With nothing selected: **Optimise path**, then the path's cards in PathPlanner's
    order (Waypoints open; Global constraints, Start and end state, Rotation targets, Event markers,
    Constraint zones, Point-towards zones, Named points, Optimiser settings). With something selected: only
    that item, with Back (Esc) and, for waypoints, previous/next.
  - The rail and the inspector resize by dragging their edges, and the sizes are remembered.
- **Settings page.** Robot (robot.json), Path defaults (what a new path starts with), Field (size and
  obstacles), Project (the robot project folder, red flip, PathName enum), Sim preview drivetrain (latency,
  lag, grip), and Editor (snapping, footprints, the quick start).
- **Telemetry page.** The NT4 connection and live data. The connection stays up on the other pages, and a
  switch shows the live robot on the editor's field.
- **Feel.** Number fields scrub when you drag their label (Shift for finer) and step with the arrow keys or
  their steppers. Selects, switches, sliders, menus, tooltips (with shortcuts) and toasts are the app's own.
  The field pans with a little inertia and zooms smoothly toward the pointer. Ctrl+wheel, pinch and
  Ctrl +/−/0 zoom the field, never the page. There is no text selection or drag ghost on the app's chrome.
  Motion is 120-180 ms and off with the system's reduced-motion setting.
- **Screenshots.** `npm run screenshots` saves the empty editor, a drawn path, a selected waypoint, Settings
  and Telemetry at 1440×900 and 1280×800 to `app/screenshots/`.

## Features

**Field.**
- The 2026 field image, with grid lines every metre and the origin at the centre (WPILib 2027).
- Wheel to zoom about the pointer. Drag the empty field, or right- or middle-drag anywhere, to pan.
  **Fit** (F) resets the view.
- **Blue/red view** (A). Paths are always stored for blue. The red view draws them through the
  project's red frame (a half-turn about the centre, or a mirror; set in Settings > Project), and edits
  made there are converted back to blue. Obstacles are physical, so they never flip.
- The path line is the robot's centre, coloured by planned speed. Every waypoint is drawn as the robot
  there (its bumper footprint). Faint footprints between waypoints are optional (every 0.5, 1 or 2 m).
- A one-line hint at the bottom of the field says what the current tool does. A quick start shows on the
  first run; **?** shows it again with the keyboard shortcuts.

**Drawing a swerve path.** A swerve's heading is its own, so a waypoint is a robot pose: a position and,
optionally, a heading.
- **Adding waypoints: double-click.** A new project's path is empty: the first double-click places the
  start, the second the end. After that, select a waypoint and double-click the field: the new point goes
  right after it (between it and the next, or as the new end), at the double-clicked spot, and is selected,
  so more double-clicks build forward from it. With nothing selected a double-click only says how. A single
  click never adds a point: it selects or deselects. A new path (**+**) starts with just its start, at the
  last path's end, selected. **Add after** in a waypoint's inspector does the same from the keyboard
  (halfway to the next point, or 1 m past the end), and right-clicking the field offers it too. There is no
  separate drawing tool any more: double-click does its job in Select.
  Drag a robot to move it; Delete or right-click removes it. Pressing on the path and dragging still inserts a
  point there.
- Drag the knob on a robot's ring to turn it. Headings snap to 15° (hold Alt to turn freely); the
  waypoint's **Heading** field takes a typed angle. Right-click a waypoint for **Set heading…**,
  pass-through, lock and delete.
- A waypoint without a heading is **pass-through** (dashed): the robot blends between the set headings.
  The first and last waypoints always have one. A new end takes the old end's heading, and the old end
  becomes pass-through.
- **Guided adding.** Headings, events, zones and named points are each added in a few steps. A step bar
  over the field says which step is next (the current one lit), Esc cancels, and nothing changes until the
  last step, so the whole thing is one undo step. The toolbar, the keys, the command palette and the
  inspector's **+** buttons all start the same flows:
  - **Heading** (R): click the path, then turn the ghost robot there by its knob (letting go sets it) or
    type an angle. Afterwards, drag its robot along the path, or its knob to turn it.
  - **Event marker** (M): click the path, then name it. The project's names are offered as
    you type and as one-click chips.
  - **Constraint zone** (Z): click where it starts on the path, then where it ends, then give its top speed
    (and, optionally, its acceleration; blank is none).
  - **Point-towards zone** (T): its start and end on the path, then the point to face. That click snaps to a
    named point or a field element's centre (a hub, say) when it's close; Shift doesn't.
  - **Named point** (N): click the spot, then its name and (optionally) its heading.
- The path's **Heading between set headings** is *hold* (blend between them: the default) or *face
  travel*. Files from schemas 1 and 2 read as face travel, as they always behaved.
- Snapping: to named points (a magnet, which also links the waypoint), to a 25 cm grid (**Grid** in the
  tabs row), and to 5 cm with Shift.
- **+** starts a new path where the active one ends, facing the way the robot ends up. A named point's
  **Start a path here** starts one there, linked to it.
- Travel direction is automatic, so double-clicks alone make a smooth path. **Shape the curve by hand** (in the
  waypoint's inspector) shows its tangent handle, to set the direction and `tangentScale` by hand; **Reset
  to auto** undoes that.
- **Obstacle** (O): click corners, then double-click or press Enter to finish.
- **Select** (V): drag robots, knobs, face points, named points, events (they slide along the path) and
  obstacles; press on the path and drag to insert a point. Each zone has a chip beside its middle (its
  speed limit, or "faces n") that selects it.
- **Removing things.** Hover an event, a heading, a zone's chip or a named point and a small × shows by it
  (the selected one keeps its ×): click it to delete. Right-click any of them for **Delete**; the Delete key
  deletes the selection; every row in the inspector's lists has a trash icon. All of them undo.
- **Measure lengths** (L): drag between two points for the distance, direction and Δx, Δy. Its ends snap to
  waypoints and named points, and to 5 cm with Shift.
- Hovering the path shows the stretch under the pointer: which waypoints, its length and the plan's time
  on it.
- **Facing.** **Face next** (the waypoint's inspector, or its right-click menu) turns a waypoint toward the
  next one (the last one: the way it arrives). **Face target** turns it toward a named point or a field
  element's centre.
- **Poses by field elements.** Double-click a waypoint down beside a field element and a toast offers the
  pose in front of it: square to the element's nearest face, the bumpers 0.45 m off it, facing it
  ("Snap to 0.45 m in front of the blue hub, facing it?"). The elements are the field file's obstacles.
- **Copy and paste** (Ctrl+C, Ctrl+V). With a waypoint selected: a copy of it goes in after the selected
  one, 30 cm on. With nothing selected: the path, pasted as a new path (and its file is put on the system
  clipboard as text, so a path file copied from anywhere pastes as a new path too).

**Waypoints.**
- Add, insert, drag, delete and nudge (arrow keys: 1 cm, or 5 cm with Shift). Q and E turn the selected
  waypoint, heading or named point 5° left or right; Tab and Shift+Tab step through the waypoints. **Locked** (the waypoint's
  inspector, its right-click menu, or **Optimiser settings**) keeps a waypoint where it is whatever the
  optimiser does (a padlock marks it). Pass-through is only about heading: it never fixes or frees a point.
- The path's Global constraints and Start and end state cards set the top speed, the maximum acceleration
  and the start and end speeds.
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
  changes) or `atMeters`; switch in the event's inspector.
- The name field suggests every event name already used in the project.
- The inspector shows each event's distance along the path and its fire time in the plan.
- Event markers on the field and the charts light up as the playback passes them.

**Flip and mirror** (the Paths rail's flip button, a path's right-click menu, or the palette).
- **Flip** turns a path half-way round the field's centre (the other alliance's version). **Mirror top ↔
  bottom** (y → −y) makes the other side's auto. **Mirror left ↔ right** (x → −x) turns it end to end.
- Everything on the path turns with it: waypoints, headings, tangents, rotation targets and face points.
  Events and zones keep their places along it.
- The result can replace the path itself, be a new path (a copy), or be a **linked copy**. A linked copy
  follows its source: edit the source and the copy is remade, and the copy's own shape can't be edited (its
  named-point waypoints can still be dragged, which moves the shared point). Both cards show a link badge
  with **Unlink**.
- Named points the path uses get **twins** ("Score L" ↔ "Score L (mirror)"), made the first time and reused
  after. The pair stays in step: move either one and the other moves to match, and with it every waypoint on
  it. The point's inspector shows the twin, with **Unlink**.
- Links between paths, and between twin points, are kept in the sidecar (`.pathproject.json`).

**Named points (linked targets).**
- Field points shared across paths. Link any waypoint to one, from the waypoint's inspector (pick a point,
  or make one there).
- Moving the point, by dragging it, dragging a linked waypoint or typing, moves every linked waypoint
  in every path, as one undo step.

**Obstacles.**
- Edits the polygons in `field.json`: draw, drag corners, drag whole obstacles, rename and delete
  (on the field, or in **Settings → Field**).
- Each polygon is used as its convex hull, as `FieldMap` does. The editor always writes `field.json`, in
  `FieldMap`'s format, so the robot loads the same obstacles. A project's own `field.json` keeps its own.
- **The 2026 field (REBUILT) by default**, origin at the centre. The positions come from WPILib's
  2026 AprilTag layout (`2026-rebuilt-welded`), the sizes from the game manual:
  - **Hubs** (47 in square at x = ±3.645 m). Solid.
  - **Towers** (49.25 in by 45 in, against each alliance wall). Solid.
  - **Trench supports**: the solid part of each trench, between the bump and the 50.34 in lane. The lane
    itself, under the 22.25 in arm, is left free, so a robot under 22.25 in tall can use it. Taller robots:
    add the lanes as obstacles.
  - **Not obstacles**: the bumps (6.5 in ramps at 15°, driven over), the depots (a 1 in barrier) and the
    outposts (part of the wall, which is checked anyway).

**Live checks** (the badge on the field lists them, and they are marked on the path). All of them run on the
core's plan, and each has its fix-it buttons in the list:
- a cusp (the path doubles back, so the robot overshoots and comes back): **Reset waypoint n's curve to
  auto** or **Optimise**;
- a collision: **Route around** or **Optimise**;
- a tight bend: **Slow to … m/s here** (a constraint zone over the bend at its cornering speed) or **Optimise**;
- a turn too sharp for its distance: **Lengthen the blend** of the face-point zone that makes it (or **Edit
  the heading** that does);
- an event or heading off the path: **Snap to the path**; a zone running off it: **Clip to the path**;
- a start or end speed the plan can't hold: **Start at …** or **End at …** the speed it can.

A zone's inspector also has **Split at the middle**.

- **Tight bend.** The steady cornering speed `sqrt(μg/|κ|)` is under half the path's top speed.
- **Turn too sharp for its distance.** The planned heading slows the plan to under 75 % of the same path
  planned without turning, and by at least 0.3 m/s. The message gives the turn, the distance, and both
  speeds. On lib's example route it reports "turns 128° over 1.54 m … slows to 0.77 m/s".
- **Cusp** (an error). The path reverses direction somewhere: the spline's direction flips between two
  samples 1 cm apart. A tangent set against the way the path goes makes one (as in an auto of Joe's, whose
  end tangent pointed back against its approach); so can automatic tangents at very sharp zigzags. The
  message gives where ("The path reverses direction at s = 11.67 m (a cusp): the robot will overshoot and
  come back.") and the fix names the waypoint whose curve to reset. The core will report cusps itself
  (`Path.cusps()`, from core-builder); the editor's scan stands in until then.
- **Bumpers.** The bumper rectangle is swept along the plan every 2 cm, with the planned heading.
  Touching an obstacle or the field wall is an error, and the path turns red there with the bumpers drawn
  where they hit. Passing closer than 5 cm (the pathfinder's default margin) is a warning. The optimiser
  panel says how far the path clears them.
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

`e2e-author/author.spec.ts` builds the end-to-end paths entirely through the UI: the inspector and Settings, plus Event-tool clicks
on the path. The paths are the bench courses, a short hop, a turn on the spot, zones, a named point and a rolling
pair. It saves them with the editor's save flow into a folder on disk (a directory handle whose writes go through
Playwright to the file system). It reads each path's preview through `window.pathEditorTest.preview()`, a
read-only hook that waits for the worker to plan the current document and returns the total time, the peak speed
and each event's s and t(s). `E2E_ONLY=Straight,Hairpin` authors just those paths.

## Sim preview

**Sim** in the playback bar runs the robot's own follower on a simulated drivetrain. It is a TS port of
`PathFollower` that matches Java exactly: `core/src/follow.ts`, checked against the golden traces in
`fixtures/follow`. It runs in a worker (`app/src/sim/`) whenever the path, the robot or a sim setting changes.

- **The playback bar** shows when it finished against the plan, the settle time and the cross-track max.
- **The field** shows the simulated trace, shaded by cross-track error; where each event fired against its
  flag; the shove; and the simulated robot (dashed) at the playback time, next to the plan's.
- **The inspector's Sim preview card** has a shove (a hit at a chosen time), cross-track over time (click to
  seek), and a table of when each event fired against the plan.
- **The drivetrain** is in **Settings > Sim preview**: command latency, velocity lag and floor grip. It is
  saved in the project's sidecar file, not robot.json, because it describes the sim, not the robot. The
  defaults are the end-to-end suite's: 20 ms, 40 ms, no grip cap.
- **What it assumes.** The floor grips: slip sensing and adaptive grip run only on the robot.

## Optimiser

Shaping a path is mostly the optimiser's job: draw the poses, then press **Optimise path** at the top of the
inspector. It is enabled for any path with a start and an end.

- **Curves and speeds (the default).** Waypoint positions and headings never move. It changes only the curve:
  the tangent's direction (up to 60° either way) and scale (half to double) at every waypoint, the ends
  included. The speed plan follows (it slows where the curve bends harder), and that is also how it clears a
  wall: bend harder and slow down. Applying it changes only tangents, so the poses stay byte for byte.
- **Also move unlocked waypoints** (a switch in **Optimiser settings**, off by default). The interior
  waypoints may also move, each within the corridor (0.5 m by default), with or without a heading; a heading
  goes with its waypoint unchanged.
- **Never moves, in either mode.** The start and end poses, locked waypoints, waypoints linked to a named
  point, and any that a zone starts or ends on or between. **Only between waypoint … and waypoint …** limits it
  to one segment (the tangents at its two ends included). Events, rotation targets and zones placed by waypoint
  keep their waypoint position; those placed by metres keep the same fraction of the path's length. It never
  adds waypoints.
- **Never doubles back.** A candidate with a tangent more than 100° from its local chord, or with a cusp
  anywhere, is ruled out (the speed profile can't see a cusp, so a reversed tangent could look faster). A
  tangent that already points backwards starts from its automatic direction, and a path with a reversal is
  repaired first, like a collision, so running Optimise on one fixes it.
- **What it must keep.** A candidate counts only if it plans, its swept bumpers clear every obstacle and the
  field wall by the 5 cm margin, and it adds no heading warnings or other errors. Clearance is a hard rule:
  a path that starts in collision (or under the margin) is first reshaped clear, by minimising its time plus a
  large weight times the clearance it lacks along the path. It is then made faster among clear paths, even if
  the clear result is slower than the colliding one. A clear path never comes back unclear.
- **When it can't.** It says what and where ("Can't clear the wall by reshaping the curve between waypoints 2
  and 3: add a waypoint or move one"), marks the spot on the field with a suggested waypoint, and offers no
  Apply. **Add a waypoint here** puts that point in (one undo step); it is never added on its own.
- **Route around obstacles.** Enabled when the bumpers hit (or come within 5 cm of) something. The robot's
  own pathfinder (the core's port of lib's Pathfinder, bit for bit) plans from the waypoint before the first
  colliding stretch to the one after it. Its corners go in as unlocked pass-through waypoints, replacing the
  plain pass-through points in between. A waypoint in between that is locked, linked to a named point or has
  a heading is kept, and the route goes through it leg by leg. The curves-only optimiser then shapes the
  result. It shows dashed, with how many points it adds; **Apply** puts it all in as one undo step.
- **Budget and repeatability.** The budget (3 s by default) sets how many plans it tries: 500 per second of
  budget. So the same path, budget and seed give the same result on any machine. The clock only stops a slow
  machine at twice the budget.
- **Results.** The field shows the best path so far, dashed. The inspector gives the before and after times.
  **Apply** is one undo step (it refuses if the path changed meanwhile); **Discard** keeps the path.
- **Method.** Nelder-Mead with adaptive coefficients, restarted around the best point as it converges, in
  `app/src/compute/optimiser.ts`. In the editor it runs in a Web Worker in chunks, so **Stop** keeps the best
  so far. Every number comes from the core's plan.

**Headless.** The same optimiser runs from the repo root under Node 22.6+ (it loads the TypeScript sources
with type stripping):

```
node tools/optimise.mjs <path.json> --robot <robot.json> --field <field.json> --budget-ms 3000 --seed 1     [--out <file>] [--move-waypoints] [--corridor 0.5] [--pin 2,5] [--segment 1,4] [--no-scales]
```

It writes the optimised path file (to `--out`, or stdout) and a one-line summary to stderr, with the clearance.
By default it reshapes the curve only, as in the editor; `--move-waypoints` also moves the unlocked interior
waypoints within `--corridor`. If it can't get the path clear, it writes the path unchanged, says what blocks it
and where (with a waypoint that would help), and exits with code 3. `--robot` and
`--field` are the robot project's `robot.json` and `field.json`; both default to the editor's defaults. There is
no clock here, so the result depends only on the inputs, the budget and the seed. A unit test checks it against
the in-browser optimiser byte for byte.

## Bench comparison

The **Bench** page is a read-only view of the bench's results, laid out like a model launch page.

- **The verdict**, written from the data. For example: "Ours settles first on 11 of 14 targets; PathPlanner
  (Bezier, tuned + hold) on 2. Ours gets within 5 cm first on 9. Didn't settle, or ended off: Choreo on 4,
  BLine (tuned) on 1."
- **The pose switch** (perfect pose, wheel odometry, slip-aware) drives everything on the page, with a line
  saying what each means. It opens on perfect pose: the follower alone. On odometry no method settles inside
  2 cm of the true goal, and the verdict says so.
- **One table across every course and 2026 route.**
  - The columns are Ours, PathPlanner (Bezier, tuned + hold), PathPlanner path2 (tuned), Choreo and BLine
    (tuned). **+ more methods** adds the as-shipped variants.
  - Each cell is the time settled. Ours' column is tinted, and the best valid cell in each row is bold.
  - A run counts only if every seed settled (2 cm and 1°, held 0.25 s) and it ended within 5 cm. One that
    doesn't shows "✕ never", "✕ 3/10" (seeds settled) or "38 cm off" in muted text, and can't be bold.
  - Summary rows give the targets won and the mean time settled, over the targets every method settled on.
  - Clicking a target opens it in the course detail.
- **How we compare.** A horizontal bar chart per metric across the methods, best first, with ours in the
  accent and values at the bar tips. The metrics are time settled, time to 5 cm, final error, cross-track
  RMS, wheel slip, reliability, and plan time against actual. The bars are over the same targets for every
  method, so none is flattered by skipping the hard ones. **Per target** shows one small chart per course and
  route.
- **Optimised paths.** The PathPlanner comparison (`run.py --pp-compare`): the same waypoints as drawn and after
  each one's own optimiser. Each cell gives the time settled and the planned time.
- **Course detail.** The field with each method's authored path (dashed) and true trace (solid), the
  "what to look at" line, a scoreboard ranked by time settled, then seeds settled, then final error, and
  every metric.
  - Bench courses are drawn at one fixed spot, labelled "Bench course, not a field route".
  - The 2026 routes are drawn where they are on the field.
- **Colours.** The five headline methods take the documented categorical palette's first five slots, validated
  for colour vision on this app's surface. Ours is slot 1, blue.
- **Data.** The page opens with the committed snapshot, `app/public/bench/results-lite.json` (0.9 MB: every table,
  the PathPlanner comparison, geometry to the millimetre, and perfect-pose traces every 0.2 s). Refresh it with
  `python bench/run.py --export-editor`, which writes it and the full export (`bench/out/editor_bench.json`).
  The file picker at the bottom of the page loads the full export.

## Live robot (NT4)

`nt4/client.ts` is an NT4 client following WPILib 2027's ntcore sources.

- **Protocol.** WebSocket to `ws://<robot>:5810/nt/<name>` with the v4.1 subprotocol. It subscribes by prefix,
  handles announce, unannounce and properties, and decodes MessagePack binary values.
- **Time sync.** It sends `[-1, 0, int, its time]` and the server answers with its own clock. The client keeps
  the shortest round trip's offset, as ntcore's client does.
- **Connection.** It reconnects when the robot goes away. It is read-only.
- **The Telemetry page** shows how well the robot tracks a path. The connect box takes a team number
  (5712 → 10.57.12.2) or an address (`localhost` for a simulation) and the pose topic.
  - **Recording.** Every run of the follower is recorded loop by loop (`nt4/recorder.ts`), from
    PathFollower/Active naming a path until it goes back to "". The last 10 runs are listed, and each can be
    exported as JSON or imported back.
  - **The field.** The path the robot is following: the plan it published, or the open project's path of that
    name. Where it should be (the reference, ghosted), where it is (solid), a line between them, the trail
    coloured by cross-track error (green on the line, red from 10 cm), and the path's events lighting up as
    they fire.
  - **Graphs** over time or over distance along the path: cross-track, along-track (ahead of or behind the
    plan's schedule) and heading error, planned and actual speed, and planned and actual turn rate. The
    finish phase is shaded and events are marked where they fired.
  - **A cursor** is shared by the field and the graphs. It follows the live run (**Live**), or scrubs and
    replays a recorded one.
  - **The summary** gives cross-track RMS and max, final position and heading error, the time against the plan,
    and the events fired against the path's.
- **On the editor's field.** The robot (pink), the reference, and the follower's progress, drawn on the path when
  the editor shows the path the robot is running (a switch on the Telemetry page turns this off).
- **Topics.** It reads what `PathTelemetry` publishes.
  - `/Telemetry/PathFollower/{Active, EventsFired, LastEvent}`.
  - Each loop, `/Telemetry/Path/<path>/{Phase, Progress, Remaining, CrossTrack, AlongTrack, HeadingError, Speed,
    PlannedSpeed, Omega, PlannedOmega, Elapsed, Reference, Goal}`.
  - Once per start, `Plan`: the planned path every 5 cm as [s, x, y, heading, t, v] per sample, in the frame the
    robot runs it in. About 1,100 doubles for a 9 m path. Also `PlanEvents`, `PlanEventMeters` and `PlanSeconds`.
  - It subscribes with "all" so no loop is coalesced away.

**Tested against the real thing.** `npm run test:nt4` starts lib's `Nt4TelemetryServer` with `gradlew nt4Server`.
That is WPILib's ntcore as an NT4 server, from the alpha-7 jars and desktop natives, with a real `PathFollower` in
`FollowPathCommand` running the S-curve on the simulated drivetrain in real time. The client connects and checks:

- the v4.1 protocol is negotiated;
- the telemetry arrives: the active path, progress rising inside the path's length, cross-track, and the
  reference and robot pose as Pose2d structs;
- a whole run's events arrive in order;
- the time sync works. The harness publishes ntcore's own clock (ns) and flushes each loop. On arrival, the
  client's estimate of server time sits at or just after that clock: measured median 0.8 ms, never earlier than
  half a round trip, with about a 1 ms round trip;
- a whole run, recorded as the Telemetry page records it, matches the follower's own account of it. At the end
  of each run the harness publishes `Harness/RunSummary` from the follower's values each loop. Measured: 183 of
  183 loops, cross-track RMS 0.389 cm (follower 0.389), max 1.105 cm (1.105), final error 0.16 cm (0.15).

`npm run e2e:live` runs the Telemetry page against the same harness in a browser. It records runs, checks the
summary and the graphs, and saves `screenshots/telemetry-during-run.png` and `telemetry-after-run.png`.

It hasn't been on a robot yet.

**From the deployed (https) site.** The robot speaks plain `ws://`. In Chrome 155 and Edge 155, measured from
https://paths.frc5712.com, a `ws://` connection is not blocked as mixed content when it goes to a private IP
(the robot's 10.TE.AM.2, or any 10.x, 172.16-31.x, 192.168.x) or to localhost. Instead it is gated by **Local
Network Access**. The browser asks once per site: "local network" for the robot and "apps on this device" for a
simulation on localhost. Until that is allowed, the connection fails with
`net::ERR_BLOCKED_BY_LOCAL_NETWORK_ACCESS_CHECKS`. Granting it, the same connections opened.

- **What the Telemetry page does.** On https it checks the permission (`navigator.permissions.query` with
  `local-network` or `loopback-network`). If the browser will ask, the page says to allow it. If it was
  blocked, the page says where to re-allow it (the site settings, left of the address bar).
- **Other browsers.** If the browser has no such permission (the query throws), it blocks `ws://` from https
  pages outright, so the page says to use Chrome or Edge or to run locally. A bare host name
  (`roborio-5712-frc`) or a public address counts as mixed content too.
- **Managed computers.** An administrator can pre-allow the site with Chrome's `LocalNetworkAccessAllowedForUrls`
  policy.
- **Running locally always works.** `npm --prefix editor/app run dev` serves the editor over http on this
  computer, where none of this applies.
- **Tests.** `e2e/https.spec.ts` serves the production build at the real site's origin and checks the Telemetry page's
  notes, before and after the permission is granted. The Chrome background is
  [the Local Network Access permission prompt](https://developer.chrome.com/blog/local-network-access) and
  [its extension to WebSockets (Chrome 147)](https://groups.google.com/a/chromium.org/g/blink-dev/c/O6GMKt44Ups).

## Not done

- Freehand stroke to waypoints.
- **Motor presets other than the Kraken X60 FOC** use vendor datasheet figures (stall torque, stall
  current, free speed) and haven't been checked against dyno data. The FOC numbers are the core's.
- **No tab reordering, no multi-select, and no numeric editing of obstacle corners** (drag them on the
  field).
