# Myelin: plan

**Simplified 2026-10-23.** The library was cut back to what Swerve School lesson 4 teaches; lib/README.md's "How it
works" is the current design. Removed after being built: adaptive grip and the slip package (slip detection,
slip-aware odometry), the raw and path-frame overrides, re-planning the start from the measured speed, rejoin pieces
after a shove, Choreo trajectory import, and 24 follower settings (now constants). The bench kept only what the Bench
page shows and the on-the-fly head-to-head (bench/RESULTS.md).

A full rewrite of the path follower, the path maths and the editor. `master` keeps the old code for reference.
The goal is a follower that finishes more accurately and sooner than PathPlanner, Choreo, BLine and CTRE.
It should hand off from a path to aligning on a target with no seam, build paths around obstacles during a
match, and cost nothing to flip for red. Every claim gets checked in the bench before it goes on a robot.

## What we learned from the old code (master) and from BLine

**Old path-editor (master)**
- **The stall on red.** `PathData.mirrorForRedAlliance()` returns a new object. The spline and profile caches
  are `IdentityHashMap`s, so the flipped copy misses them and rebuilds both at 1,000 samples per metre in
  `autonomousInit`. The curvature-smoothing loop alone is about 900 comparisons per sample. `withStartingPoint()`
  has the same problem.
- **Catmull-Rom curves are only C1.** Curvature jumps at every waypoint, so the profile has to smear it.
- **The follower aims at a look-ahead point,** which cuts corners, and then cross-track PD fights that. Speed is
  read at the robot's position with no allowance for lag. Rotation isn't in the plan (it's capped at 30% after the
  fact). There's no final pose convergence and no handoff to align.
- **The robot code drifted from itself.** `FollowPath` calls `getOmega`, but `RotationSupplier` defines
  `getTargetHeading`.
- **Paths compile to Java,** so every path edit needs a redeploy.
- **Keep:** progress measured by distance along the path, a bounded projection window, forward and backward
  passes using grip and motor torque, rotation and constraint zones, waypoint flags, centre-of-rotation zones and
  per-axis overrides.

**BLine (Lib on `wpilib-2027` at 4402250, Web on `bline-2027`; BSD-3)**
- **Its paths are polylines.** The robot hands off to the next point inside a radius around each one. Speed is
  P on the distance left, aimed at the next point. There's no velocity profile. Rotation is a separate PID.
  Translation and rotation have separate rate limits.
- **Our drive-to-pose race measured this law:** with the README gains it never settles, and once tuned it is
  0.1-1.7 s slower than ours.
- **The editor's sim brakes with `sqrt(v² + 2·a·d)`, but the robot uses P on distance,** so the time the editor
  shows is better than what the robot does.
- **The editor is tied to the polyline model.** 59 files import `PathModel`. It has no NT4, no obstacles and no
  flipping.
- **We won't fork it.** We borrow these instead:
  - **Rolling end:** a path with a minimum speed above zero hands its velocity straight to the next command.
  - **Placement and events:** rotation targets and events sit at positions along the path. Skipped events fire
    when the path completes and are cancelled if it's interrupted.
  - **Rotation override,** with an option to respect or bypass the limits.
  - **Clean structure:** a follower core that knows nothing about the command framework, with v2 and v3
    adapters. It also fails safely on NaN inputs, and has execution-contract tests.
  - **Flip and mirror** both supported.
  - **Fixed-precision JSON,** so git diffs stay small. Runtime files are kept separate from editor-only
    metadata.
  - **Linked targets:** named field points shared across paths.
  - **Freehand stroke → simplified control points.**
  - **Swept-bumper collision checks.**
  - **A corridor-deviation metric** for checking paths.
  - **CMA-ES tuning against a sim** (we'd use it for gains).
  - **Web Worker previews** that run one job at a time with one replaceable pending input.
  - **Tauri's safe file writes.**
- **We'll also read BLine path JSON,** so the bench runs the same courses through both followers.

**Drive-to-pose race (`bench/reference/race_drive_to_pose_results.md`)**
- **Our stopping law wins:** `v = sqrt(2a(d − v·0.05))`, braking with a 50 ms lag margin. It has no overshoot
  and settles first.
- **But `AccelerationLimiter` MU = 1.0 sits right at the grip limit,** so launches spin the wheels and odometry
  ends about 4 cm off. Grip has to be a measured setting, with a default around 0.7 g.

## Design

### Geometry
- Use quintic Hermite splines: curvature is continuous, so no smoothing hack is needed. Each waypoint has a
  pose plus optional tangent handles.
- Build an arc-length table with adaptive sampling (about 2-5 cm steps, with exact curvature). Every query works
  in metres along the path (`s`).
- Headings are a separate profile over `s`, made from rotation targets, face-point zones or face-forward.

### Velocity profile
- A forward pass and a backward pass over `s`, limited by:
  - the grip circle (measured traction),
  - the motor torque-speed curve,
  - constraint zones,
  - the extra wheel speed spin needs (module speed = |v + ω × r|), so planned heading changes take wheel speed
    away from translation and the plan knows it.
- The pass runs in O(N), with N about 500 for a 15 m path.

### Follower: one controller from path to pose
- **Along the path:** speed = min(profile(s + v·latency), the DriveToPose brake on the distance left).
- **Across the path:** PD on cross-track error, plus a v²κ feed-forward for the sideways acceleration.
- **Heading:** the planned heading at `s`, plus feedback.
- **Near the end:** the reference becomes the goal pose and the law is DriveToPose's.
- **Every output goes through one limiter that knows the grip budget,** with translation and rotation sharing
  it.
- **Progress is measured by position:** a bounded projection window around the last `s`. A hit robot just
  re-projects.

### Flipping
The path is never rebuilt. With WPILib 2027's origin at the centre, each loop converts the measured pose into
the blue frame, follows the blue path, and converts the output speeds back:
- **Half-turn fields:** (x, y) → (−x, −y), θ → θ + π.
- **Mirror fields:** x → −x, θ → π − θ.

Neither changes the arc length, |κ| or the profile.

### Changing the path while it runs (aligning to tags and game pieces)
There are three layers, from the most to the least preferred.

1. **Moving goal (the main tool).**
   - Any waypoint, usually the last, can be bound to `Supplier<Optional<Pose2d>>`. Examples: an AprilTag frame
     ("tag 18, 0.45 m out, facing it") or a game piece from the detector camera.
   - When the goal moves, only the tail is rebuilt: the stretch from the robot's current `s` to the goal, with
     its profile. That's a sub-millisecond job.
   - The speed plan stays valid for the new curve, the heading can face the target, and the ending is the same
     DriveToPose law.
   - For intakes, give the end a minimum speed, so the robot drives through the piece instead of stopping on it.
   - Estimates are latched in the odometry frame, as DriveToTag does, so vision pose jumps don't move the goal.
     The last estimate is held once the piece leaves the camera's view up close.
2. **Path-frame offsets** (`lateralOffset`, `alongTrackScale`) and 3. **field-axis overrides** (`overrideX/Y/Omega`)
   were built, then removed on 2026-10-23: nothing used them, and a moving goal is the way to steer a path's end.

### Pathfinding during a match
- Static obstacles come from a field file as polygons, grown by the robot's half-diagonal plus a margin.
- A visibility graph and A* find the exact shortest route around them.
- The route is then fitted with a G2 spline inside the free corridor, checked with swept bumpers, and
  profiled.
- **Target:** under 5 ms on the robot for a 10 m route with about 10 obstacles.

### Files
- **Paths:** `deploy/paths/<name>.path.json`, with a `schema` version, written with fixed precision.
- **Editor metadata:** an editor-only `.pathproject.json` next to them.
- **Field obstacles:** `deploy/paths/field.json`.
- **Code generation:** only a `PathName` enum.
- **On the robot:** everything loads and precomputes at boot, plainly, in robot init (no background thread). Boot
  has plenty of time (the robot, cameras and field link take far longer); the rule is that pressing go does zero
  path work.

### One maths library, two languages
- The Java library (`lib/`) is the reference version. The editor uses a TypeScript port of it.
- Both must reproduce `fixtures/*.json` (points, curvature and the profile along `s`) to 1e-9.

### Speed budget
| | Target |
|---|---|
| Flipping for red | 0 ms |
| Path work when auto starts (all paths prebuilt, flip is a transform) | 0 ms |
| Loading and precomputing every path at boot | no hard limit; done before the match starts |
| Rebuilding the tail when the goal moves | < 1 ms |
| Pathfinding plus spline plus profile | < 5 ms |
| Follower per loop | < 0.2 ms |

## The bench (`bench/`)
- **Plant:** `alignsim` (copied from Precision-Videos `record/v2-race/sim`), with 500 Hz stick-slip tyres, the
  Kraken FOC curve, steering lag, and odometry that over-reads when wheels slip.
- **Fairness:** every method gets the same maximum speed, acceleration and rotation limits, and every method is
  also run at its best tuned gains.
- **Courses:** straight, S-curve, hairpin, slalom through gates, a long auto-style route with rotation targets, a
  moving start, a hit mid-path (an impulse), low grip, end on a tag with a goal that moves, drive through a
  game piece, and pathfinding around obstacles.
- **Metrics:**
  - against the true pose: cross-track RMS / max, final position and heading error, overshoot
  - time to the end, time settled (2 cm / 1° held 0.25 s), time to aligned after a goal update
  - wheel-slip ms
  - generation time (offline and on the robot), flip cost, pathfinding time and route length
- **Methods:**
  - Ours (v2)
  - old path-editor `FollowPath`
  - BLine (port of the `wpilib-2027` follower)
  - PathPlanner (port of `PathPlannerTrajectory` and `PPHolonomicDriveController` from the `2027` branch)
  - Choreo (trajectories from its solver, followed time-sampled)
  - Choreo geometry with our follower: the `.traj` resampled by distance along the path, with Choreo's speed as
    the profile cap, followed by position and finished with the DriveToPose law. If it wins on time and keeps our
    accuracy, importing Choreo paths becomes a supported option for offline autos. Paths built during a match
    always use our own planner, because Choreo's solver can take seconds and doesn't always converge.
  - CTRE LinearPath (point-to-point courses only)
- `reference/race_drive_to_pose_results.md` is the earlier point-to-point race's result (its script was retired
  2026-10-23).

## Phases
1. **Bench:** courses, metrics, the competitors' followers, baseline numbers.
2. **Core** (Java plus the TS port plus fixtures): spline, arc length, profile.
3. **Follower:** DriveToPose ending, moving goal, overrides, flip/mirror, events, rolling end, v2/v3 adapters.
4. **Pathfinding.**
5. **Editor:**
   - Keep from master: the canvas, zones, charts and playback.
   - Add: obstacles, live robot over NT4, a sim preview using the shared core, an optimiser, and loading the
     bench comparison.
   - Named events are authored in the editor at positions along the path and saved in the path file (today they
     are set in code only).
6. **End-to-end tests, every path start to finish:** author each path in the editor (including named events along
   it), save the file, load it with `PathLibrary.loadAll()`, and run it with the real Java follower and the WPILib
   adapters in simulation on blue and red. Check that it reaches the end within tolerance, that every named event
   fires once, in order, at its authored position (within a few cm), that skipped events fire on completion,
   that an interrupt cancels pending ones, and that what the editor previewed (time, speed, event times) matches
   what the robot code did.
7. **Robot runs** on the same courses, with vision on and off.

## Later (planned, not started): autos as a graph

Joe asked for this to be planned only. What we have gets made solid first.

**The idea.** Paths are the building blocks, and an auto joins them into a flowchart that students can read.
PathPlanner's `new-path` branch (path2, af86ef2) does something similar: waypoint graphs with branches chosen at runtime by
project-wide named conditions.

**Nodes** (steps):
- follow a path;
- pathfind on the fly to a pose or named point;
- pathfind, then follow a path;
- run a named command;
- wait;
- parallel group (e.g. a path plus the intake).

**Edges:**
- plain "then";
- a decision on a named condition (yes/no), with "wait up to X s for true", debounce, and what to do on timeout.

**Example:**

    Start -> Path "ToPiece1" (intake event)
          -> hasPiece?
               yes -> Path "Piece1ToScore" (optimised) -> Command "Shoot"
               no  -> Pathfind to "Piece2" (on the fly) -> Path "Piece2ToScore" (optimised) -> Command "Shoot"

**Where things live:**
- The graph is data: `deploy/paths/<name>.auto.json`, authored in the editor.
- Robot code only binds meanings by name: `library.bindCondition("hasPiece", intake::hasPiece)`, like `bindEvent`.
- `PathLibrary` builds one Commands v2/v3 command at boot, so pressing go still does no path work.
- `unboundConditionNames()` catches typos.

**Why it suits us:** on-the-fly pathfinding joins a branch from wherever the robot really is. A missed pickup rarely
leaves the robot on the next path's planned start, and this is the hard part of branching autos elsewhere.

**Guard rails:**
- A forward-only flow to start: no arbitrary loops. Bounded "retry up to N" can come later.
- Each branch's total time is checked against the 15 s auto period.
- A straight auto stays one row of nodes, with no extra ceremony.
- Checks for dead ends, unbound names, and branch joins that don't connect.

**Order:**
1. Straight autos plus "plan across the chain". One joint speed profile over consecutive paths sets each handoff speed,
   and optionally the join directions. bench/chain_plan.py is the prototype: three_piece_chain 5.06 s -> 4.99 s, or
   4.77 s with free join directions.
2. Decision nodes with conditions, timeouts and debounce.
3. The flowchart view in an Autos tab, branch colours on the field, and per-branch times.
4. Whole-auto sim preview with condition toggles.
5. End-to-end tests of every branch through the WPILib adapters.
