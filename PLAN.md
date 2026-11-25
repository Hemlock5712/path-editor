# Myelin: plan

The rewrite this file planned is done. What is current lives elsewhere:

- **The design as built:** [lib/README.md](lib/README.md), "How it works".
- **What to try next in the controller:** [RESEARCH.md](RESEARCH.md).
- **The numbers:** [bench/RESULTS.md](bench/RESULTS.md) and [e2e/RESULTS.md](e2e/RESULTS.md).

This file keeps the reasons the rewrite was done the way it was, the speed budget it holds itself to, and the one
piece planned and not started. The goal has not changed: a follower that finishes more accurately and sooner than
PathPlanner, Choreo and BLine, hands off from a path to aligning on a target with no seam, builds paths around
obstacles during a match, and costs nothing to flip for red. Every claim gets checked in the bench before it goes on
a robot.

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
  - **Clean structure:** a follower core that knows nothing about the command framework, with a thin
    Commands adapter on top. It also fails safely on NaN inputs, and has execution-contract tests.
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

## Speed budget
| | Target |
|---|---|
| Flipping for red | 0 ms |
| Path work when auto starts (all paths prebuilt, flip is a transform) | 0 ms |
| Loading and precomputing every path at boot | no hard limit; done before the match starts |
| Rebuilding the tail when the goal moves | < 1 ms |
| Pathfinding plus spline plus profile | < 5 ms |
| Follower per loop | < 0.2 ms |

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
- `PathLibrary` builds one Commands v3 command at boot, so pressing go still does no path work.
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
