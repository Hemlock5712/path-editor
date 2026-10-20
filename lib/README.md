# lib: the path maths and the follower

| Package | What | Depends on |
| --- | --- | --- |
| `frc.team5712.path.core` (phase 2) | geometry, heading and velocity profiles, frame flips, path files | nothing |
| `frc.team5712.path.follow` (phase 3) | `PathFollower`: one controller from path to pose | core, slip |
| `frc.team5712.path.slip` | `SlipDetector`, `SlipAwareOdometry` (optional), `SlipSignal` | core |
| `frc.team5712.path.plan` (phase 4) | `Pathfinder`: obstacle-avoiding paths during a match | core |
| `frc.team5712.path.tools` | `FixtureGen`, `ProfileCli`, `PlanCli`, `BenchServer` (the bench bridge) | core, follow, plan, slip |
| `frc.team5712.path.wpilib` (source set `src/wpilib`) | Commands v3 and v2 adapters, `PathLibrary`, telemetry | WPILib 2027 |

The core and the follower have no dependencies, so they test in seconds; the core ports line for
line to TypeScript (`editor/core/`).

## Running it (from the repo root)

| What | Command |
| --- | --- |
| Tests (prints timings) | `lib/gradlew -p lib test` |
| WPILib adapter tests | `lib/gradlew -p lib wpilibTest` (or `check` for everything) |
| Regenerate `fixtures/*.json` | `lib/gradlew -p lib fixtures` |
| Build the jar for the bench | `lib/gradlew -p lib jar` (core, follower and tools: `lib/build/libs/pathcore.jar`) |
| Bench bridge | `java -cp lib/build/libs/pathcore.jar frc.team5712.path.tools.BenchServer` (protocol below) |
| Time a pathfinding query (cold, then warm) | `java -cp lib/build/libs/pathcore.jar frc.team5712.path.tools.PlanCli field.json sx sy sdeg gx gy gdeg [vx vy] [--repeat 2000]` |
| Profile a path for the bench | `java -cp lib/build/libs/pathcore.jar frc.team5712.path.tools.ProfileCli my.path.json [--robot robot.json] [--step 0.05] [--grid 0.03]` |
| TS port tests | `cd editor/core && npm install && npm test` |

Gradle 9.4.1 comes from the WPILib permwrapper cache; it runs on the WPILib 2027 JDK (Java 25).
`ProfileCli` prints JSON on stdout: the inputs, `lengthMeters`, `totalTimeSeconds`, and columns
`s, x, y, tangentX, tangentY, curvature, dCurvatureDs, heading, headingPerMeter, velocity,
acceleration, time` (metres, radians, seconds). Exit code 2 with a message on stderr if anything
is wrong. A robot file is optional JSON with any of `RobotConfig`'s fields (see `RobotFile`).

## Design

**Geometry (`QuinticSpline`, `PathGeometry`).** Quintic Hermite pieces between waypoints. Each knot
has a unit tangent (given, or Bessel/parabolic for interior points, mirrored across the chord for
automatic ends) and a curvature κ. A piece's end derivatives are `V = a·d` and `A = a²·κ·n`, so both
pieces meeting at a knot have exactly curvature κ there: **G2 by construction**, whatever κ is.
`a` is the piece's arc-length estimate `chord · h / sin h` (h = half the end turn, capped at π/2)
times the waypoint's `tangentScale`. The knot curvatures minimise `Σ ∫|p'''|² du / L⁵`, one
tridiagonal solve. With these choices, points on a circle give that circle's constant curvature
(to 1e-3). In a prototype, peak curvature came out about 10-25 % lower than with zero or
circle-fit knot curvatures.

Arc length: each piece is split into sub-pieces with 5-point Gauss-Legendre lengths, and inside a
sub-piece `u(s)` is a cubic Hermite interpolant. Sub-pieces are halved until the interpolant is
within 0.01 mm at the sub-piece's middle and halving no longer changes the quadrature. The 16 m
example has 97 sub-pieces (the old code used 16,000 samples). Measured lookup error against an
independent 4,000-step integral: **0.023 mm** worst on the fixture paths, 0.086 mm on 40 random
paths with odd tangents (limit 0.5 mm). Queries by s return the exact spline point, unit tangent,
signed κ and dκ/ds at that `u`. Projection: a coarse scan of the window (5 cm; 10 cm for the global
search) then safeguarded Newton on `(p − q)·T = 0`, clamped to the window.

**Heading (`HeadingProfile`).** θ(s), dθ/ds and d²θ/ds². No targets: face the direction of travel.
Targets (waypoint-relative, as a fraction of the arc between waypoints, or in metres) are joined by
the minimum-jerk blend `10τ³ − 15τ⁴ + 6τ⁵` the short way round, holding the first and last.
Face-point zones pull the heading towards a field point, easing in and out over `blendMeters`
inside the zone.

**Velocity (`VelocityProfile`).** Forward and backward passes, O(N), on a 3 cm grid (547 points on the
16 m example), constant acceleration per cell so v(s), t(s) and s(t) are exact between grid points.
The model (also in the class comment):

- grip, for the worst wheel: `√(a² + (v²κ)²) + c·|α| ≤ μg`, with `α = a·θ' + v²·θ''`,
  `c = I/(m·r)` and μ = 0.7 by default;
- motors, speeding up: `a + c·|α| ≤` the Kraken X60 FOC push at the fastest wheel's speed
  `v·(1 + r|θ'|)` (12 V minus back-EMF, capped by the stator limit). The sideways `v²κ` comes from
  tyre side-grip, not torque. Braking: only the stator limit;
- wheel speed: `v·(1 + r|θ'|) ≤ v_wheel` (the motor's free speed unless set);
- path, zone and spin caps (`v·|θ'| ≤ ω_max`), and start and end speeds.

`|α| ≤ |a||θ'| + v²|θ''|` gives a closed form for the largest |a|. Each step keeps the limits at
both of its grid points via a two-round fixed point that keeps the answer from below. There's no
smoothing window: curvature is continuous, and the tests check the budget holds at every grid point
(exactly) and between them (within 2 %; the worst spot is inside a fast face-point blend).

**Flipping (`FieldFrame`).** `RED_HALF_TURN`: (x, y, θ) → (−x, −y, θ + π), (vx, vy, ω) → (−vx, −vy, ω).
`RED_MIRROR`: x → −x, θ → π − θ, vx → −vx, ω → −ω. Both are their own inverse. Tests build each fixture
path again from flipped inputs and match the transformed blue plan (position to 1e-8 m, κ with the
mirror's sign, heading, speeds and times to a few parts in 10⁹), then drive a small follower on red
twice, once converting to blue and once on the red-built path: the traces agree to 1e-6 m.

**Files (`PathFile`, `RobotFile`, `Json`).** `*.path.json` with `"schema": 1`, waypoints
(`x`, `y`, optional `tangentDegrees`, `tangentScale`), `rotationTargets` (`waypoint` or `meters`,
`degrees`), `facePointZones`, `constraintZones`, path-wide maxima and start/end speeds. Written to 5
decimals with trailing zeros dropped, keys in a fixed order, one object per line; unknown keys are
ignored. The TS writer prints the same characters (tested on `src/test/resources/example.path.json`).

**Fixtures.** `FixtureGen` writes six paths (straight, S-curve, hairpin, slalom with a slow zone,
the 16 m auto route, tangent scales with a moving start) to `fixtures/`, each with its inputs and
every quantity every 5 cm in shortest round-trip precision. A Java test fails if they are stale.
**The TS port reproduces all six exactly: worst deviation 0** (the bound is 1e-9). Trig goes
through `StrictMath` in Java and `Math` in V8, both fdlibm.

## The follower (phase 3)

`PathFollower` (`frc.team5712.path.follow`) is one controller from path to pose. Build the path in
robot init, make the follower there too, then `start()` when the path begins, `step()` each loop and
stop when `isFinished()`. `start()` does no path work: no building, no flipping, no copying (a test
counts geometry and profile builds across `start()` and a second of steps, on both alliances).

Each loop, in the blue frame:

1. **Frame.** The measured pose and velocity go into the blue frame through `FieldFrame`; the output
   comes back out. Overrides and goals are given in the robot's own frame and converted the same way.
2. **Progress.** `s` is the closest point within ±0.5 m of the last `s`. If that point is more than
   0.5 m away (a shove), the whole path is searched too and the nearer answer wins.
3. **Along the path.** The plan is read one latency (50 ms) ahead *in plan time*:
   `s⁺ = s_plan(t_plan(s) + τ)`. That is what gets a robot off a standing start, where the profile
   says 0 m/s. The speed is `min(v_plan(s⁺), sqrt(v_end² + 2·a·(d − v·τ)))`, which is DriveToPose's
   brake on the distance left `d`. The speed points along the tangent at `s⁺`, so the command turns
   into a curve early by `v²κτ`: the curvature feed-forward.
4. **Across the path.** PD on the cross-track error (P 5/s, D 0), pushing along the normal at `s`.
5. **Heading.** Feed-forward `v·dθ/ds` at `s⁺`, plus P 5/s on the planned heading at `s`.
6. **Near the end** (0.3 m to go), the reference becomes the goal pose and DriveToPose's law takes
   over: `min(v_max, sqrt(2a(d − vτ)), k·d)` straight at the goal, and the same law for heading. The
   ease-in line `k` is 12/s, against DriveToPose's 6/s. The bench sweep picked that value on its
   tuning seeds, and it sits inside the swept range. Both
   laws brake on the same distance with the same 6 m/s², so the hand-over has no seam.
7. **One limiter.** Every output goes through `OutputLimiter`. It caps spin acceleration, then caps
   the motors' push at the fastest wheel's speed when speeding up. Then `|a| + c·|α| ≤ μ·g`
   (c = I/(m·r), the profile's grip model), scaling translation and spin together. Its μ defaults
   to 1.0, above the plan's 0.7, so feedback has room. The phase 3 sweep preferred 2.0, which
   effectively switches the grip check off. With a perfect pose that saved 0.02 s per run. With
   wheel odometry it doubled wheel slip and left the moving start 19.8 cm off instead of 3.9 cm,
   because the robot outran its grip and the odometry over-read. So 1.0 ships.
8. **Finished.** Within 2 cm and 1°, measured slower than 5 cm/s, held 0.1 s. A rolling end (end
   speed above 0) never stops: it finishes as the robot passes the end, and `endCommand()` is the
   velocity to hand to the next follower (`start(..., handedCommand)`).

The other pieces:

- **Moving goal.** `bindGoal(Supplier<Optional<Pose>>)` binds the end of the path. When the goal
  moves more than 2 cm or 2°, only the tail is rebuilt: one quintic piece from the robot's progress
  to the goal. The piece's first curvature is fixed, so position, tangent and curvature all match at
  the join. The heading blends from the planned heading there to the goal's, and the profile starts
  at the planned speed. The approach direction turns with the goal (the old end tangent plus the
  goal's turn). An empty value holds the last goal. In the terminal phase the DriveToPose law just
  drives to the new goal. Latching estimates in the odometry frame is the supplier's job, as in
  DriveToTag.
- **Overrides.** `lateralOffset` (metres left of the path; it also moves the goal sideways) and
  `alongTrackScale` (0..1). Raw `overrideX/Y/Omega` take a supplier (NaN = off) and an
  `OverrideMode`: `RESPECT_LIMITS` replaces the wish before the limiter, `BYPASS` replaces the
  output after it.
- **Events.** `addEvent(name, s, action)` and `addEventAtWaypoint`. They fire in order as the robot
  passes them; any that haven't fired by a normal finish fire then; `cancel()` drops them.
  `onEvent(listener)` sees every name (the adapters map names to commands).
- **Robustness.** A NaN pose brakes within the limits instead of throwing.

All gains are in `FollowerConfig` (defaults from DriveToPose and the hybrid prototype).
`FollowerConfig.Builder.set(name, value)` sets a field by its name, which is how the bench passes
options.

### Slip: detection, adaptive grip and slip-aware odometry (`frc.team5712.path.slip`)

- **`SlipDetector`** takes one sample per odometry update: wheel speeds and angles, the gyro
  heading, optionally the accelerometer and the command. It weighs three kinds of evidence:
  - each module's residual against the rigid-body fit (median chassis velocity, with the gyro's yaw
    rate known);
  - a wheel turning faster than its commanded module speed;
  - the gap between the wheels' implied acceleration and the accelerometer's (both low-passed,
    30 ms).

  The output is a flag per module and a chassis confidence, which rises at once and decays over
  0.15 s. On the bench the IMU gap is what finds slip. In that plant all four wheels tend to slip
  together, so the per-module residuals rarely stand out.
- **Adaptive grip** in `PathFollower` (`withSlipSignal`, on by default; `adaptiveGrip(false)` turns
  it off). Once the confidence reaches 0.3, it takes the acceleration achieved, less 10 %, as the
  floor's μ (never below 0.3). The limiter's budget for speeding up, the brake law's deceleration
  and the planned speed (× sqrt(μ_slip / μ_plan)) all follow it. Braking keeps a limiter budget of
  at least μ 0.85 (`slipBrakeFriction`): the brake law plans with the low μ, so the robot starts
  braking early, but the limiter never forbids it to brake harder. Capping braking at the measured
  μ made the robot overshoot when the floor turned slick mid-run (up to 24.9 cm on the bench's grip
  drop). The μ it measured was below what a sliding tread still gives, so it braked too gently.
  Afterwards μ_slip climbs back at 0.05 μ/s, but never in the terminal phase. The profile is not
  rebuilt.
- **`SlipAwareOdometry`** is optional and replaces `SwerveDriveOdometry`:
  - The wheels' least-squares step down-weights slipping modules (weight 0.01).
  - On a clean run it equals WPILib's odometry exactly (tested to 1e-9).
  - While slipping, an IMU-driven velocity carries the pose. The IMU's bias is learnt while
    gripping. Each spell starts by rewinding the detection lag (60 ms) and replaying the stored IMU
    samples. A spell lasts at most 0.1 s with no bias learnt, growing to 1.5 s as the bias is
    learnt; after that the wheels take over again.
- **Tuning.** All thresholds were tuned on the bench's tuning seeds only: IMU gap 2.5 m/s²,
  follower confidence 0.3, recovery 0.05 μ/s. The IMU-gap threshold and the spell limits are where
  the sweep scored best after widening the grid once.

### WPILib adapters (`src/wpilib`)

These are compiled against WPILib 2027.0.0-alpha-7 from the installer's offline maven repo
(`C:/Users/Public/wpilib/2027_alpha7/maven`; `-PwpilibMaven=<dir>` points elsewhere). The build
skips them with a warning if the repo is missing.

- `PathLibrary.loadAll()` builds every `deploy/paths/*.path.json` (with `deploy/paths/robot.json` if
  present) in robot init. A bad file fails at boot.
- `FollowPathCommand` (Commands v3) runs the follower in its coroutine and forks event commands as
  children. `FollowPathCommandV2` (Commands v2) schedules event commands independently. Both read
  the alliance as a `FieldFrame` when the path starts, send zero at the end (except on a rolling
  end), and chain rolling ends with `continuingFrom(previous)`.
- `Conversions` covers `Pose2d` and `ChassisVelocities`, and the alliance as a frame.
  `PathTelemetry` logs the phase, progress, cross-track, reference and goal.
- `wpilibTest` runs both commands for real: v2 by hand, v3 under an independent scheduler, with the
  desktop HAL natives extracted from the same repo. Commands v3 needs
  `--add-opens java.base/jdk.internal.vm` and `java.base/java.lang`, which GradleRIO adds on a robot.

### Bench bridge protocol (`BenchServer`)

One JSON object per line on stdin, one reply per line on stdout. Errors reply
`{"error": "..."}` and the server carries on. Poses are `[x, y, heading]` (m, rad) and velocities
`[vx, vy, omega]` (m/s, rad/s, field-relative).

| Request | Reply |
| --- | --- |
| `{"op": "load", "path": <path file tree>, "robot": <robot file tree, optional>, "options": {<FollowerConfig field>: number}}`: builds the path and a follower (robot init) | `{"ok", "lengthMeters", "totalTimeSeconds", "buildMs"}` |
| `{"op": "reset", "t", "pose", "vel", "frame": "BLUE" \| "RED_HALF_TURN" \| "RED_MIRROR"}`: `start()` | `{"ok", "startMicros"}` |
| `{"op": "step", "t", "pose", "vel"}` | `{"cmd": [vx, vy, omega], "done", "phase", "s", "crossTrack", "micros"}` plus `"rebuilds"` and `"rebuildMicros"` after a tail rebuild |
| `{"op": "goal", "pose": [x, y, heading] \| null}`: the moving goal's next value (null = no estimate) | `{"ok": true}` |
| `{"op": "geometry", "step": 0.02}`: the built path, sampled | `{"x": [...], "y": [...]}` |
| `{"op": "pathfind", "field": <field file tree>, "start": pose, "vel": vel, "goal": pose}` plus optional `"robot"`, `"options"`, `"margin"`, `"maxSpeed"`, `"maxAccel"`: plans with our pathfinder and loads the result like `load` | `{"ok", "lengthMeters", "totalTimeSeconds", "planMs", "searchMs", "smoothMs", "profileMs", "routeLengthMeters", "straightLineMeters", "startSnapMeters", "goalSnapMeters", "refits", "clear", "minClearanceMeters", "waypoints"}`, plus `"bootMs"` when the pathfinder was (re)built |
| `{"op": "quit"}` | `{"ok": true}`, then exits |

`bench/followers/ours_v2.py` starts one server per bench worker process and keeps it for all that
worker's runs.

## Measured timings

This PC (Windows 11, JDK 25, Node 22), while the bench was running on other cores, so the spread
is wide. The path is the 16 m, 10-waypoint auto route with targets, a face-point zone and a slow
zone.

| | Java | TS (Node) |
| --- | --- | --- |
| Full build (spline + table + heading + profile), warm | 0.19 ms best, 0.2-0.45 ms mean | 0.6 ms |
| First build in a fresh JVM (class loading + interpreter) | 23-65 ms (one run at 150 ms under load) | 51 ms |
| Second build in that JVM (still interpreted) | 1.7-4.6 ms | |
| `Path.sample(s)` | 0.14-0.47 µs | 1.0 µs |
| Windowed projection (±0.5 m) | 0.95-1.6 µs | |
| `PathFollower.step()`, warm (16 m route, red) | 2.2 µs mean, 260 µs worst in 8,300 steps (GC/JIT) | |
| `PathFollower.start()`: warm / first call in a fresh JVM | 12 µs / 1.4 ms (class loading, no path work) | |
| Tail rebuild for a moved goal (8.8 m tail), warm | 186-265 µs mean, 109-116 µs best | |

The cold number is mostly the JVM loading and linking classes, which every path after the first
shares. See the notes below for what that means for the boot budget.

## Notes and departures from PLAN.md

- **Tangent handles are a direction plus a scale,** not handles in metres. `tangentScale`
  multiplies the arc-length estimate on both sides of the knot. That keeps uneven spacing loop-free
  and lets the curvature solve stay linear. The robot's heading is never on a waypoint: put a
  rotation target at that waypoint's index.
- **Distances are table distances.** The u(s) interpolant may stretch s by up to about one part in
  10,000 against the true arc, so finite differences of positions match the unit tangent to 1e-4,
  not 1e-9. Everything (projection, profile, heading) uses the same s, so it is self-consistent.
  An extra Newton step per lookup (five more speed evaluations) would make it exact, if that
  ever matters.
- **The grip model is more conservative than `AccelerationLimiter`'s.** That adds the spin push to
  the translation push for the worst wheel, where the limiter takes the hypotenuse. The speed
  ceiling uses the steady-speed case (a = 0). The bench should decide whether either costs time.
- **A 3 cm grid is within 0.1-0.9 % of a 1 mm grid's total time.** The larger figures are on paths
  with constraint zones, because a zone caps every grid cell it touches (it never starts late).
  Zone ends snap to the grid within a millionth of a cell. Before that, a zone starting exactly on
  a grid point (common on symmetric paths) could cap a different cell on red than on blue.
- **Face-point blends can be fast.** In the auto route the default 0.5 m blend turns the robot by
  over 100° and the profile slows to 0.7 m/s for it. That is correct, but the editor should show
  it and maybe default to a longer blend.
- **Boot time is not a constraint.** The first path costs 23-65 ms in a fresh JVM on this PC
  and about 2 ms each after that until the JIT warms up. The robot waits far longer than that for
  cameras and the field link, so paths are simply built in robot init. The real rule is
  that starting auto does no path work at all.

### Phase 3 departures from PLAN.md

- **The v²κ feed-forward is the look-ahead tangent.** The drivetrain takes velocity commands, so the
  curvature term is applied by pointing the speed along the tangent one latency ahead. That adds
  `v²κτ` sideways, rather than a separate acceleration feed-forward.
- **Cross-track is PD with D = 0 by default.** The bench sweeps didn't need the D term.
- **The limiter's grip (1.0) is above the plan's (0.7) on purpose,** so feedback has room. The bench
  sweep prefers it even looser, but that costs odometry accuracy (see bench/RESULTS.md), so the
  default stays 1.0.
- **A rebuilt tail drops the zones in that stretch.** Its heading blends from the planned heading to
  the goal's starting at zero spin rate, so ω can kink at the join if the old plan was mid-turn.
- **Events are set in code (`addEvent`), not yet in the path file.**
- **Slip handling is not in PLAN.md.** The detector, adaptive grip and slip-aware odometry were
  added after the phase 3 bench showed the low-grip weakness with odometry.
- **Commands v3 events are forked children,** so they end with the path command unless they finish
  first. v2 schedules them independently.
- **The first `start()` in a fresh JVM takes about 1.4 ms of class loading,** but no path work. One
  throwaway start/step in robot init would remove it.

## Pathfinding (phase 4)

`Pathfinder` (`frc.team5712.path.plan`) is built once at boot from a `FieldMap`. Calling `plan(pose, velocity,
goal)` returns a `PlanResult` whose `path()` is an ordinary `Path` for the `PathFollower`.

**Field file** (`deploy/paths/field.json`; `bench/fields/obstacles_v1.json` is the example and the test resource):
`{"field": {"length_m", "width_m"}, "obstacles": [{"name", "polygon": [[x, y], ...]}]}`. Metres, origin at the
field centre. Each polygon is replaced by its convex hull. `FieldMap.read(file)` loads it.

**Boot.**
- Each obstacle is grown by the bumpers' half-diagonal plus `marginMeters` (default 5 cm), so the result is safe
  in any heading.
- The grown shape is a Minkowski sum with a disc. Each corner's arc is cut into steps of at most 30°, and the arc
  points sit at r / cos(step / 2), so the polygon contains the exact sum and overshoots it by at most 2.3 cm.
- The walls are moved in by the same distance.
- The visibility graph joins every pair of grown corners whose straight segment misses every grown interior
  (Cyrus-Beck clipping, touching allowed). On the bench field: 83 corners.

**Query.**
1. A start or goal inside a grown obstacle, or too close to a wall, is snapped to the nearest free point. The path
   still starts at the robot and ends at the goal, through the snapped points, and the result reports the snap
   distances.
2. Start and goal are joined to the graph, and A* (Euclidean heuristic, lowest index on ties) finds the shortest
   route around the grown shapes.
3. The route's corners become quintic spline waypoints. The first tangent is the robot's velocity direction when it
   is moving faster than 0.1 m/s; the other tangents are automatic. The heading blends from the current heading to
   the goal's.
4. The oriented bumper rectangle is swept every 2 cm with the planned heading. Each stretch closer than the margin
   to an obstacle or a wall gets the route point under its worst spot added as a waypoint, and the spline is
   refit, up to 8 times. Snapped end stretches are not checked.
5. The core `VelocityProfile` is built: start speed = the robot's speed along the path, end at rest, 4 m/s and
   6 m/s² by default.

**Timings** (this PC, `PlannerPerformanceTest` and `PlanCli`, other work running):

| | |
| --- | --- |
| Query, warm, 9.2 m route through six obstacles | 1.9-2.3 ms median (best 1.2-1.6 ms, 95th percentile 4.8-8.1 ms) |
| The same, by part | search 0.08 ms, smoothing and sweep 1.7 ms, profile 0.15 ms |
| Queries on the other three bench routes | 0.47-1.0 ms median |
| First query in a fresh JVM | 17-35 ms on the bench routes (class loading and the interpreter) |
| Boot (grow + graph) in a fresh JVM | 32-135 ms |

The sweep dominates and could be cut, but the warm query is already inside PLAN.md's 5 ms. The roboRIO has not
been measured.

**Tests** (`PathfinderTest`):
- shortest-route lengths against an independent hull-wrap calculation (60 random single-obstacle cases), plus a
  straight line, symmetry and a blocked field;
- 300 random start/goal pairs whose routes never enter a grown obstacle;
- 300 random pairs (random headings, a third moving) whose swept bumpers keep the margin on the profiled path
  (34 needed refits, none failed, closest 5.1 cm);
- determinism, snapping, a moving start, the end heading, and file errors.

### Phase 4 departures from PLAN.md

- **The obstacles are grown by the half-diagonal, for any heading.** The swept check then uses the real heading.
  A heading-aware grow would allow tighter gaps but needs a configuration-space search.
- **Smoothing adds waypoints only.** Tangent magnitudes stay at the core's defaults. Inserting the route point under
  a violation always converges here because the corner-to-corner route itself keeps the margin.
- **Short hops and turns on the spot** plan in the core, not as a special case in the planner (see below).
- **Concave obstacles become their convex hulls,** which is conservative.
- **Not done yet:** dynamic obstacles, a WPILib command wrapper, and loading `deploy/paths/field.json` in
  `PathLibrary` (callers use `FieldMap.read`).
- **The margin stays 0.05 m by default.** On the bench's four routes with wheel odometry, 0.10 m and 0.15 m
  shut gaps and forced detours up to three times longer, and still touched once the odometry drifted (see
  bench/RESULTS.md). Vision fixes the contacts; a wider berth doesn't.

### Short hops with big turns (core)

The heading is tied to s, so a turn that needs more time than the translation slows the translation down. Two
changes make that work for any hop:

- **Bisection in the step.** The velocity profile's step used to solve `v = f(v)` with two fixed-point rounds kept
  from below. Where the budget falls steeply with speed (a fast turn over a few centimetres) the rounds never closed
  and the plan stalled: 0.2 m with an 80° turn could not be built. Now, when the rounds leave a gap, the step
  bisects inside their bracket. It stays O(N); other paths move by about 1e-5.
- **Spin acceleration cap.** `RobotConfig.maxAngularRadPerSecSquared` (optional; +∞ by default, and
  `"maxAngularRadPerSecSquared"` in the robot file) adds `|a·θ' + v²·θ''| ≤ α_max` to the budget and
  `v ≤ √(α_max/|θ''|)` to the ceiling, next to the existing `v·|θ'| ≤ ω_max`.

Every path also gets at least 16 grid cells. Two waypoints at the same place are a turn on the spot, planned on a
1 mm segment (`Path.PURE_ROTATION_METERS`) that ends exactly on the point along the final heading.

With 540°/s and 720°/s² (`ShortHopTest`):

| Hop | Time | Minimum-jerk floor |
| --- | --- | --- |
| 0.2 m, 80° | 0.84 s | 0.80 s |
| 0.05 m, 180° | 1.27 s | 1.20 s |
| 0 m, 90° | 0.90 s | 0.85 s |

Peak spin acceleration is within 1 % of the cap. The TS port matches, with new `short_hop` and `pure_rotation`
fixtures; parity is still exact.
