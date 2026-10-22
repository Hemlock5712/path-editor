# lib: the path maths and the follower

| Package | What | Depends on |
| --- | --- | --- |
| `frc.team5712.path.core` (phase 2) | geometry, heading and velocity profiles, frame flips, path files | nothing |
| `frc.team5712.path.follow` (phase 3) | `PathFollower`: one controller from path to pose | core, slip |
| `frc.team5712.path.slip` | `SlipDetector`, `SlipAwareOdometry` (optional), `SlipSignal` | core |
| `frc.team5712.path.plan` (phase 4) | `Pathfinder`: obstacle-avoiding paths during a match; `OnTheFly`: plan, follow, re-plan | core |
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

- traction, for the worst wheel: `√(a² + (v²κ)²) + c·|α| ≤ A`, with `α = a·θ' + v²·θ''`,
  `c = I/(m·r)` and `A` the traction limit, by the robot file's `tractionRule`:
  - `"equalPush"` (the default; old robot files without the key get it too): `A = 0.9 · μ·g / (1 + μ·h/x)`.
    Swerve velocity control pushes every wheel about equally, so the wheels that weight transfer unloads
    slip first;
  - `"tipOrGrip"` (opt-in, Precision-Drive's AccelerationLimiter): `A = 0.9 · min(μ·g, g·x/h)`, whichever
    comes first, the tread's grip or tipping over.

  The defaults are SDS MK5 Spike Grip (μ 1.5), the centre of mass 0.2 m up (to be measured) and
  modules at ±0.29 m (x = 0.29 m), giving `tipOrGrip` 12.8 m/s² (tipping first) and `equalPush`
  6.5 m/s². With Joe's ±0.254 m modules they are 11.2 and 6.1 m/s². The other robot-file keys are
  `frictionCoefficient`, `centerOfMassHeightMeters` (0 = no load transfer: both rules give 0.9·μ·g) and
  the optional `tipDistanceMeters`. The last is min(|module x|, |module y|), one value for every
  direction, and defaults to a square base's `moduleRadiusMeters / √2`.
  `RobotConfig.tipOrGripMetersPerSecondSquared()`, `equalPushMetersPerSecondSquared()` and
  `tractionLimit()` (grip, tipping or equal push) give the editor both values and which one binds. The
  plan and the limiter (below) use the same `A`;
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

**Files (`PathFile`, `RobotFile`, `Json`).** `*.path.json` with `"schema": 2`, waypoints
(`x`, `y`, optional `tangentDegrees`, `tangentScale`), `rotationTargets` (`waypoint` or `meters`,
`degrees`), `facePointZones`, `constraintZones`, `events` (`name`, and `atWaypoint` or `atMeters`),
path-wide maxima and start/end speeds. Written to 5 decimals with trailing zeros dropped, keys in a
fixed order, one object per line; unknown keys are ignored. Schema 1 files (no events) still read;
writing always gives schema 2. The TS writer prints the same characters: tested on
`src/test/resources/example.path.json`, and on the 40 pseudo-random files in
`fixtures/files/path_files.json`, which `PathFileSamples` and its TS port build from the same LCG
(every key, awkward event names, numbers that need rounding). The robot file may also carry
`bumperLengthMeters` and `bumperWidthMeters`; `RobotFile` ignores them and
`PlannerConfig.fromRobotFile` reads them.

**Fixtures.** `FixtureGen` writes six paths (straight, S-curve, hairpin, slalom with a slow zone,
the 16 m auto route, tangent scales with a moving start) to `fixtures/`, each with its inputs and
every quantity every 5 cm in shortest round-trip precision. A Java test fails if they are stale.
**The TS port reproduces all six exactly: worst deviation 0** (the bound is 1e-9). Trig goes
through `StrictMath` in Java and `Math` in V8, both fdlibm.

## The follower (phase 3)

`PathFollower` (`frc.team5712.path.follow`) is one controller from path to pose. Build the path in
robot init, make the follower there too, then `start()` when the path begins, `step()` each loop and
stop when `isFinished()`. `start()` does no path work: no building, no flipping, no copying (a test
counts geometry and profile builds across `start()` and a second of steps, on both alliances). The
one exception is `replanStartFromMeasuredVelocity`, on by default. A path planned from rest, started with
the robot already moving (not backwards along the path), gets its speeds re-planned from the measured
speed, as PathPlanner does. Only the profile is rebuilt, never the curve: 0.2-0.3 ms warm for the 16 m
route on this PC, less on shorter paths. At rest it does nothing. It also does nothing on paths that plan
their own start speed (rolling hand-overs, on-the-fly paths). Under equalPush on the bench it changes only
the moving start: 2.39 → 2.32 s with a perfect pose, wheel slip 1.69 → 1.06 s, and 3.6 → 3.4 cm end error
with odometry. three_piece_chain and the courses that start at rest give the same runs to the bit.

Each loop, in the blue frame:

1. **Frame.** The measured pose and velocity go into the blue frame through `FieldFrame`; the output
   comes back out. Overrides and goals are given in the robot's own frame and converted the same way.
2. **Progress.** `s` is the closest point within ±0.5 m of the last `s`. If that point is more than
   0.5 m away (a shove), the whole path is searched too and the nearer answer wins.
3. **Along the path.** The plan is read one latency (50 ms) ahead *in plan time*:
   `s⁺ = s_plan(t_plan(s) + τ)`. That is what gets a robot off a standing start, where the profile
   says 0 m/s. The speed is `min(v_plan(s⁺), sqrt(v_end² + 2·a·(d − v·τ)))`, which is DriveToPose's
   brake on the distance left `d`. The speed points along the tangent at `s⁺`, so the command turns
   into a curve early by `v²κτ`: the curvature feed-forward. On a path that starts moving (a rolling
   hand-over), the read-ahead grows from 0 to `τ` over the first 0.3 s. The last path's command led into its own
   end, not into this one speeding up, and taking the lead `a·τ` in one loop slipped all four wheels at the
   hand-over (bench three_piece_chain: wheel slip 600 → 330 ms, same time).
4. **Across the path.** PD on the cross-track error (P 5/s, D 0), pushing along the normal at `s`.
   Knocked more than 0.4 m off (`rejoinAboveMeters`), the follower builds a piece back onto the path:
   from the robot's pose, along the way it is really going (from the pose's change; after a hit the
   wheels slide, so their velocity misleads), at that speed, meeting the path 1 m ahead with matching
   position, tangent and curvature. Near the end it goes straight for the goal instead. The join has to
   be ahead of the robot, and at most one piece is tried each 0.5 s. Progress, events and the finish
   carry on in the path's own metres.
5. **Heading.** Feed-forward `v·dθ/ds` at `s⁺`, plus P 5/s on the planned heading at `s`.
6. **Near the end** (0.3 m to go, and no more than `terminalTurnRadians`, 2°, left in the plan's
   turn), the reference becomes the goal pose and DriveToPose's law takes over: `min(v_max, sqrt(2a(d − vτ)), k·d)` straight at the goal, and the same law for heading. The
   ease-in line `k` is 12/s, against DriveToPose's 6/s. The bench sweep picked that value on its
   tuning seeds, and it sits inside the swept range. Both
   laws brake on the same distance with the same 6 m/s², so the hand-over has no seam.
7. **One limiter.** Every output goes through `OutputLimiter`. It caps spin acceleration, then caps
   the motors' push at the fastest wheel's speed when speeding up. Then `|a| + c·|α| ≤ A`
   (c = I/(m·r), the profile's model), scaling translation and spin together. `A` is the robot's
   traction limit, the same as the plan's (`limiterFrictionCoefficient` NaN, the default; a number
   sets μ·g instead). Before the μ 1.5 change the plan used μ 0.7 and the limiter 1.0 on a 1.15
   floor; see "Traction: μ 1.5 and tipping" below for why one limit for both now.
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
- **Events.** The path file's `events` are registered when the follower is made; code can add more
  with `addEvent(name, s, action)` and `addEventAtWaypoint`. They fire in order, each on the loop
  nearest to passing it (less than half a loop's progress ahead), so the error is at most half a loop
  of travel either side. Events at the end of a path that stops fire when the robot gets inside the
  finish band (2 cm and 1°), not after it settles. Any that haven't fired by a normal finish fire
  then; `cancel()` drops them. `onEvent(listener)` adds a listener that sees every name (the adapters
  add theirs to map names to commands). `eventNames()` and `eventMeters()` list them in firing order.
- **Turns on the spot.** A path whose two waypoints coincide is 1 mm long, so progress comes from the
  heading: where the planned heading reaches the measured one, never going back. The follower spins
  as the plan does and holds the point.
- **Robustness.** A NaN pose brakes within the limits instead of throwing.

All gains are in `FollowerConfig` (defaults from DriveToPose and the hybrid prototype).
`FollowerConfig.Builder.set(name, value)` sets a field by its name, which is how the bench passes
options.

### Traction: μ 1.5 and tipping

The robot runs SDS MK5 Spike Grip, μ 1.5. Acceleration and cornering are held to a traction limit `A`
(`RobotConfig.tractionMetersPerSecondSquared`). By default that's the equal-push rule,
`0.9 · μ·g / (1 + μ·h/x)`: 6.5 m/s² for the defaults, 6.1 m/s² with Joe's ±0.254 m modules. The other rule,
`tipOrGrip`, is 90 % of whichever comes first, grip or tipping, `0.9 · min(μ·g, g·x/h)`. That's 12.8 and
11.2 m/s².

Joe chose `equalPush` from bench-builder's re-baseline of ours under both rules (240 A plant, 1.5 / 1.3,
centre of mass 0.2 m, bench/out/rebaseline/rules.md). Under wheel odometry, equalPush:
- ends 8.5 cm off on average against 11.2 cm;
- settles 24 runs of 90 against 17;
- on the grip drop, ends 4.2 cm off against 10.2 cm.

With a perfect pose `tipOrGrip` is faster on most clean courses, because the loaded wheels make up for the
slipping light ones. That costs wheel slip and so odometry. `tipOrGrip` is worth trying once the centre of
mass's height is measured and slip has been checked on the real robot, with vision correcting the pose.
The planner's grip circle and the follower's output limiter both use `A`, so the editor's plan is what
the robot will try. Before this change they were separate: the plan used μ 0.7 and the limiter 1.0, on a
1.15 / 1.0 bench floor. Scaled to the 1.5 tread, that 0.7 plan would be 0.7 of `A` and the limiter about
`A`.

Whether the plan needs its own margin under `A` was checked on the bench plant set to 1.5 / 1.3, low grip
0.91 / 0.79, and the centre of mass 0.2 m up (weight transfer on). The figures below use the `tipOrGrip` rule. They are ours over all 16
courses that stop, 10 seeds, perfect pose: the sum of the settle times, and the mean wheel slip per run.

| plan at | as shipped (adaptive grip) | adaptive grip off |
|---|---|---|
| 1.0 · A | 45.4 s, slip 1.58 s | 41.5 s, slip 2.55 s |
| 0.85 · A | 43.6 s, slip 1.77 s | 41.8 s, slip 2.34 s |
| 0.7 · A | 43.6 s, slip 1.70 s | 42.5 s, slip 2.06 s |
| 0.55 · A | 44.5 s, slip 1.33 s | 44.0 s, slip 1.55 s |
| old: μ 0.7 plan, limiter 1.0, on the 1.15 floor without weight transfer | 44.9 s, slip 0.58 s | 44.8 s, slip 1.14 s |

No fraction wins outright, so the plan uses `A` itself, with no extra margin. Most of the slip comes from
weight transfer, not from cornering:
- On this plant the straight 3 m slips for 0.34 s at the path's 6 m/s², against none with the centre of
  mass at 0.
- Every module pushes about the same, so the wheels that unload slip first. That happens at about
  `μ·g / (1 + μ·h/x)`: 7.2 m/s² static for these numbers, well under `tipOrGrip`'s 12.8. bench-builder
  found the same, and that is the `equalPush` rule.
- Planning above that is deliberate under `tipOrGrip`: the loaded wheels make up for a slipping light one
  while the stators have headroom, at some cost in odometry. A real centre-of-mass height and a slip check
  on the real robot should decide between the two.

Adaptive grip used to cost time at `1.0 · A` (45.4 s against 41.5 s off), because it read the light wheels
letting go as a slick floor. Now it ignores slip that load transfer explains (see Slip, below). The figures
are from bench-builder's 240 A plant (1.5 / 1.3, centre of mass 0.2 m), all 26 courses, 10 evaluation seeds
(tuned on 100-102), ours with adaptive grip on and off:

| | tipOrGrip: on / off | equalPush: on / off | old adaptive grip (tipOrGrip) |
|---|---|---|---|
| 21 clean courses, perfect pose, summed settle | 53.7 / 52.8 s | 63.0 / 63.2 s | 58.4 s |
| clean runs where adaptive grip acted (false positives) | 14 / 190 | 0 / 190 | 152 / 190 |
| low grip, wheel odometry, end error | 4.1 / 8.0 cm | 4.1 / 4.1 cm (no slip to react to) | 4.1 cm |
| grip drop, perfect pose, settle | 2.06 / 2.22 s | 2.22 / 2.22 s | 2.28 s |
| grip drop, slip-aware odometry, end error | 6.7 / 7.4 cm | 4.2 / 4.2 cm | 4.2 cm |

The false positives left under `tipOrGrip` are mostly launches, where the detector flags slip at 60 ms,
before the filtered command has risen. Holding the command's peak instead removes most of them. But low
grip is caught the same way, at its launch, so that costs the low-grip win (8.3 cm), and it wasn't taken.
Under `equalPush` the plan stays under the onset, so adaptive grip never acts on these courses.

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
  floor's μ (never below 0.39). It ignores slip the tread's own weight transfer explains, though.
  Every wheel pushes about the same, so on a good floor the light wheels let go once the robot asks for
  the equal-push onset, `μ·g / (1 + μ·h/x)` (7.2 m/s² for the defaults). Slip in a spell that has asked
  for at least 0.8 of that (`expectedSlipShare`) changes nothing, unless the wheels are spinning up at least
  3.5 m/s² faster than the IMU says the chassis is (`slideGapMetersPerSecondSquared`). That means the loaded
  wheels are sliding too. On the bench that gap stays under about 3 m/s² for load-transfer slip on a good
  floor, and is 3.4-7.8 m/s² when the whole robot slides on a slick one (`SlipSignal.imuGapMetersPerSecondSquared`).
  Expected slip early in a spell also undoes what the spell's first samples latched, since the filtered
  acceleration lags the slip. Slip while asking for less than the onset means a slicker floor. The limiter's budget for speeding up, the brake law's deceleration
  and the planned speed (× sqrt(μ_slip / μ_plan), μ_plan = A/g) all follow it. Braking keeps a limiter budget of
  at least μ 1.11 (`slipBrakeFriction`): the brake law plans with the low μ, so the robot starts
  braking early, but the limiter never forbids it to brake harder. Capping braking at the measured
  μ made the robot overshoot when the floor turned slick mid-run (up to 24.9 cm on the bench's grip
  drop). The μ it measured was below what a sliding tread still gives, so it braked too gently.
  Afterwards μ_slip climbs back at 0.065 μ/s, but never in the terminal phase. The profile is not
  rebuilt. The three μ values were 0.3, 0.85 and 0.05 μ/s, tuned on a 1.15 / 1.0 floor; they are scaled
  with the tread (× 1.5 / 1.15).
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

- **Names are constants.** The editor writes `PathEditorNames.java` into the robot project on every save
  (package set in its Settings > Project; `frc.robot` by default): three enums, `Paths`, `Events` and `Points`,
  each constant carrying its exact name and implementing `PathId`, `EventId` or `PointId`. Every method below
  that takes a name also takes one of these, so a misspelt or renamed path, event or point is a compile error:

  ```java
  import static frc.robot.PathEditorNames.*;

  PathLibrary paths = PathLibrary.loadAll();
  paths.bindEvent(Events.INTAKE_DOWN, intake.lower())
       .bindEvent(Events.SHOOT, shooter.shoot())
       .requireAllEventsBound();               // a forgotten event fails at boot
  Command toReef = otf.follow(drive, Paths.TO_REEF);   // or new FollowPathCommand(Paths.TO_REEF, drive, ...)
  Command toDepot = otf.toPoint(drive, Points.DEPOT, Rotation2d.k180deg);
  ```

  The String versions stay, but the constants are the way to name things.
- `PathLibrary.loadAll()` builds every `deploy/paths/*.path.json` (with `deploy/paths/robot.json` if
  present) in robot init. A bad file fails at boot. `loadAll(File dir)` does the same for any folder. If the
  project's `PathEditorNames` is compiled in (the package named in `.pathproject.json`), it checks every
  constant against deploy: a stale class (deploy changed and the code wasn't rebuilt, or the other way round)
  fails at boot, naming the missing constants. `requireNames(Class)` runs the same check by hand.
- `PathLibrary.bindEvent(event, command)` (a v3 or a v2 command) binds an event to a command once for every
  path; a follow command picks the bindings up with `withEvents(library)`, and its own `onEvent` wins for the
  same name. `requireAllEventsBound()` fails if any event the files use has nothing bound;
  `unboundEventNames()` lists them.
- `FollowPathCommand` (Commands v3) runs the follower in its coroutine and forks event commands as
  children. `FollowPathCommandV2` (Commands v2) schedules event commands independently, and sends its
  first command from `initialize()`, as v3 does, so a path chained after a rolling end doesn't coast a
  loop. Both read
  the alliance as a `FieldFrame` when the path starts, send zero at the end (except on a rolling
  end), and chain rolling ends with `continuingFrom(previous)`.
- `Conversions` covers `Pose2d` and `ChassisVelocities`, and the alliance as a frame.
  `PathTelemetry` logs, under `/Telemetry/Path/<path>/` in NetworkTables, the phase, progress, the distance left,
  cross-track, the reference and goal (Pose2d structs, field frame), tail rebuilds and the events fired. The
  shared `/Telemetry/PathFollower/` table says which path is running (`Active`), its `EventsFired` and the
  `LastEvent`. The editor's live view reads these.
- `lib/gradlew -p lib nt4Server -Pnt4Port=5810 -Pnt4Seconds=60` runs `Nt4TelemetryServer` (in `src/e2eTest`). It is
  WPILib's ntcore as a real NT4 server, with a `PathFollower` in `FollowPathCommand` driving the simulated
  drivetrain in real time and publishing that telemetry, the robot's pose and ntcore's clock. The editor's
  `npm run test:nt4` tests its client against it. Note that `NetworkTablesJNI.now()` is in nanoseconds in 2027,
  while NT4 values carry microseconds on the wire.
- `wpilibTest` runs both commands for real: v2 by hand, v3 under an independent scheduler, with the
  desktop HAL natives extracted from the same repo. Commands v3 needs
  `--add-opens java.base/jdk.internal.vm` and `java.base/java.lang`, which GradleRIO adds on a robot.

### Bench bridge protocol (`BenchServer`)

One JSON object per line on stdin, one reply per line on stdout. Errors reply
`{"error": "..."}` and the server carries on. Poses are `[x, y, heading]` (m, rad) and velocities
`[vx, vy, omega]` (m/s, rad/s, field-relative).

| Request | Reply |
| --- | --- |
| `{"op": "load", "path": <path file tree>, "robot": <robot file tree, optional>, "options": {<FollowerConfig field>: number}}` plus optional `"keepSensing": true` (the next path of a chain: keep the run's sensing session) and `"startCurvature"` (fix the first waypoint's curvature, e.g. the last leg's `endCurvature`): builds the path and a follower (robot init) | `{"ok", "lengthMeters", "totalTimeSeconds", "buildMs", "endCurvature"}` |
| `{"op": "reset", "t", "pose", "vel", "frame": "BLUE" \| "RED_HALF_TURN" \| "RED_MIRROR"}`: `start()`, plus optional `"handed": [vx, vy, omega]` (the last path's `endCommand`, for a rolling hand-over; default the measured velocity) | `{"ok", "startMicros"}` plus `"startReplanMicros"` when the start re-planned the speeds |
| `{"op": "step", "t", "pose", "vel"}` | `{"cmd": [vx, vy, omega], "done", "phase", "s", "crossTrack", "micros"}` plus `"rebuilds"` and `"rebuildMicros"` after a tail rebuild, and `"rejoins"` and `"rejoinMicros"` after a rejoin |
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
| `start()` re-planning the speeds from 2 m/s (16.4 m route, `replanStartFromMeasuredVelocity`), warm | 296 µs mean, 203 µs best | |

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
  over 100° and the profile slows to 0.7 m/s for it (at the old μ 0.7; about 0.9 m/s at μ 1.5). That is correct, but the editor should show
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
- **The limiter and the plan now share one traction limit** (see below). Before the μ 1.5 change
  the limiter's grip (1.0) sat above the plan's (0.7) so feedback had room.
- **A rebuilt tail drops the zones in that stretch.** Its heading blends from the planned heading to
  the goal's starting at zero spin rate, so ω can kink at the join if the old plan was mid-turn.
- **Events were set in code only in phase 3.** Phase 5 added them to the path file (schema 2).
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
- **Not done yet:** dynamic obstacles (the field is static; a moving goal is supported, a moving robot in the way
  isn't). The command wrapper and loading field.json are done: see "On-the-fly paths on the robot" below.
- **The margin stays 0.05 m by default.** On the bench's four routes with wheel odometry, 0.10 m and 0.15 m
  shut gaps and forced detours up to three times longer, and still touched once the odometry drifted (see
  bench/RESULTS.md). Vision fixes the contacts; a wider berth doesn't.

### On-the-fly paths on the robot

`PathLibrary.loadAll()` also loads, from `deploy/paths`:

- `field.json`, the editor's format (`{"field": {"length_m", "width_m"}, "obstacles": [{"name", "polygon"}]}`).
  It builds the `Pathfinder` (grown obstacles and visibility graph) once, then warms it up with 40 throwaway plans so
  the JIT has compiled it before the first real one. On the desktop that's about 100 ms at boot on the 2026 field, 80 ms of it the
  warm-up. A bad file throws at boot, naming the file. With no field.json the field is empty (walls only) at the
  2026 size.
- The bumper size from `robot.json` (`bumperLengthMeters`, `bumperWidthMeters`).
- The editor's named points (`namedPoints` in `.pathproject.json`).

`Pathfinding` (`src/wpilib`) builds the commands, for Commands v3 and (with a `V2` suffix) v2:

```java
Pathfinding otf = new Pathfinding(paths, FollowerConfig.defaults(paths.robot()),
    drive::getPose, drive::getFieldVelocity, drive::driveFieldVelocity, FieldFrame.RED_HALF_TURN);
otf.to(drive, new Pose2d(-7.6, 3.1, Rotation2d.k180deg));   // blue frame, flipped on red
otf.toPoint(drive, Points.DEPOT, Rotation2d.k180deg);       // an editor named point (blue frame)
otf.toTarget(drive, vision::scoringPose);                   // real field frame, used as given
otf.thenFollow(drive, Paths.TO_REEF);                       // to the path's start, then the path
otf.follow(drive, Paths.TO_REEF);                           // the path on its own, its events bound
otf.to(drive, goal).withMovingGoal(vision::pieceIfSeen);     // the end follows a moving goal (real frame)
otf.follow(drive, "ToScore").withMovingGoal(vision::scoringPoseIfSeen); // a library path as drawn, its end on vision
```

- **Moving goals on library paths.** `follow(drive, name)` (and `followV2`) is the plain `FollowPathCommand` for a
  library path, an optimised one included, built with the same drivetrain, settings, clock and alliance. It plans
  nothing at go. `withMovingGoal(Supplier<Optional<Pose2d>>)` is on `FollowPathCommand` and `FollowPathCommandV2`
  themselves too, for commands built by hand. The supplier gives the goal in the real field frame, like the
  robot's pose (not flipped on red), or empty to keep the last one. When the goal moves more than 2 cm or 2°, only
  the rest of the path is rebuilt, from the robot's progress to the goal, joining with matching position,
  tangent and curvature (`PathFollower.bindGoal`, 0.1-0.3 ms). In the last 0.3 m DriveToPose's law drives
  straight to it. The pathfinding e2e suite runs it on v3 and v2, blue and red, with the goal seen 8 cm and 3°
  away once the robot is 1.5 m out. `PathLibrary` holds no drivetrain, so the factory lives on `Pathfinding`
  rather than as `paths.follow(...)`.

- **Planning.** Each command plans when it starts, from the measured pose and velocity, with `OnTheFly` (in `plan`,
  no WPILib). This is the one place path work happens at go: about 1 ms median and under 5 ms at the 95th
  percentile on the desktop.
  - A start or goal inside a grown obstacle is snapped out.
  - If the robot ends up more than 20 cm off the plan (`replanAbove`, e.g. after a hard shove), it plans again from
    where it is, at the speed it's going, at most once per 0.5 s. `replan()` asks for a re-plan at any time.
- **Hand-over.** `thenFollow` arrives along the path's start direction at the path's start speed. Inside the
  pathfinder, `plan(..., endSpeed, endTangent)` ends the route with a straight approach of 0.5 m plus the braking
  distance. So a path that starts moving gets a rolling hand-over, and one that starts at rest gets a stop.
- **Frames.** It plans in the real field frame, because field.json's obstacles are where they physically are. On
  red, authored goals (poses, named points, a path's start) are flipped with the alliance's frame, read when the
  command starts. `toTarget` and moving goals are taken as given.
- **Telemetry** goes out under `Path/<name>`, like a followed path's, so the Telemetry page draws it:
  - Plan, published again at every re-plan.
  - Each loop's state.
  - The planning figures: PlanOk, PlanMicros, Replans, StartSnap, GoalSnap, MinClearance and PlanFailure.
- **The follower fix this needed.** A rolling end used to finish as soon as the robot was past the end's line. An
  on-the-fly path can start beyond that line, so now the robot also has to be within the projection window
  (0.5 m) of the end. The Java and TS followers both changed. The golden traces didn't change.

The TS port (`editor/core/src/plan`) is bit-exact with this planner, for the editor's "Route around obstacles". The
check is `fixtures/plan/plans.json` (`PlanFixtureGen`): 13 starts and goals on the 2026 field, worst deviation 0.
The planner's trig goes through `StrictMath` (fdlibm, as V8's does) for that.

**The 2026 field** (`fixtures/plan/field_2026.json`) is the editor's default field.json, byte for byte, so the robot,
the editor and the tests use one set of obstacles:

- both hubs (47 in squares);
- both towers (45 in deep, 49.25 in wide, against each alliance wall);
- the four trench supports (the solid part of each trench, between the bump and the lane).

The bumps are drivable (6.5 in ramps), so they aren't obstacles, and the trench lanes are open to robots up to
22.25 in tall. A team whose robot avoids the bumps, or is too tall for the trench, adds them as obstacles in the
editor.

**End-to-end** (`lib/gradlew -p lib e2eTest --tests '*PathfindEndToEndTest'`, report in `e2e/PATHFIND.md`): through
the WPILib adapters on the e2e `SimDrive`, with contact checked against the true pose every 5 ms.

| Scenario | Runs | Passed | Worst clearance |
|---|---|---|---|
| 50 seeded random routes, v3 and v2, blue and red | 200 | 200 | 5.7 cm |
| Sideways shove a third of the way in; every run planned again | 20 | 20 | 5.9 cm |
| Path hand-overs | 12 | 12 | 7.8 cm |
| Moving named-point goal | 2 | 2 | 6.8 cm |

The shoves are placed where the robot has room to be pushed: a push into an obstacle is contact by the pusher,
which no planner can prevent.

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

## End-to-end tests (phase 6)

`node tools/e2e.mjs` (from the repo root) authors every test path in the editor with Playwright, saves it
through the editor's own save flow, and runs `lib/gradlew -p lib e2eTest`. That suite (`src/e2eTest`) loads the
folder with `PathLibrary.loadAll(dir)`, binds every event name to a recording command, and runs each path
through Commands v3 and v2, on blue and both red flips, on a simulated drivetrain at 50 Hz (`SimDrive`: 20 ms
latency, 40 ms first-order lag, pose and velocity noise). It also runs each path once more and cancels it
halfway through. The results table is `e2e/RESULTS.md`, and the pass marks are in `Tolerances.java`.
`-Pe2eTrace=<path>` prints one path's run loop by loop.

**What it found, and what was fixed:**

| Found | Fix |
| --- | --- |
| Events fired on the first loop after the robot passed them: up to 8 cm late at 4 m/s | They fire on the loop nearest to passing (`PathFollower`) |
| A path shorter than the 0.3 m terminal radius (a 0.2 m hop with an 80° turn) ignored its plan: 1.0 m/s against 0.29 planned, with events 0.48 s off | The DriveToPose hand-over also waits until the plan has no more than `terminalTurnRadians` (2°) left to turn. At 10° the robot was still spinning too fast to stop and overshot by 7° |
| A turn on the spot fired every event at once and stalled mid-turn: progress came from projecting onto a 1 mm segment, and the distance brake capped the spin | Progress comes from the heading; the follower spins on the plan and holds the point |
| The first step after `start()` comes at the same timestamp, so dt clamped to 1 ms and the limiter allowed a millisecond's acceleration: every path started a loop late | That step plans for a nominal loop |
| An event at the end fired only after the robot settled (0.1 s hold plus the ease-in) | It fires when the robot gets inside the finish band |
| `onEvent` replaced the listener, so robot code calling it silently disconnected the adapters' event commands | Listeners add up |
| Commands v2: a path chained after a rolling end sent nothing for one loop | `FollowPathCommandV2` steps in `initialize()` |

None of these touch the core, so the fixtures are unchanged. The bench's tests pass, but its RESULTS.md numbers
were measured before these changes and haven't been re-run.
