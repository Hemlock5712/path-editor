# lib: the path maths (phase 2)

The reference implementation of the path geometry, heading profile, velocity profile, frame flips
and path files. Package `frc.team5712.path.core` has no dependencies (no WPILib), so it tests in
seconds and ports line for line to TypeScript (`editor/core/`). The follower (phase 3) will sit on
top of it.

## Running it (from the repo root)

| What | Command |
| --- | --- |
| Tests (prints timings) | `lib/gradlew -p lib test` |
| Regenerate `fixtures/*.json` | `lib/gradlew -p lib fixtures` |
| Build the jar for the bench | `lib/gradlew -p lib jar` |
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
- **Boot budget risk.** PLAN.md allows < 50 ms to load and precompute every path at boot. On this
  PC the first path costs 23-65 ms in a fresh JVM and each later one about 2 ms until the JIT
  warms up, and the robot's CPU is slower. Ways to stay inside it: build the paths on a background
  thread in `robotInit`, or build one throwaway path early to warm up.
