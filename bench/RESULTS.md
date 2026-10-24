# Bench results

The current numbers, 2026-10-23, after the library was simplified for Swerve School lesson 4. Older reports are in git
history (this file was 3,000 lines before).

## What is measured

- **The Bench page** (`bench/matched.py`, snapshot in `editor/app/public/bench/matched.json`): ours, Choreo, PathPlanner
  path2 and BLine planning to the same limits (95 % of the robot's grip-or-tipping limit, 11.84 m/s², and up to
  4.5 m/s) on the same simulated robot (0.254 m modules, centre of mass 0.2 m, μ 1.5). Ten courses, plus the S-curve
  shoved halfway and on a slippery floor (μ 0.91); 10 seeds; perfect pose and wheel odometry. Ours runs the library's
  defaults with nothing set.
- **On the fly** (`bench/h2h.py`): ours (`OnTheFly`, as the robot's `Pathfinding` commands build it) against
  PathPlanner's AD* pathfinding + hold and BLine fed our route, on six fields (the 2026 field, open, narrow gaps, a U
  trap, dense clutter, a dog-leg corridor), 40 start/goal pairs each.
- A run **finishes** within 2 cm and 1° of the target, under 5 cm/s, held 0.25 s.

Reproduce (from the repo root; `lib/gradlew -p lib jar` first):

```
python bench/matched.py plans
python bench/matched.py run --procs 12 --seeds 10 --poses truth,odom --courses both
python bench/matched.py export
python bench/h2h.py run --procs 12
```

`bench/matched.py run --out FILE` and `PATHCORE_JAR=other.jar` measure another build of the library (how each cut of
the simplification was judged).

## The page: 10 courses, perfect pose

| Method | Finished (of 100) | First on | Total time, s | End error, cm | Wheel slip, wheel-s | Odometry only: end error, cm | Shoved / slippery floor: time, s (end error, cm) |
|---|---|---|---|---|---|---|---|
| Ours, optimised | 100 | 5 | 24.32 | 0.08 | 34.6 | 6.7 | 3.13 (0.1) / 2.97 (0.2) |
| Ours, as drawn | 100 | 1 | 24.68 | 0.07 | 34.9 | 6.0 | 2.90 (0.1) / 2.89 (0.2) |
| Choreo + final hold | 100 | 1 | 25.83 | 0.27 | 58.4 | 12.4 | 2.73 (0.0) / 3.51 (0.1) |
| Choreo (ChoreoLib as shipped) | 0 | 0 | never | 19.4 | 50.4 | 13.6 | never (45) / never (84) |
| PathPlanner path2 | 99 | 3 | 27.39 | 1.29 | 28.9 | 5.8 | 2.47 (1.5) / 2.30 (0.4) |
| BLine | 100 | 0 | 37.42 | 0.73 | 21.2 | 5.8 | 3.62 (0.7) / 3.55 (0.7) |

Planned time over the ten courses: Choreo 21.41 s, ours optimised 21.83 s, ours as drawn 22.20 s. path2 wins the
straight, the hairpin and the long fast course; Choreo + hold the spin while driving. Ours brakes as hard as its plan
(the library's default, the traction limit), so shoved or on a slippery floor it recovers more slowly than path2 and
Choreo + hold. The rivals' gains are in `matched.py`.

**An unknown robot.** The planner keeps believing the bench robot while the simulated one changes: centre of mass
0.15, 0.25 or 0.30 m, μ 1.2 or 1.0, +20 or +40 ms latency, velocity gain halved or doubled, 68 kg, slow steering
(10 courses × 6 seeds each). Ours, optimised and as drawn, finished every run on every one, worst end error 0.30 cm.

## On the fly

Six fields × 40 pairs, every planner at 4 m/s and 6 m/s². Success = finished, no bumper contact, within 2 cm
(perfect pose) or 10 cm (odometry). Time is the median to settle (perfect pose) or to finish (odometry).

| Case | Ours: success / contact | PathPlanner AD* + hold | BLine fed our route | Ours: median time, plan |
|---|---|---|---|---|
| At rest, perfect pose (240) | **240 / 0** | 223 / 17 | 228 / 10 | 2.89 s, 1.3 ms |
| At rest, wheel odometry (240) | **196 / 3** | 166 / 29 | 152 / 15 | 2.93 s |
| Already moving 1.5-3.5 m/s, odometry (72) | **55 / 9** | 13 / 41 | 36 / 26 | 2.87 s |
| Goal jumps 0.5 m at 1 s, perfect pose (72) | **72 / 0** | 67 / 4 | (no pathfinder) | 2.72 s |
| Goal jumps 0.5 m at 1 s, odometry (72) | **63 / 0** | 49 / 11 | (no pathfinder) | 2.77 s |
| Re-plan forced at 1 s, odometry (72) | 64 / 0 | | | 2.84 s |

PathPlanner's median plan is 30-60 ms (a Python port of AD*, run in one go rather than on a thread); BLine has no
pathfinder of its own. With a perfect pose ours never touches anything; with odometry alone, drift larger than the
margins still lets a few runs touch.

## Sources

| What | Repository, branch, commit | Files ported |
|---|---|---|
| BLine follower | github.com/edanliahovetsky/BLine-Lib, `wpilib-2027`, 44022501faa981e5cbb9ea9492d8ac4bc67acdc5 (2026-10-15, "2027 beta.3") | `following/Follower`, `TranslationProgress`, `RotationProgress`, `TranslationGuidance`, `HolonomicController`, `HolonomicRateLimiter`; `path/PreparedPath`, `Path` (waypoint expansion); README gains |
| BLine defaults, curve tool | github.com/edanliahovetsky/BLine-Web, `bline-2027`, 57127fd43ae5d8f650160a68bf37afb2bc0b49c1 | `core/config/projectConfig.ts` (0.45 m handoff, 3 cm / 2 deg tolerances), `core/pathProfile/curveProfile.ts` |
| PathPlanner | github.com/mjansen4857/pathplanner, `2027`, 7fbd7980c5f81874d56a5b64c39fb8d8c3037352 (2026-10-03) | `path/PathPlannerPath`, `trajectory/PathPlannerTrajectory`, `PathPlannerTrajectoryState`, `config/RobotConfig`, `ModuleConfig`, `util/GeometryUtil`, `controllers/PPHolonomicDriveController`, `commands/FollowPathCommand`; Kraken X60 FOC numbers from `lib/trajectory/dc_motor.dart` |
| BLine-Web corner seeding | same BLine-Web commit | `core/constraints/autoVelocityConstraints.ts` (`createAutoVelocitySolveSetup`, `buildCorners`, `seedCapsFromCorners`, `segmentCapsFromSolvedCaps`, `roundConstraintValue`), `core/bend/cornerBend.ts` (read for the fillet geometry) |
| Choreo, ChoreoLib | github.com/SleipnirGroup/Choreo, tag `v2027.0.0-alpha-3`, 3a9e15f022f081c93420c3ccc712fd1809e49d9e (2026-09-11); release binary `Choreo-v2027.0.0-alpha-3-Windows-x86_64-standalone.zip` | `choreo-cli.exe` (run to generate); `choreolib/.../trajectory/Trajectory.java`, `SwerveSample.java`, `auto/AutoTrajectory.java`; `docs/choreolib/getting-started.md` (follower gains); `src-core/src/spec/*.rs` (file formats) |
| PathPlanner pathfinding | same PathPlanner commit | `pathfinding/LocalADStar`, `commands/PathfindingCommand`, `path/PathPlannerPath.waypointsFromPoses`, `path/Waypoint.autoControlPoints`; navgrid rule from `Writerside/topics/gui-Navigation-Grid-Editor.md` |
| PathPlanner path2 (pre-release) | github.com/mjansen4857/pathplanner, `new-path`, af86ef2b55b502c7929884f08ff2b7214035b9fb (2026-09-09); no PathPlannerLib release ships it yet | `lib/path2/simulation/path2_path_follower.dart` (FollowDynamicPath's controller), `path2_simulator.dart` (the GUI's time), `swerve_setpoint_generator.dart`, `swerve_math.dart`, `lib/path2/waypoint.dart`, `graph.dart` (handoff and end-tolerance defaults); PathPlannerLib `util/swerve/SwerveSetpointGenerator.java` at the 2027 commit above (the robot side, with PathConstraints). Read as data only |
| DriveToPose, AccelerationLimiter | this repo, `bench/reference/DriveToPose.java`, `AccelerationLimiter.java` | `execute`, `isFinished`, `limit` |
| Plant | this repo, `bench/alignsim` (plant, odometry, params, geom) | |
