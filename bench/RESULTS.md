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

## Loop rate and acceleration feed-forward (2026-11-25)

Ours as drawn on the 12 courses above (the ten plus shoved and slippery), 6 seeds, the follower otherwise at its
defaults. "250 Hz" runs the robot loop at the odometry rate (`LP.main_period = 0.004`); "feed-forward" sends
`PathFollower.commandedAcceleration()` to the wheels as force (`OursV2(ff=True)`), worked out by the library's own
`WheelForces`, the class the follow commands use (`withWheelForces`):
`python bench/matched.py run --seeds 6 --methods ours,ours_ff --poses truth,odom --courses both --out out/ff.json`.

The plant adds the force as volts on a voltage velocity loop. A torque-current drive loop has not been simulated.

**Where the acceleration comes from (2026-11-26, later the same day; the tables below this paragraph are from
before it).** The feed-forward was first the step the limiter had just taken: one command minus the last. The
commands follow the measured pose (the correction across the path, and the last centimetres' ease-in), so that
difference carries the pose's noise: standing on its goal with a pose wobbling by 4 mm, the robot was asked for
up to 12 m/s². In the CTRE simulator it twitched on its goal and a drive-to-pose took 7 s to finish. It is now
the plan's own acceleration (`PathFollower.commandedAcceleration`), which asks for under 2 m/s² in the same
test. The same change stops a launch being fed at 100 % of the limit while the command catches up with the
plan's lead. With it, on the six courses of "How hard to plan" below (4 seeds, perfect pose; total time, s /
wheel slip, wheel-s / overshoot summed, cm):

| Planned at | Velocity only | With the feed-forward |
|---|---|---|
| 6 m/s² | 15.02 / 14.0 / 22.7 | 15.06 / 9.7 / 15.4 |
| 9 m/s² | 14.20 / 17.7 / 25.9 | 14.22 / 13.4 / 15.2 |
| 11.2 m/s² | 13.95 / 17.0 / 30.9 | 14.04 / 16.5 / 14.1 |

It no longer makes a path faster: the earlier 1 to 4 % came from feeding the launch's catch-up at the full limit.
What it buys is grip: a quarter to a third less wheel slip and no overshoot at the same time, so planning at 9
with it (14.22 s, 13.4) beats planning at 6 without it (15.02 s, 14.0) on both. On the 12 courses at the
tipping limit it is 31.46 s against 31.83 s with the same slip (51.8 against 52.2).

**How the push is shared between the wheels (2026-11-26).** The first version gave every wheel a quarter of
mass x acceleration. A robot speeding up leans back: at 6 m/s² this one's front wheels carry half their share of
the weight, and a quarter of the push is more than their grip. They spun, and the feed-forward kept pushing a
spinning wheel where the velocity loop alone backs off. `WheelForces` now gives each wheel the share of the push
that it carries of the weight (from the robot file's centre of mass height), so every wheel uses the same fraction
of its grip.

On paths planned at 6 m/s² (6 seeds, perfect pose; time, s / overshoot, cm / wheel slip, wheel-s):

| Course | Velocity only | Equal shares | Shared by weight (as shipped) |
|---|---|---|---|
| `tag_align_moving_goal` (time to settle) | 2.00 / 0.9 / 1.12 | 1.95 / 0.0 / 1.44 | 1.96 / 0.0 / 0.34 |
| `three_piece_chain` (time to settle) | 4.76 / 0.8 / 4.25 | 4.73 / 0.0 / 4.75 | 4.74 / 0.0 / 3.11 |
| `intake_drive_through` (time to the piece; miss in place of overshoot) | 1.42 / 0.4 / 1.24 | 1.40 / 0.4 / 1.51 | 1.38 / 0.4 / 0.75 |

Shared by weight keeps the time and slips 27 to 70 % less than sending no feed-forward at all.

On the 12 courses planned to 95 % of the tipping limit (11.84 m/s²):

| | Total time, s (perfect pose) | Cross-track rms, cm | Wheel slip, wheel-s | Odometry only: time, s / end error, cm |
|---|---|---|---|---|
| 50 Hz, velocity only | 31.83 | 4.06 | 52.2 | 28.98 / 12.1 |
| 50 Hz + feed-forward, equal shares | 30.59 | 4.10 | 52.2 | 29.20 / 13.9 |
| 50 Hz + feed-forward, shared by weight (as shipped) | 31.40 | 4.14 | 52.4 | 29.34 / 12.1 |
| 250 Hz, velocity only | 30.86 | 4.45 | 49.2 | 28.66 / 12.2 |
| 250 Hz + feed-forward, equal shares (not run again) | 29.70 | 4.47 | 48.9 | 28.89 / 13.4 |

Here the wheel slip is the same whatever is sent, and sharing by weight gives back most of the time (1.4 % faster
than velocity only, against 4 %). What the traces show (a straight, wheel by wheel):

- **The slip at the limit isn't the feed-forward's.** At 11.84 m/s² the wheels the robot leans off carry 7 N each
  (of 147 at rest). The drive motors' velocity loops push every wheel by the same speed error, so those wheels are
  asked for 50 to 150 N and slide, with or without feed-forward. Only a plan with more margin from tipping, or a
  drivetrain that shares its own correction by weight, would stop it.
- **The follower launches at 100 % of the limit, not the plan's 95 %.** The limiter's budget is the whole
  grip-or-tipping limit (so feedback has room), and the first commands use all of it to catch up with the plan's
  50 ms lead. With velocity only, the wheels' lag softens that. With feed-forward the robot really does it: 12.5
  m/s² for the first quarter second, the loaded wheels sliding the whole time.
- **Equal shares were faster at the finish because the robot lagged into it.** It ran 0.3 m/s behind its command
  and braked at 10 m/s² almost to the goal; sharing by weight it follows the command, and so the ease-in, and
  creeps the last 5 cm. A steeper ease-in (`easeInPerSecond` 24 for 12) gives a third of that back (29.14 s against
  29.45 s over 11 of the courses, 3 seeds) for 3 % more slip. The rest is not explained.

**A softer drive velocity loop.** The velocity loop pushes every wheel by the same speed error, so its share is
equal whatever the feed-forward's is. With the feed-forward doing the work, the loop's gain can come down (the
plant's `drive_kp`, 3 V per m/s; 3 seeds on 11 of the limit courses, 4 on the three 6 m/s² ones; total time, s /
wheel slip, wheel-s):

| | At the tipping limit | At 6 m/s² |
|---|---|---|
| Velocity only, gain 3 | 29.81 / 49.2 | 8.40 / 6.60 |
| Velocity only, gain 1 | 31.52 / 50.7 | 8.60 / 6.62 |
| Feed-forward, gain 3 | 29.45 / 49.4 | 8.32 / 4.19 |
| Feed-forward, gain 1 | 29.21 / 48.2 | 8.26 / 4.00 |
| Feed-forward, gain 0.3 | 29.01 / 47.7 | 8.24 / 3.94 |
| Feed-forward, gain 0 | 28.89 / 47.5 | 8.22 / 3.89 |

With the feed-forward, a softer loop is faster and slips less on both sets (3 % faster at the limit with gain 0,
where gain 3 gave 1.2 %); without it, a softer loop is only slower. The plant's back-EMF still acts as a velocity
gain of 2.6 V per m/s at gain 0, so this is not "no feedback". On the robot this is the drive motors' kP.

**How hard to plan** (2026-11-26). One limit for speeding up and slowing down, from the old DriveToPose's 6 m/s²
up to the robot's traction limit (11.2), with the feed-forward sent. Six courses at 4 m/s (`straight_3m`,
`s_curve`, `hairpin`, `tag_align_moving_goal`, `three_piece_chain`, `intake_drive_through`), 4 seeds; totals:

| Limit, m/s² | Time, s (perfect pose) | Wheel slip, wheel-s | Wheel odometry off at the end, cm (sum) | Odometry only: time, s |
|---|---|---|---|---|
| 6 | 14.84 | 9.1 | 29.0 | 14.66 |
| 7 | 14.47 | 10.7 | 29.5 | 14.28 |
| 8 | 14.18 | 12.4 | 28.4 | 14.02 |
| 9 | 14.00 | 14.3 | 29.6 | 13.86 |
| 10 | 13.93 | 15.9 | 33.8 | 13.76 |
| 11.2 | 13.86 | 16.8 | 36.1 | 13.70 |

6 to 9 is 5.7 % faster with the odometry no further off. 9 to 11.2 is 1 % more (the 4 m/s cap and the motors
take over) and the odometry drifts 22 % further. No run overshot at any limit. Precision-Drive's `robot.json`
sets 9 (`onTheFlyMaxMetersPerSecondSquared`); the library's default with no key stays 6.

**Game pieces: the newest photo, or a filter** (2026-11-26; `PieceFilterTest.theFilterAgainstTheNewestPhoto`
prints this, so it is re-run with every build). The intake assist at full stick (3 m/s) toward pieces seen by a
simulated camera with Precision-Drive's geometry: angles good to 0.3°, a photo every 33 ms that is 40 ms old, a
fifth of detections missing and a false one in 1 photo of 20 unless said. 300 runs each; mean miss, cm / worst
miss, cm / the nudge's jitter, m/s² rms / changes of piece per run:

| Case | Newest photo as it comes, plain choice | `PieceFilter`, sticky choice |
|---|---|---|
| One piece, 25 cm to the side | 0.2 / 8.4 / 7.0 / 0.9 | 0.1 / 1.2 / 5.6 / 0 |
| One piece, clean camera | 0.1 / 0.3 / 6.9 / 0.5 | 0.1 / 0.3 / 5.6 / 0 |
| One piece, half the detections missing | 0.9 / 43.8 / 7.1 / 1.4 | 0.2 / 4.8 / 5.6 / 0 |
| One piece, a false detection in 1 photo of 4 | 0.7 / 21.3 / 7.9 / 3.1 | 0.2 / 4.5 / 5.7 / 0 |
| Two pieces about equally in line | 0.5 / 22.8 / 8.6 / 5.5 | 0.1 / 2.0 / 5.9 / 0 |
| Two pieces, at 1.5 m/s | 0.2 / 7.0 / 7.7 / 10.7 | 0.1 / 0.4 / 3.4 / 0 |
| One piece well off to the side (0.9 m at 2.6 m) | 1.2 / 89.5 / 7.4 / 1.4 | 2.6 / 95.1 / 6.7 / 0 |

The average miss is the same either way: a camera's error shrinks with the square of the distance, so the last
few photos before the piece are good whatever came before. The filter is for the bad runs: a piece lost for a
few photos, a false detection, two pieces the choice swaps between. The worst miss falls from 20 to 45 cm to
under 5 cm, the piece being chased never changes, and the nudge is a fifth to a half smoother. The last row is
the assist's own reach, not the tracking: the piece leaves the 30° the assist will help within before the
capped nudge gets there, 2 % of the time with either.

**Teleop** (a scratch run, not a bench course: 20 s of full-stick direction changes through the Python port of
AccelerationLimiter, 6 seeds; wheel slip, wheel-s / wheel odometry off at the end, cm / mean speed error, m/s):

| Limiter's traction limit | Velocity only | + equal shares | + shared by weight |
|---|---|---|---|
| 11.2 m/s² (the robot's) | 23.7 / 10.0 / 0.28 | 24.9 / 11.4 / 0.22 | 23.0 / 7.1 / 0.19 |
| 8 m/s² | 17.8 / 3.2 / 0.22 | 19.4 / 3.5 / 0.16 | 12.6 / 3.1 / 0.15 |
| 6 m/s² | 8.6 / 2.5 / 0.17 | 10.2 / 2.3 / 0.12 | 3.6 / 2.7 / 0.12 |

Tried and no help: capping or scaling the feed-forward (0.8 x, or at the plan's 6 m/s²), averaging it over two
loops, letting the lean follow the command late (20 to 200 ms), `latencySeconds` 0.02 to 0.04 (slower and more
slip at the limit), and blends of the two shares (a straight trade of one set's time for the other's slip).

**A noisy goal.** `tag_align_moving_goal` with the goal estimate re-sent every loop with gaussian noise, as a camera
gives it (6 seeds, perfect pose; time to settle on the true goal, 2.00 s with no noise):

| Noise (1 sigma) | As given (`bindGoal(goal)`) | Averaged (`bindGoal(goal, 0.5)`, `OursV2(goal_smoothing=0.5)`) |
|---|---|---|
| 1 cm | 4.09 s (5 of 6) | 2.00 s (6 of 6) |
| 2 cm | never (0 of 6) | 2.02 s (6 of 6) |
| 4 cm | never (0 of 6) | 2.18 s (6 of 6) |

`intake_drive_through` at 4 cm of noise: the piece is passed 0.7 cm off averaged, 3.1 cm as given.

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
