# Path-following bench: results (phase 1 baseline, pass 2, ours v2 from phase 3, slip handling)

How to run: build our jar first (`lib/gradlew -p lib jar`), then `python bench/run.py --both --procs 12` (about 15 minutes; results in `bench/out/results.json`,
gitignored). `--quick` is a 2-seed smoke test on 3 courses, `--perfect-pose` runs the true pose only,
`--md-only` rebuilds the generated part of this file from `results.json`. Tests: `python -m pytest -q bench/tests`.
Choreo trajectories: `python bench/choreo/make_project.py --cli <choreo-cli.exe>` (they are committed).

What runs: every method drives the team's `alignsim` swerve plant (500 Hz stick-slip tyres, Kraken X60 FOC curve,
steering lag, 250 Hz odometry thread, 50 Hz loop with 1 ms jitter, 2 ms actuation delay) along 9 courses, 10 noise
seeds each. Each seed draws a wheel-radius error (1 % sigma) and a gyro scale error (0.2 % sigma). Every method gets
the same limits: 4.0 m/s, 6 m/s^2, 540 deg/s, 720 deg/s^2. Three pose sources:
- wheel odometry + gyro (WPILib's, no vision);
- slip-aware odometry: lib's `SlipAwareOdometry` with an accelerometer, for every method;
- the perfect pose (the control law alone).

All accuracy numbers are against the TRUE plant pose. There is also a
pathfinding course. "Ours v2" is lib/'s Java `PathFollower`, driven through `BenchServer` (`bench/followers/ours_v2.py`).

The accelerometer (`bench/imu.py`) is a wrapper around the plant, which has none. It reports the true chassis
acceleration plus errors, and draws from its own random stream, so it changes no other number. Its numbers:
- 400 Hz output (Systemcore technical specifications);
- +/-8 g over 16 bits (CTRE Pigeon 2.0 User's Guide);
- 70 ug/sqrt(Hz) white noise (TDK ICM-42688-P; neither Systemcore nor the Pigeon 2 names its part or a noise
  density);
- a 10 mg 1-sigma bias per run: ASSUMED;
- 0.5 m/s^2 RMS of drivetrain vibration: ASSUMED, not from a datasheet.

Course 9, `grip_drop_s_curve`, is the S-curve whose floor drops from mu 1.15 / 1.0 to 0.70 / 0.61 halfway along,
like a worn carpet seam.

## Summary

Every number below is from the run in the generated section (10 seeds per cell).

**Perfect pose (the control laws):**
- **Choreo with ChoreoLib's documented follower is the fastest on the clean courses.** Its trajectories are solved
  offline for the plant's motors and grip. It settles in 1.48 s on the straight, 2.17 s on the S-curve, 3.07 s on
  the hairpin and 2.99 s on the slalom, and tracks its own path at 1.2-1.5 cm RMS. It is time-sampled and stops when
  its clock runs out, so it can't recover: after the shove it ends 39 cm off, at low grip 114 cm (57 of 80 runs
  settled).
- **The "Choreo geometry + DriveToPose finish" prototype keeps Choreo's speed and fixes the ending.** It settles in
  all 80 runs, 0.1 cm off, with no overshoot on the normal-grip unshoved courses. It is 0.10-0.18 s behind Choreo
  (2.28 s vs 2.17 s on the S-curve). It recovers from the shove (3.20 s) and low grip (2.92 s).
- **PathPlanner + hold, tuned (kP 10 / 10, wheel COF 0.5), is the best library follower.** It settles in all 80
  runs and tracks the smooth courses at 0.5-0.9 cm RMS. It settles 0.2-0.4 s after Choreo on five clean courses
  (2.46 s vs 2.17 s on the S-curve) and level on the moving start (1.92 s vs 1.93 s), but it recovers from the shove (2.67 s) and low grip (2.52 s). As configured
  (COF 1.15 = the plant's grip) it plans corners at the slip limit and runs 18.7 cm RMS wide on the hairpin.
- **BLine-Web's corner caps help BLine on gentle corners only.** With the README gains they cut cross-track on the
  slalom from 30.0 to 6.4 cm RMS and on the 12 m route from 31.2 to 10.2 cm. The hairpin gets no caps, because
  BLine-Web raises the lateral factor to 1.6 on paths with more than 7 targets, so it still loops (98.7 cm worst).
  Tuned with caps (kP 2, cross-track 6, 0.6 m handoff) BLine settles in all 80 runs but slower: 4.33 s on the slalom
  and 5.70 s on the 12 m route, against 3.32 s and 4.38 s for tuned PathPlanner.
- **Sweep optima are now inside the grids.** BLine without caps peaks at a 1.2 m handoff (swept 0.6-3.0 m). With
  caps it peaks at 0.6 m. PathPlanner peaks at COF 0.5 (swept 0.4-1.15) and kP 10 (swept 5-20).

**Wheel odometry (no vision):** a 2-8 cm floor from the 1 % wheel-radius error hides most differences. Slip
becomes odometry error:
- Choreo ends 27 cm off on the moving start; PathPlanner ends 54-57 cm off.
- The shove is invisible to odometry, so every method ends 20-103 cm off on average. Choreo's at-the-limit S-curve slides
  furthest, about 1 m.

**Pathfinding (four start/goal pairs; `python bench/run.py --pathfind-only`):**
- Our planner (`lib/`, `frc.team5712.path.plan`) takes 0.26-1.13 ms per query in Java on this PC (warm medians,
  95th percentile up to 1.4 ms). The first query in a fresh JVM takes 10-12 ms, and boot (growing the obstacles
  and building the 83-node graph) 16 ms. PathPlanner's AD* port needs 3-7 ms in Python for its first path. These
  timings are from a run on a quieter PC than the earlier ones (0.5-2.2 ms warm), so treat the range as the spread.
- With a perfect pose both settle in every run, and ours (ease-in 12, now the default) settles first on all four
  routes: across 3.32 s vs 3.60 s for PathPlanner + hold, behind_wall 2.85 vs 2.91 s, moving_start 3.13 vs 3.21 s,
  turn_at_goal 2.24 vs 2.39 s.
- Ours keeps 4.4-16.9 cm of bumper clearance. PathPlanner's grid route grazes wall_b's end on behind_wall (0.0 cm,
  no contact) and passes at 1.5 cm on the moving start. Ours slips 0-98 ms per run, PathPlanner 1.9-3.7 s.
- With odometry both robots touch obstacles briefly: ours for 20 ms (across) and 12 ms (behind_wall), PathPlanner
  + hold for 4 ms on both, one run of ten each time. The odometry drifts 4-7 cm, more than the 5 cm margin. Ours
  ends 3.9-7.3 cm off on average, PathPlanner + hold 8.4-14.9 cm.
- **A bigger bumper margin doesn't fix odometry drift; it closes gaps** (table "Ours: bumper margin"). On wheel
  odometry, the runs with contact over the four routes were 2 of 40 at 0.05 m, 2 of 40 at 0.10 m and 1 of 40 at
  0.15 m. But 0.10 m and 0.15 m shut the gaps on three routes and force detours:
  - behind_wall: 6.25 m becomes 16.15-19.05 m;
  - moving_start: 9.58 m becomes 13.75-16.54 m;
  - across at 0.15 m: 9.25 m becomes 12.05 m.
  With a perfect pose, time to 5 cm rises 1.7-2.4 times on behind_wall and moving_start (behind_wall 2.74 s to
  6.00-6.45 s), and by 23 % on across at 0.15 m. The longer detours drift further, so behind_wall still touches at
  every margin. The default stays 0.05 m (a parameter,
  `PlannerConfig.marginMeters`); the fix for the contacts is a better pose (vision), not a wider berth.
- **Short hops with big turns now plan in the core.** The velocity profile's step used to stall when a fast turn
  over a short distance made the budget fall steeply with speed: 0.2 m with an 80° turn could not be built. With the
  bench's 540 deg/s and 720 deg/s^2 spin limits, it now takes:
  - 0.2 m with 80°: 0.84 s;
  - 0.05 m with 180°: 1.27 s;
  - 0 m with 90° (a turn on the spot, planned on a 1 mm segment): 0.90 s.
  Peak spin acceleration is within 1 % of the cap, and the times are 5 % above the minimum-jerk floor. The fixed
  core moves every other path by about 1e-5 (the old step was slightly conservative). Re-running ours' rows on all
  eight courses with both pose sources gave the same printed numbers as the main tables (32 of 32 rows).

**The bar for ours v2 was:** match the hybrid prototype's ending (0.1 cm, no overshoot, recovers from shoves and
low grip) at Choreo's speed, without an offline solver.

**Ours v2.** This is lib/'s Java `PathFollower` on our own quintic spline and profile. It plans at mu 0.7,
follows by position and finishes with DriveToPose's law. Since phase 3 its ease-in line defaults to 12/s (phase
3's sweep pick; the limiter's mu stays 1.0, see lib/README.md). It also has adaptive grip: when lib's
`SlipDetector` sees slip, the limiter's budget for speeding up, the brake law and the speed plan drop to the grip
the floor gave. Braking keeps a limiter budget of at least mu 0.85 (`slipBrakeFriction`, chosen on the tuning
seeds). Ours' rows (and the pathfinding grid, which runs ours) come from a targeted re-run after that change;
every other row is from the full run, and its code did not change. The sweep again picked exactly the shipped
defaults, so "tuned" equals "as shipped".
- **Perfect pose:** settled in all 90 runs, 0.1 cm off. On the clean courses it is level with Choreo, within
  -0.10 to +0.06 s: 1.43 s vs 1.48 s on the straight, 2.18 s vs 2.17 s on the S-curve, 2.97 s vs 3.07 s on the
  hairpin, 2.94 s vs 2.99 s on the slalom, 4.01 s vs 3.98 s on the 12 m route, 1.99 s vs 1.93 s on the moving
  start. It is ahead of tuned PathPlanner on five of the six; PathPlanner takes the moving start in 1.92 s.
- **Where ours loses (perfect pose):**
  - Against the authored Catmull-Rom path it is 1.2-1.5 cm RMS, because it follows its own curve.
  - On the 12 m route it is 3.9 cm RMS.
  - At low grip, adaptive grip costs time: 2.68 s against 2.46 s with it off (tuned PathPlanner 2.52 s). In
    return the wheels slip 0.3 s summed instead of 8.0 s, and it overshoots 0.2 cm instead of 6.7 cm.
- **The grip-drop overshoot is fixed.** Before, adaptive grip overshot up to 24.9 cm there (6.2 cm with it off),
  and the traces showed why. The grip it latched was below what a sliding tread still gives, and the limiter
  capped braking at it. The robot came into the stop braking gentler than the floor allowed, slid past, and
  came back. Now braking keeps at least mu 0.85 in the limiter while the brake law still plans with the
  measured grip. Grip drop: 0.4 cm worst overshoot, settled in 2.25 s (2.39 s with adaptive grip off, tuned
  PathPlanner 2.52 s). Shove: 0.3 cm, 2.22 s (was 6.0 cm, 2.36 s). Low grip unchanged. Clean-course settle
  times are identical to the last run.

**Slip handling (wheel odometry, the main condition):**
- **Low grip:** ours ends 4.1 cm off, against 12.1 cm with adaptive grip off. Tuned PathPlanner ends 4.4 cm off,
  the chain 4.8 cm and Choreo 10.7 cm. Its wheels slip 0.9 s summed, against 7.3 s with adaptive grip off and
  6.3 s for PathPlanner. It settles 4 of 10 runs, as PathPlanner does.
- **Grip drop:** ours 7.9 cm (6.8 cm with adaptive grip off), PathPlanner 6.9 cm, the chain 3.9 cm, Choreo 20.4 cm.
- **Shove:** ours 39.6 cm (41.6 cm off), PathPlanner 41.3 cm, the chain 33.3 cm, Choreo 103.1 cm.
- **The cost of the grip-drop fix:** with odometry, harder braking means more sliding for the odometry to
  over-read. Ours' grip-drop error went from 6.5 to 7.9 cm and the shove's from 29.9 to 39.6 cm.
- **Moving start:** ours 3.7 cm, the chain 3.8 cm, PathPlanner 54.2 cm, Choreo 27.3 cm.
- **All courses:** ours averages 8.7 cm final error (7.5 cm before the fix). The chain gets 7.5 cm, PathPlanner
  14.4 cm and Choreo 20.5 cm. Ours slips least, at 0.9 s mean. It settles 20 of 90 runs, against the chain's 32
  and PathPlanner's 23.

**Slip-aware odometry, for every method:**
- **Mean final error over all courses:** ours 8.7 to 5.8 cm, PathPlanner 14.4 to 12.4, the chain 7.5 to 4.3 (36
  of 90 settled, the most), Choreo 20.5 to 12.1, the hybrid 16.4 to 9.3.
- **It wins on shoves:** the chain 33.3 to 3.9 cm, PathPlanner 41.3 to 24.3, Choreo 103.1 to 28.5, ours 39.6 to
  11.3 (with adaptive grip off, 41.6 to 13.7).
- **It can lose on long slides,** because the IMU's bias is not yet learnt and it carries the pose for up to
  1.5 s per spell. At low grip PathPlanner goes from 4.4 to 7.0 cm, the chain 4.8 to 5.4, the hybrid 7.2 to
  15.7; on the grip drop PathPlanner 6.9 to 7.9, Choreo 20.4 to 21.5, ours 7.9 to 10.3. Ours is level at low
  grip (4.0 to 4.0).
- **Detector, false positives:** on the five clean courses it flagged 0.0 % of ticks, for every method.
  Adaptive grip never fired there: ours' settle times on those courses are identical with it on and off. On
  the moving start, which does slip, it costs 0.04 s.
- **Detector, recall:** on the slip courses it flags 46 % of the ticks on which the plant reports any wheel
  slipping. That count includes micro-slip below the detection threshold; the detector is tuned to the
  acceleration gap the IMU can trust.
- **Tuning:** every slip threshold (IMU gap 2.5 m/s^2, follower confidence 0.3, recovery 0.05 mu/s, IMU spell
  0.1-1.5 s, braking floor 0.85) was chosen on the tuning seeds only. The IMU-gap threshold was widened once.
  The braking floor was swept over 0.6-1.0. 0.85 scored best on the bench's score (perfect pose); 0.6 gave the
  lowest odometry error on the tuning seeds, at the cost of a 2.9 cm shove overshoot.

<!-- generated:start -->
_Generated by `python bench/run.py --both` (10 seeds, 1401 s wall)._

## Tuning sweeps

Score: settled fraction first, then mean(settle time or sim time) + 10 s per metre of RMS cross-track.

**BLine, no caps** (210 variants, perfect pose, seeds [100, 101, 102], all courses). Chosen: `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 1.2}`. Best five:

| options | settled fraction | score s |
|---|---|---|
| `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 1.2}` | 1.00 | 4.92 |
| `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 0.9}` | 1.00 | 4.94 |
| `{'kp': 2.0, 'kp_ct': 4.0, 'tol': 0.01, 'radius': 0.9}` | 1.00 | 4.95 |
| `{'kp': 4.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 2.0}` | 1.00 | 5.05 |
| `{'kp': 2.0, 'kp_ct': 4.0, 'tol': 0.01, 'radius': 0.6}` | 1.00 | 5.08 |

Best variant at each `radius`: 0.6: 1.00 settled, 5.08 s; 0.9: 1.00 settled, 4.94 s; 1.2: 1.00 settled, 4.92 s; 1.6: 1.00 settled, 5.19 s; 2: 1.00 settled, 5.05 s; 2.5: 1.00 settled, 5.28 s; 3: 1.00 settled, 5.39 s

**BLine with corner caps** (90 variants, perfect pose, seeds [100, 101, 102], all courses). Chosen: `{'kp': 2.0, 'kp_ct': 6.0, 'tol': 0.01, 'radius': 0.6, 'caps': True}`. Best five:

| options | settled fraction | score s |
|---|---|---|
| `{'kp': 2.0, 'kp_ct': 6.0, 'tol': 0.01, 'radius': 0.6, 'caps': True}` | 1.00 | 4.66 |
| `{'kp': 2.0, 'kp_ct': 4.0, 'tol': 0.01, 'radius': 0.9, 'caps': True}` | 1.00 | 4.82 |
| `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 0.9, 'caps': True}` | 1.00 | 4.83 |
| `{'kp': 2.0, 'kp_ct': 4.0, 'tol': 0.01, 'radius': 0.6, 'caps': True}` | 1.00 | 4.85 |
| `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 1.2, 'caps': True}` | 1.00 | 4.89 |

Best variant at each `radius`: 0.3: 1.00 settled, 5.18 s; 0.45: 1.00 settled, 5.11 s; 0.6: 1.00 settled, 4.66 s; 0.9: 1.00 settled, 4.82 s; 1.2: 1.00 settled, 4.89 s; 1.6: 1.00 settled, 5.17 s

**PathPlanner + hold** (60 variants, perfect pose, seeds [100, 101, 102], all courses). Chosen: `{'kp': 10.0, 'kp_rot': 10.0, 'hold': True, 'cof': 0.5}`. Best five:

| options | settled fraction | score s |
|---|---|---|
| `{'kp': 10.0, 'kp_rot': 10.0, 'hold': True, 'cof': 0.5}` | 1.00 | 3.16 |
| `{'kp': 10.0, 'kp_rot': 5.0, 'hold': True, 'cof': 0.5}` | 1.00 | 3.16 |
| `{'kp': 15.0, 'kp_rot': 10.0, 'hold': True, 'cof': 0.5}` | 1.00 | 3.19 |
| `{'kp': 15.0, 'kp_rot': 5.0, 'hold': True, 'cof': 0.5}` | 1.00 | 3.19 |
| `{'kp': 7.0, 'kp_rot': 10.0, 'hold': True, 'cof': 0.5}` | 1.00 | 3.21 |

Best variant at each `cof`: 0.4: 1.00 settled, 3.35 s; 0.5: 1.00 settled, 3.16 s; 0.6: 1.00 settled, 3.31 s; 0.7: 1.00 settled, 3.39 s; 0.9: 1.00 settled, 3.47 s; 1.15: 1.00 settled, 3.81 s

**Ours v2** (27 variants, perfect pose, seeds [100, 101, 102], all courses). Chosen: `{'easeInPerSecond': 12.0, 'latencySeconds': 0.05, 'limiterFrictionCoefficient': 1.0}`. Best five:

| options | settled fraction | score s |
|---|---|---|
| `{'easeInPerSecond': 12.0, 'latencySeconds': 0.05, 'limiterFrictionCoefficient': 1.0}` | 1.00 | 2.93 |
| `{'easeInPerSecond': 12.0, 'latencySeconds': 0.05, 'limiterFrictionCoefficient': 2.0}` | 1.00 | 2.93 |
| `{'easeInPerSecond': 16.0, 'latencySeconds': 0.05, 'limiterFrictionCoefficient': 1.0}` | 1.00 | 2.93 |
| `{'easeInPerSecond': 12.0, 'latencySeconds': 0.05, 'limiterFrictionCoefficient': 1.4}` | 1.00 | 2.93 |
| `{'easeInPerSecond': 16.0, 'latencySeconds': 0.05, 'limiterFrictionCoefficient': 2.0}` | 1.00 | 2.93 |

Best variant at each `easeInPerSecond`: 9: 1.00 settled, 2.94 s; 12: 1.00 settled, 2.93 s; 16: 1.00 settled, 2.93 s

## Per-course tables

Accuracy is always against the TRUE plant pose. Cross-track is measured from the start until the robot first reaches the end of the path. Times are means over the runs that got there; 'settled' = within 2 cm and 1 deg of the end pose, true speed under 5 cm/s, held 0.25 s. Wheel slip = milliseconds of slip summed over the four wheels. Path prep = median wall-clock of the method's own preparation in Python (relative only; for Choreo it is only loading the .traj, its solver time is in the geometry section).

### Wheel odometry + gyro (no vision): the main condition

| method | runs settled | courses settled every seed | mean RMS cross-track cm | worst cross-track cm | mean final error cm | mean wheel slip ms |
|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 10/90 | 0/9 | 8.5 | 43.4 | 10.9 | 1547 |
| BLine (README gains, r 0.30) | 3/90 | 0/9 | 24.1 | 104.3 | 22.0 | 2780 |
| BLine (README gains, r 0.45 default) | 10/90 | 0/9 | 21.2 | 104.2 | 18.7 | 2526 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 8/90 | 0/9 | 12.5 | 104.2 | 16.1 | 1442 |
| BLine (tuned) | 20/90 | 0/9 | 18.7 | 101.8 | 9.4 | 1078 |
| BLine (tuned, with corner caps) | 21/90 | 0/9 | 11.4 | 103.4 | 7.1 | 1196 |
| PathPlanner (kP 5) | 3/90 | 0/9 | 14.4 | 60.7 | 28.8 | 3273 |
| PathPlanner (kP 5) + hold | 10/90 | 0/9 | 14.4 | 60.7 | 28.2 | 3300 |
| PathPlanner (tuned) + hold | 23/90 | 0/9 | 9.8 | 57.7 | 14.4 | 1569 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 13/90 | 0/9 | 12.3 | 60.7 | 19.9 | 3051 |
| DriveToPose chain | 32/90 | 0/9 | 6.6 | 58.7 | 7.5 | 2270 |
| Choreo + ChoreoLib follower (as shipped) | 22/90 | 0/9 | 16.2 | 100.7 | 20.5 | 1983 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 14/90 | 0/9 | 15.1 | 99.5 | 16.4 | 1931 |
| Ours v2 (as shipped) | 20/90 | 0/9 | 6.5 | 39.2 | 8.7 | 872 |
| Ours v2 (as shipped, adaptive grip off) | 15/90 | 0/9 | 7.0 | 41.0 | 9.7 | 1724 |
| Ours v2 (tuned) | 20/90 | 0/9 | 6.5 | 39.2 | 8.7 | 872 |

**straight_3m**: 3 m straight, heading held at 0 (6.5 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.1 / 0.3 | 0.1 / 0.3 | 4.1 / 8.7 | 0.1 | 8.7 | 1.36 | 1.53 (1/10) | 89 | 105.80 |
| BLine (README gains, r 0.30) | 0.1 / 0.3 | 0.1 / 0.3 | 38.2 / 47.4 | 0.1 | 69.4 | 1.13 | 2.27 (1/10) | 954 | 0.04 |
| BLine (README gains, r 0.45 default) | 0.1 / 0.3 | 0.1 / 0.3 | 38.2 / 47.4 | 0.1 | 69.4 | 1.13 | 2.27 (1/10) | 954 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 0.1 / 0.3 | 0.1 / 0.3 | 38.2 / 47.4 | 0.1 | 69.4 | 1.13 | 2.27 (1/10) | 954 | 0.03 |
| BLine (tuned) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.8 | 0.1 | 4.5 | 2.18 (9/10) | 2.67 (7/10) | 0 | 0.03 |
| BLine (tuned, with corner caps) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.8 | 0.1 | 4.5 | 2.18 (9/10) | 2.67 (7/10) | 0 | 0.04 |
| PathPlanner (kP 5) | 0.1 / 0.3 | 0.1 / 0.3 | 3.9 / 8.3 | 0.1 | 8.5 | 1.26 | 1.46 (1/10) | 0 | 3.43 |
| PathPlanner (kP 5) + hold | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.1 | 8.5 | 1.26 | 1.71 (4/10) | 0 | 2.56 |
| PathPlanner (tuned) + hold | 0.1 / 0.3 | 0.1 / 0.3 | 2.7 / 6.5 | 0.1 | 6.6 | 1.42 | 1.71 (5/10) | 6 | 4.66 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.1 | 8.5 | 1.26 | 1.71 (4/10) | 0 | 4.14 |
| DriveToPose chain | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.0 | 5.2 | 1.33 (9/10) | 1.52 (5/10) | 0 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 0.3 / 1.1 | 0.1 / 0.4 | 3.0 / 6.9 | 0.0 | 7.8 | 1.26 | 1.48 (5/10) | 66 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 0.3 / 1.0 | 0.1 / 0.5 | 2.3 / 6.1 | 0.1 | 5.1 | 1.32 (9/10) | 1.58 (5/10) | 24 | 3.70 |
| Ours v2 (as shipped) | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 5.4 | 1.29 (9/10) | 1.43 (4/10) | 16 | 0.91 |
| Ours v2 (as shipped, adaptive grip off) | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 5.4 | 1.29 (9/10) | 1.43 (4/10) | 16 | 0.96 |
| Ours v2 (tuned) | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 5.4 | 1.29 (9/10) | 1.43 (4/10) | 16 | 1.01 |

**s_curve**: S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.5 / 12.0 | 4.5 / 12.0 | 5.4 / 12.7 | 0.1 | 12.4 | 2.24 (9/10) | 2.42 (1/10) | 239 | 135.65 |
| BLine (README gains, r 0.30) | 13.0 / 37.3 | 14.1 / 37.4 | 6.7 / 15.1 | 0.1 | 96.7 | 3.01 (8/10) | never | 346 | 0.06 |
| BLine (README gains, r 0.45 default) | 8.2 / 27.0 | 9.0 / 27.0 | 5.9 / 13.2 | 0.1 | 91.5 | 2.93 (9/10) | never | 282 | 0.05 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 9.7 | 1.8 / 7.1 | 12.5 / 21.0 | 0.1 | 20.9 | 2.22 | never | 523 | 0.04 |
| BLine (tuned) | 6.6 / 16.6 | 4.4 / 15.9 | 4.2 / 11.3 | 0.1 | 8.2 | 2.61 (8/10) | 3.15 (3/10) | 0 | 0.04 |
| BLine (tuned, with corner caps) | 2.9 / 7.2 | 3.2 / 10.5 | 4.2 / 11.2 | 0.1 | 8.2 | 2.71 (8/10) | 3.26 (3/10) | 0 | 0.04 |
| PathPlanner (kP 5) | 1.6 / 3.9 | 1.6 / 3.8 | 5.8 / 13.1 | 0.1 | 12.8 | 1.88 (9/10) | 2.09 (1/10) | 1688 | 3.76 |
| PathPlanner (kP 5) + hold | 1.6 / 3.9 | 1.6 / 3.8 | 4.3 / 10.1 | 0.1 | 12.8 | 1.88 (9/10) | 2.34 (4/10) | 1688 | 4.33 |
| PathPlanner (tuned) + hold | 1.3 / 4.5 | 1.2 / 4.5 | 4.1 / 10.6 | 0.1 | 10.3 | 2.20 (9/10) | 2.44 (4/10) | 77 | 3.70 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.3 / 3.9 | 1.2 / 3.8 | 4.2 / 10.0 | 0.1 | 12.3 | 1.91 (9/10) | 2.37 (4/10) | 1411 | 3.93 |
| DriveToPose chain | 2.9 / 8.7 | 0.8 / 3.3 | 4.1 / 10.5 | 0.0 | 8.9 | 4.04 (8/10) | 4.20 (4/10) | 0 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 4.6 / 11.2 | 2.0 / 5.6 | 4.5 / 10.8 | 0.1 | 11.3 | 1.95 (8/10) | 2.18 (4/10) | 0 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 4.9 / 12.7 | 2.9 / 6.8 | 4.1 / 10.5 | 0.1 | 8.8 | 2.01 (8/10) | 2.27 (4/10) | 18 | 3.64 |
| Ours v2 (as shipped) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.5 | 0.1 | 9.0 | 2.02 (8/10) | 2.17 (4/10) | 16 | 0.84 |
| Ours v2 (as shipped, adaptive grip off) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.5 | 0.1 | 9.0 | 2.02 (8/10) | 2.17 (4/10) | 16 | 1.05 |
| Ours v2 (tuned) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.5 | 0.1 | 9.0 | 2.02 (8/10) | 2.17 (4/10) | 16 | 1.01 |

**hairpin**: 2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held (9.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 7.0 / 18.4 | 7.0 / 18.4 | 1.8 / 3.8 | 0.1 | 0.8 | 2.85 | 3.01 (7/10) | 1395 | 103.06 |
| BLine (README gains, r 0.30) | 45.5 / 104.3 | 45.9 / 107.5 | 7.8 / 9.9 | 0.1 | 8.8 | 6.54 (9/10) | never | 451 | 0.06 |
| BLine (README gains, r 0.45 default) | 47.0 / 104.2 | 47.3 / 107.4 | 1.7 / 4.5 | 0.1 | 53.7 | 5.02 | 5.44 (7/10) | 0 | 0.07 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 47.0 / 104.2 | 47.3 / 107.4 | 1.7 / 4.5 | 0.1 | 53.7 | 5.02 | 5.44 (7/10) | 0 | 0.07 |
| BLine (tuned) | 40.0 / 101.8 | 40.9 / 104.5 | 3.8 / 5.8 | 0.1 | 0.0 | 4.33 (9/10) | never | 0 | 0.07 |
| BLine (tuned, with corner caps) | 40.6 / 103.4 | 41.5 / 106.6 | 3.3 / 5.3 | 0.1 | 0.0 | 4.14 (9/10) | never | 0 | 0.09 |
| PathPlanner (kP 5) | 13.5 / 24.1 | 13.5 / 24.6 | 49.7 / 55.1 | 0.1 | 0.0 | never | never | 4220 | 7.45 |
| PathPlanner (kP 5) + hold | 13.5 / 24.1 | 13.5 / 24.6 | 52.7 / 57.8 | 0.1 | 0.0 | never | never | 4220 | 6.78 |
| PathPlanner (tuned) + hold | 1.6 / 7.1 | 1.6 / 7.1 | 2.3 / 4.1 | 0.1 | 0.0 | 3.16 | 3.28 (4/10) | 77 | 5.16 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.8 / 7.0 | 1.9 / 6.9 | 5.7 / 6.8 | 0.1 | 0.0 | 2.65 | never | 3500 | 5.39 |
| DriveToPose chain | 2.1 / 9.2 | 1.6 / 6.2 | 1.7 / 3.8 | 0.0 | 0.0 | 7.61 | 7.81 (8/10) | 0 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 2.4 / 8.8 | 2.3 / 8.7 | 1.9 / 4.0 | 0.1 | 0.4 | 2.88 | 3.07 (7/10) | 258 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 3.1 / 10.2 | 3.0 / 10.1 | 3.1 / 4.6 | 0.1 | 0.0 | 3.02 | never | 19 | 7.70 |
| Ours v2 (as shipped) | 2.0 / 8.2 | 1.8 / 8.2 | 3.0 / 4.6 | 0.1 | 0.0 | 2.87 | never | 18 | 1.41 |
| Ours v2 (as shipped, adaptive grip off) | 2.0 / 8.2 | 1.8 / 8.2 | 3.0 / 4.6 | 0.1 | 0.0 | 2.87 | never | 18 | 1.54 |
| Ours v2 (tuned) | 2.0 / 8.2 | 1.8 / 8.2 | 3.0 / 4.6 | 0.1 | 0.0 | 2.87 | never | 18 | 1.38 |

**slalom**: through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m (10.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.2 / 13.3 | 5.2 / 13.3 | 6.5 / 15.7 | 0.1 | 15.4 | 3.09 (9/10) | 3.26 (1/10) | 146 | 180.97 |
| BLine (README gains, r 0.30) | 37.0 / 82.5 | 36.2 / 78.7 | 28.7 / 43.0 | 0.1 | 42.2 | 4.06 | 4.61 (1/10) | 778 | 0.05 |
| BLine (README gains, r 0.45 default) | 29.7 / 69.0 | 29.8 / 64.4 | 28.8 / 40.4 | 0.1 | 38.8 | 3.85 | never | 762 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 6.6 / 14.1 | 2.6 / 10.5 | 9.6 / 20.8 | 0.1 | 20.6 | 3.30 (9/10) | never | 437 | 0.06 |
| BLine (tuned) | 10.3 / 28.4 | 7.1 / 25.6 | 5.6 / 14.8 | 0.1 | 11.2 | 3.10 (8/10) | 3.65 (2/10) | 0 | 0.06 |
| BLine (tuned, with corner caps) | 4.3 / 10.4 | 2.4 / 9.3 | 5.7 / 14.8 | 0.1 | 11.2 | 3.78 (8/10) | 4.33 (2/10) | 0 | 0.06 |
| PathPlanner (kP 5) | 4.1 / 11.8 | 4.1 / 11.8 | 7.3 / 16.1 | 0.1 | 16.1 | 2.46 (9/10) | never | 3810 | 6.26 |
| PathPlanner (kP 5) + hold | 4.1 / 11.8 | 4.1 / 11.8 | 5.9 / 13.1 | 0.1 | 16.1 | 2.46 (9/10) | 2.92 (2/10) | 3810 | 7.59 |
| PathPlanner (tuned) + hold | 1.5 / 5.0 | 1.4 / 5.0 | 5.9 / 13.7 | 0.1 | 13.5 | 3.07 (8/10) | 3.39 (3/10) | 581 | 8.94 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.9 / 6.9 | 1.8 / 6.9 | 5.8 / 14.0 | 0.1 | 16.8 | 2.55 (9/10) | 3.02 (4/10) | 3657 | 8.05 |
| DriveToPose chain | 4.2 / 11.3 | 1.2 / 4.0 | 5.4 / 14.0 | 0.0 | 12.1 | 4.68 (8/10) | 4.84 (3/10) | 0 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 8.5 / 22.1 | 2.7 / 8.9 | 5.8 / 13.7 | 0.1 | 14.1 | 2.79 (6/10) | 3.00 (4/10) | 196 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.7 / 23.3 | 3.6 / 9.6 | 6.0 / 14.1 | 0.3 | 11.7 | 2.89 (7/10) | 3.10 (1/10) | 22 | 6.61 |
| Ours v2 (as shipped) | 1.6 / 7.4 | 1.6 / 6.9 | 5.6 / 14.1 | 0.1 | 12.1 | 2.79 (8/10) | 2.93 (3/10) | 16 | 1.43 |
| Ours v2 (as shipped, adaptive grip off) | 1.6 / 7.4 | 1.6 / 6.9 | 5.6 / 14.1 | 0.1 | 12.1 | 2.79 (8/10) | 2.93 (3/10) | 16 | 1.29 |
| Ours v2 (tuned) | 1.6 / 7.4 | 1.6 / 6.9 | 5.6 / 14.1 | 0.1 | 12.1 | 2.79 (8/10) | 2.93 (3/10) | 16 | 1.23 |

**long_auto**: auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end (14.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 6.1 / 25.4 | 6.1 / 25.4 | 8.4 / 18.9 | 3.0 | 5.8 | 4.11 (4/10) | never | 452 | 282.05 |
| BLine (README gains, r 0.30) | 33.0 / 99.5 | 37.3 / 111.8 | 17.8 / 37.7 | 1.8 | 62.9 | 5.73 (5/10) | 6.62 (1/10) | 465 | 0.07 |
| BLine (README gains, r 0.45 default) | 31.7 / 90.8 | 34.8 / 101.9 | 8.7 / 20.0 | 1.2 | 51.8 | 5.71 (4/10) | 5.98 (1/10) | 149 | 0.05 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 10.8 / 34.0 | 4.8 / 24.1 | 8.7 / 19.1 | 2.4 | 5.9 | 4.93 (4/10) | never | 387 | 0.06 |
| BLine (tuned) | 23.4 / 70.3 | 25.8 / 79.9 | 8.5 / 20.0 | 1.5 | 0.9 | 3.93 (4/10) | never | 27 | 0.08 |
| BLine (tuned, with corner caps) | 10.0 / 32.8 | 5.9 / 22.6 | 8.1 / 19.0 | 1.2 | 1.6 | 5.29 (4/10) | 5.72 (2/10) | 0 | 0.05 |
| PathPlanner (kP 5) | 5.8 / 21.7 | 5.8 / 21.7 | 9.8 / 22.1 | 0.9 | 1.1 | 3.74 (4/10) | 3.80 (1/10) | 2330 | 10.61 |
| PathPlanner (kP 5) + hold | 5.8 / 21.7 | 5.8 / 21.7 | 11.6 / 22.9 | 0.9 | 1.1 | 3.74 (4/10) | never | 2330 | 10.33 |
| PathPlanner (tuned) + hold | 4.7 / 19.3 | 4.7 / 19.3 | 8.2 / 19.2 | 0.9 | 4.3 | 4.17 (4/10) | 4.45 (3/10) | 13 | 10.44 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 4.8 / 20.8 | 4.8 / 20.8 | 8.7 / 20.7 | 0.9 | 5.8 | 3.72 (4/10) | 4.15 (1/10) | 1715 | 10.46 |
| DriveToPose chain | 8.1 / 31.5 | 4.6 / 18.9 | 8.0 / 17.8 | 0.9 | 5.0 | 6.54 (4/10) | 6.64 (3/10) | 2 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 9.2 / 27.8 | 5.3 / 20.5 | 8.0 / 18.7 | 1.8 | 5.7 | 3.76 (4/10) | 3.98 (2/10) | 182 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 9.8 / 29.1 | 6.1 / 22.8 | 8.3 / 19.3 | 1.0 | 3.2 | 3.85 (4/10) | 4.09 (2/10) | 176 | 11.08 |
| Ours v2 (as shipped) | 6.2 / 22.1 | 5.1 / 19.9 | 8.2 / 19.7 | 1.0 | 3.0 | 3.88 (4/10) | 4.00 (2/10) | 270 | 1.29 |
| Ours v2 (as shipped, adaptive grip off) | 6.2 / 22.1 | 5.1 / 19.9 | 8.2 / 19.7 | 1.0 | 3.0 | 3.88 (4/10) | 4.00 (2/10) | 270 | 1.32 |
| Ours v2 (tuned) | 6.2 / 22.1 | 5.1 / 19.9 | 8.2 / 19.7 | 1.0 | 3.0 | 3.88 (4/10) | 4.00 (2/10) | 270 | 1.08 |

**moving_start**: starts moving 2 m/s sideways (+y), path heads +x then curves left (7.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 13.0 / 27.7 | 13.0 / 27.7 | 5.9 / 11.3 | 0.1 | 10.7 | 1.88 (9/10) | never | 1544 | 140.92 |
| BLine (README gains, r 0.30) | 30.8 / 59.4 | 31.9 / 66.2 | 29.4 / 35.3 | 0.1 | 35.2 | 3.44 | never | 840 | 0.05 |
| BLine (README gains, r 0.45 default) | 33.1 / 54.8 | 34.2 / 51.5 | 23.7 / 39.3 | 0.1 | 55.8 | 2.14 | never | 587 | 0.04 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 24.3 / 49.6 | 24.9 / 47.1 | 11.0 / 16.4 | 0.1 | 15.9 | 2.17 | never | 510 | 0.04 |
| BLine (tuned) | 34.9 / 56.2 | 33.7 / 52.2 | 3.7 / 8.5 | 0.1 | 6.0 | 2.09 (9/10) | 2.37 (4/10) | 0 | 0.05 |
| BLine (tuned, with corner caps) | 18.7 / 47.6 | 19.5 / 45.6 | 3.7 / 9.5 | 0.1 | 5.0 | 3.07 (8/10) | 3.56 (3/10) | 0 | 0.03 |
| PathPlanner (kP 5) | 46.8 / 60.7 | 46.8 / 60.7 | 56.8 / 59.8 | 0.1 | 12.0 | never | never | 2514 | 4.27 |
| PathPlanner (kP 5) + hold | 46.8 / 60.7 | 46.8 / 60.7 | 56.5 / 59.9 | 0.1 | 12.0 | never | never | 2514 | 4.77 |
| PathPlanner (tuned) + hold | 45.4 / 57.7 | 45.3 / 57.6 | 54.2 / 56.5 | 0.1 | 16.6 | never | never | 1656 | 5.33 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 46.8 / 60.7 | 46.8 / 60.6 | 56.5 / 59.9 | 0.1 | 12.1 | never | never | 2506 | 5.06 |
| DriveToPose chain | 21.1 / 58.7 | 19.5 / 54.8 | 3.8 / 8.3 | 0.0 | 7.6 | 3.65 (9/10) | 3.86 (4/10) | 0 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 20.4 / 27.0 | 21.8 / 26.8 | 27.3 / 30.6 | 0.0 | 28.4 | never | never | 1412 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 11.3 / 28.1 | 11.7 / 29.5 | 3.8 / 9.6 | 0.3 | 6.2 | 1.82 (8/10) | 2.09 (2/10) | 1922 | 3.68 |
| Ours v2 (as shipped) | 11.4 / 28.6 | 11.5 / 28.6 | 3.7 / 9.2 | 0.1 | 6.0 | 1.84 (8/10) | 1.95 (3/10) | 1478 | 0.85 |
| Ours v2 (as shipped, adaptive grip off) | 11.4 / 28.1 | 11.6 / 28.1 | 4.0 / 9.6 | 0.1 | 6.4 | 1.79 (8/10) | 1.94 (2/10) | 1804 | 0.77 |
| Ours v2 (tuned) | 11.4 / 28.6 | 11.5 / 28.6 | 3.7 / 9.2 | 0.1 | 6.0 | 1.84 (8/10) | 1.95 (3/10) | 1478 | 0.90 |

**hit_mid_path**: the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 31.2 / 43.4 | 31.2 / 43.4 | 41.8 / 48.6 | 0.1 | 1.7 | never | never | 2182 | 97.25 |
| BLine (README gains, r 0.30) | 32.0 / 80.7 | 33.8 / 81.9 | 55.6 / 65.4 | 0.1 | 72.7 | never | never | 2989 | 0.05 |
| BLine (README gains, r 0.45 default) | 25.2 / 64.9 | 26.9 / 65.8 | 49.1 / 55.4 | 0.1 | 70.1 | never | never | 2838 | 0.05 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 10.3 / 21.4 | 11.7 / 22.9 | 20.1 / 23.2 | 0.1 | 8.6 | never | never | 2313 | 0.05 |
| BLine (tuned) | 38.7 / 44.5 | 39.4 / 44.9 | 46.0 / 48.8 | 0.1 | 0.0 | never | never | 2373 | 0.05 |
| BLine (tuned, with corner caps) | 19.9 / 26.0 | 20.9 / 31.7 | 27.0 / 32.4 | 0.1 | 0.0 | never | never | 1992 | 0.05 |
| PathPlanner (kP 5) | 41.6 / 51.7 | 41.6 / 51.7 | 51.8 / 53.6 | 0.1 | 1.3 | never | never | 3722 | 5.00 |
| PathPlanner (kP 5) + hold | 41.5 / 51.7 | 41.5 / 51.7 | 52.4 / 54.9 | 0.1 | 1.3 | never | never | 3722 | 4.68 |
| PathPlanner (tuned) + hold | 30.1 / 41.5 | 30.1 / 41.5 | 41.3 / 42.3 | 0.1 | 3.3 | never | never | 2408 | 6.87 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 40.2 / 50.1 | 40.1 / 50.1 | 50.7 / 52.8 | 0.1 | 1.0 | never | never | 3491 | 6.19 |
| DriveToPose chain | 15.0 / 29.8 | 16.8 / 32.8 | 33.3 / 41.6 | 0.0 | 0.0 | never | never | 1931 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 87.5 / 100.7 | 83.6 / 97.4 | 103.1 / 106.2 | 0.1 | 0.0 | never | never | 3316 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 84.1 / 99.5 | 80.1 / 96.7 | 98.8 / 105.9 | 0.2 | 0.0 | never | never | 3212 | 5.18 |
| Ours v2 (as shipped) | 32.2 / 39.2 | 32.4 / 39.4 | 39.6 / 44.2 | 0.1 | 0.0 | never | never | 2403 | 0.89 |
| Ours v2 (as shipped, adaptive grip off) | 34.1 / 41.0 | 34.3 / 41.5 | 41.6 / 48.2 | 0.1 | 0.0 | never | never | 2427 | 0.75 |
| Ours v2 (tuned) | 32.2 / 39.2 | 32.4 / 39.4 | 39.6 / 44.2 | 0.1 | 0.0 | never | never | 2403 | 0.87 |

**low_grip_s_curve**: the S-curve on mu 0.70 static / 0.61 kinetic (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 11.6 | 4.6 / 11.6 | 12.0 / 20.2 | 0.1 | 20.0 | 2.23 | never | 4626 | 94.39 |
| BLine (README gains, r 0.30) | 12.2 / 36.6 | 13.4 / 36.6 | 6.4 / 15.6 | 0.1 | 97.2 | 3.06 (8/10) | never | 11148 | 0.05 |
| BLine (README gains, r 0.45 default) | 7.5 / 25.4 | 8.3 / 25.5 | 5.0 / 13.0 | 0.1 | 92.1 | 3.01 (8/10) | 3.14 (1/10) | 10434 | 0.04 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 9.7 | 1.8 / 7.5 | 19.9 / 28.8 | 0.1 | 28.6 | 2.23 | never | 5260 | 0.04 |
| BLine (tuned) | 7.1 / 17.7 | 5.0 / 16.9 | 5.0 / 14.0 | 0.1 | 5.6 | 2.77 (7/10) | 3.19 (3/10) | 5220 | 0.05 |
| BLine (tuned, with corner caps) | 2.7 / 6.0 | 2.9 / 10.5 | 4.3 / 12.1 | 0.1 | 6.9 | 2.77 (8/10) | 3.26 (2/10) | 6199 | 0.05 |
| PathPlanner (kP 5) | 10.1 / 19.8 | 10.0 / 19.7 | 30.5 / 39.4 | 0.1 | 39.3 | 1.83 (8/10) | never | 6984 | 5.31 |
| PathPlanner (kP 5) + hold | 10.1 / 19.8 | 10.0 / 19.7 | 27.4 / 36.5 | 0.1 | 39.2 | 1.83 (8/10) | never | 7102 | 5.64 |
| PathPlanner (tuned) + hold | 1.4 / 4.2 | 1.4 / 4.2 | 4.4 / 10.0 | 0.1 | 9.3 | 2.22 (9/10) | 2.43 (4/10) | 6278 | 7.25 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 7.7 / 15.8 | 7.6 / 15.6 | 16.1 / 24.8 | 0.1 | 27.5 | 1.89 (9/10) | never | 7003 | 5.60 |
| DriveToPose chain | 3.1 / 9.6 | 1.1 / 4.1 | 4.8 / 13.2 | 0.0 | 4.8 | 4.00 (7/10) | 4.13 (3/10) | 12394 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 4.5 / 13.9 | 3.2 / 9.0 | 10.7 / 19.3 | 0.1 | 19.0 | 1.94 (9/10) | never | 8298 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.5 / 12.1 | 4.8 / 12.9 | 7.2 / 14.9 | 0.0 | 4.7 | 2.03 (4/10) | never | 8080 | 4.77 |
| Ours v2 (as shipped) | 1.6 / 6.4 | 1.5 / 5.3 | 4.0 / 10.0 | 0.1 | 8.8 | 2.40 (8/10) | 2.62 (4/10) | 998 | 0.96 |
| Ours v2 (as shipped, adaptive grip off) | 4.1 / 11.4 | 4.0 / 10.3 | 12.1 / 23.6 | 0.1 | 0.0 | 2.11 (1/10) | never | 7330 | 1.04 |
| Ours v2 (tuned) | 1.6 / 6.4 | 1.5 / 5.3 | 4.0 / 10.0 | 0.1 | 8.8 | 2.40 (8/10) | 2.62 (4/10) | 998 | 1.18 |

**grip_drop_s_curve**: the S-curve; halfway along, the floor drops from mu 1.15 / 1.0 to 0.70 / 0.61 (a worn carpet seam) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.5 / 11.1 | 4.5 / 11.1 | 12.3 / 19.4 | 0.1 | 19.7 | 2.22 | never | 3247 | 136.20 |
| BLine (README gains, r 0.30) | 13.3 / 38.7 | 14.4 / 38.7 | 6.9 / 13.2 | 0.1 | 102.4 | 3.07 (6/10) | never | 7046 | 0.06 |
| BLine (README gains, r 0.45 default) | 8.5 / 28.3 | 9.4 / 28.4 | 6.9 / 15.1 | 0.1 | 97.3 | 3.02 (6/10) | never | 6726 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.2 / 8.8 | 1.7 / 6.7 | 22.9 / 31.9 | 0.1 | 31.7 | 2.21 | never | 2590 | 0.07 |
| BLine (tuned) | 6.8 / 16.3 | 4.6 / 15.5 | 5.2 / 12.2 | 0.1 | 11.6 | 2.48 (9/10) | 3.15 (1/10) | 2081 | 0.04 |
| BLine (tuned, with corner caps) | 3.0 / 7.6 | 3.5 / 11.7 | 4.9 / 11.8 | 0.1 | 11.1 | 2.59 (9/10) | 3.24 (2/10) | 2574 | 0.05 |
| PathPlanner (kP 5) | 5.9 / 22.1 | 5.8 / 22.1 | 43.3 / 54.4 | 0.1 | 50.2 | never | never | 4194 | 6.09 |
| PathPlanner (kP 5) + hold | 5.9 / 22.1 | 5.8 / 22.1 | 40.4 / 51.7 | 0.1 | 50.0 | never | never | 4313 | 7.24 |
| PathPlanner (tuned) + hold | 1.8 / 8.0 | 1.8 / 8.0 | 6.9 / 15.2 | 0.1 | 13.6 | 2.21 (6/10) | never | 3023 | 4.52 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 5.8 / 20.1 | 5.8 / 20.1 | 28.9 / 39.7 | 0.1 | 38.0 | never | never | 4173 | 6.52 |
| DriveToPose chain | 2.9 / 8.5 | 0.9 / 3.1 | 3.9 / 10.5 | 0.0 | 6.7 | 4.03 (8/10) | 4.16 (2/10) | 6102 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 8.3 / 21.9 | 5.8 / 19.3 | 20.4 / 32.1 | 0.1 | 29.1 | never | never | 4118 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.1 / 17.4 | 5.5 / 13.6 | 13.7 / 19.1 | 0.0 | 16.6 | never | never | 3908 | 6.65 |
| Ours v2 (as shipped) | 2.2 / 6.5 | 2.4 / 6.5 | 7.9 / 13.5 | 0.1 | 12.0 | 2.06 (3/10) | never | 2630 | 0.90 |
| Ours v2 (as shipped, adaptive grip off) | 1.6 / 6.1 | 1.7 / 6.1 | 6.8 / 13.4 | 0.1 | 12.3 | 2.03 (7/10) | never | 3616 | 0.99 |
| Ours v2 (tuned) | 2.2 / 6.5 | 2.4 / 6.5 | 7.9 / 13.5 | 0.1 | 12.0 | 2.06 (3/10) | never | 2630 | 0.91 |

### Slip-aware odometry + gyro + IMU (lib's SlipAwareOdometry, no vision)

| method | runs settled | courses settled every seed | mean RMS cross-track cm | worst cross-track cm | mean final error cm | mean wheel slip ms |
|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 10/90 | 0/9 | 6.4 | 37.3 | 7.3 | 1676 |
| BLine (README gains, r 0.30) | 3/90 | 0/9 | 24.3 | 104.3 | 19.5 | 2922 |
| BLine (README gains, r 0.45 default) | 10/90 | 0/9 | 21.1 | 104.2 | 14.3 | 2576 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 8/90 | 0/9 | 12.0 | 104.2 | 15.1 | 1512 |
| BLine (tuned) | 20/90 | 0/9 | 16.2 | 101.8 | 5.1 | 1141 |
| BLine (tuned, with corner caps) | 21/90 | 0/9 | 10.0 | 103.4 | 4.8 | 1287 |
| PathPlanner (kP 5) | 5/90 | 0/9 | 11.9 | 52.6 | 23.3 | 3850 |
| PathPlanner (kP 5) + hold | 7/90 | 0/9 | 11.9 | 52.6 | 17.1 | 3991 |
| PathPlanner (tuned) + hold | 20/90 | 0/9 | 7.8 | 54.0 | 12.4 | 1915 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 15/90 | 0/9 | 9.3 | 52.6 | 14.6 | 3413 |
| DriveToPose chain | 36/90 | 0/9 | 5.7 | 58.7 | 4.3 | 2316 |
| Choreo + ChoreoLib follower (as shipped) | 22/90 | 0/9 | 11.8 | 97.2 | 12.1 | 2298 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 14/90 | 0/9 | 11.0 | 103.7 | 9.3 | 2322 |
| Ours v2 (as shipped) | 20/90 | 0/9 | 4.5 | 35.9 | 5.8 | 1026 |
| Ours v2 (as shipped, adaptive grip off) | 14/90 | 0/9 | 4.8 | 38.4 | 6.5 | 1872 |
| Ours v2 (tuned) | 20/90 | 0/9 | 4.5 | 35.9 | 5.8 | 1026 |

**straight_3m**: 3 m straight, heading held at 0 (6.5 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.1 / 0.3 | 0.1 / 0.3 | 4.1 / 8.7 | 0.1 | 8.7 | 1.36 | 1.53 (1/10) | 89 | 64.92 |
| BLine (README gains, r 0.30) | 0.1 / 0.3 | 0.1 / 0.3 | 38.2 / 47.4 | 0.1 | 69.4 | 1.13 | 2.27 (1/10) | 954 | 0.04 |
| BLine (README gains, r 0.45 default) | 0.1 / 0.3 | 0.1 / 0.3 | 38.2 / 47.4 | 0.1 | 69.4 | 1.13 | 2.27 (1/10) | 954 | 0.04 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 0.1 / 0.3 | 0.1 / 0.3 | 38.2 / 47.4 | 0.1 | 69.4 | 1.13 | 2.27 (1/10) | 954 | 0.05 |
| BLine (tuned) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.8 | 0.1 | 4.5 | 2.18 (9/10) | 2.67 (7/10) | 0 | 0.05 |
| BLine (tuned, with corner caps) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.8 | 0.1 | 4.5 | 2.18 (9/10) | 2.67 (7/10) | 0 | 0.05 |
| PathPlanner (kP 5) | 0.1 / 0.3 | 0.1 / 0.3 | 3.9 / 8.3 | 0.1 | 8.5 | 1.26 | 1.46 (1/10) | 0 | 2.33 |
| PathPlanner (kP 5) + hold | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.1 | 8.5 | 1.26 | 1.71 (4/10) | 0 | 2.26 |
| PathPlanner (tuned) + hold | 0.1 / 0.3 | 0.1 / 0.3 | 2.7 / 6.5 | 0.1 | 6.6 | 1.42 | 1.71 (5/10) | 6 | 2.22 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.1 | 8.5 | 1.26 | 1.71 (4/10) | 0 | 2.45 |
| DriveToPose chain | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.0 | 5.2 | 1.33 (9/10) | 1.52 (5/10) | 0 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 0.3 / 1.1 | 0.1 / 0.4 | 3.0 / 6.9 | 0.0 | 7.8 | 1.26 | 1.48 (5/10) | 66 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 0.3 / 1.0 | 0.1 / 0.5 | 2.3 / 6.1 | 0.1 | 5.1 | 1.32 (9/10) | 1.58 (5/10) | 24 | 2.02 |
| Ours v2 (as shipped) | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 5.4 | 1.29 (9/10) | 1.43 (4/10) | 16 | 0.89 |
| Ours v2 (as shipped, adaptive grip off) | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 5.4 | 1.29 (9/10) | 1.43 (4/10) | 16 | 0.88 |
| Ours v2 (tuned) | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 5.4 | 1.29 (9/10) | 1.43 (4/10) | 16 | 0.93 |

**s_curve**: S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.5 / 12.0 | 4.5 / 12.0 | 5.3 / 12.7 | 0.1 | 12.4 | 2.24 (9/10) | 2.42 (1/10) | 239 | 172.53 |
| BLine (README gains, r 0.30) | 13.0 / 37.3 | 14.1 / 37.4 | 6.7 / 15.1 | 0.1 | 96.7 | 3.01 (8/10) | never | 346 | 0.06 |
| BLine (README gains, r 0.45 default) | 8.2 / 27.0 | 9.0 / 27.0 | 5.9 / 13.2 | 0.1 | 91.5 | 2.93 (9/10) | never | 282 | 0.05 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 9.7 | 1.8 / 7.1 | 12.5 / 21.0 | 0.1 | 20.9 | 2.22 | never | 523 | 0.05 |
| BLine (tuned) | 6.6 / 16.6 | 4.4 / 15.9 | 4.2 / 11.3 | 0.1 | 8.2 | 2.61 (8/10) | 3.15 (3/10) | 0 | 0.04 |
| BLine (tuned, with corner caps) | 2.9 / 7.2 | 3.2 / 10.5 | 4.2 / 11.2 | 0.1 | 8.2 | 2.71 (8/10) | 3.26 (3/10) | 0 | 0.04 |
| PathPlanner (kP 5) | 1.7 / 5.8 | 1.6 / 5.8 | 6.0 / 13.9 | 0.1 | 12.6 | 1.89 (8/10) | 2.09 (2/10) | 1718 | 4.05 |
| PathPlanner (kP 5) + hold | 1.7 / 5.8 | 1.6 / 5.8 | 4.9 / 11.1 | 0.1 | 12.6 | 1.89 (8/10) | 2.34 (3/10) | 1718 | 3.88 |
| PathPlanner (tuned) + hold | 1.3 / 4.5 | 1.2 / 4.5 | 4.1 / 10.6 | 0.1 | 10.3 | 2.20 (9/10) | 2.44 (4/10) | 77 | 3.99 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.3 / 3.9 | 1.2 / 3.8 | 4.2 / 10.0 | 0.1 | 12.3 | 1.91 (9/10) | 2.37 (4/10) | 1411 | 3.89 |
| DriveToPose chain | 2.9 / 8.7 | 0.8 / 3.3 | 4.1 / 10.5 | 0.0 | 8.9 | 4.04 (8/10) | 4.20 (4/10) | 0 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 4.6 / 11.2 | 2.0 / 5.6 | 4.5 / 10.8 | 0.1 | 11.3 | 1.95 (8/10) | 2.18 (4/10) | 0 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 4.9 / 12.7 | 2.9 / 6.8 | 4.1 / 10.5 | 0.1 | 8.8 | 2.01 (8/10) | 2.27 (4/10) | 18 | 5.43 |
| Ours v2 (as shipped) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.5 | 0.1 | 9.0 | 2.02 (8/10) | 2.17 (4/10) | 16 | 0.81 |
| Ours v2 (as shipped, adaptive grip off) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.5 | 0.1 | 9.0 | 2.02 (8/10) | 2.17 (4/10) | 16 | 0.57 |
| Ours v2 (tuned) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.5 | 0.1 | 9.0 | 2.02 (8/10) | 2.17 (4/10) | 16 | 0.53 |

**hairpin**: 2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held (9.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 7.0 / 18.4 | 7.0 / 18.4 | 1.8 / 3.8 | 0.1 | 0.8 | 2.85 | 3.01 (7/10) | 1395 | 81.97 |
| BLine (README gains, r 0.30) | 45.5 / 104.3 | 45.9 / 107.5 | 7.8 / 9.9 | 0.1 | 8.8 | 6.54 (9/10) | never | 451 | 0.05 |
| BLine (README gains, r 0.45 default) | 47.0 / 104.2 | 47.3 / 107.4 | 1.7 / 4.5 | 0.1 | 53.7 | 5.02 | 5.44 (7/10) | 0 | 0.05 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 47.0 / 104.2 | 47.3 / 107.4 | 1.7 / 4.5 | 0.1 | 53.7 | 5.02 | 5.44 (7/10) | 0 | 0.06 |
| BLine (tuned) | 40.0 / 101.8 | 40.9 / 104.5 | 3.8 / 5.8 | 0.1 | 0.0 | 4.33 (9/10) | never | 0 | 0.06 |
| BLine (tuned, with corner caps) | 40.6 / 103.4 | 41.5 / 106.6 | 3.3 / 5.3 | 0.1 | 0.0 | 4.14 (9/10) | never | 0 | 0.06 |
| PathPlanner (kP 5) | 18.8 / 42.3 | 18.9 / 42.4 | 19.2 / 26.4 | 0.1 | 5.3 | never | never | 6165 | 5.71 |
| PathPlanner (kP 5) + hold | 18.7 / 42.3 | 18.8 / 42.4 | 19.5 / 26.7 | 0.1 | 5.3 | never | never | 6165 | 5.44 |
| PathPlanner (tuned) + hold | 1.6 / 7.1 | 1.6 / 7.1 | 2.3 / 4.1 | 0.1 | 0.0 | 3.16 | 3.28 (4/10) | 77 | 5.16 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.7 / 6.8 | 1.7 / 6.8 | 5.1 / 6.2 | 0.1 | 0.0 | 2.63 | never | 3485 | 4.92 |
| DriveToPose chain | 2.1 / 9.2 | 1.6 / 6.2 | 1.7 / 3.8 | 0.0 | 0.0 | 7.61 | 7.81 (8/10) | 0 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 2.4 / 8.8 | 2.3 / 8.7 | 1.9 / 3.9 | 0.1 | 0.4 | 2.88 | 3.07 (7/10) | 263 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 3.1 / 10.2 | 3.0 / 10.1 | 3.1 / 4.6 | 0.1 | 0.0 | 3.02 | never | 19 | 5.93 |
| Ours v2 (as shipped) | 2.0 / 8.2 | 1.8 / 8.2 | 3.0 / 4.6 | 0.1 | 0.0 | 2.87 | never | 18 | 1.36 |
| Ours v2 (as shipped, adaptive grip off) | 2.0 / 8.2 | 1.8 / 8.2 | 3.0 / 4.6 | 0.1 | 0.0 | 2.87 | never | 18 | 1.41 |
| Ours v2 (tuned) | 2.0 / 8.2 | 1.8 / 8.2 | 3.0 / 4.6 | 0.1 | 0.0 | 2.87 | never | 18 | 1.40 |

**slalom**: through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m (10.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.2 / 13.3 | 5.2 / 13.3 | 6.5 / 15.7 | 0.1 | 15.4 | 3.09 (9/10) | 3.26 (1/10) | 146 | 139.11 |
| BLine (README gains, r 0.30) | 37.0 / 82.5 | 36.2 / 78.7 | 28.7 / 43.0 | 0.1 | 42.2 | 4.06 | 4.61 (1/10) | 778 | 0.06 |
| BLine (README gains, r 0.45 default) | 29.7 / 69.0 | 29.8 / 64.4 | 28.8 / 40.4 | 0.1 | 38.8 | 3.85 | never | 762 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 6.6 / 14.1 | 2.6 / 10.5 | 9.6 / 20.8 | 0.1 | 20.6 | 3.30 (9/10) | never | 437 | 0.06 |
| BLine (tuned) | 10.3 / 28.4 | 7.1 / 25.6 | 5.6 / 14.8 | 0.1 | 11.2 | 3.10 (8/10) | 3.65 (2/10) | 0 | 0.06 |
| BLine (tuned, with corner caps) | 4.3 / 10.4 | 2.4 / 9.3 | 5.7 / 14.8 | 0.1 | 11.2 | 3.78 (8/10) | 4.33 (2/10) | 0 | 0.06 |
| PathPlanner (kP 5) | 4.5 / 14.2 | 4.5 / 14.2 | 7.3 / 16.5 | 0.1 | 15.1 | 2.48 (6/10) | 2.63 (1/10) | 4800 | 5.59 |
| PathPlanner (kP 5) + hold | 4.5 / 14.2 | 4.5 / 14.2 | 6.7 / 13.8 | 0.1 | 15.1 | 2.48 (6/10) | never | 4800 | 5.65 |
| PathPlanner (tuned) + hold | 1.5 / 5.0 | 1.4 / 5.0 | 5.9 / 13.7 | 0.1 | 13.5 | 3.07 (8/10) | 3.39 (3/10) | 581 | 5.17 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 2.0 / 5.4 | 1.9 / 5.4 | 5.6 / 13.2 | 0.1 | 14.4 | 2.56 (9/10) | 3.02 (4/10) | 3736 | 4.34 |
| DriveToPose chain | 4.2 / 11.3 | 1.2 / 4.0 | 5.4 / 14.0 | 0.0 | 12.1 | 4.68 (8/10) | 4.84 (3/10) | 0 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 8.5 / 22.1 | 2.7 / 8.9 | 5.8 / 13.7 | 0.1 | 14.1 | 2.79 (6/10) | 3.00 (4/10) | 196 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.7 / 23.3 | 3.6 / 9.6 | 6.0 / 14.1 | 0.3 | 11.7 | 2.89 (7/10) | 3.10 (1/10) | 22 | 7.65 |
| Ours v2 (as shipped) | 1.6 / 7.4 | 1.6 / 6.9 | 5.6 / 14.1 | 0.1 | 12.1 | 2.79 (8/10) | 2.93 (3/10) | 16 | 1.03 |
| Ours v2 (as shipped, adaptive grip off) | 1.6 / 7.4 | 1.6 / 6.9 | 5.6 / 14.1 | 0.1 | 12.1 | 2.79 (8/10) | 2.93 (3/10) | 16 | 1.06 |
| Ours v2 (tuned) | 1.6 / 7.4 | 1.6 / 6.9 | 5.6 / 14.1 | 0.1 | 12.1 | 2.79 (8/10) | 2.93 (3/10) | 16 | 0.81 |

**long_auto**: auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end (14.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 6.1 / 25.4 | 6.1 / 25.4 | 8.4 / 18.9 | 3.0 | 5.8 | 4.11 (4/10) | never | 452 | 117.16 |
| BLine (README gains, r 0.30) | 33.0 / 99.5 | 37.3 / 111.8 | 8.4 / 19.8 | 1.1 | 62.9 | 5.73 (5/10) | 6.62 (1/10) | 623 | 0.02 |
| BLine (README gains, r 0.45 default) | 31.7 / 90.8 | 34.8 / 101.9 | 8.7 / 20.0 | 1.2 | 51.8 | 5.71 (4/10) | 5.98 (1/10) | 149 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 10.8 / 34.0 | 4.8 / 24.1 | 8.4 / 19.1 | 2.4 | 5.9 | 4.93 (4/10) | never | 426 | 0.02 |
| BLine (tuned) | 23.4 / 70.3 | 25.8 / 79.9 | 8.5 / 20.0 | 1.5 | 0.9 | 3.93 (4/10) | never | 27 | 0.02 |
| BLine (tuned, with corner caps) | 10.0 / 32.8 | 5.9 / 22.6 | 8.1 / 19.0 | 1.2 | 1.6 | 5.29 (4/10) | 5.72 (2/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 5.7 / 21.7 | 5.8 / 21.7 | 9.7 / 22.1 | 0.9 | 1.1 | 3.73 (4/10) | 3.80 (1/10) | 2333 | 7.36 |
| PathPlanner (kP 5) + hold | 5.8 / 21.7 | 5.8 / 21.7 | 11.6 / 22.9 | 0.9 | 1.1 | 3.73 (4/10) | never | 2333 | 6.87 |
| PathPlanner (tuned) + hold | 4.7 / 19.3 | 4.7 / 19.3 | 8.2 / 19.2 | 0.9 | 4.3 | 4.17 (4/10) | 4.45 (3/10) | 13 | 6.74 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 4.8 / 20.8 | 4.8 / 20.8 | 8.7 / 20.7 | 0.9 | 5.8 | 3.72 (4/10) | 4.15 (1/10) | 1715 | 6.13 |
| DriveToPose chain | 8.1 / 31.5 | 4.6 / 18.9 | 8.0 / 17.8 | 0.9 | 5.0 | 6.54 (4/10) | 6.64 (3/10) | 2 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 9.2 / 27.8 | 5.3 / 20.5 | 8.0 / 18.7 | 1.8 | 5.7 | 3.76 (4/10) | 3.98 (2/10) | 182 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 9.8 / 29.1 | 6.1 / 22.8 | 8.3 / 19.3 | 1.0 | 3.2 | 3.85 (4/10) | 4.09 (2/10) | 176 | 3.53 |
| Ours v2 (as shipped) | 6.2 / 22.1 | 5.1 / 19.9 | 8.2 / 19.7 | 1.0 | 3.0 | 3.88 (4/10) | 4.00 (2/10) | 270 | 0.57 |
| Ours v2 (as shipped, adaptive grip off) | 6.2 / 22.1 | 5.1 / 19.9 | 8.2 / 19.7 | 1.0 | 3.0 | 3.88 (4/10) | 4.00 (2/10) | 270 | 0.59 |
| Ours v2 (tuned) | 6.2 / 22.1 | 5.1 / 19.9 | 8.2 / 19.7 | 1.0 | 3.0 | 3.88 (4/10) | 4.00 (2/10) | 270 | 0.56 |

**moving_start**: starts moving 2 m/s sideways (+y), path heads +x then curves left (7.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 13.2 / 27.7 | 13.2 / 27.7 | 6.2 / 12.0 | 0.1 | 11.7 | 1.87 (9/10) | never | 1546 | 42.44 |
| BLine (README gains, r 0.30) | 30.8 / 59.4 | 31.9 / 66.2 | 29.4 / 35.3 | 0.1 | 35.2 | 3.44 | never | 840 | 0.03 |
| BLine (README gains, r 0.45 default) | 33.1 / 54.8 | 34.2 / 51.5 | 23.7 / 39.3 | 0.1 | 55.8 | 2.14 | never | 587 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 24.3 / 49.6 | 24.9 / 47.1 | 11.0 / 16.4 | 0.1 | 15.9 | 2.17 | never | 510 | 0.02 |
| BLine (tuned) | 34.9 / 56.2 | 33.7 / 52.2 | 3.7 / 8.5 | 0.1 | 6.0 | 2.09 (9/10) | 2.37 (4/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 18.7 / 47.6 | 19.5 / 45.6 | 3.7 / 9.5 | 0.1 | 5.0 | 3.07 (8/10) | 3.56 (3/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 40.7 / 52.6 | 40.7 / 52.6 | 47.2 / 51.7 | 0.1 | 9.3 | never | never | 2772 | 1.73 |
| PathPlanner (kP 5) + hold | 40.7 / 52.6 | 40.7 / 52.6 | 47.1 / 52.0 | 0.1 | 9.3 | never | never | 2772 | 1.68 |
| PathPlanner (tuned) + hold | 41.7 / 54.0 | 41.7 / 54.0 | 49.6 / 53.1 | 0.1 | 12.7 | never | never | 1929 | 2.21 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 40.7 / 52.6 | 40.6 / 52.5 | 47.0 / 52.0 | 0.1 | 9.5 | never | never | 2781 | 1.90 |
| DriveToPose chain | 21.1 / 58.7 | 19.5 / 54.8 | 3.8 / 8.3 | 0.0 | 7.6 | 3.65 (9/10) | 3.86 (4/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 19.5 / 25.8 | 21.2 / 25.7 | 25.2 / 26.7 | 0.0 | 18.0 | never | never | 1937 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 11.3 / 28.1 | 11.7 / 29.5 | 4.0 / 9.7 | 0.3 | 6.5 | 1.82 (8/10) | 2.09 (2/10) | 1921 | 1.34 |
| Ours v2 (as shipped) | 11.4 / 28.6 | 11.5 / 28.6 | 3.7 / 9.2 | 0.1 | 6.0 | 1.84 (8/10) | 1.95 (3/10) | 1478 | 0.43 |
| Ours v2 (as shipped, adaptive grip off) | 11.5 / 28.1 | 11.6 / 28.1 | 4.1 / 9.6 | 0.1 | 6.4 | 1.79 (8/10) | 1.94 (1/10) | 1812 | 0.38 |
| Ours v2 (tuned) | 11.4 / 28.6 | 11.5 / 28.6 | 3.7 / 9.2 | 0.1 | 6.0 | 1.84 (8/10) | 1.95 (3/10) | 1478 | 0.39 |

**hit_mid_path**: the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 12.6 / 37.3 | 12.6 / 37.3 | 9.3 / 15.1 | 0.1 | 11.0 | never | never | 3287 | 42.63 |
| BLine (README gains, r 0.30) | 33.7 / 74.3 | 35.6 / 78.2 | 42.8 / 49.6 | 0.1 | 47.8 | 4.32 (4/10) | never | 4110 | 0.02 |
| BLine (README gains, r 0.45 default) | 24.0 / 60.4 | 25.8 / 63.4 | 9.5 / 14.6 | 0.1 | 86.5 | 3.07 (3/10) | never | 3293 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 6.3 / 18.3 | 7.5 / 23.0 | 11.5 / 19.9 | 0.1 | 18.7 | 2.28 (7/10) | never | 2909 | 0.02 |
| BLine (tuned) | 16.9 / 37.6 | 18.3 / 43.0 | 7.1 / 12.0 | 0.1 | 5.0 | 2.10 (5/10) | never | 2946 | 0.02 |
| BLine (tuned, with corner caps) | 7.8 / 22.4 | 9.4 / 26.0 | 6.5 / 11.7 | 0.1 | 5.2 | 2.53 (4/10) | never | 2808 | 0.02 |
| PathPlanner (kP 5) | 21.0 / 52.3 | 21.0 / 52.3 | 23.5 / 34.6 | 0.1 | 28.0 | never | never | 5226 | 2.22 |
| PathPlanner (kP 5) + hold | 21.0 / 52.3 | 21.0 / 52.3 | 10.6 / 21.3 | 0.1 | 27.9 | 2.36 (3/10) | never | 5266 | 2.64 |
| PathPlanner (tuned) + hold | 15.7 / 39.9 | 15.7 / 40.2 | 24.3 / 33.6 | 0.1 | 5.0 | 2.28 (1/10) | never | 4773 | 2.19 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 20.8 / 49.7 | 20.9 / 49.7 | 20.7 / 31.4 | 0.1 | 18.6 | 2.16 (1/10) | never | 5301 | 2.51 |
| DriveToPose chain | 7.2 / 29.0 | 7.8 / 32.0 | 3.9 / 9.1 | 0.0 | 7.5 | 4.22 (8/10) | 4.29 (3/10) | 2272 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 47.9 / 97.2 | 46.3 / 94.8 | 28.5 / 32.6 | 0.2 | 28.5 | 2.24 (1/10) | never | 5520 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 47.3 / 103.7 | 45.7 / 99.7 | 24.4 / 27.4 | 0.3 | 58.1 | never | never | 6361 | 1.88 |
| Ours v2 (as shipped) | 12.7 / 35.9 | 12.8 / 36.9 | 11.3 / 17.0 | 0.1 | 7.3 | 2.16 (1/10) | never | 3572 | 0.36 |
| Ours v2 (as shipped, adaptive grip off) | 13.0 / 38.4 | 13.2 / 39.5 | 13.7 / 19.5 | 0.1 | 3.2 | never | never | 3606 | 0.38 |
| Ours v2 (tuned) | 12.7 / 35.9 | 12.8 / 36.9 | 11.3 / 17.0 | 0.1 | 7.3 | 2.16 (1/10) | never | 3572 | 0.38 |

**low_grip_s_curve**: the S-curve on mu 0.70 static / 0.61 kinetic (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.4 / 11.9 | 4.4 / 11.9 | 12.1 / 19.9 | 0.1 | 19.7 | 2.23 | never | 4682 | 46.02 |
| BLine (README gains, r 0.30) | 12.2 / 36.6 | 13.4 / 36.6 | 6.4 / 15.6 | 0.1 | 97.2 | 3.06 (8/10) | never | 11148 | 0.02 |
| BLine (README gains, r 0.45 default) | 7.5 / 25.4 | 8.3 / 25.5 | 4.9 / 13.0 | 0.1 | 92.1 | 3.01 (8/10) | 3.14 (1/10) | 10434 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 9.7 | 1.8 / 7.5 | 19.9 / 28.8 | 0.1 | 28.6 | 2.23 | never | 5261 | 0.02 |
| BLine (tuned) | 7.2 / 17.9 | 5.0 / 17.1 | 5.1 / 14.0 | 0.1 | 5.6 | 2.77 (7/10) | 3.19 (3/10) | 5218 | 0.02 |
| BLine (tuned, with corner caps) | 2.7 / 6.0 | 2.9 / 10.5 | 4.3 / 12.1 | 0.1 | 6.9 | 2.77 (8/10) | 3.26 (2/10) | 6199 | 0.02 |
| PathPlanner (kP 5) | 9.5 / 20.6 | 9.5 / 20.3 | 42.0 / 50.9 | 0.1 | 50.0 | 1.81 (7/10) | never | 7222 | 2.34 |
| PathPlanner (kP 5) + hold | 9.5 / 20.6 | 9.5 / 20.3 | 40.9 / 50.9 | 0.1 | 50.0 | 1.81 (7/10) | never | 7272 | 3.21 |
| PathPlanner (tuned) + hold | 2.0 / 9.1 | 1.9 / 9.1 | 7.0 / 14.5 | 0.1 | 11.3 | 2.21 (6/10) | 2.36 (1/10) | 6546 | 2.86 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 7.0 / 16.3 | 6.9 / 16.2 | 29.3 / 38.2 | 0.1 | 36.9 | 1.86 (4/10) | never | 7166 | 2.96 |
| DriveToPose chain | 3.1 / 9.4 | 1.4 / 4.2 | 5.4 / 13.2 | 0.0 | 4.2 | 4.00 (5/10) | 4.12 (3/10) | 12448 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 4.9 / 13.8 | 4.0 / 12.5 | 10.2 / 15.8 | 0.1 | 15.7 | 1.94 (7/10) | never | 8315 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.3 / 13.6 | 3.8 / 7.4 | 15.7 / 33.1 | 0.0 | 28.9 | 1.97 (6/10) | never | 8326 | 2.01 |
| Ours v2 (as shipped) | 1.6 / 6.4 | 1.5 / 5.3 | 4.0 / 10.3 | 0.1 | 9.0 | 2.48 (8/10) | 2.70 (4/10) | 516 | 0.45 |
| Ours v2 (as shipped, adaptive grip off) | 5.6 / 14.7 | 5.5 / 13.6 | 10.6 / 21.1 | 0.1 | 2.0 | 2.11 (1/10) | never | 7502 | 0.43 |
| Ours v2 (tuned) | 1.6 / 6.4 | 1.5 / 5.3 | 4.0 / 10.3 | 0.1 | 9.0 | 2.48 (8/10) | 2.70 (4/10) | 516 | 0.47 |

**grip_drop_s_curve**: the S-curve; halfway along, the floor drops from mu 1.15 / 1.0 to 0.70 / 0.61 (a worn carpet seam) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.3 / 11.3 | 4.3 / 11.3 | 12.2 / 20.6 | 0.1 | 20.4 | 2.22 | never | 3244 | 49.21 |
| BLine (README gains, r 0.30) | 13.3 / 38.7 | 14.4 / 38.7 | 6.9 / 13.2 | 0.1 | 102.4 | 3.07 (6/10) | never | 7046 | 0.02 |
| BLine (README gains, r 0.45 default) | 8.5 / 28.3 | 9.4 / 28.4 | 6.9 / 15.1 | 0.1 | 97.3 | 3.02 (6/10) | never | 6726 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.2 / 8.8 | 1.7 / 6.7 | 22.9 / 31.9 | 0.1 | 31.7 | 2.21 | never | 2590 | 0.02 |
| BLine (tuned) | 6.8 / 16.3 | 4.6 / 15.5 | 5.2 / 12.2 | 0.1 | 11.6 | 2.48 (9/10) | 3.15 (1/10) | 2081 | 0.02 |
| BLine (tuned, with corner caps) | 3.0 / 7.6 | 3.5 / 11.7 | 4.9 / 11.8 | 0.1 | 11.1 | 2.59 (9/10) | 3.24 (2/10) | 2574 | 0.03 |
| PathPlanner (kP 5) | 5.5 / 21.9 | 5.5 / 21.9 | 50.4 / 60.6 | 0.1 | 60.0 | never | never | 4416 | 2.45 |
| PathPlanner (kP 5) + hold | 5.5 / 21.9 | 5.5 / 21.9 | 10.2 / 22.3 | 0.1 | 59.1 | 2.89 (1/10) | never | 5596 | 3.03 |
| PathPlanner (tuned) + hold | 1.8 / 10.6 | 1.8 / 10.6 | 7.9 / 17.8 | 0.1 | 14.5 | 2.20 (5/10) | never | 3236 | 2.75 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 5.4 / 19.1 | 5.4 / 19.0 | 8.2 / 20.7 | 0.1 | 46.7 | 2.76 (3/10) | 2.95 (2/10) | 5126 | 2.54 |
| DriveToPose chain | 2.9 / 8.4 | 0.9 / 3.1 | 4.0 / 10.4 | 0.0 | 7.0 | 4.03 (8/10) | 4.17 (3/10) | 6122 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 8.4 / 22.8 | 6.0 / 20.9 | 21.5 / 33.8 | 0.1 | 23.5 | never | never | 4202 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.1 / 17.4 | 5.5 / 13.6 | 16.2 / 19.7 | 0.0 | 16.0 | never | never | 4034 | 1.59 |
| Ours v2 (as shipped) | 3.1 / 9.8 | 3.3 / 9.8 | 10.3 / 14.9 | 0.1 | 7.3 | never | never | 3336 | 0.40 |
| Ours v2 (as shipped, adaptive grip off) | 1.9 / 6.1 | 2.1 / 6.1 | 7.2 / 13.4 | 0.1 | 12.3 | 2.07 (2/10) | never | 3586 | 0.38 |
| Ours v2 (tuned) | 3.1 / 9.8 | 3.3 / 9.8 | 10.3 / 14.9 | 0.1 | 7.3 | never | never | 3336 | 0.36 |

### Perfect pose (the control law alone)

| method | runs settled | courses settled every seed | mean RMS cross-track cm | worst cross-track cm | mean final error cm | mean wheel slip ms |
|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 11/90 | 1/9 | 6.4 | 37.4 | 4.8 | 1684 |
| BLine (README gains, r 0.30) | 11/90 | 0/9 | 24.9 | 98.9 | 19.4 | 2989 |
| BLine (README gains, r 0.45 default) | 6/90 | 0/9 | 21.5 | 98.7 | 14.4 | 2719 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 0/90 | 0/9 | 11.9 | 98.7 | 14.9 | 1547 |
| BLine (tuned) | 85/90 | 8/9 | 16.4 | 96.8 | 0.8 | 1199 |
| BLine (tuned, with corner caps) | 90/90 | 9/9 | 10.1 | 98.0 | 0.8 | 1381 |
| PathPlanner (kP 5) | 0/90 | 0/9 | 9.9 | 53.4 | 23.3 | 4352 |
| PathPlanner (kP 5) + hold | 90/90 | 9/9 | 9.9 | 53.4 | 0.1 | 4965 |
| PathPlanner (tuned) + hold | 90/90 | 9/9 | 4.0 | 40.1 | 0.4 | 2353 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 90/90 | 9/9 | 7.1 | 50.0 | 0.1 | 4076 |
| DriveToPose chain | 90/90 | 9/9 | 5.6 | 58.6 | 0.4 | 2430 |
| Choreo + ChoreoLib follower (as shipped) | 57/90 | 5/9 | 12.7 | 97.6 | 22.7 | 2793 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 90/90 | 9/9 | 11.5 | 103.4 | 0.1 | 2734 |
| Ours v2 (as shipped) | 90/90 | 9/9 | 4.1 | 36.4 | 0.1 | 991 |
| Ours v2 (as shipped, adaptive grip off) | 90/90 | 9/9 | 4.1 | 37.8 | 0.1 | 1988 |
| Ours v2 (tuned) | 90/90 | 9/9 | 4.1 | 36.4 | 0.1 | 991 |

**straight_3m**: 3 m straight, heading held at 0 (6.5 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.0 / 0.1 | 0.0 / 0.0 | 3.4 / 3.5 | 0.0 | 3.5 | 1.35 | never | 83 | 23.31 |
| BLine (README gains, r 0.30) | 0.0 / 0.1 | 0.0 / 0.1 | 41.5 / 42.8 | 0.0 | 42.8 | 1.13 | never | 1059 | 0.02 |
| BLine (README gains, r 0.45 default) | 0.0 / 0.1 | 0.0 / 0.1 | 41.5 / 42.8 | 0.0 | 42.8 | 1.13 | never | 1059 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 0.0 / 0.1 | 0.0 / 0.1 | 41.5 / 42.8 | 0.0 | 42.8 | 1.13 | never | 1059 | 0.02 |
| BLine (tuned) | 0.0 / 0.1 | 0.0 / 0.1 | 0.8 / 0.9 | 0.0 | 0.0 | 2.20 | 2.67 (10/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 0.0 / 0.1 | 0.0 / 0.1 | 0.8 / 0.9 | 0.0 | 0.0 | 2.20 | 2.67 (10/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 0.0 / 0.1 | 0.0 / 0.1 | 3.2 / 3.4 | 0.0 | 3.4 | 1.26 | never | 0 | 1.19 |
| PathPlanner (kP 5) + hold | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.1 | 0.0 | 3.4 | 1.26 | 1.71 (10/10) | 0 | 1.22 |
| PathPlanner (tuned) + hold | 0.0 / 0.1 | 0.0 / 0.0 | 0.4 / 1.1 | 0.0 | 1.3 | 1.42 | 1.67 (10/10) | 0 | 1.32 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.1 | 0.0 | 3.4 | 1.26 | 1.71 (10/10) | 0 | 1.28 |
| DriveToPose chain | 0.1 / 0.1 | 0.0 / 0.1 | 0.1 / 0.1 | 0.0 | 0.0 | 1.34 | 1.55 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 0.3 / 1.0 | 0.1 / 0.3 | 1.5 / 1.7 | 0.0 | 2.5 | 1.26 | 1.48 (10/10) | 30 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 0.3 / 1.0 | 0.1 / 0.3 | 0.1 / 0.1 | 0.0 | 0.0 | 1.33 | 1.58 (10/10) | 37 | 1.01 |
| Ours v2 (as shipped) | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.2 | 0.0 | 0.2 | 1.29 | 1.43 (10/10) | 19 | 0.34 |
| Ours v2 (as shipped, adaptive grip off) | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.2 | 0.0 | 0.2 | 1.29 | 1.43 (10/10) | 19 | 0.33 |
| Ours v2 (tuned) | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.2 | 0.0 | 0.2 | 1.29 | 1.43 (10/10) | 19 | 0.31 |

**s_curve**: S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 9.1 | 4.6 / 9.1 | 3.3 / 3.4 | 0.0 | 3.4 | 2.24 | never | 254 | 40.43 |
| BLine (README gains, r 0.30) | 12.8 / 37.6 | 14.0 / 37.7 | 5.7 / 6.3 | 0.0 | 89.9 | 2.99 | never | 365 | 0.02 |
| BLine (README gains, r 0.45 default) | 8.4 / 26.6 | 9.4 / 26.6 | 4.2 / 4.5 | 0.0 | 84.9 | 2.93 | never | 299 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.1 / 7.5 | 1.4 / 6.2 | 11.6 / 12.6 | 0.0 | 12.6 | 2.22 | never | 528 | 0.02 |
| BLine (tuned) | 6.8 / 15.6 | 4.4 / 15.0 | 0.8 / 0.9 | 0.0 | 0.0 | 2.70 | 3.17 (10/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 2.5 / 5.9 | 3.0 / 9.5 | 0.8 / 0.9 | 0.0 | 0.0 | 2.79 | 3.25 (10/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 0.8 / 2.5 | 0.8 / 2.6 | 3.3 / 3.5 | 0.0 | 3.6 | 1.89 | never | 1938 | 3.36 |
| PathPlanner (kP 5) + hold | 0.8 / 2.5 | 0.8 / 2.6 | 0.1 / 0.1 | 0.0 | 3.6 | 1.89 | 2.34 (10/10) | 1938 | 1.92 |
| PathPlanner (tuned) + hold | 0.5 / 1.1 | 0.4 / 1.1 | 0.4 / 1.2 | 0.0 | 1.3 | 2.21 | 2.46 (10/10) | 0 | 2.07 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.7 / 1.8 | 0.6 / 1.8 | 0.1 / 0.1 | 0.0 | 3.5 | 1.92 | 2.36 (10/10) | 1544 | 2.10 |
| DriveToPose chain | 2.8 / 6.0 | 0.2 / 0.4 | 0.1 / 0.1 | 0.0 | 0.0 | 4.08 | 4.28 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 4.6 / 8.6 | 1.2 / 1.9 | 1.5 / 1.8 | 0.0 | 2.4 | 1.95 | 2.17 (10/10) | 48 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.1 / 10.3 | 2.0 / 3.7 | 0.1 / 0.1 | 0.0 | 0.0 | 2.02 | 2.28 (10/10) | 26 | 1.55 |
| Ours v2 (as shipped) | 1.2 / 3.0 | 1.0 / 2.0 | 0.0 / 0.1 | 0.0 | 0.1 | 2.04 | 2.18 (10/10) | 19 | 0.38 |
| Ours v2 (as shipped, adaptive grip off) | 1.2 / 3.0 | 1.0 / 2.0 | 0.0 / 0.1 | 0.0 | 0.1 | 2.04 | 2.18 (10/10) | 19 | 0.40 |
| Ours v2 (tuned) | 1.2 / 3.0 | 1.0 / 2.0 | 0.0 / 0.1 | 0.0 | 0.1 | 2.04 | 2.18 (10/10) | 19 | 0.36 |

**hairpin**: 2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held (9.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 7.3 / 13.3 | 7.3 / 13.3 | 3.4 / 3.5 | 0.0 | 3.5 | 2.82 | never | 1429 | 57.80 |
| BLine (README gains, r 0.30) | 45.4 / 98.9 | 45.8 / 102.1 | 9.8 / 11.5 | 0.0 | 11.5 | 6.57 | never | 472 | 0.03 |
| BLine (README gains, r 0.45 default) | 47.2 / 98.7 | 47.4 / 101.8 | 2.3 / 2.4 | 0.0 | 54.7 | 5.40 | never | 0 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 47.2 / 98.7 | 47.4 / 101.8 | 2.3 / 2.4 | 0.0 | 54.7 | 5.40 | never | 0 | 0.03 |
| BLine (tuned) | 40.9 / 96.8 | 41.8 / 99.3 | 0.8 / 0.9 | 0.0 | 0.0 | 4.06 | 4.53 (10/10) | 0 | 0.03 |
| BLine (tuned, with corner caps) | 41.0 / 98.0 | 42.0 / 101.2 | 0.8 / 0.9 | 0.0 | 0.0 | 3.89 | 4.36 (10/10) | 0 | 0.03 |
| PathPlanner (kP 5) | 18.7 / 40.8 | 18.8 / 40.9 | 21.2 / 22.1 | 0.0 | 21.9 | 2.44 | never | 6552 | 2.49 |
| PathPlanner (kP 5) + hold | 18.7 / 40.8 | 18.8 / 40.9 | 0.1 / 0.1 | 0.0 | 21.6 | 2.44 | 3.26 (10/10) | 6950 | 3.67 |
| PathPlanner (tuned) + hold | 0.9 / 1.5 | 0.7 / 1.5 | 0.9 / 1.2 | 0.0 | 1.3 | 3.12 | 3.30 (10/10) | 29 | 4.10 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.8 / 1.9 | 0.7 / 1.8 | 0.1 / 0.1 | 0.0 | 3.4 | 2.57 | 3.02 (10/10) | 3560 | 2.46 |
| DriveToPose chain | 1.2 / 3.2 | 0.1 / 0.3 | 0.1 / 0.1 | 0.0 | 0.0 | 7.74 | 7.95 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 1.4 / 2.4 | 1.2 / 2.0 | 1.4 / 1.6 | 0.0 | 2.5 | 2.84 | 3.07 (10/10) | 138 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 2.5 / 4.2 | 2.3 / 3.9 | 0.1 / 0.1 | 0.1 | 0.0 | 2.93 | 3.19 (10/10) | 35 | 2.36 |
| Ours v2 (as shipped) | 1.5 / 3.0 | 1.4 / 2.5 | 0.1 / 0.1 | 0.0 | 0.1 | 2.83 | 2.97 (10/10) | 19 | 0.51 |
| Ours v2 (as shipped, adaptive grip off) | 1.5 / 3.0 | 1.4 / 2.5 | 0.1 / 0.1 | 0.0 | 0.1 | 2.83 | 2.97 (10/10) | 19 | 0.53 |
| Ours v2 (tuned) | 1.5 / 3.0 | 1.4 / 2.5 | 0.1 / 0.1 | 0.0 | 0.1 | 2.83 | 2.97 (10/10) | 19 | 0.54 |

**slalom**: through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m (10.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.5 / 10.2 | 5.5 / 10.2 | 3.3 / 3.5 | 0.0 | 3.5 | 3.08 | 3.26 (1/10) | 168 | 66.79 |
| BLine (README gains, r 0.30) | 40.3 / 83.4 | 40.9 / 80.8 | 13.3 / 31.0 | 0.0 | 40.9 | 4.29 | 4.57 (3/10) | 362 | 0.02 |
| BLine (README gains, r 0.45 default) | 30.0 / 67.8 | 30.0 / 63.2 | 29.0 / 30.6 | 0.0 | 28.9 | 3.87 | never | 854 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 6.4 / 11.9 | 2.3 / 9.0 | 8.1 / 8.6 | 0.0 | 8.6 | 3.30 | never | 432 | 0.02 |
| BLine (tuned) | 11.0 / 28.7 | 7.5 / 25.8 | 0.8 / 0.9 | 0.0 | 0.0 | 3.19 | 3.66 (10/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 4.2 / 9.5 | 2.0 / 8.5 | 0.8 / 0.9 | 0.0 | 0.0 | 3.87 | 4.33 (10/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 5.3 / 12.3 | 5.3 / 12.4 | 3.4 / 3.6 | 0.0 | 3.6 | 2.47 | never | 5242 | 2.79 |
| PathPlanner (kP 5) + hold | 5.3 / 12.3 | 5.3 / 12.4 | 0.1 / 0.1 | 0.0 | 3.6 | 2.47 | 2.93 (10/10) | 5242 | 2.91 |
| PathPlanner (tuned) + hold | 0.7 / 1.6 | 0.5 / 1.5 | 0.6 / 1.2 | 0.0 | 1.3 | 3.10 | 3.32 (10/10) | 328 | 2.76 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.0 / 3.1 | 1.1 / 3.4 | 0.1 / 0.1 | 0.0 | 3.4 | 2.57 | 3.02 (10/10) | 4203 | 2.59 |
| DriveToPose chain | 4.2 / 8.7 | 0.3 / 0.7 | 0.1 / 0.1 | 0.0 | 0.0 | 4.74 | 4.94 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 8.4 / 17.0 | 1.4 / 2.0 | 1.5 / 1.5 | 0.0 | 2.2 | 2.77 | 2.99 (10/10) | 261 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 9.0 / 18.4 | 2.4 / 3.8 | 0.1 / 0.1 | 0.2 | 0.0 | 2.85 | 3.10 (10/10) | 16 | 2.14 |
| Ours v2 (as shipped) | 1.4 / 3.0 | 1.3 / 2.2 | 0.1 / 0.1 | 0.0 | 0.1 | 2.80 | 2.94 (10/10) | 21 | 0.43 |
| Ours v2 (as shipped, adaptive grip off) | 1.4 / 3.0 | 1.3 / 2.2 | 0.1 / 0.1 | 0.0 | 0.1 | 2.80 | 2.94 (10/10) | 21 | 0.43 |
| Ours v2 (tuned) | 1.4 / 3.0 | 1.3 / 2.2 | 0.1 / 0.1 | 0.0 | 0.1 | 2.80 | 2.94 (10/10) | 21 | 0.42 |

**long_auto**: auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end (14.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.0 / 10.8 | 5.0 / 10.8 | 3.0 / 3.2 | 2.3 | 3.2 | 4.11 | never | 466 | 102.92 |
| BLine (README gains, r 0.30) | 32.2 / 86.2 | 36.5 / 100.5 | 1.8 / 2.4 | 0.4 | 64.0 | 5.55 | 6.65 (7/10) | 630 | 0.02 |
| BLine (README gains, r 0.45 default) | 31.2 / 76.8 | 34.3 / 89.8 | 2.2 / 2.4 | 0.5 | 51.4 | 5.10 | never | 206 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 10.2 / 24.7 | 2.4 / 11.9 | 2.5 / 2.9 | 1.8 | 4.1 | 4.94 | never | 454 | 0.02 |
| BLine (tuned) | 23.1 / 56.0 | 25.7 / 65.2 | 0.9 / 0.9 | 1.7 | 0.0 | 3.93 | 4.46 (5/10) | 50 | 0.02 |
| BLine (tuned, with corner caps) | 8.6 / 22.4 | 2.2 / 13.4 | 0.8 / 0.9 | 0.4 | 0.0 | 5.24 | 5.70 (10/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 1.8 / 5.2 | 1.9 / 5.4 | 3.1 / 3.3 | 0.0 | 3.3 | 3.63 | never | 2550 | 4.61 |
| PathPlanner (kP 5) + hold | 1.8 / 5.2 | 1.9 / 5.4 | 0.1 / 0.1 | 0.0 | 3.3 | 3.63 | 4.07 (10/10) | 2550 | 4.61 |
| PathPlanner (tuned) + hold | 0.7 / 1.6 | 0.6 / 1.6 | 0.6 / 1.1 | 0.0 | 1.2 | 4.16 | 4.38 (10/10) | 0 | 6.50 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.3 / 2.9 | 1.3 / 2.9 | 0.1 / 0.1 | 0.0 | 3.4 | 3.70 | 4.14 (10/10) | 1817 | 4.52 |
| DriveToPose chain | 6.9 / 21.3 | 1.1 / 3.3 | 0.1 / 0.4 | 0.2 | 0.4 | 6.40 | 6.55 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 7.3 / 21.8 | 1.5 / 2.9 | 1.5 / 1.7 | 1.1 | 2.3 | 3.75 | 3.98 (7/10) | 214 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.0 / 22.1 | 2.8 / 5.7 | 0.1 / 0.2 | 0.3 | 0.0 | 3.84 | 4.10 (10/10) | 179 | 2.81 |
| Ours v2 (as shipped) | 3.9 / 12.3 | 1.6 / 6.0 | 0.1 / 0.1 | 0.3 | 0.1 | 3.87 | 4.01 (10/10) | 265 | 0.49 |
| Ours v2 (as shipped, adaptive grip off) | 3.9 / 12.3 | 1.6 / 6.0 | 0.1 / 0.1 | 0.3 | 0.1 | 3.87 | 4.01 (10/10) | 265 | 0.56 |
| Ours v2 (tuned) | 3.9 / 12.3 | 1.6 / 6.0 | 0.1 / 0.1 | 0.3 | 0.1 | 3.87 | 4.01 (10/10) | 265 | 0.53 |

**moving_start**: starts moving 2 m/s sideways (+y), path heads +x then curves left (7.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 12.0 / 27.7 | 12.0 / 27.7 | 3.4 / 3.5 | 0.0 | 3.5 | 1.90 | never | 1584 | 41.53 |
| BLine (README gains, r 0.30) | 31.3 / 58.4 | 32.4 / 66.0 | 28.5 / 30.3 | 0.0 | 30.1 | 3.48 | never | 866 | 0.02 |
| BLine (README gains, r 0.45 default) | 33.4 / 54.6 | 34.5 / 51.3 | 18.5 / 36.2 | 0.0 | 54.3 | 2.08 | 3.06 (5/10) | 514 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 24.6 / 49.4 | 25.3 / 47.0 | 11.0 / 11.8 | 0.0 | 11.8 | 2.18 | never | 504 | 0.02 |
| BLine (tuned) | 35.1 / 55.9 | 33.9 / 51.9 | 0.8 / 0.8 | 0.0 | 4.3 | 2.06 | 2.44 (10/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 20.2 / 47.5 | 21.1 / 45.5 | 0.8 / 0.9 | 0.0 | 0.0 | 3.10 | 3.57 (10/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 20.0 / 33.5 | 20.0 / 33.5 | 3.2 / 3.4 | 0.0 | 3.5 | 1.44 | never | 4275 | 1.82 |
| PathPlanner (kP 5) + hold | 20.0 / 33.5 | 20.0 / 33.5 | 0.1 / 0.1 | 0.0 | 3.5 | 1.44 | 1.89 (10/10) | 4275 | 1.79 |
| PathPlanner (tuned) + hold | 14.5 / 31.3 | 14.5 / 31.3 | 0.3 / 1.1 | 0.0 | 1.3 | 1.65 | 1.92 (10/10) | 4646 | 1.78 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 20.0 / 33.5 | 20.0 / 33.5 | 0.1 / 0.1 | 0.0 | 3.5 | 1.45 | 1.89 (10/10) | 4224 | 2.22 |
| DriveToPose chain | 21.5 / 58.6 | 19.9 / 54.7 | 0.1 / 0.1 | 0.0 | 0.0 | 3.70 | 3.90 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 10.6 / 24.7 | 10.5 / 25.6 | 1.5 / 1.8 | 0.0 | 2.4 | 1.72 | 1.93 (10/10) | 4562 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 11.7 / 28.1 | 11.7 / 29.5 | 0.1 / 0.1 | 0.3 | 0.0 | 1.85 | 2.11 (10/10) | 1957 | 1.65 |
| Ours v2 (as shipped) | 11.5 / 28.6 | 11.6 / 28.6 | 0.1 / 0.1 | 0.0 | 0.1 | 1.84 | 1.98 (10/10) | 1515 | 0.40 |
| Ours v2 (as shipped, adaptive grip off) | 11.2 / 28.1 | 11.3 / 28.1 | 0.1 / 0.1 | 0.0 | 0.1 | 1.81 | 1.95 (10/10) | 1782 | 0.38 |
| Ours v2 (tuned) | 11.5 / 28.6 | 11.6 / 28.6 | 0.1 / 0.1 | 0.0 | 0.1 | 1.84 | 1.98 (10/10) | 1515 | 0.37 |

**hit_mid_path**: the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 14.2 / 37.4 | 14.2 / 37.4 | 1.6 / 2.0 | 0.0 | 1.8 | 2.31 | 2.49 (10/10) | 3178 | 51.21 |
| BLine (README gains, r 0.30) | 34.5 / 74.5 | 36.4 / 78.1 | 37.1 / 42.6 | 0.0 | 67.3 | 4.32 | 5.46 (1/10) | 4037 | 0.02 |
| BLine (README gains, r 0.45 default) | 25.1 / 63.0 | 26.9 / 66.0 | 2.1 / 2.3 | 0.0 | 80.8 | 3.12 | 3.73 (1/10) | 3318 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 6.6 / 17.9 | 7.7 / 22.5 | 12.0 / 12.8 | 0.0 | 12.7 | 2.28 | never | 2938 | 0.02 |
| BLine (tuned) | 17.5 / 38.2 | 18.9 / 43.4 | 0.8 / 0.8 | 0.0 | 1.6 | 2.08 | 2.30 (10/10) | 2978 | 0.02 |
| BLine (tuned, with corner caps) | 9.1 / 22.2 | 10.8 / 26.2 | 0.8 / 0.9 | 0.0 | 0.0 | 2.39 | 2.74 (10/10) | 2848 | 0.02 |
| PathPlanner (kP 5) | 22.4 / 53.4 | 22.5 / 53.4 | 32.0 / 32.5 | 0.0 | 32.6 | never | never | 5517 | 2.10 |
| PathPlanner (kP 5) + hold | 22.4 / 53.4 | 22.5 / 53.4 | 0.1 / 0.1 | 0.0 | 31.1 | 2.49 | 2.79 (10/10) | 5856 | 2.07 |
| PathPlanner (tuned) + hold | 16.6 / 40.1 | 16.7 / 40.2 | 0.2 / 0.2 | 0.0 | 11.6 | 2.53 | 2.67 (10/10) | 5157 | 2.00 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 21.5 / 50.0 | 21.5 / 50.0 | 0.1 / 0.1 | 0.0 | 24.4 | 2.44 | 2.75 (10/10) | 5578 | 2.04 |
| DriveToPose chain | 8.2 / 29.2 | 8.8 / 31.5 | 0.1 / 0.1 | 0.0 | 0.0 | 4.18 | 4.39 (10/10) | 2293 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 49.1 / 97.6 | 47.6 / 95.3 | 39.0 / 41.3 | 0.2 | 24.1 | never | never | 5688 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 48.3 / 103.4 | 46.8 / 99.3 | 0.1 / 0.1 | 0.2 | 62.6 | 2.94 | 3.20 (10/10) | 7141 | 1.57 |
| Ours v2 (as shipped) | 15.1 / 36.4 | 15.4 / 37.5 | 0.1 / 0.3 | 0.0 | 0.3 | 2.08 | 2.22 (10/10) | 3771 | 0.39 |
| Ours v2 (as shipped, adaptive grip off) | 16.0 / 37.8 | 16.3 / 38.9 | 0.1 / 0.1 | 0.0 | 0.1 | 2.08 | 2.22 (10/10) | 3675 | 0.40 |
| Ours v2 (tuned) | 15.1 / 36.4 | 15.4 / 37.5 | 0.1 / 0.3 | 0.0 | 0.3 | 2.08 | 2.22 (10/10) | 3771 | 0.37 |

**low_grip_s_curve**: the S-curve on mu 0.70 static / 0.61 kinetic (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.5 / 8.8 | 4.5 / 8.8 | 10.7 / 10.9 | 0.0 | 10.9 | 2.23 | never | 4717 | 49.62 |
| BLine (README gains, r 0.30) | 14.0 / 39.9 | 15.2 / 39.9 | 18.5 / 19.3 | 0.0 | 96.5 | 3.06 | never | 11706 | 0.03 |
| BLine (README gains, r 0.45 default) | 9.3 / 28.6 | 10.3 / 28.6 | 15.1 / 15.6 | 0.0 | 91.7 | 2.99 | never | 11048 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.8 / 7.0 | 1.2 / 5.9 | 22.5 / 24.4 | 0.0 | 24.4 | 2.23 | never | 5354 | 0.02 |
| BLine (tuned) | 6.5 / 15.5 | 4.3 / 14.8 | 0.8 / 0.8 | 0.0 | 0.0 | 2.66 | 3.13 (10/10) | 5515 | 0.02 |
| BLine (tuned, with corner caps) | 2.7 / 6.1 | 3.5 / 10.9 | 0.8 / 0.8 | 0.0 | 0.0 | 2.70 | 3.16 (10/10) | 6779 | 0.02 |
| PathPlanner (kP 5) | 13.5 / 36.8 | 13.6 / 36.8 | 88.0 / 88.5 | 0.0 | 84.8 | never | never | 8528 | 1.96 |
| PathPlanner (kP 5) + hold | 13.5 / 36.8 | 13.6 / 36.8 | 0.1 / 0.1 | 0.0 | 83.5 | 2.91 | 3.35 (10/10) | 11646 | 2.60 |
| PathPlanner (tuned) + hold | 1.0 / 3.3 | 1.1 / 3.5 | 0.1 / 0.2 | 0.0 | 2.7 | 2.18 | 2.52 (10/10) | 7259 | 1.93 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 12.8 / 33.3 | 12.8 / 33.3 | 0.1 / 0.1 | 0.0 | 61.7 | 2.84 | 3.14 (10/10) | 10456 | 2.00 |
| DriveToPose chain | 2.7 / 6.0 | 0.6 / 1.4 | 1.6 / 1.7 | 0.0 | 1.7 | 3.80 | 3.94 (10/10) | 13094 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 24.3 / 68.9 | 22.6 / 66.4 | 113.8 / 114.1 | 0.3 | 103.7 | never | never | 9624 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 10.2 / 22.2 | 7.9 / 17.8 | 0.1 / 0.1 | 0.2 | 32.8 | 2.66 | 2.92 (10/10) | 10082 | 2.10 |
| Ours v2 (as shipped) | 1.3 / 3.3 | 1.1 / 2.3 | 0.1 / 0.2 | 0.0 | 0.2 | 2.54 | 2.68 (10/10) | 296 | 0.37 |
| Ours v2 (as shipped, adaptive grip off) | 1.0 / 2.2 | 1.1 / 3.0 | 0.1 / 0.2 | 0.0 | 6.7 | 2.04 | 2.46 (10/10) | 8034 | 0.39 |
| Ours v2 (tuned) | 1.3 / 3.3 | 1.1 / 2.3 | 0.1 / 0.2 | 0.0 | 0.2 | 2.54 | 2.68 (10/10) | 296 | 0.38 |

**grip_drop_s_curve**: the S-curve; halfway along, the floor drops from mu 1.15 / 1.0 to 0.70 / 0.61 (a worn carpet seam) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 8.8 | 4.6 / 8.8 | 10.7 / 10.9 | 0.0 | 10.9 | 2.22 | never | 3275 | 43.95 |
| BLine (README gains, r 0.30) | 13.1 / 39.1 | 14.3 / 39.1 | 18.1 / 18.9 | 0.0 | 96.3 | 3.03 | never | 7408 | 0.02 |
| BLine (README gains, r 0.45 default) | 8.7 / 28.0 | 9.7 / 28.0 | 15.0 / 15.4 | 0.0 | 91.4 | 2.97 | never | 7174 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.9 / 6.8 | 1.3 / 6.2 | 22.7 / 24.1 | 0.0 | 24.1 | 2.21 | never | 2656 | 0.02 |
| BLine (tuned) | 6.7 / 15.4 | 4.4 / 14.7 | 0.8 / 0.8 | 0.0 | 0.0 | 2.65 | 3.11 (10/10) | 2245 | 0.02 |
| BLine (tuned, with corner caps) | 2.7 / 5.9 | 3.4 / 10.7 | 0.8 / 0.9 | 0.0 | 0.0 | 2.68 | 3.14 (10/10) | 2805 | 0.02 |
| PathPlanner (kP 5) | 6.4 / 19.9 | 6.4 / 19.8 | 52.3 / 52.7 | 0.0 | 52.6 | never | never | 4563 | 2.03 |
| PathPlanner (kP 5) + hold | 6.4 / 19.9 | 6.4 / 19.8 | 0.1 / 0.1 | 0.0 | 51.2 | 2.74 | 3.04 (10/10) | 6232 | 2.09 |
| PathPlanner (tuned) + hold | 1.0 / 3.1 | 1.0 / 3.3 | 0.1 / 0.2 | 0.0 | 2.4 | 2.18 | 2.52 (10/10) | 3758 | 1.96 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 5.9 / 16.8 | 5.9 / 16.8 | 0.1 / 0.1 | 0.0 | 37.9 | 2.67 | 2.97 (10/10) | 5299 | 1.90 |
| DriveToPose chain | 2.7 / 6.0 | 0.4 / 1.4 | 1.5 / 1.6 | 0.0 | 1.6 | 3.98 | 4.12 (10/10) | 6486 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 8.5 / 18.7 | 6.1 / 14.4 | 42.5 / 43.1 | 0.2 | 41.8 | never | never | 4576 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.3 / 17.0 | 5.7 / 12.5 | 0.1 / 0.1 | 0.1 | 25.2 | 2.53 | 2.78 (10/10) | 5131 | 1.54 |
| Ours v2 (as shipped) | 1.2 / 4.3 | 1.4 / 5.4 | 0.1 / 0.3 | 0.0 | 0.4 | 2.08 | 2.25 (10/10) | 2995 | 0.40 |
| Ours v2 (as shipped, adaptive grip off) | 0.9 / 1.8 | 1.1 / 2.8 | 0.1 / 0.2 | 0.0 | 6.2 | 1.99 | 2.39 (10/10) | 4059 | 0.39 |
| Ours v2 (tuned) | 1.2 / 4.3 | 1.4 / 5.4 | 0.1 / 0.3 | 0.0 | 0.4 | 2.08 | 2.25 (10/10) | 2995 | 0.41 |

## Pathfinding course

The pathfinding course (PathPlanner AD* and ours, four start/goal pairs) has its own section below, written by `python bench/run.py --pathfind-only` (and by the full run).

## Geometry each method authored

**straight_3m**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (3.00, 0.00) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (1.00, 0.00); (3.00, 0.00): (2.00, 0.00), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, R 0 deg @t 1, (3.00, 0.00) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 4
- Choreo `straight_3m.traj`: 30 samples, 1.418 s
- DriveToPose chain goals: (3.00, 0.00) @0 deg
- Neutral path length 3.00 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**s_curve**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.42, 0.00); (1.25, 0.00): (0.83, -0.13), (1.67, 0.13); (2.50, 0.80): (2.08, 0.53), (2.92, 1.07); (3.75, 1.60): (3.33, 1.47), (4.17, 1.73); (5.00, 1.60): (4.58, 1.60), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), R 0 deg @t 1, (5.00, 1.60) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 2.8, 2.8, 2.8, 2.8
- Choreo `s_curve.traj`: 79 samples, 2.105 s
- DriveToPose chain goals: (1.25, 0.00) @0 deg, (2.50, 0.80) @0 deg, (3.75, 1.60) @0 deg, (5.00, 1.60) @0 deg
- Neutral path length 5.50 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**hairpin**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.00, 0.00), (2.00, 0.00), (2.50, 0.13), (2.87, 0.50), (3.00, 1.00), (2.87, 1.50), (2.50, 1.87), (2.00, 2.00), (1.00, 2.00), (0.00, 2.00) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.33, 0.00); (1.00, 0.00): (0.67, 0.00), (1.33, 0.00); (2.00, 0.00): (1.75, -0.02), (2.25, 0.02); (2.50, 0.13): (2.36, 0.05), (2.64, 0.22); (2.87, 0.50): (2.78, 0.36), (2.95, 0.64); (3.00, 1.00): (3.00, 0.83), (3.00, 1.17); (2.87, 1.50): (2.95, 1.36), (2.78, 1.64); (2.50, 1.87): (2.64, 1.78), (2.36, 1.95); (2.00, 2.00): (2.25, 1.98), (1.75, 2.02); (1.00, 2.00): (1.33, 2.00), (0.67, 2.00); (0.00, 2.00): (0.33, 2.00), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.00, 0.00), (2.00, 0.00), (2.50, 0.13), (2.87, 0.50), (3.00, 1.00), (2.87, 1.50), (2.50, 1.87), (2.00, 2.00), (1.00, 2.00), R 0 deg @t 1, (0.00, 2.00) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4
- Choreo `hairpin.traj`: 141 samples, 2.999 s
- DriveToPose chain goals: (1.00, 0.00) @0 deg, (2.00, 0.00) @0 deg, (2.50, 0.13) @0 deg, (2.87, 0.50) @0 deg, (3.00, 1.00) @0 deg, (2.87, 1.50) @0 deg, (2.50, 1.87) @0 deg, (2.00, 2.00) @0 deg, (1.00, 2.00) @0 deg, (0.00, 2.00) @0 deg
- Neutral path length 7.14 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**slalom**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.75, 0.40), (3.50, -0.40), (5.25, 0.40), (7.00, 0.00) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.58, 0.13); (1.75, 0.40): (1.17, 0.47), (2.33, 0.33); (3.50, -0.40): (2.92, -0.40), (4.08, -0.40); (5.25, 0.40): (4.67, 0.33), (5.83, 0.47); (7.00, 0.00): (6.42, 0.13), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.75, 0.40), (3.50, -0.40), (5.25, 0.40), R 0 deg @t 1, (7.00, 0.00) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 2.6, 2.24, 2.24, 2.6
- Choreo `slalom.traj`: 91 samples, 2.924 s
- DriveToPose chain goals: (1.75, 0.40) @0 deg, (3.50, -0.40) @0 deg, (5.25, 0.40) @0 deg, (7.00, 0.00) @0 deg
- Neutral path length 7.51 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**long_auto**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (3.00, 0.50), (5.50, 2.00) @90 deg, (7.50, 3.20), (9.00, 2.20), (9.30, 0.30) @180 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (1.00, 0.17); (3.00, 0.50): (2.08, 0.17), (3.92, 0.83); (5.50, 2.00): (4.75, 1.55), (6.25, 2.45); (7.50, 3.20): (6.92, 3.17), (8.08, 3.23); (9.00, 2.20): (8.70, 2.68), (9.30, 1.72); (9.30, 0.30): (9.20, 0.93), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (3.00, 0.50), R 90 deg @t 1, (5.50, 2.00), (7.50, 3.20), (9.00, 2.20), R 180 deg @t 1, (9.30, 0.30) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 3.47, 3.47, 1.9, 1.9, 2.28
- Choreo `long_auto.traj`: 152 samples, 3.907 s
- DriveToPose chain goals: (3.00, 0.50) @45 deg, (5.50, 2.00) @90 deg, (7.50, 3.20) @120 deg, (9.00, 2.20) @150 deg, (9.30, 0.30) @180 deg
- Neutral path length 12.14 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**moving_start**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.50, 0.40), (2.60, 1.40), (3.00, 3.00) @0 deg
- Start velocity (field): (0.0, 2.0, 0.0)
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.50, 0.13); (1.50, 0.40): (1.07, 0.17), (1.93, 0.63); (2.60, 1.40): (2.35, 0.97), (2.85, 1.83); (3.00, 3.00): (2.87, 2.47), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.50, 0.40), (2.60, 1.40), R 0 deg @t 1, (3.00, 3.00) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 3.07, 2.75, 2.75
- Choreo `moving_start.traj`: 63 samples, 1.871 s
- DriveToPose chain goals: (1.50, 0.40) @0 deg, (2.60, 1.40) @0 deg, (3.00, 3.00) @0 deg
- Neutral path length 4.72 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**hit_mid_path**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60) @0 deg
- Shove: 800 N for 0.3 s at 50% of the path, to the left of travel
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.42, 0.00); (1.25, 0.00): (0.83, -0.13), (1.67, 0.13); (2.50, 0.80): (2.08, 0.53), (2.92, 1.07); (3.75, 1.60): (3.33, 1.47), (4.17, 1.73); (5.00, 1.60): (4.58, 1.60), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), R 0 deg @t 1, (5.00, 1.60) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 2.8, 2.8, 2.8, 2.8
- Choreo `s_curve.traj`: 79 samples, 2.105 s
- DriveToPose chain goals: (1.25, 0.00) @0 deg, (2.50, 0.80) @0 deg, (3.75, 1.60) @0 deg, (5.00, 1.60) @0 deg
- Neutral path length 5.50 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**low_grip_s_curve**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60) @0 deg
- Grip: mu 0.7 static / 0.61 kinetic
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.42, 0.00); (1.25, 0.00): (0.83, -0.13), (1.67, 0.13); (2.50, 0.80): (2.08, 0.53), (2.92, 1.07); (3.75, 1.60): (3.33, 1.47), (4.17, 1.73); (5.00, 1.60): (4.58, 1.60), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), R 0 deg @t 1, (5.00, 1.60) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 2.8, 2.8, 2.8, 2.8
- Choreo `s_curve.traj`: 79 samples, 2.105 s
- DriveToPose chain goals: (1.25, 0.00) @0 deg, (2.50, 0.80) @0 deg, (3.75, 1.60) @0 deg, (5.00, 1.60) @0 deg
- Neutral path length 5.50 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**grip_drop_s_curve**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.42, 0.00); (1.25, 0.00): (0.83, -0.13), (1.67, 0.13); (2.50, 0.80): (2.08, 0.53), (2.92, 1.07); (3.75, 1.60): (3.33, 1.47), (4.17, 1.73); (5.00, 1.60): (4.58, 1.60), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), R 0 deg @t 1, (5.00, 1.60) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 2.8, 2.8, 2.8, 2.8
- Choreo `s_curve.traj`: 79 samples, 2.105 s
- DriveToPose chain goals: (1.25, 0.00) @0 deg, (2.50, 0.80) @0 deg, (3.75, 1.60) @0 deg, (5.00, 1.60) @0 deg
- Neutral path length 5.50 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

<!-- generated:end -->

## Pathfinding: PathPlanner AD* and ours

Ours (`lib/src/main/java/frc/team5712/path/plan`, details in lib/README.md):
- each obstacle is grown by the bumpers' half-diagonal plus 5 cm, its corners rounded and approximated from
  outside, and the walls are moved in by the same amount;
- a visibility graph between the grown corners is built at boot, and A* finds the exact shortest route for those
  shapes;
- the route's corners become waypoints of the core quintic spline. The oriented bumper rectangle is swept along it
  with the planned heading, and a route point is added and the spline refit wherever the 5 cm margin breaks;
- the core velocity profile is built from the robot's current velocity.

The follower is our PathFollower as shipped (the ours_v2 row's settings). No query on these four routes needed a
refit. In 300 random JUnit pairs, 34 did and all ended clear, with the closest bumper 5.1 cm from an obstacle.

<!-- pathfind:start -->
_Generated by `python bench/run.py --pathfind-only` (10 seeds per cell)._

Field `bench/fields/obstacles_v1.json` (six convex obstacles, centre origin), routes `bench/fields/routes_v1.json`, robot 0.85 x 0.85 m with bumpers, limits 4 m/s and 6 m/s^2.

| route | start -> goal | straight line m | note |
|---|---|---|---|
| across | (-4, -1.5, 0 deg) -> (3.8, 1.4, 0 deg) | 8.32 | the original route: 8.3 m across the field through the obstacles |
| behind_wall | (-4.2, 1.8, 0 deg) -> (0, 1.6, 0 deg) | 4.20 | the goal is hidden behind wall_b: the route must go round its end |
| moving_start | (-5, 0, 0 deg), moving (0, 2) m/s -> (2.5, 3, 0 deg) | 8.08 | starts moving 2 m/s sideways (+y) |
| turn_at_goal | (1, -3, 0 deg) -> (4.5, -0.5, 90 deg) | 4.30 | past the triangle, turning 90 deg on the way |

**Planning time.** Ours in Java: the first query in a fresh JVM (cold) and the median / 95th percentile of 2,000 more (warm), from `PlanCli`; boot = growing the obstacles and building the visibility graph. In the bench: our Java time per query in the long-running BenchServer, and the whole round trip from Python through the pipe (both medians). PathPlanner's AD* is the bench's Python port: its first path and all four anytime paths (medians), relative only.

| route | ours cold ms | ours warm median / p95 ms | ours boot ms | ours in bench: Java / round trip ms | PathPlanner (Python) first / all ms |
|---|---|---|---|---|---|
| across | 11.9 | 1.13 / 1.40 | 16 | 2.48 / 3.7 | 4 / 31 |
| behind_wall | 10.0 | 0.35 / 0.59 | 16 | 1.09 / 2.4 | 4 / 8 |
| moving_start | 10.7 | 0.51 / 0.78 | 16 | 0.86 / 2.2 | 7 / 30 |
| turn_at_goal | 9.7 | 0.26 / 0.48 | 16 | 0.62 / 1.9 | 3 / 12 |

### Perfect pose

**across**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 9.36 | 3.19 | never | 2.7 / 2.9 | 6.5 | 0 | 3597 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 9.36 | 3.19 | 3.60 (10/10) | 0.1 / 0.1 | 6.5 | 0 | 3597 |
| Ours (visibility graph + A* + quintic, our follower) | 9.25 (corners 9.19) | 3.18 | 3.32 (10/10) | 0.1 / 0.2 | 4.4 | 0 | 16 |

**behind_wall**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 6.33 | 2.49 | never | 2.7 / 3.1 | 0.0 | 0 | 3678 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 6.33 | 2.49 | 2.91 (10/10) | 0.1 / 0.1 | 0.0 | 0 | 3678 |
| Ours (visibility graph + A* + quintic, our follower) | 6.25 (corners 6.19) | 2.71 | 2.85 (10/10) | 0.1 / 0.2 | 5.4 | 0 | 0 |

**moving_start**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 9.48 | 2.76 | never | 3.2 / 3.5 | 1.5 | 0 | 3178 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 9.48 | 2.76 | 3.21 (10/10) | 0.1 / 0.1 | 1.5 | 0 | 3178 |
| Ours (visibility graph + A* + quintic, our follower) | 9.58 (corners 9.41) | 2.99 | 3.13 (10/10) | 0.1 / 0.2 | 6.1 | 0 | 98 |

**turn_at_goal**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 4.97 | 1.96 | never | 3.0 / 3.2 | 16.2 | 0 | 1945 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 4.97 | 1.96 | 2.39 (10/10) | 0.1 / 0.1 | 16.2 | 0 | 1945 |
| Ours (visibility graph + A* + quintic, our follower) | 4.91 (corners 4.82) | 2.10 | 2.24 (10/10) | 0.1 / 0.2 | 16.9 | 0 | 21 |

### Wheel odometry + gyro

**across**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 9.36 | 3.28 (4/10) | never | 9.1 / 20.2 | 0.0 | 4 | 3192 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 9.36 | 3.28 (4/10) | never | 10.0 / 19.9 | 0.0 | 4 | 3192 |
| Ours (visibility graph + A* + quintic, our follower) | 9.25 (corners 9.19) | 3.22 (4/10) | never | 7.3 / 16.5 | 0.0 | 20 | 16 |

**behind_wall**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 6.33 | never | never | 10.6 / 14.4 | 0.0 | 4 | 3208 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 6.33 | never | never | 13.1 / 16.2 | 0.0 | 4 | 3208 |
| Ours (visibility graph + A* + quintic, our follower) | 6.25 (corners 6.19) | 2.75 (8/10) | 2.87 (2/10) | 4.0 / 8.5 | 0.0 | 12 | 0 |

**moving_start**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 9.48 | never | never | 12.7 / 22.4 | 6.6 | 0 | 2590 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 9.48 | never | never | 14.9 / 25.8 | 6.6 | 0 | 2590 |
| Ours (visibility graph + A* + quintic, our follower) | 9.58 (corners 9.41) | 3.02 (7/10) | 3.13 (1/10) | 6.7 / 16.9 | 4.8 | 0 | 87 |

**turn_at_goal**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 4.97 | never | never | 7.4 / 12.3 | 15.8 | 0 | 1746 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 4.97 | never | never | 8.4 / 11.8 | 15.8 | 0 | 1746 |
| Ours (visibility graph + A* + quintic, our follower) | 4.91 (corners 4.82) | 2.13 (8/10) | 2.24 (3/10) | 3.9 / 9.1 | 16.3 | 0 | 18 |

### Slip-aware odometry + IMU

**across**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 9.36 | 3.23 (5/10) | 3.34 (1/10) | 8.0 / 21.4 | 1.4 | 0 | 3521 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 9.36 | 3.23 (5/10) | never | 8.9 / 22.8 | 1.4 | 0 | 3521 |
| Ours (visibility graph + A* + quintic, our follower) | 9.25 (corners 9.19) | 3.22 (4/10) | never | 7.3 / 16.5 | 0.0 | 20 | 16 |

**behind_wall**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 6.33 | 2.58 (5/10) | never | 6.3 / 12.7 | 0.0 | 4 | 3624 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 6.33 | 2.58 (5/10) | never | 8.3 / 13.1 | 0.0 | 4 | 3624 |
| Ours (visibility graph + A* + quintic, our follower) | 6.25 (corners 6.19) | 2.75 (8/10) | 2.87 (2/10) | 4.0 / 8.5 | 0.0 | 12 | 0 |

**moving_start**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 9.48 | 2.80 (7/10) | never | 7.3 / 18.8 | 0.0 | 12 | 2998 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 9.48 | 2.80 (7/10) | 3.21 (2/10) | 7.7 / 21.4 | 0.0 | 12 | 2998 |
| Ours (visibility graph + A* + quintic, our follower) | 9.58 (corners 9.41) | 3.02 (7/10) | 3.13 (1/10) | 6.7 / 16.9 | 4.8 | 0 | 87 |

**turn_at_goal**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 4.97 | 2.03 (4/10) | never | 6.1 / 11.6 | 15.8 | 0 | 1836 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 4.97 | 2.03 (4/10) | never | 7.7 / 11.5 | 15.8 | 0 | 1836 |
| Ours (visibility graph + A* + quintic, our follower) | 4.91 (corners 4.82) | 2.13 (8/10) | 2.24 (3/10) | 3.9 / 9.1 | 16.3 | 0 | 18 |

### Ours: bumper margin

The margin is how far beyond the bumpers' half-diagonal the obstacles are grown, and what the swept check demands of the planned path. Contact runs = runs (of the seeds) where the TRUE bumper square touched an obstacle or wall.

| route | pose | margin m | path m | t to 5 cm s | t settled s (n/N) | min clearance cm (worst run) | contact runs | contact ms (worst run) |
|---|---|---|---|---|---|---|---|---|
| across | Perfect pose | 0.05 | 9.25 | 3.18 | 3.32 (10/10) | 4.4 | 0/10 | 0 |
| across | Perfect pose | 0.10 | 9.40 | 3.50 | 3.64 (10/10) | 10.3 | 0/10 | 0 |
| across | Perfect pose | 0.15 | 12.05 | 3.92 | 4.05 (10/10) | 15.7 | 0/10 | 0 |
| across | Wheel odometry + gyro | 0.05 | 9.25 | 3.22 (4/10) | never | 0.0 | 1/10 | 20 |
| across | Wheel odometry + gyro | 0.10 | 9.40 | 3.53 (4/10) | never | 2.7 | 0/10 | 0 |
| across | Wheel odometry + gyro | 0.15 | 12.05 | 3.95 (4/10) | never | 4.0 | 0/10 | 0 |
| across | Slip-aware odometry + IMU | 0.05 | 9.25 | 3.22 (4/10) | never | 0.0 | 1/10 | 20 |
| across | Slip-aware odometry + IMU | 0.10 | 9.40 | 3.53 (4/10) | never | 2.7 | 0/10 | 0 |
| across | Slip-aware odometry + IMU | 0.15 | 12.05 | 3.95 (4/10) | never | 4.0 | 0/10 | 0 |
| behind_wall | Perfect pose | 0.05 | 6.25 | 2.71 | 2.85 (10/10) | 5.4 | 0/10 | 0 |
| behind_wall | Perfect pose | 0.10 | 16.15 | 5.97 | 6.12 (10/10) | 10.5 | 0/10 | 0 |
| behind_wall | Perfect pose | 0.15 | 19.05 | 6.42 | 6.56 (10/10) | 15.7 | 0/10 | 0 |
| behind_wall | Wheel odometry + gyro | 0.05 | 6.25 | 2.75 (8/10) | 2.87 (2/10) | 0.0 | 1/10 | 12 |
| behind_wall | Wheel odometry + gyro | 0.10 | 16.15 | 5.98 (9/10) | 6.10 (4/10) | 0.0 | 1/10 | 48 |
| behind_wall | Wheel odometry + gyro | 0.15 | 19.05 | 6.43 (9/10) | 6.54 (4/10) | 0.0 | 1/10 | 8 |
| behind_wall | Slip-aware odometry + IMU | 0.05 | 6.25 | 2.75 (8/10) | 2.87 (2/10) | 0.0 | 1/10 | 12 |
| behind_wall | Slip-aware odometry + IMU | 0.10 | 16.15 | 5.98 (9/10) | 6.10 (4/10) | 0.0 | 1/10 | 48 |
| behind_wall | Slip-aware odometry + IMU | 0.15 | 19.05 | 6.43 (9/10) | 6.54 (4/10) | 0.0 | 1/10 | 8 |
| moving_start | Perfect pose | 0.05 | 9.58 | 2.99 | 3.13 (10/10) | 6.1 | 0/10 | 0 |
| moving_start | Perfect pose | 0.10 | 13.75 | 5.14 | 5.28 (10/10) | 10.5 | 0/10 | 0 |
| moving_start | Perfect pose | 0.15 | 16.54 | 5.57 | 5.71 (10/10) | 15.6 | 0/10 | 0 |
| moving_start | Wheel odometry + gyro | 0.05 | 9.58 | 3.02 (7/10) | 3.13 (1/10) | 4.8 | 0/10 | 0 |
| moving_start | Wheel odometry + gyro | 0.10 | 13.75 | 5.15 (4/10) | 5.28 (2/10) | 0.0 | 1/10 | 24 |
| moving_start | Wheel odometry + gyro | 0.15 | 16.54 | 5.57 (4/10) | 5.69 (1/10) | 0.1 | 0/10 | 0 |
| moving_start | Slip-aware odometry + IMU | 0.05 | 9.58 | 3.02 (7/10) | 3.13 (1/10) | 4.8 | 0/10 | 0 |
| moving_start | Slip-aware odometry + IMU | 0.10 | 13.75 | 5.15 (4/10) | 5.28 (2/10) | 0.0 | 1/10 | 24 |
| moving_start | Slip-aware odometry + IMU | 0.15 | 16.54 | 5.57 (4/10) | 5.69 (1/10) | 0.1 | 0/10 | 0 |
| turn_at_goal | Perfect pose | 0.05 | 4.91 | 2.10 | 2.24 (10/10) | 16.9 | 0/10 | 0 |
| turn_at_goal | Perfect pose | 0.10 | 4.97 | 2.10 | 2.24 (10/10) | 21.9 | 0/10 | 0 |
| turn_at_goal | Perfect pose | 0.15 | 5.02 | 2.10 | 2.24 (10/10) | 26.8 | 0/10 | 0 |
| turn_at_goal | Wheel odometry + gyro | 0.05 | 4.91 | 2.13 (8/10) | 2.24 (3/10) | 16.3 | 0/10 | 0 |
| turn_at_goal | Wheel odometry + gyro | 0.10 | 4.97 | 2.14 (8/10) | 2.24 (3/10) | 21.2 | 0/10 | 0 |
| turn_at_goal | Wheel odometry + gyro | 0.15 | 5.02 | 2.12 (7/10) | 2.24 (2/10) | 26.1 | 0/10 | 0 |
| turn_at_goal | Slip-aware odometry + IMU | 0.05 | 4.91 | 2.13 (8/10) | 2.24 (3/10) | 16.3 | 0/10 | 0 |
| turn_at_goal | Slip-aware odometry + IMU | 0.10 | 4.97 | 2.14 (8/10) | 2.24 (3/10) | 21.2 | 0/10 | 0 |
| turn_at_goal | Slip-aware odometry + IMU | 0.15 | 5.02 | 2.12 (7/10) | 2.24 (2/10) | 26.1 | 0/10 | 0 |

<!-- pathfind:end -->

## Fidelity caveats

**The plant and the run**
- Plant parameters (grip, motor limits, steering lag, actuation delay) are the `alignsim` defaults, marked ASSUMED
  in `params.py`. None has been fitted to the robot. Low grip (mu 0.70 / 0.61) is an illustration.
- Only one pose source is used at a time, and neither has vision. With odometry, each seed's 1 % wheel-radius
  error and the slip over-read set a 2-8 cm floor for everyone. The perfect-pose tables rank the control laws.
- **The shove** goes through `sim.DisturbedPlant`, which uses the plant's existing `push()` external force. The
  core physics is untouched. The shove fires when the TRUE closest point on the path passes 50 % of its length,
  perpendicular to the path. Its size, 800 N for 0.3 s, was calibrated on a robot driving straight at 3 m/s
  with an open-loop command: it moves the robot 22.5 cm sideways. The tread holds up to about 680 N, so 600 N
  moves the robot 1 cm and 1000 N moves it 53 cm. On the S-curve the robot is already cornering, so the
  displacement differs by method; the worst cross-track on that course shows it.
- **Cross-track** is the distance to the authored neutral path, measured from the start until the robot first
  reaches the end (within 5 cm of the end point, or its closest path point within 5 cm of the end). "Own
  geometry" measures to the path the method itself authored: BLine's polyline, the chain's straight legs, or
  PathPlanner's 0.2 m path points.
- **Generation time** is Python wall-clock under a loaded 15-process pool. It is noisy and only meaningful as a
  ratio. The old profile's curvature-window maximum is vectorized (`maximum_filter1d`, same result), so the Java's
  roughly 800 comparisons per sample are understated here.
- Each method's field-relative request becomes robot speeds at apply time, using the latest odometry heading
  (CTRE ApplyFieldSpeeds behaviour). BLine and PathPlanner do that conversion in the control step instead, so they
  differ by at most one 4 ms odometry tick.
- **Tuning** sweeps ran with the perfect pose on seeds 100-102, across all eight courses. Evaluation used seeds 0-9.
  The score ranks settled fraction first, then mean settle time (an unsettled run counts as its whole simulated
  time) plus 10 s per metre of RMS cross-track. After widening, every chosen value sits inside its grid (the
  "best variant at each" lines in the sweep section show the score on both sides). The tuned rows are this sim's
  best, not the authors' recommendations.

**Port approximations**
- **BLine.**
  - Swerve only. Handoff mode is RADIUS, the library default; PROGRESS is not ported.
  - Not ported: events, rotation override, flip/mirror, rolling ends (min velocity 0), per-element acceleration and rotation ranges (per-target max velocity is used for the caps).
  - BLine-Web's automatic velocity constraints are ported only as far as their corner seeding
    (`courses.bline_corner_caps`, from `core/constraints/autoVelocityConstraints.ts`): effective radius
    max(d, d / tan(theta / 2)) from the handoff distance d and turn angle theta, cap sqrt(6 m/s^2 x r x f) on the
    targets before and after the corner, f = 0.85 for paths of at most 7 targets and 1.6 otherwise, floored to
    0.01 m/s, the first target at 2.0 m/s. In BLine-Web these seeds are only the start of a simulation-based solver
    (pair passes, lifting caps within a time budget, a joint radius and cap search with CMA-ES restarts, about
    7,000 lines that drive BLine-Web's own simulator), and the seeding is skipped on paths with rotation keyframes.
    Here the seeds are the final caps on every course. The solver and CMA-ES were not ported: a faithful port
    would need BLine-Web's simulator as well, so there is no CMA-ES row.
  - Not ported: BLine-Web's automatic handoff radii. The tuned rows sweep the radius instead.
  - Gains are P-only, as in the README. End tolerances are 3 cm / 2 deg from the BLine-Web defaults.
  - BLine-Web's curve-to-targets tool (`to_bline_curve`) is kept in the code. Its row was dropped this pass: it
    only adds one target, on the 12 m route, so elsewhere it equalled the r 0.45 row.
- **PathPlanner.**
  - `PathPlannerPath.createPath`, `PathPlannerTrajectory` (module-level forward/reverse passes with motor torque,
    friction and curve-radius limits, timing) and `PPHolonomicDriveController` are ported line for line in structure.
  - RobotConfig comes from the sim: 60 kg, MOI 6, 2 in wheels, 6.75:1, Kraken X60 FOC at 120 A, max drive speed =
    the sim's 4.57 m/s free speed, wheel COF 1.15 = the plant's static mu. It is not changed for low grip.
  - Drive feedforwards are not computed, because the plant takes velocity requests.
  - The trajectory is always generated from the measured start speeds. This is FollowPathCommand's branch when the
    robot doesn't match the IdealStartingState; when it does, the ideal trajectory is the same generation.
  - Not ported: events, point-towards zones, constraint zones, flipping, Choreo.
  - "+ hold" is the drive-to-pose race's variant, not a PathPlanner feature: after the trajectory ends it keeps the
    P controllers on the final state until within 2 cm / 2 deg and under 5 cm/s.
- **Old path-editor FollowPath.**
  - `FollowPath` calls `rotationSupplier.getOmega`, but `RotationSupplier` defines `getTargetHeading`. The supplier
    is read as a target heading (`interpolateAlongPath` over the course's headings). A P controller turns it into
    omega, with kP 5 (PathPlanner's rotation kP); the repo has no heading gain.
  - `AccelerationLimiter.integrateVelocity`, `normalizeSpeeds` and `Motor.getMaxAcceleration` are not in the repo.
    The real AccelerationLimiter was taken out of the public repo in 5bf4708 ("Make AccelerationLimiter and Motor
    private"), so it was not ported. Stand-ins:
    - `integrateVelocity` = the drive-to-pose race's `AccelerationLimiter.limit` with MU 1.0, as shipped;
    - `normalizeSpeeds` = scale vx, vy and omega so no module exceeds 4.0 m/s;
    - `Motor.getMaxAcceleration` = the limiter's Kraken FOC model at 150 A.
  - `VelocityConstraints` uses the shared 4.0 m/s and 6 m/s^2, with the shipped 0.9 g curvature fraction.
- **DriveToPose chain.**
  - One DriveToPose command per waypoint, each ending on DriveToPose's own isFinished (2 cm, 2 deg, measured speed
    under 5 cm/s).
  - Limits are set to the shared ones. The limiter traction is 6 m/s^2 instead of the file's MU 1.0, which is
    DriveToPose's only acceleration setting. The limiter's 8 rad/s^2 turning cap is kept.
  - Headings at waypoints that have none come from the neutral heading profile.
- **Choreo.**
  - Trajectories come from the official Choreo v2027.0.0-alpha-3 release, generated headless with its
    `choreo-cli.exe --chor bench/choreo/bench.chor --all-trajectory --generate`. The project and the six `.traj`
    files are in `bench/choreo/`. The shoved and low-grip courses reuse the S-curve trajectory.
  - Robot config (documented in `bench/choreo/make_project.py`): modules at (+-0.29, +-0.29) m, 60 kg, MOI 6 kg m^2,
    gearing 6.75, 2 in wheels, motor vmax 607.4 rad/s (Kraken X60 FOC free speed), motor tmax 2.328 N m (0.0194 N m/A
    x 120 A), cof 1.15 (the plant's static mu), bumpers 0.425 m each way. Constraints: stop points at both ends,
    max velocity 4.0 m/s, max acceleration 6 m/s^2, max angular velocity 540 deg/s. Choreo has no angular-acceleration
    constraint, so the 720 deg/s^2 limit is not imposed.
  - Waypoints with a course heading fix it; the others are translation-only, so Choreo picks the heading there.
    Choreo has no initial-velocity constraint, so the moving-start trajectory starts at rest.
  - Choreo optimizes its own path through the waypoints, so it does not follow the Catmull-Rom neutral curve.
    Cross-track "vs authored path" for Choreo is mostly that choice. "Vs own geometry" is its tracking.
  - ChoreoLib ships no controller. The follower is ChoreoLib's sampling (`Trajectory.sampleAt`,
    `SwerveSample.interpolate`) and command lifecycle (`AutoTrajectory`), with the PID example from ChoreoLib's
    getting-started docs (x/y kP 10, heading kP 7.5). After the trajectory ends the bench applies zero speeds, as a
    drivetrain default command would.
  - Generation time is rough. Wall-clock of choreo-cli per trajectory was 19-336 s (straight 19 s, S-curve
    105 s, slalom 73 s, moving start 20 s, hairpin 256 s, 12 m route 336 s), measured on this PC with other work
    running. These figures are not in `bench/choreo/`.
- **Choreo geometry + DriveToPose finish (prototype, not ours v2).**
  - Choreo's path is resampled every 2 cm of arc length and followed by projection. Speed is min(Choreo's speed
    50 ms ahead, DriveToPose's brake on the remaining arc length), with P 5 on cross-track and heading. DriveToPose
    takes over within 0.30 m of the end.
  - Every output goes through the AccelerationLimiter at its shipped MU 1.0. At 6 m/s^2 the prototype could not
    hold Choreo's corners, which use up to cof x g sideways: 8-13 cm RMS off its own path and 31-47 cm overshoot in
    a first try. That limiter setting is the only tuning it got.
- **Pathfinding (PathPlanner AD*).**
  - `bench/pathfind.py` ports LocalADStar (grid, AD* with eps 2.5 then 2.0, 1.5, 1.0, path extraction, line-of-sight
    simplification, 0.8 corner smoothing), waypointsFromPoses (0.4 handles) and PathfindingCommand (path swaps more
    than 2 m from the end, time offset from the two closest states, finish on the trajectory clock).
  - The planner runs synchronously before the command. Its four anytime paths are handed over one per 20 ms loop.
    The real planner is a background thread whose timing depends on the roboRIO.
  - Tie-breaking between equal keys follows Python's dict order, not Java's HashMap order, so a tie could pick a
    different but equally short grid path.
  - PathPlanner's grid uses a corner origin. The field file is centre-origin, so the planner shifts by half the
    field.
  - The navgrid is rasterised from the polygons: a node is blocked when its centre is within 0.60 m of an obstacle
    or (since phase 4) a field wall. That is the 0.85 m bumpers' half-diagonal, following PathPlanner's rule that a
    free node must let the robot centre pass. PathPlanner users paint this grid by hand in the GUI.
  - Clearance is the distance between the rotated bumper square and each obstacle polygon or wall at every 4 ms
    sample of the TRUE pose. PathPlanner's planning times are Python wall-clock, relative only.
- **Pathfinding (ours).**
  - The planner is Java, called through BenchServer's `pathfind` op from the measured start state, as a command
    would at its start. Each BenchServer plans once before its first timed query, as robot init would.
  - The in-bench Java times are therefore only lightly warmed, and higher than PlanCli's warm figures, which come
    after 2,000 queries. Round trip = Python wall-clock through the pipe.
  - All timings are on this PC with other work running. The roboRIO is slower and has not been measured.
  - Obstacles are convex hulls of the file's polygons. They are grown for any heading (half-diagonal), and the swept
    check then uses the real heading.
  - The 5 cm margin is checked against the planned path only. Tracking error and odometry drift come out of it,
    hence the odometry contacts.
- **Courses.** The neutral curve is the old editor's Catmull-Rom through the waypoints. PathPlanner gets Bezier
  handles that reproduce those exact cubics (checked in `tests/test_courses.py`). Catmull-Rom tightens the hairpin
  to 0.75 m at the joins.

**PathPlanner path2 (`--pp-compare` and the main tables)**
- path2 is a pre-release on PathPlanner's `new-path` branch. The robot side here is the GUI's own port of
  FollowDynamicPath's controller, followed by PathPlannerLib's Java setpoint generator. The real command may differ
  when it ships.
- Its "plan" is a simulated run (the GUI's ideal robot, ending at its end tolerance), not a profile, so it is not
  the same kind of number as ours or PathPlanner 2027's planned time. The tables put them side by side and say so.
- As shipped it has no linear acceleration limit (the setpoint generator allows what the motors and wheel COF
  1.15 allow); the tuned row adds the shared 6 m/s^2 as PathConstraints, which PathPlannerLib's generator takes.
- It finishes on distance alone (no speed check), so with a tight tolerance it can finish while passing through
  the end and coast. That is its behaviour, not a bench artefact.
- The 2026 routes' depot position is assumed; the bench does not collide with field elements.

## Sources

| What | Repository, branch, commit | Files ported |
|---|---|---|
| BLine follower | github.com/edanliahovetsky/BLine-Lib, `wpilib-2027`, 44022501faa981e5cbb9ea9492d8ac4bc67acdc5 (2026-10-15, "2027 beta.3") | `following/Follower`, `TranslationProgress`, `RotationProgress`, `TranslationGuidance`, `HolonomicController`, `HolonomicRateLimiter`; `path/PreparedPath`, `Path` (waypoint expansion); README gains |
| BLine defaults, curve tool | github.com/edanliahovetsky/BLine-Web, `bline-2027`, 57127fd43ae5d8f650160a68bf37afb2bc0b49c1 | `core/config/projectConfig.ts` (0.45 m handoff, 3 cm / 2 deg tolerances), `core/pathProfile/curveProfile.ts` |
| PathPlanner | github.com/mjansen4857/pathplanner, `2027`, 7fbd7980c5f81874d56a5b64c39fb8d8c3037352 (2026-10-03) | `path/PathPlannerPath`, `trajectory/PathPlannerTrajectory`, `PathPlannerTrajectoryState`, `config/RobotConfig`, `ModuleConfig`, `util/GeometryUtil`, `controllers/PPHolonomicDriveController`, `commands/FollowPathCommand`; Kraken X60 FOC numbers from `lib/trajectory/dc_motor.dart` |
| Old path-editor | github.com/Hemlock5712/path-editor, `master`, 29d7ada471fbee71af8325ad7fd842697d3dc83e (2026-04-25) | `public/java/commands/FollowPath.java`, `utils/path/SplinePath`, `CubicSegment`, `VelocityProfile`, `VelocityConstraints`, `RotationSupplier` |
| BLine-Web corner seeding | same BLine-Web commit | `core/constraints/autoVelocityConstraints.ts` (`createAutoVelocitySolveSetup`, `buildCorners`, `seedCapsFromCorners`, `segmentCapsFromSolvedCaps`, `roundConstraintValue`), `core/bend/cornerBend.ts` (read for the fillet geometry) |
| Choreo, ChoreoLib | github.com/SleipnirGroup/Choreo, tag `v2027.0.0-alpha-3`, 3a9e15f022f081c93420c3ccc712fd1809e49d9e (2026-09-11); release binary `Choreo-v2027.0.0-alpha-3-Windows-x86_64-standalone.zip` | `choreo-cli.exe` (run to generate); `choreolib/.../trajectory/Trajectory.java`, `SwerveSample.java`, `auto/AutoTrajectory.java`; `docs/choreolib/getting-started.md` (follower gains); `src-core/src/spec/*.rs` (file formats) |
| PathPlanner pathfinding | same PathPlanner commit | `pathfinding/LocalADStar`, `commands/PathfindingCommand`, `path/PathPlannerPath.waypointsFromPoses`, `path/Waypoint.autoControlPoints`; navgrid rule from `Writerside/topics/gui-Navigation-Grid-Editor.md` |
| PathPlanner path2 (pre-release) | github.com/mjansen4857/pathplanner, `new-path`, af86ef2b55b502c7929884f08ff2b7214035b9fb (2026-09-09); no PathPlannerLib release ships it yet | `lib/path2/simulation/path2_path_follower.dart` (FollowDynamicPath's controller), `path2_simulator.dart` (the GUI's time), `swerve_setpoint_generator.dart`, `swerve_math.dart`, `lib/path2/waypoint.dart`, `graph.dart` (handoff and end-tolerance defaults); PathPlannerLib `util/swerve/SwerveSetpointGenerator.java` at the 2027 commit above (the robot side, with PathConstraints). Read as data only |
| 2026 field (the `--pp-compare` routes) | github.com/wpilibsuite/allwpilib, `main`, c610ffd17fb144e7348af48caad7f3944a7765b6 | `fields/src/generate/resources/org/wpilib/fields/apriltag/2026-rebuilt-welded.csv` (hub, trench, outpost and tower positions) |
| DriveToPose, AccelerationLimiter | this repo, `bench/reference/DriveToPose.java`, `AccelerationLimiter.java` (via `race_drive_to_pose.py`) | `execute`, `isFinished`, `limit` |
| Plant | this repo, `bench/alignsim` (unchanged) | |
