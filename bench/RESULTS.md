# Path-following bench: results (phase 1 baseline, pass 2, ours v2 from phase 3, slip handling)

How to run: build our jar first (`lib/gradlew -p lib jar`), then `python bench/run.py --both --procs 4` (about 27 minutes; `--resume` carries on after a crash; results in `bench/out/results.json`,
gitignored). `--quick` is a 2-seed smoke test on 3 courses, `--perfect-pose` runs the true pose only,
`--md-only` rebuilds the generated part of this file from `results.json`. Tests: `python -m pytest -q bench/tests`.
Choreo trajectories: `python bench/choreo/make_project.py --cli <choreo-cli.exe>` (they are committed).

What runs: every method drives the team's `alignsim` swerve plant (500 Hz stick-slip tyres, Kraken X60 FOC curve,
steering lag, 250 Hz odometry thread, 50 Hz loop with 1 ms jitter, 2 ms actuation delay) along 17 courses (the original nine and eight harder ones), 10 noise
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

Every number below is from the run in the generated section (10 seeds per cell). That run added the eight
harder courses; the original nine courses' rows came out identical to the run before it, so the bullets about
them still hold.

**Harder courses (perfect pose; "settled" = every seed within 2 cm and 1 deg of the true end, held 0.25 s):**

| course | winner on time settled | ours v2 | what else to know |
|---|---|---|---|
| spin_while_driving | ours 2.83 s | 2.83 s | PathPlanner + hold 3.35 s; Choreo and BLine tuned never settle every seed |
| figure_eight | PathPlanner + hold, COF 0.9: 4.23 s | 4.50 s (4th) | that PathPlanner runs 9.3 cm RMS off the line (24 cm worst) with 10 s of wheel slip (summed over the four wheels); ours 1.3 cm, 16 ms |
| wall_hug | ours 3.11 s | 3.11 s | ours keeps the full 15 cm to the walls. Choreo 3.17 s at 11.7 cm. PathPlanner kP 5 comes within 1.4 cm; path2 (-20 to -26 cm) and every BLine (-54 to -79 cm) would hit the wall |
| tag_align_moving_goal | ours 1.95 s | 1.95 s | hybrid 2.09 s, tuned PathPlanner (on-the-fly path) 2.16 s, Choreo + DriveToPose handoff 2.38 s |
| intake_drive_through | (ends moving) | miss 0.3 cm at 2.05 m/s | PathPlanner's on-the-fly path misses by 0.1 cm at 2.12 m/s; BLine's rebuilt path by 0.3 cm but at 2.8 m/s (min velocity is a floor). Choreo, the old follower and path2 can't take the update: about 10 cm. DriveToPose and the hybrid stop on it |
| three_piece_chain | hybrid 4.79 s | 5.06 s (3rd) | PathPlanner + hold 5.02 s; Choreo plans the auto as one trajectory |
| fast_long | ours 2.56 s | 2.56 s | Choreo 2.61 s, hybrid 2.86 s, tuned PathPlanner 2.92 s |
| heavy_robot | Choreo and ours tie, 2.18 s | 2.18 s | tuned PathPlanner 2.46 s |

**With wheel odometry** nobody settles every seed on any harder course: the 1 % wheel-radius error alone is more
than 2 cm over these distances. By seeds settled, ours leads or ties on tag_align (9 of 10), heavy_robot (4) and
wall_hug's time (3 of 10 against Choreo's 4). It is behind PathPlanner + hold on the spin (2 against 3), behind
Choreo on the figure eight (7 against 8) and fast_long (3 against 4), and level with the hybrid on the three-piece
chain (2). The intake miss stays 0.4 cm for ours and 0.3 cm for PathPlanner. Ours' worst wall clearance drops to
3.4 cm (the odometry drift; DriveToPose 4.3 cm, PathPlanner kP 5 -10 cm).

**Where ours loses on the harder courses:**
- **The figure eight:** 0.27 s behind PathPlanner + hold at COF 0.9 and 0.01 s behind Choreo. That PathPlanner
  run is 7x further off the line and slides for 10 s, but it finishes first.
- **The three-piece chain:** 0.27 s behind the hybrid and 0.04 s behind PathPlanner + hold. Ours plans each leg
  to hand over at 2 m/s; the hybrid follows one Choreo trajectory with no handoffs at all.
- **The intake:** PathPlanner's re-planned path passes 0.2 cm closer to the piece. The difference is below
  anything an intake would notice.
- **With odometry:** fewer seeds settled than PathPlanner + hold on the spin and than Choreo on the figure eight
  and fast_long, by one seed each.
- **Heavy robot:** level with Choreo, not ahead.


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
_Generated by `python bench/run.py --both` (10 seeds, 1611 s wall)._

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
| `{'easeInPerSecond': 12.0, 'latencySeconds': 0.03, 'limiterFrictionCoefficient': 1.4}` | 1.00 | 2.93 |
| `{'easeInPerSecond': 12.0, 'latencySeconds': 0.03, 'limiterFrictionCoefficient': 2.0}` | 1.00 | 2.94 |
| `{'easeInPerSecond': 12.0, 'latencySeconds': 0.05, 'limiterFrictionCoefficient': 2.0}` | 1.00 | 2.94 |
| `{'easeInPerSecond': 16.0, 'latencySeconds': 0.05, 'limiterFrictionCoefficient': 1.0}` | 1.00 | 2.94 |

Best variant at each `easeInPerSecond`: 9: 1.00 settled, 2.94 s; 12: 1.00 settled, 2.93 s; 16: 1.00 settled, 2.94 s

**PathPlanner path2** (96 variants, perfect pose, seeds [100, 101, 102], all courses). Chosen: `{'accel_cap': 6.0, 'kp': 2.0, 'kp_ct': 2.0, 'handoff': 0.75, 'end_distance': 0.02, 'end_degrees': 1.0}`. Best five:

| options | settled fraction | score s |
|---|---|---|
| `{'accel_cap': 6.0, 'kp': 2.0, 'kp_ct': 2.0, 'handoff': 0.75, 'end_distance': 0.02, 'end_degrees': 1.0}` | 1.00 | 4.24 |
| `{'accel_cap': 6.0, 'kp': 2.0, 'kp_ct': 1.0, 'handoff': 0.75, 'end_distance': 0.02, 'end_degrees': 1.0}` | 1.00 | 4.26 |
| `{'accel_cap': 6.0, 'kp': 2.0, 'kp_ct': 2.0, 'handoff': 0.5, 'end_distance': 0.02, 'end_degrees': 1.0}` | 1.00 | 4.28 |
| `{'accel_cap': 6.0, 'kp': 2.0, 'kp_ct': 1.0, 'handoff': 0.5, 'end_distance': 0.02, 'end_degrees': 1.0}` | 1.00 | 4.30 |
| `{'accel_cap': 6.0, 'kp': 2.0, 'kp_ct': 4.0, 'handoff': 0.5, 'end_distance': 0.02, 'end_degrees': 1.0}` | 1.00 | 4.44 |

Best variant at each `handoff`: 0.1: 1.00 settled, 5.67 s; 0.25: 1.00 settled, 4.65 s; 0.5: 1.00 settled, 4.28 s; 0.75: 1.00 settled, 4.24 s

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
| Ours v2 (as shipped) | 20/90 | 0/9 | 6.6 | 39.2 | 8.7 | 842 |
| Ours v2 (as shipped, adaptive grip off) | 15/90 | 0/9 | 7.0 | 41.0 | 9.7 | 1696 |
| Ours v2 (tuned) | 20/90 | 0/9 | 6.6 | 39.2 | 8.7 | 842 |
| PathPlanner path2 (new-path, pre-release), as shipped | 3/90 | 0/9 | 14.9 | 53.6 | 24.7 | 5748 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 6/90 | 0/9 | 15.3 | 61.0 | 20.0 | 3501 |

**straight_3m**: 3 m straight, heading held at 0 (6.5 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.1 / 0.3 | 0.1 / 0.3 | 4.1 / 8.7 | 0.1 | 8.7 | 1.36 | 1.53 (1/10) | 89 | 29.43 |
| BLine (README gains, r 0.30) | 0.1 / 0.3 | 0.1 / 0.3 | 38.2 / 47.4 | 0.1 | 69.4 | 1.13 | 2.27 (1/10) | 954 | 0.02 |
| BLine (README gains, r 0.45 default) | 0.1 / 0.3 | 0.1 / 0.3 | 38.2 / 47.4 | 0.1 | 69.4 | 1.13 | 2.27 (1/10) | 954 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 0.1 / 0.3 | 0.1 / 0.3 | 38.2 / 47.4 | 0.1 | 69.4 | 1.13 | 2.27 (1/10) | 954 | 0.02 |
| BLine (tuned) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.8 | 0.1 | 4.5 | 2.18 (9/10) | 2.67 (7/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.8 | 0.1 | 4.5 | 2.18 (9/10) | 2.67 (7/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 0.1 / 0.3 | 0.1 / 0.3 | 3.9 / 8.3 | 0.1 | 8.5 | 1.26 | 1.46 (1/10) | 0 | 1.95 |
| PathPlanner (kP 5) + hold | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.1 | 8.5 | 1.26 | 1.71 (4/10) | 0 | 1.85 |
| PathPlanner (tuned) + hold | 0.1 / 0.3 | 0.1 / 0.3 | 2.7 / 6.5 | 0.1 | 6.6 | 1.42 | 1.71 (5/10) | 6 | 1.99 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.1 | 8.5 | 1.26 | 1.71 (4/10) | 0 | 2.00 |
| DriveToPose chain | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.0 | 5.2 | 1.33 (9/10) | 1.52 (5/10) | 0 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 0.3 / 1.1 | 0.1 / 0.4 | 3.0 / 6.9 | 0.0 | 7.8 | 1.26 | 1.48 (5/10) | 66 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 0.3 / 1.0 | 0.1 / 0.5 | 2.3 / 6.1 | 0.1 | 5.1 | 1.32 (9/10) | 1.58 (5/10) | 24 | 1.51 |
| Ours v2 (as shipped) | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 5.4 | 1.29 (9/10) | 1.43 (4/10) | 16 | 0.36 |
| Ours v2 (as shipped, adaptive grip off) | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 5.4 | 1.29 (9/10) | 1.43 (4/10) | 16 | 0.36 |
| Ours v2 (tuned) | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 5.4 | 1.29 (9/10) | 1.43 (4/10) | 16 | 0.36 |
| PathPlanner path2 (new-path, pre-release), as shipped | 0.1 / 0.3 | 0.1 / 0.3 | 12.2 / 15.8 | 0.1 | 0.0 | never | never | 2656 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 0.1 / 0.3 | 0.1 / 0.3 | 2.7 / 7.7 | 0.1 | 3.6 | 2.18 (9/10) | 2.53 (4/10) | 0 | 0.02 |

**s_curve**: S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.5 / 12.0 | 4.5 / 12.0 | 5.4 / 12.7 | 0.1 | 12.4 | 2.24 (9/10) | 2.42 (1/10) | 239 | 62.66 |
| BLine (README gains, r 0.30) | 13.0 / 37.3 | 14.1 / 37.4 | 6.7 / 15.1 | 0.1 | 96.7 | 3.01 (8/10) | never | 346 | 0.03 |
| BLine (README gains, r 0.45 default) | 8.2 / 27.0 | 9.0 / 27.0 | 5.9 / 13.2 | 0.1 | 91.5 | 2.93 (9/10) | never | 282 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 9.7 | 1.8 / 7.1 | 12.5 / 21.0 | 0.1 | 20.9 | 2.22 | never | 523 | 0.02 |
| BLine (tuned) | 6.6 / 16.6 | 4.4 / 15.9 | 4.2 / 11.3 | 0.1 | 8.2 | 2.61 (8/10) | 3.15 (3/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 2.9 / 7.2 | 3.2 / 10.5 | 4.2 / 11.2 | 0.1 | 8.2 | 2.71 (8/10) | 3.26 (3/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 1.6 / 3.9 | 1.6 / 3.8 | 5.8 / 13.1 | 0.1 | 12.8 | 1.88 (9/10) | 2.09 (1/10) | 1688 | 3.16 |
| PathPlanner (kP 5) + hold | 1.6 / 3.9 | 1.6 / 3.8 | 4.3 / 10.1 | 0.1 | 12.8 | 1.88 (9/10) | 2.34 (4/10) | 1688 | 3.06 |
| PathPlanner (tuned) + hold | 1.3 / 4.5 | 1.2 / 4.5 | 4.1 / 10.6 | 0.1 | 10.3 | 2.20 (9/10) | 2.44 (4/10) | 77 | 2.91 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.3 / 3.9 | 1.2 / 3.8 | 4.2 / 10.0 | 0.1 | 12.3 | 1.91 (9/10) | 2.37 (4/10) | 1411 | 3.25 |
| DriveToPose chain | 2.9 / 8.7 | 0.8 / 3.3 | 4.1 / 10.5 | 0.0 | 8.9 | 4.04 (8/10) | 4.20 (4/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 4.6 / 11.2 | 2.0 / 5.6 | 4.5 / 10.8 | 0.1 | 11.3 | 1.95 (8/10) | 2.18 (4/10) | 0 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 4.9 / 12.7 | 2.9 / 6.8 | 4.1 / 10.5 | 0.1 | 8.8 | 2.01 (8/10) | 2.27 (4/10) | 18 | 2.11 |
| Ours v2 (as shipped) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.5 | 0.1 | 9.0 | 2.02 (8/10) | 2.17 (4/10) | 16 | 0.43 |
| Ours v2 (as shipped, adaptive grip off) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.5 | 0.1 | 9.0 | 2.02 (8/10) | 2.17 (4/10) | 16 | 0.46 |
| Ours v2 (tuned) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.5 | 0.1 | 9.0 | 2.02 (8/10) | 2.17 (4/10) | 16 | 0.43 |
| PathPlanner path2 (new-path, pre-release), as shipped | 4.0 / 10.6 | 4.9 / 13.9 | 5.0 / 11.3 | 0.1 | 10.6 | 1.72 (8/10) | 1.85 (3/10) | 4829 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 7.2 / 16.6 | 5.2 / 14.6 | 5.8 / 14.1 | 0.1 | 6.9 | 2.77 (7/10) | never | 1975 | 0.02 |

**hairpin**: 2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held (9.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 7.0 / 18.4 | 7.0 / 18.4 | 1.8 / 3.8 | 0.1 | 0.8 | 2.85 | 3.01 (7/10) | 1395 | 68.31 |
| BLine (README gains, r 0.30) | 45.5 / 104.3 | 45.9 / 107.5 | 7.8 / 9.9 | 0.1 | 8.8 | 6.54 (9/10) | never | 451 | 0.03 |
| BLine (README gains, r 0.45 default) | 47.0 / 104.2 | 47.3 / 107.4 | 1.7 / 4.5 | 0.1 | 53.7 | 5.02 | 5.44 (7/10) | 0 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 47.0 / 104.2 | 47.3 / 107.4 | 1.7 / 4.5 | 0.1 | 53.7 | 5.02 | 5.44 (7/10) | 0 | 0.03 |
| BLine (tuned) | 40.0 / 101.8 | 40.9 / 104.5 | 3.8 / 5.8 | 0.1 | 0.0 | 4.33 (9/10) | never | 0 | 0.03 |
| BLine (tuned, with corner caps) | 40.6 / 103.4 | 41.5 / 106.6 | 3.3 / 5.3 | 0.1 | 0.0 | 4.14 (9/10) | never | 0 | 0.03 |
| PathPlanner (kP 5) | 13.5 / 24.1 | 13.5 / 24.6 | 49.7 / 55.1 | 0.1 | 0.0 | never | never | 4220 | 3.13 |
| PathPlanner (kP 5) + hold | 13.5 / 24.1 | 13.5 / 24.6 | 52.7 / 57.8 | 0.1 | 0.0 | never | never | 4220 | 4.15 |
| PathPlanner (tuned) + hold | 1.6 / 7.1 | 1.6 / 7.1 | 2.3 / 4.1 | 0.1 | 0.0 | 3.16 | 3.28 (4/10) | 77 | 4.13 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.8 / 7.0 | 1.9 / 6.9 | 5.7 / 6.8 | 0.1 | 0.0 | 2.65 | never | 3500 | 2.70 |
| DriveToPose chain | 2.1 / 9.2 | 1.6 / 6.2 | 1.7 / 3.8 | 0.0 | 0.0 | 7.61 | 7.81 (8/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 2.4 / 8.8 | 2.3 / 8.7 | 1.9 / 4.0 | 0.1 | 0.4 | 2.88 | 3.07 (7/10) | 258 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 3.1 / 10.2 | 3.0 / 10.1 | 3.1 / 4.6 | 0.1 | 0.0 | 3.02 | never | 19 | 2.73 |
| Ours v2 (as shipped) | 2.0 / 8.2 | 1.8 / 8.2 | 3.0 / 4.6 | 0.1 | 0.0 | 2.87 | never | 18 | 0.59 |
| Ours v2 (as shipped, adaptive grip off) | 2.0 / 8.2 | 1.8 / 8.2 | 3.0 / 4.6 | 0.1 | 0.0 | 2.87 | never | 18 | 0.62 |
| Ours v2 (tuned) | 2.0 / 8.2 | 1.8 / 8.2 | 3.0 / 4.6 | 0.1 | 0.0 | 2.87 | never | 18 | 0.55 |
| PathPlanner path2 (new-path, pre-release), as shipped | 22.8 / 47.6 | 23.1 / 50.6 | 53.6 / 63.7 | 0.1 | 0.0 | never | never | 7074 | 0.03 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 31.0 / 61.0 | 31.3 / 62.5 | 64.6 / 69.1 | 0.1 | 0.0 | never | never | 6966 | 0.02 |

**slalom**: through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m (10.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.2 / 13.3 | 5.2 / 13.3 | 6.5 / 15.7 | 0.1 | 15.4 | 3.09 (9/10) | 3.26 (1/10) | 146 | 81.30 |
| BLine (README gains, r 0.30) | 37.0 / 82.5 | 36.2 / 78.7 | 28.7 / 43.0 | 0.1 | 42.2 | 4.06 | 4.61 (1/10) | 778 | 0.03 |
| BLine (README gains, r 0.45 default) | 29.7 / 69.0 | 29.8 / 64.4 | 28.8 / 40.4 | 0.1 | 38.8 | 3.85 | never | 762 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 6.6 / 14.1 | 2.6 / 10.5 | 9.6 / 20.8 | 0.1 | 20.6 | 3.30 (9/10) | never | 437 | 0.02 |
| BLine (tuned) | 10.3 / 28.4 | 7.1 / 25.6 | 5.6 / 14.8 | 0.1 | 11.2 | 3.10 (8/10) | 3.65 (2/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 4.3 / 10.4 | 2.4 / 9.3 | 5.7 / 14.8 | 0.1 | 11.2 | 3.78 (8/10) | 4.33 (2/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 4.1 / 11.8 | 4.1 / 11.8 | 7.3 / 16.1 | 0.1 | 16.1 | 2.46 (9/10) | never | 3810 | 4.04 |
| PathPlanner (kP 5) + hold | 4.1 / 11.8 | 4.1 / 11.8 | 5.9 / 13.1 | 0.1 | 16.1 | 2.46 (9/10) | 2.92 (2/10) | 3810 | 4.28 |
| PathPlanner (tuned) + hold | 1.5 / 5.0 | 1.4 / 5.0 | 5.9 / 13.7 | 0.1 | 13.5 | 3.07 (8/10) | 3.39 (3/10) | 581 | 3.94 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.9 / 6.9 | 1.8 / 6.9 | 5.8 / 14.0 | 0.1 | 16.8 | 2.55 (9/10) | 3.02 (4/10) | 3657 | 4.20 |
| DriveToPose chain | 4.2 / 11.3 | 1.2 / 4.0 | 5.4 / 14.0 | 0.0 | 12.1 | 4.68 (8/10) | 4.84 (3/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 8.5 / 22.1 | 2.7 / 8.9 | 5.8 / 13.7 | 0.1 | 14.1 | 2.79 (6/10) | 3.00 (4/10) | 196 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.7 / 23.3 | 3.6 / 9.6 | 6.0 / 14.1 | 0.3 | 11.7 | 2.89 (7/10) | 3.10 (1/10) | 22 | 2.87 |
| Ours v2 (as shipped) | 1.6 / 7.4 | 1.6 / 6.9 | 5.6 / 14.1 | 0.1 | 12.1 | 2.79 (8/10) | 2.93 (3/10) | 16 | 0.45 |
| Ours v2 (as shipped, adaptive grip off) | 1.6 / 7.4 | 1.6 / 6.9 | 5.6 / 14.1 | 0.1 | 12.1 | 2.79 (8/10) | 2.93 (3/10) | 16 | 0.45 |
| Ours v2 (tuned) | 1.6 / 7.4 | 1.6 / 6.9 | 5.6 / 14.1 | 0.1 | 12.1 | 2.79 (8/10) | 2.93 (3/10) | 16 | 0.48 |
| PathPlanner path2 (new-path, pre-release), as shipped | 16.6 / 38.5 | 18.2 / 44.7 | 8.9 / 14.0 | 0.1 | 13.0 | 2.40 (3/10) | never | 7892 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 7.7 / 19.5 | 4.5 / 17.6 | 5.7 / 15.5 | 0.1 | 10.1 | 3.14 (8/10) | 3.57 (1/10) | 2513 | 0.02 |

**long_auto**: auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end (14.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 6.1 / 25.4 | 6.1 / 25.4 | 8.4 / 18.9 | 3.0 | 5.8 | 4.11 (4/10) | never | 452 | 112.30 |
| BLine (README gains, r 0.30) | 33.0 / 99.5 | 37.3 / 111.8 | 17.8 / 37.7 | 1.8 | 62.9 | 5.73 (5/10) | 6.62 (1/10) | 465 | 0.03 |
| BLine (README gains, r 0.45 default) | 31.7 / 90.8 | 34.8 / 101.9 | 8.7 / 20.0 | 1.2 | 51.8 | 5.71 (4/10) | 5.98 (1/10) | 149 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 10.8 / 34.0 | 4.8 / 24.1 | 8.7 / 19.1 | 2.4 | 5.9 | 4.93 (4/10) | never | 387 | 0.03 |
| BLine (tuned) | 23.4 / 70.3 | 25.8 / 79.9 | 8.5 / 20.0 | 1.5 | 0.9 | 3.93 (4/10) | never | 27 | 0.03 |
| BLine (tuned, with corner caps) | 10.0 / 32.8 | 5.9 / 22.6 | 8.1 / 19.0 | 1.2 | 1.6 | 5.29 (4/10) | 5.72 (2/10) | 0 | 0.03 |
| PathPlanner (kP 5) | 5.8 / 21.7 | 5.8 / 21.7 | 9.8 / 22.1 | 0.9 | 1.1 | 3.74 (4/10) | 3.80 (1/10) | 2330 | 7.68 |
| PathPlanner (kP 5) + hold | 5.8 / 21.7 | 5.8 / 21.7 | 11.6 / 22.9 | 0.9 | 1.1 | 3.74 (4/10) | never | 2330 | 7.31 |
| PathPlanner (tuned) + hold | 4.7 / 19.3 | 4.7 / 19.3 | 8.2 / 19.2 | 0.9 | 4.3 | 4.17 (4/10) | 4.45 (3/10) | 13 | 7.37 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 4.8 / 20.8 | 4.8 / 20.8 | 8.7 / 20.7 | 0.9 | 5.8 | 3.72 (4/10) | 4.15 (1/10) | 1715 | 6.95 |
| DriveToPose chain | 8.1 / 31.5 | 4.6 / 18.9 | 8.0 / 17.8 | 0.9 | 5.0 | 6.54 (4/10) | 6.64 (3/10) | 2 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 9.2 / 27.8 | 5.3 / 20.5 | 8.0 / 18.7 | 1.8 | 5.7 | 3.76 (4/10) | 3.98 (2/10) | 182 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 9.8 / 29.1 | 6.1 / 22.8 | 8.3 / 19.3 | 1.0 | 3.2 | 3.85 (4/10) | 4.09 (2/10) | 176 | 3.83 |
| Ours v2 (as shipped) | 6.6 / 21.7 | 4.9 / 19.7 | 8.2 / 19.6 | 0.9 | 2.9 | 3.87 (4/10) | 4.00 (2/10) | 18 | 0.52 |
| Ours v2 (as shipped, adaptive grip off) | 6.6 / 21.7 | 4.9 / 19.7 | 8.2 / 19.6 | 0.9 | 2.9 | 3.87 (4/10) | 4.00 (2/10) | 18 | 0.56 |
| Ours v2 (tuned) | 6.6 / 21.7 | 4.9 / 19.7 | 8.2 / 19.6 | 0.9 | 2.9 | 3.87 (4/10) | 4.00 (2/10) | 18 | 0.54 |
| PathPlanner path2 (new-path, pre-release), as shipped | 11.7 / 44.8 | 15.8 / 66.3 | 10.8 / 24.6 | 0.9 | 2.6 | 3.80 (2/10) | never | 4936 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 10.1 / 34.0 | 11.8 / 51.3 | 11.1 / 24.7 | 0.9 | 1.4 | 4.61 (1/10) | 4.95 (1/10) | 3049 | 0.02 |

**moving_start**: starts moving 2 m/s sideways (+y), path heads +x then curves left (7.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 13.0 / 27.7 | 13.0 / 27.7 | 5.9 / 11.3 | 0.1 | 10.7 | 1.88 (9/10) | never | 1544 | 39.14 |
| BLine (README gains, r 0.30) | 30.8 / 59.4 | 31.9 / 66.2 | 29.4 / 35.3 | 0.1 | 35.2 | 3.44 | never | 840 | 0.02 |
| BLine (README gains, r 0.45 default) | 33.1 / 54.8 | 34.2 / 51.5 | 23.7 / 39.3 | 0.1 | 55.8 | 2.14 | never | 587 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 24.3 / 49.6 | 24.9 / 47.1 | 11.0 / 16.4 | 0.1 | 15.9 | 2.17 | never | 510 | 0.03 |
| BLine (tuned) | 34.9 / 56.2 | 33.7 / 52.2 | 3.7 / 8.5 | 0.1 | 6.0 | 2.09 (9/10) | 2.37 (4/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 18.7 / 47.6 | 19.5 / 45.6 | 3.7 / 9.5 | 0.1 | 5.0 | 3.07 (8/10) | 3.56 (3/10) | 0 | 0.03 |
| PathPlanner (kP 5) | 46.8 / 60.7 | 46.8 / 60.7 | 56.8 / 59.8 | 0.1 | 12.0 | never | never | 2514 | 2.72 |
| PathPlanner (kP 5) + hold | 46.8 / 60.7 | 46.8 / 60.7 | 56.5 / 59.9 | 0.1 | 12.0 | never | never | 2514 | 2.54 |
| PathPlanner (tuned) + hold | 45.4 / 57.7 | 45.3 / 57.6 | 54.2 / 56.5 | 0.1 | 16.6 | never | never | 1656 | 2.42 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 46.8 / 60.7 | 46.8 / 60.6 | 56.5 / 59.9 | 0.1 | 12.1 | never | never | 2506 | 1.89 |
| DriveToPose chain | 21.1 / 58.7 | 19.5 / 54.8 | 3.8 / 8.3 | 0.0 | 7.6 | 3.65 (9/10) | 3.86 (4/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 20.4 / 27.0 | 21.8 / 26.8 | 27.3 / 30.6 | 0.0 | 28.4 | never | never | 1412 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 11.3 / 28.1 | 11.7 / 29.5 | 3.8 / 9.6 | 0.3 | 6.2 | 1.82 (8/10) | 2.09 (2/10) | 1922 | 1.37 |
| Ours v2 (as shipped) | 11.2 / 28.6 | 11.4 / 28.6 | 3.6 / 8.7 | 0.1 | 5.9 | 1.86 (8/10) | 1.96 (3/10) | 1464 | 0.41 |
| Ours v2 (as shipped, adaptive grip off) | 11.3 / 28.1 | 11.5 / 28.1 | 3.9 / 9.6 | 0.1 | 6.5 | 1.80 (8/10) | 1.95 (2/10) | 1804 | 0.40 |
| Ours v2 (tuned) | 11.2 / 28.6 | 11.4 / 28.6 | 3.6 / 8.7 | 0.1 | 5.9 | 1.86 (8/10) | 1.96 (3/10) | 1464 | 0.43 |
| PathPlanner path2 (new-path, pre-release), as shipped | 19.3 / 38.6 | 17.2 / 36.2 | 11.0 / 15.6 | 0.1 | 3.1 | never | never | 4998 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 20.6 / 38.5 | 18.2 / 36.2 | 13.3 / 18.5 | 0.1 | 3.2 | never | never | 3407 | 0.02 |

**hit_mid_path**: the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 31.2 / 43.4 | 31.2 / 43.4 | 41.8 / 48.6 | 0.1 | 1.7 | never | never | 2182 | 47.29 |
| BLine (README gains, r 0.30) | 32.0 / 80.7 | 33.8 / 81.9 | 55.6 / 65.4 | 0.1 | 72.7 | never | never | 2989 | 0.03 |
| BLine (README gains, r 0.45 default) | 25.2 / 64.9 | 26.9 / 65.8 | 49.1 / 55.4 | 0.1 | 70.1 | never | never | 2838 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 10.3 / 21.4 | 11.7 / 22.9 | 20.1 / 23.2 | 0.1 | 8.6 | never | never | 2313 | 0.03 |
| BLine (tuned) | 38.7 / 44.5 | 39.4 / 44.9 | 46.0 / 48.8 | 0.1 | 0.0 | never | never | 2373 | 0.02 |
| BLine (tuned, with corner caps) | 19.9 / 26.0 | 20.9 / 31.7 | 27.0 / 32.4 | 0.1 | 0.0 | never | never | 1992 | 0.03 |
| PathPlanner (kP 5) | 41.6 / 51.7 | 41.6 / 51.7 | 51.8 / 53.6 | 0.1 | 1.3 | never | never | 3722 | 2.82 |
| PathPlanner (kP 5) + hold | 41.5 / 51.7 | 41.5 / 51.7 | 52.4 / 54.9 | 0.1 | 1.3 | never | never | 3722 | 3.20 |
| PathPlanner (tuned) + hold | 30.1 / 41.5 | 30.1 / 41.5 | 41.3 / 42.3 | 0.1 | 3.3 | never | never | 2408 | 3.25 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 40.2 / 50.1 | 40.1 / 50.1 | 50.7 / 52.8 | 0.1 | 1.0 | never | never | 3491 | 3.26 |
| DriveToPose chain | 15.0 / 29.8 | 16.8 / 32.8 | 33.3 / 41.6 | 0.0 | 0.0 | never | never | 1931 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 87.5 / 100.7 | 83.6 / 97.4 | 103.1 / 106.2 | 0.1 | 0.0 | never | never | 3316 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 84.1 / 99.5 | 80.1 / 96.7 | 98.8 / 105.9 | 0.2 | 0.0 | never | never | 3212 | 1.82 |
| Ours v2 (as shipped) | 32.2 / 39.2 | 32.4 / 39.4 | 39.6 / 44.2 | 0.1 | 0.0 | never | never | 2403 | 0.44 |
| Ours v2 (as shipped, adaptive grip off) | 34.1 / 41.0 | 34.3 / 41.5 | 41.6 / 48.2 | 0.1 | 0.0 | never | never | 2427 | 0.44 |
| Ours v2 (tuned) | 32.2 / 39.2 | 32.4 / 39.4 | 39.6 / 44.2 | 0.1 | 0.0 | never | never | 2403 | 0.42 |
| PathPlanner path2 (new-path, pre-release), as shipped | 44.9 / 53.6 | 45.5 / 55.7 | 52.7 / 54.1 | 0.1 | 0.0 | never | never | 6219 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 42.7 / 53.0 | 43.6 / 53.3 | 52.4 / 56.3 | 0.1 | 0.4 | never | never | 3806 | 0.02 |

**low_grip_s_curve**: the S-curve on mu 0.70 static / 0.61 kinetic (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 11.6 | 4.6 / 11.6 | 12.0 / 20.2 | 0.1 | 20.0 | 2.23 | never | 4626 | 57.42 |
| BLine (README gains, r 0.30) | 12.2 / 36.6 | 13.4 / 36.6 | 6.4 / 15.6 | 0.1 | 97.2 | 3.06 (8/10) | never | 11148 | 0.02 |
| BLine (README gains, r 0.45 default) | 7.5 / 25.4 | 8.3 / 25.5 | 5.0 / 13.0 | 0.1 | 92.1 | 3.01 (8/10) | 3.14 (1/10) | 10434 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 9.7 | 1.8 / 7.5 | 19.9 / 28.8 | 0.1 | 28.6 | 2.23 | never | 5260 | 0.03 |
| BLine (tuned) | 7.1 / 17.7 | 5.0 / 16.9 | 5.0 / 14.0 | 0.1 | 5.6 | 2.77 (7/10) | 3.19 (3/10) | 5220 | 0.02 |
| BLine (tuned, with corner caps) | 2.7 / 6.0 | 2.9 / 10.5 | 4.3 / 12.1 | 0.1 | 6.9 | 2.77 (8/10) | 3.26 (2/10) | 6199 | 0.02 |
| PathPlanner (kP 5) | 10.1 / 19.8 | 10.0 / 19.7 | 30.5 / 39.4 | 0.1 | 39.3 | 1.83 (8/10) | never | 6984 | 2.54 |
| PathPlanner (kP 5) + hold | 10.1 / 19.8 | 10.0 / 19.7 | 27.4 / 36.5 | 0.1 | 39.2 | 1.83 (8/10) | never | 7102 | 2.95 |
| PathPlanner (tuned) + hold | 1.4 / 4.2 | 1.4 / 4.2 | 4.4 / 10.0 | 0.1 | 9.3 | 2.22 (9/10) | 2.43 (4/10) | 6278 | 2.93 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 7.7 / 15.8 | 7.6 / 15.6 | 16.1 / 24.8 | 0.1 | 27.5 | 1.89 (9/10) | never | 7003 | 3.15 |
| DriveToPose chain | 3.1 / 9.6 | 1.1 / 4.1 | 4.8 / 13.2 | 0.0 | 4.8 | 4.00 (7/10) | 4.13 (3/10) | 12394 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 4.5 / 13.9 | 3.2 / 9.0 | 10.7 / 19.3 | 0.1 | 19.0 | 1.94 (9/10) | never | 8298 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.5 / 12.1 | 4.8 / 12.9 | 7.2 / 14.9 | 0.0 | 4.7 | 2.03 (4/10) | never | 8080 | 1.73 |
| Ours v2 (as shipped) | 1.6 / 6.4 | 1.5 / 5.3 | 4.0 / 10.0 | 0.1 | 8.8 | 2.40 (8/10) | 2.62 (4/10) | 998 | 0.41 |
| Ours v2 (as shipped, adaptive grip off) | 4.1 / 11.4 | 4.0 / 10.3 | 12.1 / 23.6 | 0.1 | 0.0 | 2.11 (1/10) | never | 7330 | 0.43 |
| Ours v2 (tuned) | 1.6 / 6.4 | 1.5 / 5.3 | 4.0 / 10.0 | 0.1 | 8.8 | 2.40 (8/10) | 2.62 (4/10) | 998 | 0.46 |
| PathPlanner path2 (new-path, pre-release), as shipped | 6.6 / 16.3 | 4.1 / 15.2 | 6.6 / 13.1 | 0.1 | 13.0 | 1.92 | never | 7070 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 11.0 / 23.2 | 9.3 / 22.2 | 7.7 / 11.9 | 0.1 | 11.5 | 2.44 (3/10) | never | 6232 | 0.02 |

**grip_drop_s_curve**: the S-curve; halfway along, the floor drops from mu 1.15 / 1.0 to 0.70 / 0.61 (a worn carpet seam) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.5 / 11.1 | 4.5 / 11.1 | 12.3 / 19.4 | 0.1 | 19.7 | 2.22 | never | 3247 | 54.46 |
| BLine (README gains, r 0.30) | 13.3 / 38.7 | 14.4 / 38.7 | 6.9 / 13.2 | 0.1 | 102.4 | 3.07 (6/10) | never | 7046 | 0.03 |
| BLine (README gains, r 0.45 default) | 8.5 / 28.3 | 9.4 / 28.4 | 6.9 / 15.1 | 0.1 | 97.3 | 3.02 (6/10) | never | 6726 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.2 / 8.8 | 1.7 / 6.7 | 22.9 / 31.9 | 0.1 | 31.7 | 2.21 | never | 2590 | 0.02 |
| BLine (tuned) | 6.8 / 16.3 | 4.6 / 15.5 | 5.2 / 12.2 | 0.1 | 11.6 | 2.48 (9/10) | 3.15 (1/10) | 2081 | 0.02 |
| BLine (tuned, with corner caps) | 3.0 / 7.6 | 3.5 / 11.7 | 4.9 / 11.8 | 0.1 | 11.1 | 2.59 (9/10) | 3.24 (2/10) | 2574 | 0.03 |
| PathPlanner (kP 5) | 5.9 / 22.1 | 5.8 / 22.1 | 43.3 / 54.4 | 0.1 | 50.2 | never | never | 4194 | 3.15 |
| PathPlanner (kP 5) + hold | 5.9 / 22.1 | 5.8 / 22.1 | 40.4 / 51.7 | 0.1 | 50.0 | never | never | 4313 | 3.14 |
| PathPlanner (tuned) + hold | 1.8 / 8.0 | 1.8 / 8.0 | 6.9 / 15.2 | 0.1 | 13.6 | 2.21 (6/10) | never | 3023 | 2.85 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 5.8 / 20.1 | 5.8 / 20.1 | 28.9 / 39.7 | 0.1 | 38.0 | never | never | 4173 | 3.16 |
| DriveToPose chain | 2.9 / 8.5 | 0.9 / 3.1 | 3.9 / 10.5 | 0.0 | 6.7 | 4.03 (8/10) | 4.16 (2/10) | 6102 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 8.3 / 21.9 | 5.8 / 19.3 | 20.4 / 32.1 | 0.1 | 29.1 | never | never | 4118 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.1 / 17.4 | 5.5 / 13.6 | 13.7 / 19.1 | 0.0 | 16.6 | never | never | 3908 | 2.13 |
| Ours v2 (as shipped) | 2.2 / 6.5 | 2.4 / 6.5 | 7.9 / 13.5 | 0.1 | 12.0 | 2.06 (3/10) | never | 2630 | 0.41 |
| Ours v2 (as shipped, adaptive grip off) | 1.6 / 6.1 | 1.7 / 6.1 | 6.8 / 13.4 | 0.1 | 12.3 | 2.03 (7/10) | never | 3616 | 0.44 |
| Ours v2 (tuned) | 2.2 / 6.5 | 2.4 / 6.5 | 7.9 / 13.5 | 0.1 | 12.0 | 2.06 (3/10) | never | 2630 | 0.43 |
| PathPlanner path2 (new-path, pre-release), as shipped | 8.6 / 30.1 | 9.7 / 30.1 | 61.2 / 68.6 | 0.1 | 61.3 | never | never | 6054 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 7.2 / 12.9 | 6.5 / 12.9 | 16.4 / 24.5 | 0.1 | 20.7 | never | never | 3556 | 0.02 |

**spin_while_driving**: 5 m straight while the robot turns a full 360 deg (a heading target at every waypoint) (8.2 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 2.1 / 6.9 | 2.1 / 6.9 | 5.9 / 12.0 | 14.2 | 11.8 | 2.20 (8/10) | never | 654 | 50.06 |
| BLine (README gains, r 0.30) | 2.8 / 7.7 | 2.8 / 7.7 | 5.9 / 12.5 | 3.1 | 63.4 | 1.93 (8/10) | never | 402 | 0.03 |
| BLine (README gains, r 0.45 default) | 3.2 / 7.7 | 3.2 / 7.7 | 5.9 / 12.6 | 3.1 | 63.6 | 1.94 (8/10) | never | 420 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.2 / 7.7 | 3.2 / 7.7 | 5.9 / 12.6 | 3.1 | 63.6 | 1.94 (8/10) | never | 420 | 0.03 |
| BLine (tuned) | 3.6 / 8.2 | 3.6 / 8.2 | 4.6 / 11.2 | 3.1 | 8.4 | 2.78 (8/10) | never | 34 | 0.03 |
| BLine (tuned, with corner caps) | 2.3 / 5.1 | 2.3 / 5.1 | 4.6 / 11.3 | 3.1 | 8.4 | 2.77 (8/10) | never | 38 | 0.03 |
| PathPlanner (kP 5) | 2.2 / 7.9 | 2.2 / 7.9 | 6.0 / 11.9 | 1.7 | 11.4 | 2.93 (8/10) | never | 108 | 3.25 |
| PathPlanner (kP 5) + hold | 2.2 / 7.9 | 2.2 / 7.9 | 5.1 / 12.0 | 1.8 | 11.3 | 2.93 (8/10) | 3.35 (3/10) | 108 | 3.20 |
| PathPlanner (tuned) + hold | 1.6 / 7.2 | 1.6 / 7.2 | 5.1 / 11.5 | 1.8 | 9.9 | 3.47 (8/10) | 3.72 (3/10) | 11 | 3.01 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 2.2 / 7.9 | 2.2 / 7.9 | 5.1 / 12.1 | 1.8 | 11.4 | 2.94 (8/10) | 3.36 (3/10) | 84 | 3.20 |
| DriveToPose chain | 1.5 / 7.9 | 1.5 / 7.9 | 5.0 / 10.8 | 3.4 | 9.6 | 4.85 (8/10) | never | 279 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 3.1 / 7.9 | 2.4 / 7.7 | 5.5 / 11.3 | 5.2 | 11.2 | 1.94 (8/10) | never | 794 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.4 / 11.8 | 4.8 / 11.6 | 5.0 / 12.1 | 3.4 | 8.8 | 2.08 (8/10) | never | 300 | 2.10 |
| Ours v2 (as shipped) | 2.9 / 8.3 | 2.9 / 8.3 | 5.1 / 11.9 | 2.0 | 8.9 | 2.55 (8/10) | 2.84 (2/10) | 140 | 0.48 |
| Ours v2 (as shipped, adaptive grip off) | 2.9 / 8.3 | 2.9 / 8.3 | 5.1 / 11.9 | 2.0 | 8.9 | 2.55 (8/10) | 2.84 (2/10) | 140 | 0.47 |
| Ours v2 (tuned) | 2.9 / 8.3 | 2.9 / 8.3 | 5.1 / 11.9 | 2.0 | 8.9 | 2.55 (8/10) | 2.84 (2/10) | 140 | 0.49 |
| PathPlanner path2 (new-path, pre-release), as shipped | 2.0 / 6.5 | 2.0 / 6.5 | 11.0 / 20.1 | 2.9 | 0.0 | 2.22 (1/10) | never | 2065 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 3.3 / 9.3 | 3.3 / 9.3 | 6.4 / 13.6 | 1.8 | 5.0 | 2.73 (3/10) | 3.22 (1/10) | 1805 | 0.02 |

**figure_eight**: a self-crossing loop (a lemniscate, 4 m x 1.8 m) through the origin twice; heading held (12.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.5 / 15.4 | 5.5 / 15.4 | 1.9 / 4.1 | 0.1 | 3.4 | 5.12 | 5.31 (7/10) | 679 | 105.56 |
| BLine (README gains, r 0.30) | 43.7 / 117.5 | 44.1 / 117.6 | 2.2 / 3.9 | 0.1 | 58.3 | 9.88 | 9.98 (6/10) | 7 | 0.05 |
| BLine (README gains, r 0.45 default) | 50.1 / 117.1 | 51.0 / 117.9 | 4.0 / 5.3 | 0.1 | 67.8 | 6.59 (9/10) | never | 0 | 0.05 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 40.2 / 90.9 | 41.2 / 93.2 | 3.2 / 4.4 | 0.1 | 48.8 | 6.05 | never | 96 | 0.05 |
| BLine (tuned) | 42.2 / 102.5 | 43.0 / 104.9 | 1.8 / 3.0 | 0.1 | 0.5 | 4.99 | 5.33 (8/10) | 0 | 0.05 |
| BLine (tuned, with corner caps) | 43.4 / 96.4 | 44.4 / 98.2 | 2.7 / 4.2 | 0.1 | 0.0 | 6.00 | never | 0 | 0.05 |
| PathPlanner (kP 5) | 43.7 / 52.6 | 43.7 / 52.5 | 52.6 / 55.0 | 0.1 | 80.6 | never | never | 8634 | 6.33 |
| PathPlanner (kP 5) + hold | 43.6 / 52.6 | 43.5 / 52.5 | 53.6 / 56.0 | 0.1 | 80.6 | never | never | 8634 | 6.43 |
| PathPlanner (tuned) + hold | 1.8 / 9.2 | 1.7 / 9.1 | 2.0 / 3.0 | 0.1 | 1.7 | 4.45 | 4.70 (6/10) | 250 | 6.41 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 4.1 / 12.0 | 4.1 / 12.1 | 11.8 / 13.1 | 0.1 | 4.4 | never | never | 8291 | 6.90 |
| DriveToPose chain | 2.1 / 10.6 | 1.5 / 7.7 | 20.1 / 44.7 | 0.0 | 80.6 | 12.25 (3/10) | never | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 2.5 / 8.7 | 2.2 / 8.7 | 1.5 / 3.7 | 0.1 | 2.8 | 4.30 | 4.50 (8/10) | 119 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 3.3 / 10.5 | 3.0 / 10.5 | 1.6 / 2.6 | 0.3 | 0.9 | 4.42 | 4.66 (8/10) | 403 | 4.51 |
| Ours v2 (as shipped) | 2.1 / 9.8 | 2.1 / 9.5 | 1.9 / 2.9 | 0.1 | 0.4 | 4.38 | 4.50 (7/10) | 18 | 0.82 |
| Ours v2 (as shipped, adaptive grip off) | 2.1 / 9.8 | 2.1 / 9.5 | 1.9 / 2.9 | 0.1 | 0.4 | 4.38 | 4.50 (7/10) | 18 | 0.75 |
| Ours v2 (tuned) | 2.1 / 9.8 | 2.1 / 9.5 | 1.9 / 2.9 | 0.1 | 0.4 | 4.38 | 4.50 (7/10) | 18 | 0.73 |
| PathPlanner path2 (new-path, pre-release), as shipped | 31.1 / 77.2 | 31.7 / 77.8 | 23.5 / 45.0 | 0.1 | 44.2 | 4.14 (1/10) | never | 13012 | 0.03 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 47.8 / 73.0 | 47.0 / 71.1 | 70.5 / 75.1 | 0.1 | 80.6 | never | never | 10123 | 0.02 |

**wall_hug**: along a wall with the bumpers 15 cm off it, then a 90 deg turn to run 15 cm off the next wall (11.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.1 / 22.8 | 5.0 / 21.8 | 6.0 / 14.0 | 0.1 | 7.7 | 3.05 (4/10) | 3.23 (3/10) | 417 | 83.43 |
| BLine (README gains, r 0.30) | 40.1 / 103.8 | 39.9 / 103.8 | 6.1 / 14.7 | 0.1 | 68.8 | 5.42 (6/10) | 5.70 (3/10) | 91 | 0.04 |
| BLine (README gains, r 0.45 default) | 38.8 / 99.7 | 38.9 / 99.7 | 5.9 / 14.5 | 0.1 | 61.7 | 5.27 (6/10) | 5.56 (3/10) | 6 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 38.8 / 99.7 | 38.9 / 99.7 | 5.9 / 14.5 | 0.1 | 61.7 | 5.27 (6/10) | 5.56 (3/10) | 6 | 0.03 |
| BLine (tuned) | 30.3 / 79.0 | 30.4 / 79.0 | 5.9 / 13.5 | 0.1 | 5.0 | 2.98 (6/10) | 3.52 (3/10) | 10 | 0.03 |
| BLine (tuned, with corner caps) | 40.5 / 102.6 | 40.7 / 102.6 | 5.3 / 12.7 | 0.1 | 8.8 | 3.84 (6/10) | 4.36 (4/10) | 0 | 0.03 |
| PathPlanner (kP 5) | 11.5 / 24.9 | 11.5 / 24.9 | 17.4 / 25.4 | 0.1 | 0.0 | never | never | 2216 | 4.56 |
| PathPlanner (kP 5) + hold | 11.5 / 24.9 | 11.5 / 24.9 | 19.3 / 26.1 | 0.1 | 0.0 | never | never | 2216 | 4.49 |
| PathPlanner (tuned) + hold | 3.2 / 12.3 | 3.1 / 12.1 | 5.9 / 13.6 | 0.1 | 4.8 | 3.24 (4/10) | 3.37 (1/10) | 28 | 4.78 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 3.4 / 13.8 | 3.4 / 13.8 | 7.5 / 14.0 | 0.1 | 5.2 | 2.88 (5/10) | never | 1584 | 4.89 |
| DriveToPose chain | 3.3 / 15.0 | 3.0 / 12.1 | 5.2 / 13.5 | 0.0 | 5.6 | 7.16 (7/10) | 7.30 (3/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 3.4 / 14.7 | 3.1 / 12.7 | 5.3 / 13.0 | 0.1 | 7.0 | 2.96 (4/10) | 3.17 (4/10) | 169 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 4.1 / 16.3 | 3.9 / 14.3 | 5.7 / 13.0 | 0.2 | 4.4 | 3.05 (4/10) | 3.26 (3/10) | 23 | 2.91 |
| Ours v2 (as shipped) | 3.3 / 13.4 | 3.2 / 13.3 | 5.6 / 12.9 | 0.1 | 4.6 | 2.98 (4/10) | 3.10 (3/10) | 16 | 0.48 |
| Ours v2 (as shipped, adaptive grip off) | 3.3 / 13.4 | 3.2 / 13.3 | 5.6 / 12.9 | 0.1 | 4.6 | 2.98 (4/10) | 3.10 (3/10) | 16 | 0.52 |
| Ours v2 (tuned) | 3.3 / 13.4 | 3.2 / 13.3 | 5.6 / 12.9 | 0.1 | 4.6 | 2.98 (4/10) | 3.10 (3/10) | 16 | 0.48 |
| PathPlanner path2 (new-path, pre-release), as shipped | 11.4 / 38.8 | 11.6 / 38.8 | 13.7 / 24.2 | 0.1 | 4.2 | 2.57 (1/10) | never | 6101 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 26.3 / 45.2 | 26.3 / 45.2 | 31.0 / 40.2 | 0.1 | 0.0 | never | never | 4409 | 0.02 |

**tag_align_moving_goal**: 5 m to a scoring pose; the true goal is 8 cm sideways and 3 deg off the map pose, revealed 1.5 m out (8.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 3.0 / 8.1 | 3.0 / 8.1 | 1.4 / 3.4 | 0.3 | 14.4 | 2.20 | 2.43 (8/10) | 103 | 53.34 |
| BLine (README gains, r 0.30) | 3.4 / 7.4 | 4.5 / 13.7 | 1.3 / 2.9 | 0.2 | 86.8 | 2.81 | 3.01 (8/10) | 68 | 0.03 |
| BLine (README gains, r 0.45 default) | 3.3 / 7.4 | 2.3 / 8.6 | 1.1 / 2.8 | 0.1 | 86.7 | 2.80 | 2.99 (8/10) | 97 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.3 / 8.0 | 0.8 / 4.2 | 1.3 / 2.8 | 0.2 | 57.7 | 2.71 | 2.92 (8/10) | 47 | 0.03 |
| BLine (tuned) | 6.6 / 17.4 | 4.0 / 16.5 | 1.1 / 3.1 | 0.3 | 9.4 | 3.33 | 3.58 (7/10) | 8 | 0.03 |
| BLine (tuned, with corner caps) | 3.4 / 8.2 | 1.1 / 5.5 | 1.3 / 3.1 | 0.3 | 9.4 | 3.36 | 3.61 (7/10) | 15 | 0.03 |
| PathPlanner (kP 5) | 3.1 / 7.8 | 3.0 / 7.8 | 2.8 / 5.5 | 0.0 | 5.8 | 1.80 | 1.98 (2/10) | 15 | 3.00 |
| PathPlanner (kP 5) + hold | 3.1 / 7.8 | 3.0 / 7.8 | 1.4 / 3.0 | 0.0 | 5.8 | 1.80 | 2.25 (7/10) | 15 | 2.91 |
| PathPlanner (tuned) + hold | 3.1 / 7.8 | 3.1 / 7.8 | 2.1 / 3.9 | 0.1 | 4.1 | 1.94 | 2.19 (5/10) | 228 | 2.92 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 3.1 / 7.8 | 3.0 / 7.8 | 1.4 / 3.0 | 0.0 | 5.8 | 1.80 | 2.25 (7/10) | 16 | 2.71 |
| DriveToPose chain | 4.3 / 8.7 | 2.5 / 7.6 | 1.0 / 3.0 | 0.3 | 2.4 | 2.63 | 2.82 (8/10) | 0 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 1.1 / 5.7 | 1.2 / 5.5 | 1.2 / 3.2 | 0.3 | 12.5 | 2.14 | 2.38 (8/10) | 34 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.9 / 6.9 | 2.0 / 6.7 | 1.2 / 3.0 | 0.3 | 3.0 | 1.84 | 2.09 (8/10) | 291 | 2.03 |
| Ours v2 (as shipped) | 2.9 / 7.7 | 2.9 / 7.7 | 1.0 / 3.9 | 0.2 | 1.7 | 1.82 | 1.95 (9/10) | 16 | 0.42 |
| Ours v2 (as shipped, adaptive grip off) | 2.9 / 7.7 | 2.9 / 7.7 | 1.0 / 3.9 | 0.2 | 1.7 | 1.82 | 1.95 (9/10) | 16 | 0.39 |
| Ours v2 (tuned) | 2.9 / 7.7 | 2.9 / 7.7 | 1.0 / 3.9 | 0.2 | 1.7 | 1.82 | 1.95 (9/10) | 16 | 0.40 |
| PathPlanner path2 (new-path, pre-release), as shipped | 4.3 / 9.0 | 1.2 / 4.3 | 5.0 / 6.9 | 0.3 | 6.9 | 1.85 (9/10) | never | 3155 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 4.4 / 11.7 | 1.7 / 8.3 | 1.2 / 3.0 | 0.2 | 8.4 | 3.17 | 3.41 (7/10) | 374 | 0.02 |

**intake_drive_through**: drive through a game piece at 2 m/s; its detected position moves 10 cm sideways 2 m out (7.4 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.9 / 2.3 | 0.9 / 2.3 | 11.3 / 14.3 | 0.1 | 10.6 | never | never | 88 | 38.52 |
| BLine (README gains, r 0.30) | 3.6 / 9.9 | 3.4 / 9.9 | 39.4 / 44.7 | 0.1 | 68.7 | 1.39 | never | 1042 | 0.03 |
| BLine (README gains, r 0.45 default) | 3.7 / 9.8 | 3.5 / 9.9 | 39.5 / 44.8 | 0.1 | 68.7 | 1.39 | never | 1046 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.7 / 9.8 | 3.5 / 9.9 | 39.5 / 44.8 | 0.1 | 68.7 | 1.39 | never | 1046 | 0.02 |
| BLine (tuned) | 4.5 / 9.8 | 4.3 / 9.8 | 22.3 / 25.5 | 0.1 | 41.9 | 1.65 | never | 770 | 0.02 |
| BLine (tuned, with corner caps) | 4.2 / 9.9 | 4.1 / 9.9 | 22.3 / 25.5 | 0.1 | 41.9 | 1.65 | never | 778 | 0.03 |
| PathPlanner (kP 5) | 3.9 / 10.1 | 3.9 / 10.1 | 24.8 / 28.8 | 0.1 | 28.7 | 1.41 | never | 707 | 2.39 |
| PathPlanner (kP 5) + hold | 3.9 / 10.1 | 3.9 / 10.1 | 24.8 / 28.8 | 0.1 | 28.7 | 1.41 | never | 707 | 2.34 |
| PathPlanner (tuned) + hold | 3.9 / 10.0 | 3.9 / 10.0 | 21.9 / 26.0 | 0.1 | 26.0 | 1.51 | never | 830 | 2.24 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 3.9 / 10.1 | 3.9 / 10.1 | 24.8 / 28.8 | 0.1 | 28.7 | 1.41 | never | 707 | 2.40 |
| DriveToPose chain | 3.7 / 9.8 | 3.6 / 9.8 | 1.6 / 4.2 | 0.0 | 3.4 | 2.34 | 2.55 (7/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 0.3 / 0.7 | 0.3 / 0.8 | 26.9 / 34.1 | 0.3 | 32.6 | never | never | 675 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.7 / 8.9 | 1.7 / 9.0 | 1.7 / 4.1 | 0.2 | 4.1 | 1.60 | 1.85 (7/10) | 354 | 1.38 |
| Ours v2 (as shipped) | 4.0 / 9.9 | 4.0 / 9.9 | 24.0 / 27.7 | 0.1 | 27.7 | 1.40 | never | 742 | 0.38 |
| Ours v2 (as shipped, adaptive grip off) | 4.0 / 9.9 | 4.0 / 9.9 | 24.0 / 27.7 | 0.1 | 27.7 | 1.40 | never | 742 | 0.38 |
| Ours v2 (tuned) | 4.0 / 9.9 | 4.0 / 9.9 | 24.0 / 27.7 | 0.1 | 27.7 | 1.40 | never | 742 | 0.40 |
| PathPlanner path2 (new-path, pre-release), as shipped | 1.0 / 2.6 | 0.9 / 1.4 | 16.0 / 21.1 | 0.1 | 0.0 | never | never | 2661 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 1.1 / 4.9 | 1.4 / 5.2 | 10.8 / 14.2 | 0.1 | 5.3 | never | never | 562 | 0.02 |

**three_piece_chain**: three paths with rolling handoffs at 2 m/s (core-builder's neutral_three_piece, 2026 field) (15.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 7.7 / 29.8 | 5.4 / 22.7 | 5.1 / 13.1 | 3.8 | 12.3 | 5.63 (7/10) | never | 832 | 38.27 |
| BLine (README gains, r 0.30) | 54.0 / 223.3 | 52.1 / 220.2 | 136.6 / 244.5 | 0.5 | 252.3 | never | never | 4040 | 0.03 |
| BLine (README gains, r 0.45 default) | 54.0 / 143.7 | 51.7 / 136.6 | 122.1 / 133.7 | 0.5 | 199.8 | never | never | 4033 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 46.3 / 79.1 | 45.2 / 72.7 | 62.1 / 70.0 | 1.9 | 398.2 | never | never | 2768 | 0.03 |
| BLine (tuned) | 61.0 / 240.0 | 59.7 / 237.6 | 136.9 / 234.5 | 1.8 | 398.2 | never | never | 3522 | 0.03 |
| BLine (tuned, with corner caps) | 55.6 / 226.8 | 53.7 / 223.3 | 137.3 / 225.6 | 0.8 | 398.2 | never | never | 3294 | 0.03 |
| PathPlanner (kP 5) | 5.6 / 20.1 | 5.6 / 18.3 | 10.7 / 16.2 | 0.3 | 398.2 | never | never | 3732 | 2.23 |
| PathPlanner (kP 5) + hold | 5.7 / 20.1 | 5.6 / 18.3 | 12.4 / 17.0 | 0.3 | 398.2 | never | never | 3732 | 2.25 |
| PathPlanner (tuned) + hold | 3.8 / 14.5 | 3.6 / 14.5 | 6.4 / 12.3 | 0.3 | 398.2 | 5.47 (5/10) | 5.56 (1/10) | 991 | 2.11 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 4.4 / 15.9 | 4.4 / 14.9 | 7.0 / 11.7 | 0.3 | 398.2 | 4.76 (7/10) | never | 2670 | 2.14 |
| DriveToPose chain | 6.3 / 31.5 | 3.6 / 15.0 | 5.1 / 11.7 | 1.9 | 398.2 | 10.69 (7/10) | never | 434 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 7.5 / 23.5 | 3.8 / 14.3 | 5.2 / 12.5 | 2.3 | 398.2 | 4.46 (8/10) | never | 436 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 7.7 / 25.5 | 4.1 / 16.5 | 5.6 / 11.6 | 0.5 | 398.2 | 4.56 (6/10) | 4.78 (2/10) | 238 | 4.68 |
| Ours v2 (as shipped) | 4.2 / 17.7 | 3.9 / 16.9 | 5.7 / 11.4 | 0.3 | 398.2 | 4.91 (6/10) | 5.05 (2/10) | 611 | 0.48 |
| Ours v2 (as shipped, adaptive grip off) | 4.2 / 17.7 | 3.9 / 16.9 | 5.7 / 11.4 | 0.3 | 398.2 | 4.91 (6/10) | 5.03 (2/10) | 648 | 0.39 |
| Ours v2 (tuned) | 4.2 / 17.7 | 3.9 / 16.9 | 5.7 / 11.4 | 0.3 | 398.2 | 4.91 (6/10) | 5.05 (2/10) | 611 | 0.33 |
| PathPlanner path2 (new-path, pre-release), as shipped | 13.1 / 35.2 | 15.1 / 47.1 | 14.3 / 24.8 | 1.3 | 24.5 | 4.04 (1/10) | never | 8557 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 8.3 / 26.9 | 7.5 / 29.8 | 8.9 / 12.5 | 0.3 | 398.2 | never | never | 5747 | 0.02 |

**fast_long**: an 8 m gentle curve with a 5 m/s cap (above the robot's 4.57 m/s free speed) (10.8 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 1.9 / 5.7 | 1.9 / 5.7 | 7.1 / 17.4 | 0.1 | 17.0 | 2.65 (9/10) | 2.81 (2/10) | 86 | 53.33 |
| BLine (README gains, r 0.30) | 2.2 / 5.5 | 1.0 / 3.9 | 40.9 / 54.6 | 0.1 | 54.5 | 2.41 | never | 1050 | 0.02 |
| BLine (README gains, r 0.45 default) | 2.8 / 6.5 | 1.2 / 5.5 | 41.1 / 53.7 | 0.1 | 53.6 | 2.41 | never | 1046 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 2.8 / 6.5 | 1.2 / 5.5 | 41.1 / 53.7 | 0.1 | 53.6 | 2.41 | never | 1046 | 0.02 |
| BLine (tuned) | 3.8 / 9.2 | 2.5 / 8.9 | 6.5 / 16.8 | 0.1 | 12.8 | 3.38 (8/10) | 3.92 (1/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 2.2 / 6.1 | 1.4 / 4.6 | 6.5 / 16.8 | 0.1 | 12.8 | 3.38 (8/10) | 3.93 (1/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 1.1 / 4.2 | 1.1 / 4.2 | 7.0 / 17.0 | 0.1 | 16.8 | 2.55 (9/10) | 2.76 (3/10) | 0 | 2.66 |
| PathPlanner (kP 5) + hold | 1.1 / 4.2 | 1.1 / 4.2 | 6.4 / 16.0 | 0.1 | 16.8 | 2.55 (9/10) | 2.99 (3/10) | 0 | 2.69 |
| PathPlanner (tuned) + hold | 1.2 / 4.2 | 1.2 / 4.2 | 6.2 / 15.2 | 0.1 | 14.8 | 2.68 (8/10) | 2.86 (3/10) | 0 | 2.65 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.1 / 4.2 | 1.1 / 4.2 | 6.4 / 16.0 | 0.1 | 16.8 | 2.55 (9/10) | 2.99 (3/10) | 0 | 2.63 |
| DriveToPose chain | 2.2 / 6.4 | 1.1 / 4.0 | 6.2 / 16.0 | 0.0 | 13.7 | 4.87 (8/10) | 5.03 (3/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 1.4 / 4.3 | 1.4 / 4.4 | 7.0 / 18.6 | 0.0 | 19.0 | 2.36 (8/10) | 2.61 (4/10) | 272 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.6 / 4.3 | 1.6 / 5.0 | 6.4 / 16.1 | 0.0 | 13.4 | 2.59 (8/10) | 2.86 (3/10) | 24 | 1.54 |
| Ours v2 (as shipped) | 1.2 / 4.2 | 1.1 / 4.2 | 6.4 / 16.0 | 0.1 | 13.6 | 2.41 (8/10) | 2.55 (3/10) | 16 | 0.36 |
| Ours v2 (as shipped, adaptive grip off) | 1.2 / 4.2 | 1.1 / 4.2 | 6.4 / 16.0 | 0.1 | 13.6 | 2.41 (8/10) | 2.55 (3/10) | 16 | 0.38 |
| Ours v2 (tuned) | 1.2 / 4.2 | 1.1 / 4.2 | 6.4 / 16.0 | 0.1 | 13.6 | 2.41 (8/10) | 2.55 (3/10) | 16 | 0.37 |
| PathPlanner path2 (new-path, pre-release), as shipped | 4.3 / 9.3 | 3.7 / 7.3 | 12.1 / 25.5 | 0.1 | 1.1 | 2.40 (1/10) | 2.54 (1/10) | 2922 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 2.0 / 6.3 | 2.3 / 7.2 | 7.0 / 17.8 | 0.1 | 11.6 | 3.42 (8/10) | never | 1504 | 0.02 |

**heavy_robot**: the S-curve with a 68 kg robot (MOI 6.8) and a 96 A current limit (20 % lower) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.5 / 12.1 | 4.5 / 12.1 | 5.9 / 13.2 | 0.1 | 12.9 | 2.25 (9/10) | never | 174 | 35.33 |
| BLine (README gains, r 0.30) | 13.2 / 36.9 | 14.3 / 36.9 | 7.0 / 15.6 | 0.1 | 97.4 | 3.01 (8/10) | never | 360 | 0.02 |
| BLine (README gains, r 0.45 default) | 8.1 / 26.5 | 8.9 / 26.5 | 6.2 / 13.8 | 0.1 | 92.3 | 2.93 (9/10) | never | 294 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.3 / 10.4 | 1.7 / 7.7 | 13.1 / 22.2 | 0.1 | 22.0 | 2.22 | never | 539 | 0.02 |
| BLine (tuned) | 6.6 / 16.6 | 4.4 / 15.8 | 4.2 / 11.2 | 0.1 | 8.1 | 2.60 (8/10) | 3.14 (3/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 2.9 / 7.2 | 3.1 / 10.9 | 4.2 / 11.1 | 0.1 | 8.1 | 2.70 (8/10) | 3.25 (3/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 1.6 / 3.9 | 1.6 / 3.8 | 6.0 / 13.5 | 0.1 | 13.1 | 1.87 (9/10) | 2.09 (1/10) | 1642 | 1.85 |
| PathPlanner (kP 5) + hold | 1.6 / 3.9 | 1.6 / 3.8 | 4.3 / 10.0 | 0.1 | 13.1 | 1.87 (9/10) | 2.36 (4/10) | 1642 | 1.82 |
| PathPlanner (tuned) + hold | 1.3 / 4.5 | 1.2 / 4.5 | 4.1 / 10.6 | 0.1 | 10.4 | 2.20 (9/10) | 2.44 (4/10) | 22 | 1.80 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.3 / 3.8 | 1.2 / 3.7 | 4.2 / 10.0 | 0.1 | 12.7 | 1.91 (9/10) | 2.38 (4/10) | 1291 | 1.84 |
| DriveToPose chain | 2.9 / 8.8 | 0.9 / 3.3 | 4.0 / 10.0 | 0.0 | 9.0 | 3.88 (8/10) | 4.01 (4/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 4.7 / 11.4 | 2.1 / 5.6 | 4.5 / 10.9 | 0.1 | 11.5 | 1.95 (8/10) | 2.18 (4/10) | 1 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.0 / 12.9 | 2.9 / 7.0 | 4.1 / 10.5 | 0.1 | 8.9 | 2.01 (8/10) | 2.26 (4/10) | 5 | 1.25 |
| Ours v2 (as shipped) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.3 | 0.1 | 9.2 | 2.03 (8/10) | 2.17 (4/10) | 0 | 0.33 |
| Ours v2 (as shipped, adaptive grip off) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.3 | 0.1 | 9.2 | 2.03 (8/10) | 2.17 (4/10) | 0 | 0.33 |
| Ours v2 (tuned) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.3 | 0.1 | 9.2 | 2.03 (8/10) | 2.17 (4/10) | 0 | 0.32 |
| PathPlanner path2 (new-path, pre-release), as shipped | 3.9 / 10.4 | 4.7 / 13.6 | 5.6 / 12.4 | 0.1 | 11.7 | 1.72 (9/10) | 1.85 (2/10) | 4739 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 7.1 / 16.4 | 5.0 / 14.5 | 5.6 / 13.8 | 0.1 | 6.9 | 2.74 (7/10) | never | 1962 | 0.02 |

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
| Ours v2 (as shipped) | 20/90 | 0/9 | 4.5 | 35.8 | 5.8 | 996 |
| Ours v2 (as shipped, adaptive grip off) | 15/90 | 0/9 | 4.9 | 38.4 | 6.5 | 1845 |
| Ours v2 (tuned) | 20/90 | 0/9 | 4.5 | 35.8 | 5.8 | 996 |
| PathPlanner path2 (new-path, pre-release), as shipped | 0/90 | 0/9 | 13.2 | 57.5 | 27.2 | 5892 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 7/90 | 0/9 | 9.5 | 58.9 | 11.9 | 3702 |

**straight_3m**: 3 m straight, heading held at 0 (6.5 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.1 / 0.3 | 0.1 / 0.3 | 4.1 / 8.7 | 0.1 | 8.7 | 1.36 | 1.53 (1/10) | 89 | 18.84 |
| BLine (README gains, r 0.30) | 0.1 / 0.3 | 0.1 / 0.3 | 38.2 / 47.4 | 0.1 | 69.4 | 1.13 | 2.27 (1/10) | 954 | 0.02 |
| BLine (README gains, r 0.45 default) | 0.1 / 0.3 | 0.1 / 0.3 | 38.2 / 47.4 | 0.1 | 69.4 | 1.13 | 2.27 (1/10) | 954 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 0.1 / 0.3 | 0.1 / 0.3 | 38.2 / 47.4 | 0.1 | 69.4 | 1.13 | 2.27 (1/10) | 954 | 0.02 |
| BLine (tuned) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.8 | 0.1 | 4.5 | 2.18 (9/10) | 2.67 (7/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.8 | 0.1 | 4.5 | 2.18 (9/10) | 2.67 (7/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 0.1 / 0.3 | 0.1 / 0.3 | 3.9 / 8.3 | 0.1 | 8.5 | 1.26 | 1.46 (1/10) | 0 | 1.08 |
| PathPlanner (kP 5) + hold | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.1 | 8.5 | 1.26 | 1.71 (4/10) | 0 | 1.07 |
| PathPlanner (tuned) + hold | 0.1 / 0.3 | 0.1 / 0.3 | 2.7 / 6.5 | 0.1 | 6.6 | 1.42 | 1.71 (5/10) | 6 | 1.09 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.1 | 8.5 | 1.26 | 1.71 (4/10) | 0 | 1.07 |
| DriveToPose chain | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.0 | 5.2 | 1.33 (9/10) | 1.52 (5/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 0.3 / 1.1 | 0.1 / 0.4 | 3.0 / 6.9 | 0.0 | 7.8 | 1.26 | 1.48 (5/10) | 66 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 0.3 / 1.0 | 0.1 / 0.5 | 2.3 / 6.1 | 0.1 | 5.1 | 1.32 (9/10) | 1.58 (5/10) | 24 | 0.86 |
| Ours v2 (as shipped) | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 5.4 | 1.29 (9/10) | 1.43 (4/10) | 16 | 0.27 |
| Ours v2 (as shipped, adaptive grip off) | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 5.4 | 1.29 (9/10) | 1.43 (4/10) | 16 | 0.25 |
| Ours v2 (tuned) | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 5.4 | 1.29 (9/10) | 1.43 (4/10) | 16 | 0.24 |
| PathPlanner path2 (new-path, pre-release), as shipped | 0.6 / 1.2 | 0.6 / 1.2 | 12.2 / 16.6 | 0.1 | 0.0 | never | never | 2811 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 0.1 / 0.3 | 0.1 / 0.3 | 2.7 / 7.7 | 0.1 | 3.6 | 2.18 (9/10) | 2.53 (4/10) | 0 | 0.01 |

**s_curve**: S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.5 / 12.0 | 4.5 / 12.0 | 5.3 / 12.7 | 0.1 | 12.4 | 2.24 (9/10) | 2.42 (1/10) | 239 | 35.59 |
| BLine (README gains, r 0.30) | 13.0 / 37.3 | 14.1 / 37.4 | 6.7 / 15.1 | 0.1 | 96.7 | 3.01 (8/10) | never | 346 | 0.02 |
| BLine (README gains, r 0.45 default) | 8.2 / 27.0 | 9.0 / 27.0 | 5.9 / 13.2 | 0.1 | 91.5 | 2.93 (9/10) | never | 282 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 9.7 | 1.8 / 7.1 | 12.5 / 21.0 | 0.1 | 20.9 | 2.22 | never | 523 | 0.02 |
| BLine (tuned) | 6.6 / 16.6 | 4.4 / 15.9 | 4.2 / 11.3 | 0.1 | 8.2 | 2.61 (8/10) | 3.15 (3/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 2.9 / 7.2 | 3.2 / 10.5 | 4.2 / 11.2 | 0.1 | 8.2 | 2.71 (8/10) | 3.26 (3/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 1.7 / 5.8 | 1.6 / 5.8 | 6.0 / 13.9 | 0.1 | 12.6 | 1.89 (8/10) | 2.09 (2/10) | 1718 | 1.79 |
| PathPlanner (kP 5) + hold | 1.7 / 5.8 | 1.6 / 5.8 | 4.9 / 11.1 | 0.1 | 12.6 | 1.89 (8/10) | 2.34 (3/10) | 1718 | 1.72 |
| PathPlanner (tuned) + hold | 1.3 / 4.5 | 1.2 / 4.5 | 4.1 / 10.6 | 0.1 | 10.3 | 2.20 (9/10) | 2.44 (4/10) | 77 | 1.77 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.3 / 3.9 | 1.2 / 3.8 | 4.2 / 10.0 | 0.1 | 12.3 | 1.91 (9/10) | 2.37 (4/10) | 1411 | 1.73 |
| DriveToPose chain | 2.9 / 8.7 | 0.8 / 3.3 | 4.1 / 10.5 | 0.0 | 8.9 | 4.04 (8/10) | 4.20 (4/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 4.6 / 11.2 | 2.0 / 5.6 | 4.5 / 10.8 | 0.1 | 11.3 | 1.95 (8/10) | 2.18 (4/10) | 0 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 4.9 / 12.7 | 2.9 / 6.8 | 4.1 / 10.5 | 0.1 | 8.8 | 2.01 (8/10) | 2.27 (4/10) | 18 | 1.29 |
| Ours v2 (as shipped) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.5 | 0.1 | 9.0 | 2.02 (8/10) | 2.17 (4/10) | 16 | 0.32 |
| Ours v2 (as shipped, adaptive grip off) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.5 | 0.1 | 9.0 | 2.02 (8/10) | 2.17 (4/10) | 16 | 0.30 |
| Ours v2 (tuned) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.5 | 0.1 | 9.0 | 2.02 (8/10) | 2.17 (4/10) | 16 | 0.33 |
| PathPlanner path2 (new-path, pre-release), as shipped | 5.1 / 12.2 | 4.9 / 13.6 | 8.7 / 11.7 | 0.1 | 10.9 | 1.70 (8/10) | never | 4969 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 5.0 / 12.0 | 3.0 / 10.7 | 4.5 / 9.9 | 0.1 | 8.4 | 2.68 (8/10) | 3.12 (1/10) | 2048 | 0.02 |

**hairpin**: 2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held (9.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 7.0 / 18.4 | 7.0 / 18.4 | 1.8 / 3.8 | 0.1 | 0.8 | 2.85 | 3.01 (7/10) | 1395 | 46.38 |
| BLine (README gains, r 0.30) | 45.5 / 104.3 | 45.9 / 107.5 | 7.8 / 9.9 | 0.1 | 8.8 | 6.54 (9/10) | never | 451 | 0.03 |
| BLine (README gains, r 0.45 default) | 47.0 / 104.2 | 47.3 / 107.4 | 1.7 / 4.5 | 0.1 | 53.7 | 5.02 | 5.44 (7/10) | 0 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 47.0 / 104.2 | 47.3 / 107.4 | 1.7 / 4.5 | 0.1 | 53.7 | 5.02 | 5.44 (7/10) | 0 | 0.02 |
| BLine (tuned) | 40.0 / 101.8 | 40.9 / 104.5 | 3.8 / 5.8 | 0.1 | 0.0 | 4.33 (9/10) | never | 0 | 0.02 |
| BLine (tuned, with corner caps) | 40.6 / 103.4 | 41.5 / 106.6 | 3.3 / 5.3 | 0.1 | 0.0 | 4.14 (9/10) | never | 0 | 0.03 |
| PathPlanner (kP 5) | 18.8 / 42.3 | 18.9 / 42.4 | 19.2 / 26.4 | 0.1 | 5.3 | never | never | 6165 | 2.22 |
| PathPlanner (kP 5) + hold | 18.7 / 42.3 | 18.8 / 42.4 | 19.5 / 26.7 | 0.1 | 5.3 | never | never | 6165 | 2.24 |
| PathPlanner (tuned) + hold | 1.6 / 7.1 | 1.6 / 7.1 | 2.3 / 4.1 | 0.1 | 0.0 | 3.16 | 3.28 (4/10) | 77 | 2.36 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.7 / 6.8 | 1.7 / 6.8 | 5.1 / 6.2 | 0.1 | 0.0 | 2.63 | never | 3485 | 2.31 |
| DriveToPose chain | 2.1 / 9.2 | 1.6 / 6.2 | 1.7 / 3.8 | 0.0 | 0.0 | 7.61 | 7.81 (8/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 2.4 / 8.8 | 2.3 / 8.7 | 1.9 / 3.9 | 0.1 | 0.4 | 2.88 | 3.07 (7/10) | 263 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 3.1 / 10.2 | 3.0 / 10.1 | 3.1 / 4.6 | 0.1 | 0.0 | 3.02 | never | 19 | 1.83 |
| Ours v2 (as shipped) | 2.0 / 8.2 | 1.8 / 8.2 | 3.0 / 4.6 | 0.1 | 0.0 | 2.87 | never | 18 | 0.40 |
| Ours v2 (as shipped, adaptive grip off) | 2.0 / 8.2 | 1.8 / 8.2 | 3.0 / 4.6 | 0.1 | 0.0 | 2.87 | never | 18 | 0.41 |
| Ours v2 (tuned) | 2.0 / 8.2 | 1.8 / 8.2 | 3.0 / 4.6 | 0.1 | 0.0 | 2.87 | never | 18 | 0.40 |
| PathPlanner path2 (new-path, pre-release), as shipped | 19.2 / 47.6 | 19.5 / 49.3 | 46.5 / 52.7 | 0.1 | 0.0 | never | never | 7143 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 16.5 / 58.9 | 16.9 / 62.1 | 38.8 / 47.0 | 0.1 | 0.0 | never | never | 7038 | 0.02 |

**slalom**: through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m (10.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.2 / 13.3 | 5.2 / 13.3 | 6.5 / 15.7 | 0.1 | 15.4 | 3.09 (9/10) | 3.26 (1/10) | 146 | 48.26 |
| BLine (README gains, r 0.30) | 37.0 / 82.5 | 36.2 / 78.7 | 28.7 / 43.0 | 0.1 | 42.2 | 4.06 | 4.61 (1/10) | 778 | 0.02 |
| BLine (README gains, r 0.45 default) | 29.7 / 69.0 | 29.8 / 64.4 | 28.8 / 40.4 | 0.1 | 38.8 | 3.85 | never | 762 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 6.6 / 14.1 | 2.6 / 10.5 | 9.6 / 20.8 | 0.1 | 20.6 | 3.30 (9/10) | never | 437 | 0.02 |
| BLine (tuned) | 10.3 / 28.4 | 7.1 / 25.6 | 5.6 / 14.8 | 0.1 | 11.2 | 3.10 (8/10) | 3.65 (2/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 4.3 / 10.4 | 2.4 / 9.3 | 5.7 / 14.8 | 0.1 | 11.2 | 3.78 (8/10) | 4.33 (2/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 4.5 / 14.2 | 4.5 / 14.2 | 7.3 / 16.5 | 0.1 | 15.1 | 2.48 (6/10) | 2.63 (1/10) | 4800 | 2.37 |
| PathPlanner (kP 5) + hold | 4.5 / 14.2 | 4.5 / 14.2 | 6.7 / 13.8 | 0.1 | 15.1 | 2.48 (6/10) | never | 4800 | 2.36 |
| PathPlanner (tuned) + hold | 1.5 / 5.0 | 1.4 / 5.0 | 5.9 / 13.7 | 0.1 | 13.5 | 3.07 (8/10) | 3.39 (3/10) | 581 | 2.31 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 2.0 / 5.4 | 1.9 / 5.4 | 5.6 / 13.2 | 0.1 | 14.4 | 2.56 (9/10) | 3.02 (4/10) | 3736 | 2.41 |
| DriveToPose chain | 4.2 / 11.3 | 1.2 / 4.0 | 5.4 / 14.0 | 0.0 | 12.1 | 4.68 (8/10) | 4.84 (3/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 8.5 / 22.1 | 2.7 / 8.9 | 5.8 / 13.7 | 0.1 | 14.1 | 2.79 (6/10) | 3.00 (4/10) | 196 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.7 / 23.3 | 3.6 / 9.6 | 6.0 / 14.1 | 0.3 | 11.7 | 2.89 (7/10) | 3.10 (1/10) | 22 | 1.74 |
| Ours v2 (as shipped) | 1.6 / 7.4 | 1.6 / 6.9 | 5.6 / 14.1 | 0.1 | 12.1 | 2.79 (8/10) | 2.93 (3/10) | 16 | 0.36 |
| Ours v2 (as shipped, adaptive grip off) | 1.6 / 7.4 | 1.6 / 6.9 | 5.6 / 14.1 | 0.1 | 12.1 | 2.79 (8/10) | 2.93 (3/10) | 16 | 0.35 |
| Ours v2 (tuned) | 1.6 / 7.4 | 1.6 / 6.9 | 5.6 / 14.1 | 0.1 | 12.1 | 2.79 (8/10) | 2.93 (3/10) | 16 | 0.36 |
| PathPlanner path2 (new-path, pre-release), as shipped | 15.9 / 57.5 | 17.5 / 64.1 | 11.9 / 32.7 | 0.1 | 16.3 | 2.36 (3/10) | never | 7974 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 9.0 / 24.2 | 6.1 / 21.5 | 8.0 / 15.1 | 0.1 | 10.1 | 2.96 (2/10) | never | 2389 | 0.02 |

**long_auto**: auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end (14.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 6.1 / 25.4 | 6.1 / 25.4 | 8.4 / 18.9 | 3.0 | 5.8 | 4.11 (4/10) | never | 452 | 75.74 |
| BLine (README gains, r 0.30) | 33.0 / 99.5 | 37.3 / 111.8 | 8.4 / 19.8 | 1.1 | 62.9 | 5.73 (5/10) | 6.62 (1/10) | 623 | 0.02 |
| BLine (README gains, r 0.45 default) | 31.7 / 90.8 | 34.8 / 101.9 | 8.7 / 20.0 | 1.2 | 51.8 | 5.71 (4/10) | 5.98 (1/10) | 149 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 10.8 / 34.0 | 4.8 / 24.1 | 8.4 / 19.1 | 2.4 | 5.9 | 4.93 (4/10) | never | 426 | 0.02 |
| BLine (tuned) | 23.4 / 70.3 | 25.8 / 79.9 | 8.5 / 20.0 | 1.5 | 0.9 | 3.93 (4/10) | never | 27 | 0.02 |
| BLine (tuned, with corner caps) | 10.0 / 32.8 | 5.9 / 22.6 | 8.1 / 19.0 | 1.2 | 1.6 | 5.29 (4/10) | 5.72 (2/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 5.7 / 21.7 | 5.8 / 21.7 | 9.7 / 22.1 | 0.9 | 1.1 | 3.73 (4/10) | 3.80 (1/10) | 2333 | 4.24 |
| PathPlanner (kP 5) + hold | 5.8 / 21.7 | 5.8 / 21.7 | 11.6 / 22.9 | 0.9 | 1.1 | 3.73 (4/10) | never | 2333 | 4.17 |
| PathPlanner (tuned) + hold | 4.7 / 19.3 | 4.7 / 19.3 | 8.2 / 19.2 | 0.9 | 4.3 | 4.17 (4/10) | 4.45 (3/10) | 13 | 4.10 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 4.8 / 20.8 | 4.8 / 20.8 | 8.7 / 20.7 | 0.9 | 5.8 | 3.72 (4/10) | 4.15 (1/10) | 1715 | 4.13 |
| DriveToPose chain | 8.1 / 31.5 | 4.6 / 18.9 | 8.0 / 17.8 | 0.9 | 5.0 | 6.54 (4/10) | 6.64 (3/10) | 2 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 9.2 / 27.8 | 5.3 / 20.5 | 8.0 / 18.7 | 1.8 | 5.7 | 3.76 (4/10) | 3.98 (2/10) | 182 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 9.8 / 29.1 | 6.1 / 22.8 | 8.3 / 19.3 | 1.0 | 3.2 | 3.85 (4/10) | 4.09 (2/10) | 176 | 2.32 |
| Ours v2 (as shipped) | 6.6 / 21.7 | 4.9 / 19.7 | 8.2 / 19.6 | 0.9 | 2.9 | 3.87 (4/10) | 4.00 (2/10) | 18 | 0.43 |
| Ours v2 (as shipped, adaptive grip off) | 6.6 / 21.7 | 4.9 / 19.7 | 8.2 / 19.6 | 0.9 | 2.9 | 3.87 (4/10) | 4.00 (2/10) | 18 | 0.44 |
| Ours v2 (tuned) | 6.6 / 21.7 | 4.9 / 19.7 | 8.2 / 19.6 | 0.9 | 2.9 | 3.87 (4/10) | 4.00 (2/10) | 18 | 0.43 |
| PathPlanner path2 (new-path, pre-release), as shipped | 11.4 / 45.9 | 15.2 / 67.0 | 10.2 / 20.8 | 0.8 | 2.6 | 3.80 (2/10) | never | 5020 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 9.7 / 35.0 | 11.1 / 51.8 | 12.2 / 25.0 | 0.9 | 0.0 | 4.82 (1/10) | never | 3158 | 0.02 |

**moving_start**: starts moving 2 m/s sideways (+y), path heads +x then curves left (7.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 13.2 / 27.7 | 13.2 / 27.7 | 6.2 / 12.0 | 0.1 | 11.7 | 1.87 (9/10) | never | 1546 | 30.43 |
| BLine (README gains, r 0.30) | 30.8 / 59.4 | 31.9 / 66.2 | 29.4 / 35.3 | 0.1 | 35.2 | 3.44 | never | 840 | 0.02 |
| BLine (README gains, r 0.45 default) | 33.1 / 54.8 | 34.2 / 51.5 | 23.7 / 39.3 | 0.1 | 55.8 | 2.14 | never | 587 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 24.3 / 49.6 | 24.9 / 47.1 | 11.0 / 16.4 | 0.1 | 15.9 | 2.17 | never | 510 | 0.02 |
| BLine (tuned) | 34.9 / 56.2 | 33.7 / 52.2 | 3.7 / 8.5 | 0.1 | 6.0 | 2.09 (9/10) | 2.37 (4/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 18.7 / 47.6 | 19.5 / 45.6 | 3.7 / 9.5 | 0.1 | 5.0 | 3.07 (8/10) | 3.56 (3/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 40.7 / 52.6 | 40.7 / 52.6 | 47.2 / 51.7 | 0.1 | 9.3 | never | never | 2772 | 1.54 |
| PathPlanner (kP 5) + hold | 40.7 / 52.6 | 40.7 / 52.6 | 47.1 / 52.0 | 0.1 | 9.3 | never | never | 2772 | 1.52 |
| PathPlanner (tuned) + hold | 41.7 / 54.0 | 41.7 / 54.0 | 49.6 / 53.1 | 0.1 | 12.7 | never | never | 1929 | 1.50 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 40.7 / 52.6 | 40.6 / 52.5 | 47.0 / 52.0 | 0.1 | 9.5 | never | never | 2781 | 1.48 |
| DriveToPose chain | 21.1 / 58.7 | 19.5 / 54.8 | 3.8 / 8.3 | 0.0 | 7.6 | 3.65 (9/10) | 3.86 (4/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 19.5 / 25.8 | 21.2 / 25.7 | 25.2 / 26.7 | 0.0 | 18.0 | never | never | 1937 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 11.3 / 28.1 | 11.7 / 29.5 | 4.0 / 9.7 | 0.3 | 6.5 | 1.82 (8/10) | 2.09 (2/10) | 1921 | 1.11 |
| Ours v2 (as shipped) | 11.3 / 28.6 | 11.4 / 28.6 | 3.7 / 8.7 | 0.1 | 5.9 | 1.85 (8/10) | 1.96 (3/10) | 1462 | 0.30 |
| Ours v2 (as shipped, adaptive grip off) | 11.4 / 28.1 | 11.5 / 28.1 | 4.1 / 9.6 | 0.1 | 6.5 | 1.80 (8/10) | 1.95 (2/10) | 1826 | 0.30 |
| Ours v2 (tuned) | 11.3 / 28.6 | 11.4 / 28.6 | 3.7 / 8.7 | 0.1 | 5.9 | 1.85 (8/10) | 1.96 (3/10) | 1462 | 0.32 |
| PathPlanner path2 (new-path, pre-release), as shipped | 10.7 / 38.5 | 10.6 / 36.2 | 15.0 / 24.8 | 0.1 | 0.0 | never | never | 5287 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 11.4 / 38.4 | 10.0 / 36.2 | 10.7 / 23.8 | 0.1 | 0.0 | 2.62 (2/10) | never | 3573 | 0.02 |

**hit_mid_path**: the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 12.6 / 37.3 | 12.6 / 37.3 | 9.3 / 15.1 | 0.1 | 11.0 | never | never | 3287 | 33.73 |
| BLine (README gains, r 0.30) | 33.7 / 74.3 | 35.6 / 78.2 | 42.8 / 49.6 | 0.1 | 47.8 | 4.32 (4/10) | never | 4110 | 0.02 |
| BLine (README gains, r 0.45 default) | 24.0 / 60.4 | 25.8 / 63.4 | 9.5 / 14.6 | 0.1 | 86.5 | 3.07 (3/10) | never | 3293 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 6.3 / 18.3 | 7.5 / 23.0 | 11.5 / 19.9 | 0.1 | 18.7 | 2.28 (7/10) | never | 2909 | 0.02 |
| BLine (tuned) | 16.9 / 37.6 | 18.3 / 43.0 | 7.1 / 12.0 | 0.1 | 5.0 | 2.10 (5/10) | never | 2946 | 0.02 |
| BLine (tuned, with corner caps) | 7.8 / 22.4 | 9.4 / 26.0 | 6.5 / 11.7 | 0.1 | 5.2 | 2.53 (4/10) | never | 2808 | 0.02 |
| PathPlanner (kP 5) | 21.0 / 52.3 | 21.0 / 52.3 | 23.5 / 34.6 | 0.1 | 28.0 | never | never | 5226 | 1.78 |
| PathPlanner (kP 5) + hold | 21.0 / 52.3 | 21.0 / 52.3 | 10.6 / 21.3 | 0.1 | 27.9 | 2.36 (3/10) | never | 5266 | 1.79 |
| PathPlanner (tuned) + hold | 15.7 / 39.9 | 15.7 / 40.2 | 24.3 / 33.6 | 0.1 | 5.0 | 2.28 (1/10) | never | 4773 | 1.74 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 20.8 / 49.7 | 20.9 / 49.7 | 20.7 / 31.4 | 0.1 | 18.6 | 2.16 (1/10) | never | 5301 | 1.83 |
| DriveToPose chain | 7.2 / 29.0 | 7.8 / 32.0 | 3.9 / 9.1 | 0.0 | 7.5 | 4.22 (8/10) | 4.29 (3/10) | 2272 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 47.9 / 97.2 | 46.3 / 94.8 | 28.5 / 32.6 | 0.2 | 28.5 | 2.24 (1/10) | never | 5520 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 47.3 / 103.7 | 45.7 / 99.7 | 24.4 / 27.4 | 0.3 | 58.1 | never | never | 6361 | 1.31 |
| Ours v2 (as shipped) | 12.7 / 35.8 | 12.8 / 36.9 | 11.3 / 17.0 | 0.1 | 7.3 | 2.16 (1/10) | never | 3567 | 0.32 |
| Ours v2 (as shipped, adaptive grip off) | 13.0 / 38.4 | 13.2 / 39.5 | 13.7 / 19.5 | 0.1 | 3.2 | never | never | 3606 | 0.31 |
| Ours v2 (tuned) | 12.7 / 35.8 | 12.8 / 36.9 | 11.3 / 17.0 | 0.1 | 7.3 | 2.16 (1/10) | never | 3567 | 0.36 |
| PathPlanner path2 (new-path, pre-release), as shipped | 41.0 / 57.4 | 41.4 / 59.3 | 46.4 / 56.9 | 0.1 | 0.0 | never | never | 6190 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 21.9 / 51.7 | 23.7 / 57.1 | 16.9 / 24.4 | 0.1 | 19.7 | never | never | 4672 | 0.02 |

**low_grip_s_curve**: the S-curve on mu 0.70 static / 0.61 kinetic (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.4 / 11.9 | 4.4 / 11.9 | 12.1 / 19.9 | 0.1 | 19.7 | 2.23 | never | 4682 | 56.18 |
| BLine (README gains, r 0.30) | 12.2 / 36.6 | 13.4 / 36.6 | 6.4 / 15.6 | 0.1 | 97.2 | 3.06 (8/10) | never | 11148 | 0.03 |
| BLine (README gains, r 0.45 default) | 7.5 / 25.4 | 8.3 / 25.5 | 4.9 / 13.0 | 0.1 | 92.1 | 3.01 (8/10) | 3.14 (1/10) | 10434 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 9.7 | 1.8 / 7.5 | 19.9 / 28.8 | 0.1 | 28.6 | 2.23 | never | 5261 | 0.03 |
| BLine (tuned) | 7.2 / 17.9 | 5.0 / 17.1 | 5.1 / 14.0 | 0.1 | 5.6 | 2.77 (7/10) | 3.19 (3/10) | 5218 | 0.03 |
| BLine (tuned, with corner caps) | 2.7 / 6.0 | 2.9 / 10.5 | 4.3 / 12.1 | 0.1 | 6.9 | 2.77 (8/10) | 3.26 (2/10) | 6199 | 0.03 |
| PathPlanner (kP 5) | 9.5 / 20.6 | 9.5 / 20.3 | 42.0 / 50.9 | 0.1 | 50.0 | 1.81 (7/10) | never | 7222 | 3.11 |
| PathPlanner (kP 5) + hold | 9.5 / 20.6 | 9.5 / 20.3 | 40.9 / 50.9 | 0.1 | 50.0 | 1.81 (7/10) | never | 7272 | 2.93 |
| PathPlanner (tuned) + hold | 2.0 / 9.1 | 1.9 / 9.1 | 7.0 / 14.5 | 0.1 | 11.3 | 2.21 (6/10) | 2.36 (1/10) | 6546 | 2.95 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 7.0 / 16.3 | 6.9 / 16.2 | 29.3 / 38.2 | 0.1 | 36.9 | 1.86 (4/10) | never | 7166 | 3.27 |
| DriveToPose chain | 3.1 / 9.4 | 1.4 / 4.2 | 5.4 / 13.2 | 0.0 | 4.2 | 4.00 (5/10) | 4.12 (3/10) | 12448 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 4.9 / 13.8 | 4.0 / 12.5 | 10.2 / 15.8 | 0.1 | 15.7 | 1.94 (7/10) | never | 8315 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.3 / 13.6 | 3.8 / 7.4 | 15.7 / 33.1 | 0.0 | 28.9 | 1.97 (6/10) | never | 8326 | 2.05 |
| Ours v2 (as shipped) | 1.6 / 6.4 | 1.5 / 5.3 | 4.0 / 10.3 | 0.1 | 9.0 | 2.48 (8/10) | 2.70 (4/10) | 516 | 0.40 |
| Ours v2 (as shipped, adaptive grip off) | 5.6 / 14.7 | 5.5 / 13.6 | 10.6 / 21.1 | 0.1 | 2.0 | 2.11 (1/10) | never | 7502 | 0.40 |
| Ours v2 (tuned) | 1.6 / 6.4 | 1.5 / 5.3 | 4.0 / 10.3 | 0.1 | 9.0 | 2.48 (8/10) | 2.70 (4/10) | 516 | 0.39 |
| PathPlanner path2 (new-path, pre-release), as shipped | 7.3 / 19.6 | 5.2 / 18.4 | 22.2 / 31.9 | 0.1 | 31.0 | 1.84 (3/10) | never | 7238 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 6.7 / 16.3 | 4.9 / 15.5 | 7.2 / 15.9 | 0.1 | 15.9 | 2.44 (8/10) | never | 6310 | 0.02 |

**grip_drop_s_curve**: the S-curve; halfway along, the floor drops from mu 1.15 / 1.0 to 0.70 / 0.61 (a worn carpet seam) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.3 / 11.3 | 4.3 / 11.3 | 12.2 / 20.6 | 0.1 | 20.4 | 2.22 | never | 3244 | 56.80 |
| BLine (README gains, r 0.30) | 13.3 / 38.7 | 14.4 / 38.7 | 6.9 / 13.2 | 0.1 | 102.4 | 3.07 (6/10) | never | 7046 | 0.03 |
| BLine (README gains, r 0.45 default) | 8.5 / 28.3 | 9.4 / 28.4 | 6.9 / 15.1 | 0.1 | 97.3 | 3.02 (6/10) | never | 6726 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.2 / 8.8 | 1.7 / 6.7 | 22.9 / 31.9 | 0.1 | 31.7 | 2.21 | never | 2590 | 0.03 |
| BLine (tuned) | 6.8 / 16.3 | 4.6 / 15.5 | 5.2 / 12.2 | 0.1 | 11.6 | 2.48 (9/10) | 3.15 (1/10) | 2081 | 0.03 |
| BLine (tuned, with corner caps) | 3.0 / 7.6 | 3.5 / 11.7 | 4.9 / 11.8 | 0.1 | 11.1 | 2.59 (9/10) | 3.24 (2/10) | 2574 | 0.03 |
| PathPlanner (kP 5) | 5.5 / 21.9 | 5.5 / 21.9 | 50.4 / 60.6 | 0.1 | 60.0 | never | never | 4416 | 2.87 |
| PathPlanner (kP 5) + hold | 5.5 / 21.9 | 5.5 / 21.9 | 10.2 / 22.3 | 0.1 | 59.1 | 2.89 (1/10) | never | 5596 | 3.08 |
| PathPlanner (tuned) + hold | 1.8 / 10.6 | 1.8 / 10.6 | 7.9 / 17.8 | 0.1 | 14.5 | 2.20 (5/10) | never | 3236 | 3.13 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 5.4 / 19.1 | 5.4 / 19.0 | 8.2 / 20.7 | 0.1 | 46.7 | 2.76 (3/10) | 2.95 (2/10) | 5126 | 2.94 |
| DriveToPose chain | 2.9 / 8.4 | 0.9 / 3.1 | 4.0 / 10.4 | 0.0 | 7.0 | 4.03 (8/10) | 4.17 (3/10) | 6122 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 8.4 / 22.8 | 6.0 / 20.9 | 21.5 / 33.8 | 0.1 | 23.5 | never | never | 4202 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.1 / 17.4 | 5.5 / 13.6 | 16.2 / 19.7 | 0.0 | 16.0 | never | never | 4034 | 2.00 |
| Ours v2 (as shipped) | 3.1 / 9.8 | 3.3 / 9.8 | 10.3 / 14.9 | 0.1 | 7.3 | never | never | 3336 | 0.42 |
| Ours v2 (as shipped, adaptive grip off) | 1.9 / 6.1 | 2.1 / 6.1 | 7.2 / 13.4 | 0.1 | 12.3 | 2.07 (2/10) | never | 3586 | 0.42 |
| Ours v2 (tuned) | 3.1 / 9.8 | 3.3 / 9.8 | 10.3 / 14.9 | 0.1 | 7.3 | never | never | 3336 | 0.42 |
| PathPlanner path2 (new-path, pre-release), as shipped | 7.6 / 26.9 | 8.1 / 27.0 | 71.7 / 76.9 | 0.1 | 74.6 | never | never | 6392 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 5.4 / 11.8 | 5.1 / 14.2 | 6.3 / 14.2 | 0.1 | 10.4 | 2.49 (4/10) | 2.85 (2/10) | 4131 | 0.02 |

**spin_while_driving**: 5 m straight while the robot turns a full 360 deg (a heading target at every waypoint) (8.2 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 2.1 / 6.9 | 2.1 / 6.9 | 5.9 / 12.0 | 14.2 | 11.8 | 2.20 (8/10) | never | 660 | 32.55 |
| BLine (README gains, r 0.30) | 2.8 / 7.7 | 2.8 / 7.7 | 5.9 / 12.5 | 3.1 | 63.4 | 1.93 (8/10) | never | 402 | 0.02 |
| BLine (README gains, r 0.45 default) | 3.2 / 7.7 | 3.2 / 7.7 | 5.9 / 12.6 | 3.1 | 63.6 | 1.94 (8/10) | never | 420 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.2 / 7.7 | 3.2 / 7.7 | 5.9 / 12.6 | 3.1 | 63.6 | 1.94 (8/10) | never | 420 | 0.02 |
| BLine (tuned) | 3.6 / 8.2 | 3.6 / 8.2 | 4.6 / 11.2 | 3.1 | 8.4 | 2.78 (8/10) | never | 34 | 0.02 |
| BLine (tuned, with corner caps) | 2.3 / 5.1 | 2.3 / 5.1 | 4.6 / 11.3 | 3.1 | 8.4 | 2.77 (8/10) | never | 38 | 0.02 |
| PathPlanner (kP 5) | 2.2 / 7.9 | 2.2 / 7.9 | 6.0 / 11.9 | 1.7 | 11.4 | 2.93 (8/10) | never | 108 | 1.82 |
| PathPlanner (kP 5) + hold | 2.2 / 7.9 | 2.2 / 7.9 | 5.1 / 12.0 | 1.8 | 11.3 | 2.93 (8/10) | 3.35 (3/10) | 108 | 1.84 |
| PathPlanner (tuned) + hold | 1.6 / 7.2 | 1.6 / 7.2 | 5.1 / 11.5 | 1.8 | 9.9 | 3.47 (8/10) | 3.72 (3/10) | 11 | 1.80 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 2.2 / 7.9 | 2.2 / 7.9 | 5.1 / 12.1 | 1.8 | 11.4 | 2.94 (8/10) | 3.36 (3/10) | 84 | 1.81 |
| DriveToPose chain | 1.5 / 7.9 | 1.5 / 7.9 | 5.0 / 10.8 | 3.4 | 9.6 | 4.85 (8/10) | never | 279 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 3.1 / 8.0 | 2.4 / 7.7 | 5.5 / 11.3 | 5.2 | 11.2 | 1.94 (8/10) | never | 798 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.4 / 11.8 | 4.8 / 11.6 | 5.0 / 12.1 | 3.4 | 8.8 | 2.08 (8/10) | never | 300 | 1.44 |
| Ours v2 (as shipped) | 2.9 / 8.3 | 2.9 / 8.3 | 5.1 / 11.9 | 2.0 | 8.9 | 2.55 (8/10) | 2.84 (2/10) | 140 | 0.35 |
| Ours v2 (as shipped, adaptive grip off) | 2.9 / 8.3 | 2.9 / 8.3 | 5.1 / 11.9 | 2.0 | 8.9 | 2.55 (8/10) | 2.84 (2/10) | 140 | 0.34 |
| Ours v2 (tuned) | 2.9 / 8.3 | 2.9 / 8.3 | 5.1 / 11.9 | 2.0 | 8.9 | 2.55 (8/10) | 2.84 (2/10) | 140 | 0.37 |
| PathPlanner path2 (new-path, pre-release), as shipped | 2.0 / 6.3 | 2.0 / 6.3 | 8.6 / 19.3 | 2.8 | 0.1 | 2.19 (4/10) | never | 2081 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 3.4 / 10.6 | 3.4 / 10.6 | 4.3 / 11.1 | 1.8 | 6.5 | 2.83 (8/10) | 3.20 (5/10) | 1877 | 0.02 |

**figure_eight**: a self-crossing loop (a lemniscate, 4 m x 1.8 m) through the origin twice; heading held (12.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.5 / 15.4 | 5.5 / 15.4 | 1.9 / 4.1 | 0.1 | 3.4 | 5.12 | 5.31 (7/10) | 679 | 70.38 |
| BLine (README gains, r 0.30) | 43.7 / 117.5 | 44.1 / 117.6 | 2.2 / 3.9 | 0.1 | 58.3 | 9.88 | 9.98 (6/10) | 7 | 0.04 |
| BLine (README gains, r 0.45 default) | 50.1 / 117.1 | 51.0 / 117.9 | 4.0 / 5.3 | 0.1 | 67.8 | 6.59 (9/10) | never | 0 | 0.04 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 40.2 / 90.9 | 41.2 / 93.2 | 3.2 / 4.4 | 0.1 | 48.8 | 6.05 | never | 96 | 0.04 |
| BLine (tuned) | 42.2 / 102.5 | 43.0 / 104.9 | 1.8 / 3.0 | 0.1 | 0.5 | 4.99 | 5.33 (8/10) | 0 | 0.04 |
| BLine (tuned, with corner caps) | 43.4 / 96.4 | 44.4 / 98.2 | 2.7 / 4.2 | 0.1 | 0.0 | 6.00 | never | 0 | 0.04 |
| PathPlanner (kP 5) | 30.6 / 82.2 | 30.6 / 82.0 | 96.3 / 103.7 | 0.1 | 80.6 | never | never | 9129 | 4.04 |
| PathPlanner (kP 5) + hold | 32.5 / 82.2 | 32.5 / 82.0 | 94.8 / 102.7 | 0.1 | 80.6 | never | never | 9129 | 3.97 |
| PathPlanner (tuned) + hold | 1.8 / 9.2 | 1.7 / 9.1 | 2.0 / 3.0 | 0.1 | 1.7 | 4.45 | 4.70 (6/10) | 250 | 4.16 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 3.9 / 12.8 | 3.9 / 12.8 | 8.7 / 14.4 | 0.1 | 80.6 | 3.73 (5/10) | 4.08 (1/10) | 8554 | 4.01 |
| DriveToPose chain | 2.1 / 10.6 | 1.5 / 7.7 | 20.1 / 44.7 | 0.0 | 80.6 | 12.25 (3/10) | never | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 2.5 / 8.7 | 2.2 / 8.7 | 1.5 / 3.7 | 0.1 | 2.8 | 4.30 | 4.50 (8/10) | 119 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 3.3 / 10.5 | 3.0 / 10.5 | 1.6 / 2.6 | 0.3 | 0.9 | 4.42 | 4.66 (8/10) | 403 | 4.61 |
| Ours v2 (as shipped) | 2.1 / 9.8 | 2.1 / 9.5 | 1.9 / 2.9 | 0.1 | 0.4 | 4.38 | 4.50 (7/10) | 18 | 0.67 |
| Ours v2 (as shipped, adaptive grip off) | 2.1 / 9.8 | 2.1 / 9.5 | 1.9 / 2.9 | 0.1 | 0.4 | 4.38 | 4.50 (7/10) | 18 | 0.61 |
| Ours v2 (tuned) | 2.1 / 9.8 | 2.1 / 9.5 | 1.9 / 2.9 | 0.1 | 0.4 | 4.38 | 4.50 (7/10) | 18 | 0.57 |
| PathPlanner path2 (new-path, pre-release), as shipped | 35.7 / 80.8 | 36.3 / 81.5 | 45.6 / 61.9 | 0.1 | 35.7 | 4.13 (1/10) | never | 13674 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 27.7 / 73.0 | 27.5 / 74.4 | 39.6 / 49.3 | 0.1 | 80.6 | never | never | 10857 | 0.02 |

**wall_hug**: along a wall with the bumpers 15 cm off it, then a 90 deg turn to run 15 cm off the next wall (11.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.1 / 22.8 | 5.0 / 21.8 | 6.0 / 14.0 | 0.1 | 7.7 | 3.05 (4/10) | 3.23 (3/10) | 417 | 57.08 |
| BLine (README gains, r 0.30) | 40.1 / 103.8 | 39.9 / 103.8 | 6.1 / 14.7 | 0.1 | 68.8 | 5.42 (6/10) | 5.70 (3/10) | 91 | 0.03 |
| BLine (README gains, r 0.45 default) | 38.8 / 99.7 | 38.9 / 99.7 | 5.9 / 14.5 | 0.1 | 61.7 | 5.27 (6/10) | 5.56 (3/10) | 6 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 38.8 / 99.7 | 38.9 / 99.7 | 5.9 / 14.5 | 0.1 | 61.7 | 5.27 (6/10) | 5.56 (3/10) | 6 | 0.02 |
| BLine (tuned) | 30.3 / 79.0 | 30.4 / 79.0 | 5.9 / 13.5 | 0.1 | 5.0 | 2.98 (6/10) | 3.52 (3/10) | 10 | 0.02 |
| BLine (tuned, with corner caps) | 40.5 / 102.6 | 40.7 / 102.6 | 5.3 / 12.7 | 0.1 | 8.8 | 3.84 (6/10) | 4.36 (4/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 5.6 / 24.5 | 5.6 / 24.5 | 7.8 / 17.0 | 0.1 | 3.7 | 2.80 (5/10) | 2.93 (1/10) | 2518 | 2.85 |
| PathPlanner (kP 5) + hold | 5.6 / 24.5 | 5.6 / 24.5 | 8.5 / 17.2 | 0.1 | 3.7 | 2.80 (5/10) | 3.19 (1/10) | 2518 | 2.85 |
| PathPlanner (tuned) + hold | 3.2 / 12.3 | 3.1 / 12.1 | 5.9 / 13.6 | 0.1 | 4.8 | 3.24 (4/10) | 3.37 (1/10) | 28 | 2.69 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 3.3 / 13.2 | 3.3 / 13.2 | 7.0 / 13.5 | 0.1 | 5.5 | 2.87 (5/10) | never | 1572 | 2.83 |
| DriveToPose chain | 3.3 / 15.0 | 3.0 / 12.1 | 5.2 / 13.5 | 0.0 | 5.6 | 7.16 (7/10) | 7.30 (3/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 3.4 / 14.7 | 3.1 / 12.7 | 5.3 / 13.0 | 0.1 | 7.0 | 2.96 (4/10) | 3.17 (4/10) | 172 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 4.1 / 16.3 | 3.9 / 14.3 | 5.7 / 13.0 | 0.2 | 4.4 | 3.05 (4/10) | 3.26 (3/10) | 23 | 1.92 |
| Ours v2 (as shipped) | 3.3 / 13.4 | 3.2 / 13.3 | 5.6 / 12.9 | 0.1 | 4.6 | 2.98 (4/10) | 3.10 (3/10) | 16 | 0.46 |
| Ours v2 (as shipped, adaptive grip off) | 3.3 / 13.4 | 3.2 / 13.3 | 5.6 / 12.9 | 0.1 | 4.6 | 2.98 (4/10) | 3.10 (3/10) | 16 | 0.39 |
| Ours v2 (tuned) | 3.3 / 13.4 | 3.2 / 13.3 | 5.6 / 12.9 | 0.1 | 4.6 | 2.98 (4/10) | 3.10 (3/10) | 16 | 0.37 |
| PathPlanner path2 (new-path, pre-release), as shipped | 11.2 / 42.0 | 11.4 / 42.0 | 39.3 / 70.7 | 0.1 | 20.1 | 2.56 (2/10) | never | 6571 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 22.3 / 44.9 | 22.4 / 44.9 | 37.2 / 46.5 | 0.1 | 0.0 | never | never | 4460 | 0.02 |

**tag_align_moving_goal**: 5 m to a scoring pose; the true goal is 8 cm sideways and 3 deg off the map pose, revealed 1.5 m out (8.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 3.0 / 8.1 | 3.0 / 8.1 | 1.3 / 3.4 | 0.3 | 14.4 | 2.20 | 2.43 (8/10) | 103 | 32.91 |
| BLine (README gains, r 0.30) | 3.4 / 7.4 | 4.5 / 13.7 | 1.3 / 2.9 | 0.2 | 86.8 | 2.81 | 3.01 (8/10) | 68 | 0.02 |
| BLine (README gains, r 0.45 default) | 3.3 / 7.4 | 2.3 / 8.6 | 1.1 / 2.8 | 0.1 | 86.7 | 2.80 | 2.99 (8/10) | 97 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.3 / 8.0 | 0.8 / 4.2 | 1.3 / 2.8 | 0.2 | 57.7 | 2.71 | 2.92 (8/10) | 47 | 0.02 |
| BLine (tuned) | 6.6 / 17.4 | 4.0 / 16.5 | 1.1 / 3.1 | 0.3 | 9.4 | 3.33 | 3.58 (7/10) | 8 | 0.02 |
| BLine (tuned, with corner caps) | 3.4 / 8.2 | 1.1 / 5.5 | 1.3 / 3.1 | 0.3 | 9.4 | 3.36 | 3.61 (7/10) | 15 | 0.02 |
| PathPlanner (kP 5) | 3.1 / 7.8 | 3.0 / 7.8 | 2.8 / 5.5 | 0.0 | 5.8 | 1.80 | 1.98 (2/10) | 15 | 2.08 |
| PathPlanner (kP 5) + hold | 3.1 / 7.8 | 3.0 / 7.8 | 1.4 / 3.0 | 0.0 | 5.8 | 1.80 | 2.25 (7/10) | 15 | 1.68 |
| PathPlanner (tuned) + hold | 3.2 / 8.0 | 3.2 / 8.0 | 1.9 / 3.8 | 0.1 | 3.4 | 1.95 | 2.17 (6/10) | 267 | 1.64 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 3.1 / 7.8 | 3.0 / 7.8 | 1.4 / 3.0 | 0.0 | 5.8 | 1.80 | 2.25 (7/10) | 16 | 1.66 |
| DriveToPose chain | 4.3 / 8.7 | 2.5 / 7.6 | 1.0 / 3.0 | 0.3 | 2.4 | 2.63 | 2.82 (8/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 1.1 / 5.7 | 1.2 / 5.5 | 1.2 / 3.2 | 0.3 | 12.5 | 2.14 | 2.38 (8/10) | 34 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.9 / 6.9 | 2.0 / 6.7 | 1.3 / 3.2 | 0.3 | 3.2 | 1.84 | 2.09 (8/10) | 291 | 1.32 |
| Ours v2 (as shipped) | 2.9 / 7.7 | 2.9 / 7.7 | 1.0 / 3.9 | 0.2 | 1.7 | 1.82 | 1.95 (9/10) | 16 | 0.31 |
| Ours v2 (as shipped, adaptive grip off) | 2.9 / 7.7 | 2.9 / 7.7 | 1.0 / 3.9 | 0.2 | 1.7 | 1.82 | 1.95 (9/10) | 16 | 0.28 |
| Ours v2 (tuned) | 2.9 / 7.7 | 2.9 / 7.7 | 1.0 / 3.9 | 0.2 | 1.7 | 1.82 | 1.95 (9/10) | 16 | 0.32 |
| PathPlanner path2 (new-path, pre-release), as shipped | 3.7 / 7.6 | 1.6 / 5.8 | 2.4 / 5.0 | 0.2 | 3.0 | 1.80 | 2.01 (4/10) | 3190 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 4.4 / 11.7 | 1.7 / 8.3 | 1.2 / 3.0 | 0.2 | 8.4 | 3.17 | 3.41 (7/10) | 374 | 0.02 |

**intake_drive_through**: drive through a game piece at 2 m/s; its detected position moves 10 cm sideways 2 m out (7.4 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.9 / 2.3 | 0.9 / 2.3 | 11.3 / 14.3 | 0.1 | 10.6 | never | never | 88 | 26.54 |
| BLine (README gains, r 0.30) | 3.6 / 9.9 | 3.4 / 9.9 | 39.4 / 44.7 | 0.1 | 68.7 | 1.39 | never | 1042 | 0.02 |
| BLine (README gains, r 0.45 default) | 3.7 / 9.8 | 3.5 / 9.9 | 39.5 / 44.8 | 0.1 | 68.7 | 1.39 | never | 1046 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.7 / 9.8 | 3.5 / 9.9 | 39.5 / 44.8 | 0.1 | 68.7 | 1.39 | never | 1046 | 0.02 |
| BLine (tuned) | 4.5 / 9.8 | 4.3 / 9.8 | 22.3 / 25.5 | 0.1 | 41.9 | 1.65 | never | 770 | 0.02 |
| BLine (tuned, with corner caps) | 4.2 / 9.9 | 4.1 / 9.9 | 22.3 / 25.5 | 0.1 | 41.9 | 1.65 | never | 778 | 0.02 |
| PathPlanner (kP 5) | 3.9 / 10.1 | 3.9 / 10.1 | 24.8 / 28.8 | 0.1 | 28.7 | 1.41 | never | 707 | 1.35 |
| PathPlanner (kP 5) + hold | 3.9 / 10.1 | 3.9 / 10.1 | 24.8 / 28.8 | 0.1 | 28.7 | 1.41 | never | 707 | 1.37 |
| PathPlanner (tuned) + hold | 3.9 / 10.1 | 3.9 / 10.1 | 21.8 / 26.1 | 0.1 | 26.1 | 1.51 | never | 836 | 1.39 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 3.9 / 10.1 | 3.9 / 10.1 | 24.8 / 28.8 | 0.1 | 28.7 | 1.41 | never | 707 | 1.34 |
| DriveToPose chain | 3.7 / 9.8 | 3.6 / 9.8 | 1.6 / 4.2 | 0.0 | 3.4 | 2.34 | 2.55 (7/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 0.3 / 0.7 | 0.3 / 0.8 | 26.9 / 34.1 | 0.3 | 32.6 | never | never | 675 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.7 / 8.9 | 1.7 / 9.0 | 1.7 / 4.1 | 0.2 | 4.1 | 1.60 | 1.85 (7/10) | 354 | 0.89 |
| Ours v2 (as shipped) | 4.0 / 9.9 | 4.0 / 9.9 | 24.0 / 27.7 | 0.1 | 27.7 | 1.40 | never | 742 | 0.27 |
| Ours v2 (as shipped, adaptive grip off) | 4.0 / 9.9 | 4.0 / 9.9 | 24.0 / 27.7 | 0.1 | 27.7 | 1.40 | never | 742 | 0.31 |
| Ours v2 (tuned) | 4.0 / 9.9 | 4.0 / 9.9 | 24.0 / 27.7 | 0.1 | 27.7 | 1.40 | never | 742 | 0.29 |
| PathPlanner path2 (new-path, pre-release), as shipped | 0.8 / 2.4 | 0.6 / 1.4 | 15.6 / 19.7 | 0.1 | 0.0 | never | never | 2821 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 1.1 / 4.9 | 1.4 / 5.2 | 10.8 / 14.2 | 0.1 | 5.3 | never | never | 562 | 0.02 |

**three_piece_chain**: three paths with rolling handoffs at 2 m/s (core-builder's neutral_three_piece, 2026 field) (15.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 7.9 / 30.2 | 5.7 / 23.5 | 5.4 / 14.1 | 3.8 | 398.2 | 5.62 (7/10) | never | 865 | 29.91 |
| BLine (README gains, r 0.30) | 20.5 / 56.6 | 21.2 / 56.6 | 5.2 / 9.0 | 0.4 | 109.0 | 6.87 (6/10) | 7.01 (1/10) | 4857 | 0.03 |
| BLine (README gains, r 0.45 default) | 19.8 / 54.2 | 20.2 / 54.2 | 5.3 / 9.8 | 0.4 | 107.8 | 6.84 (6/10) | 6.94 (1/10) | 4726 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 10.6 / 34.3 | 10.6 / 37.9 | 7.2 / 17.2 | 1.8 | 398.2 | 5.15 (4/10) | never | 3597 | 0.03 |
| BLine (tuned) | 14.5 / 51.2 | 13.2 / 39.7 | 9.1 / 17.0 | 1.8 | 398.2 | 7.04 (4/10) | never | 3821 | 0.02 |
| BLine (tuned, with corner caps) | 11.3 / 49.9 | 10.6 / 41.6 | 52.6 / 444.4 | 136.9 | 398.2 | 8.08 (2/10) | never | 3688 | 0.03 |
| PathPlanner (kP 5) | 4.8 / 18.0 | 4.7 / 16.1 | 5.5 / 13.3 | 0.3 | 398.2 | 4.64 (6/10) | 4.78 (2/10) | 3920 | 1.41 |
| PathPlanner (kP 5) + hold | 4.8 / 18.0 | 4.7 / 16.1 | 6.5 / 11.8 | 0.3 | 398.2 | 4.64 (6/10) | 5.02 (1/10) | 3920 | 1.42 |
| PathPlanner (tuned) + hold | 4.1 / 16.9 | 4.0 / 17.9 | 5.7 / 11.9 | 0.3 | 398.2 | 5.44 (6/10) | 5.64 (2/10) | 1200 | 1.39 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 4.4 / 15.9 | 4.4 / 15.7 | 7.4 / 12.3 | 0.3 | 398.2 | 4.76 (7/10) | never | 2676 | 1.35 |
| DriveToPose chain | 6.3 / 31.5 | 3.6 / 15.0 | 5.1 / 11.7 | 1.9 | 398.2 | 10.69 (7/10) | never | 434 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 7.5 / 23.5 | 3.8 / 14.3 | 5.2 / 12.5 | 2.3 | 398.2 | 4.46 (8/10) | never | 436 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 7.7 / 25.5 | 4.1 / 16.5 | 5.6 / 11.6 | 0.5 | 398.2 | 4.56 (6/10) | 4.78 (2/10) | 238 | 2.82 |
| Ours v2 (as shipped) | 4.2 / 17.7 | 3.9 / 16.9 | 5.7 / 11.4 | 0.3 | 398.2 | 4.91 (6/10) | 5.05 (2/10) | 611 | 0.38 |
| Ours v2 (as shipped, adaptive grip off) | 4.2 / 17.7 | 3.9 / 16.9 | 5.7 / 11.4 | 0.3 | 398.2 | 4.91 (6/10) | 5.03 (2/10) | 648 | 0.42 |
| Ours v2 (tuned) | 4.2 / 17.7 | 3.9 / 16.9 | 5.7 / 11.4 | 0.3 | 398.2 | 4.91 (6/10) | 5.05 (2/10) | 611 | 0.44 |
| PathPlanner path2 (new-path, pre-release), as shipped | 14.1 / 39.3 | 17.0 / 50.9 | 7.8 / 12.8 | 0.6 | 18.9 | 4.26 (7/10) | never | 9102 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 15.6 / 32.5 | 15.0 / 32.5 | 20.6 / 26.2 | 0.3 | 398.2 | never | never | 6029 | 0.02 |

**fast_long**: an 8 m gentle curve with a 5 m/s cap (above the robot's 4.57 m/s free speed) (10.8 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 1.9 / 5.7 | 1.9 / 5.7 | 7.1 / 17.4 | 0.1 | 17.0 | 2.65 (9/10) | 2.81 (2/10) | 86 | 56.72 |
| BLine (README gains, r 0.30) | 2.2 / 5.5 | 1.0 / 3.9 | 40.9 / 54.6 | 0.1 | 54.5 | 2.41 | never | 1050 | 0.03 |
| BLine (README gains, r 0.45 default) | 2.8 / 6.5 | 1.2 / 5.5 | 41.1 / 53.7 | 0.1 | 53.6 | 2.41 | never | 1046 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 2.8 / 6.5 | 1.2 / 5.5 | 41.1 / 53.7 | 0.1 | 53.6 | 2.41 | never | 1046 | 0.02 |
| BLine (tuned) | 3.8 / 9.2 | 2.5 / 8.9 | 6.5 / 16.8 | 0.1 | 12.8 | 3.38 (8/10) | 3.92 (1/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 2.2 / 6.1 | 1.4 / 4.6 | 6.5 / 16.8 | 0.1 | 12.8 | 3.38 (8/10) | 3.93 (1/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 1.1 / 4.2 | 1.1 / 4.2 | 7.0 / 17.0 | 0.1 | 16.8 | 2.55 (9/10) | 2.76 (3/10) | 0 | 2.80 |
| PathPlanner (kP 5) + hold | 1.1 / 4.2 | 1.1 / 4.2 | 6.4 / 16.0 | 0.1 | 16.8 | 2.55 (9/10) | 2.99 (3/10) | 0 | 2.96 |
| PathPlanner (tuned) + hold | 1.2 / 4.2 | 1.2 / 4.2 | 6.2 / 15.2 | 0.1 | 14.8 | 2.68 (8/10) | 2.86 (3/10) | 0 | 2.72 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.1 / 4.2 | 1.1 / 4.2 | 6.4 / 16.0 | 0.1 | 16.8 | 2.55 (9/10) | 2.99 (3/10) | 0 | 2.79 |
| DriveToPose chain | 2.2 / 6.4 | 1.1 / 4.0 | 6.2 / 16.0 | 0.0 | 13.7 | 4.87 (8/10) | 5.03 (3/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 1.4 / 4.3 | 1.4 / 4.4 | 6.7 / 17.2 | 0.0 | 17.6 | 2.37 (8/10) | 2.61 (4/10) | 316 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.6 / 4.3 | 1.6 / 5.0 | 6.4 / 16.1 | 0.0 | 13.4 | 2.59 (8/10) | 2.86 (3/10) | 24 | 1.66 |
| Ours v2 (as shipped) | 1.2 / 4.2 | 1.1 / 4.2 | 6.4 / 16.0 | 0.1 | 13.6 | 2.41 (8/10) | 2.55 (3/10) | 16 | 0.40 |
| Ours v2 (as shipped, adaptive grip off) | 1.2 / 4.2 | 1.1 / 4.2 | 6.4 / 16.0 | 0.1 | 13.6 | 2.41 (8/10) | 2.55 (3/10) | 16 | 0.39 |
| Ours v2 (tuned) | 1.2 / 4.2 | 1.1 / 4.2 | 6.4 / 16.0 | 0.1 | 13.6 | 2.41 (8/10) | 2.55 (3/10) | 16 | 0.39 |
| PathPlanner path2 (new-path, pre-release), as shipped | 3.3 / 8.3 | 2.7 / 6.7 | 11.3 / 24.6 | 0.1 | 0.6 | 2.37 (1/10) | 2.48 (1/10) | 3064 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 2.0 / 6.3 | 2.3 / 7.2 | 6.9 / 17.8 | 0.1 | 11.6 | 3.42 (8/10) | never | 1506 | 0.02 |

**heavy_robot**: the S-curve with a 68 kg robot (MOI 6.8) and a 96 A current limit (20 % lower) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.5 / 12.1 | 4.5 / 12.1 | 5.9 / 13.2 | 0.1 | 12.9 | 2.25 (9/10) | never | 174 | 36.97 |
| BLine (README gains, r 0.30) | 13.2 / 36.9 | 14.3 / 36.9 | 7.0 / 15.6 | 0.1 | 97.4 | 3.01 (8/10) | never | 360 | 0.02 |
| BLine (README gains, r 0.45 default) | 8.1 / 26.5 | 8.9 / 26.5 | 6.2 / 13.8 | 0.1 | 92.3 | 2.93 (9/10) | never | 294 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.3 / 10.4 | 1.7 / 7.7 | 13.1 / 22.2 | 0.1 | 22.0 | 2.22 | never | 539 | 0.02 |
| BLine (tuned) | 6.6 / 16.6 | 4.4 / 15.8 | 4.2 / 11.2 | 0.1 | 8.1 | 2.60 (8/10) | 3.14 (3/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 2.9 / 7.2 | 3.1 / 10.9 | 4.2 / 11.1 | 0.1 | 8.1 | 2.70 (8/10) | 3.25 (3/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 1.7 / 5.7 | 1.6 / 5.7 | 6.3 / 14.2 | 0.1 | 13.0 | 1.88 (8/10) | 2.09 (1/10) | 1669 | 2.03 |
| PathPlanner (kP 5) + hold | 1.7 / 5.7 | 1.6 / 5.7 | 5.0 / 11.0 | 0.1 | 13.0 | 1.88 (8/10) | 2.36 (3/10) | 1669 | 1.90 |
| PathPlanner (tuned) + hold | 1.3 / 4.5 | 1.2 / 4.5 | 4.1 / 10.6 | 0.1 | 10.4 | 2.20 (9/10) | 2.44 (4/10) | 22 | 2.03 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.3 / 3.8 | 1.2 / 3.7 | 4.2 / 10.0 | 0.1 | 12.7 | 1.91 (9/10) | 2.38 (4/10) | 1291 | 1.88 |
| DriveToPose chain | 2.9 / 8.8 | 0.9 / 3.3 | 4.0 / 10.0 | 0.0 | 9.0 | 3.88 (8/10) | 4.01 (4/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 4.7 / 11.4 | 2.1 / 5.6 | 4.5 / 10.9 | 0.1 | 11.5 | 1.95 (8/10) | 2.18 (4/10) | 1 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.0 / 12.9 | 2.9 / 7.0 | 4.1 / 10.5 | 0.1 | 8.9 | 2.01 (8/10) | 2.26 (4/10) | 5 | 1.54 |
| Ours v2 (as shipped) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.3 | 0.1 | 9.2 | 2.03 (8/10) | 2.17 (4/10) | 0 | 0.36 |
| Ours v2 (as shipped, adaptive grip off) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.3 | 0.1 | 9.2 | 2.03 (8/10) | 2.17 (4/10) | 0 | 0.32 |
| Ours v2 (tuned) | 1.6 / 6.7 | 1.5 / 5.6 | 4.1 / 10.3 | 0.1 | 9.2 | 2.03 (8/10) | 2.17 (4/10) | 0 | 0.33 |
| PathPlanner path2 (new-path, pre-release), as shipped | 5.8 / 14.2 | 5.6 / 14.4 | 11.9 / 14.5 | 0.1 | 13.4 | 1.68 (6/10) | never | 4928 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 5.0 / 12.1 | 2.9 / 10.7 | 4.3 / 9.4 | 0.1 | 8.3 | 2.66 (8/10) | 3.10 (2/10) | 2004 | 0.02 |

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
| Ours v2 (as shipped) | 90/90 | 9/9 | 4.2 | 36.4 | 0.1 | 965 |
| Ours v2 (as shipped, adaptive grip off) | 90/90 | 9/9 | 4.2 | 37.8 | 0.1 | 1965 |
| Ours v2 (tuned) | 90/90 | 9/9 | 4.2 | 36.4 | 0.1 | 965 |
| PathPlanner path2 (new-path, pre-release), as shipped | 0/90 | 0/9 | 14.1 | 73.4 | 6.8 | 7389 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 90/90 | 9/9 | 9.3 | 54.5 | 1.7 | 3985 |

**straight_3m**: 3 m straight, heading held at 0 (6.5 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.0 / 0.1 | 0.0 / 0.0 | 3.4 / 3.5 | 0.0 | 3.5 | 1.35 | never | 83 | 19.26 |
| BLine (README gains, r 0.30) | 0.0 / 0.1 | 0.0 / 0.1 | 41.5 / 42.8 | 0.0 | 42.8 | 1.13 | never | 1059 | 0.02 |
| BLine (README gains, r 0.45 default) | 0.0 / 0.1 | 0.0 / 0.1 | 41.5 / 42.8 | 0.0 | 42.8 | 1.13 | never | 1059 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 0.0 / 0.1 | 0.0 / 0.1 | 41.5 / 42.8 | 0.0 | 42.8 | 1.13 | never | 1059 | 0.02 |
| BLine (tuned) | 0.0 / 0.1 | 0.0 / 0.1 | 0.8 / 0.9 | 0.0 | 0.0 | 2.20 | 2.67 (10/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 0.0 / 0.1 | 0.0 / 0.1 | 0.8 / 0.9 | 0.0 | 0.0 | 2.20 | 2.67 (10/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 0.0 / 0.1 | 0.0 / 0.1 | 3.2 / 3.4 | 0.0 | 3.4 | 1.26 | never | 0 | 1.36 |
| PathPlanner (kP 5) + hold | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.1 | 0.0 | 3.4 | 1.26 | 1.71 (10/10) | 0 | 1.96 |
| PathPlanner (tuned) + hold | 0.0 / 0.1 | 0.0 / 0.0 | 0.4 / 1.1 | 0.0 | 1.3 | 1.42 | 1.67 (10/10) | 0 | 1.64 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.1 | 0.0 | 3.4 | 1.26 | 1.71 (10/10) | 0 | 1.34 |
| DriveToPose chain | 0.1 / 0.1 | 0.0 / 0.1 | 0.1 / 0.1 | 0.0 | 0.0 | 1.34 | 1.55 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 0.3 / 1.0 | 0.1 / 0.3 | 1.5 / 1.7 | 0.0 | 2.5 | 1.26 | 1.48 (10/10) | 30 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 0.3 / 1.0 | 0.1 / 0.3 | 0.1 / 0.1 | 0.0 | 0.0 | 1.33 | 1.58 (10/10) | 37 | 0.92 |
| Ours v2 (as shipped) | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.2 | 0.0 | 0.2 | 1.29 | 1.43 (10/10) | 19 | 0.25 |
| Ours v2 (as shipped, adaptive grip off) | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.2 | 0.0 | 0.2 | 1.29 | 1.43 (10/10) | 19 | 0.26 |
| Ours v2 (tuned) | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.2 | 0.0 | 0.2 | 1.29 | 1.43 (10/10) | 19 | 0.24 |
| PathPlanner path2 (new-path, pre-release), as shipped | 0.0 / 0.1 | 0.0 / 0.1 | 5.0 / 6.2 | 0.0 | 0.0 | 1.15 (4/10) | never | 2816 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 0.0 / 0.1 | 0.0 / 0.1 | 1.7 / 1.8 | 0.0 | 0.0 | 2.20 | 2.52 (10/10) | 0 | 0.02 |

**s_curve**: S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 9.1 | 4.6 / 9.1 | 3.3 / 3.4 | 0.0 | 3.4 | 2.24 | never | 254 | 37.25 |
| BLine (README gains, r 0.30) | 12.8 / 37.6 | 14.0 / 37.7 | 5.7 / 6.3 | 0.0 | 89.9 | 2.99 | never | 365 | 0.02 |
| BLine (README gains, r 0.45 default) | 8.4 / 26.6 | 9.4 / 26.6 | 4.2 / 4.5 | 0.0 | 84.9 | 2.93 | never | 299 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.1 / 7.5 | 1.4 / 6.2 | 11.6 / 12.6 | 0.0 | 12.6 | 2.22 | never | 528 | 0.02 |
| BLine (tuned) | 6.8 / 15.6 | 4.4 / 15.0 | 0.8 / 0.9 | 0.0 | 0.0 | 2.70 | 3.17 (10/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 2.5 / 5.9 | 3.0 / 9.5 | 0.8 / 0.9 | 0.0 | 0.0 | 2.79 | 3.25 (10/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 0.8 / 2.5 | 0.8 / 2.6 | 3.3 / 3.5 | 0.0 | 3.6 | 1.89 | never | 1938 | 1.87 |
| PathPlanner (kP 5) + hold | 0.8 / 2.5 | 0.8 / 2.6 | 0.1 / 0.1 | 0.0 | 3.6 | 1.89 | 2.34 (10/10) | 1938 | 1.82 |
| PathPlanner (tuned) + hold | 0.5 / 1.1 | 0.4 / 1.1 | 0.4 / 1.2 | 0.0 | 1.3 | 2.21 | 2.46 (10/10) | 0 | 1.85 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.7 / 1.8 | 0.6 / 1.8 | 0.1 / 0.1 | 0.0 | 3.5 | 1.92 | 2.36 (10/10) | 1544 | 1.84 |
| DriveToPose chain | 2.8 / 6.0 | 0.2 / 0.4 | 0.1 / 0.1 | 0.0 | 0.0 | 4.08 | 4.28 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 4.6 / 8.6 | 1.2 / 1.9 | 1.5 / 1.8 | 0.0 | 2.4 | 1.95 | 2.17 (10/10) | 48 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.1 / 10.3 | 2.0 / 3.7 | 0.1 / 0.1 | 0.0 | 0.0 | 2.02 | 2.28 (10/10) | 26 | 1.36 |
| Ours v2 (as shipped) | 1.2 / 3.0 | 1.0 / 2.0 | 0.0 / 0.1 | 0.0 | 0.1 | 2.04 | 2.18 (10/10) | 19 | 0.32 |
| Ours v2 (as shipped, adaptive grip off) | 1.2 / 3.0 | 1.0 / 2.0 | 0.0 / 0.1 | 0.0 | 0.1 | 2.04 | 2.18 (10/10) | 19 | 0.33 |
| Ours v2 (tuned) | 1.2 / 3.0 | 1.0 / 2.0 | 0.0 / 0.1 | 0.0 | 0.1 | 2.04 | 2.18 (10/10) | 19 | 0.30 |
| PathPlanner path2 (new-path, pre-release), as shipped | 5.6 / 12.2 | 7.2 / 15.8 | 11.6 / 13.2 | 0.0 | 13.1 | 1.71 (7/10) | never | 5491 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 4.8 / 9.4 | 2.3 / 8.6 | 1.7 / 1.8 | 0.0 | 0.0 | 2.79 | 3.11 (10/10) | 2082 | 0.02 |

**hairpin**: 2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held (9.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 7.3 / 13.3 | 7.3 / 13.3 | 3.4 / 3.5 | 0.0 | 3.5 | 2.82 | never | 1429 | 46.44 |
| BLine (README gains, r 0.30) | 45.4 / 98.9 | 45.8 / 102.1 | 9.8 / 11.5 | 0.0 | 11.5 | 6.57 | never | 472 | 0.03 |
| BLine (README gains, r 0.45 default) | 47.2 / 98.7 | 47.4 / 101.8 | 2.3 / 2.4 | 0.0 | 54.7 | 5.40 | never | 0 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 47.2 / 98.7 | 47.4 / 101.8 | 2.3 / 2.4 | 0.0 | 54.7 | 5.40 | never | 0 | 0.03 |
| BLine (tuned) | 40.9 / 96.8 | 41.8 / 99.3 | 0.8 / 0.9 | 0.0 | 0.0 | 4.06 | 4.53 (10/10) | 0 | 0.03 |
| BLine (tuned, with corner caps) | 41.0 / 98.0 | 42.0 / 101.2 | 0.8 / 0.9 | 0.0 | 0.0 | 3.89 | 4.36 (10/10) | 0 | 0.03 |
| PathPlanner (kP 5) | 18.7 / 40.8 | 18.8 / 40.9 | 21.2 / 22.1 | 0.0 | 21.9 | 2.44 | never | 6552 | 2.33 |
| PathPlanner (kP 5) + hold | 18.7 / 40.8 | 18.8 / 40.9 | 0.1 / 0.1 | 0.0 | 21.6 | 2.44 | 3.26 (10/10) | 6950 | 2.35 |
| PathPlanner (tuned) + hold | 0.9 / 1.5 | 0.7 / 1.5 | 0.9 / 1.2 | 0.0 | 1.3 | 3.12 | 3.30 (10/10) | 29 | 2.31 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.8 / 1.9 | 0.7 / 1.8 | 0.1 / 0.1 | 0.0 | 3.4 | 2.57 | 3.02 (10/10) | 3560 | 2.45 |
| DriveToPose chain | 1.2 / 3.2 | 0.1 / 0.3 | 0.1 / 0.1 | 0.0 | 0.0 | 7.74 | 7.95 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 1.4 / 2.4 | 1.2 / 2.0 | 1.4 / 1.6 | 0.0 | 2.5 | 2.84 | 3.07 (10/10) | 138 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 2.5 / 4.2 | 2.3 / 3.9 | 0.1 / 0.1 | 0.1 | 0.0 | 2.93 | 3.19 (10/10) | 35 | 1.88 |
| Ours v2 (as shipped) | 1.5 / 3.0 | 1.4 / 2.5 | 0.1 / 0.1 | 0.0 | 0.1 | 2.83 | 2.97 (10/10) | 19 | 0.44 |
| Ours v2 (as shipped, adaptive grip off) | 1.5 / 3.0 | 1.4 / 2.5 | 0.1 / 0.1 | 0.0 | 0.1 | 2.83 | 2.97 (10/10) | 19 | 0.38 |
| Ours v2 (tuned) | 1.5 / 3.0 | 1.4 / 2.5 | 0.1 / 0.1 | 0.0 | 0.1 | 2.83 | 2.97 (10/10) | 19 | 0.39 |
| PathPlanner path2 (new-path, pre-release), as shipped | 15.8 / 48.2 | 16.6 / 49.9 | 4.8 / 6.2 | 0.0 | 0.0 | 2.59 (5/10) | never | 7979 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 20.6 / 54.5 | 21.5 / 57.7 | 1.7 / 1.8 | 0.0 | 0.0 | 3.60 | 3.92 (10/10) | 7334 | 0.02 |

**slalom**: through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m (10.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.5 / 10.2 | 5.5 / 10.2 | 3.3 / 3.5 | 0.0 | 3.5 | 3.08 | 3.26 (1/10) | 168 | 50.13 |
| BLine (README gains, r 0.30) | 40.3 / 83.4 | 40.9 / 80.8 | 13.3 / 31.0 | 0.0 | 40.9 | 4.29 | 4.57 (3/10) | 362 | 0.02 |
| BLine (README gains, r 0.45 default) | 30.0 / 67.8 | 30.0 / 63.2 | 29.0 / 30.6 | 0.0 | 28.9 | 3.87 | never | 854 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 6.4 / 11.9 | 2.3 / 9.0 | 8.1 / 8.6 | 0.0 | 8.6 | 3.30 | never | 432 | 0.02 |
| BLine (tuned) | 11.0 / 28.7 | 7.5 / 25.8 | 0.8 / 0.9 | 0.0 | 0.0 | 3.19 | 3.66 (10/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 4.2 / 9.5 | 2.0 / 8.5 | 0.8 / 0.9 | 0.0 | 0.0 | 3.87 | 4.33 (10/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 5.3 / 12.3 | 5.3 / 12.4 | 3.4 / 3.6 | 0.0 | 3.6 | 2.47 | never | 5242 | 2.49 |
| PathPlanner (kP 5) + hold | 5.3 / 12.3 | 5.3 / 12.4 | 0.1 / 0.1 | 0.0 | 3.6 | 2.47 | 2.93 (10/10) | 5242 | 2.49 |
| PathPlanner (tuned) + hold | 0.7 / 1.6 | 0.5 / 1.5 | 0.6 / 1.2 | 0.0 | 1.3 | 3.10 | 3.32 (10/10) | 328 | 2.48 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.0 / 3.1 | 1.1 / 3.4 | 0.1 / 0.1 | 0.0 | 3.4 | 2.57 | 3.02 (10/10) | 4203 | 2.57 |
| DriveToPose chain | 4.2 / 8.7 | 0.3 / 0.7 | 0.1 / 0.1 | 0.0 | 0.0 | 4.74 | 4.94 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 8.4 / 17.0 | 1.4 / 2.0 | 1.5 / 1.5 | 0.0 | 2.2 | 2.77 | 2.99 (10/10) | 261 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 9.0 / 18.4 | 2.4 / 3.8 | 0.1 / 0.1 | 0.2 | 0.0 | 2.85 | 3.10 (10/10) | 16 | 1.82 |
| Ours v2 (as shipped) | 1.4 / 3.0 | 1.3 / 2.2 | 0.1 / 0.1 | 0.0 | 0.1 | 2.80 | 2.94 (10/10) | 21 | 0.34 |
| Ours v2 (as shipped, adaptive grip off) | 1.4 / 3.0 | 1.3 / 2.2 | 0.1 / 0.1 | 0.0 | 0.1 | 2.80 | 2.94 (10/10) | 21 | 0.34 |
| Ours v2 (tuned) | 1.4 / 3.0 | 1.3 / 2.2 | 0.1 / 0.1 | 0.0 | 0.1 | 2.80 | 2.94 (10/10) | 21 | 0.34 |
| PathPlanner path2 (new-path, pre-release), as shipped | 16.8 / 69.3 | 18.3 / 77.7 | 7.5 / 7.8 | 0.0 | 0.0 | never | never | 9650 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 7.2 / 16.6 | 3.9 / 14.5 | 1.7 / 1.8 | 0.0 | 0.0 | 3.25 | 3.57 (10/10) | 2882 | 0.02 |

**long_auto**: auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end (14.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.0 / 10.8 | 5.0 / 10.8 | 3.0 / 3.2 | 2.3 | 3.2 | 4.11 | never | 466 | 80.58 |
| BLine (README gains, r 0.30) | 32.2 / 86.2 | 36.5 / 100.5 | 1.8 / 2.4 | 0.4 | 64.0 | 5.55 | 6.65 (7/10) | 630 | 0.02 |
| BLine (README gains, r 0.45 default) | 31.2 / 76.8 | 34.3 / 89.8 | 2.2 / 2.4 | 0.5 | 51.4 | 5.10 | never | 206 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 10.2 / 24.7 | 2.4 / 11.9 | 2.5 / 2.9 | 1.8 | 4.1 | 4.94 | never | 454 | 0.02 |
| BLine (tuned) | 23.1 / 56.0 | 25.7 / 65.2 | 0.9 / 0.9 | 1.7 | 0.0 | 3.93 | 4.46 (5/10) | 50 | 0.02 |
| BLine (tuned, with corner caps) | 8.6 / 22.4 | 2.2 / 13.4 | 0.8 / 0.9 | 0.4 | 0.0 | 5.24 | 5.70 (10/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 1.8 / 5.2 | 1.9 / 5.4 | 3.1 / 3.3 | 0.0 | 3.3 | 3.63 | never | 2550 | 4.28 |
| PathPlanner (kP 5) + hold | 1.8 / 5.2 | 1.9 / 5.4 | 0.1 / 0.1 | 0.0 | 3.3 | 3.63 | 4.07 (10/10) | 2550 | 4.23 |
| PathPlanner (tuned) + hold | 0.7 / 1.6 | 0.6 / 1.6 | 0.6 / 1.1 | 0.0 | 1.2 | 4.16 | 4.38 (10/10) | 0 | 4.32 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.3 / 2.9 | 1.3 / 2.9 | 0.1 / 0.1 | 0.0 | 3.4 | 3.70 | 4.14 (10/10) | 1817 | 4.42 |
| DriveToPose chain | 6.9 / 21.3 | 1.1 / 3.3 | 0.1 / 0.4 | 0.2 | 0.4 | 6.40 | 6.55 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 7.3 / 21.8 | 1.5 / 2.9 | 1.5 / 1.7 | 1.1 | 2.3 | 3.75 | 3.98 (7/10) | 214 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.0 / 22.1 | 2.8 / 5.7 | 0.1 / 0.2 | 0.3 | 0.0 | 3.84 | 4.10 (10/10) | 179 | 2.38 |
| Ours v2 (as shipped) | 4.6 / 12.4 | 1.6 / 4.2 | 0.1 / 0.1 | 0.3 | 0.1 | 3.86 | 4.00 (10/10) | 19 | 0.39 |
| Ours v2 (as shipped, adaptive grip off) | 4.6 / 12.4 | 1.6 / 4.2 | 0.1 / 0.1 | 0.3 | 0.1 | 3.86 | 4.00 (10/10) | 19 | 0.45 |
| Ours v2 (tuned) | 4.6 / 12.4 | 1.6 / 4.2 | 0.1 / 0.1 | 0.3 | 0.1 | 3.86 | 4.00 (10/10) | 19 | 0.42 |
| PathPlanner path2 (new-path, pre-release), as shipped | 8.1 / 38.9 | 12.0 / 60.5 | 5.5 / 6.4 | 0.0 | 0.0 | 3.87 (2/10) | never | 5071 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 7.8 / 23.1 | 11.1 / 41.9 | 1.7 / 1.8 | 0.0 | 0.0 | 4.66 | 4.98 (10/10) | 3239 | 0.02 |

**moving_start**: starts moving 2 m/s sideways (+y), path heads +x then curves left (7.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 12.0 / 27.7 | 12.0 / 27.7 | 3.4 / 3.5 | 0.0 | 3.5 | 1.90 | never | 1584 | 31.09 |
| BLine (README gains, r 0.30) | 31.3 / 58.4 | 32.4 / 66.0 | 28.5 / 30.3 | 0.0 | 30.1 | 3.48 | never | 866 | 0.02 |
| BLine (README gains, r 0.45 default) | 33.4 / 54.6 | 34.5 / 51.3 | 18.5 / 36.2 | 0.0 | 54.3 | 2.08 | 3.06 (5/10) | 514 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 24.6 / 49.4 | 25.3 / 47.0 | 11.0 / 11.8 | 0.0 | 11.8 | 2.18 | never | 504 | 0.02 |
| BLine (tuned) | 35.1 / 55.9 | 33.9 / 51.9 | 0.8 / 0.8 | 0.0 | 4.3 | 2.06 | 2.44 (10/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 20.2 / 47.5 | 21.1 / 45.5 | 0.8 / 0.9 | 0.0 | 0.0 | 3.10 | 3.57 (10/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 20.0 / 33.5 | 20.0 / 33.5 | 3.2 / 3.4 | 0.0 | 3.5 | 1.44 | never | 4275 | 1.62 |
| PathPlanner (kP 5) + hold | 20.0 / 33.5 | 20.0 / 33.5 | 0.1 / 0.1 | 0.0 | 3.5 | 1.44 | 1.89 (10/10) | 4275 | 1.65 |
| PathPlanner (tuned) + hold | 14.5 / 31.3 | 14.5 / 31.3 | 0.3 / 1.1 | 0.0 | 1.3 | 1.65 | 1.92 (10/10) | 4646 | 1.62 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 20.0 / 33.5 | 20.0 / 33.5 | 0.1 / 0.1 | 0.0 | 3.5 | 1.45 | 1.89 (10/10) | 4224 | 1.63 |
| DriveToPose chain | 21.5 / 58.6 | 19.9 / 54.7 | 0.1 / 0.1 | 0.0 | 0.0 | 3.70 | 3.90 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 10.6 / 24.7 | 10.5 / 25.6 | 1.5 / 1.8 | 0.0 | 2.4 | 1.72 | 1.93 (10/10) | 4562 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 11.7 / 28.1 | 11.7 / 29.5 | 0.1 / 0.1 | 0.3 | 0.0 | 1.85 | 2.11 (10/10) | 1957 | 1.20 |
| Ours v2 (as shipped) | 11.3 / 28.6 | 11.4 / 28.6 | 0.1 / 0.2 | 0.0 | 0.2 | 1.85 | 2.00 (10/10) | 1530 | 0.30 |
| Ours v2 (as shipped, adaptive grip off) | 11.1 / 28.1 | 11.2 / 28.1 | 0.1 / 0.1 | 0.0 | 0.1 | 1.82 | 1.96 (10/10) | 1822 | 0.29 |
| Ours v2 (tuned) | 11.3 / 28.6 | 11.4 / 28.6 | 0.1 / 0.2 | 0.0 | 0.2 | 1.85 | 2.00 (10/10) | 1530 | 0.29 |
| PathPlanner path2 (new-path, pre-release), as shipped | 18.9 / 38.2 | 20.2 / 36.0 | 4.2 / 5.2 | 0.0 | 0.0 | 1.71 (9/10) | never | 5734 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 14.2 / 38.1 | 13.3 / 36.0 | 1.7 / 1.8 | 0.0 | 0.0 | 2.52 | 2.84 (10/10) | 4107 | 0.02 |

**hit_mid_path**: the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 14.2 / 37.4 | 14.2 / 37.4 | 1.6 / 2.0 | 0.0 | 1.8 | 2.31 | 2.49 (10/10) | 3178 | 35.47 |
| BLine (README gains, r 0.30) | 34.5 / 74.5 | 36.4 / 78.1 | 37.1 / 42.6 | 0.0 | 67.3 | 4.32 | 5.46 (1/10) | 4037 | 0.02 |
| BLine (README gains, r 0.45 default) | 25.1 / 63.0 | 26.9 / 66.0 | 2.1 / 2.3 | 0.0 | 80.8 | 3.12 | 3.73 (1/10) | 3318 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 6.6 / 17.9 | 7.7 / 22.5 | 12.0 / 12.8 | 0.0 | 12.7 | 2.28 | never | 2938 | 0.02 |
| BLine (tuned) | 17.5 / 38.2 | 18.9 / 43.4 | 0.8 / 0.8 | 0.0 | 1.6 | 2.08 | 2.30 (10/10) | 2978 | 0.02 |
| BLine (tuned, with corner caps) | 9.1 / 22.2 | 10.8 / 26.2 | 0.8 / 0.9 | 0.0 | 0.0 | 2.39 | 2.74 (10/10) | 2848 | 0.02 |
| PathPlanner (kP 5) | 22.4 / 53.4 | 22.5 / 53.4 | 32.0 / 32.5 | 0.0 | 32.6 | never | never | 5517 | 1.86 |
| PathPlanner (kP 5) + hold | 22.4 / 53.4 | 22.5 / 53.4 | 0.1 / 0.1 | 0.0 | 31.1 | 2.49 | 2.79 (10/10) | 5856 | 1.89 |
| PathPlanner (tuned) + hold | 16.6 / 40.1 | 16.7 / 40.2 | 0.2 / 0.2 | 0.0 | 11.6 | 2.53 | 2.67 (10/10) | 5157 | 1.79 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 21.5 / 50.0 | 21.5 / 50.0 | 0.1 / 0.1 | 0.0 | 24.4 | 2.44 | 2.75 (10/10) | 5578 | 1.88 |
| DriveToPose chain | 8.2 / 29.2 | 8.8 / 31.5 | 0.1 / 0.1 | 0.0 | 0.0 | 4.18 | 4.39 (10/10) | 2293 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 49.1 / 97.6 | 47.6 / 95.3 | 39.0 / 41.3 | 0.2 | 24.1 | never | never | 5688 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 48.3 / 103.4 | 46.8 / 99.3 | 0.1 / 0.1 | 0.2 | 62.6 | 2.94 | 3.20 (10/10) | 7141 | 1.28 |
| Ours v2 (as shipped) | 15.1 / 36.4 | 15.4 / 37.5 | 0.1 / 0.3 | 0.0 | 0.2 | 2.08 | 2.22 (10/10) | 3770 | 0.30 |
| Ours v2 (as shipped, adaptive grip off) | 16.0 / 37.8 | 16.3 / 38.9 | 0.1 / 0.1 | 0.0 | 0.1 | 2.08 | 2.22 (10/10) | 3675 | 0.30 |
| Ours v2 (tuned) | 15.1 / 36.4 | 15.4 / 37.5 | 0.1 / 0.3 | 0.0 | 0.2 | 2.08 | 2.22 (10/10) | 3770 | 0.31 |
| PathPlanner path2 (new-path, pre-release), as shipped | 34.5 / 73.4 | 36.6 / 78.9 | 7.6 / 8.3 | 0.0 | 11.3 | never | never | 7691 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 20.4 / 52.1 | 22.2 / 57.7 | 1.7 / 1.8 | 0.0 | 0.0 | 2.93 | 3.25 (10/10) | 4939 | 0.02 |

**low_grip_s_curve**: the S-curve on mu 0.70 static / 0.61 kinetic (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.5 / 8.8 | 4.5 / 8.8 | 10.7 / 10.9 | 0.0 | 10.9 | 2.23 | never | 4717 | 35.58 |
| BLine (README gains, r 0.30) | 14.0 / 39.9 | 15.2 / 39.9 | 18.5 / 19.3 | 0.0 | 96.5 | 3.06 | never | 11706 | 0.02 |
| BLine (README gains, r 0.45 default) | 9.3 / 28.6 | 10.3 / 28.6 | 15.1 / 15.6 | 0.0 | 91.7 | 2.99 | never | 11048 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.8 / 7.0 | 1.2 / 5.9 | 22.5 / 24.4 | 0.0 | 24.4 | 2.23 | never | 5354 | 0.02 |
| BLine (tuned) | 6.5 / 15.5 | 4.3 / 14.8 | 0.8 / 0.8 | 0.0 | 0.0 | 2.66 | 3.13 (10/10) | 5515 | 0.02 |
| BLine (tuned, with corner caps) | 2.7 / 6.1 | 3.5 / 10.9 | 0.8 / 0.8 | 0.0 | 0.0 | 2.70 | 3.16 (10/10) | 6779 | 0.02 |
| PathPlanner (kP 5) | 13.5 / 36.8 | 13.6 / 36.8 | 88.0 / 88.5 | 0.0 | 84.8 | never | never | 8528 | 1.82 |
| PathPlanner (kP 5) + hold | 13.5 / 36.8 | 13.6 / 36.8 | 0.1 / 0.1 | 0.0 | 83.5 | 2.91 | 3.35 (10/10) | 11646 | 1.85 |
| PathPlanner (tuned) + hold | 1.0 / 3.3 | 1.1 / 3.5 | 0.1 / 0.2 | 0.0 | 2.7 | 2.18 | 2.52 (10/10) | 7259 | 1.86 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 12.8 / 33.3 | 12.8 / 33.3 | 0.1 / 0.1 | 0.0 | 61.7 | 2.84 | 3.14 (10/10) | 10456 | 1.81 |
| DriveToPose chain | 2.7 / 6.0 | 0.6 / 1.4 | 1.6 / 1.7 | 0.0 | 1.7 | 3.80 | 3.94 (10/10) | 13094 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 24.3 / 68.9 | 22.6 / 66.4 | 113.8 / 114.1 | 0.3 | 103.7 | never | never | 9624 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 10.2 / 22.2 | 7.9 / 17.8 | 0.1 / 0.1 | 0.2 | 32.8 | 2.66 | 2.92 (10/10) | 10082 | 1.29 |
| Ours v2 (as shipped) | 1.3 / 3.3 | 1.1 / 2.3 | 0.1 / 0.2 | 0.0 | 0.2 | 2.54 | 2.68 (10/10) | 296 | 0.30 |
| Ours v2 (as shipped, adaptive grip off) | 1.0 / 2.2 | 1.1 / 3.0 | 0.1 / 0.2 | 0.0 | 6.7 | 2.04 | 2.46 (10/10) | 8034 | 0.29 |
| Ours v2 (tuned) | 1.3 / 3.3 | 1.1 / 2.3 | 0.1 / 0.2 | 0.0 | 0.2 | 2.54 | 2.68 (10/10) | 296 | 0.31 |
| PathPlanner path2 (new-path, pre-release), as shipped | 17.6 / 45.0 | 18.8 / 45.0 | 7.8 / 8.1 | 0.0 | 80.1 | never | never | 11842 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 3.9 / 8.4 | 4.6 / 11.7 | 1.8 / 1.8 | 0.0 | 0.0 | 2.59 | 2.90 (10/10) | 7114 | 0.02 |

**grip_drop_s_curve**: the S-curve; halfway along, the floor drops from mu 1.15 / 1.0 to 0.70 / 0.61 (a worn carpet seam) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 8.8 | 4.6 / 8.8 | 10.7 / 10.9 | 0.0 | 10.9 | 2.22 | never | 3275 | 36.12 |
| BLine (README gains, r 0.30) | 13.1 / 39.1 | 14.3 / 39.1 | 18.1 / 18.9 | 0.0 | 96.3 | 3.03 | never | 7408 | 0.02 |
| BLine (README gains, r 0.45 default) | 8.7 / 28.0 | 9.7 / 28.0 | 15.0 / 15.4 | 0.0 | 91.4 | 2.97 | never | 7174 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.9 / 6.8 | 1.3 / 6.2 | 22.7 / 24.1 | 0.0 | 24.1 | 2.21 | never | 2656 | 0.02 |
| BLine (tuned) | 6.7 / 15.4 | 4.4 / 14.7 | 0.8 / 0.8 | 0.0 | 0.0 | 2.65 | 3.11 (10/10) | 2245 | 0.02 |
| BLine (tuned, with corner caps) | 2.7 / 5.9 | 3.4 / 10.7 | 0.8 / 0.9 | 0.0 | 0.0 | 2.68 | 3.14 (10/10) | 2805 | 0.02 |
| PathPlanner (kP 5) | 6.4 / 19.9 | 6.4 / 19.8 | 52.3 / 52.7 | 0.0 | 52.6 | never | never | 4563 | 1.85 |
| PathPlanner (kP 5) + hold | 6.4 / 19.9 | 6.4 / 19.8 | 0.1 / 0.1 | 0.0 | 51.2 | 2.74 | 3.04 (10/10) | 6232 | 1.79 |
| PathPlanner (tuned) + hold | 1.0 / 3.1 | 1.0 / 3.3 | 0.1 / 0.2 | 0.0 | 2.4 | 2.18 | 2.52 (10/10) | 3758 | 1.84 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 5.9 / 16.8 | 5.9 / 16.8 | 0.1 / 0.1 | 0.0 | 37.9 | 2.67 | 2.97 (10/10) | 5299 | 1.87 |
| DriveToPose chain | 2.7 / 6.0 | 0.4 / 1.4 | 1.5 / 1.6 | 0.0 | 1.6 | 3.98 | 4.12 (10/10) | 6486 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 8.5 / 18.7 | 6.1 / 14.4 | 42.5 / 43.1 | 0.2 | 41.8 | never | never | 4576 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.3 / 17.0 | 5.7 / 12.5 | 0.1 / 0.1 | 0.1 | 25.2 | 2.53 | 2.78 (10/10) | 5131 | 1.28 |
| Ours v2 (as shipped) | 1.2 / 4.3 | 1.4 / 5.4 | 0.1 / 0.3 | 0.0 | 0.4 | 2.08 | 2.25 (10/10) | 2995 | 0.31 |
| Ours v2 (as shipped, adaptive grip off) | 0.9 / 1.8 | 1.1 / 2.8 | 0.1 / 0.2 | 0.0 | 6.2 | 1.99 | 2.39 (10/10) | 4059 | 0.30 |
| Ours v2 (tuned) | 1.2 / 4.3 | 1.4 / 5.4 | 0.1 / 0.3 | 0.0 | 0.4 | 2.08 | 2.25 (10/10) | 2995 | 0.32 |
| PathPlanner path2 (new-path, pre-release), as shipped | 9.3 / 29.2 | 10.8 / 29.2 | 7.6 / 7.8 | 0.0 | 75.5 | never | never | 10227 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 4.7 / 9.0 | 4.5 / 12.0 | 1.7 / 1.8 | 0.0 | 0.0 | 2.52 | 2.84 (10/10) | 4170 | 0.02 |

**spin_while_driving**: 5 m straight while the robot turns a full 360 deg (a heading target at every waypoint) (8.2 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 1.9 / 3.3 | 1.9 / 3.3 | 2.9 / 3.0 | 14.4 | 3.0 | 2.20 | never | 688 | 32.38 |
| BLine (README gains, r 0.30) | 2.8 / 5.1 | 2.8 / 5.1 | 2.3 / 2.5 | 1.8 | 56.0 | 1.84 | never | 361 | 0.02 |
| BLine (README gains, r 0.45 default) | 3.2 / 5.7 | 3.2 / 5.7 | 2.3 / 2.5 | 1.8 | 55.7 | 1.84 | never | 395 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.2 / 5.7 | 3.2 / 5.7 | 2.3 / 2.5 | 1.8 | 55.7 | 1.84 | never | 395 | 0.02 |
| BLine (tuned) | 3.3 / 7.1 | 3.3 / 7.1 | 0.9 / 0.9 | 1.9 | 0.0 | 2.88 | never | 36 | 0.02 |
| BLine (tuned, with corner caps) | 1.6 / 3.6 | 1.6 / 3.6 | 0.9 / 0.9 | 1.9 | 0.0 | 2.88 | never | 42 | 0.02 |
| PathPlanner (kP 5) | 1.6 / 3.3 | 1.6 / 3.3 | 2.4 / 2.6 | 0.4 | 2.5 | 2.95 | never | 109 | 1.92 |
| PathPlanner (kP 5) + hold | 1.6 / 3.3 | 1.6 / 3.3 | 0.1 / 0.1 | 0.0 | 2.5 | 2.95 | 3.35 (10/10) | 109 | 1.84 |
| PathPlanner (tuned) + hold | 0.6 / 1.4 | 0.6 / 1.4 | 0.5 / 1.0 | 0.5 | 1.0 | 3.49 | 3.69 (10/10) | 13 | 1.87 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.6 / 3.3 | 1.6 / 3.3 | 0.1 / 0.1 | 0.0 | 2.6 | 2.95 | 3.36 (10/10) | 79 | 1.88 |
| DriveToPose chain | 0.9 / 3.4 | 0.9 / 3.4 | 0.2 / 0.2 | 1.7 | 0.6 | 4.90 | never | 306 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 2.7 / 6.0 | 1.8 / 3.5 | 1.7 / 1.9 | 3.4 | 2.2 | 1.95 | never | 744 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.2 / 11.4 | 4.5 / 11.1 | 0.2 / 0.3 | 1.7 | 0.0 | 2.09 | never | 289 | 1.29 |
| Ours v2 (as shipped) | 2.2 / 3.8 | 2.2 / 3.8 | 0.1 / 0.2 | 0.2 | 0.0 | 2.55 | 2.83 (10/10) | 148 | 0.35 |
| Ours v2 (as shipped, adaptive grip off) | 2.2 / 3.8 | 2.2 / 3.8 | 0.1 / 0.2 | 0.2 | 0.0 | 2.55 | 2.83 (10/10) | 148 | 0.37 |
| Ours v2 (tuned) | 2.2 / 3.8 | 2.2 / 3.8 | 0.1 / 0.2 | 0.2 | 0.0 | 2.55 | 2.83 (10/10) | 148 | 0.39 |
| PathPlanner path2 (new-path, pre-release), as shipped | 1.2 / 3.5 | 1.2 / 3.5 | 1.7 / 1.8 | 1.6 | 0.0 | 2.09 | never | 2108 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 2.9 / 7.8 | 2.9 / 7.8 | 1.7 / 1.8 | 0.0 | 0.0 | 2.88 | 3.20 (10/10) | 1919 | 0.02 |

**figure_eight**: a self-crossing loop (a lemniscate, 4 m x 1.8 m) through the origin twice; heading held (12.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.4 / 9.6 | 5.4 / 9.6 | 2.8 / 2.9 | 0.0 | 2.9 | 5.10 | never | 720 | 70.10 |
| BLine (README gains, r 0.30) | 43.5 / 112.9 | 43.9 / 113.0 | 2.2 / 2.4 | 0.0 | 60.7 | 9.90 | never | 0 | 0.04 |
| BLine (README gains, r 0.45 default) | 50.6 / 109.8 | 51.5 / 112.1 | 2.2 / 2.4 | 0.0 | 69.0 | 6.53 | never | 0 | 0.04 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 40.6 / 88.9 | 41.7 / 92.2 | 2.1 / 2.7 | 0.0 | 49.6 | 6.07 | 6.18 (5/10) | 98 | 0.04 |
| BLine (tuned) | 42.4 / 96.5 | 43.1 / 99.0 | 0.8 / 0.8 | 0.0 | 0.0 | 5.00 | 5.32 (10/10) | 0 | 0.03 |
| BLine (tuned, with corner caps) | 43.8 / 92.4 | 44.8 / 94.8 | 0.8 / 0.9 | 0.0 | 0.0 | 5.89 | 6.32 (10/10) | 0 | 0.03 |
| PathPlanner (kP 5) | 23.0 / 56.0 | 23.0 / 56.1 | 43.7 / 44.6 | 0.0 | 20.2 | never | never | 10278 | 3.98 |
| PathPlanner (kP 5) + hold | 23.0 / 56.0 | 23.0 / 56.1 | 0.1 / 0.1 | 0.0 | 21.7 | 4.14 | 4.44 (10/10) | 10840 | 4.01 |
| PathPlanner (tuned) + hold | 0.8 / 1.8 | 0.6 / 1.7 | 0.4 / 1.0 | 0.0 | 1.3 | 4.44 | 4.69 (10/10) | 56 | 4.00 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 9.3 / 24.3 | 9.4 / 24.3 | 0.1 / 0.1 | 0.0 | 5.9 | 3.92 | 4.23 (10/10) | 10030 | 4.05 |
| DriveToPose chain | 1.3 / 3.9 | 0.1 / 0.4 | 36.7 / 39.1 | 0.0 | 80.6 | never | never | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 1.8 / 5.4 | 1.3 / 2.0 | 1.6 / 1.8 | 0.0 | 2.2 | 4.29 | 4.50 (10/10) | 32 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 2.8 / 5.9 | 2.4 / 3.8 | 0.1 / 0.1 | 0.2 | 0.0 | 4.41 | 4.66 (10/10) | 437 | 2.97 |
| Ours v2 (as shipped) | 1.3 / 2.8 | 1.3 / 2.2 | 0.0 / 0.1 | 0.0 | 0.1 | 4.37 | 4.50 (10/10) | 16 | 0.55 |
| Ours v2 (as shipped, adaptive grip off) | 1.3 / 2.8 | 1.3 / 2.2 | 0.0 / 0.1 | 0.0 | 0.1 | 4.37 | 4.50 (10/10) | 16 | 0.49 |
| Ours v2 (tuned) | 1.3 / 2.8 | 1.3 / 2.2 | 0.0 / 0.1 | 0.0 | 0.1 | 4.37 | 4.50 (10/10) | 16 | 0.51 |
| PathPlanner path2 (new-path, pre-release), as shipped | 25.2 / 66.9 | 26.2 / 67.2 | 7.9 / 8.2 | 0.0 | 15.9 | never | never | 14845 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 22.8 / 65.5 | 23.4 / 67.5 | 1.7 / 1.8 | 0.0 | 0.0 | 4.52 | 4.84 (10/10) | 11350 | 0.02 |

**wall_hug**: along a wall with the bumpers 15 cm off it, then a 90 deg turn to run 15 cm off the next wall (11.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.7 / 13.0 | 4.6 / 12.4 | 3.4 / 3.6 | 0.0 | 3.6 | 3.04 | never | 392 | 57.01 |
| BLine (README gains, r 0.30) | 39.8 / 94.2 | 39.6 / 94.2 | 1.1 / 1.5 | 0.0 | 64.8 | 5.60 | 5.71 (10/10) | 90 | 0.02 |
| BLine (README gains, r 0.45 default) | 38.4 / 90.8 | 38.6 / 90.8 | 2.1 / 2.3 | 0.0 | 58.1 | 5.47 | 5.63 (3/10) | 0 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 38.4 / 90.8 | 38.6 / 90.8 | 2.1 / 2.3 | 0.0 | 58.1 | 5.47 | 5.63 (3/10) | 0 | 0.02 |
| BLine (tuned) | 29.9 / 69.6 | 30.0 / 69.6 | 0.8 / 0.8 | 0.0 | 3.7 | 2.99 | 3.52 (10/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 40.0 / 92.5 | 40.3 / 92.5 | 0.8 / 0.9 | 0.0 | 8.2 | 4.02 | 4.45 (10/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 4.2 / 13.6 | 4.3 / 13.6 | 2.2 / 2.4 | 0.0 | 2.5 | 2.76 | never | 2576 | 2.84 |
| PathPlanner (kP 5) + hold | 4.2 / 13.6 | 4.3 / 13.6 | 0.7 / 0.7 | 0.0 | 2.5 | 2.76 | 3.19 (10/10) | 2576 | 2.78 |
| PathPlanner (tuned) + hold | 0.6 / 1.7 | 0.5 / 1.6 | 0.7 / 0.9 | 0.0 | 0.4 | 3.22 | 3.45 (10/10) | 70 | 2.77 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.3 / 1.4 | 0.3 / 1.5 | 0.7 / 0.7 | 0.0 | 2.6 | 2.83 | 3.26 (10/10) | 1587 | 2.88 |
| DriveToPose chain | 1.0 / 4.0 | 0.1 / 0.2 | 0.1 / 0.1 | 0.0 | 0.0 | 7.24 | 7.45 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 1.3 / 3.1 | 0.8 / 2.0 | 1.4 / 1.6 | 0.0 | 2.5 | 2.94 | 3.17 (10/10) | 71 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.8 / 4.8 | 1.5 / 3.5 | 0.1 / 0.1 | 0.2 | 0.0 | 3.01 | 3.27 (10/10) | 39 | 1.91 |
| Ours v2 (as shipped) | 1.1 / 2.9 | 0.9 / 2.4 | 0.1 / 0.1 | 0.0 | 0.1 | 2.97 | 3.11 (10/10) | 19 | 0.38 |
| Ours v2 (as shipped, adaptive grip off) | 1.1 / 2.9 | 0.9 / 2.4 | 0.1 / 0.1 | 0.0 | 0.1 | 2.97 | 3.11 (10/10) | 19 | 0.37 |
| Ours v2 (tuned) | 1.1 / 2.9 | 0.9 / 2.4 | 0.1 / 0.1 | 0.0 | 0.1 | 2.97 | 3.11 (10/10) | 19 | 0.35 |
| PathPlanner path2 (new-path, pre-release), as shipped | 13.6 / 40.8 | 13.9 / 40.8 | 2.4 / 3.1 | 0.0 | 2.5 | 2.62 | 2.73 (5/10) | 6350 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 10.3 / 35.5 | 10.6 / 35.5 | 1.7 / 1.8 | 0.0 | 0.0 | 3.63 | 3.95 (10/10) | 4379 | 0.02 |

**tag_align_moving_goal**: 5 m to a scoring pose; the true goal is 8 cm sideways and 3 deg off the map pose, revealed 1.5 m out (8.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 3.2 / 7.2 | 3.2 / 7.2 | 0.1 / 0.1 | 0.2 | 6.5 | 2.18 | 2.44 (10/10) | 93 | 33.46 |
| BLine (README gains, r 0.30) | 3.4 / 7.3 | 4.4 / 12.6 | 0.1 / 0.1 | 0.2 | 81.5 | 2.81 | 3.03 (10/10) | 41 | 0.02 |
| BLine (README gains, r 0.45 default) | 3.3 / 7.3 | 2.0 / 6.8 | 0.1 / 0.1 | 0.1 | 82.8 | 2.80 | 3.00 (10/10) | 62 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.5 / 7.9 | 0.7 / 4.3 | 0.1 / 0.1 | 0.2 | 51.5 | 2.71 | 2.92 (10/10) | 35 | 0.02 |
| BLine (tuned) | 6.8 / 16.9 | 4.1 / 16.1 | 0.1 / 0.2 | 0.2 | 1.2 | 3.32 | 3.58 (10/10) | 0 | 0.02 |
| BLine (tuned, with corner caps) | 3.4 / 7.3 | 0.6 / 4.5 | 0.1 / 0.1 | 0.2 | 1.2 | 3.34 | 3.61 (10/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 3.0 / 7.8 | 2.9 / 7.8 | 3.2 / 3.5 | 0.0 | 3.5 | 1.80 | never | 3 | 1.71 |
| PathPlanner (kP 5) + hold | 3.0 / 7.8 | 2.9 / 7.8 | 0.2 / 0.9 | 0.0 | 3.5 | 1.80 | 2.25 (10/10) | 3 | 1.68 |
| PathPlanner (tuned) + hold | 3.1 / 7.8 | 3.0 / 7.8 | 0.9 / 1.4 | 0.0 | 1.4 | 1.95 | 2.16 (10/10) | 306 | 1.68 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 3.0 / 7.8 | 2.9 / 7.8 | 0.2 / 0.9 | 0.0 | 3.5 | 1.80 | 2.24 (10/10) | 4 | 1.70 |
| DriveToPose chain | 4.3 / 7.6 | 2.4 / 7.6 | 0.1 / 0.1 | 0.3 | 0.0 | 2.65 | 2.85 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 0.7 / 1.6 | 0.6 / 1.7 | 0.1 / 0.2 | 0.2 | 4.0 | 2.12 | 2.38 (10/10) | 2 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.5 / 6.2 | 1.6 / 6.0 | 0.1 / 0.1 | 0.2 | 0.0 | 1.84 | 2.09 (10/10) | 380 | 1.17 |
| Ours v2 (as shipped) | 2.8 / 7.6 | 2.8 / 7.6 | 0.0 / 0.1 | 0.2 | 0.1 | 1.81 | 1.95 (10/10) | 19 | 0.26 |
| Ours v2 (as shipped, adaptive grip off) | 2.8 / 7.6 | 2.8 / 7.6 | 0.0 / 0.1 | 0.2 | 0.1 | 1.81 | 1.95 (10/10) | 19 | 0.28 |
| Ours v2 (tuned) | 2.8 / 7.6 | 2.8 / 7.6 | 0.0 / 0.1 | 0.2 | 0.1 | 1.81 | 1.95 (10/10) | 19 | 0.28 |
| PathPlanner path2 (new-path, pre-release), as shipped | 3.4 / 7.3 | 1.7 / 7.4 | 0.1 / 0.2 | 0.2 | 1.4 | 1.77 | 2.04 (10/10) | 3251 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 4.5 / 10.7 | 1.6 / 7.5 | 0.1 / 0.2 | 0.2 | 0.5 | 3.15 | 3.42 (10/10) | 406 | 0.02 |

**intake_drive_through**: drive through a game piece at 2 m/s; its detected position moves 10 cm sideways 2 m out (7.4 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.9 / 2.1 | 0.9 / 2.1 | 10.6 / 10.6 | 0.0 | 3.5 | never | never | 94 | 26.41 |
| BLine (README gains, r 0.30) | 3.6 / 9.9 | 3.5 / 9.9 | 41.3 / 44.1 | 0.0 | 44.0 | 1.39 | never | 1075 | 0.02 |
| BLine (README gains, r 0.45 default) | 3.7 / 9.9 | 3.5 / 9.9 | 41.3 / 44.1 | 0.0 | 44.0 | 1.39 | never | 1075 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.7 / 9.9 | 3.5 / 9.9 | 41.3 / 44.1 | 0.0 | 44.0 | 1.39 | never | 1075 | 0.02 |
| BLine (tuned) | 4.5 / 9.9 | 4.3 / 9.9 | 21.8 / 22.6 | 0.0 | 39.0 | 1.65 | never | 771 | 0.02 |
| BLine (tuned, with corner caps) | 4.3 / 9.8 | 4.2 / 9.8 | 21.8 / 22.7 | 0.0 | 39.1 | 1.65 | never | 776 | 0.02 |
| PathPlanner (kP 5) | 3.9 / 10.0 | 3.9 / 10.0 | 24.8 / 25.5 | 0.0 | 25.5 | 1.42 | never | 715 | 1.38 |
| PathPlanner (kP 5) + hold | 3.9 / 10.0 | 3.9 / 10.0 | 24.8 / 25.5 | 0.0 | 25.5 | 1.42 | never | 715 | 1.33 |
| PathPlanner (tuned) + hold | 4.0 / 9.9 | 4.0 / 9.9 | 22.4 / 23.6 | 0.0 | 23.6 | 1.52 | never | 808 | 1.30 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 3.9 / 10.0 | 3.9 / 10.0 | 24.8 / 25.5 | 0.0 | 25.5 | 1.42 | never | 715 | 1.34 |
| DriveToPose chain | 3.7 / 9.8 | 3.6 / 9.8 | 0.1 / 0.1 | 0.0 | 0.0 | 2.36 | 2.57 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 0.2 / 0.4 | 0.2 / 0.5 | 26.8 / 27.1 | 0.3 | 25.1 | never | never | 652 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.6 / 7.8 | 1.6 / 7.8 | 0.1 / 0.1 | 0.2 | 0.0 | 1.59 | 1.85 (10/10) | 390 | 0.93 |
| Ours v2 (as shipped) | 4.0 / 9.8 | 4.0 / 9.8 | 24.7 / 26.2 | 0.0 | 26.2 | 1.40 | never | 746 | 0.25 |
| Ours v2 (as shipped, adaptive grip off) | 4.0 / 9.8 | 4.0 / 9.8 | 24.7 / 26.2 | 0.0 | 26.2 | 1.40 | never | 746 | 0.26 |
| Ours v2 (tuned) | 4.0 / 9.8 | 4.0 / 9.8 | 24.7 / 26.2 | 0.0 | 26.2 | 1.40 | never | 746 | 0.26 |
| PathPlanner path2 (new-path, pre-release), as shipped | 0.7 / 1.5 | 0.1 / 0.5 | 10.9 / 11.7 | 0.0 | 0.0 | never | never | 2832 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 1.1 / 5.2 | 1.4 / 5.5 | 10.2 / 10.2 | 0.0 | 0.0 | never | never | 578 | 0.02 |

**three_piece_chain**: three paths with rolling handoffs at 2 m/s (core-builder's neutral_three_piece, 2026 field) (15.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 7.2 / 20.9 | 4.2 / 11.1 | 2.7 / 3.0 | 4.0 | 3.0 | 5.60 | never | 822 | 25.77 |
| BLine (README gains, r 0.30) | 20.5 / 53.1 | 21.8 / 53.1 | 7.9 / 8.9 | 0.4 | 91.9 | 6.86 | never | 4881 | 0.02 |
| BLine (README gains, r 0.45 default) | 19.8 / 51.9 | 20.9 / 51.9 | 8.0 / 8.6 | 0.4 | 90.2 | 6.82 | never | 4747 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 9.5 / 29.4 | 10.2 / 32.5 | 2.5 / 2.7 | 1.8 | 4.2 | 5.15 | never | 3629 | 0.02 |
| BLine (tuned) | 13.2 / 40.3 | 13.4 / 33.6 | 0.8 / 0.9 | 1.8 | 0.2 | 7.20 | never | 3668 | 0.02 |
| BLine (tuned, with corner caps) | 10.2 / 32.7 | 10.4 / 32.7 | 0.8 / 0.9 | 0.7 | 0.0 | 8.07 | 8.52 (10/10) | 3808 | 0.02 |
| PathPlanner (kP 5) | 2.4 / 6.1 | 2.3 / 5.0 | 2.9 / 3.2 | 0.1 | 3.2 | 4.59 | never | 4073 | 1.35 |
| PathPlanner (kP 5) + hold | 2.4 / 6.1 | 2.3 / 5.0 | 0.1 / 0.1 | 0.0 | 3.2 | 4.59 | 5.02 (10/10) | 4073 | 1.37 |
| PathPlanner (tuned) + hold | 1.0 / 5.2 | 0.9 / 4.5 | 0.4 / 1.0 | 0.0 | 1.1 | 5.43 | 5.67 (10/10) | 1198 | 1.37 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.8 / 5.0 | 1.8 / 4.3 | 0.1 / 0.1 | 0.0 | 3.2 | 4.71 | 5.14 (10/10) | 2809 | 1.34 |
| DriveToPose chain | 5.0 / 18.9 | 0.5 / 1.9 | 0.2 / 0.2 | 1.7 | 0.7 | 10.69 | never | 466 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 6.7 / 15.9 | 1.7 / 3.2 | 1.6 / 1.7 | 2.1 | 2.1 | 4.44 | never | 356 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 7.4 / 18.3 | 3.0 / 5.7 | 0.2 / 0.2 | 0.5 | 0.0 | 4.53 | 4.79 (10/10) | 247 | 3.11 |
| Ours v2 (as shipped) | 2.2 / 7.3 | 1.6 / 4.6 | 0.1 / 0.1 | 0.3 | 0.1 | 4.92 | 5.06 (10/10) | 600 | 0.41 |
| Ours v2 (as shipped, adaptive grip off) | 2.2 / 7.5 | 1.6 / 4.6 | 0.1 / 0.1 | 0.3 | 0.1 | 4.90 | 5.05 (10/10) | 657 | 0.37 |
| Ours v2 (tuned) | 2.2 / 7.3 | 1.6 / 4.6 | 0.1 / 0.1 | 0.3 | 0.1 | 4.92 | 5.06 (10/10) | 600 | 0.34 |
| PathPlanner path2 (new-path, pre-release), as shipped | 14.2 / 32.5 | 17.2 / 40.9 | 8.0 / 8.3 | 0.4 | 17.3 | never | never | 9133 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 8.3 / 21.5 | 8.3 / 23.4 | 1.7 / 1.8 | 0.0 | 0.0 | 5.03 | 5.35 (10/10) | 5846 | 0.02 |

**fast_long**: an 8 m gentle curve with a 5 m/s cap (above the robot's 4.57 m/s free speed) (10.8 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 2.0 / 3.4 | 2.0 / 3.4 | 3.4 / 3.5 | 0.0 | 3.5 | 2.63 | never | 91 | 62.26 |
| BLine (README gains, r 0.30) | 2.2 / 4.6 | 0.3 / 1.5 | 41.2 / 43.7 | 0.0 | 43.7 | 2.41 | never | 1051 | 0.02 |
| BLine (README gains, r 0.45 default) | 2.9 / 5.4 | 0.9 / 3.1 | 41.2 / 42.8 | 0.0 | 42.8 | 2.41 | never | 1051 | 0.02 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 2.9 / 5.4 | 0.9 / 3.1 | 41.2 / 42.8 | 0.0 | 42.8 | 2.41 | never | 1051 | 0.02 |
| BLine (tuned) | 4.1 / 8.2 | 2.5 / 8.0 | 0.8 / 0.8 | 0.0 | 0.0 | 3.46 | 3.93 (10/10) | 0 | 0.03 |
| BLine (tuned, with corner caps) | 2.1 / 4.6 | 0.5 / 2.8 | 0.8 / 0.9 | 0.0 | 0.0 | 3.47 | 3.93 (10/10) | 0 | 0.02 |
| PathPlanner (kP 5) | 0.1 / 0.3 | 0.1 / 0.3 | 3.1 / 3.4 | 0.0 | 3.4 | 2.55 | never | 0 | 3.87 |
| PathPlanner (kP 5) + hold | 0.1 / 0.3 | 0.1 / 0.3 | 0.1 / 0.1 | 0.0 | 3.4 | 2.55 | 2.99 (10/10) | 0 | 3.78 |
| PathPlanner (tuned) + hold | 0.1 / 0.2 | 0.1 / 0.2 | 0.6 / 1.2 | 0.0 | 1.2 | 2.70 | 2.92 (10/10) | 0 | 4.42 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.1 / 0.3 | 0.1 / 0.3 | 0.1 / 0.1 | 0.0 | 3.4 | 2.55 | 2.99 (10/10) | 0 | 4.60 |
| DriveToPose chain | 1.9 / 4.4 | 0.1 / 0.2 | 0.1 / 0.1 | 0.0 | 0.0 | 4.93 | 5.14 (10/10) | 0 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 0.4 / 0.9 | 0.4 / 0.7 | 1.6 / 1.7 | 0.0 | 2.5 | 2.39 | 2.61 (10/10) | 130 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 0.6 / 1.1 | 0.6 / 1.2 | 0.1 / 0.1 | 0.0 | 0.0 | 2.61 | 2.86 (10/10) | 37 | 2.89 |
| Ours v2 (as shipped) | 0.6 / 1.2 | 0.5 / 1.0 | 0.0 / 0.1 | 0.0 | 0.1 | 2.42 | 2.56 (10/10) | 19 | 0.54 |
| Ours v2 (as shipped, adaptive grip off) | 0.6 / 1.2 | 0.5 / 1.0 | 0.0 / 0.1 | 0.0 | 0.1 | 2.42 | 2.56 (10/10) | 19 | 0.53 |
| Ours v2 (tuned) | 0.6 / 1.2 | 0.5 / 1.0 | 0.0 / 0.1 | 0.0 | 0.1 | 2.42 | 2.56 (10/10) | 19 | 0.48 |
| PathPlanner path2 (new-path, pre-release), as shipped | 1.8 / 4.7 | 0.1 / 1.4 | 4.6 / 6.3 | 0.0 | 0.0 | 2.43 (6/10) | never | 3078 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 1.6 / 3.9 | 1.4 / 4.5 | 1.8 / 1.8 | 0.0 | 0.0 | 3.48 | 3.79 (10/10) | 1565 | 0.02 |

**heavy_robot**: the S-curve with a 68 kg robot (MOI 6.8) and a 96 A current limit (20 % lower) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 9.1 | 4.6 / 9.1 | 3.9 / 4.1 | 0.0 | 4.1 | 2.25 | never | 192 | 62.26 |
| BLine (README gains, r 0.30) | 13.3 / 37.6 | 14.5 / 37.7 | 6.6 / 6.9 | 0.0 | 90.7 | 3.00 | never | 389 | 0.03 |
| BLine (README gains, r 0.45 default) | 8.4 / 26.4 | 9.3 / 26.5 | 4.9 / 5.4 | 0.0 | 85.9 | 2.93 | never | 314 | 0.03 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.0 / 7.6 | 1.3 / 6.2 | 12.7 / 13.7 | 0.0 | 13.7 | 2.22 | never | 534 | 0.03 |
| BLine (tuned) | 6.8 / 15.6 | 4.4 / 14.9 | 0.8 / 0.8 | 0.0 | 0.0 | 2.69 | 3.15 (10/10) | 0 | 0.03 |
| BLine (tuned, with corner caps) | 2.5 / 5.8 | 3.0 / 9.2 | 0.8 / 0.8 | 0.0 | 0.0 | 2.77 | 3.24 (10/10) | 0 | 0.03 |
| PathPlanner (kP 5) | 0.9 / 2.6 | 0.8 / 2.6 | 3.6 / 3.8 | 0.0 | 4.0 | 1.88 | never | 1862 | 3.27 |
| PathPlanner (kP 5) + hold | 0.9 / 2.6 | 0.8 / 2.6 | 0.0 / 0.1 | 0.0 | 4.0 | 1.88 | 2.36 (10/10) | 1862 | 3.07 |
| PathPlanner (tuned) + hold | 0.5 / 1.1 | 0.4 / 1.1 | 0.6 / 1.3 | 0.0 | 1.5 | 2.21 | 2.46 (10/10) | 0 | 3.32 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.7 / 1.9 | 0.6 / 1.8 | 0.0 / 0.1 | 0.0 | 3.9 | 1.92 | 2.38 (10/10) | 1435 | 3.23 |
| DriveToPose chain | 2.8 / 6.0 | 0.2 / 0.6 | 0.0 / 0.1 | 0.0 | 0.0 | 3.92 | 4.09 (10/10) | 0 | 0.00 |
| Choreo + ChoreoLib follower (as shipped) | 4.6 / 8.7 | 1.2 / 1.9 | 1.6 / 1.9 | 0.0 | 2.6 | 1.95 | 2.18 (10/10) | 77 | 0.00 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.2 / 10.4 | 2.1 / 3.6 | 0.1 / 0.1 | 0.0 | 0.0 | 2.02 | 2.27 (10/10) | 5 | 2.15 |
| Ours v2 (as shipped) | 1.2 / 2.9 | 1.0 / 2.0 | 0.2 / 0.3 | 0.0 | 0.3 | 2.04 | 2.18 (10/10) | 0 | 0.46 |
| Ours v2 (as shipped, adaptive grip off) | 1.2 / 2.9 | 1.0 / 2.0 | 0.2 / 0.3 | 0.0 | 0.3 | 2.04 | 2.18 (10/10) | 0 | 0.49 |
| Ours v2 (tuned) | 1.2 / 2.9 | 1.0 / 2.0 | 0.2 / 0.3 | 0.0 | 0.3 | 2.04 | 2.18 (10/10) | 0 | 0.42 |
| PathPlanner path2 (new-path, pre-release), as shipped | 5.6 / 13.0 | 7.2 / 17.0 | 12.2 / 13.8 | 0.0 | 13.8 | 1.71 (7/10) | never | 5440 | 0.02 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 4.8 / 9.3 | 2.3 / 8.8 | 1.7 / 1.8 | 0.0 | 0.0 | 2.78 | 3.09 (10/10) | 2048 | 0.02 |

## Harder courses

Eight courses beyond the original nine (bench/courses_extra.py; what they need beyond a path is in bench/scenarios.py, whose docstring says how each method meets a moving end). Every method in the main grid runs every course; none is skipped, because each either has a way to do the task or shows what happens without one. The tuning sweeps use only the original nine courses.

| course | what it tests | what to look at |
|---|---|---|
| spin_while_driving | 5 m straight while the robot turns a full 360 deg (a heading target at every waypoint) | the full turn on the way: who keeps to the line while spinning, and who slows or slips to do it |
| figure_eight | a self-crossing loop (a lemniscate, 4 m x 1.8 m) through the origin twice; heading held | the crossing: does progress stay on the right lap, or jump to the other one |
| wall_hug | along a wall with the bumpers 15 cm off it, then a 90 deg turn to run 15 cm off the next wall | the closest the bumpers get to the walls (15 cm to spare): who runs wide at the turn |
| tag_align_moving_goal | 5 m to a scoring pose; the true goal is 8 cm sideways and 3 deg off the map pose, revealed 1.5 m out | time to settle on the TRUE goal after the tag moves it 8 cm and 3 deg, and the error left there |
| intake_drive_through | drive through a game piece at 2 m/s; its detected position moves 10 cm sideways 2 m out | how close each passes the piece after it moves 10 cm, and whether it is still going at 2 m/s |
| three_piece_chain | three paths with rolling handoffs at 2 m/s (core-builder's neutral_three_piece, 2026 field) | the whole auto's time with two rolling handoffs, and whether the hand-overs stay smooth |
| fast_long | an 8 m gentle curve with a 5 m/s cap (above the robot's 4.57 m/s free speed) | who can use the top speed: time on a long gentle curve with the cap above what the robot can do |
| heavy_robot | the S-curve with a 68 kg robot (MOI 6.8) and a 96 A current limit (20 % lower) | a heavier, weaker robot: who plans with the right robot and still settles cleanly |

Course-specific numbers: **wall clearance** = the closest the TRUE bumper square (0.85 m) comes to either wall over the run (the plant has no walls, so negative = through it); **miss** = the closest the robot's centre comes to the TRUE piece; **at piece** = true speed there; **keeps going** = runs at least 1 m/s at the piece. On tag_align_moving_goal every time and error is against the TRUE goal. intake_drive_through ends moving, so it has no settle time.

### Perfect pose

| course | winner, time settled s (every run settled, final error within 5 cm) | ours v2 as shipped | ours v2 tuned | course-specific: best / ours v2 / ours tuned |
|---|---|---|---|---|
| spin_while_driving | Ours v2 (as shipped) 2.83 | 2.83 | 2.83 | - |
| figure_eight | PathPlanner (kP 5) + hold, wheel COF 0.9 4.23 | 4.50 | 4.50 | - |
| wall_hug | Ours v2 (as shipped) 3.11 | 3.11 | 3.11 | wall clearance worst run cm: best Ours v2 (as shipped) 15.0; ours 15.0 / 15.0 |
| tag_align_moving_goal | Ours v2 (as shipped) 1.95 | 1.95 | 1.95 | - |
| intake_drive_through | n/a (ends moving) | - | - | miss cm (keeps going in every run): best PathPlanner (kP 5) 0.1 at 2.12 m/s; ours 0.3 at 2.05 m/s / 0.3 at 2.05 m/s |
| three_piece_chain | Choreo geometry + DriveToPose finish (hybrid prototype) 4.79 | 5.06 | 5.06 | - |
| fast_long | Ours v2 (as shipped) 2.56 | 2.56 | 2.56 | - |
| heavy_robot | Choreo + ChoreoLib follower (as shipped) 2.18 | 2.18 | 2.18 | - |

Every method on the course-specific numbers (mean; wall clearance and speed at the piece: worst run = lowest):

| method | wall clearance cm, mean / worst | miss at piece cm, mean / worst | at piece m/s, mean / lowest | keeps going |
|---|---|---|---|---|
| Old path-editor FollowPath | 14.2 / 14.2 | 10.0 / 10.0 | 0.63 / 0.61 | 0/10 |
| BLine (README gains, r 0.30) | -78.8 / -79.2 | 0.3 / 0.6 | 2.83 / 2.74 | 10/10 |
| BLine (README gains, r 0.45 default) | -75.2 / -75.8 | 0.3 / 0.5 | 2.83 / 2.74 | 10/10 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | -75.2 / -75.8 | 0.3 / 0.5 | 2.83 / 2.74 | 10/10 |
| BLine (tuned) | -54.2 / -54.6 | 0.3 / 0.4 | 2.02 / 2.01 | 10/10 |
| BLine (tuned, with corner caps) | -77.3 / -77.5 | 0.3 / 0.4 | 2.02 / 2.01 | 10/10 |
| PathPlanner (kP 5) | 1.8 / 1.4 | 0.1 / 0.1 | 2.12 / 2.11 | 10/10 |
| PathPlanner (kP 5) + hold | 1.8 / 1.4 | 0.1 / 0.1 | 2.12 / 2.11 | 10/10 |
| PathPlanner (tuned) + hold | 15.0 / 15.0 | 0.1 / 0.1 | 2.03 / 2.02 | 10/10 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 15.0 / 15.0 | 0.1 / 0.1 | 2.12 / 2.11 | 10/10 |
| DriveToPose chain | 14.9 / 14.9 | 0.1 / 0.1 | 0.00 / 0.00 | 0/10 |
| Choreo + ChoreoLib follower (as shipped) | 11.7 / 11.7 | 10.0 / 10.0 | 2.07 / 2.05 | 10/10 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 10.1 / 10.1 | 0.1 / 0.1 | 0.00 / 0.00 | 0/10 |
| Ours v2 (as shipped) | 15.0 / 15.0 | 0.3 / 0.5 | 2.05 / 2.05 | 10/10 |
| Ours v2 (as shipped, adaptive grip off) | 15.0 / 15.0 | 0.3 / 0.5 | 2.05 / 2.05 | 10/10 |
| Ours v2 (tuned) | 15.0 / 15.0 | 0.3 / 0.5 | 2.05 / 2.05 | 10/10 |
| PathPlanner path2 (new-path, pre-release), as shipped | -23.7 / -25.8 | 10.9 / 11.7 | 0.00 / 0.00 | 0/10 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | -19.8 / -20.5 | 10.2 / 10.2 | 0.00 / 0.00 | 0/10 |

### Wheel odometry + gyro

| course | winner, time settled s (every run settled, final error within 5 cm) | ours v2 as shipped | ours v2 tuned | course-specific: best / ours v2 / ours tuned |
|---|---|---|---|---|
| spin_while_driving | nobody settles every run | 2.84 (2/10) | 2.84 (2/10) | - |
| figure_eight | nobody settles every run | 4.50 (7/10) | 4.50 (7/10) | - |
| wall_hug | nobody settles every run | 3.10 (3/10) | 3.10 (3/10) | wall clearance worst run cm: best DriveToPose chain 4.3; ours 3.4 / 3.4 |
| tag_align_moving_goal | nobody settles every run | 1.95 (9/10) | 1.95 (9/10) | - |
| intake_drive_through | n/a (ends moving) | - | - | miss cm (keeps going in every run): best PathPlanner (kP 5) + hold, wheel COF 0.9 0.3 at 2.10 m/s; ours 0.4 at 2.04 m/s / 0.4 at 2.04 m/s |
| three_piece_chain | nobody settles every run | 5.05 (2/10) | 5.05 (2/10) | - |
| fast_long | nobody settles every run | 2.55 (3/10) | 2.55 (3/10) | - |
| heavy_robot | nobody settles every run | 2.17 (4/10) | 2.17 (4/10) | - |

Every method on the course-specific numbers (mean; wall clearance and speed at the piece: worst run = lowest):

| method | wall clearance cm, mean / worst | miss at piece cm, mean / worst | at piece m/s, mean / lowest | keeps going |
|---|---|---|---|---|
| Old path-editor FollowPath | 11.6 / 3.3 | 10.1 / 10.7 | 0.62 / 0.00 | 1/10 |
| BLine (README gains, r 0.30) | -79.5 / -88.8 | 0.3 / 0.5 | 2.74 / 1.95 | 10/10 |
| BLine (README gains, r 0.45 default) | -76.0 / -84.7 | 0.3 / 0.5 | 2.74 / 1.95 | 10/10 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | -76.0 / -84.7 | 0.3 / 0.5 | 2.74 / 1.95 | 10/10 |
| BLine (tuned) | -55.0 / -64.0 | 0.3 / 0.4 | 2.02 / 2.00 | 10/10 |
| BLine (tuned, with corner caps) | -78.3 / -87.6 | 0.3 / 0.4 | 2.02 / 2.00 | 10/10 |
| PathPlanner (kP 5) | 1.4 / -10.0 | 0.3 / 0.5 | 2.10 / 1.95 | 10/10 |
| PathPlanner (kP 5) + hold | 1.4 / -10.0 | 0.3 / 0.5 | 2.10 / 1.95 | 10/10 |
| PathPlanner (tuned) + hold | 12.0 / 3.5 | 0.3 / 0.5 | 2.02 / 1.81 | 10/10 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 10.3 / 1.1 | 0.3 / 0.5 | 2.10 / 1.95 | 10/10 |
| DriveToPose chain | 12.5 / 4.3 | 0.8 / 4.2 | 0.11 / 0.00 | 0/10 |
| Choreo + ChoreoLib follower (as shipped) | 8.9 / 0.2 | 10.0 / 10.5 | 2.06 / 1.75 | 10/10 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 7.8 / -1.3 | 1.4 / 4.0 | 0.08 / 0.00 | 0/10 |
| Ours v2 (as shipped) | 12.0 / 3.4 | 0.4 / 0.5 | 2.04 / 1.97 | 10/10 |
| Ours v2 (as shipped, adaptive grip off) | 12.0 / 3.4 | 0.4 / 0.5 | 2.04 / 1.97 | 10/10 |
| Ours v2 (tuned) | 12.0 / 3.4 | 0.4 / 0.5 | 2.04 / 1.97 | 10/10 |
| PathPlanner path2 (new-path, pre-release), as shipped | -16.5 / -23.9 | 16.0 / 21.1 | 0.00 / 0.00 | 0/10 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | -21.3 / -30.2 | 10.6 / 14.2 | 0.05 / 0.00 | 0/10 |

### Slip-aware odometry + IMU

| course | winner, time settled s (every run settled, final error within 5 cm) | ours v2 as shipped | ours v2 tuned | course-specific: best / ours v2 / ours tuned |
|---|---|---|---|---|
| spin_while_driving | nobody settles every run | 2.84 (2/10) | 2.84 (2/10) | - |
| figure_eight | nobody settles every run | 4.50 (7/10) | 4.50 (7/10) | - |
| wall_hug | nobody settles every run | 3.10 (3/10) | 3.10 (3/10) | wall clearance worst run cm: best DriveToPose chain 4.3; ours 3.4 / 3.4 |
| tag_align_moving_goal | nobody settles every run | 1.95 (9/10) | 1.95 (9/10) | - |
| intake_drive_through | n/a (ends moving) | - | - | miss cm (keeps going in every run): best PathPlanner (kP 5) + hold, wheel COF 0.9 0.3 at 2.10 m/s; ours 0.4 at 2.04 m/s / 0.4 at 2.04 m/s |
| three_piece_chain | nobody settles every run | 5.05 (2/10) | 5.05 (2/10) | - |
| fast_long | nobody settles every run | 2.55 (3/10) | 2.55 (3/10) | - |
| heavy_robot | nobody settles every run | 2.17 (4/10) | 2.17 (4/10) | - |

Every method on the course-specific numbers (mean; wall clearance and speed at the piece: worst run = lowest):

| method | wall clearance cm, mean / worst | miss at piece cm, mean / worst | at piece m/s, mean / lowest | keeps going |
|---|---|---|---|---|
| Old path-editor FollowPath | 11.6 / 3.3 | 10.1 / 10.7 | 0.62 / 0.00 | 1/10 |
| BLine (README gains, r 0.30) | -79.5 / -88.8 | 0.3 / 0.5 | 2.74 / 1.95 | 10/10 |
| BLine (README gains, r 0.45 default) | -76.0 / -84.7 | 0.3 / 0.5 | 2.74 / 1.95 | 10/10 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | -76.0 / -84.7 | 0.3 / 0.5 | 2.74 / 1.95 | 10/10 |
| BLine (tuned) | -55.0 / -64.0 | 0.3 / 0.4 | 2.02 / 2.00 | 10/10 |
| BLine (tuned, with corner caps) | -78.3 / -87.6 | 0.3 / 0.4 | 2.02 / 2.00 | 10/10 |
| PathPlanner (kP 5) | 1.9 / -9.5 | 0.3 / 0.5 | 2.10 / 1.95 | 10/10 |
| PathPlanner (kP 5) + hold | 1.9 / -9.5 | 0.3 / 0.5 | 2.10 / 1.95 | 10/10 |
| PathPlanner (tuned) + hold | 12.0 / 3.5 | 0.3 / 0.6 | 2.01 / 1.81 | 10/10 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 10.9 / 1.7 | 0.3 / 0.5 | 2.10 / 1.95 | 10/10 |
| DriveToPose chain | 12.5 / 4.3 | 0.8 / 4.2 | 0.11 / 0.00 | 0/10 |
| Choreo + ChoreoLib follower (as shipped) | 8.9 / 0.2 | 10.0 / 10.5 | 2.06 / 1.75 | 10/10 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 7.8 / -1.3 | 1.4 / 4.0 | 0.08 / 0.00 | 0/10 |
| Ours v2 (as shipped) | 12.0 / 3.4 | 0.4 / 0.5 | 2.04 / 1.97 | 10/10 |
| Ours v2 (as shipped, adaptive grip off) | 12.0 / 3.4 | 0.4 / 0.5 | 2.04 / 1.97 | 10/10 |
| Ours v2 (tuned) | 12.0 / 3.4 | 0.4 / 0.5 | 2.04 / 1.97 | 10/10 |
| PathPlanner path2 (new-path, pre-release), as shipped | -31.7 / -55.7 | 15.6 / 19.7 | 0.00 / 0.00 | 0/10 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | -21.7 / -30.0 | 10.7 / 14.2 | 0.05 / 0.00 | 0/10 |

## Pathfinding course

The pathfinding course (PathPlanner AD* and ours, four start/goal pairs) has its own section below, written by `python bench/run.py --pathfind-only` (and by the full run).

## Geometry each method authored

**straight_3m**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (3.00, 0.00) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (1.00, 0.00); (3.00, 0.00): (2.00, 0.00), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, R 0 deg @t 1, (3.00, 0.00) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 4
- Choreo `straight_3m.traj`: 30 samples, 1.418 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (3.00, 0.00) @0 deg
- Neutral path length 3.00 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**s_curve**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.42, 0.00); (1.25, 0.00): (0.83, -0.13), (1.67, 0.13); (2.50, 0.80): (2.08, 0.53), (2.92, 1.07); (3.75, 1.60): (3.33, 1.47), (4.17, 1.73); (5.00, 1.60): (4.58, 1.60), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), R 0 deg @t 1, (5.00, 1.60) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 2.8, 2.8, 2.8, 2.8
- Choreo `s_curve.traj`: 79 samples, 2.105 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (1.25, 0.00) @0 deg, (2.50, 0.80) @0 deg, (3.75, 1.60) @0 deg, (5.00, 1.60) @0 deg
- Neutral path length 5.50 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**hairpin**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.00, 0.00), (2.00, 0.00), (2.50, 0.13), (2.87, 0.50), (3.00, 1.00), (2.87, 1.50), (2.50, 1.87), (2.00, 2.00), (1.00, 2.00), (0.00, 2.00) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.33, 0.00); (1.00, 0.00): (0.67, 0.00), (1.33, 0.00); (2.00, 0.00): (1.75, -0.02), (2.25, 0.02); (2.50, 0.13): (2.36, 0.05), (2.64, 0.22); (2.87, 0.50): (2.78, 0.36), (2.95, 0.64); (3.00, 1.00): (3.00, 0.83), (3.00, 1.17); (2.87, 1.50): (2.95, 1.36), (2.78, 1.64); (2.50, 1.87): (2.64, 1.78), (2.36, 1.95); (2.00, 2.00): (2.25, 1.98), (1.75, 2.02); (1.00, 2.00): (1.33, 2.00), (0.67, 2.00); (0.00, 2.00): (0.33, 2.00), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.00, 0.00), (2.00, 0.00), (2.50, 0.13), (2.87, 0.50), (3.00, 1.00), (2.87, 1.50), (2.50, 1.87), (2.00, 2.00), (1.00, 2.00), R 0 deg @t 1, (0.00, 2.00) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4
- Choreo `hairpin.traj`: 141 samples, 2.999 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (1.00, 0.00) @0 deg, (2.00, 0.00) @0 deg, (2.50, 0.13) @0 deg, (2.87, 0.50) @0 deg, (3.00, 1.00) @0 deg, (2.87, 1.50) @0 deg, (2.50, 1.87) @0 deg, (2.00, 2.00) @0 deg, (1.00, 2.00) @0 deg, (0.00, 2.00) @0 deg
- Neutral path length 7.14 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**slalom**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.75, 0.40), (3.50, -0.40), (5.25, 0.40), (7.00, 0.00) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.58, 0.13); (1.75, 0.40): (1.17, 0.47), (2.33, 0.33); (3.50, -0.40): (2.92, -0.40), (4.08, -0.40); (5.25, 0.40): (4.67, 0.33), (5.83, 0.47); (7.00, 0.00): (6.42, 0.13), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.75, 0.40), (3.50, -0.40), (5.25, 0.40), R 0 deg @t 1, (7.00, 0.00) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 2.6, 2.24, 2.24, 2.6
- Choreo `slalom.traj`: 91 samples, 2.924 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (1.75, 0.40) @0 deg, (3.50, -0.40) @0 deg, (5.25, 0.40) @0 deg, (7.00, 0.00) @0 deg
- Neutral path length 7.51 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**long_auto**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (3.00, 0.50), (5.50, 2.00) @90 deg, (7.50, 3.20), (9.00, 2.20), (9.30, 0.30) @180 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (1.00, 0.17); (3.00, 0.50): (2.08, 0.17), (3.92, 0.83); (5.50, 2.00): (4.75, 1.55), (6.25, 2.45); (7.50, 3.20): (6.92, 3.17), (8.08, 3.23); (9.00, 2.20): (8.70, 2.68), (9.30, 1.72); (9.30, 0.30): (9.20, 0.93), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (3.00, 0.50), R 90 deg @t 1, (5.50, 2.00), (7.50, 3.20), (9.00, 2.20), R 180 deg @t 1, (9.30, 0.30) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 3.47, 3.47, 1.9, 1.9, 2.28
- Choreo `long_auto.traj`: 152 samples, 3.907 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (3.00, 0.50) @45 deg, (5.50, 2.00) @90 deg, (7.50, 3.20) @120 deg, (9.00, 2.20) @150 deg, (9.30, 0.30) @180 deg
- Neutral path length 12.14 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**moving_start**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.50, 0.40), (2.60, 1.40), (3.00, 3.00) @0 deg
- Start velocity (field): (0.0, 2.0, 0.0)
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.50, 0.13); (1.50, 0.40): (1.07, 0.17), (1.93, 0.63); (2.60, 1.40): (2.35, 0.97), (2.85, 1.83); (3.00, 3.00): (2.87, 2.47), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.50, 0.40), (2.60, 1.40), R 0 deg @t 1, (3.00, 3.00) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 3.07, 2.75, 2.75
- Choreo `moving_start.traj`: 63 samples, 1.871 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (1.50, 0.40) @0 deg, (2.60, 1.40) @0 deg, (3.00, 3.00) @0 deg
- Neutral path length 4.72 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**hit_mid_path**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60) @0 deg
- Shove: 800 N for 0.3 s at 50% of the path, to the left of travel
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.42, 0.00); (1.25, 0.00): (0.83, -0.13), (1.67, 0.13); (2.50, 0.80): (2.08, 0.53), (2.92, 1.07); (3.75, 1.60): (3.33, 1.47), (4.17, 1.73); (5.00, 1.60): (4.58, 1.60), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), R 0 deg @t 1, (5.00, 1.60) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 2.8, 2.8, 2.8, 2.8
- Choreo `s_curve.traj`: 79 samples, 2.105 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (1.25, 0.00) @0 deg, (2.50, 0.80) @0 deg, (3.75, 1.60) @0 deg, (5.00, 1.60) @0 deg
- Neutral path length 5.50 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**low_grip_s_curve**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60) @0 deg
- Grip: mu 0.7 static / 0.61 kinetic
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.42, 0.00); (1.25, 0.00): (0.83, -0.13), (1.67, 0.13); (2.50, 0.80): (2.08, 0.53), (2.92, 1.07); (3.75, 1.60): (3.33, 1.47), (4.17, 1.73); (5.00, 1.60): (4.58, 1.60), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), R 0 deg @t 1, (5.00, 1.60) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 2.8, 2.8, 2.8, 2.8
- Choreo `s_curve.traj`: 79 samples, 2.105 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (1.25, 0.00) @0 deg, (2.50, 0.80) @0 deg, (3.75, 1.60) @0 deg, (5.00, 1.60) @0 deg
- Neutral path length 5.50 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**grip_drop_s_curve**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.42, 0.00); (1.25, 0.00): (0.83, -0.13), (1.67, 0.13); (2.50, 0.80): (2.08, 0.53), (2.92, 1.07); (3.75, 1.60): (3.33, 1.47), (4.17, 1.73); (5.00, 1.60): (4.58, 1.60), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), R 0 deg @t 1, (5.00, 1.60) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 2.8, 2.8, 2.8, 2.8
- Choreo `s_curve.traj`: 79 samples, 2.105 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (1.25, 0.00) @0 deg, (2.50, 0.80) @0 deg, (3.75, 1.60) @0 deg, (5.00, 1.60) @0 deg
- Neutral path length 5.50 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**spin_while_driving**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.25, 0.00) @90 deg, (2.50, 0.00) @180 deg, (3.75, 0.00) @270 deg, (5.00, 0.00) @360 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.42, 0.00); (1.25, 0.00): (0.83, 0.00), (1.67, 0.00); (2.50, 0.00): (2.08, 0.00), (2.92, 0.00); (3.75, 0.00): (3.33, 0.00), (4.17, 0.00); (5.00, 0.00): (4.58, 0.00), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, R 90 deg @t 1, (1.25, 0.00), R 180 deg @t 1, (2.50, 0.00), R 270 deg @t 1, (3.75, 0.00), R 360 deg @t 1, (5.00, 0.00) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 4, 4, 4, 4
- Choreo `extra/spin_while_driving.traj`: 105 samples, 2.101 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (1.25, 0.00) @90 deg, (2.50, 0.00) @180 deg, (3.75, 0.00) @270 deg, (5.00, 0.00) @360 deg
- Neutral path length 5.00 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**figure_eight**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (-1.86, -0.61) @0 deg, (-1.56, -0.88), (-1.10, -0.83), (-0.55, -0.47), (0.06, 0.06), (0.67, 0.56), (1.21, 0.87), (1.63, 0.85), (1.91, 0.52), (2.00, -0.00), (1.91, -0.52), (1.63, -0.85), (1.21, -0.87), (0.67, -0.56), (0.06, -0.06), (-0.55, 0.47), (-1.10, 0.83), (-1.56, 0.88), (-1.86, 0.61) @0 deg
- PathPlanner Bezier handles (prev, next): (-1.86, -0.61): -, (-1.76, -0.70); (-1.56, -0.88): (-1.68, -0.84), (-1.43, -0.92); (-1.10, -0.83): (-1.27, -0.90), (-0.93, -0.76); (-0.55, -0.47): (-0.74, -0.62), (-0.35, -0.33); (0.06, 0.06): (-0.14, -0.12), (0.26, 0.23); (0.67, 0.56): (0.48, 0.43), (0.86, 0.70); (1.21, 0.87): (1.04, 0.82), (1.37, 0.91); (1.63, 0.85): (1.52, 0.91), (1.75, 0.79); (1.91, 0.52): (1.84, 0.66), (1.97, 0.38); (2.00, -0.00): (2.00, 0.17), (2.00, -0.17); (1.91, -0.52): (1.97, -0.38), (1.84, -0.66); (1.63, -0.85): (1.75, -0.79), (1.52, -0.91); (1.21, -0.87): (1.37, -0.91), (1.04, -0.82); (0.67, -0.56): (0.86, -0.70), (0.48, -0.43); (0.06, -0.06): (0.26, -0.23), (-0.14, 0.12); (-0.55, 0.47): (-0.35, 0.33), (-0.74, 0.62); (-1.10, 0.83): (-0.93, 0.76), (-1.27, 0.90); (-1.56, 0.88): (-1.43, 0.92), (-1.68, 0.84); (-1.86, 0.61): (-1.76, 0.70), -
- BLine (waypoints): (-1.86, -0.61), R 0 deg @t 0, (-1.56, -0.88), (-1.10, -0.83), (-0.55, -0.47), (0.06, 0.06), (0.67, 0.56), (1.21, 0.87), (1.63, 0.85), (1.91, 0.52), (2.00, -0.00), (1.91, -0.52), (1.63, -0.85), (1.21, -0.87), (0.67, -0.56), (0.06, -0.06), (-0.55, 0.47), (-1.10, 0.83), (-1.56, 0.88), R 0 deg @t 1, (-1.86, 0.61) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 3.11, 3.11, 4, 4, 4, 3.92, 3.11, 3.11, 4, 4, 3.11, 3.11, 3.92, 4, 4, 4, 3.11, 3.11
- Choreo `extra/figure_eight.traj`: 229 samples, 4.432 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (-1.56, -0.88) @0 deg, (-1.10, -0.83) @0 deg, (-0.55, -0.47) @0 deg, (0.06, 0.06) @0 deg, (0.67, 0.56) @0 deg, (1.21, 0.87) @0 deg, (1.63, 0.85) @0 deg, (1.91, 0.52) @0 deg, (2.00, -0.00) @0 deg, (1.91, -0.52) @0 deg, (1.63, -0.85) @0 deg, (1.21, -0.87) @0 deg, (0.67, -0.56) @0 deg, (0.06, -0.06) @0 deg, (-0.55, 0.47) @0 deg, (-1.10, 0.83) @0 deg, (-1.56, 0.88) @0 deg, (-1.86, 0.61) @0 deg
- Neutral path length 10.33 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**wall_hug**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (2.00, 0.00), (4.00, 0.00), (5.00, 0.00), (5.50, -0.13), (5.87, -0.50), (6.00, -1.00), (6.00, -2.00), (6.00, -3.00) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.67, 0.00); (2.00, 0.00): (1.33, 0.00), (2.67, 0.00); (4.00, 0.00): (3.50, 0.00), (4.50, 0.00); (5.00, 0.00): (4.75, 0.00), (5.25, 0.00); (5.50, -0.13): (5.36, -0.05), (5.64, -0.22); (5.87, -0.50): (5.78, -0.36), (5.95, -0.64); (6.00, -1.00): (6.00, -0.75), (6.00, -1.25); (6.00, -2.00): (6.00, -1.67), (6.00, -2.33); (6.00, -3.00): (6.00, -2.67), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (2.00, 0.00), (4.00, 0.00), (5.00, 0.00), (5.50, -0.13), (5.87, -0.50), (6.00, -1.00), (6.00, -2.00), R 0 deg @t 1, (6.00, -3.00) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 4, 4, 4, 4, 4, 4, 4, 4
- Choreo `extra/wall_hug.traj`: 136 samples, 3.098 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (2.00, 0.00) @0 deg, (4.00, 0.00) @0 deg, (5.00, 0.00) @0 deg, (5.50, -0.13) @0 deg, (5.87, -0.50) @0 deg, (6.00, -1.00) @0 deg, (6.00, -2.00) @0 deg, (6.00, -3.00) @0 deg
- Neutral path length 8.57 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**tag_align_moving_goal**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (2.50, 0.50), (5.00, 0.00) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.83, 0.17); (2.50, 0.50): (1.67, 0.50), (3.33, 0.50); (5.00, 0.00): (4.17, 0.17), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (2.50, 0.50), R 0 deg @t 1, (5.00, 0.00) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 3.38, 3.38
- Choreo `extra/tag_align_moving_goal.traj`: 55 samples, 1.945 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (2.50, 0.50) @0 deg, (5.00, 0.00) @0 deg
- Neutral path length 5.11 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**intake_drive_through**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (2.00, 0.40), (4.00, 0.60) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.67, 0.13); (2.00, 0.40): (1.33, 0.30), (2.67, 0.50); (4.00, 0.60): (3.33, 0.53), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (2.00, 0.40), R 0 deg @t 1, (4.00, 0.60) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 4, 4
- Choreo `extra/intake_drive_through.traj`: 49 samples, 1.430 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (2.00, 0.40) @0 deg, (4.00, 0.60) @0 deg
- Neutral path length 4.05 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**three_piece_chain**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (-4.70, -3.45) @0 deg, (-3.65, -3.45), (-2.40, -3.20), (-1.20, -2.20) @90 deg, (-1.20, -0.80), (-1.20, 0.80), (-1.20, 2.20) @90 deg, (-2.40, 3.20), (-3.65, 3.45), (-4.90, 3.30), (-5.70, 2.20) @-46.9517 deg
- PathPlanner Bezier handles (prev, next): (-4.70, -3.45): -, (-4.35, -3.45); (-3.65, -3.45): (-4.03, -3.49), (-3.26, -3.41); (-2.40, -3.20): (-2.81, -3.41), (-1.99, -2.99); (-1.20, -2.20): (-1.20, -2.65), (-1.20, -1.75); (-1.20, -0.80): (-1.20, -1.30), (-1.20, -0.30); (-1.20, 0.80): (-1.20, 0.30), (-1.20, 1.30); (-1.20, 2.20): (-1.20, 1.75), (-1.20, 2.65); (-2.40, 3.20): (-1.99, 2.99), (-2.81, 3.41); (-3.65, 3.45): (-3.23, 3.43), (-4.06, 3.47); (-4.90, 3.30): (-4.56, 3.51), (-5.24, 3.09); (-5.70, 2.20): (-5.43, 2.57), -
- BLine (waypoints): (-4.70, -3.45), R 0 deg @t 0, (-3.65, -3.45), (-2.40, -3.20), R 90 deg @t 1, (-1.20, -2.20), (-1.20, -0.80), (-1.20, 0.80), R 90 deg @t 1, (-1.20, 2.20), (-2.40, 3.20), (-3.65, 3.45), (-4.90, 3.30), R -46.9517 deg @t 1, (-5.70, 2.20) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 4, 4, 3.03, 3.03, 4, 3.03, 3.03, 4, 3.14, 3.14
- Choreo `extra/three_piece_chain.traj`: 226 samples, 4.586 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (-3.65, -3.45) @30 deg, (-2.40, -3.20) @60 deg, (-1.20, -2.20) @90 deg, (-1.20, -0.80) @90 deg, (-1.20, 0.80) @90 deg, (-1.20, 2.20) @90 deg, (-2.40, 3.20) @56 deg, (-3.65, 3.45) @22 deg, (-4.90, 3.30) @-13 deg, (-5.70, 2.20) @-47 deg
- Neutral path length 13.92 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**fast_long**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (2.00, 0.50), (4.00, 0.70), (6.00, 0.50), (8.00, 0.00) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.67, 0.17); (2.00, 0.50): (1.33, 0.38), (2.67, 0.62); (4.00, 0.70): (3.33, 0.70), (4.67, 0.70); (6.00, 0.50): (5.33, 0.62), (6.67, 0.38); (8.00, 0.00): (7.33, 0.17), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (2.00, 0.50), (4.00, 0.70), (6.00, 0.50), R 0 deg @t 1, (8.00, 0.00) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2.5, 5, 4.79, 4.79, 5
- Choreo `extra/fast_long.traj`: 97 samples, 2.548 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (2.00, 0.50) @0 deg, (4.00, 0.70) @0 deg, (6.00, 0.50) @0 deg, (8.00, 0.00) @0 deg
- Neutral path length 8.15 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**heavy_robot**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.42, 0.00); (1.25, 0.00): (0.83, -0.13), (1.67, 0.13); (2.50, 0.80): (2.08, 0.53), (2.92, 1.07); (3.75, 1.60): (3.33, 1.47), (4.17, 1.73); (5.00, 1.60): (4.58, 1.60), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), R 0 deg @t 1, (5.00, 1.60) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 2.8, 2.8, 2.8, 2.8
- Choreo `heavy/heavy_robot.traj`: 79 samples, 2.105 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
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
| across | 13.3 | 1.20 / 1.73 | 18 | 2.26 / 3.1 | 5 / 39 |
| behind_wall | 11.5 | 0.39 / 0.74 | 26 | 0.83 / 1.3 | 3 / 9 |
| moving_start | 13.0 | 0.54 / 0.91 | 18 | 0.91 / 1.5 | 6 / 25 |
| turn_at_goal | 10.7 | 0.29 / 0.58 | 16 | 0.86 / 1.3 | 4 / 15 |

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

## Path speed: PathPlanner and ours

<!-- ppcompare:start -->
_Generated by `python bench/run.py --pp-compare` (10 seeds per cell, 2738 s wall)._

Plan = each tool's own figure for the path as drawn, and they are not measured the same way. Ours (`Path.totalTimeSeconds`) and PathPlanner 2027's (the trajectory's total time) are plans: a time-parameterised profile the follower then tracks. path2 has no plan: its figure is the GUI's 50 Hz simulation of its own follower and setpoint generator on an ideal robot, a simulated run that ends at the follower's end tolerance (10 cm as shipped). path2 is PathPlanner's new-path branch (af86ef2b), a pre-release: no PathPlannerLib release ships it yet. To 5 cm / settled: on the bench plant, from the start (settled = 2 cm, 1 deg, under 5 cm/s, held 0.25 s). sim - plan = time to 5 cm minus the plan. Cross-track is against the authored reference (the neutral curve through the same waypoints; path2 drives the polyline between them, so its cross-track there is the cut of its corners).

### 2026 routes

| route | legs | what |
|---|---|---|
| start_to_shot | (-4.7, -2.2, 0 deg) -> (-5.3, -2.05) -> (-5.9, -1.4, 32 deg) | from the starting line back to a scoring pose, turning to face the hub (1.5 m) |
| depot_and_back | (-4.7, 2.2, 0 deg) -> (-6.2, 2.45) -> (-7.5, 2, 180 deg); (-7.5, 2, 180 deg) -> (-6.7, 1.55) -> (-5.8, 1.3, -31 deg) | starting line to the depot (stop, intake facing the wall), then back to a scoring pose |
| neutral_three_piece | (-4.7, -3.45, 0 deg) -> (-3.645, -3.45) -> (-2.4, -3.2) -> (-1.2, -2.2, 90 deg) at 2 m/s; (-1.2, -2.2, 90 deg) -> (-1.2, -0.8) -> (-1.2, 0.8) -> (-1.2, 2.2, 90 deg) at 2 m/s; (-1.2, 2.2, 90 deg) -> (-2.4, 3.2) -> (-3.645, 3.45) -> (-4.9, 3.3) -> (-5.7, 2.2, -47 deg) | through the lower trench to the neutral zone, a sweep along the fuel facing +y, back through the upper trench to a scoring pose: three paths with rolling handoffs at 2 m/s |
| around_hub_trench | (-6, -0.9, 0 deg) -> (-5.2, -2.7) -> (-4.25, -3.45) -> (-3.04, -3.45) -> (-2.4, -2.6) -> (-2.2, -1) -> (-2.2, 1.2, 90 deg) | from behind the hub round it through the lower trench to the hub's neutral side |
| shot_to_tower | (-5.7, 2.2, -47 deg) -> (-6.6, 1) -> (-7.15, -0.07, 180 deg) | from a scoring pose to the tower for a level-1 climb, turning to face the wall |

### The optimiser, rows (c) and (d)

The editor's waypoint optimiser at its default budget (3 s = 1500 plans per path file) and seed 1, on each tool's own control points, scored by that tool's own figure: ours through `tools/optimise.mjs`; PathPlanner through `pp_objective.py`, a line-for-line port of the same search (`optimiser.ts`) with PathPlanner's generator as the objective. Pass-through waypoints move within 0.5 m and their tangents scale by half to double (2027: both handles; path2 has none); the ends and waypoints with a set heading stay put. The bench has no obstacles, so nothing stops a corridor cutting into the hub, bumps or trench arms; the same holds for every tool. Planned time before -> after, summed over the target's path files:

| target | ours | PathPlanner 2027 | path2 |
|---|---|---|---|
| start_to_shot | 1.05 -> 1.00 (-5%) | 1.03 -> 1.00 (-3%) | 0.88 -> 0.82 (-7%) |
| depot_and_back | 2.84 -> 2.81 (-1%) | 3.17 -> 2.92 (-8%) | 2.88 -> 2.86 (-1%) |
| neutral_three_piece | 5.05 -> 4.80 (-5%) | 4.67 -> 4.53 (-3%) | 4.24 -> 4.10 (-3%) |
| around_hub_trench | 3.33 -> 3.03 (-9%) | 3.06 -> 2.78 (-9%) | 2.88 -> 2.80 (-3%) |
| shot_to_tower | 1.46 -> 1.46 (-0%) | 1.49 -> 1.44 (-3%) | 1.34 -> 1.34 (+0%) |
| straight_3m | 1.42 -> 1.42 (+0%) | 1.42 -> 1.42 (+0%) | 1.24 -> 1.24 (+0%) |
| s_curve | 2.17 -> 1.98 (-9%) | 2.04 -> 1.98 (-3%) | 1.86 -> 1.80 (-3%) |
| hairpin | 2.98 -> 2.81 (-6%) | 2.60 -> 2.48 (-5%) | 2.48 -> 2.40 (-3%) |
| slalom | 2.95 -> 2.42 (-18%) | 2.63 -> 2.42 (-8%) | 2.48 -> 2.24 (-10%) |
| long_auto | 4.01 -> 3.69 (-8%) | 3.78 -> 3.52 (-7%) | 3.82 -> 3.56 (-7%) |
| moving_start | 1.88 -> 1.75 (-7%) | 1.85 -> 1.75 (-5%) | 1.66 -> 1.56 (-6%) |
| hit_mid_path | 2.17 -> 1.98 (-9%) | 2.04 -> 1.98 (-3%) | 1.86 -> 1.80 (-3%) |
| low_grip_s_curve | 2.17 -> 1.98 (-9%) | 2.04 -> 1.98 (-3%) | 1.86 -> 1.80 (-3%) |
| grip_drop_s_curve | 2.17 -> 1.98 (-9%) | 2.04 -> 1.98 (-3%) | 1.86 -> 1.80 (-3%) |

### Perfect pose

All targets (the 2026 routes and the existing courses):

| method | mean plan s | mean to 5 cm s | vs ours to 5 cm | targets reached every seed | mean end error cm | mean wheel slip ms |
|---|---|---|---|---|---|---|
| PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.42 | 2.45 | +2% | 14/14 | 0.1 | 3736 |
| PathPlanner 2027 (Bezier), tuned + hold | 2.83 | 2.70 | +11% | 14/14 | 0.4 | 1601 |
| PathPlanner path2 (new-path, pre-release), as shipped | 2.24 | 2.15 | -10% | 2/14 | 6.2 | 6663 |
| PathPlanner path2, tuned (6 m/s^2 cap) | 3.81 | 3.16 | +38% | 14/14 | 1.7 | 3617 |
| Ours v2, as shipped | 2.55 | 2.43 | +0% | 14/14 | 0.1 | 1370 |
| Ours v2 after the waypoint optimiser | 2.37 | 2.24 | -7% | 14/14 | 0.1 | 1028 |
| PathPlanner 2027 after the optimiser (anchors + handles), + hold | 2.30 | 2.15 | -11% | 14/14 | 0.5 | 2231 |
| PathPlanner path2 after the optimiser (waypoints) | 2.15 | 2.36 | -15% | 4/14 | 9.5 | 5016 |

**start_to_shot**: from the starting line back to a scoring pose, turning to face the hub (1.5 m) (5.3 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.03 | 0.88 | 1.31 | -0.15 | 0.1 / 0.1 | 0.0 | 3.2 | 1.3 / 3.1 | 84 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 1.20 | 1.06 | 1.32 | -0.15 | 0.4 / 1.3 | 0.1 | 1.3 | 0.9 / 2.0 | 0 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 0.88 | 0.78 | never | -0.10 | 2.1 / 2.5 | 3.2 | 0.2 | 5.0 / 11.1 | 2981 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 2.32 | 1.71 | 2.03 | -0.61 | 1.7 / 1.8 | 0.0 | 0.0 | 2.6 / 6.0 | 356 |
| (b) | Ours v2, as shipped | 1.05 | 0.93 | 1.07 | -0.12 | 0.1 / 0.2 | 0.2 | 0.2 | 0.9 / 1.6 | 47 |
| (c) | Ours v2 after the waypoint optimiser | 1.00 | 0.88 | 1.02 | -0.12 | 0.1 / 0.2 | 0.3 | 0.1 | 7.8 / 15.7 | 48 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.00 | 0.85 | 1.28 | -0.15 | 0.1 / 0.1 | 0.0 | 3.1 | 10.5 / 20.8 | 39 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 0.82 | never | never | - | 7.7 / 8.2 | 1.1 | 0.0 | 1.9 / 9.0 | 2294 |

**depot_and_back**: starting line to the depot (stop, intake facing the wall), then back to a scoring pose (8.0 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 3.17 | 3.33 | 3.75 | +0.16 | 0.1 / 0.1 | 0.0 | 2.8 | 1.5 / 3.1 | 49 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.70 | 3.66 | 3.87 | -0.05 | 0.7 / 1.3 | 0.1 | 1.4 | 0.6 / 1.4 | 5 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.88 | 2.46 | never | -0.42 | 2.6 / 2.7 | 1.7 | 0.0 | 3.5 / 7.7 | 4917 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 5.26 | 4.40 | 4.72 | -0.86 | 1.7 / 1.8 | 0.0 | 0.0 | 5.5 / 16.8 | 2145 |
| (b) | Ours v2, as shipped | 2.84 | 2.84 | 3.43 | +0.01 | 0.2 / 0.3 | 0.2 | 0.1 | 2.4 / 5.6 | 557 |
| (c) | Ours v2 after the waypoint optimiser | 2.81 | 2.82 | 3.42 | +0.01 | 0.1 / 0.2 | 0.2 | 0.1 | 12.8 / 31.1 | 319 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 2.92 | 3.08 | 3.50 | +0.16 | 0.1 / 0.1 | 0.0 | 2.9 | 17.6 / 46.0 | 114 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 2.86 | 2.46 | never | -0.40 | 2.3 / 2.4 | 1.6 | 0.0 | 14.0 / 33.6 | 4367 |

**neutral_three_piece**: through the lower trench to the neutral zone, a sweep along the fuel facing +y, back through the upper trench to a scoring pose: three paths with rolling handoffs at 2 m/s (15.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 4.67 | 4.59 | 5.02 | -0.09 | 0.1 / 0.1 | 0.0 | 3.2 | 2.2 / 4.8 | 4073 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 5.51 | 5.43 | 5.67 | -0.08 | 0.4 / 1.0 | 0.0 | 1.1 | 0.9 / 4.0 | 1198 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 4.24 | never | never | - | 8.0 / 8.3 | 0.4 | 17.3 | 10.3 / 32.5 | 9133 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 5.62 | 5.03 | 5.35 | -0.59 | 1.7 / 1.8 | 0.0 | 0.0 | 8.7 / 23.8 | 5846 |
| (b) | Ours v2, as shipped | 5.05 | 4.91 | 5.05 | -0.14 | 0.1 / 0.1 | 0.3 | 0.1 | 1.7 / 5.3 | 650 |
| (c) | Ours v2 after the waypoint optimiser | 4.80 | 4.68 | 4.82 | -0.12 | 0.1 / 0.1 | 0.3 | 0.1 | 13.5 / 42.0 | 1077 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 4.53 | 4.46 | 4.88 | -0.07 | 0.9 / 1.0 | 0.0 | 2.1 | 17.0 / 49.7 | 3332 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 4.10 | 4.05 | never | -0.05 | 5.0 / 8.3 | 0.9 | 8.3 | 10.6 / 19.9 | 8038 |

**around_hub_trench**: from behind the hub round it through the lower trench to the hub's neutral side (11.8 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 3.06 | 2.90 | 3.34 | -0.15 | 0.1 / 0.1 | 0.0 | 3.4 | 2.2 / 7.5 | 3248 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.60 | 3.45 | 3.66 | -0.15 | 0.6 / 1.2 | 0.0 | 1.2 | 0.8 / 1.7 | 14 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.88 | 2.96 (6/10) | never | +0.08 | 4.8 / 6.5 | 0.0 | 0.0 | 8.3 / 37.2 | 7466 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 4.38 | 3.82 | 4.14 | -0.56 | 1.7 / 1.8 | 0.0 | 0.0 | 7.1 / 19.6 | 5548 |
| (b) | Ours v2, as shipped | 3.33 | 3.19 | 3.33 | -0.15 | 0.1 / 0.1 | 0.1 | 0.1 | 2.8 / 6.6 | 2 |
| (c) | Ours v2 after the waypoint optimiser | 3.03 | 2.88 | 3.02 | -0.15 | 0.1 / 0.1 | 0.2 | 0.1 | 27.3 / 53.8 | 2 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 2.78 | 2.63 | 3.06 | -0.15 | 0.1 / 0.1 | 0.0 | 3.2 | 24.7 / 48.4 | 3987 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 2.80 | 2.93 (6/10) | never | +0.13 | 4.9 / 6.5 | 0.0 | 0.0 | 11.0 / 36.2 | 7630 |

**shot_to_tower**: from a scoring pose to the tower for a level-1 climb, turning to face the wall (6.3 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.49 | 1.34 | 1.77 | -0.15 | 0.1 / 0.1 | 0.0 | 3.2 | 1.8 / 3.0 | 158 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 1.73 | 1.58 | 1.84 | -0.15 | 0.3 / 1.2 | 0.0 | 1.3 | 0.6 / 1.2 | 18 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.34 | never | never | - | 7.5 / 8.1 | 1.1 | 0.0 | 0.6 / 2.3 | 2279 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 2.80 | 2.20 | 2.52 | -0.60 | 1.7 / 1.8 | 0.0 | 0.0 | 3.6 / 10.6 | 877 |
| (b) | Ours v2, as shipped | 1.46 | 1.35 | 1.49 | -0.11 | 0.1 / 0.2 | 0.3 | 0.2 | 2.1 / 4.5 | 33 |
| (c) | Ours v2 after the waypoint optimiser | 1.46 | 1.35 | 1.48 | -0.11 | 0.1 / 0.1 | 0.3 | 0.1 | 2.6 / 5.0 | 29 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.44 | 1.29 | 1.72 | -0.15 | 0.1 / 0.1 | 0.0 | 3.2 | 10.0 / 19.5 | 69 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 1.34 | never | never | - | 7.5 / 8.1 | 1.1 | 0.0 | 0.6 / 2.3 | 2279 |

**straight_3m**: 3 m straight, heading held at 0 (6.5 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.42 | 1.26 | 1.71 | -0.15 | 0.1 / 0.1 | 0.0 | 3.4 | 0.0 / 0.1 | 0 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 1.57 | 1.42 | 1.67 | -0.15 | 0.4 / 1.1 | 0.0 | 1.3 | 0.0 / 0.0 | 0 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.24 | 1.15 (4/10) | never | -0.09 | 5.0 / 6.2 | 0.0 | 0.0 | 0.0 / 0.1 | 2816 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 2.86 | 2.20 | 2.52 | -0.66 | 1.7 / 1.8 | 0.0 | 0.0 | 0.0 / 0.1 | 0 |
| (b) | Ours v2, as shipped | 1.42 | 1.29 | 1.43 | -0.12 | 0.1 / 0.2 | 0.0 | 0.2 | 0.0 / 0.1 | 19 |
| (c) | Ours v2 after the waypoint optimiser | 1.42 | 1.29 | 1.43 | -0.12 | 0.1 / 0.2 | 0.0 | 0.2 | 0.0 / 0.1 | 19 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.42 | 1.26 | 1.71 | -0.15 | 0.1 / 0.1 | 0.0 | 3.4 | 0.0 / 0.1 | 0 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 1.24 | 1.15 (4/10) | never | -0.09 | 5.0 / 6.2 | 0.0 | 0.0 | 0.0 / 0.1 | 2816 |

**s_curve**: S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | 1.89 | 2.34 | -0.16 | 0.1 / 0.1 | 0.0 | 3.6 | 0.8 / 2.5 | 1938 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | 2.21 | 2.46 | -0.15 | 0.4 / 1.2 | 0.0 | 1.3 | 0.5 / 1.1 | 0 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.86 | 1.71 (7/10) | never | -0.15 | 11.6 / 13.2 | 0.0 | 13.1 | 7.3 / 13.2 | 5491 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.42 | 2.79 | 3.11 | -0.63 | 1.7 / 1.8 | 0.0 | 0.0 | 4.8 / 9.4 | 2082 |
| (b) | Ours v2, as shipped | 2.17 | 2.04 | 2.18 | -0.13 | 0.0 / 0.1 | 0.0 | 0.1 | 1.2 / 3.0 | 19 |
| (c) | Ours v2 after the waypoint optimiser | 1.98 | 1.86 | 1.99 | -0.13 | 0.1 / 0.2 | 0.0 | 0.2 | 18.9 / 34.0 | 19 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.98 | 1.84 | 2.26 | -0.15 | 0.9 / 1.0 | 0.0 | 2.3 | 19.7 / 36.4 | 0 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 1.80 | 1.67 | 1.78 (8/10) | -0.13 | 1.4 / 2.4 | 0.0 | 2.4 | 13.9 / 24.9 | 3554 |

**hairpin**: 2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held (9.9 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.60 | 2.44 | 3.26 | -0.16 | 0.1 / 0.1 | 0.0 | 21.6 | 18.7 / 40.8 | 6950 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.27 | 3.12 | 3.30 | -0.15 | 0.9 / 1.2 | 0.0 | 1.3 | 0.9 / 1.5 | 29 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.48 | 2.59 (5/10) | never | +0.11 | 4.8 / 6.2 | 0.0 | 0.0 | 15.8 / 48.2 | 7979 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 4.00 | 3.60 | 3.92 | -0.40 | 1.7 / 1.8 | 0.0 | 0.0 | 20.6 / 54.5 | 7334 |
| (b) | Ours v2, as shipped | 2.98 | 2.83 | 2.97 | -0.15 | 0.1 / 0.1 | 0.0 | 0.1 | 1.5 / 3.0 | 19 |
| (c) | Ours v2 after the waypoint optimiser | 2.81 | 2.66 | 2.79 | -0.15 | 0.1 / 0.1 | 0.0 | 0.1 | 12.7 / 22.2 | 51 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 2.48 | 2.33 | 3.16 | -0.15 | 0.8 / 0.9 | 0.0 | 23.3 | 9.7 / 18.2 | 6894 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 2.40 | 2.50 (8/10) | never | +0.10 | 4.5 / 6.3 | 0.0 | 0.0 | 12.4 / 33.3 | 7330 |

**slalom**: through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m (10.3 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.63 | 2.47 | 2.93 | -0.16 | 0.1 / 0.1 | 0.0 | 3.6 | 5.3 / 12.3 | 5242 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.25 | 3.10 | 3.32 | -0.15 | 0.6 / 1.2 | 0.0 | 1.3 | 0.7 / 1.6 | 328 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.48 | never | never | - | 7.5 / 7.8 | 0.0 | 0.0 | 16.8 / 69.3 | 9650 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.90 | 3.25 | 3.57 | -0.65 | 1.7 / 1.8 | 0.0 | 0.0 | 7.2 / 16.6 | 2882 |
| (b) | Ours v2, as shipped | 2.95 | 2.80 | 2.94 | -0.15 | 0.1 / 0.1 | 0.0 | 0.1 | 1.4 / 3.0 | 21 |
| (c) | Ours v2 after the waypoint optimiser | 2.42 | 2.29 | 2.43 | -0.13 | 0.1 / 0.2 | 0.0 | 0.2 | 22.4 / 43.5 | 19 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 2.42 | 2.27 | 2.70 | -0.15 | 0.9 / 1.0 | 0.0 | 2.3 | 21.1 / 46.0 | 0 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 2.24 | 2.16 (6/10) | never | -0.08 | 4.8 / 6.4 | 0.0 | 0.0 | 13.3 / 36.7 | 3938 |

**long_auto**: auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end (14.1 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 3.78 | 3.63 | 4.07 | -0.15 | 0.1 / 0.1 | 0.0 | 3.3 | 1.8 / 5.2 | 2550 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 4.31 | 4.16 | 4.38 | -0.15 | 0.6 / 1.1 | 0.0 | 1.2 | 0.7 / 1.6 | 0 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 3.82 | 3.87 (2/10) | never | +0.05 | 5.5 / 6.4 | 0.0 | 0.0 | 8.1 / 38.9 | 5071 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 5.24 | 4.66 | 4.98 | -0.58 | 1.7 / 1.8 | 0.0 | 0.0 | 7.8 / 23.1 | 3239 |
| (b) | Ours v2, as shipped | 4.01 | 3.87 | 4.01 | -0.13 | 0.1 / 0.1 | 0.3 | 0.1 | 3.9 / 12.3 | 265 |
| (c) | Ours v2 after the waypoint optimiser | 3.69 | 3.56 | 3.70 | -0.13 | 0.0 / 0.1 | 0.3 | 0.1 | 30.5 / 56.5 | 480 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 3.52 | 3.36 | 3.81 | -0.15 | 0.1 / 0.1 | 0.0 | 3.4 | 27.3 / 52.4 | 2036 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 3.56 | 3.56 (9/10) | never | +0.00 | 4.1 / 5.5 | 0.3 | 0.0 | 17.5 / 42.7 | 4884 |

**moving_start**: starts moving 2 m/s sideways (+y), path heads +x then curves left (7.9 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.85 | 1.44 | 1.89 | -0.40 | 0.1 / 0.1 | 0.0 | 3.5 | 20.0 / 33.5 | 4275 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.10 | 1.65 | 1.92 | -0.44 | 0.3 / 1.1 | 0.0 | 1.3 | 14.5 / 31.3 | 4646 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.66 | 1.71 (9/10) | never | +0.05 | 4.2 / 5.2 | 0.0 | 0.0 | 18.9 / 38.2 | 5734 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.24 | 2.52 | 2.84 | -0.72 | 1.7 / 1.8 | 0.0 | 0.0 | 14.1 / 38.1 | 4107 |
| (b) | Ours v2, as shipped | 1.88 | 1.81 | 1.95 | -0.07 | 0.1 / 0.1 | 0.0 | 0.1 | 11.2 / 28.1 | 1782 |
| (c) | Ours v2 after the waypoint optimiser | 1.75 | 1.61 | 1.75 | -0.14 | 0.1 / 0.1 | 0.0 | 0.1 | 35.2 / 58.5 | 1806 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.75 | 1.35 | 1.79 | -0.40 | 0.1 / 0.1 | 0.0 | 3.4 | 40.0 / 62.3 | 2118 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 1.56 | 1.47 (2/10) | never | -0.09 | 5.4 / 6.3 | 0.0 | 0.0 | 19.7 / 52.4 | 3978 |

**hit_mid_path**: the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | 2.49 | 2.79 | +0.44 | 0.1 / 0.1 | 0.0 | 29.9 | 24.0 / 53.4 | 5856 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | 2.53 | 2.67 | +0.17 | 0.2 / 0.2 | 0.0 | 11.6 | 16.2 / 40.1 | 5157 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.86 | never | never | - | 7.6 / 8.3 | 0.0 | 11.3 | 17.7 / 73.4 | 7691 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.42 | 2.93 | 3.25 | -0.49 | 1.7 / 1.8 | 0.0 | 0.0 | 20.4 / 52.1 | 4939 |
| (b) | Ours v2, as shipped | 2.17 | 2.08 | 2.22 | -0.09 | 0.1 / 0.1 | 0.0 | 0.1 | 15.9 / 37.8 | 3675 |
| (c) | Ours v2 after the waypoint optimiser | 1.98 | 1.87 | 2.01 | -0.11 | 0.1 / 0.2 | 0.0 | 0.2 | 14.6 / 33.0 | 2238 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.98 | 1.84 | 2.31 | -0.15 | 0.9 / 1.0 | 0.0 | 3.8 | 14.6 / 35.5 | 3328 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 1.80 | 1.70 | never | -0.10 | 8.3 / 9.2 | 0.0 | 9.1 | 12.1 / 24.9 | 5096 |

**low_grip_s_curve**: the S-curve on mu 0.70 static / 0.61 kinetic (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | 2.91 | 3.35 | +0.86 | 0.1 / 0.1 | 0.0 | 29.4 | 40.6 / 85.2 | 11646 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | 2.18 | 2.52 | -0.18 | 0.1 / 0.2 | 0.0 | 2.7 | 1.0 / 3.3 | 7259 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.86 | never | never | - | 7.8 / 8.1 | 0.0 | 20.6 | 24.8 / 84.3 | 11842 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.42 | 2.59 | 2.90 | -0.83 | 1.8 / 1.8 | 0.0 | 0.0 | 3.9 / 8.4 | 7114 |
| (b) | Ours v2, as shipped | 2.17 | 2.04 | 2.46 | -0.13 | 0.1 / 0.2 | 0.0 | 6.7 | 1.0 / 2.2 | 8034 |
| (c) | Ours v2 after the waypoint optimiser | 1.98 | 1.86 | 2.21 | -0.12 | 0.1 / 0.1 | 0.0 | 4.0 | 18.9 / 34.0 | 5472 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.98 | 1.79 | 2.65 | -0.19 | 0.9 / 1.0 | 0.0 | 17.9 | 19.8 / 36.4 | 6219 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 1.80 | never | never | - | 33.2 / 60.1 | 0.0 | 59.0 | 34.9 / 61.7 | 7872 |

**grip_drop_s_curve**: the S-curve; halfway along, the floor drops from mu 1.15 / 1.0 to 0.70 / 0.61 (a worn carpet seam) (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | 2.74 | 3.04 | +0.69 | 0.1 / 0.1 | 0.0 | 51.2 | 21.8 / 51.2 | 6232 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | 2.18 | 2.52 | -0.18 | 0.1 / 0.2 | 0.0 | 2.4 | 1.0 / 3.1 | 3758 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.86 | never | never | - | 7.6 / 7.8 | 0.0 | 75.5 | 21.0 / 77.3 | 10227 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.42 | 2.52 | 2.84 | -0.90 | 1.7 / 1.8 | 0.0 | 0.0 | 4.7 / 9.0 | 4170 |
| (b) | Ours v2, as shipped | 2.17 | 1.99 | 2.39 | -0.18 | 0.1 / 0.2 | 0.0 | 6.2 | 0.9 / 1.8 | 4059 |
| (c) | Ours v2 after the waypoint optimiser | 1.98 | 1.81 | 2.16 | -0.17 | 0.1 / 0.2 | 0.0 | 4.0 | 19.0 / 34.0 | 2816 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.98 | 1.79 | 2.64 | -0.19 | 0.9 / 1.0 | 0.0 | 17.3 | 19.9 / 36.4 | 3093 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 1.80 | never | never | - | 38.4 / 59.7 | 0.0 | 58.9 | 38.7 / 61.4 | 6150 |

### Wheel odometry + gyro

All targets (the 2026 routes and the existing courses):

| method | mean plan s | mean to 5 cm s | vs ours to 5 cm | targets reached every seed | mean end error cm | mean wheel slip ms |
|---|---|---|---|---|---|---|
| PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.42 | 2.09 | -4% | 4/14 | 20.5 | 2618 |
| PathPlanner 2027 (Bezier), tuned + hold | 2.83 | 2.81 | +12% | 5/14 | 10.4 | 1087 |
| PathPlanner path2 (new-path, pre-release), as shipped | 2.24 | 2.46 | -11% | 1/14 | 20.4 | 5463 |
| PathPlanner path2, tuned (6 m/s^2 cap) | 3.81 | 2.97 | +46% | 0/14 | 16.1 | 3275 |
| Ours v2, as shipped | 2.55 | 2.47 | +0% | 2/14 | 7.3 | 1204 |
| Ours v2 after the waypoint optimiser | 2.37 | 2.29 | -7% | 4/14 | 6.1 | 939 |
| PathPlanner 2027 after the optimiser (anchors + handles), + hold | 2.30 | 1.96 | -8% | 6/14 | 16.8 | 1768 |
| PathPlanner path2 after the optimiser (waypoints) | 2.15 | 2.64 | -14% | 0/14 | 19.4 | 4620 |

**start_to_shot**: from the starting line back to a scoring pose, turning to face the hub (1.5 m) (5.3 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.03 | 0.88 | 1.31 (8/10) | -0.15 | 1.4 / 3.0 | 0.2 | 5.2 | 1.3 / 3.1 | 79 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 1.20 | 1.06 | 1.33 (8/10) | -0.15 | 1.4 / 3.8 | 0.2 | 3.6 | 0.9 / 2.0 | 0 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 0.88 | 0.88 (5/10) | never | -0.00 | 5.1 / 6.0 | 0.3 | 0.0 | 2.2 / 6.2 | 2656 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 2.32 | 1.76 (9/10) | 2.05 (4/10) | -0.56 | 2.7 / 5.3 | 0.2 | 0.0 | 2.4 / 6.1 | 350 |
| (b) | Ours v2, as shipped | 1.05 | 0.94 | 1.07 (8/10) | -0.12 | 1.4 / 3.0 | 0.4 | 2.5 | 1.1 / 2.6 | 48 |
| (c) | Ours v2 after the waypoint optimiser | 1.00 | 0.88 | 1.02 (8/10) | -0.12 | 1.3 / 3.0 | 0.3 | 2.5 | 7.8 / 15.7 | 46 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.00 | 0.85 | 1.28 (8/10) | -0.15 | 1.2 / 2.9 | 0.2 | 5.4 | 10.5 / 20.8 | 42 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 0.82 | never | never | - | 18.3 / 22.7 | 1.3 | 0.0 | 1.8 / 10.0 | 2067 |

**depot_and_back**: starting line to the depot (stop, intake facing the wall), then back to a scoring pose (8.0 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 3.17 | 3.34 | 3.75 (8/10) | +0.16 | 1.3 / 3.2 | 0.2 | 4.5 | 2.0 / 8.1 | 54 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.70 | 3.66 | 3.89 (7/10) | -0.04 | 1.6 / 3.4 | 0.1 | 3.0 | 1.2 / 6.5 | 10 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.88 | 2.48 (5/10) | never | -0.40 | 5.2 / 8.6 | 2.9 | 5.8 | 4.6 / 10.7 | 4698 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 5.26 | 4.57 (8/10) | 4.71 (1/10) | -0.69 | 4.1 / 6.0 | 1.7 | 0.0 | 4.6 / 17.3 | 2267 |
| (b) | Ours v2, as shipped | 2.84 | 2.85 (9/10) | 3.43 (7/10) | +0.01 | 1.1 / 5.4 | 1.8 | 3.0 | 2.7 / 7.3 | 607 |
| (c) | Ours v2 after the waypoint optimiser | 2.81 | 2.83 | 3.43 (7/10) | +0.02 | 1.1 / 5.3 | 1.8 | 3.0 | 12.7 / 31.2 | 336 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 2.92 | 3.09 | 3.52 (8/10) | +0.17 | 1.6 / 3.4 | 0.2 | 4.4 | 17.7 / 46.0 | 120 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 2.86 | 2.50 (9/10) | never | -0.36 | 3.7 / 5.9 | 2.9 | 4.1 | 13.2 / 33.9 | 4306 |

**neutral_three_piece**: through the lower trench to the neutral zone, a sweep along the fuel facing +y, back through the upper trench to a scoring pose: three paths with rolling handoffs at 2 m/s (15.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 4.67 | never | never | - | 12.4 / 17.0 | 0.3 | 7.4 | 7.3 / 18.0 | 3732 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 5.51 | 5.47 (5/10) | 5.56 (1/10) | -0.04 | 6.4 / 12.3 | 0.3 | 8.3 | 4.0 / 14.5 | 991 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 4.24 | 4.04 (1/10) | never | -0.20 | 14.3 / 24.8 | 1.3 | 24.5 | 13.6 / 34.4 | 8557 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 5.62 | never | never | - | 8.9 / 12.5 | 0.3 | 5.6 | 7.9 / 29.0 | 5747 |
| (b) | Ours v2, as shipped | 5.05 | 4.91 (6/10) | 5.03 (2/10) | -0.14 | 5.7 / 11.4 | 0.3 | 9.4 | 4.3 / 15.2 | 661 |
| (c) | Ours v2 after the waypoint optimiser | 4.80 | 4.72 (5/10) | 4.80 (2/10) | -0.08 | 5.6 / 11.3 | 0.3 | 9.1 | 11.8 / 51.5 | 1026 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 4.53 | never | never | - | 15.7 / 19.3 | 0.3 | 2.9 | 12.5 / 60.5 | 3080 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 4.10 | 4.02 (3/10) | never | -0.08 | 13.4 / 26.5 | 0.4 | 24.2 | 14.3 / 38.8 | 8087 |

**around_hub_trench**: from behind the hub round it through the lower trench to the hub's neutral side (11.8 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 3.06 | never | never | - | 16.2 / 17.4 | 0.5 | 0.0 | 7.2 / 14.8 | 2979 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.60 | 3.49 (8/10) | 3.59 (1/10) | -0.11 | 4.3 / 10.9 | 0.4 | 2.6 | 2.6 / 9.9 | 80 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.88 | never | never | - | 26.5 / 31.6 | 0.5 | 0.0 | 11.0 / 18.6 | 6734 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 4.38 | never | never | - | 25.9 / 27.8 | 0.4 | 0.0 | 15.4 / 26.6 | 5114 |
| (b) | Ours v2, as shipped | 3.33 | 3.24 (7/10) | never | -0.10 | 4.5 / 11.7 | 0.5 | 1.4 | 3.7 / 13.4 | 0 |
| (c) | Ours v2 after the waypoint optimiser | 3.03 | 2.94 (7/10) | never | -0.09 | 4.7 / 11.2 | 0.5 | 1.3 | 23.0 / 58.3 | 0 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 2.78 | never | never | - | 32.3 / 36.1 | 0.5 | 0.0 | 19.3 / 51.6 | 3452 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 2.80 | never | never | - | 24.5 / 28.4 | 0.4 | 0.0 | 11.1 / 22.3 | 6759 |

**shot_to_tower**: from a scoring pose to the tower for a level-1 climb, turning to face the wall (6.3 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.49 | 1.34 | 1.77 (5/10) | -0.15 | 2.2 / 5.6 | 0.7 | 7.6 | 1.8 / 3.8 | 109 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 1.73 | 1.59 | 1.87 (4/10) | -0.15 | 2.5 / 6.1 | 0.7 | 6.0 | 0.8 / 2.1 | 18 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.34 | never | never | - | 12.5 / 16.4 | 1.3 | 0.0 | 0.8 / 1.9 | 2114 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 2.80 | 2.29 (8/10) | 2.53 (4/10) | -0.51 | 3.8 / 8.8 | 0.7 | 1.0 | 3.2 / 12.7 | 874 |
| (b) | Ours v2, as shipped | 1.46 | 1.35 (9/10) | 1.49 (5/10) | -0.11 | 2.2 / 5.3 | 0.6 | 4.8 | 2.0 / 4.3 | 31 |
| (c) | Ours v2 after the waypoint optimiser | 1.46 | 1.34 (9/10) | 1.48 (5/10) | -0.11 | 2.2 / 5.4 | 0.7 | 4.8 | 2.6 / 5.1 | 25 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.44 | 1.29 | 1.73 (5/10) | -0.15 | 2.2 / 5.5 | 0.7 | 7.7 | 10.1 / 19.6 | 70 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 1.34 | never | never | - | 12.5 / 16.4 | 1.3 | 0.0 | 0.8 / 1.9 | 2114 |

**straight_3m**: 3 m straight, heading held at 0 (6.5 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.42 | 1.26 | 1.71 (4/10) | -0.16 | 2.3 / 5.9 | 0.1 | 8.5 | 0.1 / 0.3 | 0 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 1.57 | 1.42 | 1.71 (5/10) | -0.15 | 2.7 / 6.5 | 0.1 | 6.6 | 0.1 / 0.3 | 6 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.24 | never | never | - | 12.2 / 15.8 | 0.1 | 0.0 | 0.1 / 0.3 | 2656 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 2.86 | 2.18 (9/10) | 2.53 (4/10) | -0.68 | 2.7 / 7.7 | 0.1 | 3.6 | 0.1 / 0.3 | 0 |
| (b) | Ours v2, as shipped | 1.42 | 1.29 (9/10) | 1.43 (4/10) | -0.13 | 2.3 / 5.8 | 0.1 | 5.4 | 0.1 / 0.3 | 16 |
| (c) | Ours v2 after the waypoint optimiser | 1.42 | 1.29 (9/10) | 1.43 (4/10) | -0.13 | 2.3 / 5.8 | 0.1 | 5.4 | 0.1 / 0.3 | 16 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.42 | 1.26 | 1.71 (4/10) | -0.16 | 2.3 / 5.9 | 0.1 | 8.5 | 0.1 / 0.3 | 0 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 1.24 | never | never | - | 12.2 / 15.8 | 0.1 | 0.0 | 0.1 / 0.3 | 2656 |

**s_curve**: S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | 1.88 (9/10) | 2.34 (4/10) | -0.17 | 4.3 / 10.1 | 0.1 | 12.8 | 1.6 / 3.9 | 1688 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | 2.20 (9/10) | 2.44 (4/10) | -0.16 | 4.1 / 10.6 | 0.1 | 10.3 | 1.3 / 4.5 | 77 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.86 | 1.72 (8/10) | 1.85 (3/10) | -0.14 | 5.0 / 11.3 | 0.1 | 10.6 | 4.5 / 11.3 | 4829 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.42 | 2.77 (7/10) | never | -0.65 | 5.8 / 14.1 | 0.1 | 6.9 | 7.0 / 16.6 | 1975 |
| (b) | Ours v2, as shipped | 2.17 | 2.02 (8/10) | 2.17 (4/10) | -0.15 | 4.1 / 10.5 | 0.1 | 9.0 | 1.6 / 6.7 | 16 |
| (c) | Ours v2 after the waypoint optimiser | 1.98 | 1.84 (8/10) | 1.99 (4/10) | -0.14 | 4.1 / 10.4 | 0.1 | 9.0 | 17.0 / 34.6 | 16 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.98 | 1.83 (9/10) | 2.27 (3/10) | -0.15 | 4.3 / 11.4 | 0.1 | 11.0 | 18.7 / 36.9 | 0 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 1.80 | 1.70 (1/10) | 1.80 (1/10) | -0.10 | 9.7 / 17.8 | 0.1 | 0.0 | 7.1 / 26.1 | 3375 |

**hairpin**: 2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held (9.9 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.60 | never | never | - | 52.7 / 57.8 | 0.1 | 0.0 | 13.5 / 24.1 | 4220 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.27 | 3.16 | 3.28 (4/10) | -0.11 | 2.3 / 4.1 | 0.1 | 0.0 | 1.6 / 7.1 | 77 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.48 | never | never | - | 53.6 / 63.7 | 0.1 | 0.0 | 22.8 / 47.6 | 7074 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 4.00 | never | never | - | 64.6 / 69.1 | 0.1 | 0.0 | 31.0 / 61.0 | 6966 |
| (b) | Ours v2, as shipped | 2.98 | 2.87 | never | -0.11 | 3.0 / 4.6 | 0.1 | 0.0 | 2.0 / 8.2 | 18 |
| (c) | Ours v2 after the waypoint optimiser | 2.81 | 2.70 | never | -0.11 | 2.8 / 4.4 | 0.1 | 0.0 | 12.4 / 27.8 | 49 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 2.48 | never | never | - | 66.9 / 74.1 | 0.1 | 0.0 | 15.8 / 21.8 | 4505 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 2.40 | never | never | - | 52.5 / 66.2 | 0.1 | 0.0 | 14.3 / 26.5 | 7861 |

**slalom**: through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m (10.3 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.63 | 2.46 (9/10) | 2.92 (2/10) | -0.17 | 5.9 / 13.1 | 0.1 | 16.1 | 4.1 / 11.8 | 3810 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.25 | 3.07 (8/10) | 3.39 (3/10) | -0.17 | 5.9 / 13.7 | 0.1 | 13.5 | 1.5 / 5.0 | 581 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.48 | 2.40 (3/10) | never | -0.08 | 8.9 / 14.0 | 0.1 | 13.0 | 13.4 / 38.5 | 7892 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.90 | 3.14 (8/10) | 3.57 (1/10) | -0.76 | 5.7 / 15.5 | 0.1 | 10.1 | 7.7 / 19.5 | 2513 |
| (b) | Ours v2, as shipped | 2.95 | 2.79 (8/10) | 2.93 (3/10) | -0.16 | 5.6 / 14.1 | 0.1 | 12.1 | 1.6 / 7.4 | 16 |
| (c) | Ours v2 after the waypoint optimiser | 2.42 | 2.27 (8/10) | 2.43 (3/10) | -0.15 | 5.4 / 13.9 | 0.1 | 12.1 | 20.1 / 43.9 | 16 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 2.42 | 2.27 (9/10) | 2.71 (2/10) | -0.15 | 5.6 / 14.9 | 0.1 | 14.2 | 20.1 / 46.3 | 0 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 2.24 | 2.16 (1/10) | 2.26 (1/10) | -0.08 | 11.9 / 23.6 | 0.1 | 0.0 | 9.6 / 35.9 | 3702 |

**long_auto**: auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end (14.1 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 3.78 | 3.74 (4/10) | never | -0.05 | 11.6 / 22.9 | 0.9 | 1.1 | 7.6 / 21.7 | 2330 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 4.31 | 4.17 (4/10) | 4.45 (3/10) | -0.14 | 8.2 / 19.2 | 0.9 | 4.3 | 6.8 / 19.3 | 13 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 3.82 | 3.80 (2/10) | never | -0.02 | 10.8 / 24.6 | 0.9 | 2.6 | 11.7 / 44.8 | 4936 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 5.24 | 4.61 (1/10) | 4.95 (1/10) | -0.63 | 11.1 / 24.7 | 0.9 | 1.4 | 10.0 / 34.0 | 3049 |
| (b) | Ours v2, as shipped | 4.01 | 3.88 (4/10) | 4.00 (2/10) | -0.13 | 8.2 / 19.7 | 1.0 | 3.0 | 8.0 / 22.1 | 270 |
| (c) | Ours v2 after the waypoint optimiser | 3.69 | 3.57 (4/10) | 3.71 (2/10) | -0.12 | 8.3 / 19.6 | 0.9 | 2.7 | 23.4 / 64.5 | 481 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 3.52 | 3.45 (4/10) | never | -0.07 | 11.0 / 22.5 | 0.9 | 2.3 | 21.3 / 52.9 | 1898 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 3.56 | 3.54 (1/10) | never | -0.02 | 10.0 / 23.0 | 0.9 | 0.8 | 13.3 / 50.6 | 4518 |

**moving_start**: starts moving 2 m/s sideways (+y), path heads +x then curves left (7.9 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.85 | never | never | - | 56.5 / 59.9 | 0.1 | 0.0 | 54.9 / 60.7 | 2514 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.10 | never | never | - | 54.2 / 56.5 | 0.1 | 0.0 | 52.6 / 57.7 | 1656 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.66 | never | never | - | 11.0 / 15.6 | 0.1 | 3.1 | 13.0 / 38.6 | 4998 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.24 | never | never | - | 13.3 / 18.5 | 0.1 | 3.2 | 15.9 / 38.5 | 3407 |
| (b) | Ours v2, as shipped | 1.88 | 1.79 (8/10) | 1.94 (2/10) | -0.10 | 4.0 / 9.6 | 0.1 | 6.4 | 10.9 / 28.1 | 1804 |
| (c) | Ours v2 after the waypoint optimiser | 1.75 | 1.61 (8/10) | 1.74 (3/10) | -0.14 | 3.9 / 10.3 | 0.1 | 5.6 | 32.4 / 60.4 | 1752 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.75 | never | never | - | 33.5 / 38.8 | 0.1 | 2.8 | 37.7 / 81.5 | 1579 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 1.56 | never | never | - | 16.0 / 21.2 | 0.1 | 3.4 | 23.6 / 63.6 | 3625 |

**hit_mid_path**: the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | never | never | - | 52.4 / 54.9 | 0.1 | 0.0 | 45.9 / 51.7 | 3722 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | never | never | - | 41.3 / 42.3 | 0.1 | 0.0 | 36.1 / 41.5 | 2408 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.86 | never | never | - | 52.7 / 54.1 | 0.1 | 0.0 | 47.0 / 53.6 | 6219 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.42 | never | never | - | 52.4 / 56.3 | 0.1 | 0.0 | 46.1 / 53.0 | 3806 |
| (b) | Ours v2, as shipped | 2.17 | never | never | - | 41.6 / 48.2 | 0.1 | 0.0 | 35.5 / 41.0 | 2427 |
| (c) | Ours v2 after the waypoint optimiser | 1.98 | never | never | - | 22.5 / 26.1 | 0.1 | 0.0 | 18.6 / 33.0 | 1701 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.98 | never | never | - | 37.7 / 39.9 | 0.1 | 0.0 | 28.9 / 35.5 | 1764 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 1.80 | never | never | - | 33.4 / 38.4 | 0.1 | 0.0 | 26.5 / 29.5 | 4720 |

**low_grip_s_curve**: the S-curve on mu 0.70 static / 0.61 kinetic (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | 1.83 (8/10) | never | -0.22 | 27.4 / 36.5 | 0.1 | 39.2 | 11.8 / 25.5 | 7102 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | 2.22 (9/10) | 2.43 (4/10) | -0.14 | 4.4 / 10.0 | 0.1 | 9.3 | 1.4 / 4.2 | 6278 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.86 | 1.92 | never | +0.06 | 6.6 / 13.1 | 0.1 | 13.0 | 6.6 / 16.3 | 7070 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.42 | 2.44 (3/10) | never | -0.98 | 7.7 / 11.9 | 0.1 | 11.5 | 9.2 / 23.2 | 6232 |
| (b) | Ours v2, as shipped | 2.17 | 2.11 (1/10) | never | -0.06 | 12.1 / 23.6 | 0.1 | 0.0 | 4.1 / 11.4 | 7330 |
| (c) | Ours v2 after the waypoint optimiser | 1.98 | 2.02 (1/10) | never | +0.03 | 14.9 / 26.6 | 0.1 | 0.0 | 11.6 / 35.7 | 5168 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.98 | 1.87 | 2.33 (1/10) | -0.12 | 5.5 / 12.6 | 0.1 | 14.5 | 19.7 / 36.9 | 5554 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 1.80 | 1.92 (3/10) | never | +0.12 | 7.0 / 12.9 | 0.1 | 1.8 | 8.6 / 28.4 | 6140 |

**grip_drop_s_curve**: the S-curve; halfway along, the floor drops from mu 1.15 / 1.0 to 0.70 / 0.61 (a worn carpet seam) (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | never | never | - | 40.4 / 51.7 | 0.1 | 50.0 | 35.8 / 54.2 | 4313 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | 2.21 (6/10) | never | -0.15 | 6.9 / 15.2 | 0.1 | 13.6 | 4.0 / 15.2 | 3023 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.86 | never | never | - | 61.2 / 68.6 | 0.1 | 56.1 | 54.4 / 68.6 | 6054 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.42 | never | never | - | 16.4 / 24.5 | 0.1 | 20.7 | 14.3 / 24.5 | 3556 |
| (b) | Ours v2, as shipped | 2.17 | 2.03 (7/10) | never | -0.14 | 6.8 / 13.4 | 0.1 | 12.3 | 3.1 / 13.6 | 3616 |
| (c) | Ours v2 after the waypoint optimiser | 1.98 | 1.81 | 2.05 (1/10) | -0.17 | 6.5 / 14.3 | 0.1 | 13.7 | 19.1 / 34.6 | 2508 |
| (d) | PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.98 | 1.78 | never | -0.20 | 15.4 / 25.7 | 0.1 | 27.1 | 20.0 / 36.9 | 2682 |
| (d) | PathPlanner path2 after the optimiser (waypoints) | 1.80 | never | never | - | 46.2 / 54.6 | 0.1 | 50.9 | 41.5 / 54.6 | 4752 |

<!-- ppcompare:end -->

## Planning grip: ours at mu 0.7 to 1.15

<!-- musweep:start -->
_Generated by `python bench/run.py --mu-sweep` (10 seeds per cell, 1158 s wall)._

Ours planned at mu 0.7 (as shipped), 0.85, 1.0 and 1.15, as drawn and optimised (the optimiser re-run per mu, default budget, seed 1). The output limiter stays at mu 1.0 except at planning mu 1.15, where it is 1.15 too. The plant's grip is mu 1.15 static / 1.0 kinetic (0.70 / 0.61 on the low-grip course, and from halfway on the grip-drop course): planning at 1.15 plans at the very limit of this plant. A real carpet has to be measured. Means over all 14 --pp-compare targets; PathPlanner's rows are from the --pp-compare run.

### Perfect pose: all targets

| method | to 5 cm s | settled s (targets settled every seed) | end error cm | worst overshoot cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 2.43 | 2.64 (14/14) | 0.1 | 6.7 | 1370 |
| Ours v2, planning mu 0.85 | 2.41 | 2.64 (14/14) | 0.1 | 25.2 | 2043 |
| Ours v2, planning mu 1 | 2.45 | 2.64 (14/14) | 0.1 | 42.0 | 3244 |
| Ours v2, planning mu 1.15 | 2.43 | 2.63 (14/14) | 0.1 | 49.8 | 3611 |
| Ours v2 after the waypoint optimiser | 2.24 | 2.45 (14/14) | 0.1 | 4.0 | 1028 |
| Ours v2 optimised, planning mu 0.85 | 2.20 | 2.42 (14/14) | 0.1 | 4.3 | 1307 |
| Ours v2 optimised, planning mu 1 | 2.19 | 2.41 (14/14) | 0.1 | 5.2 | 1824 |
| Ours v2 optimised, planning mu 1.15 | 2.18 | 2.40 (14/14) | 0.1 | 5.3 | 1964 |
| PathPlanner 2027 after the optimiser (anchors + handles), + hold | 2.15 | 2.68 (14/14) | 0.5 | 23.3 | 2231 |
| PathPlanner path2 after the optimiser (waypoints) | 2.36 | 1.78 (0/14) | 9.5 | 59.0 | 5016 |

### Wheel odometry + gyro: all targets

| method | to 5 cm s | settled s (targets settled every seed) | end error cm | worst overshoot cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 2.47 | 2.61 (0/14) | 7.3 | 12.3 | 1204 |
| Ours v2, planning mu 0.85 | 2.43 | 2.56 (0/14) | 8.1 | 17.6 | 1634 |
| Ours v2, planning mu 1 | 2.39 | 2.54 (0/14) | 9.3 | 26.8 | 2685 |
| Ours v2, planning mu 1.15 | 2.28 | 2.02 (0/14) | 13.7 | 33.8 | 2917 |
| Ours v2 after the waypoint optimiser | 2.29 | 2.41 (0/14) | 6.1 | 13.7 | 939 |
| Ours v2 optimised, planning mu 0.85 | 2.27 | 2.39 (0/14) | 6.3 | 13.8 | 1209 |
| Ours v2 optimised, planning mu 1 | 2.25 | 2.13 (0/14) | 6.4 | 14.1 | 1710 |
| Ours v2 optimised, planning mu 1.15 | 1.91 | 2.17 (0/14) | 9.0 | 14.0 | 1845 |
| PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.96 | 2.22 (0/14) | 16.8 | 27.1 | 1768 |
| PathPlanner path2 after the optimiser (waypoints) | 2.64 | 2.03 (0/14) | 19.4 | 50.9 | 4620 |

### Slip-aware odometry + IMU: the bench courses

| method | to 5 cm s | settled s (targets settled every seed) | end error cm | worst overshoot cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 2.42 | 2.53 (0/9) | 5.8 | 12.1 | 1026 |
| Ours v2, planning mu 0.85 | 2.34 | 2.47 (0/9) | 6.4 | 11.9 | 1492 |
| Ours v2, planning mu 1 | 2.40 | 2.50 (0/9) | 5.9 | 11.5 | 2412 |
| Ours v2, planning mu 1.15 | 2.46 | 2.36 (0/9) | 5.6 | 11.9 | 1665 |
| Ours v2 after the waypoint optimiser | 2.14 | 2.26 (0/9) | 4.8 | 13.9 | 759 |
| Ours v2 optimised, planning mu 0.85 | 2.15 | 2.29 (0/9) | 4.7 | 14.4 | 917 |
| Ours v2 optimised, planning mu 1 | 2.18 | 2.32 (0/9) | 4.9 | 14.4 | 1327 |
| Ours v2 optimised, planning mu 1.15 | 2.25 | 2.18 (0/9) | 4.9 | 14.6 | 915 |
| PathPlanner 2027 after the optimiser (anchors + handles), + hold | 2.10 | 2.35 (0/9) | 9.5 | 28.8 | 2452 |
| PathPlanner path2 after the optimiser (waypoints) | 2.12 | 3.58 (0/9) | 17.2 | 57.3 | 4810 |

**low_grip_s_curve**, Perfect pose: the S-curve on mu 0.70 static / 0.61 kinetic

| method | to 5 cm s | settled s | end error cm mean / worst | overshoot worst cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 2.04 | 2.46 | 0.1 / 0.2 | 6.7 | 8034 |
| Ours v2, planning mu 0.85 | 2.56 | 2.73 | 0.1 / 0.3 | 25.2 | 9056 |
| Ours v2, planning mu 1 | 2.70 | 2.91 | 0.1 / 0.1 | 42.0 | 9627 |
| Ours v2, planning mu 1.15 | 2.74 | 2.97 | 0.0 / 0.1 | 49.8 | 10024 |
| Ours v2 after the waypoint optimiser | 1.86 | 2.21 | 0.1 / 0.1 | 4.0 | 5472 |
| Ours v2 optimised, planning mu 0.85 | 1.86 | 2.22 | 0.1 / 0.1 | 4.3 | 5501 |
| Ours v2 optimised, planning mu 1 | 1.86 | 2.23 | 0.1 / 0.1 | 5.0 | 5549 |
| Ours v2 optimised, planning mu 1.15 | 1.86 | 2.23 | 0.1 / 0.2 | 5.0 | 5560 |
| PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.79 | 2.65 | 0.9 / 1.0 | 17.9 | 6219 |
| PathPlanner path2 after the optimiser (waypoints) | never | never | 33.2 / 60.1 | 59.0 | 7872 |

**low_grip_s_curve**, Wheel odometry + gyro: the S-curve on mu 0.70 static / 0.61 kinetic

| method | to 5 cm s | settled s | end error cm mean / worst | overshoot worst cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 2.11 (1/10) | never | 12.1 / 23.6 | 0.0 | 7330 |
| Ours v2, planning mu 0.85 | 2.01 (1/10) | never | 9.5 / 18.4 | 2.5 | 7120 |
| Ours v2, planning mu 1 | 1.93 (1/10) | never | 9.4 / 13.3 | 10.9 | 7112 |
| Ours v2, planning mu 1.15 | never | never | 12.2 / 16.6 | 16.3 | 7155 |
| Ours v2 after the waypoint optimiser | 2.02 (1/10) | never | 14.9 / 26.6 | 0.0 | 5168 |
| Ours v2 optimised, planning mu 0.85 | never | never | 15.2 / 27.0 | 0.0 | 5157 |
| Ours v2 optimised, planning mu 1 | 2.04 (1/10) | never | 14.9 / 26.1 | 0.0 | 5168 |
| Ours v2 optimised, planning mu 1.15 | never | never | 15.6 / 26.5 | 0.0 | 5169 |
| PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.87 | 2.33 (1/10) | 5.5 / 12.6 | 14.5 | 5554 |
| PathPlanner path2 after the optimiser (waypoints) | 1.92 (3/10) | never | 7.0 / 12.9 | 1.8 | 6140 |

**low_grip_s_curve**, Slip-aware odometry + IMU: the S-curve on mu 0.70 static / 0.61 kinetic

| method | to 5 cm s | settled s | end error cm mean / worst | overshoot worst cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 2.48 (8/10) | 2.70 (4/10) | 4.0 / 10.3 | 9.0 | 516 |
| Ours v2, planning mu 0.85 | 2.68 (8/10) | 2.90 (4/10) | 4.1 / 10.6 | 8.9 | 149 |
| Ours v2, planning mu 1 | 2.88 (8/10) | 3.11 (4/10) | 4.0 / 10.5 | 9.0 | 141 |
| Ours v2, planning mu 1.15 | 3.12 (8/10) | 3.31 (4/10) | 4.1 / 10.5 | 8.9 | 83 |
| Ours v2 after the waypoint optimiser | 2.26 (8/10) | 2.49 (4/10) | 4.1 / 10.2 | 8.9 | 281 |
| Ours v2 optimised, planning mu 0.85 | 2.52 (8/10) | 2.75 (4/10) | 4.1 / 10.5 | 8.9 | 178 |
| Ours v2 optimised, planning mu 1 | 2.77 (8/10) | 2.99 (4/10) | 4.0 / 10.5 | 9.0 | 146 |
| Ours v2 optimised, planning mu 1.15 | 3.02 (8/10) | 3.22 (4/10) | 4.1 / 10.5 | 9.0 | 98 |
| PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.87 (9/10) | 2.55 (1/10) | 8.1 / 12.9 | 18.2 | 5845 |
| PathPlanner path2 after the optimiser (waypoints) | 1.86 (9/10) | never | 6.8 / 10.9 | 10.8 | 6260 |

**grip_drop_s_curve**, Perfect pose: the S-curve; halfway along, the floor drops from mu 1.15 / 1.0 to 0.70 / 0.61 (a worn carpet seam)

| method | to 5 cm s | settled s | end error cm mean / worst | overshoot worst cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 1.99 | 2.39 | 0.1 / 0.2 | 6.2 | 4059 |
| Ours v2, planning mu 0.85 | 1.88 | 2.61 | 0.2 / 0.3 | 22.5 | 4808 |
| Ours v2, planning mu 1 | 2.57 | 2.78 | 0.0 / 0.1 | 36.8 | 6235 |
| Ours v2, planning mu 1.15 | 2.61 | 2.83 | 0.0 / 0.1 | 44.3 | 6509 |
| Ours v2 after the waypoint optimiser | 1.81 | 2.16 | 0.1 / 0.2 | 4.0 | 2816 |
| Ours v2 optimised, planning mu 0.85 | 1.81 | 2.17 | 0.1 / 0.1 | 4.2 | 2808 |
| Ours v2 optimised, planning mu 1 | 1.81 | 2.18 | 0.1 / 0.1 | 5.2 | 2859 |
| Ours v2 optimised, planning mu 1.15 | 1.80 | 2.18 | 0.1 / 0.1 | 5.3 | 2854 |
| PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.79 | 2.64 | 0.9 / 1.0 | 17.3 | 3093 |
| PathPlanner path2 after the optimiser (waypoints) | never | never | 38.4 / 59.7 | 58.9 | 6150 |

**grip_drop_s_curve**, Wheel odometry + gyro: the S-curve; halfway along, the floor drops from mu 1.15 / 1.0 to 0.70 / 0.61 (a worn carpet seam)

| method | to 5 cm s | settled s | end error cm mean / worst | overshoot worst cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 2.03 (7/10) | never | 6.8 / 13.4 | 12.3 | 3616 |
| Ours v2, planning mu 0.85 | never | never | 15.2 / 21.4 | 17.6 | 3559 |
| Ours v2, planning mu 1 | never | never | 24.9 / 31.6 | 26.8 | 4392 |
| Ours v2, planning mu 1.15 | never | never | 31.9 / 38.5 | 33.8 | 4449 |
| Ours v2 after the waypoint optimiser | 1.81 | 2.05 (1/10) | 6.5 / 14.3 | 13.7 | 2508 |
| Ours v2 optimised, planning mu 0.85 | 1.81 | 2.05 (1/10) | 6.6 / 14.3 | 13.8 | 2500 |
| Ours v2 optimised, planning mu 1 | 1.81 | 2.05 (1/10) | 6.8 / 14.5 | 14.1 | 2513 |
| Ours v2 optimised, planning mu 1.15 | 1.80 | 1.98 (1/10) | 6.9 / 14.5 | 14.0 | 2504 |
| PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.78 | never | 15.4 / 25.7 | 27.1 | 2682 |
| PathPlanner path2 after the optimiser (waypoints) | never | never | 46.2 / 54.6 | 50.9 | 4752 |

**grip_drop_s_curve**, Slip-aware odometry + IMU: the S-curve; halfway along, the floor drops from mu 1.15 / 1.0 to 0.70 / 0.61 (a worn carpet seam)

| method | to 5 cm s | settled s | end error cm mean / worst | overshoot worst cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | never | never | 10.3 / 14.9 | 7.3 | 3336 |
| Ours v2, planning mu 0.85 | 2.06 (3/10) | 2.19 (1/10) | 10.9 / 17.3 | 10.7 | 3074 |
| Ours v2, planning mu 1 | 2.04 (6/10) | 2.22 (1/10) | 6.6 / 13.6 | 10.1 | 4235 |
| Ours v2, planning mu 1.15 | 2.17 (6/10) | 2.27 (1/10) | 5.7 / 12.9 | 8.8 | 2990 |
| Ours v2 after the waypoint optimiser | 1.82 (9/10) | 2.05 (2/10) | 6.0 / 14.6 | 13.9 | 2488 |
| Ours v2 optimised, planning mu 0.85 | 1.82 (9/10) | 2.05 (4/10) | 5.1 / 15.3 | 14.4 | 2449 |
| Ours v2 optimised, planning mu 1 | 1.82 (9/10) | 1.98 (1/10) | 6.1 / 15.3 | 14.4 | 2486 |
| Ours v2 optimised, planning mu 1.15 | 1.82 (9/10) | 2.05 (1/10) | 6.2 / 15.5 | 14.6 | 2370 |
| PathPlanner 2027 after the optimiser (anchors + handles), + hold | 1.78 | 2.50 (2/10) | 9.5 / 23.2 | 28.8 | 2964 |
| PathPlanner path2 after the optimiser (waypoints) | never | never | 38.6 / 59.0 | 57.3 | 5552 |

Planned time before -> after the optimiser, summed over the target's path files:

| target | mu 0.85 | mu 1 | mu 1.15 |
|---|---|---|---|
| start_to_shot | 1.03 -> 0.98 | 1.01 -> 0.98 | 1.01 -> 0.98 |
| depot_and_back | 2.77 -> 2.74 | 2.75 -> 2.72 | 2.74 -> 2.71 |
| neutral_three_piece | 4.89 -> 4.68 | 4.80 -> 4.61 | 4.73 -> 4.55 |
| around_hub_trench | 3.20 -> 2.92 | 3.11 -> 2.85 | 3.07 -> 2.80 |
| shot_to_tower | 1.44 -> 1.43 | 1.43 -> 1.43 | 1.43 -> 1.42 |
| straight_3m | 1.42 -> 1.42 | 1.42 -> 1.42 | 1.42 -> 1.42 |
| s_curve | 2.10 -> 1.98 | 2.07 -> 1.98 | 2.05 -> 1.98 |
| hairpin | 2.84 -> 2.64 | 2.74 -> 2.58 | 2.66 -> 2.47 |
| slalom | 2.80 -> 2.42 | 2.69 -> 2.42 | 2.62 -> 2.42 |
| long_auto | 3.90 -> 3.61 | 3.83 -> 3.56 | 3.79 -> 3.54 |
| moving_start | 1.85 -> 1.75 | 1.85 -> 1.75 | 1.85 -> 1.75 |
| hit_mid_path | 2.10 -> 1.98 | 2.07 -> 1.98 | 2.05 -> 1.98 |
| low_grip_s_curve | 2.10 -> 1.98 | 2.07 -> 1.98 | 2.05 -> 1.98 |
| grip_drop_s_curve | 2.10 -> 1.98 | 2.07 -> 1.98 | 2.05 -> 1.98 |

<!-- musweep:end -->

**Recommendation: keep mu 0.7 as the default.** No planning grip beats PathPlanner optimised on both measures with a
perfect pose. Optimised, ours settles first at every mu (2.40-2.45 s against 2.68 s), but PathPlanner reaches 5 cm
first at every mu (2.15 s against 2.18 s at mu 1.15 and 2.24 s at 0.7). Raising mu gains at most 0.06 s to 5 cm. On
most targets the 6 m/s^2 and 4 m/s limits bind, not cornering grip. The cost is wheel slip (1028 -> 1964 ms) and a
worse end error on wheel odometry (6.1 -> 9.0 cm, still below PathPlanner's 16.8 cm). The rest of the 5 cm gap is
elsewhere:
- moving_start: 1.35 s against 1.61 s. PathPlanner regenerates its trajectory from the measured start speed; ours plans
  from rest.
- A steady 0.02-0.03 s on every path: our end approach (the ease-in and the DriveToPose finish) gets to 5 cm later than
  PathPlanner's time-sampled trajectory, then settles sooner.
The plant's grip is mu 1.15 static / 1.0 kinetic, so 1.15 plans at its very limit. A team that measures more grip on
its carpet can raise `frictionCoefficient` in robot.json; it stays a robot setting.

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
