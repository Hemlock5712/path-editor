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

**Why ours loses the figure eight and the three-piece chain** (`python bench/losses.py [--pose odom]`; speed
against distance from `run_one` traces; 10 seeds, perfect pose unless stated):
- **figure_eight: planning grip, nothing else.** Ours plans at mu 0.7; Choreo plans at the plant's 1.15, and
  PathPlanner at its COF. Speed against distance is close to Choreo's on the straights (within 0.15 m/s). Ours brakes earlier
  into each lobe (2.18 against 2.67 m/s at 3.5 m, 2.09 against 2.46 m/s at 9.5 m), and then finishes faster
  (5 cm to settled 0.14 s against 0.20 s). Mean settle time is level with Choreo (4.504 s against 4.495 s). The
  crossing costs nothing: no projection jump, 1.3 cm RMS. The heading is held, so the spin limits don't come in.
  Planned at mu 0.85, ours settles in 4.244 s, level with PathPlanner at COF 0.9 (4.226 s). It does so at 1.5 cm
  RMS and 1.4 s of slip, against PathPlanner's 9.3 cm and 10.0 s. At mu 1.0 it settles in 4.151 s. With wheel
  odometry PathPlanner at COF 0.9 never settles (11.8 cm off), and ours at 0.7 settles 7 of 10, Choreo 8 of 10.
  Grip stays a robot setting (`frictionCoefficient`); no library change.
- **three_piece_chain: the join directions, then the handoffs.** The route data sets the direction of travel at
  both joins to 90 deg, so the sweep runs straight along the fuel at x = -1.2. The hybrid's Choreo trajectory
  fixes only the robot's heading there and rounds the corners (7.4 cm RMS off the authored route).
  - Ours as one path through every waypoint plans 5.038 s against 5.046 s as the chain, so the fixed 2 m/s
    handoffs cost only 8 ms of plan. On the plant the chain settles 0.048 s later (5.058 s against 5.010 s) and
    slips 600 ms against 21 ms. The cause is a transient at each new leg: its first commands climb about
    11 m/s^2 (2.00 -> 2.65 m/s in three loops) against the leg's 6 m/s^2 plan, and all four wheels slip. That
    is in the follower and has gone to core-builder.
  - A joint plan (prototype, `bench/chain_plan.py`): profile the whole route once and give each join the joint
    profile's speed (2.38 m/s at both joins). The chain then settles in 4.994 s, still with the authored
    directions.
  - Taking the joins' directions from the spline too (66.5 and 113.5 deg) gives 4.769 s as a chain and 4.740 s as
    one path, ahead of the hybrid's 4.788 s. But the sweep then bows 8.5 cm off the fuel line. So the 0.21 s still
    behind the hybrid is the price of the straight sweep the route asks for, not a follower loss.
  - With wheel odometry every variant ends about 5.5-6 cm off and settles 0-2 of 10. The joint plan with free
    directions settles 0 of 10, because of the handoff slip.
  - What the editor would need: an auto made of linked path files, and a "plan across the chain" step. It would
    profile the linked files as one route and write each join's handoff speed (and, if the user unlocks it, its
    direction of travel) into both files, re-running when either changes. The library already takes
    `startMetersPerSecond` / `endMetersPerSecond` and set tangents, so it needs no new path format.


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
_Generated by `python bench/run.py --both` (10 seeds, 519 s wall)._

## Tuning sweeps

Score: settled fraction first, then mean(settle time or sim time) + 10 s per metre of RMS cross-track.

**BLine, no caps** (210 variants, perfect pose, seeds [100, 101, 102], all courses). Chosen: `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 2.0}`. Best five:

| options | settled fraction | score s |
|---|---|---|
| `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 2.0}` | 1.00 | 4.80 |
| `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 1.6}` | 1.00 | 4.84 |
| `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 0.9}` | 1.00 | 4.93 |
| `{'kp': 2.0, 'kp_ct': 4.0, 'tol': 0.01, 'radius': 0.9}` | 1.00 | 4.98 |
| `{'kp': 4.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 2.0}` | 1.00 | 5.02 |

Best variant at each `radius`: 0.6: 1.00 settled, 5.10 s; 0.9: 1.00 settled, 4.93 s; 1.2: 1.00 settled, 5.05 s; 1.6: 1.00 settled, 4.84 s; 2: 1.00 settled, 4.80 s; 2.5: 1.00 settled, 5.25 s; 3: 1.00 settled, 5.38 s

**BLine with corner caps** (90 variants, perfect pose, seeds [100, 101, 102], all courses). Chosen: `{'kp': 2.0, 'kp_ct': 6.0, 'tol': 0.01, 'radius': 0.6, 'caps': True}`. Best five:

| options | settled fraction | score s |
|---|---|---|
| `{'kp': 2.0, 'kp_ct': 6.0, 'tol': 0.01, 'radius': 0.6, 'caps': True}` | 1.00 | 4.73 |
| `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 0.9, 'caps': True}` | 1.00 | 4.81 |
| `{'kp': 2.0, 'kp_ct': 4.0, 'tol': 0.01, 'radius': 0.6, 'caps': True}` | 1.00 | 4.88 |
| `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 1.2, 'caps': True}` | 1.00 | 4.89 |
| `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 0.6, 'caps': True}` | 1.00 | 4.93 |

Best variant at each `radius`: 0.3: 1.00 settled, 5.21 s; 0.45: 1.00 settled, 5.15 s; 0.6: 1.00 settled, 4.73 s; 0.9: 1.00 settled, 4.81 s; 1.2: 1.00 settled, 4.89 s; 1.6: 1.00 settled, 5.09 s

**PathPlanner + hold** (70 variants, perfect pose, seeds [100, 101, 102], all courses). Chosen: `{'kp': 10.0, 'kp_rot': 10.0, 'hold': True, 'cof': 1.15}`. Best five:

| options | settled fraction | score s |
|---|---|---|
| `{'kp': 10.0, 'kp_rot': 10.0, 'hold': True, 'cof': 1.15}` | 1.00 | 2.84 |
| `{'kp': 10.0, 'kp_rot': 5.0, 'hold': True, 'cof': 0.9}` | 1.00 | 2.86 |
| `{'kp': 10.0, 'kp_rot': 10.0, 'hold': True, 'cof': 0.9}` | 1.00 | 2.86 |
| `{'kp': 10.0, 'kp_rot': 5.0, 'hold': True, 'cof': 0.7}` | 1.00 | 2.86 |
| `{'kp': 10.0, 'kp_rot': 10.0, 'hold': True, 'cof': 0.7}` | 1.00 | 2.86 |

Best variant at each `cof`: 0.4: 1.00 settled, 3.29 s; 0.5: 1.00 settled, 3.07 s; 0.6: 1.00 settled, 2.93 s; 0.7: 1.00 settled, 2.86 s; 0.9: 1.00 settled, 2.86 s; 1.15: 1.00 settled, 2.84 s; 1.5: 1.00 settled, 3.01 s

**Ours v2** (9 variants, perfect pose, seeds [100, 101, 102], all courses). Chosen: `{'easeInPerSecond': 12.0, 'latencySeconds': 0.05}`. Best five:

| options | settled fraction | score s |
|---|---|---|
| `{'easeInPerSecond': 12.0, 'latencySeconds': 0.05}` | 1.00 | 2.90 |
| `{'easeInPerSecond': 16.0, 'latencySeconds': 0.05}` | 1.00 | 2.90 |
| `{'easeInPerSecond': 9.0, 'latencySeconds': 0.05}` | 1.00 | 2.92 |
| `{'easeInPerSecond': 9.0, 'latencySeconds': 0.03}` | 1.00 | 2.94 |
| `{'easeInPerSecond': 12.0, 'latencySeconds': 0.03}` | 1.00 | 3.00 |

Best variant at each `easeInPerSecond`: 9: 1.00 settled, 2.92 s; 12: 1.00 settled, 2.90 s; 16: 1.00 settled, 2.90 s

**PathPlanner path2** (96 variants, perfect pose, seeds [100, 101, 102], all courses). Chosen: `{'accel_cap': 6.0, 'kp': 2.0, 'kp_ct': 4.0, 'handoff': 0.5, 'end_distance': 0.02, 'end_degrees': 1.0}`. Best five:

| options | settled fraction | score s |
|---|---|---|
| `{'accel_cap': 6.0, 'kp': 2.0, 'kp_ct': 4.0, 'handoff': 0.5, 'end_distance': 0.02, 'end_degrees': 1.0}` | 1.00 | 3.87 |
| `{'accel_cap': 6.0, 'kp': 2.0, 'kp_ct': 2.0, 'handoff': 0.5, 'end_distance': 0.02, 'end_degrees': 1.0}` | 1.00 | 3.97 |
| `{'accel_cap': 6.0, 'kp': 2.0, 'kp_ct': 4.0, 'handoff': 0.25, 'end_distance': 0.02, 'end_degrees': 1.0}` | 1.00 | 3.97 |
| `{'accel_cap': 6.0, 'kp': 2.0, 'kp_ct': 1.0, 'handoff': 0.5, 'end_distance': 0.02, 'end_degrees': 1.0}` | 1.00 | 3.98 |
| `{'accel_cap': 6.0, 'kp': 2.0, 'kp_ct': 2.0, 'handoff': 0.25, 'end_distance': 0.02, 'end_degrees': 1.0}` | 1.00 | 4.04 |

Best variant at each `handoff`: 0.1: 1.00 settled, 4.15 s; 0.25: 1.00 settled, 3.97 s; 0.5: 1.00 settled, 3.87 s; 0.75: 1.00 settled, 4.07 s

## Per-course tables

Accuracy is always against the TRUE plant pose. Cross-track is measured from the start until the robot first reaches the end of the path. Times are means over the runs that got there; 'settled' = within 2 cm and 1 deg of the end pose, true speed under 5 cm/s, held 0.25 s. Wheel slip = milliseconds of slip summed over the four wheels. Path prep = median wall-clock of the method's own preparation in Python (relative only; for Choreo it is only loading the .traj, its solver time is in the geometry section).

### Wheel odometry + gyro (no vision): the main condition

| method | runs settled | courses settled every seed | mean RMS cross-track cm | worst cross-track cm | mean final error cm | mean wheel slip ms |
|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 9/90 | 0/9 | 8.9 | 45.0 | 10.3 | 1328 |
| BLine (README gains, r 0.30) | 1/90 | 0/9 | 24.3 | 104.7 | 19.7 | 2640 |
| BLine (README gains, r 0.45 default) | 8/90 | 0/9 | 21.4 | 104.6 | 16.8 | 2350 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 8/90 | 0/9 | 13.4 | 104.6 | 14.1 | 1380 |
| BLine (tuned) | 22/90 | 0/9 | 17.6 | 66.2 | 11.8 | 1092 |
| BLine (tuned, with corner caps) | 22/90 | 0/9 | 11.9 | 103.7 | 7.7 | 1091 |
| PathPlanner (kP 5) | 3/90 | 0/9 | 12.5 | 50.5 | 19.6 | 2890 |
| PathPlanner (kP 5) + hold | 8/90 | 0/9 | 12.5 | 50.5 | 18.8 | 2898 |
| PathPlanner (tuned) + hold | 12/90 | 0/9 | 10.6 | 51.0 | 15.7 | 2812 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 17/90 | 0/9 | 9.6 | 48.6 | 13.6 | 2498 |
| DriveToPose chain | 31/90 | 0/9 | 7.4 | 58.8 | 8.1 | 1860 |
| Choreo + ChoreoLib follower (as shipped) | 26/90 | 0/9 | 14.2 | 94.4 | 16.8 | 2287 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 23/90 | 0/9 | 13.7 | 94.2 | 14.5 | 2059 |
| Ours v2 (as shipped) | 17/90 | 0/9 | 8.2 | 51.8 | 11.2 | 2149 |
| Ours v2 (as shipped, adaptive grip off) | 12/90 | 0/9 | 8.4 | 49.7 | 11.4 | 2689 |
| Ours v2 (tuned) | 17/90 | 0/9 | 8.2 | 51.8 | 11.2 | 2149 |
| PathPlanner path2 (new-path, pre-release), as shipped | 3/90 | 0/9 | 10.5 | 49.6 | 17.8 | 4124 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 12/90 | 0/9 | 11.0 | 56.5 | 13.4 | 2857 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 24/90 | 0/9 | 7.4 | 40.4 | 8.5 | 1581 |

**straight_3m**: 3 m straight, heading held at 0 (6.5 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.1 / 0.3 | 0.1 / 0.3 | 5.0 / 9.8 | 0.1 | 9.8 | 1.35 | 1.53 (2/10) | 501 | 39.58 |
| BLine (README gains, r 0.30) | 0.1 / 0.3 | 0.1 / 0.3 | 30.3 / 39.3 | 0.1 | 69.4 | 1.13 | never | 742 | 0.06 |
| BLine (README gains, r 0.45 default) | 0.1 / 0.3 | 0.1 / 0.3 | 30.3 / 39.3 | 0.1 | 69.4 | 1.13 | never | 742 | 0.04 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 0.1 / 0.3 | 0.1 / 0.3 | 30.3 / 39.3 | 0.1 | 69.4 | 1.13 | never | 742 | 0.04 |
| BLine (tuned) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.8 | 0.1 | 4.5 | 2.18 (9/10) | 2.67 (7/10) | 6 | 0.04 |
| BLine (tuned, with corner caps) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.8 | 0.1 | 4.5 | 2.18 (9/10) | 2.67 (7/10) | 6 | 0.04 |
| PathPlanner (kP 5) | 0.1 / 0.3 | 0.1 / 0.3 | 4.2 / 8.7 | 0.1 | 8.8 | 1.26 | 1.46 (1/10) | 470 | 2.42 |
| PathPlanner (kP 5) + hold | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 8.8 | 1.26 | 1.72 (4/10) | 471 | 2.34 |
| PathPlanner (tuned) + hold | 0.1 / 0.3 | 0.1 / 0.3 | 2.9 / 6.8 | 0.1 | 7.0 | 1.28 | 1.45 (4/10) | 570 | 2.26 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 8.8 | 1.26 | 1.72 (4/10) | 471 | 2.34 |
| DriveToPose chain | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.0 | 5.2 | 1.33 (9/10) | 1.52 (5/10) | 9 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 0.1 / 0.3 | 0.1 / 0.3 | 3.0 / 6.8 | 0.1 | 7.7 | 1.26 | 1.48 (5/10) | 667 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 0.1 / 0.3 | 0.1 / 0.3 | 2.4 / 6.5 | 0.3 | 4.8 | 1.33 (9/10) | 1.58 (5/10) | 531 | 2.43 |
| Ours v2 (as shipped) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.4 | 0.1 | 5.0 | 1.30 (9/10) | 1.43 (6/10) | 368 | 3.61 |
| Ours v2 (as shipped, adaptive grip off) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.4 | 0.1 | 5.0 | 1.30 (9/10) | 1.43 (6/10) | 368 | 1.62 |
| Ours v2 (tuned) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.4 | 0.1 | 5.0 | 1.30 (9/10) | 1.43 (6/10) | 368 | 1.35 |
| PathPlanner path2 (new-path, pre-release), as shipped | 0.1 / 0.3 | 0.1 / 0.3 | 14.1 / 19.2 | 0.1 | 0.0 | never | never | 1590 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 0.1 / 0.3 | 0.1 / 0.3 | 2.7 / 7.7 | 0.1 | 3.5 | 2.18 (9/10) | 2.53 (4/10) | 59 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 0.1 / 0.3 | 0.1 / 0.3 | 2.4 / 6.2 | 0.1 | 5.1 | 1.32 (9/10) | 1.45 (5/10) | 226 | 1.33 |

**s_curve**: S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.5 / 12.0 | 4.5 / 12.0 | 5.8 / 13.4 | 0.1 | 13.2 | 2.24 (9/10) | never | 452 | 181.19 |
| BLine (README gains, r 0.30) | 13.0 / 37.3 | 14.1 / 37.4 | 5.9 / 13.4 | 0.1 | 97.2 | 3.01 (8/10) | never | 1085 | 0.07 |
| BLine (README gains, r 0.45 default) | 8.2 / 27.0 | 9.0 / 27.0 | 5.4 / 12.0 | 0.1 | 91.9 | 2.94 (9/10) | never | 972 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 9.7 | 1.8 / 7.1 | 10.3 / 18.7 | 0.1 | 18.5 | 2.22 | never | 576 | 0.07 |
| BLine (tuned) | 6.7 / 16.5 | 4.7 / 15.8 | 4.2 / 11.2 | 0.1 | 8.2 | 2.63 (8/10) | 3.17 (3/10) | 446 | 0.07 |
| BLine (tuned, with corner caps) | 2.9 / 7.2 | 3.2 / 10.5 | 4.2 / 11.0 | 0.1 | 8.4 | 2.69 (8/10) | 3.26 (3/10) | 385 | 0.05 |
| PathPlanner (kP 5) | 1.5 / 4.6 | 1.4 / 4.6 | 5.8 / 12.7 | 0.1 | 12.5 | 1.87 (9/10) | never | 1562 | 4.33 |
| PathPlanner (kP 5) + hold | 1.5 / 4.6 | 1.4 / 4.6 | 4.2 / 10.2 | 0.1 | 12.5 | 1.87 (9/10) | 2.34 (4/10) | 1565 | 4.51 |
| PathPlanner (tuned) + hold | 1.5 / 4.5 | 1.4 / 4.5 | 4.8 / 10.8 | 0.1 | 10.6 | 1.90 (9/10) | 2.15 (3/10) | 1605 | 4.97 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.3 / 4.3 | 1.2 / 4.3 | 4.1 / 10.4 | 0.1 | 12.1 | 1.91 (9/10) | 2.37 (4/10) | 1366 | 7.96 |
| DriveToPose chain | 2.9 / 9.0 | 0.9 / 3.3 | 4.1 / 10.6 | 0.2 | 8.7 | 4.03 (8/10) | 4.20 (4/10) | 833 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 4.6 / 10.9 | 2.0 / 5.6 | 4.5 / 10.7 | 0.1 | 11.1 | 1.95 (8/10) | 2.18 (4/10) | 1187 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 4.9 / 12.4 | 2.8 / 6.5 | 4.1 / 10.6 | 0.1 | 8.8 | 2.01 (8/10) | 2.27 (4/10) | 1178 | 13.23 |
| Ours v2 (as shipped) | 2.1 / 7.8 | 2.0 / 6.7 | 4.2 / 10.7 | 0.0 | 9.0 | 1.89 (8/10) | 2.04 (3/10) | 1623 | 2.05 |
| Ours v2 (as shipped, adaptive grip off) | 2.1 / 7.8 | 2.0 / 6.7 | 4.2 / 10.7 | 0.0 | 9.0 | 1.89 (8/10) | 2.04 (3/10) | 1623 | 1.99 |
| Ours v2 (tuned) | 2.1 / 7.8 | 2.0 / 6.7 | 4.2 / 10.7 | 0.0 | 9.0 | 1.89 (8/10) | 2.04 (3/10) | 1623 | 1.87 |
| PathPlanner path2 (new-path, pre-release), as shipped | 2.6 / 6.4 | 2.5 / 8.1 | 9.1 / 17.7 | 1.1 | 0.0 | 1.79 (2/10) | 1.86 (1/10) | 3287 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 3.6 / 9.8 | 2.2 / 5.5 | 4.3 / 10.7 | 0.1 | 10.2 | 2.62 (9/10) | 3.11 (4/10) | 1694 | 0.07 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.6 / 6.6 | 1.5 / 5.5 | 4.1 / 10.7 | 0.1 | 8.6 | 2.07 (8/10) | 2.21 (3/10) | 676 | 2.58 |

**hairpin**: 2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held (9.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 6.9 / 18.3 | 6.9 / 18.3 | 2.3 / 4.3 | 0.1 | 1.9 | 2.85 | 3.01 (4/10) | 1720 | 147.20 |
| BLine (README gains, r 0.30) | 45.5 / 104.7 | 46.0 / 107.9 | 6.1 / 8.2 | 0.1 | 6.8 | 6.54 (9/10) | never | 2825 | 0.09 |
| BLine (README gains, r 0.45 default) | 47.1 / 104.6 | 47.4 / 107.7 | 1.6 / 4.3 | 0.1 | 53.5 | 5.02 | 5.44 (7/10) | 1937 | 0.08 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 47.1 / 104.6 | 47.4 / 107.7 | 1.6 / 4.3 | 0.1 | 53.5 | 5.02 | 5.44 (7/10) | 1937 | 0.08 |
| BLine (tuned) | 14.8 / 38.9 | 15.4 / 39.8 | 3.9 / 5.3 | 0.1 | 0.0 | 3.96 (8/10) | never | 1142 | 0.08 |
| BLine (tuned, with corner caps) | 40.6 / 103.7 | 41.5 / 106.9 | 3.6 / 5.3 | 0.1 | 0.0 | 4.17 (9/10) | never | 1046 | 0.09 |
| PathPlanner (kP 5) | 20.0 / 26.8 | 20.0 / 26.8 | 33.8 / 38.8 | 0.1 | 0.0 | never | never | 3967 | 8.43 |
| PathPlanner (kP 5) + hold | 20.0 / 26.9 | 20.0 / 26.9 | 36.5 / 41.2 | 0.1 | 0.0 | never | never | 3968 | 5.48 |
| PathPlanner (tuned) + hold | 1.6 / 7.0 | 1.7 / 7.5 | 5.7 / 7.1 | 0.1 | 0.0 | 2.56 (6/10) | never | 3170 | 9.45 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.6 / 6.7 | 1.6 / 6.6 | 4.8 / 5.9 | 0.1 | 0.0 | 2.63 | never | 2586 | 4.84 |
| DriveToPose chain | 2.1 / 9.3 | 1.6 / 6.2 | 1.6 / 3.9 | 0.0 | 0.0 | 7.59 | 7.84 (7/10) | 689 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 2.4 / 8.6 | 2.3 / 8.5 | 1.8 / 4.0 | 0.1 | 0.6 | 2.87 | 3.07 (7/10) | 1834 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 2.9 / 9.8 | 2.8 / 9.7 | 2.8 / 4.3 | 0.0 | 0.0 | 3.01 | never | 1347 | 7.49 |
| Ours v2 (as shipped) | 3.1 / 7.2 | 3.1 / 7.2 | 8.2 / 9.9 | 0.1 | 0.0 | never | never | 3461 | 2.52 |
| Ours v2 (as shipped, adaptive grip off) | 3.1 / 7.2 | 3.1 / 7.2 | 8.2 / 9.9 | 0.1 | 0.0 | never | never | 3461 | 2.25 |
| Ours v2 (tuned) | 3.1 / 7.2 | 3.1 / 7.2 | 8.2 / 9.9 | 0.1 | 0.0 | never | never | 3461 | 2.29 |
| PathPlanner path2 (new-path, pre-release), as shipped | 8.9 / 19.8 | 9.1 / 22.8 | 13.7 / 16.8 | 0.1 | 0.0 | never | never | 5107 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 11.4 / 29.9 | 11.8 / 33.0 | 19.1 / 22.2 | 0.1 | 0.0 | never | never | 5047 | 0.07 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.6 / 6.3 | 1.5 / 6.3 | 3.0 / 4.6 | 0.1 | 0.0 | 2.95 | never | 1245 | 2.25 |

**slalom**: through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m (10.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.2 / 13.3 | 5.2 / 13.3 | 7.0 / 16.5 | 0.1 | 16.2 | 3.08 (9/10) | 3.27 (1/10) | 581 | 218.29 |
| BLine (README gains, r 0.30) | 37.0 / 82.6 | 36.2 / 78.8 | 23.1 / 36.8 | 0.2 | 42.3 | 4.07 | never | 2316 | 0.07 |
| BLine (README gains, r 0.45 default) | 29.7 / 69.1 | 29.8 / 64.5 | 23.4 / 34.7 | 0.1 | 33.3 | 3.85 | never | 2037 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 6.4 / 14.2 | 2.5 / 10.5 | 8.4 / 19.1 | 0.1 | 18.9 | 3.30 (9/10) | 3.43 (1/10) | 650 | 0.08 |
| BLine (tuned) | 11.1 / 35.6 | 8.1 / 31.8 | 5.7 / 15.0 | 0.1 | 11.1 | 3.12 (8/10) | 3.66 (1/10) | 338 | 0.06 |
| BLine (tuned, with corner caps) | 4.3 / 10.3 | 2.4 / 9.2 | 5.7 / 14.8 | 0.1 | 11.2 | 3.78 (8/10) | 4.33 (2/10) | 531 | 0.06 |
| PathPlanner (kP 5) | 3.2 / 9.6 | 3.1 / 9.5 | 8.6 / 17.6 | 0.1 | 17.7 | 2.39 (9/10) | never | 3519 | 15.09 |
| PathPlanner (kP 5) + hold | 3.2 / 9.6 | 3.1 / 9.5 | 6.9 / 14.6 | 0.1 | 17.7 | 2.39 (9/10) | never | 3520 | 11.33 |
| PathPlanner (tuned) + hold | 1.6 / 6.0 | 1.6 / 6.0 | 5.7 / 13.8 | 0.1 | 13.5 | 2.47 (8/10) | 2.67 (4/10) | 3437 | 7.23 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.4 / 4.9 | 1.3 / 4.9 | 5.6 / 13.1 | 0.1 | 16.0 | 2.56 (9/10) | 3.02 (4/10) | 3079 | 5.51 |
| DriveToPose chain | 4.2 / 11.3 | 1.1 / 3.8 | 5.4 / 14.1 | 0.0 | 11.7 | 4.65 (8/10) | 4.81 (3/10) | 1235 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 8.4 / 21.7 | 2.6 / 8.5 | 5.6 / 13.7 | 0.1 | 14.1 | 2.78 (7/10) | 3.00 (4/10) | 1965 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.6 / 22.9 | 3.5 / 9.1 | 5.8 / 14.3 | 0.3 | 11.6 | 2.88 (7/10) | 3.13 (2/10) | 1694 | 7.65 |
| Ours v2 (as shipped) | 2.0 / 7.5 | 2.1 / 7.4 | 5.5 / 14.6 | 0.1 | 11.4 | 2.55 (8/10) | 3.07 (2/10) | 2657 | 2.00 |
| Ours v2 (as shipped, adaptive grip off) | 2.0 / 7.5 | 2.1 / 7.4 | 5.6 / 14.6 | 0.1 | 11.4 | 2.43 (8/10) | 2.57 (2/10) | 2944 | 1.70 |
| Ours v2 (tuned) | 2.0 / 7.5 | 2.1 / 7.4 | 5.5 / 14.6 | 0.1 | 11.4 | 2.55 (8/10) | 3.07 (2/10) | 2657 | 1.64 |
| PathPlanner path2 (new-path, pre-release), as shipped | 5.3 / 22.0 | 5.6 / 27.0 | 10.6 / 22.9 | 0.2 | 0.4 | 2.29 (1/10) | 2.42 (1/10) | 5085 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 5.3 / 11.9 | 2.9 / 11.4 | 5.9 / 14.7 | 0.1 | 14.3 | 3.04 (9/10) | 3.59 (3/10) | 2918 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.6 / 7.1 | 1.6 / 6.4 | 5.6 / 14.5 | 0.1 | 11.6 | 2.87 (8/10) | 3.01 (3/10) | 1324 | 1.53 |

**long_auto**: auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end (14.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 6.1 / 25.3 | 6.1 / 25.3 | 8.7 / 18.8 | 2.7 | 6.6 | 4.11 (4/10) | never | 719 | 260.18 |
| BLine (README gains, r 0.30) | 33.0 / 99.6 | 37.3 / 111.9 | 15.3 / 29.5 | 1.8 | 63.2 | 5.73 (5/10) | 6.62 (1/10) | 2048 | 0.09 |
| BLine (README gains, r 0.45 default) | 31.9 / 90.9 | 35.0 / 102.0 | 8.8 / 19.9 | 1.2 | 52.1 | 5.71 (4/10) | 6.05 (1/10) | 2080 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 10.8 / 33.9 | 4.7 / 24.1 | 8.3 / 19.0 | 2.5 | 5.2 | 4.93 (4/10) | never | 762 | 0.07 |
| BLine (tuned) | 22.7 / 59.8 | 19.8 / 59.1 | 8.3 / 20.6 | 1.5 | 7.1 | 3.74 (5/10) | 4.15 (2/10) | 856 | 0.07 |
| BLine (tuned, with corner caps) | 10.0 / 32.7 | 5.9 / 22.7 | 8.2 / 19.1 | 1.2 | 1.5 | 5.29 (4/10) | 5.70 (1/10) | 532 | 0.09 |
| PathPlanner (kP 5) | 5.0 / 20.9 | 5.0 / 20.9 | 8.5 / 20.8 | 1.0 | 4.3 | 3.61 (4/10) | 3.74 (2/10) | 2189 | 12.76 |
| PathPlanner (kP 5) + hold | 5.0 / 20.9 | 5.0 / 20.9 | 9.3 / 21.1 | 0.9 | 4.3 | 3.61 (4/10) | never | 2190 | 13.04 |
| PathPlanner (tuned) + hold | 4.7 / 20.1 | 4.7 / 20.1 | 8.6 / 20.1 | 0.8 | 3.2 | 3.67 (4/10) | 3.80 (1/10) | 2150 | 11.05 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 4.8 / 20.3 | 4.8 / 20.3 | 8.5 / 20.1 | 0.9 | 6.0 | 3.72 (4/10) | 4.15 (1/10) | 1808 | 12.06 |
| DriveToPose chain | 8.1 / 32.0 | 4.5 / 18.8 | 7.9 / 17.7 | 0.9 | 4.7 | 6.54 (4/10) | 6.64 (3/10) | 1709 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 9.1 / 27.7 | 5.2 / 20.4 | 8.0 / 18.4 | 1.9 | 6.0 | 3.76 (4/10) | 4.00 (1/10) | 1810 | 0.02 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 9.7 / 28.9 | 6.0 / 22.5 | 8.1 / 18.9 | 1.0 | 3.7 | 3.84 (4/10) | 4.09 (2/10) | 1596 | 9.39 |
| Ours v2 (as shipped) | 6.5 / 21.5 | 4.9 / 20.1 | 8.5 / 20.1 | 0.9 | 2.0 | 3.99 (4/10) | 4.45 (2/10) | 1808 | 1.90 |
| Ours v2 (as shipped, adaptive grip off) | 6.5 / 21.5 | 4.9 / 20.1 | 8.5 / 20.1 | 0.9 | 2.0 | 3.65 (4/10) | 3.77 (1/10) | 1984 | 1.69 |
| Ours v2 (tuned) | 6.5 / 21.5 | 4.9 / 20.1 | 8.5 / 20.1 | 0.9 | 2.0 | 3.99 (4/10) | 4.45 (2/10) | 1808 | 1.43 |
| PathPlanner path2 (new-path, pre-release), as shipped | 8.9 / 35.5 | 11.0 / 47.2 | 9.9 / 24.0 | 0.9 | 2.7 | 3.62 (2/10) | 3.72 (1/10) | 3586 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 8.1 / 25.6 | 8.9 / 42.5 | 8.9 / 20.9 | 0.9 | 3.4 | 4.58 (4/10) | 4.88 (1/10) | 3040 | 0.05 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 6.6 / 21.6 | 4.9 / 19.5 | 8.1 / 19.3 | 1.0 | 2.8 | 3.94 (4/10) | 4.06 (2/10) | 748 | 2.27 |

**moving_start**: starts moving 2 m/s sideways (+y), path heads +x then curves left (7.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 12.3 / 27.1 | 12.3 / 27.1 | 5.1 / 11.0 | 0.2 | 10.4 | 1.88 (9/10) | 2.12 (2/10) | 1551 | 107.15 |
| BLine (README gains, r 0.30) | 30.8 / 59.3 | 31.9 / 65.9 | 23.7 / 30.1 | 0.1 | 30.0 | 3.44 | never | 2168 | 0.06 |
| BLine (README gains, r 0.45 default) | 33.1 / 54.9 | 34.2 / 51.6 | 19.2 / 31.9 | 0.4 | 56.0 | 2.14 | never | 1546 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 24.4 / 49.7 | 24.9 / 47.2 | 8.9 / 14.7 | 0.0 | 14.1 | 2.17 | never | 1227 | 0.05 |
| BLine (tuned) | 31.8 / 60.5 | 29.0 / 56.9 | 3.5 / 9.1 | 0.1 | 5.4 | 2.26 (8/10) | 2.75 (3/10) | 772 | 0.06 |
| BLine (tuned, with corner caps) | 18.8 / 47.8 | 19.5 / 45.7 | 3.7 / 9.6 | 0.1 | 5.0 | 3.07 (8/10) | 3.56 (3/10) | 822 | 0.07 |
| PathPlanner (kP 5) | 30.0 / 38.7 | 29.9 / 38.6 | 34.6 / 37.7 | 0.1 | 12.6 | never | never | 2476 | 5.19 |
| PathPlanner (kP 5) + hold | 30.0 / 38.7 | 29.9 / 38.6 | 34.1 / 37.9 | 0.1 | 12.6 | never | never | 2478 | 3.57 |
| PathPlanner (tuned) + hold | 32.7 / 43.1 | 32.7 / 43.0 | 39.0 / 42.4 | 0.1 | 10.9 | never | never | 2480 | 4.46 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 30.0 / 38.6 | 29.9 / 38.6 | 34.1 / 37.9 | 0.1 | 12.6 | never | never | 2453 | 6.38 |
| DriveToPose chain | 21.1 / 58.8 | 19.6 / 54.9 | 3.7 / 8.1 | 0.1 | 7.3 | 3.64 (9/10) | 3.84 (4/10) | 1310 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 14.5 / 20.2 | 15.7 / 20.8 | 19.9 / 23.6 | 0.1 | 23.0 | 1.59 (3/10) | never | 1986 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 10.4 / 27.3 | 10.7 / 28.7 | 3.6 / 10.0 | 0.3 | 5.2 | 1.84 (8/10) | 2.09 (3/10) | 1908 | 4.19 |
| Ours v2 (as shipped) | 9.3 / 23.3 | 9.5 / 23.3 | 4.1 / 9.9 | 0.1 | 6.2 | 1.71 (8/10) | never | 2054 | 1.28 |
| Ours v2 (as shipped, adaptive grip off) | 9.3 / 23.3 | 9.5 / 23.3 | 4.1 / 9.9 | 0.1 | 6.2 | 1.71 (8/10) | never | 2054 | 1.31 |
| Ours v2 (tuned) | 9.3 / 23.3 | 9.5 / 23.3 | 4.1 / 9.9 | 0.1 | 6.2 | 1.71 (8/10) | never | 2054 | 1.29 |
| PathPlanner path2 (new-path, pre-release), as shipped | 14.6 / 30.9 | 12.6 / 29.3 | 7.6 / 13.8 | 0.0 | 3.3 | 1.56 (3/10) | never | 3386 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 13.3 / 29.3 | 10.8 / 28.1 | 6.5 / 10.1 | 0.1 | 8.0 | 2.20 (5/10) | never | 2893 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 18.5 / 37.3 | 18.6 / 37.4 | 3.6 / 8.5 | 0.1 | 5.6 | 1.99 (8/10) | 2.35 (4/10) | 1629 | 1.20 |

**hit_mid_path**: the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 35.7 / 45.0 | 35.7 / 45.0 | 44.6 / 52.5 | 0.4 | 0.0 | never | never | 2332 | 130.65 |
| BLine (README gains, r 0.30) | 33.6 / 79.5 | 35.4 / 80.8 | 56.9 / 66.4 | 0.1 | 69.7 | never | never | 3282 | 0.05 |
| BLine (README gains, r 0.45 default) | 26.7 / 65.8 | 28.4 / 66.4 | 50.3 / 56.2 | 0.1 | 67.7 | never | never | 3088 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 18.5 / 27.3 | 19.4 / 29.9 | 26.5 / 27.8 | 0.1 | 2.4 | never | never | 2321 | 0.06 |
| BLine (tuned) | 57.4 / 66.2 | 59.2 / 67.0 | 69.7 / 70.8 | 0.1 | 0.0 | never | never | 2556 | 0.06 |
| BLine (tuned, with corner caps) | 24.4 / 31.3 | 25.6 / 37.1 | 33.0 / 38.0 | 0.1 | 0.0 | never | never | 2277 | 0.04 |
| PathPlanner (kP 5) | 44.8 / 50.5 | 44.8 / 50.5 | 51.8 / 54.9 | 0.2 | 0.0 | never | never | 3647 | 3.87 |
| PathPlanner (kP 5) + hold | 44.7 / 50.5 | 44.6 / 50.5 | 52.9 / 56.7 | 0.1 | 0.0 | never | never | 3650 | 4.99 |
| PathPlanner (tuned) + hold | 44.9 / 51.0 | 44.9 / 51.0 | 53.0 / 57.2 | 0.1 | 0.0 | never | never | 3709 | 6.40 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 42.9 / 48.6 | 42.8 / 48.6 | 51.1 / 54.6 | 0.1 | 0.0 | never | never | 3392 | 5.73 |
| DriveToPose chain | 22.0 / 38.7 | 23.7 / 42.8 | 39.3 / 49.2 | 0.2 | 0.0 | never | never | 2694 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 79.8 / 94.4 | 75.9 / 93.1 | 98.2 / 102.4 | 0.1 | 0.0 | never | never | 3614 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 77.1 / 94.2 | 73.3 / 93.2 | 94.8 / 102.6 | 0.3 | 0.0 | never | never | 3495 | 3.16 |
| Ours v2 (as shipped) | 45.8 / 51.8 | 46.1 / 52.4 | 53.8 / 60.3 | 0.2 | 0.0 | never | never | 3677 | 1.29 |
| Ours v2 (as shipped, adaptive grip off) | 43.9 / 49.7 | 44.2 / 50.2 | 51.6 / 58.0 | 0.3 | 0.0 | never | never | 3596 | 1.22 |
| Ours v2 (tuned) | 45.8 / 51.8 | 46.1 / 52.4 | 53.8 / 60.3 | 0.2 | 0.0 | never | never | 3677 | 1.28 |
| PathPlanner path2 (new-path, pre-release), as shipped | 44.0 / 49.6 | 45.7 / 55.1 | 56.4 / 59.3 | 1.4 | 0.0 | never | never | 4767 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 42.1 / 56.5 | 42.9 / 57.0 | 48.6 / 58.2 | 0.1 | 0.0 | never | never | 3192 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 33.5 / 40.4 | 33.8 / 40.9 | 41.3 / 49.5 | 0.2 | 0.0 | never | never | 2794 | 1.41 |

**low_grip_s_curve**: the S-curve on mu 0.91 static / 0.79 kinetic (61 % of normal grip) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.7 / 12.0 | 4.7 / 12.0 | 6.8 / 13.0 | 0.1 | 13.5 | 2.25 | never | 2493 | 78.57 |
| BLine (README gains, r 0.30) | 12.7 / 37.2 | 13.8 / 37.2 | 8.1 / 17.4 | 0.2 | 96.4 | 3.01 (8/10) | never | 5553 | 0.05 |
| BLine (README gains, r 0.45 default) | 7.9 / 26.6 | 8.8 / 26.7 | 6.5 / 14.9 | 0.1 | 91.4 | 2.93 (9/10) | never | 5222 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 9.8 | 1.7 / 7.0 | 15.6 / 24.4 | 0.1 | 24.2 | 2.22 | never | 2680 | 0.05 |
| BLine (tuned) | 6.8 / 16.7 | 4.8 / 15.9 | 4.4 / 12.1 | 0.1 | 7.2 | 2.69 (8/10) | 3.18 (2/10) | 2582 | 0.06 |
| BLine (tuned, with corner caps) | 2.8 / 6.6 | 3.0 / 10.5 | 4.3 / 11.6 | 0.1 | 7.7 | 2.73 (8/10) | 3.25 (2/10) | 2909 | 0.06 |
| PathPlanner (kP 5) | 4.8 / 9.8 | 4.7 / 9.7 | 13.0 / 22.0 | 0.2 | 21.9 | 1.83 | never | 4827 | 4.02 |
| PathPlanner (kP 5) + hold | 4.8 / 9.8 | 4.7 / 9.7 | 9.8 / 18.7 | 0.1 | 21.9 | 1.83 | never | 4858 | 3.98 |
| PathPlanner (tuned) + hold | 5.1 / 10.3 | 5.1 / 10.2 | 9.7 / 19.2 | 0.1 | 19.2 | 1.85 | never | 4811 | 4.44 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 2.4 / 5.6 | 2.4 / 5.6 | 4.3 / 10.9 | 0.1 | 13.6 | 1.91 (9/10) | 2.38 (4/10) | 4413 | 12.75 |
| DriveToPose chain | 2.9 / 9.1 | 0.9 / 3.5 | 4.2 / 11.1 | 0.3 | 7.5 | 3.99 (8/10) | 4.13 (2/10) | 5240 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 4.5 / 10.1 | 1.8 / 5.8 | 4.5 / 10.5 | 0.0 | 10.9 | 1.96 (9/10) | 2.18 (4/10) | 4558 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 4.8 / 11.5 | 2.6 / 5.8 | 4.2 / 11.1 | 0.2 | 8.2 | 2.02 (8/10) | 2.26 (3/10) | 4157 | 8.94 |
| Ours v2 (as shipped) | 2.0 / 7.0 | 1.9 / 6.0 | 4.1 / 10.1 | 0.1 | 9.3 | 2.94 (8/10) | 3.10 (4/10) | 306 | 1.33 |
| Ours v2 (as shipped, adaptive grip off) | 6.0 / 12.5 | 5.9 / 12.4 | 8.0 / 14.2 | 0.3 | 14.2 | 1.84 (8/10) | never | 4781 | 1.35 |
| Ours v2 (tuned) | 2.0 / 7.0 | 1.9 / 6.0 | 4.1 / 10.1 | 0.1 | 9.3 | 2.94 (8/10) | 3.10 (4/10) | 306 | 1.44 |
| PathPlanner path2 (new-path, pre-release), as shipped | 3.2 / 10.4 | 2.4 / 8.9 | 13.9 / 21.2 | 1.3 | 0.0 | never | never | 5863 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 7.5 / 18.1 | 6.8 / 15.2 | 8.2 / 17.0 | 0.1 | 15.9 | 2.47 (9/10) | never | 4310 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.6 / 6.6 | 1.5 / 5.5 | 4.1 / 11.0 | 0.1 | 8.1 | 2.08 (8/10) | 2.22 (3/10) | 3434 | 1.51 |

**grip_drop_s_curve**: the S-curve; halfway along, the floor drops from mu 1.5 / 1.3 to 0.91 / 0.79 (a worn carpet seam) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 11.7 | 4.6 / 11.7 | 7.8 / 15.9 | 0.2 | 15.6 | 2.24 | never | 1606 | 141.11 |
| BLine (README gains, r 0.30) | 13.1 / 37.6 | 14.2 / 37.7 | 7.4 / 15.5 | 0.2 | 98.5 | 3.02 (9/10) | never | 3739 | 0.06 |
| BLine (README gains, r 0.45 default) | 8.3 / 27.3 | 9.1 / 27.3 | 6.1 / 12.9 | 0.1 | 93.5 | 2.96 (9/10) | never | 3523 | 0.07 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 9.5 | 1.7 / 7.0 | 16.8 / 25.6 | 0.1 | 25.4 | 2.21 | never | 1526 | 0.07 |
| BLine (tuned) | 7.0 / 16.5 | 4.8 / 15.8 | 4.0 / 10.0 | 0.1 | 9.1 | 2.57 (8/10) | 3.17 (4/10) | 1135 | 0.08 |
| BLine (tuned, with corner caps) | 3.0 / 7.2 | 3.3 / 10.8 | 4.1 / 9.9 | 0.1 | 9.3 | 2.70 (9/10) | 3.25 (4/10) | 1308 | 0.06 |
| PathPlanner (kP 5) | 3.1 / 12.0 | 3.1 / 11.9 | 15.7 / 25.4 | 0.2 | 23.2 | 1.90 (1/10) | never | 3349 | 3.69 |
| PathPlanner (kP 5) + hold | 3.1 / 12.0 | 3.1 / 11.9 | 12.9 / 22.4 | 0.1 | 23.2 | 1.90 (1/10) | never | 3380 | 6.58 |
| PathPlanner (tuned) + hold | 3.1 / 12.2 | 3.1 / 12.1 | 12.3 / 22.9 | 0.1 | 20.5 | 1.93 (2/10) | never | 3379 | 4.11 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 2.4 / 8.7 | 2.3 / 8.7 | 7.7 / 14.8 | 0.1 | 15.7 | 1.96 (4/10) | never | 2910 | 3.88 |
| DriveToPose chain | 2.9 / 8.9 | 0.9 / 3.3 | 4.1 / 10.6 | 0.1 | 8.2 | 4.02 (8/10) | 4.16 (3/10) | 3020 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 4.7 / 10.9 | 2.0 / 5.9 | 5.5 / 12.8 | 0.0 | 13.0 | 1.94 (8/10) | 2.18 (1/10) | 2960 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.2 / 12.4 | 2.7 / 6.5 | 4.3 / 10.6 | 0.2 | 10.2 | 2.01 (8/10) | 2.26 (4/10) | 2624 | 4.44 |
| Ours v2 (as shipped) | 2.6 / 9.4 | 2.8 / 9.4 | 10.2 / 17.5 | 0.2 | 14.8 | never | never | 3386 | 1.44 |
| Ours v2 (as shipped, adaptive grip off) | 2.6 / 9.4 | 2.8 / 9.4 | 10.2 / 17.5 | 0.2 | 14.8 | never | never | 3389 | 1.36 |
| Ours v2 (tuned) | 2.6 / 9.4 | 2.8 / 9.4 | 10.2 / 17.5 | 0.2 | 14.8 | never | never | 3386 | 1.40 |
| PathPlanner path2 (new-path, pre-release), as shipped | 6.8 / 20.2 | 7.8 / 20.7 | 24.6 / 29.7 | 1.3 | 24.3 | never | never | 4448 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 7.3 / 15.8 | 8.1 / 15.9 | 16.1 / 23.2 | 0.1 | 16.9 | never | never | 2564 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.4 / 6.0 | 1.3 / 4.9 | 4.2 / 10.5 | 0.1 | 10.1 | 2.07 (9/10) | 2.22 (4/10) | 2156 | 1.52 |

**spin_while_driving**: 5 m straight while the robot turns a full 360 deg (a heading target at every waypoint) (8.2 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 2.1 / 6.8 | 2.1 / 6.8 | 5.8 / 11.6 | 14.3 | 11.4 | 2.20 (8/10) | never | 406 | 100.47 |
| BLine (README gains, r 0.30) | 2.7 / 7.6 | 2.7 / 7.6 | 5.8 / 12.3 | 3.1 | 63.5 | 1.93 (8/10) | never | 618 | 0.10 |
| BLine (README gains, r 0.45 default) | 3.1 / 7.6 | 3.1 / 7.6 | 6.0 / 12.5 | 3.1 | 63.1 | 1.94 (8/10) | never | 654 | 0.08 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.1 / 7.6 | 3.1 / 7.6 | 6.0 / 12.5 | 3.1 | 63.1 | 1.94 (8/10) | never | 654 | 0.08 |
| BLine (tuned) | 3.8 / 8.4 | 3.8 / 8.4 | 4.6 / 11.4 | 3.1 | 8.1 | 2.80 (8/10) | never | 150 | 0.08 |
| BLine (tuned, with corner caps) | 2.2 / 5.1 | 2.2 / 5.1 | 4.6 / 11.5 | 3.1 | 8.1 | 2.80 (8/10) | never | 152 | 0.07 |
| PathPlanner (kP 5) | 2.2 / 7.9 | 2.2 / 7.9 | 5.9 / 11.7 | 1.7 | 11.1 | 2.95 (8/10) | 3.11 (2/10) | 108 | 3.99 |
| PathPlanner (kP 5) + hold | 2.2 / 7.9 | 2.2 / 7.9 | 5.1 / 12.1 | 1.8 | 11.1 | 2.95 (8/10) | 3.36 (3/10) | 108 | 4.16 |
| PathPlanner (tuned) + hold | 1.8 / 7.1 | 1.8 / 7.1 | 5.2 / 12.1 | 1.9 | 10.3 | 2.94 (8/10) | 3.16 (2/10) | 219 | 4.11 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 2.1 / 7.9 | 2.1 / 7.9 | 5.1 / 11.9 | 1.8 | 11.5 | 2.94 (8/10) | 3.36 (3/10) | 166 | 3.94 |
| DriveToPose chain | 1.5 / 7.7 | 1.5 / 7.7 | 5.0 / 10.7 | 3.3 | 9.6 | 4.85 (8/10) | never | 935 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 3.1 / 8.2 | 2.2 / 7.2 | 5.5 / 10.9 | 5.5 | 10.6 | 1.94 (8/10) | never | 970 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.8 / 15.5 | 5.1 / 15.3 | 5.1 / 11.9 | 3.3 | 8.7 | 2.08 (8/10) | never | 494 | 4.72 |
| Ours v2 (as shipped) | 3.3 / 8.9 | 3.3 / 8.9 | 5.2 / 12.1 | 1.9 | 8.9 | 2.26 (8/10) | 2.86 (2/10) | 435 | 1.61 |
| Ours v2 (as shipped, adaptive grip off) | 3.3 / 8.9 | 3.3 / 8.9 | 5.2 / 12.1 | 1.9 | 8.9 | 2.26 (8/10) | 2.86 (2/10) | 437 | 1.60 |
| Ours v2 (tuned) | 3.3 / 8.9 | 3.3 / 8.9 | 5.2 / 12.1 | 1.9 | 8.9 | 2.26 (8/10) | 2.86 (2/10) | 435 | 1.48 |
| PathPlanner path2 (new-path, pre-release), as shipped | 3.7 / 8.7 | 3.7 / 8.7 | 12.4 / 21.0 | 2.9 | 0.0 | 2.16 (1/10) | never | 1357 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 2.9 / 8.8 | 2.9 / 8.8 | 6.1 / 13.2 | 1.8 | 5.9 | 2.82 (5/10) | 3.16 (1/10) | 1516 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 2.4 / 8.2 | 2.4 / 8.2 | 5.1 / 12.0 | 2.0 | 10.0 | 2.62 (8/10) | 3.12 (2/10) | 60 | 1.48 |

**figure_eight**: a self-crossing loop (a lemniscate, 4 m x 1.8 m) through the origin twice; heading held (12.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.5 / 15.4 | 5.5 / 15.4 | 2.0 / 4.2 | 0.1 | 3.5 | 5.12 | 5.32 (6/10) | 796 | 272.97 |
| BLine (README gains, r 0.30) | 43.5 / 117.8 | 43.8 / 117.8 | 2.3 / 3.9 | 0.2 | 62.4 | 9.88 | 9.97 (5/10) | 4139 | 0.14 |
| BLine (README gains, r 0.45 default) | 50.2 / 117.3 | 51.1 / 118.1 | 3.9 / 5.2 | 0.1 | 68.0 | 6.60 (9/10) | never | 3051 | 0.11 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 40.3 / 93.4 | 41.3 / 96.1 | 3.2 / 4.4 | 0.2 | 48.9 | 6.05 | 6.16 (1/10) | 2825 | 0.11 |
| BLine (tuned) | 56.2 / 154.0 | 56.9 / 155.7 | 2.8 / 4.2 | 0.1 | 0.0 | 3.85 | never | 1136 | 0.09 |
| BLine (tuned, with corner caps) | 43.6 / 96.5 | 44.6 / 98.4 | 2.7 / 4.1 | 0.1 | 0.0 | 6.01 | never | 2805 | 0.13 |
| PathPlanner (kP 5) | 8.0 / 26.4 | 8.0 / 26.6 | 25.4 / 26.8 | 0.1 | 0.4 | never | never | 6922 | 14.20 |
| PathPlanner (kP 5) + hold | 8.0 / 26.4 | 8.0 / 26.6 | 25.8 / 27.3 | 0.1 | 0.4 | never | never | 6930 | 9.29 |
| PathPlanner (tuned) + hold | 2.4 / 8.5 | 2.3 / 8.4 | 5.0 / 5.6 | 0.1 | 2.4 | 3.55 (4/10) | never | 6322 | 14.11 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.8 / 8.5 | 1.7 / 8.4 | 2.1 / 3.5 | 0.1 | 4.2 | 3.66 | 4.08 (6/10) | 5501 | 8.78 |
| DriveToPose chain | 2.1 / 10.8 | 1.5 / 8.0 | 15.8 / 44.4 | 0.0 | 1.2 | 12.36 (6/10) | 12.31 (1/10) | 2682 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 2.5 / 8.4 | 2.1 / 8.4 | 1.6 / 4.0 | 0.1 | 3.2 | 4.30 | 4.50 (7/10) | 2764 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 3.2 / 10.1 | 2.9 / 10.1 | 1.4 / 2.8 | 0.3 | 1.2 | 4.41 | 4.66 (8/10) | 2289 | 12.84 |
| Ours v2 (as shipped) | 3.0 / 12.6 | 3.0 / 12.3 | 2.3 / 4.0 | 0.1 | 1.2 | 3.56 | 3.69 (5/10) | 5681 | 2.12 |
| Ours v2 (as shipped, adaptive grip off) | 3.0 / 12.6 | 3.0 / 12.3 | 2.3 / 4.0 | 0.1 | 1.2 | 3.56 | 3.69 (5/10) | 5681 | 1.80 |
| Ours v2 (tuned) | 3.0 / 12.6 | 3.0 / 12.3 | 2.3 / 4.0 | 0.1 | 1.2 | 3.56 | 3.69 (5/10) | 5681 | 1.88 |
| PathPlanner path2 (new-path, pre-release), as shipped | 20.9 / 53.4 | 21.6 / 55.7 | 14.3 / 16.3 | 1.6 | 12.3 | never | never | 9483 | 0.09 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 21.6 / 54.5 | 21.9 / 56.7 | 26.8 / 28.7 | 0.1 | 3.0 | never | never | 10472 | 0.07 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 2.0 / 9.4 | 2.0 / 9.2 | 1.7 / 2.7 | 0.1 | 0.8 | 4.51 | 4.62 (8/10) | 1999 | 1.88 |

**wall_hug**: along a wall with the bumpers 15 cm off it, then a 90 deg turn to run 15 cm off the next wall (11.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.0 / 22.6 | 5.0 / 21.7 | 6.6 / 14.6 | 0.1 | 8.6 | 3.05 (4/10) | never | 911 | 169.49 |
| BLine (README gains, r 0.30) | 40.2 / 104.0 | 40.0 / 104.0 | 6.1 / 14.6 | 0.1 | 68.7 | 5.42 (6/10) | 5.70 (3/10) | 1910 | 0.09 |
| BLine (README gains, r 0.45 default) | 38.9 / 99.9 | 39.0 / 99.9 | 5.9 / 14.5 | 0.1 | 61.7 | 5.27 (6/10) | 5.56 (3/10) | 1834 | 0.09 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 38.9 / 99.9 | 39.0 / 99.9 | 5.9 / 14.5 | 0.1 | 61.7 | 5.27 (6/10) | 5.56 (3/10) | 1834 | 0.07 |
| BLine (tuned) | 20.9 / 59.9 | 20.9 / 59.9 | 6.3 / 13.4 | 0.1 | 3.6 | 2.96 (4/10) | never | 808 | 0.09 |
| BLine (tuned, with corner caps) | 40.6 / 102.8 | 40.8 / 102.8 | 5.3 / 12.7 | 0.1 | 9.0 | 3.85 (6/10) | 4.37 (4/10) | 958 | 0.09 |
| PathPlanner (kP 5) | 4.8 / 18.1 | 4.8 / 18.1 | 9.9 / 18.3 | 0.1 | 2.9 | 2.82 (1/10) | never | 2112 | 7.65 |
| PathPlanner (kP 5) + hold | 4.8 / 18.1 | 4.8 / 18.1 | 11.4 / 18.1 | 0.1 | 2.9 | 2.82 (1/10) | never | 2112 | 7.12 |
| PathPlanner (tuned) + hold | 3.5 / 14.0 | 3.5 / 14.0 | 7.2 / 14.4 | 0.2 | 3.8 | 2.82 (4/10) | never | 2048 | 7.08 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 2.8 / 12.5 | 2.8 / 12.5 | 6.6 / 13.0 | 0.1 | 6.2 | 2.87 (5/10) | never | 1710 | 6.37 |
| DriveToPose chain | 3.3 / 15.0 | 3.0 / 12.2 | 5.2 / 13.6 | 0.0 | 5.5 | 7.14 (7/10) | 7.30 (3/10) | 354 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 3.3 / 14.6 | 3.1 / 12.5 | 5.3 / 13.1 | 0.1 | 7.4 | 2.96 (5/10) | 3.17 (4/10) | 1338 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 4.1 / 15.9 | 3.8 / 14.0 | 5.6 / 13.4 | 0.2 | 4.3 | 3.07 (5/10) | 3.26 (3/10) | 1064 | 8.80 |
| Ours v2 (as shipped) | 3.5 / 13.6 | 3.5 / 13.3 | 7.0 / 13.6 | 0.1 | 2.7 | 2.79 (4/10) | never | 1851 | 1.53 |
| Ours v2 (as shipped, adaptive grip off) | 3.5 / 13.6 | 3.5 / 13.3 | 7.0 / 13.6 | 0.1 | 2.7 | 2.79 (4/10) | never | 1851 | 1.76 |
| Ours v2 (tuned) | 3.5 / 13.6 | 3.5 / 13.3 | 7.0 / 13.6 | 0.1 | 2.7 | 2.79 (4/10) | never | 1851 | 1.44 |
| PathPlanner path2 (new-path, pre-release), as shipped | 4.6 / 17.2 | 4.7 / 17.2 | 9.0 / 17.9 | 0.1 | 0.0 | never | never | 3707 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 7.3 / 31.4 | 7.7 / 31.4 | 8.8 / 17.4 | 0.1 | 3.3 | 3.81 (1/10) | never | 3849 | 0.07 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 3.2 / 12.7 | 3.2 / 12.5 | 5.7 / 13.2 | 0.1 | 4.3 | 3.04 (4/10) | 3.15 (3/10) | 795 | 1.61 |

**tag_align_moving_goal**: 5 m to a scoring pose; the true goal is 8 cm sideways and 3 deg off the map pose, revealed 1.5 m out (8.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 3.0 / 8.1 | 3.0 / 8.1 | 1.8 / 3.7 | 0.3 | 15.1 | 2.21 | 2.41 (5/10) | 463 | 117.94 |
| BLine (README gains, r 0.30) | 3.5 / 7.4 | 4.6 / 14.1 | 1.3 / 3.3 | 0.2 | 87.4 | 2.82 | 3.01 (8/10) | 524 | 0.07 |
| BLine (README gains, r 0.45 default) | 3.3 / 7.4 | 2.3 / 8.5 | 1.2 / 3.3 | 0.1 | 86.9 | 2.80 | 2.99 (8/10) | 482 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 8.1 | 0.8 / 4.2 | 1.3 / 3.0 | 0.3 | 57.8 | 2.72 | 2.91 (8/10) | 316 | 0.06 |
| BLine (tuned) | 6.4 / 17.0 | 3.9 / 16.1 | 1.1 / 3.1 | 0.3 | 9.1 | 3.33 | 3.58 (7/10) | 199 | 0.07 |
| BLine (tuned, with corner caps) | 3.4 / 8.2 | 1.0 / 5.4 | 1.3 / 3.0 | 0.3 | 9.3 | 3.37 | 3.60 (7/10) | 197 | 0.08 |
| PathPlanner (kP 5) | 3.0 / 7.8 | 3.0 / 7.8 | 3.1 / 5.8 | 0.1 | 6.0 | 1.80 | 1.98 (2/10) | 429 | 6.97 |
| PathPlanner (kP 5) + hold | 3.0 / 7.8 | 3.0 / 7.8 | 1.3 / 2.9 | 0.0 | 6.0 | 1.80 | 2.25 (7/10) | 433 | 4.33 |
| PathPlanner (tuned) + hold | 3.1 / 7.8 | 3.1 / 7.7 | 2.0 / 3.2 | 0.1 | 4.7 | 1.82 | 1.99 (5/10) | 555 | 3.32 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 3.0 / 7.8 | 3.0 / 7.8 | 1.3 / 2.9 | 0.0 | 6.0 | 1.80 | 2.25 (7/10) | 441 | 3.82 |
| DriveToPose chain | 4.3 / 8.7 | 2.5 / 7.6 | 1.0 / 2.8 | 0.3 | 2.4 | 2.62 | 2.81 (8/10) | 392 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 1.1 / 5.5 | 1.1 / 5.3 | 1.2 / 2.8 | 0.3 | 12.5 | 2.14 | 2.37 (7/10) | 709 | 0.02 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.8 / 6.9 | 1.9 / 6.7 | 1.2 / 3.2 | 0.3 | 3.2 | 1.84 | 2.10 (8/10) | 733 | 4.81 |
| Ours v2 (as shipped) | 2.8 / 7.7 | 2.9 / 7.7 | 0.9 / 3.7 | 0.2 | 1.8 | 1.89 | 2.03 (9/10) | 444 | 1.28 |
| Ours v2 (as shipped, adaptive grip off) | 2.8 / 7.7 | 2.9 / 7.7 | 0.9 / 3.7 | 0.2 | 1.8 | 1.82 | 1.95 (9/10) | 492 | 1.29 |
| Ours v2 (tuned) | 2.8 / 7.7 | 2.9 / 7.7 | 0.9 / 3.7 | 0.2 | 1.8 | 1.89 | 2.03 (9/10) | 444 | 1.37 |
| PathPlanner path2 (new-path, pre-release), as shipped | 5.2 / 11.0 | 1.8 / 4.5 | 3.7 / 6.4 | 0.3 | 6.4 | 1.88 (9/10) | 2.17 (2/10) | 1978 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 3.2 / 7.5 | 1.3 / 5.7 | 1.3 / 3.0 | 0.2 | 8.1 | 3.08 | 3.40 (7/10) | 714 | 0.10 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 2.8 / 7.6 | 2.9 / 7.6 | 1.2 / 2.8 | 0.2 | 2.7 | 1.84 | 1.98 (8/10) | 426 | 1.53 |

**intake_drive_through**: drive through a game piece at 2 m/s; its detected position moves 10 cm sideways 2 m out (7.4 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.9 / 2.3 | 0.9 / 2.3 | 11.7 / 15.2 | 0.1 | 11.7 | never | never | 499 | 90.46 |
| BLine (README gains, r 0.30) | 3.6 / 9.9 | 3.5 / 9.9 | 33.2 / 35.7 | 0.1 | 35.7 | 1.39 | never | 951 | 0.05 |
| BLine (README gains, r 0.45 default) | 3.7 / 9.8 | 3.5 / 9.9 | 33.0 / 35.8 | 0.1 | 35.8 | 1.39 | never | 941 | 0.04 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.7 / 9.8 | 3.5 / 9.9 | 33.0 / 35.8 | 0.1 | 35.8 | 1.39 | never | 941 | 0.05 |
| BLine (tuned) | 4.4 / 9.8 | 4.3 / 9.8 | 18.1 / 21.3 | 0.1 | 41.9 | 1.65 | never | 839 | 0.05 |
| BLine (tuned, with corner caps) | 4.3 / 9.9 | 4.2 / 9.9 | 18.1 / 21.1 | 0.1 | 42.0 | 1.65 | never | 859 | 0.06 |
| PathPlanner (kP 5) | 3.9 / 10.1 | 3.9 / 10.1 | 20.9 / 24.7 | 0.2 | 24.7 | 1.41 | never | 885 | 3.00 |
| PathPlanner (kP 5) + hold | 3.9 / 10.1 | 3.9 / 10.1 | 20.9 / 24.7 | 0.2 | 24.7 | 1.41 | never | 885 | 3.77 |
| PathPlanner (tuned) + hold | 3.9 / 10.1 | 3.9 / 10.1 | 19.1 / 23.4 | 0.2 | 23.4 | 1.42 | never | 1040 | 5.19 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 3.9 / 10.1 | 3.9 / 10.1 | 20.9 / 24.7 | 0.2 | 24.7 | 1.41 | never | 885 | 3.64 |
| DriveToPose chain | 3.7 / 9.8 | 3.6 / 9.8 | 1.6 / 4.2 | 0.2 | 3.3 | 2.34 | 2.53 (7/10) | 264 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 0.2 / 0.6 | 0.3 / 0.8 | 23.9 / 30.9 | 0.4 | 29.3 | never | never | 1053 | 0.02 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.7 / 8.9 | 1.7 / 9.0 | 1.7 / 4.1 | 0.3 | 4.1 | 1.60 | 1.85 (7/10) | 708 | 2.85 |
| Ours v2 (as shipped) | 4.0 / 9.9 | 4.0 / 9.9 | 18.5 / 23.7 | 0.2 | 23.7 | 1.47 | never | 903 | 0.87 |
| Ours v2 (as shipped, adaptive grip off) | 4.0 / 9.9 | 4.0 / 9.9 | 19.5 / 23.7 | 0.2 | 23.7 | 1.40 | never | 964 | 0.89 |
| Ours v2 (tuned) | 4.0 / 9.9 | 4.0 / 9.9 | 18.5 / 23.7 | 0.2 | 23.7 | 1.47 | never | 903 | 0.92 |
| PathPlanner path2 (new-path, pre-release), as shipped | 1.2 / 2.8 | 1.0 / 1.6 | 17.3 / 22.8 | 0.1 | 0.0 | never | never | 1599 | 0.05 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 1.2 / 6.5 | 1.5 / 7.2 | 10.8 / 14.0 | 0.1 | 5.5 | never | never | 807 | 0.05 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 3.9 / 10.0 | 3.9 / 10.0 | 20.3 / 23.6 | 0.2 | 23.6 | 1.42 | never | 884 | 1.18 |

**three_piece_chain**: three paths with rolling handoffs at 2 m/s (core-builder's neutral_three_piece, 2026 field) (15.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 7.6 / 28.9 | 5.5 / 21.9 | 5.0 / 12.7 | 3.5 | 11.9 | 5.64 (7/10) | never | 1658 | 59.25 |
| BLine (README gains, r 0.30) | 43.0 / 102.7 | 40.7 / 96.0 | 93.7 / 104.6 | 0.6 | 167.0 | never | never | 4726 | 0.07 |
| BLine (README gains, r 0.45 default) | 43.9 / 109.4 | 41.4 / 101.0 | 92.1 / 103.7 | 0.5 | 171.5 | never | never | 4723 | 0.07 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 28.2 / 62.8 | 26.2 / 57.4 | 45.4 / 53.7 | 1.9 | 7.9 | never | never | 2622 | 0.06 |
| BLine (tuned) | 46.7 / 206.7 | 45.0 / 203.5 | 86.3 / 188.0 | 0.9 | 398.2 | never | never | 3391 | 0.08 |
| BLine (tuned, with corner caps) | 44.5 / 189.3 | 42.6 / 184.7 | 96.5 / 183.0 | 0.8 | 398.2 | never | never | 3610 | 0.06 |
| PathPlanner (kP 5) | 4.7 / 15.3 | 4.6 / 15.1 | 6.2 / 14.8 | 0.3 | 13.0 | 4.58 (6/10) | 4.68 (2/10) | 3754 | 2.88 |
| PathPlanner (kP 5) + hold | 4.7 / 15.3 | 4.6 / 15.1 | 6.6 / 12.0 | 0.3 | 13.0 | 4.58 (6/10) | never | 3754 | 3.85 |
| PathPlanner (tuned) + hold | 4.8 / 16.4 | 4.7 / 16.6 | 7.4 / 13.8 | 0.3 | 11.5 | 4.67 (3/10) | 4.76 (1/10) | 3855 | 3.05 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 4.3 / 14.6 | 4.2 / 15.6 | 6.1 / 11.3 | 0.3 | 13.1 | 4.74 (6/10) | never | 2870 | 2.84 |
| DriveToPose chain | 6.3 / 31.6 | 3.6 / 15.2 | 5.3 / 12.2 | 1.8 | 12.7 | 10.73 (7/10) | never | 2353 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 7.7 / 23.3 | 4.0 / 14.0 | 5.3 / 13.0 | 2.8 | 12.2 | 4.45 (8/10) | never | 2450 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 7.5 / 25.4 | 4.0 / 16.3 | 5.5 / 11.5 | 0.4 | 10.2 | 4.52 (6/10) | 4.78 (2/10) | 1963 | 9.51 |
| Ours v2 (as shipped) | 4.5 / 21.8 | 4.1 / 15.2 | 6.1 / 11.0 | 0.3 | 7.5 | 4.64 (4/10) | never | 3282 | 1.25 |
| Ours v2 (as shipped, adaptive grip off) | 4.5 / 21.8 | 4.2 / 15.2 | 6.2 / 11.0 | 0.3 | 9.3 | 4.64 (4/10) | never | 3290 | 1.23 |
| Ours v2 (tuned) | 4.5 / 21.8 | 4.1 / 15.2 | 6.1 / 11.0 | 0.3 | 7.5 | 4.64 (4/10) | never | 3282 | 1.17 |
| PathPlanner path2 (new-path, pre-release), as shipped | 8.5 / 26.2 | 8.1 / 27.0 | 10.7 / 22.1 | 1.9 | 19.9 | 3.95 (8/10) | never | 7162 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 6.8 / 21.9 | 5.1 / 20.5 | 6.6 / 12.3 | 0.3 | 7.6 | 5.18 (4/10) | never | 5862 | 0.07 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 5.0 / 22.7 | 4.2 / 16.3 | 5.5 / 11.5 | 0.3 | 9.9 | 5.04 (6/10) | 5.17 (2/10) | 882 | 1.77 |

**fast_long**: an 8 m gentle curve with a 5 m/s cap (above the robot's 4.57 m/s free speed) (10.8 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 1.4 / 5.3 | 1.4 / 5.3 | 7.8 / 18.4 | 0.1 | 18.0 | 2.48 (9/10) | 2.68 (1/10) | 508 | 184.06 |
| BLine (README gains, r 0.30) | 1.8 / 5.4 | 1.4 / 5.3 | 35.5 / 73.7 | 0.7 | 130.9 | 2.20 | never | 1154 | 0.07 |
| BLine (README gains, r 0.45 default) | 2.3 / 5.8 | 1.0 / 4.1 | 40.1 / 73.3 | 0.7 | 131.3 | 2.20 | never | 1170 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 2.4 / 5.9 | 1.0 / 4.0 | 40.1 / 73.2 | 0.7 | 131.4 | 2.20 | never | 1173 | 0.06 |
| BLine (tuned) | 3.9 / 9.9 | 2.8 / 9.7 | 6.5 / 17.0 | 0.1 | 12.6 | 3.05 (8/10) | 3.60 (1/10) | 240 | 0.07 |
| BLine (tuned, with corner caps) | 2.2 / 6.2 | 1.5 / 5.2 | 6.5 / 16.8 | 0.1 | 12.7 | 3.01 (8/10) | 3.54 (1/10) | 287 | 0.06 |
| PathPlanner (kP 5) | 1.0 / 3.9 | 1.0 / 3.9 | 7.4 / 18.1 | 0.1 | 17.8 | 2.38 (9/10) | 2.60 (2/10) | 787 | 7.86 |
| PathPlanner (kP 5) + hold | 1.0 / 3.9 | 1.0 / 3.9 | 6.5 / 15.9 | 0.1 | 17.8 | 2.38 (9/10) | 2.84 (3/10) | 789 | 7.12 |
| PathPlanner (tuned) + hold | 1.1 / 4.1 | 1.1 / 4.1 | 7.1 / 17.3 | 0.1 | 17.1 | 2.38 (8/10) | 2.66 (4/10) | 850 | 7.25 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.0 / 3.9 | 1.0 / 3.9 | 6.5 / 15.9 | 0.1 | 17.8 | 2.38 (9/10) | 2.84 (3/10) | 789 | 6.37 |
| DriveToPose chain | 2.2 / 6.4 | 1.1 / 4.0 | 6.2 / 16.1 | 0.0 | 13.4 | 4.88 (8/10) | 5.03 (3/10) | 618 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 1.3 / 4.0 | 1.4 / 4.1 | 7.0 / 17.8 | 0.1 | 18.2 | 2.36 (8/10) | 2.61 (3/10) | 870 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 6.8 / 15.8 | 6.9 / 16.0 | 6.3 / 15.9 | 0.3 | 13.5 | 2.45 (8/10) | 2.72 (3/10) | 915 | 6.63 |
| Ours v2 (as shipped) | 1.2 / 4.0 | 1.2 / 4.1 | 6.3 / 16.4 | 0.1 | 13.3 | 2.54 (8/10) | 3.05 (2/10) | 454 | 1.36 |
| Ours v2 (as shipped, adaptive grip off) | 1.2 / 4.0 | 1.2 / 4.1 | 6.4 / 16.4 | 0.1 | 13.3 | 2.41 (8/10) | 2.56 (2/10) | 504 | 1.96 |
| Ours v2 (tuned) | 1.2 / 4.0 | 1.2 / 4.1 | 6.3 / 16.4 | 0.1 | 13.3 | 2.54 (8/10) | 3.05 (2/10) | 454 | 1.47 |
| PathPlanner path2 (new-path, pre-release), as shipped | 5.2 / 10.6 | 4.6 / 8.6 | 13.0 / 27.7 | 0.1 | 0.7 | 2.21 (1/10) | never | 2217 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 2.1 / 7.5 | 2.6 / 9.2 | 6.8 / 17.0 | 0.1 | 12.6 | 3.17 (8/10) | never | 1718 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.2 / 4.0 | 1.1 / 4.1 | 6.4 / 16.3 | 0.1 | 13.4 | 2.44 (8/10) | 2.58 (3/10) | 442 | 1.50 |

**heavy_robot**: the S-curve with a 68 kg robot (MOI 6.8) and a 192 A current limit (20 % lower) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 12.0 | 4.6 / 12.0 | 6.5 / 14.0 | 0.1 | 13.8 | 2.24 (9/10) | 2.44 (1/10) | 442 | 122.24 |
| BLine (README gains, r 0.30) | 13.2 / 36.8 | 14.3 / 36.9 | 6.0 / 13.9 | 0.1 | 97.8 | 3.02 (8/10) | never | 1116 | 0.06 |
| BLine (README gains, r 0.45 default) | 8.1 / 26.4 | 8.9 / 26.5 | 5.7 / 12.5 | 0.1 | 92.6 | 2.94 (9/10) | never | 950 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.3 / 10.4 | 1.7 / 7.7 | 10.9 / 19.8 | 0.1 | 19.6 | 2.22 | never | 574 | 0.06 |
| BLine (tuned) | 6.7 / 16.6 | 4.7 / 15.8 | 4.2 / 11.1 | 0.1 | 8.2 | 2.61 (8/10) | 3.16 (3/10) | 454 | 0.06 |
| BLine (tuned, with corner caps) | 2.9 / 7.3 | 3.2 / 10.8 | 4.2 / 10.9 | 0.1 | 8.4 | 2.68 (8/10) | 3.24 (3/10) | 378 | 0.06 |
| PathPlanner (kP 5) | 1.5 / 4.5 | 1.4 / 4.5 | 6.1 / 13.0 | 0.1 | 12.9 | 1.87 (9/10) | never | 1526 | 4.09 |
| PathPlanner (kP 5) + hold | 1.5 / 4.5 | 1.4 / 4.5 | 4.2 / 10.2 | 0.1 | 12.9 | 1.87 (9/10) | 2.36 (4/10) | 1529 | 3.87 |
| PathPlanner (tuned) + hold | 1.5 / 4.5 | 1.5 / 4.5 | 4.9 / 11.1 | 0.1 | 10.9 | 1.90 (9/10) | 2.17 (4/10) | 1582 | 4.08 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.3 / 4.3 | 1.3 / 4.2 | 4.1 / 10.4 | 0.1 | 12.5 | 1.91 (9/10) | 2.39 (4/10) | 1295 | 4.08 |
| DriveToPose chain | 2.9 / 8.8 | 0.8 / 3.3 | 4.0 / 10.1 | 0.2 | 8.7 | 3.88 (8/10) | 4.01 (4/10) | 895 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 4.6 / 11.1 | 2.0 / 5.6 | 4.5 / 10.8 | 0.1 | 11.3 | 1.95 (8/10) | 2.18 (4/10) | 1215 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.1 / 12.6 | 2.7 / 6.8 | 4.1 / 10.4 | 0.1 | 8.9 | 2.01 (8/10) | 2.25 (4/10) | 1161 | 4.42 |
| Ours v2 (as shipped) | 2.1 / 7.7 | 2.0 / 6.7 | 4.2 / 10.4 | 0.0 | 9.2 | 1.89 (8/10) | 2.05 (4/10) | 1445 | 1.48 |
| Ours v2 (as shipped, adaptive grip off) | 2.1 / 7.7 | 2.0 / 6.7 | 4.2 / 10.4 | 0.0 | 9.2 | 1.89 (8/10) | 2.05 (4/10) | 1445 | 1.35 |
| Ours v2 (tuned) | 2.1 / 7.7 | 2.0 / 6.7 | 4.2 / 10.4 | 0.0 | 9.2 | 1.89 (8/10) | 2.05 (4/10) | 1445 | 1.71 |
| PathPlanner path2 (new-path, pre-release), as shipped | 2.7 / 6.3 | 2.7 / 10.4 | 8.8 / 18.1 | 1.0 | 0.7 | 1.79 (3/10) | 1.86 (1/10) | 3183 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 3.6 / 9.9 | 2.3 / 5.6 | 4.4 / 10.8 | 0.1 | 10.2 | 2.60 (9/10) | 3.10 (4/10) | 1643 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.6 / 6.6 | 1.5 / 5.5 | 4.1 / 10.7 | 0.1 | 9.0 | 2.07 (8/10) | 2.21 (4/10) | 579 | 1.23 |

### Slip-aware odometry + gyro + IMU (lib's SlipAwareOdometry, no vision)

| method | runs settled | courses settled every seed | mean RMS cross-track cm | worst cross-track cm | mean final error cm | mean wheel slip ms |
|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 11/90 | 0/9 | 6.3 | 39.0 | 6.3 | 1417 |
| BLine (README gains, r 0.30) | 2/90 | 0/9 | 24.2 | 104.7 | 16.3 | 2751 |
| BLine (README gains, r 0.45 default) | 8/90 | 0/9 | 21.1 | 104.6 | 12.2 | 2414 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 8/90 | 0/9 | 12.2 | 104.6 | 12.2 | 1423 |
| BLine (tuned) | 22/90 | 0/9 | 14.1 | 60.5 | 4.9 | 1168 |
| BLine (tuned, with corner caps) | 22/90 | 0/9 | 10.1 | 103.7 | 4.7 | 1154 |
| PathPlanner (kP 5) | 4/90 | 0/9 | 7.8 | 48.6 | 10.2 | 3298 |
| PathPlanner (kP 5) + hold | 10/90 | 0/9 | 7.8 | 48.6 | 8.8 | 3347 |
| PathPlanner (tuned) + hold | 11/90 | 0/9 | 6.6 | 48.4 | 9.4 | 3294 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 18/90 | 0/9 | 6.3 | 47.1 | 8.0 | 2754 |
| DriveToPose chain | 34/90 | 0/9 | 5.9 | 58.8 | 4.2 | 1892 |
| Choreo + ChoreoLib follower (as shipped) | 26/90 | 0/9 | 9.7 | 92.3 | 8.2 | 2527 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 24/90 | 0/9 | 10.2 | 95.8 | 4.9 | 2300 |
| Ours v2 (as shipped) | 20/90 | 0/9 | 5.0 | 49.7 | 5.9 | 2387 |
| Ours v2 (as shipped, adaptive grip off) | 15/90 | 0/9 | 4.9 | 47.7 | 7.4 | 3027 |
| Ours v2 (tuned) | 20/90 | 0/9 | 5.0 | 49.7 | 5.9 | 2387 |
| PathPlanner path2 (new-path, pre-release), as shipped | 0/90 | 0/9 | 11.1 | 53.7 | 19.6 | 4247 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 9/90 | 0/9 | 7.9 | 42.3 | 9.6 | 3240 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 25/90 | 0/9 | 5.3 | 39.7 | 4.6 | 1615 |

**straight_3m**: 3 m straight, heading held at 0 (6.5 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.1 / 0.3 | 0.1 / 0.3 | 5.0 / 9.8 | 0.1 | 9.8 | 1.35 | 1.53 (2/10) | 501 | 72.72 |
| BLine (README gains, r 0.30) | 0.1 / 0.3 | 0.1 / 0.3 | 30.3 / 39.3 | 0.1 | 69.4 | 1.13 | never | 742 | 0.07 |
| BLine (README gains, r 0.45 default) | 0.1 / 0.3 | 0.1 / 0.3 | 30.3 / 39.3 | 0.1 | 69.4 | 1.13 | never | 742 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 0.1 / 0.3 | 0.1 / 0.3 | 30.3 / 39.3 | 0.1 | 69.4 | 1.13 | never | 742 | 0.06 |
| BLine (tuned) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.8 | 0.1 | 4.5 | 2.18 (9/10) | 2.67 (7/10) | 6 | 0.05 |
| BLine (tuned, with corner caps) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.8 | 0.1 | 4.5 | 2.18 (9/10) | 2.67 (7/10) | 6 | 0.06 |
| PathPlanner (kP 5) | 0.1 / 0.3 | 0.1 / 0.3 | 4.2 / 8.7 | 0.1 | 8.8 | 1.26 | 1.46 (1/10) | 470 | 2.70 |
| PathPlanner (kP 5) + hold | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 8.8 | 1.26 | 1.72 (4/10) | 471 | 2.56 |
| PathPlanner (tuned) + hold | 0.1 / 0.3 | 0.1 / 0.3 | 2.9 / 6.8 | 0.1 | 7.0 | 1.28 | 1.45 (4/10) | 570 | 2.45 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.8 | 0.1 | 8.8 | 1.26 | 1.72 (4/10) | 471 | 2.55 |
| DriveToPose chain | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.0 | 5.2 | 1.33 (9/10) | 1.52 (5/10) | 9 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 0.1 / 0.3 | 0.1 / 0.3 | 3.0 / 6.8 | 0.1 | 7.7 | 1.26 | 1.48 (5/10) | 667 | 0.02 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 0.1 / 0.3 | 0.1 / 0.3 | 2.4 / 6.5 | 0.3 | 4.8 | 1.33 (9/10) | 1.58 (5/10) | 531 | 2.81 |
| Ours v2 (as shipped) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.4 | 0.1 | 5.0 | 1.30 (9/10) | 1.43 (6/10) | 368 | 2.61 |
| Ours v2 (as shipped, adaptive grip off) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.4 | 0.1 | 5.0 | 1.30 (9/10) | 1.43 (6/10) | 368 | 1.88 |
| Ours v2 (tuned) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.4 | 0.1 | 5.0 | 1.30 (9/10) | 1.43 (6/10) | 368 | 1.79 |
| PathPlanner path2 (new-path, pre-release), as shipped | 0.7 / 1.6 | 0.7 / 1.6 | 12.8 / 19.3 | 0.1 | 0.0 | never | never | 1610 | 0.10 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 0.1 / 0.3 | 0.1 / 0.3 | 2.7 / 7.7 | 0.1 | 3.5 | 2.18 (9/10) | 2.53 (4/10) | 59 | 0.05 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 0.1 / 0.3 | 0.1 / 0.3 | 2.4 / 6.2 | 0.1 | 5.1 | 1.32 (9/10) | 1.45 (5/10) | 226 | 1.60 |

**s_curve**: S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.5 / 12.0 | 4.5 / 12.0 | 5.8 / 13.4 | 0.1 | 13.2 | 2.24 (9/10) | never | 452 | 145.99 |
| BLine (README gains, r 0.30) | 13.0 / 37.3 | 14.1 / 37.4 | 5.9 / 13.4 | 0.1 | 97.2 | 3.01 (8/10) | never | 1085 | 0.07 |
| BLine (README gains, r 0.45 default) | 8.2 / 27.0 | 9.0 / 27.0 | 5.4 / 12.0 | 0.1 | 91.9 | 2.94 (9/10) | never | 972 | 0.07 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 9.7 | 1.8 / 7.1 | 10.3 / 18.7 | 0.1 | 18.5 | 2.22 | never | 576 | 0.08 |
| BLine (tuned) | 6.7 / 16.5 | 4.7 / 15.8 | 4.2 / 11.2 | 0.1 | 8.2 | 2.63 (8/10) | 3.17 (3/10) | 446 | 0.07 |
| BLine (tuned, with corner caps) | 2.9 / 7.2 | 3.2 / 10.5 | 4.2 / 11.0 | 0.1 | 8.4 | 2.69 (8/10) | 3.26 (3/10) | 385 | 0.07 |
| PathPlanner (kP 5) | 1.5 / 4.6 | 1.4 / 4.6 | 5.8 / 12.7 | 0.1 | 12.5 | 1.87 (9/10) | never | 1562 | 3.90 |
| PathPlanner (kP 5) + hold | 1.5 / 4.6 | 1.4 / 4.6 | 4.2 / 10.2 | 0.1 | 12.5 | 1.87 (9/10) | 2.34 (4/10) | 1565 | 3.83 |
| PathPlanner (tuned) + hold | 1.5 / 4.5 | 1.4 / 4.5 | 4.8 / 11.0 | 0.1 | 10.7 | 1.90 (9/10) | 2.15 (3/10) | 1605 | 4.22 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.3 / 4.3 | 1.2 / 4.3 | 4.1 / 10.4 | 0.1 | 12.1 | 1.91 (9/10) | 2.37 (4/10) | 1366 | 4.24 |
| DriveToPose chain | 2.9 / 9.0 | 0.9 / 3.3 | 4.1 / 10.6 | 0.2 | 8.7 | 4.03 (8/10) | 4.20 (4/10) | 833 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 4.6 / 10.9 | 2.0 / 5.6 | 4.5 / 10.7 | 0.1 | 11.1 | 1.95 (8/10) | 2.18 (4/10) | 1187 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 4.9 / 12.4 | 2.8 / 6.5 | 4.1 / 10.6 | 0.1 | 8.8 | 2.01 (8/10) | 2.27 (4/10) | 1178 | 4.83 |
| Ours v2 (as shipped) | 2.1 / 7.8 | 2.0 / 6.7 | 4.2 / 10.7 | 0.0 | 9.0 | 1.89 (8/10) | 2.04 (3/10) | 1623 | 3.36 |
| Ours v2 (as shipped, adaptive grip off) | 2.1 / 7.8 | 2.0 / 6.7 | 4.2 / 10.7 | 0.0 | 9.0 | 1.89 (8/10) | 2.04 (3/10) | 1623 | 2.20 |
| Ours v2 (tuned) | 2.1 / 7.8 | 2.0 / 6.7 | 4.2 / 10.7 | 0.0 | 9.0 | 1.89 (8/10) | 2.04 (3/10) | 1623 | 2.22 |
| PathPlanner path2 (new-path, pre-release), as shipped | 4.3 / 11.5 | 4.2 / 11.4 | 10.6 / 17.1 | 1.6 | 0.0 | never | never | 3504 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 4.1 / 11.7 | 3.0 / 10.2 | 5.2 / 11.9 | 0.1 | 8.0 | 2.67 (6/10) | 3.12 (2/10) | 1931 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.6 / 6.6 | 1.5 / 5.5 | 4.1 / 10.7 | 0.1 | 8.6 | 2.07 (8/10) | 2.21 (3/10) | 676 | 4.79 |

**hairpin**: 2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held (9.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 6.9 / 18.3 | 6.9 / 18.3 | 2.3 / 4.3 | 0.1 | 1.9 | 2.85 | 3.01 (4/10) | 1720 | 168.62 |
| BLine (README gains, r 0.30) | 45.5 / 104.7 | 46.0 / 107.9 | 6.1 / 8.2 | 0.1 | 6.8 | 6.54 (9/10) | never | 2825 | 0.09 |
| BLine (README gains, r 0.45 default) | 47.1 / 104.6 | 47.4 / 107.7 | 1.6 / 4.3 | 0.1 | 53.5 | 5.02 | 5.44 (7/10) | 1937 | 0.08 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 47.1 / 104.6 | 47.4 / 107.7 | 1.6 / 4.3 | 0.1 | 53.5 | 5.02 | 5.44 (7/10) | 1937 | 0.09 |
| BLine (tuned) | 14.8 / 38.9 | 15.4 / 39.8 | 3.9 / 5.3 | 0.1 | 0.0 | 3.96 (8/10) | never | 1142 | 0.09 |
| BLine (tuned, with corner caps) | 40.6 / 103.7 | 41.5 / 106.9 | 3.6 / 5.3 | 0.1 | 0.0 | 4.17 (9/10) | never | 1046 | 0.12 |
| PathPlanner (kP 5) | 11.5 / 31.7 | 11.6 / 31.9 | 5.4 / 11.3 | 0.6 | 10.7 | 2.37 (8/10) | 2.51 (1/10) | 4954 | 5.68 |
| PathPlanner (kP 5) + hold | 11.5 / 31.7 | 11.6 / 31.9 | 5.0 / 8.9 | 0.1 | 10.7 | 2.37 (8/10) | 2.77 (1/10) | 4964 | 5.76 |
| PathPlanner (tuned) + hold | 1.6 / 6.9 | 1.7 / 7.3 | 5.4 / 7.0 | 0.1 | 0.0 | 2.55 (7/10) | never | 3183 | 6.62 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.6 / 6.7 | 1.6 / 6.6 | 4.8 / 5.9 | 0.1 | 0.0 | 2.63 | never | 2586 | 7.02 |
| DriveToPose chain | 2.1 / 9.3 | 1.6 / 6.2 | 1.6 / 3.9 | 0.0 | 0.0 | 7.59 | 7.84 (7/10) | 689 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 2.4 / 8.6 | 2.3 / 8.5 | 1.8 / 4.0 | 0.1 | 0.6 | 2.87 | 3.07 (7/10) | 1834 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 2.9 / 9.8 | 2.8 / 9.7 | 2.8 / 4.3 | 0.0 | 0.0 | 3.01 | never | 1347 | 9.41 |
| Ours v2 (as shipped) | 4.5 / 9.1 | 4.4 / 9.0 | 8.5 / 12.4 | 0.1 | 0.0 | never | never | 3467 | 2.65 |
| Ours v2 (as shipped, adaptive grip off) | 4.5 / 9.1 | 4.4 / 9.0 | 8.5 / 12.4 | 0.1 | 0.0 | never | never | 3467 | 2.33 |
| Ours v2 (tuned) | 4.5 / 9.1 | 4.4 / 9.0 | 8.5 / 12.4 | 0.1 | 0.0 | never | never | 3467 | 2.35 |
| PathPlanner path2 (new-path, pre-release), as shipped | 14.6 / 23.8 | 14.8 / 26.9 | 23.7 / 27.0 | 0.3 | 0.0 | never | never | 5201 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 17.8 / 32.3 | 18.0 / 35.5 | 25.1 / 35.2 | 0.1 | 0.0 | never | never | 5462 | 0.09 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.6 / 6.3 | 1.5 / 6.3 | 3.0 / 4.6 | 0.1 | 0.0 | 2.95 | never | 1245 | 2.27 |

**slalom**: through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m (10.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.2 / 13.3 | 5.2 / 13.3 | 7.0 / 16.5 | 0.1 | 16.2 | 3.08 (9/10) | 3.27 (1/10) | 581 | 156.30 |
| BLine (README gains, r 0.30) | 37.0 / 82.6 | 36.2 / 78.8 | 23.1 / 36.8 | 0.2 | 42.3 | 4.07 | never | 2316 | 0.08 |
| BLine (README gains, r 0.45 default) | 29.7 / 69.1 | 29.8 / 64.5 | 23.4 / 34.7 | 0.1 | 33.3 | 3.85 | never | 2037 | 0.07 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 6.4 / 14.2 | 2.5 / 10.5 | 8.4 / 19.1 | 0.1 | 18.9 | 3.30 (9/10) | 3.43 (1/10) | 650 | 0.07 |
| BLine (tuned) | 11.1 / 35.6 | 8.1 / 31.8 | 5.7 / 15.0 | 0.1 | 11.1 | 3.12 (8/10) | 3.66 (1/10) | 338 | 0.07 |
| BLine (tuned, with corner caps) | 4.3 / 10.3 | 2.4 / 9.2 | 5.7 / 14.8 | 0.1 | 11.2 | 3.78 (8/10) | 4.33 (2/10) | 531 | 0.07 |
| PathPlanner (kP 5) | 1.9 / 7.6 | 1.9 / 7.7 | 6.8 / 15.3 | 0.1 | 14.5 | 2.43 (8/10) | never | 3661 | 6.21 |
| PathPlanner (kP 5) + hold | 1.9 / 7.6 | 1.9 / 7.7 | 6.6 / 14.9 | 0.1 | 14.5 | 2.43 (8/10) | never | 3663 | 4.90 |
| PathPlanner (tuned) + hold | 1.5 / 5.5 | 1.4 / 5.4 | 6.0 / 14.2 | 0.1 | 13.4 | 2.47 (8/10) | 2.71 (2/10) | 3535 | 5.07 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.4 / 4.9 | 1.3 / 4.9 | 5.6 / 13.1 | 0.1 | 16.0 | 2.56 (9/10) | 3.02 (4/10) | 3079 | 5.20 |
| DriveToPose chain | 4.2 / 11.3 | 1.1 / 3.8 | 5.4 / 14.1 | 0.0 | 11.7 | 4.65 (8/10) | 4.81 (3/10) | 1235 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 8.4 / 21.7 | 2.6 / 8.5 | 5.6 / 13.7 | 0.1 | 14.1 | 2.78 (7/10) | 3.00 (4/10) | 1965 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.6 / 22.9 | 3.5 / 9.1 | 5.8 / 14.3 | 0.3 | 11.6 | 2.88 (7/10) | 3.13 (2/10) | 1694 | 7.92 |
| Ours v2 (as shipped) | 2.0 / 7.5 | 2.1 / 7.4 | 5.5 / 14.6 | 0.1 | 11.4 | 2.55 (8/10) | 3.07 (2/10) | 2657 | 1.42 |
| Ours v2 (as shipped, adaptive grip off) | 2.0 / 7.5 | 2.1 / 7.4 | 5.6 / 14.6 | 0.1 | 11.4 | 2.43 (8/10) | 2.57 (2/10) | 2944 | 1.19 |
| Ours v2 (tuned) | 2.0 / 7.5 | 2.1 / 7.4 | 5.5 / 14.6 | 0.1 | 11.4 | 2.55 (8/10) | 3.07 (2/10) | 2657 | 1.02 |
| PathPlanner path2 (new-path, pre-release), as shipped | 11.4 / 24.4 | 11.5 / 30.4 | 14.2 / 22.0 | 0.4 | 4.1 | 2.24 (1/10) | never | 5147 | 0.04 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 6.3 / 16.7 | 4.2 / 15.7 | 8.0 / 13.7 | 0.1 | 13.7 | 3.14 (4/10) | never | 2915 | 0.04 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.6 / 7.1 | 1.6 / 6.4 | 5.6 / 14.5 | 0.1 | 11.6 | 2.87 (8/10) | 3.01 (3/10) | 1324 | 0.94 |

**long_auto**: auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end (14.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 6.1 / 25.3 | 6.1 / 25.3 | 8.7 / 18.8 | 2.7 | 6.6 | 4.11 (4/10) | never | 718 | 157.49 |
| BLine (README gains, r 0.30) | 33.0 / 99.6 | 37.3 / 111.9 | 8.2 / 19.9 | 1.1 | 63.2 | 5.73 (5/10) | 6.41 (2/10) | 2147 | 0.05 |
| BLine (README gains, r 0.45 default) | 31.9 / 90.9 | 35.0 / 102.0 | 8.8 / 19.9 | 1.2 | 52.1 | 5.71 (4/10) | 6.05 (1/10) | 2080 | 0.05 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 10.8 / 33.9 | 4.7 / 24.1 | 8.2 / 19.0 | 2.5 | 5.2 | 4.93 (4/10) | never | 764 | 0.05 |
| BLine (tuned) | 22.7 / 59.8 | 19.8 / 59.1 | 8.3 / 20.6 | 1.5 | 7.1 | 3.74 (5/10) | 4.15 (2/10) | 856 | 0.06 |
| BLine (tuned, with corner caps) | 10.0 / 32.7 | 5.9 / 22.7 | 8.2 / 19.1 | 1.2 | 1.5 | 5.29 (4/10) | 5.70 (1/10) | 532 | 0.05 |
| PathPlanner (kP 5) | 5.0 / 20.9 | 5.0 / 20.9 | 8.6 / 20.8 | 1.0 | 4.6 | 3.62 (4/10) | 3.75 (2/10) | 2182 | 7.81 |
| PathPlanner (kP 5) + hold | 5.0 / 20.9 | 5.1 / 20.9 | 9.5 / 21.1 | 0.9 | 4.6 | 3.62 (4/10) | never | 2183 | 7.44 |
| PathPlanner (tuned) + hold | 4.7 / 20.1 | 4.7 / 20.1 | 8.7 / 20.1 | 0.8 | 3.3 | 3.67 (4/10) | 3.80 (1/10) | 2149 | 7.06 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 4.8 / 20.3 | 4.8 / 20.3 | 8.5 / 20.1 | 0.9 | 6.0 | 3.72 (4/10) | 4.15 (1/10) | 1808 | 7.92 |
| DriveToPose chain | 8.1 / 32.0 | 4.5 / 18.8 | 7.9 / 17.7 | 0.9 | 4.7 | 6.54 (4/10) | 6.64 (3/10) | 1709 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 9.1 / 27.7 | 5.2 / 20.4 | 8.0 / 18.4 | 1.9 | 6.0 | 3.76 (4/10) | 4.00 (1/10) | 1810 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 9.7 / 28.9 | 6.0 / 22.5 | 8.1 / 18.9 | 1.0 | 3.7 | 3.84 (4/10) | 4.09 (2/10) | 1596 | 6.21 |
| Ours v2 (as shipped) | 6.5 / 21.5 | 4.9 / 20.1 | 8.5 / 20.1 | 0.9 | 2.0 | 3.99 (4/10) | 4.45 (2/10) | 1808 | 1.50 |
| Ours v2 (as shipped, adaptive grip off) | 6.5 / 21.5 | 4.9 / 20.1 | 8.5 / 20.1 | 0.9 | 2.0 | 3.65 (4/10) | 3.77 (1/10) | 1984 | 1.43 |
| Ours v2 (tuned) | 6.5 / 21.5 | 4.9 / 20.1 | 8.5 / 20.1 | 0.9 | 2.0 | 3.99 (4/10) | 4.45 (2/10) | 1808 | 1.62 |
| PathPlanner path2 (new-path, pre-release), as shipped | 8.4 / 30.1 | 10.2 / 52.0 | 11.2 / 23.6 | 0.9 | 0.0 | never | never | 3618 | 0.05 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 8.6 / 24.6 | 9.3 / 43.6 | 9.3 / 21.5 | 0.9 | 2.2 | 4.64 (4/10) | 4.88 (1/10) | 3038 | 0.05 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 6.6 / 21.6 | 4.9 / 19.5 | 8.1 / 19.3 | 1.0 | 2.8 | 3.94 (4/10) | 4.06 (2/10) | 748 | 1.48 |

**moving_start**: starts moving 2 m/s sideways (+y), path heads +x then curves left (7.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 12.3 / 27.1 | 12.3 / 27.1 | 5.1 / 11.0 | 0.2 | 10.4 | 1.88 (9/10) | 2.12 (2/10) | 1550 | 66.38 |
| BLine (README gains, r 0.30) | 30.8 / 59.3 | 31.9 / 65.9 | 23.7 / 30.1 | 0.1 | 30.0 | 3.44 | never | 2168 | 0.05 |
| BLine (README gains, r 0.45 default) | 33.1 / 54.9 | 34.2 / 51.6 | 19.2 / 31.9 | 0.4 | 56.0 | 2.14 | never | 1546 | 0.05 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 24.4 / 49.7 | 24.9 / 47.2 | 8.9 / 14.7 | 0.0 | 14.1 | 2.17 | never | 1227 | 0.06 |
| BLine (tuned) | 31.8 / 60.5 | 29.0 / 56.9 | 3.5 / 9.1 | 0.1 | 5.4 | 2.26 (8/10) | 2.75 (3/10) | 772 | 0.06 |
| BLine (tuned, with corner caps) | 18.8 / 47.8 | 19.5 / 45.7 | 3.7 / 9.6 | 0.1 | 5.0 | 3.07 (8/10) | 3.56 (3/10) | 822 | 0.05 |
| PathPlanner (kP 5) | 23.3 / 29.2 | 23.3 / 29.2 | 24.4 / 28.5 | 0.1 | 9.3 | never | never | 2657 | 3.23 |
| PathPlanner (kP 5) + hold | 23.3 / 29.2 | 23.3 / 29.2 | 24.2 / 29.1 | 0.1 | 9.3 | never | never | 2659 | 3.93 |
| PathPlanner (tuned) + hold | 23.6 / 30.6 | 23.6 / 30.6 | 25.8 / 30.6 | 0.1 | 8.4 | never | never | 2798 | 3.26 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 23.3 / 29.2 | 23.3 / 29.1 | 24.2 / 29.2 | 0.1 | 9.3 | never | never | 2628 | 3.06 |
| DriveToPose chain | 21.1 / 58.8 | 19.6 / 54.9 | 3.7 / 8.1 | 0.1 | 7.3 | 3.64 (9/10) | 3.84 (4/10) | 1310 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 10.0 / 20.1 | 11.5 / 20.7 | 13.0 / 16.7 | 0.1 | 10.6 | 1.67 (1/10) | never | 2359 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 10.8 / 27.3 | 11.1 / 28.7 | 3.6 / 10.0 | 0.3 | 5.2 | 1.84 (8/10) | 2.09 (4/10) | 1907 | 3.20 |
| Ours v2 (as shipped) | 9.0 / 23.3 | 9.1 / 23.3 | 4.2 / 10.2 | 0.1 | 6.2 | 1.72 (8/10) | 1.86 (1/10) | 2072 | 1.37 |
| Ours v2 (as shipped, adaptive grip off) | 9.0 / 23.3 | 9.1 / 23.3 | 4.2 / 10.2 | 0.1 | 6.2 | 1.72 (8/10) | 1.86 (1/10) | 2072 | 1.41 |
| Ours v2 (tuned) | 9.0 / 23.3 | 9.1 / 23.3 | 4.2 / 10.2 | 0.1 | 6.2 | 1.72 (8/10) | 1.86 (1/10) | 2072 | 1.17 |
| PathPlanner path2 (new-path, pre-release), as shipped | 8.0 / 30.9 | 7.4 / 29.3 | 8.7 / 14.3 | 0.1 | 1.3 | never | never | 3498 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 9.9 / 29.3 | 8.5 / 28.1 | 5.2 / 13.0 | 0.1 | 4.6 | 2.38 (6/10) | 2.74 (1/10) | 3013 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 18.5 / 37.3 | 18.6 / 37.4 | 3.6 / 8.5 | 0.1 | 5.6 | 1.99 (8/10) | 2.35 (4/10) | 1629 | 1.36 |

**hit_mid_path**: the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 11.9 / 39.0 | 11.9 / 39.0 | 8.7 / 15.4 | 0.8 | 11.8 | 2.39 (1/10) | never | 3124 | 158.25 |
| BLine (README gains, r 0.30) | 32.8 / 69.2 | 34.7 / 74.9 | 33.5 / 42.6 | 0.1 | 41.3 | 4.28 (6/10) | never | 4182 | 0.08 |
| BLine (README gains, r 0.45 default) | 23.6 / 54.8 | 25.5 / 60.5 | 8.3 / 14.3 | 0.1 | 78.2 | 3.03 (3/10) | never | 3668 | 0.05 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 8.3 / 25.3 | 9.6 / 30.1 | 9.6 / 17.3 | 0.4 | 16.0 | 2.30 (7/10) | never | 2702 | 0.05 |
| BLine (tuned) | 26.3 / 56.6 | 28.0 / 62.5 | 7.4 / 12.0 | 1.1 | 10.6 | 2.22 (6/10) | never | 3239 | 0.06 |
| BLine (tuned, with corner caps) | 8.1 / 28.2 | 9.3 / 31.2 | 6.5 / 13.8 | 0.1 | 6.0 | 2.83 (5/10) | never | 2844 | 0.05 |
| PathPlanner (kP 5) | 20.7 / 48.6 | 20.8 / 48.6 | 9.2 / 16.5 | 8.9 | 14.0 | 1.99 (3/10) | never | 4886 | 3.89 |
| PathPlanner (kP 5) + hold | 20.7 / 48.6 | 20.8 / 48.6 | 9.7 / 16.0 | 0.4 | 14.0 | 2.05 (5/10) | never | 4907 | 4.02 |
| PathPlanner (tuned) + hold | 20.2 / 48.4 | 20.3 / 48.6 | 14.6 / 20.3 | 0.2 | 19.3 | 1.92 (5/10) | never | 5208 | 4.20 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 19.0 / 47.1 | 19.1 / 47.2 | 11.1 / 18.8 | 0.1 | 11.0 | 2.00 (3/10) | never | 4639 | 3.89 |
| DriveToPose chain | 9.1 / 37.0 | 9.7 / 40.6 | 4.3 / 10.0 | 0.2 | 7.9 | 4.33 (7/10) | 4.39 (3/10) | 2980 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 43.2 / 92.3 | 42.0 / 91.8 | 28.0 / 35.1 | 6.8 | 34.5 | 2.13 (3/10) | never | 5408 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 45.2 / 95.8 | 43.8 / 94.3 | 8.8 / 14.8 | 0.3 | 30.4 | 2.61 (5/10) | never | 5679 | 5.87 |
| Ours v2 (as shipped) | 16.7 / 49.7 | 16.9 / 50.8 | 8.4 / 15.7 | 0.2 | 5.7 | 2.00 (3/10) | never | 4945 | 1.37 |
| Ours v2 (as shipped, adaptive grip off) | 15.3 / 47.7 | 15.5 / 48.8 | 18.0 / 28.1 | 0.2 | 0.0 | never | never | 4920 | 1.54 |
| Ours v2 (tuned) | 16.7 / 49.7 | 16.9 / 50.8 | 8.4 / 15.7 | 0.2 | 5.7 | 2.00 (3/10) | never | 4945 | 1.38 |
| PathPlanner path2 (new-path, pre-release), as shipped | 41.8 / 53.7 | 43.5 / 56.9 | 54.0 / 61.5 | 0.9 | 0.0 | never | never | 4658 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 11.8 / 42.3 | 13.3 / 48.1 | 10.0 / 19.4 | 0.1 | 1.0 | 3.03 (2/10) | never | 4104 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 14.4 / 39.7 | 14.5 / 40.0 | 6.4 / 14.1 | 0.2 | 13.6 | 2.12 (7/10) | 2.65 (1/10) | 3099 | 1.15 |

**low_grip_s_curve**: the S-curve on mu 0.91 static / 0.79 kinetic (61 % of normal grip) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.7 / 12.0 | 4.7 / 12.0 | 6.6 / 13.8 | 0.1 | 13.6 | 2.25 | 2.50 (2/10) | 2496 | 114.42 |
| BLine (README gains, r 0.30) | 12.7 / 37.2 | 13.8 / 37.2 | 8.1 / 17.4 | 0.2 | 96.4 | 3.01 (8/10) | never | 5553 | 0.07 |
| BLine (README gains, r 0.45 default) | 7.9 / 26.6 | 8.8 / 26.7 | 6.5 / 14.9 | 0.1 | 91.4 | 2.93 (9/10) | never | 5222 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 9.8 | 1.7 / 7.0 | 15.6 / 24.4 | 0.1 | 24.2 | 2.22 | never | 2680 | 0.06 |
| BLine (tuned) | 6.8 / 16.7 | 4.8 / 15.9 | 4.4 / 12.1 | 0.1 | 7.2 | 2.69 (8/10) | 3.18 (2/10) | 2582 | 0.05 |
| BLine (tuned, with corner caps) | 2.8 / 6.6 | 3.0 / 10.5 | 4.3 / 11.6 | 0.1 | 7.7 | 2.73 (8/10) | 3.25 (2/10) | 2909 | 0.05 |
| PathPlanner (kP 5) | 3.2 / 10.8 | 3.2 / 10.8 | 12.5 / 21.9 | 1.9 | 21.9 | 1.84 (6/10) | never | 5502 | 4.03 |
| PathPlanner (kP 5) + hold | 3.2 / 10.8 | 3.2 / 10.8 | 10.9 / 17.2 | 0.1 | 21.9 | 1.84 (6/10) | never | 5720 | 3.62 |
| PathPlanner (tuned) + hold | 3.4 / 11.2 | 3.4 / 11.3 | 9.4 / 14.2 | 0.2 | 18.3 | 1.90 (5/10) | never | 6101 | 3.71 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 3.0 / 11.5 | 3.0 / 11.5 | 6.8 / 15.4 | 0.1 | 7.7 | 1.92 (6/10) | 2.40 (1/10) | 4896 | 3.86 |
| DriveToPose chain | 2.9 / 9.1 | 0.9 / 3.5 | 4.2 / 11.1 | 0.3 | 7.5 | 3.99 (8/10) | 4.13 (2/10) | 5240 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 4.5 / 10.1 | 1.8 / 5.8 | 4.5 / 10.5 | 0.0 | 10.9 | 1.96 (9/10) | 2.18 (4/10) | 4558 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 4.8 / 11.8 | 2.6 / 5.9 | 4.2 / 10.7 | 0.2 | 8.7 | 2.02 (8/10) | 2.26 (3/10) | 4139 | 4.22 |
| Ours v2 (as shipped) | 2.0 / 7.0 | 1.9 / 6.0 | 4.1 / 10.5 | 0.1 | 9.3 | 2.94 (8/10) | 3.10 (4/10) | 306 | 1.49 |
| Ours v2 (as shipped, adaptive grip off) | 3.1 / 11.4 | 3.1 / 11.4 | 7.3 / 12.9 | 0.2 | 9.2 | 1.88 (3/10) | never | 5632 | 1.50 |
| Ours v2 (tuned) | 2.0 / 7.0 | 1.9 / 6.0 | 4.1 / 10.5 | 0.1 | 9.3 | 2.94 (8/10) | 3.10 (4/10) | 306 | 1.44 |
| PathPlanner path2 (new-path, pre-release), as shipped | 4.9 / 15.2 | 3.1 / 15.2 | 13.9 / 16.6 | 1.6 | 7.7 | never | never | 6113 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 7.3 / 18.7 | 8.3 / 18.8 | 14.3 / 22.7 | 0.1 | 12.2 | never | never | 5275 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.6 / 6.6 | 1.5 / 5.5 | 4.1 / 11.0 | 0.1 | 8.1 | 2.08 (8/10) | 2.22 (3/10) | 3434 | 1.39 |

**grip_drop_s_curve**: the S-curve; halfway along, the floor drops from mu 1.5 / 1.3 to 0.91 / 0.79 (a worn carpet seam) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 11.7 | 4.6 / 11.7 | 7.7 / 15.9 | 0.2 | 15.7 | 2.24 | never | 1606 | 155.38 |
| BLine (README gains, r 0.30) | 13.1 / 37.6 | 14.2 / 37.7 | 7.4 / 15.5 | 0.2 | 98.5 | 3.02 (9/10) | never | 3739 | 0.08 |
| BLine (README gains, r 0.45 default) | 8.3 / 27.3 | 9.1 / 27.3 | 6.1 / 12.9 | 0.1 | 93.5 | 2.96 (9/10) | never | 3523 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 9.5 | 1.7 / 7.0 | 16.8 / 25.6 | 0.1 | 25.4 | 2.21 | never | 1526 | 0.10 |
| BLine (tuned) | 7.0 / 16.5 | 4.8 / 15.8 | 4.0 / 10.0 | 0.1 | 9.1 | 2.57 (8/10) | 3.17 (4/10) | 1135 | 0.08 |
| BLine (tuned, with corner caps) | 3.0 / 7.2 | 3.3 / 10.8 | 4.1 / 9.9 | 0.1 | 9.3 | 2.70 (9/10) | 3.25 (4/10) | 1308 | 0.07 |
| PathPlanner (kP 5) | 2.7 / 11.5 | 2.7 / 11.5 | 15.4 / 24.4 | 1.6 | 24.4 | 1.82 (8/10) | never | 3808 | 4.35 |
| PathPlanner (kP 5) + hold | 2.7 / 11.5 | 2.7 / 11.5 | 6.9 / 15.8 | 0.1 | 24.4 | 1.82 (8/10) | 2.59 (1/10) | 3995 | 3.91 |
| PathPlanner (tuned) + hold | 2.6 / 11.7 | 2.6 / 11.7 | 7.0 / 17.2 | 0.1 | 23.1 | 1.82 (8/10) | 2.36 (1/10) | 4500 | 3.83 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.9 / 8.0 | 1.9 / 8.0 | 4.8 / 11.5 | 0.1 | 14.3 | 1.89 (9/10) | 2.40 (4/10) | 3312 | 4.25 |
| DriveToPose chain | 2.9 / 8.9 | 0.9 / 3.3 | 4.1 / 10.6 | 0.1 | 8.2 | 4.02 (8/10) | 4.16 (3/10) | 3020 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 4.7 / 10.9 | 2.0 / 5.9 | 5.5 / 12.8 | 0.0 | 13.0 | 1.94 (8/10) | 2.18 (1/10) | 2960 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.2 / 12.4 | 2.7 / 6.5 | 4.3 / 10.6 | 0.2 | 10.2 | 2.01 (8/10) | 2.26 (4/10) | 2624 | 5.21 |
| Ours v2 (as shipped) | 1.8 / 7.5 | 2.1 / 8.2 | 7.4 / 14.8 | 0.2 | 11.1 | 1.82 (8/10) | 2.23 (2/10) | 4234 | 1.42 |
| Ours v2 (as shipped, adaptive grip off) | 1.8 / 7.5 | 2.1 / 8.2 | 7.4 / 14.8 | 0.2 | 11.1 | 1.82 (8/10) | 2.23 (2/10) | 4234 | 1.27 |
| Ours v2 (tuned) | 1.8 / 7.5 | 2.1 / 8.2 | 7.4 / 14.8 | 0.2 | 11.1 | 1.82 (8/10) | 2.23 (2/10) | 4234 | 1.45 |
| PathPlanner path2 (new-path, pre-release), as shipped | 6.1 / 20.7 | 6.7 / 22.0 | 27.1 / 32.9 | 1.6 | 36.7 | never | never | 4876 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 5.1 / 14.9 | 5.8 / 19.5 | 6.5 / 12.6 | 0.1 | 11.1 | 2.59 (6/10) | 3.02 (1/10) | 3361 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.4 / 6.0 | 1.3 / 4.9 | 4.2 / 10.5 | 0.1 | 10.1 | 2.07 (9/10) | 2.22 (4/10) | 2156 | 1.48 |

**spin_while_driving**: 5 m straight while the robot turns a full 360 deg (a heading target at every waypoint) (8.2 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 2.1 / 6.8 | 2.1 / 6.8 | 5.8 / 11.6 | 14.3 | 11.4 | 2.20 (8/10) | never | 411 | 91.57 |
| BLine (README gains, r 0.30) | 2.7 / 7.6 | 2.7 / 7.6 | 5.8 / 12.3 | 3.1 | 63.5 | 1.93 (8/10) | never | 618 | 0.13 |
| BLine (README gains, r 0.45 default) | 3.1 / 7.6 | 3.1 / 7.6 | 6.0 / 12.5 | 3.1 | 63.1 | 1.94 (8/10) | never | 654 | 0.08 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.1 / 7.6 | 3.1 / 7.6 | 6.0 / 12.5 | 3.1 | 63.1 | 1.94 (8/10) | never | 654 | 0.07 |
| BLine (tuned) | 3.8 / 8.4 | 3.8 / 8.4 | 4.6 / 11.4 | 3.1 | 8.1 | 2.80 (8/10) | never | 150 | 0.08 |
| BLine (tuned, with corner caps) | 2.2 / 5.1 | 2.2 / 5.1 | 4.6 / 11.5 | 3.1 | 8.1 | 2.80 (8/10) | never | 152 | 0.07 |
| PathPlanner (kP 5) | 2.2 / 7.9 | 2.2 / 7.9 | 5.9 / 11.7 | 1.7 | 11.1 | 2.95 (8/10) | 3.11 (2/10) | 108 | 4.63 |
| PathPlanner (kP 5) + hold | 2.2 / 7.9 | 2.2 / 7.9 | 5.1 / 12.1 | 1.8 | 11.1 | 2.95 (8/10) | 3.36 (3/10) | 108 | 4.83 |
| PathPlanner (tuned) + hold | 1.8 / 7.1 | 1.8 / 7.1 | 5.2 / 12.1 | 1.9 | 10.3 | 2.94 (8/10) | 3.16 (2/10) | 219 | 4.27 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 2.1 / 7.9 | 2.1 / 7.9 | 5.1 / 11.9 | 1.8 | 11.5 | 2.94 (8/10) | 3.36 (3/10) | 166 | 3.93 |
| DriveToPose chain | 1.5 / 7.7 | 1.5 / 7.7 | 5.0 / 10.7 | 3.3 | 9.6 | 4.85 (8/10) | never | 935 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 3.3 / 8.6 | 2.4 / 7.5 | 5.6 / 11.6 | 5.3 | 11.4 | 1.94 (8/10) | never | 970 | 0.02 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.8 / 15.5 | 5.1 / 15.3 | 5.1 / 11.9 | 3.3 | 8.7 | 2.08 (8/10) | never | 494 | 13.43 |
| Ours v2 (as shipped) | 3.3 / 8.9 | 3.3 / 8.9 | 5.2 / 12.1 | 1.9 | 8.9 | 2.26 (8/10) | 2.86 (2/10) | 435 | 3.37 |
| Ours v2 (as shipped, adaptive grip off) | 3.3 / 8.9 | 3.3 / 8.9 | 5.2 / 12.1 | 1.9 | 8.9 | 2.26 (8/10) | 2.86 (2/10) | 437 | 2.60 |
| Ours v2 (tuned) | 3.3 / 8.9 | 3.3 / 8.9 | 5.2 / 12.1 | 1.9 | 8.9 | 2.26 (8/10) | 2.86 (2/10) | 435 | 2.20 |
| PathPlanner path2 (new-path, pre-release), as shipped | 3.2 / 8.7 | 3.2 / 8.7 | 8.0 / 15.9 | 2.9 | 1.9 | 2.04 (2/10) | never | 1366 | 0.10 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 2.6 / 8.6 | 2.6 / 8.6 | 4.7 / 12.8 | 1.8 | 6.0 | 2.81 (8/10) | 3.18 (3/10) | 1498 | 0.07 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 2.4 / 8.2 | 2.4 / 8.2 | 5.1 / 12.0 | 2.0 | 10.0 | 2.62 (8/10) | 3.12 (2/10) | 60 | 2.17 |

**figure_eight**: a self-crossing loop (a lemniscate, 4 m x 1.8 m) through the origin twice; heading held (12.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.5 / 15.4 | 5.5 / 15.4 | 2.0 / 4.2 | 0.1 | 3.5 | 5.12 | 5.32 (6/10) | 796 | 281.67 |
| BLine (README gains, r 0.30) | 43.5 / 117.8 | 43.8 / 117.8 | 2.3 / 3.9 | 0.2 | 62.4 | 9.88 | 9.97 (5/10) | 4139 | 0.11 |
| BLine (README gains, r 0.45 default) | 50.2 / 117.3 | 51.1 / 118.1 | 3.9 / 5.2 | 0.1 | 68.0 | 6.60 (9/10) | never | 3051 | 0.13 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 40.3 / 93.4 | 41.3 / 96.1 | 3.2 / 4.4 | 0.2 | 48.9 | 6.05 | 6.16 (1/10) | 2825 | 0.13 |
| BLine (tuned) | 56.2 / 154.0 | 56.9 / 155.7 | 2.8 / 4.2 | 0.1 | 0.0 | 3.85 | never | 1136 | 0.12 |
| BLine (tuned, with corner caps) | 43.6 / 96.5 | 44.6 / 98.4 | 2.7 / 4.1 | 0.1 | 0.0 | 6.01 | never | 2805 | 0.11 |
| PathPlanner (kP 5) | 16.8 / 47.4 | 16.8 / 47.2 | 35.1 / 47.4 | 0.2 | 39.1 | never | never | 7450 | 15.53 |
| PathPlanner (kP 5) + hold | 16.8 / 47.4 | 16.8 / 47.2 | 32.3 / 44.4 | 0.1 | 39.1 | never | never | 7460 | 21.47 |
| PathPlanner (tuned) + hold | 3.2 / 12.5 | 3.1 / 12.5 | 8.9 / 13.4 | 0.1 | 5.7 | 3.59 (1/10) | never | 6471 | 18.42 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.8 / 8.5 | 1.7 / 8.4 | 2.1 / 3.5 | 0.1 | 4.2 | 3.66 | 4.08 (6/10) | 5501 | 25.47 |
| DriveToPose chain | 2.1 / 10.8 | 1.5 / 8.0 | 15.8 / 44.4 | 0.0 | 1.2 | 12.36 (6/10) | 12.31 (1/10) | 2682 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 2.5 / 8.4 | 2.1 / 8.4 | 1.6 / 4.0 | 0.1 | 3.2 | 4.30 | 4.50 (7/10) | 2764 | 0.02 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 3.2 / 10.1 | 2.9 / 10.1 | 1.4 / 2.8 | 0.3 | 1.2 | 4.41 | 4.66 (8/10) | 2289 | 18.30 |
| Ours v2 (as shipped) | 3.0 / 12.9 | 3.1 / 12.7 | 2.4 / 4.5 | 0.1 | 1.5 | 3.56 | 3.69 (4/10) | 5684 | 3.53 |
| Ours v2 (as shipped, adaptive grip off) | 3.0 / 12.9 | 3.1 / 12.7 | 2.4 / 4.5 | 0.1 | 1.5 | 3.56 | 3.69 (4/10) | 5684 | 2.84 |
| Ours v2 (tuned) | 3.0 / 12.9 | 3.1 / 12.7 | 2.4 / 4.5 | 0.1 | 1.5 | 3.56 | 3.69 (4/10) | 5684 | 2.82 |
| PathPlanner path2 (new-path, pre-release), as shipped | 34.8 / 59.6 | 35.0 / 61.4 | 43.4 / 57.2 | 1.7 | 27.1 | never | never | 10347 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 17.9 / 52.8 | 18.4 / 55.2 | 19.1 / 49.7 | 0.1 | 48.9 | 3.93 (4/10) | never | 10888 | 0.08 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 2.0 / 9.4 | 2.0 / 9.2 | 1.7 / 2.7 | 0.1 | 0.8 | 4.51 | 4.62 (8/10) | 1999 | 2.63 |

**wall_hug**: along a wall with the bumpers 15 cm off it, then a 90 deg turn to run 15 cm off the next wall (11.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.0 / 22.6 | 5.0 / 21.7 | 6.6 / 14.6 | 0.1 | 8.6 | 3.05 (4/10) | never | 911 | 199.24 |
| BLine (README gains, r 0.30) | 40.2 / 104.0 | 40.0 / 104.0 | 6.1 / 14.6 | 0.1 | 68.7 | 5.42 (6/10) | 5.70 (3/10) | 1910 | 0.08 |
| BLine (README gains, r 0.45 default) | 38.9 / 99.9 | 39.0 / 99.9 | 5.9 / 14.5 | 0.1 | 61.7 | 5.27 (6/10) | 5.56 (3/10) | 1834 | 0.09 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 38.9 / 99.9 | 39.0 / 99.9 | 5.9 / 14.5 | 0.1 | 61.7 | 5.27 (6/10) | 5.56 (3/10) | 1834 | 0.10 |
| BLine (tuned) | 20.9 / 59.9 | 20.9 / 59.9 | 6.3 / 13.4 | 0.1 | 3.6 | 2.96 (4/10) | never | 808 | 0.09 |
| BLine (tuned, with corner caps) | 40.6 / 102.8 | 40.8 / 102.8 | 5.3 / 12.7 | 0.1 | 9.0 | 3.85 (6/10) | 4.37 (4/10) | 958 | 0.09 |
| PathPlanner (kP 5) | 3.6 / 16.1 | 3.6 / 16.1 | 6.5 / 14.4 | 0.0 | 3.4 | 2.72 (5/10) | 2.88 (1/10) | 2264 | 8.25 |
| PathPlanner (kP 5) + hold | 3.6 / 16.1 | 3.6 / 16.1 | 7.1 / 15.3 | 0.1 | 3.4 | 2.72 (5/10) | 3.13 (1/10) | 2264 | 6.70 |
| PathPlanner (tuned) + hold | 3.5 / 14.4 | 3.5 / 14.4 | 7.4 / 14.6 | 0.2 | 2.9 | 2.83 (4/10) | never | 2102 | 5.83 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 2.8 / 12.4 | 2.8 / 12.4 | 6.5 / 13.0 | 0.1 | 6.3 | 2.87 (5/10) | never | 1710 | 6.68 |
| DriveToPose chain | 3.3 / 15.0 | 3.0 / 12.2 | 5.2 / 13.6 | 0.0 | 5.5 | 7.14 (7/10) | 7.30 (3/10) | 354 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 3.3 / 14.6 | 3.1 / 12.5 | 5.3 / 13.1 | 0.1 | 7.4 | 2.96 (5/10) | 3.17 (4/10) | 1337 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 4.1 / 15.9 | 3.8 / 14.0 | 5.6 / 13.4 | 0.2 | 4.3 | 3.07 (5/10) | 3.26 (3/10) | 1064 | 10.91 |
| Ours v2 (as shipped) | 3.5 / 13.6 | 3.5 / 13.3 | 7.0 / 13.6 | 0.1 | 2.7 | 2.79 (4/10) | never | 1851 | 1.96 |
| Ours v2 (as shipped, adaptive grip off) | 3.5 / 13.6 | 3.5 / 13.3 | 7.0 / 13.6 | 0.1 | 2.7 | 2.79 (4/10) | never | 1851 | 2.11 |
| Ours v2 (tuned) | 3.5 / 13.6 | 3.5 / 13.3 | 7.0 / 13.6 | 0.1 | 2.7 | 2.79 (4/10) | never | 1851 | 1.84 |
| PathPlanner path2 (new-path, pre-release), as shipped | 10.1 / 25.6 | 10.1 / 25.6 | 16.8 / 26.7 | 0.2 | 0.0 | never | never | 3801 | 0.09 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 14.2 / 31.4 | 14.4 / 31.4 | 20.2 / 28.0 | 0.1 | 2.3 | never | never | 3950 | 0.08 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 3.2 / 12.7 | 3.2 / 12.5 | 5.7 / 13.2 | 0.1 | 4.3 | 3.04 (4/10) | 3.15 (3/10) | 795 | 1.80 |

**tag_align_moving_goal**: 5 m to a scoring pose; the true goal is 8 cm sideways and 3 deg off the map pose, revealed 1.5 m out (8.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 3.0 / 8.1 | 3.0 / 8.1 | 1.8 / 3.7 | 0.3 | 15.1 | 2.21 | 2.41 (5/10) | 463 | 133.10 |
| BLine (README gains, r 0.30) | 3.5 / 7.4 | 4.6 / 14.1 | 1.3 / 3.3 | 0.2 | 87.4 | 2.82 | 3.01 (8/10) | 524 | 0.07 |
| BLine (README gains, r 0.45 default) | 3.3 / 7.4 | 2.3 / 8.5 | 1.2 / 3.3 | 0.1 | 86.9 | 2.80 | 2.99 (8/10) | 482 | 0.08 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.4 / 8.1 | 0.8 / 4.2 | 1.3 / 3.0 | 0.3 | 57.8 | 2.72 | 2.91 (8/10) | 316 | 0.08 |
| BLine (tuned) | 6.4 / 17.0 | 3.9 / 16.1 | 1.1 / 3.1 | 0.3 | 9.1 | 3.33 | 3.58 (7/10) | 199 | 0.09 |
| BLine (tuned, with corner caps) | 3.4 / 8.2 | 1.0 / 5.4 | 1.3 / 3.0 | 0.3 | 9.3 | 3.37 | 3.60 (7/10) | 197 | 0.07 |
| PathPlanner (kP 5) | 3.0 / 7.8 | 3.0 / 7.8 | 3.1 / 5.8 | 0.1 | 6.0 | 1.80 | 1.98 (2/10) | 429 | 4.28 |
| PathPlanner (kP 5) + hold | 3.0 / 7.8 | 3.0 / 7.8 | 1.3 / 2.9 | 0.0 | 6.0 | 1.80 | 2.25 (7/10) | 433 | 4.10 |
| PathPlanner (tuned) + hold | 3.1 / 8.0 | 3.1 / 8.0 | 1.9 / 3.2 | 0.1 | 4.3 | 1.82 | 1.99 (5/10) | 559 | 4.86 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 3.0 / 7.8 | 3.0 / 7.8 | 1.3 / 2.9 | 0.0 | 6.0 | 1.80 | 2.25 (7/10) | 441 | 4.06 |
| DriveToPose chain | 4.3 / 8.7 | 2.5 / 7.6 | 1.0 / 2.8 | 0.3 | 2.4 | 2.62 | 2.81 (8/10) | 392 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 1.1 / 5.5 | 1.1 / 5.3 | 1.2 / 2.8 | 0.3 | 12.5 | 2.14 | 2.37 (7/10) | 709 | 0.02 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.8 / 6.9 | 1.9 / 6.7 | 1.2 / 3.4 | 0.3 | 3.3 | 1.84 | 2.10 (8/10) | 733 | 4.88 |
| Ours v2 (as shipped) | 2.8 / 7.7 | 2.9 / 7.7 | 0.9 / 3.7 | 0.2 | 1.8 | 1.89 | 2.03 (9/10) | 444 | 1.46 |
| Ours v2 (as shipped, adaptive grip off) | 2.8 / 7.7 | 2.9 / 7.7 | 0.9 / 3.7 | 0.2 | 1.8 | 1.82 | 1.95 (9/10) | 492 | 1.64 |
| Ours v2 (tuned) | 2.8 / 7.7 | 2.9 / 7.7 | 0.9 / 3.7 | 0.2 | 1.8 | 1.89 | 2.03 (9/10) | 444 | 1.67 |
| PathPlanner path2 (new-path, pre-release), as shipped | 4.1 / 9.1 | 1.2 / 5.5 | 2.1 / 4.4 | 0.3 | 3.8 | 1.86 | 2.11 (4/10) | 2018 | 0.09 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 3.2 / 8.7 | 1.3 / 5.4 | 1.3 / 3.0 | 0.3 | 8.1 | 3.08 | 3.40 (7/10) | 713 | 0.07 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 2.8 / 7.6 | 2.9 / 7.6 | 1.2 / 2.8 | 0.2 | 2.7 | 1.84 | 1.98 (8/10) | 426 | 1.60 |

**intake_drive_through**: drive through a game piece at 2 m/s; its detected position moves 10 cm sideways 2 m out (7.4 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.9 / 2.3 | 0.9 / 2.3 | 11.7 / 15.2 | 0.1 | 11.7 | never | never | 499 | 114.98 |
| BLine (README gains, r 0.30) | 3.6 / 9.9 | 3.5 / 9.9 | 33.2 / 35.7 | 0.1 | 35.7 | 1.39 | never | 951 | 0.08 |
| BLine (README gains, r 0.45 default) | 3.7 / 9.8 | 3.5 / 9.9 | 33.0 / 35.8 | 0.1 | 35.8 | 1.39 | never | 941 | 0.08 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.7 / 9.8 | 3.5 / 9.9 | 33.0 / 35.8 | 0.1 | 35.8 | 1.39 | never | 941 | 0.07 |
| BLine (tuned) | 4.4 / 9.8 | 4.3 / 9.8 | 18.1 / 21.3 | 0.1 | 41.9 | 1.65 | never | 839 | 0.07 |
| BLine (tuned, with corner caps) | 4.3 / 9.9 | 4.2 / 9.9 | 18.1 / 21.1 | 0.1 | 42.0 | 1.65 | never | 859 | 0.06 |
| PathPlanner (kP 5) | 3.9 / 10.1 | 3.9 / 10.1 | 20.9 / 24.7 | 0.2 | 24.7 | 1.41 | never | 885 | 3.22 |
| PathPlanner (kP 5) + hold | 3.9 / 10.1 | 3.9 / 10.1 | 20.9 / 24.7 | 0.2 | 24.7 | 1.41 | never | 885 | 3.28 |
| PathPlanner (tuned) + hold | 3.9 / 10.1 | 3.9 / 10.1 | 19.1 / 23.4 | 0.2 | 23.4 | 1.42 | never | 1039 | 2.99 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 3.9 / 10.1 | 3.9 / 10.1 | 20.9 / 24.7 | 0.2 | 24.7 | 1.41 | never | 885 | 3.06 |
| DriveToPose chain | 3.7 / 9.8 | 3.6 / 9.8 | 1.6 / 4.2 | 0.2 | 3.3 | 2.34 | 2.53 (7/10) | 264 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 0.2 / 0.6 | 0.3 / 0.8 | 23.9 / 30.9 | 0.4 | 29.3 | never | never | 1053 | 0.02 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.7 / 8.9 | 1.7 / 9.0 | 1.7 / 4.1 | 0.3 | 4.1 | 1.60 | 1.85 (7/10) | 708 | 2.77 |
| Ours v2 (as shipped) | 4.0 / 9.9 | 4.0 / 9.9 | 18.5 / 23.7 | 0.2 | 23.7 | 1.47 | never | 903 | 1.65 |
| Ours v2 (as shipped, adaptive grip off) | 4.0 / 9.9 | 4.0 / 9.9 | 19.5 / 23.7 | 0.2 | 23.7 | 1.40 | never | 964 | 1.75 |
| Ours v2 (tuned) | 4.0 / 9.9 | 4.0 / 9.9 | 18.5 / 23.7 | 0.2 | 23.7 | 1.47 | never | 903 | 1.48 |
| PathPlanner path2 (new-path, pre-release), as shipped | 0.8 / 2.5 | 0.7 / 1.4 | 16.7 / 22.6 | 0.1 | 0.0 | never | never | 1626 | 0.08 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 1.2 / 6.5 | 1.4 / 7.2 | 10.8 / 14.0 | 0.1 | 5.5 | never | never | 813 | 0.07 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 3.9 / 10.0 | 3.9 / 10.0 | 20.3 / 23.6 | 0.2 | 23.6 | 1.42 | never | 884 | 1.45 |

**three_piece_chain**: three paths with rolling handoffs at 2 m/s (core-builder's neutral_three_piece, 2026 field) (15.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 7.6 / 28.9 | 5.5 / 21.9 | 5.0 / 12.7 | 3.5 | 11.9 | 5.63 (7/10) | never | 1663 | 82.88 |
| BLine (README gains, r 0.30) | 20.6 / 57.9 | 21.3 / 58.0 | 5.7 / 10.5 | 0.2 | 106.4 | 6.74 (8/10) | 6.80 (1/10) | 4891 | 0.09 |
| BLine (README gains, r 0.45 default) | 19.3 / 54.4 | 19.8 / 54.4 | 4.1 / 10.4 | 0.3 | 104.6 | 6.66 (8/10) | 6.77 (4/10) | 4947 | 0.09 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 8.9 / 28.5 | 7.8 / 31.5 | 6.2 / 16.1 | 1.8 | 16.9 | 5.04 (7/10) | never | 3011 | 0.09 |
| BLine (tuned) | 12.6 / 52.2 | 11.1 / 40.0 | 6.6 / 14.2 | 1.0 | 13.5 | 6.83 (3/10) | never | 3554 | 0.10 |
| BLine (tuned, with corner caps) | 10.6 / 43.5 | 10.4 / 39.7 | 6.7 / 13.8 | 0.7 | 13.2 | 8.65 (7/10) | never | 4242 | 0.08 |
| PathPlanner (kP 5) | 4.7 / 15.9 | 4.7 / 15.2 | 5.7 / 13.7 | 0.3 | 13.2 | 4.57 (8/10) | 4.69 (1/10) | 3792 | 4.46 |
| PathPlanner (kP 5) + hold | 4.7 / 15.9 | 4.7 / 15.2 | 5.9 / 10.6 | 0.3 | 13.2 | 4.57 (8/10) | never | 3793 | 3.04 |
| PathPlanner (tuned) + hold | 4.3 / 17.8 | 4.3 / 18.2 | 6.3 / 13.7 | 0.3 | 14.5 | 4.63 (7/10) | never | 4001 | 3.11 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 4.3 / 14.6 | 4.2 / 15.6 | 6.1 / 11.3 | 0.3 | 13.1 | 4.74 (6/10) | never | 2870 | 3.68 |
| DriveToPose chain | 6.3 / 31.6 | 3.6 / 15.2 | 5.3 / 12.2 | 1.8 | 12.7 | 10.73 (7/10) | never | 2353 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 7.7 / 23.3 | 4.0 / 14.0 | 5.3 / 13.0 | 2.8 | 12.2 | 4.45 (8/10) | never | 2450 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 7.5 / 25.4 | 4.0 / 16.3 | 5.5 / 11.5 | 0.4 | 10.2 | 4.52 (6/10) | 4.78 (2/10) | 1963 | 11.40 |
| Ours v2 (as shipped) | 4.5 / 21.8 | 4.1 / 15.2 | 6.0 / 11.0 | 0.3 | 5.3 | 4.64 (4/10) | never | 3282 | 1.15 |
| Ours v2 (as shipped, adaptive grip off) | 4.5 / 21.8 | 4.2 / 15.2 | 6.2 / 11.0 | 0.3 | 8.7 | 4.64 (4/10) | never | 3290 | 1.06 |
| Ours v2 (tuned) | 4.5 / 21.8 | 4.1 / 15.2 | 6.0 / 11.0 | 0.3 | 5.3 | 4.64 (4/10) | never | 3282 | 1.09 |
| PathPlanner path2 (new-path, pre-release), as shipped | 9.4 / 25.4 | 9.7 / 35.6 | 13.3 / 20.9 | 1.8 | 0.0 | 3.96 (1/10) | never | 7684 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 9.0 / 26.4 | 8.1 / 26.4 | 13.6 / 16.5 | 0.3 | 2.1 | never | never | 5874 | 0.05 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 5.0 / 22.7 | 4.2 / 16.3 | 5.5 / 11.5 | 0.3 | 9.9 | 5.04 (6/10) | 5.17 (2/10) | 882 | 1.53 |

**fast_long**: an 8 m gentle curve with a 5 m/s cap (above the robot's 4.57 m/s free speed) (10.8 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 1.4 / 5.3 | 1.4 / 5.3 | 7.8 / 18.4 | 0.1 | 18.0 | 2.48 (9/10) | 2.68 (1/10) | 508 | 112.72 |
| BLine (README gains, r 0.30) | 1.8 / 5.4 | 1.4 / 5.3 | 35.5 / 73.7 | 0.7 | 130.9 | 2.20 | never | 1154 | 0.06 |
| BLine (README gains, r 0.45 default) | 2.3 / 5.8 | 1.0 / 4.1 | 40.1 / 73.3 | 0.7 | 131.3 | 2.20 | never | 1170 | 0.07 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 2.4 / 5.9 | 1.0 / 4.0 | 40.1 / 73.2 | 0.7 | 131.4 | 2.20 | never | 1173 | 0.06 |
| BLine (tuned) | 3.9 / 9.9 | 2.8 / 9.7 | 6.5 / 17.0 | 0.1 | 12.6 | 3.05 (8/10) | 3.60 (1/10) | 240 | 0.05 |
| BLine (tuned, with corner caps) | 2.2 / 6.2 | 1.5 / 5.2 | 6.5 / 16.8 | 0.1 | 12.7 | 3.01 (8/10) | 3.54 (1/10) | 287 | 0.06 |
| PathPlanner (kP 5) | 1.0 / 3.9 | 1.0 / 3.9 | 7.4 / 18.1 | 0.1 | 17.8 | 2.38 (9/10) | 2.60 (2/10) | 787 | 5.59 |
| PathPlanner (kP 5) + hold | 1.0 / 3.9 | 1.0 / 3.9 | 6.5 / 15.9 | 0.1 | 17.8 | 2.38 (9/10) | 2.84 (3/10) | 789 | 5.71 |
| PathPlanner (tuned) + hold | 1.1 / 4.2 | 1.1 / 4.1 | 7.0 / 16.6 | 0.1 | 16.2 | 2.38 (8/10) | 2.66 (4/10) | 846 | 5.23 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.0 / 3.9 | 1.0 / 3.9 | 6.5 / 15.9 | 0.1 | 17.8 | 2.38 (9/10) | 2.84 (3/10) | 789 | 5.13 |
| DriveToPose chain | 2.2 / 6.4 | 1.1 / 4.0 | 6.2 / 16.1 | 0.0 | 13.4 | 4.88 (8/10) | 5.03 (3/10) | 618 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 1.3 / 4.0 | 1.4 / 4.1 | 6.8 / 16.9 | 0.1 | 17.2 | 2.36 (8/10) | 2.61 (3/10) | 862 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 6.8 / 15.8 | 6.9 / 16.0 | 6.3 / 15.9 | 0.3 | 13.5 | 2.45 (8/10) | 2.72 (3/10) | 915 | 5.28 |
| Ours v2 (as shipped) | 1.2 / 4.0 | 1.2 / 4.1 | 6.3 / 16.4 | 0.1 | 13.3 | 2.54 (8/10) | 3.05 (2/10) | 454 | 1.42 |
| Ours v2 (as shipped, adaptive grip off) | 1.2 / 4.0 | 1.2 / 4.1 | 6.4 / 16.4 | 0.1 | 13.3 | 2.41 (8/10) | 2.56 (2/10) | 504 | 1.57 |
| Ours v2 (tuned) | 1.2 / 4.0 | 1.2 / 4.1 | 6.3 / 16.4 | 0.1 | 13.3 | 2.54 (8/10) | 3.05 (2/10) | 454 | 1.44 |
| PathPlanner path2 (new-path, pre-release), as shipped | 3.2 / 8.4 | 2.6 / 6.4 | 13.5 / 29.6 | 0.3 | 0.3 | 2.18 (1/10) | 2.31 (1/10) | 2267 | 0.05 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 2.2 / 7.5 | 2.7 / 9.2 | 6.8 / 17.2 | 0.1 | 12.1 | 3.19 (8/10) | 3.61 (1/10) | 1739 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.2 / 4.0 | 1.1 / 4.1 | 6.4 / 16.3 | 0.1 | 13.4 | 2.44 (8/10) | 2.58 (3/10) | 442 | 1.61 |

**heavy_robot**: the S-curve with a 68 kg robot (MOI 6.8) and a 192 A current limit (20 % lower) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 12.0 | 4.6 / 12.0 | 6.5 / 14.0 | 0.1 | 13.8 | 2.24 (9/10) | 2.44 (1/10) | 442 | 74.63 |
| BLine (README gains, r 0.30) | 13.2 / 36.8 | 14.3 / 36.9 | 6.0 / 13.9 | 0.1 | 97.8 | 3.02 (8/10) | never | 1116 | 0.06 |
| BLine (README gains, r 0.45 default) | 8.1 / 26.4 | 8.9 / 26.5 | 5.7 / 12.5 | 0.1 | 92.6 | 2.94 (9/10) | never | 950 | 0.06 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.3 / 10.4 | 1.7 / 7.7 | 10.9 / 19.8 | 0.1 | 19.6 | 2.22 | never | 574 | 0.05 |
| BLine (tuned) | 6.7 / 16.6 | 4.7 / 15.8 | 4.2 / 11.1 | 0.1 | 8.2 | 2.61 (8/10) | 3.16 (3/10) | 454 | 0.05 |
| BLine (tuned, with corner caps) | 2.9 / 7.3 | 3.2 / 10.8 | 4.2 / 10.9 | 0.1 | 8.4 | 2.68 (8/10) | 3.24 (3/10) | 378 | 0.06 |
| PathPlanner (kP 5) | 1.5 / 4.5 | 1.4 / 4.5 | 6.1 / 13.0 | 0.1 | 12.9 | 1.87 (9/10) | never | 1526 | 3.82 |
| PathPlanner (kP 5) + hold | 1.5 / 4.5 | 1.4 / 4.5 | 4.2 / 10.2 | 0.1 | 12.9 | 1.87 (9/10) | 2.36 (4/10) | 1529 | 3.83 |
| PathPlanner (tuned) + hold | 1.5 / 4.5 | 1.5 / 4.5 | 4.9 / 11.2 | 0.1 | 10.9 | 1.90 (9/10) | 2.17 (4/10) | 1581 | 3.72 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.3 / 4.3 | 1.3 / 4.2 | 4.1 / 10.4 | 0.1 | 12.5 | 1.91 (9/10) | 2.39 (4/10) | 1295 | 3.80 |
| DriveToPose chain | 2.9 / 8.8 | 0.8 / 3.3 | 4.0 / 10.1 | 0.2 | 8.7 | 3.88 (8/10) | 4.01 (4/10) | 895 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 4.6 / 11.1 | 2.0 / 5.6 | 4.5 / 10.8 | 0.1 | 11.3 | 1.95 (8/10) | 2.18 (4/10) | 1215 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.1 / 12.6 | 2.7 / 6.8 | 4.1 / 10.4 | 0.1 | 8.9 | 2.01 (8/10) | 2.25 (4/10) | 1161 | 3.60 |
| Ours v2 (as shipped) | 2.1 / 7.7 | 2.0 / 6.7 | 4.2 / 10.4 | 0.0 | 9.2 | 1.89 (8/10) | 2.05 (4/10) | 1445 | 0.97 |
| Ours v2 (as shipped, adaptive grip off) | 2.1 / 7.7 | 2.0 / 6.7 | 4.2 / 10.4 | 0.0 | 9.2 | 1.89 (8/10) | 2.05 (4/10) | 1445 | 0.99 |
| Ours v2 (tuned) | 2.1 / 7.7 | 2.0 / 6.7 | 4.2 / 10.4 | 0.0 | 9.2 | 1.89 (8/10) | 2.05 (4/10) | 1445 | 0.97 |
| PathPlanner path2 (new-path, pre-release), as shipped | 4.4 / 11.1 | 4.3 / 11.2 | 9.3 / 16.9 | 1.5 | 0.0 | 1.81 (1/10) | never | 3473 | 0.05 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 3.8 / 11.9 | 2.8 / 11.3 | 5.1 / 11.3 | 0.1 | 8.9 | 2.66 (7/10) | 3.12 (2/10) | 1833 | 0.05 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.6 / 6.6 | 1.5 / 5.5 | 4.1 / 10.7 | 0.1 | 9.0 | 2.07 (8/10) | 2.21 (4/10) | 579 | 1.16 |

### Perfect pose (the control law alone)

| method | runs settled | courses settled every seed | mean RMS cross-track cm | worst cross-track cm | mean final error cm | mean wheel slip ms |
|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0/90 | 0/9 | 6.3 | 38.3 | 4.3 | 1459 |
| BLine (README gains, r 0.30) | 12/90 | 0/9 | 24.9 | 99.1 | 14.7 | 2782 |
| BLine (README gains, r 0.45 default) | 6/90 | 0/9 | 21.4 | 99.0 | 10.3 | 2463 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 0/90 | 0/9 | 12.2 | 99.0 | 11.6 | 1447 |
| BLine (tuned) | 89/90 | 8/9 | 14.6 | 60.4 | 0.8 | 1180 |
| BLine (tuned, with corner caps) | 90/90 | 9/9 | 10.2 | 98.3 | 0.8 | 1183 |
| PathPlanner (kP 5) | 0/90 | 0/9 | 6.6 | 49.1 | 7.5 | 3384 |
| PathPlanner (kP 5) + hold | 90/90 | 9/9 | 6.6 | 49.1 | 0.1 | 3452 |
| PathPlanner (tuned) + hold | 90/90 | 9/9 | 4.5 | 47.7 | 0.7 | 3554 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 90/90 | 9/9 | 4.6 | 47.9 | 0.1 | 2855 |
| DriveToPose chain | 90/90 | 9/9 | 5.8 | 58.6 | 0.1 | 1924 |
| Choreo + ChoreoLib follower (as shipped) | 64/90 | 5/9 | 9.3 | 93.2 | 5.7 | 2571 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 90/90 | 9/9 | 10.3 | 95.3 | 0.1 | 2361 |
| Ours v2 (as shipped) | 90/90 | 9/9 | 4.8 | 50.0 | 0.1 | 2488 |
| Ours v2 (as shipped, adaptive grip off) | 90/90 | 9/9 | 4.8 | 47.6 | 0.1 | 3105 |
| Ours v2 (tuned) | 90/90 | 9/9 | 4.8 | 50.0 | 0.1 | 2488 |
| PathPlanner path2 (new-path, pre-release), as shipped | 0/90 | 0/9 | 5.4 | 50.3 | 7.2 | 4772 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 90/90 | 9/9 | 5.6 | 38.2 | 1.7 | 3315 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 90/90 | 9/9 | 4.9 | 37.3 | 0.1 | 1672 |

**straight_3m**: 3 m straight, heading held at 0 (6.5 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.0 / 0.1 | 0.0 / 0.0 | 4.1 / 4.3 | 0.0 | 4.3 | 1.35 | never | 522 | 42.96 |
| BLine (README gains, r 0.30) | 0.0 / 0.1 | 0.0 / 0.1 | 32.5 / 33.6 | 0.0 | 33.6 | 1.13 | never | 820 | 0.04 |
| BLine (README gains, r 0.45 default) | 0.0 / 0.1 | 0.0 / 0.1 | 32.5 / 33.6 | 0.0 | 33.6 | 1.13 | never | 820 | 0.04 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 0.0 / 0.1 | 0.0 / 0.1 | 32.5 / 33.6 | 0.0 | 33.6 | 1.13 | never | 820 | 0.04 |
| BLine (tuned) | 0.0 / 0.1 | 0.0 / 0.1 | 0.8 / 0.9 | 0.0 | 0.0 | 2.20 | 2.67 (10/10) | 9 | 0.04 |
| BLine (tuned, with corner caps) | 0.0 / 0.1 | 0.0 / 0.1 | 0.8 / 0.9 | 0.0 | 0.0 | 2.20 | 2.67 (10/10) | 9 | 0.04 |
| PathPlanner (kP 5) | 0.0 / 0.1 | 0.0 / 0.1 | 3.4 / 3.5 | 0.0 | 3.6 | 1.26 | never | 427 | 2.44 |
| PathPlanner (kP 5) + hold | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.1 | 0.0 | 3.6 | 1.26 | 1.71 (10/10) | 428 | 2.32 |
| PathPlanner (tuned) + hold | 0.0 / 0.1 | 0.0 / 0.0 | 1.0 / 1.6 | 0.0 | 1.6 | 1.28 | 1.48 (10/10) | 467 | 2.55 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.1 | 0.0 | 3.6 | 1.26 | 1.71 (10/10) | 428 | 2.34 |
| DriveToPose chain | 0.1 / 0.1 | 0.0 / 0.1 | 0.1 / 0.1 | 0.0 | 0.0 | 1.34 | 1.55 (10/10) | 9 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 0.1 / 0.2 | 0.0 / 0.1 | 1.6 / 1.8 | 0.1 | 2.5 | 1.26 | 1.48 (10/10) | 625 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 0.1 / 0.2 | 0.1 / 0.2 | 0.1 / 0.1 | 0.3 | 0.0 | 1.33 | 1.58 (10/10) | 514 | 2.36 |
| Ours v2 (as shipped) | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.2 | 0.0 | 0.2 | 1.30 | 1.43 (10/10) | 337 | 0.93 |
| Ours v2 (as shipped, adaptive grip off) | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.2 | 0.0 | 0.2 | 1.30 | 1.43 (10/10) | 337 | 0.75 |
| Ours v2 (tuned) | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.2 | 0.0 | 0.2 | 1.30 | 1.43 (10/10) | 337 | 0.89 |
| PathPlanner path2 (new-path, pre-release), as shipped | 0.0 / 0.1 | 0.0 / 0.1 | 8.0 / 8.2 | 0.0 | 0.0 | never | never | 1613 | 0.04 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 0.0 / 0.1 | 0.0 / 0.1 | 1.7 / 1.8 | 0.0 | 0.0 | 2.20 | 2.52 (10/10) | 57 | 0.04 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.2 | 0.0 | 0.2 | 1.32 | 1.46 (10/10) | 226 | 0.89 |

**s_curve**: S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 9.1 | 4.6 / 9.1 | 3.9 / 4.0 | 0.0 | 4.0 | 2.24 | never | 464 | 78.99 |
| BLine (README gains, r 0.30) | 12.9 / 37.7 | 14.1 / 37.7 | 4.7 / 5.2 | 0.1 | 90.3 | 3.00 | never | 1113 | 0.05 |
| BLine (README gains, r 0.45 default) | 8.4 / 26.6 | 9.4 / 26.6 | 3.6 / 3.9 | 0.1 | 85.3 | 2.93 | never | 1018 | 0.05 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.0 / 7.5 | 1.4 / 6.2 | 9.3 / 10.3 | 0.0 | 10.3 | 2.22 | never | 585 | 0.05 |
| BLine (tuned) | 7.0 / 16.5 | 4.9 / 15.8 | 0.8 / 0.9 | 0.0 | 0.0 | 2.72 | 3.18 (10/10) | 447 | 0.04 |
| BLine (tuned, with corner caps) | 2.5 / 5.9 | 3.0 / 9.6 | 0.8 / 0.9 | 0.0 | 0.0 | 2.78 | 3.25 (10/10) | 406 | 0.04 |
| PathPlanner (kP 5) | 0.6 / 1.8 | 0.5 / 1.7 | 3.5 / 3.6 | 0.0 | 3.6 | 1.88 | never | 1596 | 4.10 |
| PathPlanner (kP 5) + hold | 0.6 / 1.8 | 0.5 / 1.7 | 0.1 / 0.1 | 0.0 | 3.6 | 1.88 | 2.34 (10/10) | 1599 | 3.83 |
| PathPlanner (tuned) + hold | 0.4 / 1.2 | 0.3 / 1.1 | 0.8 / 1.6 | 0.0 | 1.7 | 1.91 | 2.13 (10/10) | 1620 | 3.95 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.7 / 1.8 | 0.6 / 1.8 | 0.1 / 0.1 | 0.0 | 3.6 | 1.92 | 2.37 (10/10) | 1384 | 4.02 |
| DriveToPose chain | 2.8 / 6.0 | 0.2 / 0.4 | 0.1 / 0.1 | 0.2 | 0.0 | 4.03 | 4.24 (10/10) | 839 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 4.6 / 8.6 | 1.2 / 1.9 | 1.6 / 1.8 | 0.0 | 2.4 | 1.95 | 2.17 (10/10) | 1147 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.1 / 10.3 | 2.1 / 3.7 | 0.1 / 0.1 | 0.0 | 0.0 | 2.02 | 2.28 (10/10) | 1213 | 3.73 |
| Ours v2 (as shipped) | 1.3 / 3.3 | 1.1 / 2.4 | 0.1 / 0.2 | 0.0 | 0.2 | 1.91 | 2.05 (10/10) | 1655 | 0.95 |
| Ours v2 (as shipped, adaptive grip off) | 1.3 / 3.3 | 1.1 / 2.4 | 0.1 / 0.2 | 0.0 | 0.2 | 1.91 | 2.05 (10/10) | 1655 | 0.92 |
| Ours v2 (tuned) | 1.3 / 3.3 | 1.1 / 2.4 | 0.1 / 0.2 | 0.0 | 0.2 | 1.91 | 2.05 (10/10) | 1655 | 0.88 |
| PathPlanner path2 (new-path, pre-release), as shipped | 2.6 / 6.0 | 3.6 / 11.0 | 3.9 / 5.4 | 1.7 | 0.0 | 1.79 (9/10) | never | 3642 | 0.04 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 2.5 / 5.9 | 1.3 / 4.9 | 1.7 / 1.8 | 0.0 | 0.0 | 2.80 | 3.11 (10/10) | 1853 | 0.03 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.2 / 2.9 | 0.9 / 1.9 | 0.1 / 0.1 | 0.0 | 0.1 | 2.08 | 2.22 (10/10) | 701 | 0.84 |

**hairpin**: 2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held (9.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 7.3 / 13.3 | 7.3 / 13.3 | 4.0 / 4.2 | 0.0 | 4.2 | 2.81 | never | 1707 | 82.95 |
| BLine (README gains, r 0.30) | 46.2 / 99.1 | 46.6 / 102.3 | 8.4 / 9.7 | 0.2 | 9.7 | 6.63 | never | 2904 | 0.05 |
| BLine (README gains, r 0.45 default) | 47.5 / 99.0 | 47.8 / 102.1 | 2.2 / 2.4 | 0.0 | 55.9 | 5.43 | never | 1957 | 0.05 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 47.5 / 99.0 | 47.8 / 102.1 | 2.2 / 2.4 | 0.0 | 55.9 | 5.43 | never | 1957 | 0.05 |
| BLine (tuned) | 15.8 / 38.3 | 16.5 / 39.1 | 0.8 / 0.8 | 0.0 | 0.0 | 3.69 | 4.16 (10/10) | 1158 | 0.06 |
| BLine (tuned, with corner caps) | 41.2 / 98.3 | 42.2 / 101.5 | 0.8 / 0.9 | 0.0 | 0.0 | 3.88 | 4.35 (10/10) | 1058 | 0.05 |
| PathPlanner (kP 5) | 13.2 / 31.2 | 13.3 / 31.3 | 3.6 / 3.7 | 0.6 | 3.7 | 2.35 | never | 4949 | 3.63 |
| PathPlanner (kP 5) + hold | 13.2 / 31.2 | 13.3 / 31.3 | 0.1 / 0.1 | 0.0 | 3.6 | 2.35 | 2.79 (10/10) | 4960 | 3.78 |
| PathPlanner (tuned) + hold | 0.4 / 1.0 | 0.2 / 0.9 | 0.5 / 1.5 | 0.0 | 1.6 | 2.46 | 2.71 (10/10) | 3240 | 3.10 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.9 / 1.8 | 0.7 / 1.7 | 0.1 / 0.1 | 0.0 | 3.5 | 2.57 | 3.02 (10/10) | 2598 | 3.76 |
| DriveToPose chain | 1.2 / 3.2 | 0.1 / 0.3 | 0.1 / 0.1 | 0.0 | 0.0 | 7.71 | 7.92 (10/10) | 701 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 1.4 / 2.4 | 1.3 / 2.0 | 1.5 / 1.7 | 0.0 | 2.5 | 2.84 | 3.07 (10/10) | 1767 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 2.5 / 4.2 | 2.4 / 3.9 | 0.1 / 0.1 | 0.0 | 0.0 | 2.93 | 3.19 (10/10) | 1340 | 7.41 |
| Ours v2 (as shipped) | 1.3 / 2.7 | 1.2 / 2.8 | 0.1 / 0.1 | 0.1 | 0.1 | 2.46 | 2.60 (10/10) | 3479 | 1.66 |
| Ours v2 (as shipped, adaptive grip off) | 1.3 / 2.7 | 1.2 / 2.8 | 0.1 / 0.1 | 0.1 | 0.1 | 2.46 | 2.60 (10/10) | 3479 | 1.82 |
| Ours v2 (tuned) | 1.3 / 2.7 | 1.2 / 2.8 | 0.1 / 0.1 | 0.1 | 0.1 | 2.46 | 2.60 (10/10) | 3479 | 1.60 |
| PathPlanner path2 (new-path, pre-release), as shipped | 4.4 / 22.5 | 4.9 / 25.5 | 8.0 / 8.3 | 0.1 | 0.0 | never | never | 5191 | 0.09 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 8.6 / 23.9 | 9.5 / 26.5 | 1.7 / 1.8 | 0.0 | 0.0 | 3.39 | 3.71 (10/10) | 5423 | 0.08 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.1 / 2.4 | 1.0 / 2.0 | 0.1 / 0.2 | 0.0 | 0.2 | 2.90 | 3.04 (10/10) | 1284 | 2.00 |

**slalom**: through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m (10.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.5 / 10.2 | 5.5 / 10.2 | 4.0 / 4.1 | 0.1 | 4.1 | 3.08 | never | 599 | 141.94 |
| BLine (README gains, r 0.30) | 40.5 / 83.6 | 41.0 / 81.0 | 10.6 / 24.9 | 0.3 | 47.9 | 4.31 | 4.61 (5/10) | 2163 | 0.07 |
| BLine (README gains, r 0.45 default) | 30.1 / 67.9 | 30.0 / 63.4 | 23.0 / 24.8 | 0.1 | 23.3 | 3.88 | never | 2089 | 0.08 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 6.4 / 12.0 | 2.2 / 9.0 | 6.5 / 7.1 | 0.0 | 7.1 | 3.30 | never | 650 | 0.08 |
| BLine (tuned) | 12.0 / 34.1 | 8.7 / 30.7 | 0.8 / 0.9 | 0.0 | 0.0 | 3.20 | 3.67 (10/10) | 342 | 0.08 |
| BLine (tuned, with corner caps) | 4.2 / 9.5 | 2.0 / 8.5 | 0.8 / 0.9 | 0.0 | 0.0 | 3.87 | 4.33 (10/10) | 553 | 0.06 |
| PathPlanner (kP 5) | 1.7 / 6.0 | 1.7 / 6.2 | 3.4 / 3.6 | 0.0 | 3.7 | 2.41 | never | 3888 | 6.21 |
| PathPlanner (kP 5) + hold | 1.7 / 6.0 | 1.7 / 6.2 | 0.1 / 0.1 | 0.0 | 3.7 | 2.41 | 2.86 (10/10) | 3889 | 5.80 |
| PathPlanner (tuned) + hold | 0.3 / 0.8 | 0.2 / 1.3 | 1.0 / 1.7 | 0.1 | 1.7 | 2.49 | 2.69 (10/10) | 3711 | 6.56 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.5 / 1.4 | 0.4 / 1.3 | 0.1 / 0.1 | 0.0 | 3.7 | 2.57 | 3.03 (10/10) | 3170 | 5.69 |
| DriveToPose chain | 4.2 / 8.7 | 0.3 / 0.7 | 0.1 / 0.1 | 0.0 | 0.0 | 4.69 | 4.89 (10/10) | 1229 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 8.4 / 17.0 | 1.4 / 2.0 | 1.6 / 1.7 | 0.0 | 2.3 | 2.77 | 2.99 (10/10) | 1824 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 9.1 / 18.4 | 2.4 / 3.8 | 0.1 / 0.1 | 0.3 | 0.0 | 2.85 | 3.10 (10/10) | 1780 | 9.23 |
| Ours v2 (as shipped) | 1.7 / 4.2 | 1.7 / 4.3 | 0.1 / 0.2 | 0.0 | 0.2 | 2.54 | 2.68 (10/10) | 2831 | 1.33 |
| Ours v2 (as shipped, adaptive grip off) | 1.7 / 3.4 | 1.6 / 3.1 | 0.1 / 0.2 | 0.0 | 0.2 | 2.44 | 2.58 (10/10) | 3148 | 1.37 |
| Ours v2 (tuned) | 1.7 / 4.2 | 1.7 / 4.3 | 0.1 / 0.2 | 0.0 | 0.2 | 2.54 | 2.68 (10/10) | 2831 | 1.37 |
| PathPlanner path2 (new-path, pre-release), as shipped | 5.7 / 29.4 | 6.7 / 33.9 | 6.7 / 8.1 | 1.6 | 0.0 | never | never | 5932 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 4.4 / 8.8 | 2.4 / 12.0 | 1.8 / 1.8 | 0.0 | 0.0 | 3.28 | 3.60 (10/10) | 3273 | 0.07 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.3 / 2.7 | 1.2 / 2.1 | 0.1 / 0.2 | 0.0 | 0.2 | 2.88 | 3.02 (10/10) | 1396 | 1.49 |

**long_auto**: auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end (14.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.0 / 10.7 | 5.0 / 10.7 | 3.5 / 3.8 | 2.0 | 3.8 | 4.10 | never | 741 | 241.48 |
| BLine (README gains, r 0.30) | 32.3 / 86.4 | 36.5 / 100.7 | 1.7 / 2.3 | 0.7 | 64.2 | 5.55 | 6.65 (7/10) | 2164 | 0.09 |
| BLine (README gains, r 0.45 default) | 31.3 / 77.0 | 34.4 / 89.9 | 2.2 / 2.4 | 0.5 | 52.4 | 5.10 | never | 2108 | 0.04 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 10.2 / 24.6 | 2.4 / 11.9 | 2.5 / 2.9 | 1.8 | 3.4 | 4.94 | never | 784 | 0.05 |
| BLine (tuned) | 22.1 / 54.5 | 19.0 / 45.4 | 0.9 / 0.9 | 1.0 | 5.3 | 3.80 | 4.20 (9/10) | 860 | 0.08 |
| BLine (tuned, with corner caps) | 8.6 / 22.4 | 2.2 / 13.4 | 0.8 / 0.9 | 0.4 | 0.0 | 5.24 | 5.70 (10/10) | 546 | 0.04 |
| PathPlanner (kP 5) | 1.6 / 3.2 | 1.6 / 3.6 | 3.3 / 3.5 | 0.1 | 3.5 | 3.58 | never | 2204 | 5.55 |
| PathPlanner (kP 5) + hold | 1.6 / 3.2 | 1.6 / 3.6 | 0.1 / 0.1 | 0.0 | 3.5 | 3.58 | 4.03 (10/10) | 2205 | 5.87 |
| PathPlanner (tuned) + hold | 0.7 / 1.6 | 0.7 / 1.7 | 0.6 / 1.5 | 0.0 | 1.5 | 3.65 | 3.88 (10/10) | 2050 | 7.47 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.3 / 2.9 | 1.3 / 2.9 | 0.1 / 0.1 | 0.0 | 3.5 | 3.70 | 4.15 (10/10) | 1827 | 6.21 |
| DriveToPose chain | 6.8 / 21.2 | 1.2 / 3.4 | 0.1 / 0.3 | 0.2 | 0.3 | 6.57 | 6.71 (10/10) | 1720 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 7.3 / 21.7 | 1.6 / 2.9 | 1.6 / 1.8 | 1.2 | 2.4 | 3.75 | 4.02 (2/10) | 1792 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.0 / 22.1 | 2.9 / 5.8 | 0.2 / 0.2 | 0.3 | 0.0 | 3.83 | 4.09 (10/10) | 1649 | 5.07 |
| Ours v2 (as shipped) | 4.4 / 12.5 | 1.6 / 4.2 | 0.1 / 0.1 | 0.3 | 0.1 | 3.77 | 3.91 (10/10) | 1829 | 1.00 |
| Ours v2 (as shipped, adaptive grip off) | 4.4 / 12.5 | 1.6 / 4.2 | 0.1 / 0.1 | 0.2 | 0.1 | 3.63 | 3.77 (10/10) | 2004 | 0.99 |
| Ours v2 (tuned) | 4.4 / 12.5 | 1.6 / 4.2 | 0.1 / 0.1 | 0.3 | 0.1 | 3.77 | 3.91 (10/10) | 1829 | 1.03 |
| PathPlanner path2 (new-path, pre-release), as shipped | 4.0 / 22.9 | 6.4 / 45.1 | 7.9 / 8.2 | 0.0 | 0.0 | never | never | 3496 | 0.04 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 5.8 / 13.3 | 7.5 / 33.2 | 1.7 / 1.8 | 0.0 | 0.0 | 4.60 | 4.91 (10/10) | 3092 | 0.04 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 4.6 / 12.4 | 1.6 / 4.1 | 0.1 / 0.2 | 0.3 | 0.2 | 3.93 | 4.07 (10/10) | 765 | 0.93 |

**moving_start**: starts moving 2 m/s sideways (+y), path heads +x then curves left (7.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 11.8 / 27.1 | 11.8 / 27.1 | 4.0 / 4.1 | 0.1 | 4.1 | 1.90 | never | 1574 | 61.20 |
| BLine (README gains, r 0.30) | 31.3 / 58.3 | 32.4 / 65.9 | 22.8 / 24.4 | 0.1 | 24.3 | 3.49 | never | 2184 | 0.04 |
| BLine (README gains, r 0.45 default) | 33.5 / 54.6 | 34.7 / 51.4 | 12.0 / 29.3 | 0.4 | 55.4 | 2.18 | 3.07 (6/10) | 1496 | 0.05 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 24.7 / 49.6 | 25.4 / 47.1 | 8.9 / 9.9 | 0.0 | 9.9 | 2.18 | never | 1232 | 0.04 |
| BLine (tuned) | 33.1 / 60.4 | 30.2 / 56.4 | 0.8 / 0.9 | 0.0 | 0.0 | 2.27 | 2.74 (10/10) | 777 | 0.04 |
| BLine (tuned, with corner caps) | 20.3 / 47.6 | 21.1 / 45.6 | 0.8 / 0.9 | 0.0 | 0.0 | 3.10 | 3.57 (10/10) | 829 | 0.04 |
| PathPlanner (kP 5) | 12.9 / 25.3 | 12.9 / 25.3 | 3.4 / 3.6 | 0.1 | 3.7 | 1.44 | never | 3064 | 3.20 |
| PathPlanner (kP 5) + hold | 12.9 / 25.3 | 12.9 / 25.3 | 0.1 / 0.1 | 0.0 | 3.7 | 1.44 | 1.90 (10/10) | 3068 | 3.32 |
| PathPlanner (tuned) + hold | 10.9 / 24.4 | 10.9 / 24.4 | 0.8 / 1.6 | 0.0 | 1.6 | 1.46 | 1.67 (10/10) | 3444 | 3.21 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 12.9 / 25.3 | 12.9 / 25.3 | 0.1 / 0.1 | 0.0 | 3.7 | 1.44 | 1.90 (10/10) | 3081 | 3.21 |
| DriveToPose chain | 21.6 / 58.6 | 20.1 / 54.8 | 0.1 / 0.1 | 0.0 | 0.0 | 3.68 | 3.87 (10/10) | 1340 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 8.3 / 20.1 | 7.7 / 20.7 | 1.6 / 1.9 | 0.0 | 2.4 | 1.72 | 1.93 (10/10) | 3028 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 11.4 / 27.2 | 11.4 / 28.6 | 0.1 / 0.1 | 0.3 | 0.0 | 1.84 | 2.10 (10/10) | 1932 | 3.08 |
| Ours v2 (as shipped) | 8.6 / 23.3 | 8.7 / 23.3 | 0.1 / 0.2 | 0.0 | 0.2 | 1.74 | 1.87 (10/10) | 2094 | 1.10 |
| Ours v2 (as shipped, adaptive grip off) | 8.6 / 23.3 | 8.7 / 23.3 | 0.1 / 0.2 | 0.0 | 0.2 | 1.74 | 1.87 (10/10) | 2094 | 1.13 |
| Ours v2 (tuned) | 8.6 / 23.3 | 8.7 / 23.3 | 0.1 / 0.2 | 0.0 | 0.2 | 1.74 | 1.87 (10/10) | 2094 | 1.28 |
| PathPlanner path2 (new-path, pre-release), as shipped | 5.8 / 30.7 | 5.7 / 29.1 | 7.9 / 8.4 | 0.1 | 0.0 | never | never | 3552 | 0.05 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 10.0 / 29.1 | 8.9 / 27.9 | 1.7 / 1.8 | 0.0 | 0.0 | 2.44 | 2.75 (10/10) | 3169 | 0.05 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 18.7 / 37.3 | 18.8 / 37.4 | 0.1 / 0.2 | 0.0 | 7.1 | 1.98 | 2.39 (10/10) | 1690 | 1.34 |

**hit_mid_path**: the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 13.2 / 38.3 | 13.2 / 38.3 | 3.3 / 3.6 | 0.7 | 3.4 | 2.34 | never | 3169 | 151.64 |
| BLine (README gains, r 0.30) | 34.3 / 69.5 | 36.2 / 74.9 | 32.5 / 34.6 | 0.1 | 34.5 | 4.26 | never | 4206 | 0.14 |
| BLine (README gains, r 0.45 default) | 24.7 / 57.1 | 26.6 / 62.7 | 2.2 / 2.4 | 0.0 | 73.7 | 3.07 | never | 3720 | 0.08 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 8.6 / 24.9 | 9.8 / 29.6 | 9.6 / 10.5 | 0.3 | 10.5 | 2.30 | never | 2724 | 0.07 |
| BLine (tuned) | 27.0 / 57.3 | 28.7 / 63.1 | 0.8 / 0.9 | 0.0 | 9.7 | 2.10 | 3.07 (10/10) | 3258 | 0.07 |
| BLine (tuned, with corner caps) | 9.6 / 27.9 | 10.9 / 30.6 | 0.8 / 0.8 | 0.0 | 0.0 | 2.67 | 3.13 (10/10) | 2898 | 0.06 |
| PathPlanner (kP 5) | 21.5 / 49.1 | 21.6 / 49.3 | 10.5 / 10.8 | 8.2 | 10.4 | never | never | 4946 | 4.81 |
| PathPlanner (kP 5) + hold | 21.5 / 49.1 | 21.6 / 49.3 | 0.2 / 0.2 | 0.2 | 10.4 | 2.21 | 2.51 (10/10) | 5000 | 9.56 |
| PathPlanner (tuned) + hold | 20.6 / 47.7 | 20.6 / 47.9 | 0.8 / 0.9 | 0.2 | 16.0 | 1.90 | 2.43 (10/10) | 5951 | 8.83 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 20.8 / 47.9 | 20.8 / 48.2 | 0.1 / 0.1 | 0.0 | 6.1 | 2.12 | 2.44 (10/10) | 4703 | 4.13 |
| DriveToPose chain | 9.9 / 36.9 | 10.6 / 39.2 | 0.1 / 0.1 | 0.2 | 0.0 | 4.43 | 4.64 (10/10) | 3064 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 44.1 / 93.2 | 42.9 / 92.7 | 38.2 / 40.1 | 3.6 | 26.2 | 2.11 | never | 5396 | 0.02 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 46.1 / 95.3 | 44.7 / 93.5 | 0.1 / 0.1 | 0.3 | 33.6 | 2.71 | 2.97 (10/10) | 5714 | 4.29 |
| Ours v2 (as shipped) | 21.3 / 50.0 | 21.6 / 51.1 | 0.2 / 0.3 | 0.2 | 0.1 | 1.97 | 2.18 (10/10) | 4926 | 3.50 |
| Ours v2 (as shipped, adaptive grip off) | 20.2 / 47.6 | 20.5 / 48.7 | 0.1 / 0.2 | 0.2 | 0.1 | 1.97 | 2.13 (10/10) | 4814 | 2.20 |
| Ours v2 (tuned) | 21.3 / 50.0 | 21.6 / 51.1 | 0.2 / 0.3 | 0.2 | 0.1 | 1.97 | 2.18 (10/10) | 4926 | 2.01 |
| PathPlanner path2 (new-path, pre-release), as shipped | 10.1 / 50.3 | 11.2 / 56.1 | 7.9 / 8.2 | 0.2 | 0.0 | never | never | 5467 | 0.08 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 11.7 / 38.2 | 13.5 / 43.9 | 1.7 / 1.8 | 0.0 | 0.0 | 2.87 | 3.19 (10/10) | 4113 | 0.08 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 15.2 / 36.7 | 15.3 / 37.1 | 0.1 / 0.1 | 0.2 | 12.4 | 2.11 | 2.68 (10/10) | 3223 | 1.99 |

**low_grip_s_curve**: the S-curve on mu 0.91 static / 0.79 kinetic (61 % of normal grip) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 9.0 | 4.6 / 9.0 | 5.9 / 6.0 | 0.1 | 6.0 | 2.24 | never | 2610 | 134.15 |
| BLine (README gains, r 0.30) | 13.5 / 37.4 | 14.7 / 37.5 | 9.6 / 10.2 | 0.3 | 91.4 | 3.01 | never | 5671 | 0.07 |
| BLine (README gains, r 0.45 default) | 8.4 / 27.1 | 9.4 / 27.1 | 7.3 / 7.7 | 0.1 | 87.0 | 2.94 | never | 5332 | 0.07 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.9 / 6.7 | 1.3 / 6.2 | 16.4 / 17.1 | 0.0 | 17.1 | 2.22 | never | 2723 | 0.06 |
| BLine (tuned) | 7.0 / 16.3 | 4.8 / 15.5 | 0.8 / 0.9 | 0.0 | 0.0 | 2.71 | 3.17 (10/10) | 2606 | 0.07 |
| BLine (tuned, with corner caps) | 2.5 / 5.8 | 3.1 / 9.7 | 0.8 / 0.9 | 0.0 | 0.0 | 2.76 | 3.23 (10/10) | 2979 | 0.08 |
| PathPlanner (kP 5) | 4.4 / 10.2 | 4.4 / 10.2 | 18.8 / 19.2 | 2.1 | 19.0 | 1.82 | never | 5564 | 4.49 |
| PathPlanner (kP 5) + hold | 4.4 / 10.2 | 4.4 / 10.2 | 0.1 / 0.1 | 0.0 | 19.0 | 1.82 | 2.63 (10/10) | 5852 | 4.04 |
| PathPlanner (tuned) + hold | 4.0 / 9.0 | 4.1 / 9.0 | 0.6 / 0.7 | 0.2 | 16.8 | 1.81 | 2.37 (10/10) | 6730 | 5.30 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 2.3 / 6.5 | 2.3 / 6.6 | 0.1 / 0.1 | 0.0 | 5.8 | 1.88 | 2.43 (10/10) | 5097 | 4.80 |
| DriveToPose chain | 2.8 / 6.0 | 0.2 / 0.7 | 0.2 / 0.3 | 0.2 | 0.3 | 3.85 | 3.99 (10/10) | 5316 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 4.7 / 8.7 | 1.3 / 2.3 | 1.9 / 2.2 | 0.1 | 2.8 | 1.94 | 2.18 (6/10) | 4594 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.3 / 10.5 | 2.2 / 3.8 | 0.1 / 0.1 | 0.2 | 0.0 | 2.02 | 2.26 (10/10) | 4326 | 4.57 |
| Ours v2 (as shipped) | 1.9 / 8.0 | 1.8 / 8.0 | 0.2 / 0.3 | 0.1 | 9.0 | 2.84 | 3.01 (10/10) | 865 | 2.20 |
| Ours v2 (as shipped, adaptive grip off) | 3.6 / 8.2 | 3.7 / 8.1 | 0.1 / 0.1 | 0.3 | 9.6 | 1.83 | 2.28 (10/10) | 6036 | 1.71 |
| Ours v2 (tuned) | 1.9 / 8.0 | 1.8 / 8.0 | 0.2 / 0.3 | 0.1 | 9.0 | 2.84 | 3.01 (10/10) | 865 | 1.56 |
| PathPlanner path2 (new-path, pre-release), as shipped | 8.3 / 20.5 | 9.8 / 23.0 | 7.4 / 7.6 | 1.3 | 34.8 | 1.76 (1/10) | never | 8387 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 3.9 / 9.9 | 5.5 / 15.1 | 1.7 / 1.8 | 0.0 | 0.0 | 2.74 | 3.05 (10/10) | 5665 | 0.07 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.1 / 2.7 | 0.9 / 1.8 | 0.1 / 0.3 | 0.1 | 0.4 | 2.08 | 2.25 (10/10) | 3551 | 1.32 |

**grip_drop_s_curve**: the S-curve; halfway along, the floor drops from mu 1.5 / 1.3 to 0.91 / 0.79 (a worn carpet seam) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 9.0 | 4.6 / 9.0 | 5.9 / 6.0 | 0.1 | 6.0 | 2.23 | never | 1742 | 72.62 |
| BLine (README gains, r 0.30) | 12.9 / 38.0 | 14.1 / 38.0 | 9.3 / 9.8 | 0.3 | 91.8 | 3.00 | never | 3816 | 0.04 |
| BLine (README gains, r 0.45 default) | 8.5 / 26.9 | 9.5 / 26.9 | 7.4 / 7.8 | 0.1 | 87.0 | 2.94 | never | 3624 | 0.04 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.0 / 7.4 | 1.3 / 6.2 | 16.2 / 17.4 | 0.0 | 17.4 | 2.21 | never | 1551 | 0.05 |
| BLine (tuned) | 7.0 / 16.5 | 4.8 / 15.8 | 0.8 / 0.9 | 0.0 | 0.0 | 2.70 | 3.17 (10/10) | 1161 | 0.05 |
| BLine (tuned, with corner caps) | 2.5 / 5.9 | 3.1 / 9.8 | 0.8 / 0.8 | 0.0 | 0.0 | 2.75 | 3.22 (10/10) | 1369 | 0.04 |
| PathPlanner (kP 5) | 3.3 / 9.5 | 3.3 / 9.5 | 17.4 / 18.0 | 1.7 | 17.8 | 1.81 | never | 3816 | 3.60 |
| PathPlanner (kP 5) + hold | 3.3 / 9.5 | 3.3 / 9.5 | 0.1 / 0.1 | 0.0 | 17.8 | 1.81 | 2.61 (10/10) | 4066 | 3.43 |
| PathPlanner (tuned) + hold | 2.9 / 8.9 | 3.0 / 8.9 | 0.6 / 0.7 | 0.2 | 16.9 | 1.81 | 2.37 (10/10) | 4775 | 3.65 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 2.0 / 6.2 | 2.0 / 6.3 | 0.1 / 0.1 | 0.0 | 5.6 | 1.89 | 2.43 (10/10) | 3410 | 3.87 |
| DriveToPose chain | 2.8 / 6.0 | 0.2 / 0.5 | 0.1 / 0.3 | 0.1 | 0.3 | 3.98 | 4.12 (10/10) | 3096 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 4.7 / 8.6 | 1.3 / 2.3 | 1.9 / 2.2 | 0.1 | 2.8 | 1.94 | 2.18 (6/10) | 2968 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.2 / 10.3 | 2.1 / 3.7 | 0.1 / 0.1 | 0.2 | 0.0 | 2.00 | 2.24 (10/10) | 2780 | 3.92 |
| Ours v2 (as shipped) | 2.1 / 6.2 | 2.4 / 7.1 | 0.1 / 0.1 | 0.3 | 8.0 | 1.81 | 2.22 (10/10) | 4376 | 1.54 |
| Ours v2 (as shipped, adaptive grip off) | 2.1 / 6.2 | 2.4 / 7.1 | 0.1 / 0.1 | 0.3 | 8.0 | 1.81 | 2.22 (10/10) | 4376 | 1.69 |
| Ours v2 (tuned) | 2.1 / 6.2 | 2.4 / 7.1 | 0.1 / 0.1 | 0.3 | 8.0 | 1.81 | 2.22 (10/10) | 4376 | 1.70 |
| PathPlanner path2 (new-path, pre-release), as shipped | 7.2 / 21.5 | 8.5 / 22.6 | 7.2 / 7.4 | 1.3 | 37.3 | never | never | 5665 | 0.05 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 3.7 / 11.6 | 4.9 / 16.8 | 1.7 / 1.8 | 0.0 | 0.0 | 2.69 | 3.01 (10/10) | 3192 | 0.05 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.1 / 2.7 | 0.9 / 1.9 | 0.2 / 0.3 | 0.1 | 0.3 | 2.07 | 2.22 (10/10) | 2208 | 1.72 |

**spin_while_driving**: 5 m straight while the robot turns a full 360 deg (a heading target at every waypoint) (8.2 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 1.9 / 3.3 | 1.9 / 3.3 | 2.1 / 2.4 | 14.4 | 2.3 | 2.21 | never | 400 | 96.78 |
| BLine (README gains, r 0.30) | 2.8 / 5.2 | 2.8 / 5.2 | 2.3 / 2.5 | 1.8 | 55.7 | 1.84 | never | 624 | 0.07 |
| BLine (README gains, r 0.45 default) | 3.2 / 5.7 | 3.2 / 5.7 | 2.3 / 2.5 | 1.8 | 55.6 | 1.84 | never | 609 | 0.08 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.2 / 5.7 | 3.2 / 5.7 | 2.3 / 2.5 | 1.8 | 55.6 | 1.84 | never | 609 | 0.06 |
| BLine (tuned) | 3.5 / 7.4 | 3.5 / 7.4 | 0.9 / 0.9 | 1.9 | 0.0 | 2.88 | never | 152 | 0.07 |
| BLine (tuned, with corner caps) | 1.6 / 3.5 | 1.6 / 3.5 | 0.9 / 0.9 | 1.9 | 0.0 | 2.88 | never | 154 | 0.07 |
| PathPlanner (kP 5) | 1.6 / 3.4 | 1.6 / 3.4 | 2.3 / 2.5 | 0.4 | 2.5 | 2.96 | never | 107 | 5.80 |
| PathPlanner (kP 5) + hold | 1.6 / 3.4 | 1.6 / 3.4 | 0.1 / 0.1 | 0.0 | 2.5 | 2.96 | 3.36 (10/10) | 107 | 4.91 |
| PathPlanner (tuned) + hold | 0.9 / 1.9 | 0.9 / 1.9 | 0.5 / 1.1 | 0.5 | 1.3 | 2.95 | 3.17 (10/10) | 192 | 5.40 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.6 / 3.3 | 1.6 / 3.3 | 0.1 / 0.1 | 0.0 | 2.8 | 2.95 | 3.36 (10/10) | 155 | 4.01 |
| DriveToPose chain | 0.9 / 3.4 | 0.9 / 3.4 | 0.2 / 0.2 | 1.7 | 0.7 | 4.90 | never | 969 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 2.9 / 6.7 | 1.8 / 3.7 | 1.7 / 1.9 | 3.6 | 2.2 | 1.95 | never | 954 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 6.0 / 13.0 | 5.2 / 12.6 | 0.2 / 0.2 | 1.7 | 0.0 | 2.09 | never | 498 | 4.57 |
| Ours v2 (as shipped) | 2.8 / 4.5 | 2.8 / 4.5 | 0.2 / 0.2 | 0.2 | 0.2 | 2.27 | 2.81 (10/10) | 456 | 1.44 |
| Ours v2 (as shipped, adaptive grip off) | 2.8 / 4.5 | 2.8 / 4.5 | 0.2 / 0.2 | 0.2 | 0.2 | 2.27 | 2.81 (10/10) | 456 | 1.40 |
| Ours v2 (tuned) | 2.8 / 4.5 | 2.8 / 4.5 | 0.2 / 0.2 | 0.2 | 0.2 | 2.27 | 2.81 (10/10) | 456 | 1.58 |
| PathPlanner path2 (new-path, pre-release), as shipped | 1.4 / 3.2 | 1.4 / 3.2 | 1.8 / 2.0 | 1.7 | 0.0 | 2.08 | never | 1394 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 1.7 / 5.1 | 1.7 / 5.1 | 1.7 / 1.8 | 0.0 | 0.0 | 2.84 | 3.15 (10/10) | 1485 | 0.05 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 2.1 / 3.7 | 2.1 / 3.7 | 0.1 / 0.2 | 0.2 | 1.1 | 2.62 | 3.07 (10/10) | 53 | 1.58 |

**figure_eight**: a self-crossing loop (a lemniscate, 4 m x 1.8 m) through the origin twice; heading held (12.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.4 / 9.6 | 5.4 / 9.6 | 3.0 / 3.1 | 0.0 | 3.1 | 5.10 | never | 825 | 186.08 |
| BLine (README gains, r 0.30) | 43.7 / 112.4 | 44.0 / 112.5 | 2.2 / 2.4 | 0.1 | 60.2 | 9.87 | 10.02 (1/10) | 4226 | 0.11 |
| BLine (README gains, r 0.45 default) | 50.5 / 110.2 | 51.4 / 112.5 | 2.2 / 2.3 | 0.0 | 70.2 | 6.56 | never | 3064 | 0.11 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 40.8 / 89.3 | 41.8 / 92.7 | 2.2 / 2.7 | 0.1 | 49.9 | 6.08 | 6.19 (3/10) | 2836 | 0.10 |
| BLine (tuned) | 58.0 / 152.6 | 58.8 / 154.4 | 0.8 / 0.8 | 0.0 | 0.0 | 3.69 | 4.15 (10/10) | 1152 | 0.10 |
| BLine (tuned, with corner caps) | 43.8 / 93.0 | 44.8 / 95.3 | 0.8 / 0.9 | 0.0 | 0.0 | 5.91 | 6.35 (10/10) | 2806 | 0.10 |
| PathPlanner (kP 5) | 11.9 / 32.9 | 12.0 / 33.0 | 7.8 / 8.1 | 6.3 | 8.9 | never | never | 8424 | 12.52 |
| PathPlanner (kP 5) + hold | 11.9 / 32.9 | 12.0 / 33.0 | 0.2 / 0.2 | 0.2 | 8.9 | 3.58 | 3.89 (10/10) | 8382 | 11.09 |
| PathPlanner (tuned) + hold | 1.4 / 6.1 | 1.5 / 6.3 | 0.4 / 1.6 | 0.0 | 1.7 | 3.49 | 3.76 (10/10) | 6583 | 11.92 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.1 / 2.8 | 1.0 / 3.0 | 0.1 / 0.1 | 0.0 | 3.3 | 3.65 | 4.09 (10/10) | 5612 | 12.05 |
| DriveToPose chain | 1.3 / 3.9 | 0.1 / 0.4 | 21.2 / 27.8 | 0.0 | 0.0 | never | never | 2678 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 1.9 / 5.4 | 1.4 / 2.0 | 1.8 / 2.1 | 0.0 | 2.4 | 4.29 | 4.50 (9/10) | 2843 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 2.8 / 5.9 | 2.5 / 3.8 | 0.1 / 0.1 | 0.2 | 0.0 | 4.41 | 4.66 (10/10) | 2291 | 10.76 |
| Ours v2 (as shipped) | 1.6 / 3.2 | 1.6 / 2.8 | 0.1 / 0.2 | 0.2 | 0.2 | 3.56 | 3.70 (10/10) | 5830 | 1.83 |
| Ours v2 (as shipped, adaptive grip off) | 1.6 / 3.2 | 1.6 / 2.8 | 0.1 / 0.2 | 0.2 | 0.2 | 3.56 | 3.70 (10/10) | 5830 | 1.91 |
| Ours v2 (tuned) | 1.6 / 3.2 | 1.6 / 2.8 | 0.1 / 0.2 | 0.2 | 0.2 | 3.56 | 3.70 (10/10) | 5830 | 2.03 |
| PathPlanner path2 (new-path, pre-release), as shipped | 21.9 / 49.4 | 22.9 / 51.8 | 2.6 / 2.9 | 1.6 | 1.6 | 3.84 | never | 11253 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 18.3 / 50.6 | 19.0 / 53.1 | 1.8 / 1.8 | 0.0 | 0.0 | 4.57 | 4.88 (10/10) | 11082 | 0.06 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.1 / 2.4 | 1.2 / 1.9 | 0.2 / 0.2 | 0.1 | 0.2 | 4.49 | 4.63 (10/10) | 2026 | 2.24 |

**wall_hug**: along a wall with the bumpers 15 cm off it, then a 90 deg turn to run 15 cm off the next wall (11.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.7 / 13.0 | 4.6 / 12.4 | 4.1 / 4.2 | 0.1 | 4.2 | 3.04 | never | 915 | 141.96 |
| BLine (README gains, r 0.30) | 39.9 / 94.5 | 39.7 / 94.5 | 1.0 / 1.3 | 0.1 | 65.3 | 5.61 | 5.72 (10/10) | 1926 | 0.09 |
| BLine (README gains, r 0.45 default) | 38.6 / 91.1 | 38.7 / 91.1 | 2.1 / 2.3 | 0.0 | 58.3 | 5.48 | 5.62 (3/10) | 1857 | 0.08 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 38.6 / 91.1 | 38.7 / 91.1 | 2.1 / 2.3 | 0.0 | 58.3 | 5.48 | 5.62 (3/10) | 1857 | 0.07 |
| BLine (tuned) | 21.0 / 51.2 | 20.9 / 51.2 | 0.7 / 0.8 | 0.0 | 0.0 | 2.92 | 3.05 (10/10) | 822 | 0.08 |
| BLine (tuned, with corner caps) | 40.1 / 92.6 | 40.4 / 92.6 | 0.8 / 0.9 | 0.0 | 8.6 | 4.04 | 4.48 (10/10) | 983 | 0.09 |
| PathPlanner (kP 5) | 1.6 / 6.3 | 1.7 / 6.3 | 2.4 / 2.6 | 0.0 | 2.7 | 2.69 | never | 2232 | 6.05 |
| PathPlanner (kP 5) + hold | 1.6 / 6.3 | 1.7 / 6.3 | 0.7 / 0.7 | 0.0 | 2.7 | 2.69 | 3.13 (10/10) | 2232 | 8.54 |
| PathPlanner (tuned) + hold | 0.2 / 0.9 | 0.2 / 1.0 | 0.6 / 0.9 | 0.0 | 0.7 | 2.78 | 2.96 (10/10) | 2103 | 5.27 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.5 / 1.6 | 0.3 / 1.6 | 0.7 / 0.7 | 0.0 | 2.7 | 2.83 | 3.27 (10/10) | 1616 | 5.35 |
| DriveToPose chain | 1.0 / 4.0 | 0.1 / 0.2 | 0.1 / 0.1 | 0.0 | 0.0 | 7.23 | 7.43 (10/10) | 359 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 1.3 / 3.1 | 0.9 / 2.0 | 1.5 / 1.6 | 0.1 | 2.5 | 2.94 | 3.17 (10/10) | 1235 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.9 / 4.9 | 1.5 / 3.6 | 0.1 / 0.1 | 0.3 | 0.0 | 3.01 | 3.26 (10/10) | 1062 | 6.38 |
| Ours v2 (as shipped) | 1.3 / 3.6 | 1.1 / 3.1 | 0.1 / 0.2 | 0.0 | 0.2 | 2.74 | 2.88 (10/10) | 1864 | 1.73 |
| Ours v2 (as shipped, adaptive grip off) | 1.3 / 3.6 | 1.1 / 3.1 | 0.1 / 0.2 | 0.0 | 0.2 | 2.74 | 2.88 (10/10) | 1864 | 1.42 |
| Ours v2 (tuned) | 1.3 / 3.6 | 1.1 / 3.1 | 0.1 / 0.2 | 0.0 | 0.2 | 2.74 | 2.88 (10/10) | 1864 | 1.61 |
| PathPlanner path2 (new-path, pre-release), as shipped | 2.4 / 20.3 | 2.6 / 20.3 | 7.9 / 8.3 | 0.3 | 0.0 | never | never | 4046 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 5.9 / 20.5 | 6.4 / 21.2 | 1.7 / 1.8 | 0.0 | 0.0 | 3.65 | 3.96 (10/10) | 4129 | 0.08 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.1 / 2.6 | 0.8 / 2.1 | 0.1 / 0.2 | 0.0 | 0.2 | 3.02 | 3.16 (10/10) | 814 | 1.74 |

**tag_align_moving_goal**: 5 m to a scoring pose; the true goal is 8 cm sideways and 3 deg off the map pose, revealed 1.5 m out (8.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 3.2 / 7.2 | 3.2 / 7.2 | 0.1 / 0.1 | 0.2 | 7.0 | 2.18 | 2.44 (10/10) | 476 | 107.45 |
| BLine (README gains, r 0.30) | 3.4 / 7.3 | 4.4 / 12.5 | 0.1 / 0.1 | 0.2 | 82.2 | 2.81 | 3.02 (10/10) | 515 | 0.09 |
| BLine (README gains, r 0.45 default) | 3.3 / 7.3 | 2.1 / 7.6 | 0.1 / 0.1 | 0.1 | 83.4 | 2.80 | 2.99 (10/10) | 442 | 0.07 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.5 / 7.9 | 0.7 / 4.3 | 0.1 / 0.1 | 0.2 | 51.6 | 2.71 | 2.92 (10/10) | 300 | 0.06 |
| BLine (tuned) | 6.5 / 16.4 | 3.9 / 15.6 | 0.1 / 0.2 | 0.2 | 1.2 | 3.32 | 3.58 (10/10) | 198 | 0.09 |
| BLine (tuned, with corner caps) | 3.5 / 7.3 | 0.6 / 4.6 | 0.1 / 0.1 | 0.2 | 1.2 | 3.34 | 3.61 (10/10) | 206 | 0.08 |
| PathPlanner (kP 5) | 2.9 / 7.8 | 2.9 / 7.8 | 3.3 / 3.7 | 0.1 | 3.7 | 1.79 | never | 418 | 6.06 |
| PathPlanner (kP 5) + hold | 2.9 / 7.8 | 2.9 / 7.8 | 0.2 / 0.9 | 0.0 | 3.6 | 1.79 | 2.25 (10/10) | 421 | 4.03 |
| PathPlanner (tuned) + hold | 3.0 / 7.8 | 3.0 / 7.8 | 1.0 / 1.7 | 0.0 | 1.7 | 1.82 | 2.02 (10/10) | 516 | 4.50 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 2.9 / 7.8 | 2.9 / 7.8 | 0.2 / 0.9 | 0.0 | 3.7 | 1.79 | 2.25 (10/10) | 425 | 3.79 |
| DriveToPose chain | 4.3 / 7.6 | 2.4 / 7.6 | 0.1 / 0.1 | 0.3 | 0.0 | 2.64 | 2.84 (10/10) | 379 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 0.7 / 1.6 | 0.6 / 1.8 | 0.1 / 0.2 | 0.2 | 4.1 | 2.12 | 2.38 (10/10) | 657 | 0.02 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.5 / 6.2 | 1.6 / 6.0 | 0.1 / 0.1 | 0.3 | 0.0 | 1.84 | 2.09 (10/10) | 768 | 4.23 |
| Ours v2 (as shipped) | 2.8 / 7.6 | 2.8 / 7.6 | 0.1 / 0.1 | 0.2 | 0.1 | 1.89 | 2.03 (10/10) | 426 | 1.40 |
| Ours v2 (as shipped, adaptive grip off) | 2.8 / 7.6 | 2.8 / 7.6 | 0.1 / 0.1 | 0.2 | 0.1 | 1.82 | 1.95 (10/10) | 472 | 1.60 |
| Ours v2 (tuned) | 2.8 / 7.6 | 2.8 / 7.6 | 0.1 / 0.1 | 0.2 | 0.1 | 1.89 | 2.03 (10/10) | 426 | 1.66 |
| PathPlanner path2 (new-path, pre-release), as shipped | 3.7 / 7.4 | 0.7 / 4.1 | 0.1 / 0.2 | 0.2 | 0.0 | 1.82 | 2.08 (10/10) | 2018 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 3.4 / 7.3 | 0.2 / 2.1 | 0.1 / 0.2 | 0.2 | 0.5 | 3.16 | 3.42 (10/10) | 726 | 0.08 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 2.8 / 7.6 | 2.8 / 7.6 | 0.1 / 0.2 | 0.2 | 0.2 | 1.84 | 1.98 (10/10) | 424 | 1.55 |

**intake_drive_through**: drive through a game piece at 2 m/s; its detected position moves 10 cm sideways 2 m out (7.4 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.9 / 2.1 | 0.9 / 2.1 | 10.8 / 10.8 | 0.0 | 4.2 | never | never | 502 | 106.97 |
| BLine (README gains, r 0.30) | 3.6 / 9.8 | 3.5 / 9.8 | 32.7 / 35.7 | 0.1 | 35.6 | 1.39 | never | 953 | 0.07 |
| BLine (README gains, r 0.45 default) | 3.7 / 9.9 | 3.5 / 9.9 | 32.7 / 35.8 | 0.1 | 35.8 | 1.39 | never | 939 | 0.07 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 3.7 / 9.9 | 3.5 / 9.9 | 32.7 / 35.8 | 0.1 | 35.8 | 1.39 | never | 939 | 0.06 |
| BLine (tuned) | 4.4 / 9.8 | 4.3 / 9.8 | 17.5 / 18.5 | 0.0 | 39.4 | 1.65 | never | 746 | 0.08 |
| BLine (tuned, with corner caps) | 4.3 / 9.8 | 4.2 / 9.8 | 17.6 / 18.6 | 0.0 | 39.5 | 1.65 | never | 768 | 0.07 |
| PathPlanner (kP 5) | 3.9 / 10.0 | 3.9 / 10.0 | 20.7 / 21.7 | 0.2 | 21.7 | 1.42 | never | 856 | 4.20 |
| PathPlanner (kP 5) + hold | 3.9 / 10.0 | 3.9 / 10.0 | 20.7 / 21.7 | 0.2 | 21.7 | 1.42 | never | 856 | 8.01 |
| PathPlanner (tuned) + hold | 4.0 / 9.9 | 4.0 / 9.9 | 18.8 / 19.8 | 0.1 | 19.8 | 1.42 | never | 979 | 3.73 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 3.9 / 10.0 | 3.9 / 10.0 | 20.7 / 21.7 | 0.2 | 21.7 | 1.42 | never | 856 | 4.11 |
| DriveToPose chain | 3.7 / 9.8 | 3.6 / 9.8 | 0.1 / 0.1 | 0.1 | 0.0 | 2.36 | 2.56 (10/10) | 250 | 0.02 |
| Choreo + ChoreoLib follower (as shipped) | 0.2 / 0.4 | 0.2 / 0.5 | 23.8 / 24.1 | 0.4 | 21.9 | never | never | 969 | 0.02 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 1.5 / 7.5 | 1.6 / 7.6 | 0.1 / 0.1 | 0.2 | 0.0 | 1.59 | 1.84 (10/10) | 728 | 5.57 |
| Ours v2 (as shipped) | 4.0 / 9.8 | 4.0 / 9.8 | 19.3 / 21.7 | 0.2 | 21.7 | 1.47 | never | 890 | 1.71 |
| Ours v2 (as shipped, adaptive grip off) | 4.0 / 9.8 | 4.0 / 9.8 | 20.2 / 21.7 | 0.2 | 21.7 | 1.40 | never | 946 | 1.49 |
| Ours v2 (tuned) | 4.0 / 9.8 | 4.0 / 9.8 | 19.3 / 21.7 | 0.2 | 21.7 | 1.47 | never | 890 | 1.68 |
| PathPlanner path2 (new-path, pre-release), as shipped | 0.4 / 1.6 | 0.1 / 0.5 | 12.8 / 13.0 | 0.1 | 0.0 | never | never | 1634 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 1.3 / 5.9 | 1.4 / 6.6 | 10.1 / 10.2 | 0.0 | 0.0 | never | never | 828 | 0.08 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 4.0 / 9.8 | 4.0 / 9.8 | 20.0 / 21.7 | 0.2 | 21.7 | 1.42 | never | 886 | 1.38 |

**three_piece_chain**: three paths with rolling handoffs at 2 m/s (core-builder's neutral_three_piece, 2026 field) (15.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 7.2 / 20.9 | 4.2 / 11.1 | 3.2 / 3.4 | 3.8 | 3.3 | 5.58 | never | 1719 | 129.78 |
| BLine (README gains, r 0.30) | 20.1 / 54.8 | 21.3 / 54.8 | 5.6 / 7.3 | 0.2 | 92.1 | 6.71 | never | 4907 | 0.08 |
| BLine (README gains, r 0.45 default) | 18.7 / 49.7 | 19.6 / 49.7 | 5.7 / 6.3 | 0.3 | 90.4 | 6.63 | never | 4926 | 0.08 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 7.5 / 23.5 | 6.9 / 26.1 | 2.4 / 2.9 | 1.8 | 3.5 | 5.05 | never | 3038 | 0.08 |
| BLine (tuned) | 12.5 / 35.1 | 11.4 / 33.6 | 0.9 / 0.9 | 1.0 | 0.0 | 6.49 | 6.49 (7/10) | 3679 | 0.07 |
| BLine (tuned, with corner caps) | 9.7 / 33.0 | 9.6 / 33.0 | 0.9 / 0.9 | 0.7 | 0.0 | 7.86 | 8.30 (10/10) | 4197 | 0.07 |
| PathPlanner (kP 5) | 1.8 / 4.6 | 1.9 / 4.0 | 3.3 / 3.4 | 0.1 | 3.5 | 4.54 | never | 3910 | 2.89 |
| PathPlanner (kP 5) + hold | 1.8 / 4.6 | 1.9 / 4.0 | 0.1 / 0.1 | 0.0 | 3.5 | 4.54 | 5.00 (10/10) | 3911 | 3.26 |
| PathPlanner (tuned) + hold | 1.0 / 3.2 | 1.0 / 2.8 | 0.8 / 1.5 | 0.1 | 1.5 | 4.61 | 4.83 (10/10) | 3969 | 4.55 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.7 / 4.6 | 1.7 / 3.9 | 0.1 / 0.1 | 0.0 | 3.4 | 4.71 | 5.15 (10/10) | 2893 | 2.92 |
| DriveToPose chain | 5.0 / 18.9 | 0.5 / 1.9 | 0.2 / 0.2 | 1.8 | 1.0 | 10.79 | never | 2377 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 6.7 / 15.9 | 1.7 / 3.3 | 1.7 / 1.8 | 2.4 | 2.2 | 4.43 | never | 2425 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 7.5 / 18.4 | 3.0 / 5.7 | 0.2 / 0.2 | 0.5 | 0.0 | 4.53 | 4.79 (10/10) | 1975 | 19.26 |
| Ours v2 (as shipped) | 2.9 / 12.1 | 2.1 / 5.6 | 0.1 / 0.1 | 0.2 | 0.1 | 4.76 | 4.90 (10/10) | 3413 | 1.60 |
| Ours v2 (as shipped, adaptive grip off) | 2.9 / 11.3 | 2.1 / 4.7 | 0.1 / 0.1 | 0.2 | 0.1 | 4.58 | 4.72 (10/10) | 3420 | 1.67 |
| Ours v2 (tuned) | 2.9 / 12.1 | 2.1 / 5.6 | 0.1 / 0.1 | 0.2 | 0.1 | 4.76 | 4.90 (10/10) | 3413 | 1.83 |
| PathPlanner path2 (new-path, pre-release), as shipped | 8.4 / 26.1 | 10.0 / 28.8 | 2.2 / 2.5 | 1.6 | 0.7 | 4.03 | never | 8024 | 0.07 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 5.7 / 18.5 | 4.2 / 13.4 | 1.7 / 1.8 | 0.0 | 0.0 | 5.01 | 5.33 (10/10) | 6268 | 0.07 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 3.7 / 14.3 | 2.4 / 7.9 | 0.1 / 0.2 | 0.3 | 0.2 | 5.05 | 5.19 (10/10) | 886 | 1.64 |

**fast_long**: an 8 m gentle curve with a 5 m/s cap (above the robot's 4.57 m/s free speed) (10.8 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 1.1 / 2.8 | 1.1 / 2.8 | 4.1 / 4.2 | 0.0 | 4.2 | 2.48 | never | 518 | 176.95 |
| BLine (README gains, r 0.30) | 1.7 / 4.1 | 0.9 / 3.8 | 44.3 / 59.2 | 0.7 | 125.2 | 2.20 | never | 1202 | 0.07 |
| BLine (README gains, r 0.45 default) | 2.3 / 4.6 | 0.5 / 2.1 | 49.2 / 59.9 | 0.7 | 125.4 | 2.20 | never | 1222 | 0.07 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 2.4 / 4.8 | 0.5 / 2.0 | 49.2 / 59.9 | 0.7 | 125.3 | 2.20 | never | 1221 | 0.07 |
| BLine (tuned) | 4.2 / 9.0 | 2.8 / 8.8 | 0.8 / 0.9 | 0.0 | 0.0 | 3.13 | 3.60 (10/10) | 238 | 0.07 |
| BLine (tuned, with corner caps) | 2.1 / 4.7 | 0.7 / 3.4 | 0.8 / 0.9 | 0.0 | 0.0 | 3.08 | 3.54 (10/10) | 290 | 0.07 |
| PathPlanner (kP 5) | 0.2 / 0.4 | 0.2 / 0.4 | 3.4 / 3.6 | 0.0 | 3.6 | 2.39 | never | 763 | 8.15 |
| PathPlanner (kP 5) + hold | 0.2 / 0.4 | 0.2 / 0.4 | 0.1 / 0.1 | 0.0 | 3.6 | 2.39 | 2.84 (10/10) | 764 | 5.75 |
| PathPlanner (tuned) + hold | 0.1 / 0.2 | 0.1 / 0.2 | 0.5 / 1.5 | 0.0 | 1.6 | 2.41 | 2.66 (10/10) | 810 | 6.46 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.2 / 0.4 | 0.2 / 0.4 | 0.1 / 0.1 | 0.0 | 3.6 | 2.39 | 2.84 (10/10) | 764 | 7.23 |
| DriveToPose chain | 1.9 / 4.4 | 0.1 / 0.2 | 0.1 / 0.1 | 0.0 | 0.0 | 4.92 | 5.11 (10/10) | 602 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 0.4 / 0.9 | 0.4 / 0.7 | 1.7 / 1.8 | 0.1 | 2.5 | 2.39 | 2.61 (10/10) | 818 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 6.8 / 13.7 | 7.0 / 13.7 | 0.1 / 0.1 | 0.3 | 0.0 | 2.47 | 2.72 (10/10) | 935 | 7.71 |
| Ours v2 (as shipped) | 0.6 / 1.2 | 0.5 / 1.0 | 0.0 / 0.1 | 0.0 | 0.1 | 2.52 | 2.66 (10/10) | 444 | 1.47 |
| Ours v2 (as shipped, adaptive grip off) | 0.6 / 1.2 | 0.5 / 1.0 | 0.0 / 0.1 | 0.0 | 0.1 | 2.42 | 2.56 (10/10) | 489 | 1.40 |
| Ours v2 (tuned) | 0.6 / 1.2 | 0.5 / 1.0 | 0.0 / 0.1 | 0.0 | 0.1 | 2.52 | 2.66 (10/10) | 444 | 1.46 |
| PathPlanner path2 (new-path, pre-release), as shipped | 0.9 / 4.5 | 0.4 / 6.2 | 8.0 / 8.2 | 0.2 | 0.0 | never | never | 2313 | 0.06 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 1.6 / 4.6 | 1.9 / 6.3 | 1.8 / 1.8 | 0.0 | 0.0 | 3.27 | 3.58 (10/10) | 1870 | 0.07 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 0.6 / 1.2 | 0.5 / 1.0 | 0.1 / 0.2 | 0.0 | 0.2 | 2.45 | 2.59 (10/10) | 440 | 1.47 |

**heavy_robot**: the S-curve with a 68 kg robot (MOI 6.8) and a 192 A current limit (20 % lower) (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 9.1 | 4.6 / 9.1 | 4.7 / 4.9 | 0.1 | 4.8 | 2.24 | never | 460 | 59.12 |
| BLine (README gains, r 0.30) | 13.3 / 37.7 | 14.5 / 37.7 | 5.3 / 5.8 | 0.1 | 91.1 | 3.01 | never | 1140 | 0.04 |
| BLine (README gains, r 0.45 default) | 8.4 / 26.4 | 9.3 / 26.5 | 4.2 / 4.9 | 0.0 | 86.3 | 2.94 | never | 994 | 0.05 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | 4.0 / 7.6 | 1.3 / 6.2 | 10.3 / 11.3 | 0.0 | 11.3 | 2.22 | never | 574 | 0.04 |
| BLine (tuned) | 7.0 / 16.4 | 4.8 / 15.6 | 0.8 / 0.8 | 0.0 | 0.0 | 2.70 | 3.17 (10/10) | 455 | 0.04 |
| BLine (tuned, with corner caps) | 2.5 / 5.8 | 3.0 / 9.2 | 0.8 / 0.8 | 0.0 | 0.0 | 2.77 | 3.23 (10/10) | 400 | 0.04 |
| PathPlanner (kP 5) | 0.6 / 1.9 | 0.5 / 1.8 | 3.8 / 4.1 | 0.0 | 4.1 | 1.88 | never | 1593 | 2.88 |
| PathPlanner (kP 5) + hold | 0.6 / 1.9 | 0.5 / 1.8 | 0.0 / 0.1 | 0.0 | 4.0 | 1.88 | 2.36 (10/10) | 1595 | 3.15 |
| PathPlanner (tuned) + hold | 0.4 / 1.2 | 0.3 / 1.1 | 0.9 / 1.7 | 0.0 | 1.9 | 1.90 | 2.13 (10/10) | 1614 | 2.72 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.7 / 1.9 | 0.6 / 1.8 | 0.0 / 0.1 | 0.0 | 4.0 | 1.91 | 2.39 (10/10) | 1354 | 2.92 |
| DriveToPose chain | 2.8 / 6.0 | 0.2 / 0.6 | 0.0 / 0.0 | 0.2 | 0.0 | 3.90 | 4.06 (10/10) | 892 | 0.01 |
| Choreo + ChoreoLib follower (as shipped) | 4.6 / 8.8 | 1.3 / 2.0 | 1.7 / 2.0 | 0.0 | 2.6 | 1.95 | 2.19 (10/10) | 1180 | 0.01 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 5.2 / 10.5 | 2.1 / 3.7 | 0.1 / 0.1 | 0.1 | 0.0 | 2.02 | 2.26 (10/10) | 1202 | 2.91 |
| Ours v2 (as shipped) | 1.3 / 3.3 | 1.1 / 2.3 | 0.2 / 0.3 | 0.0 | 0.3 | 1.91 | 2.05 (10/10) | 1514 | 1.11 |
| Ours v2 (as shipped, adaptive grip off) | 1.3 / 3.3 | 1.1 / 2.3 | 0.2 / 0.3 | 0.0 | 0.3 | 1.91 | 2.05 (10/10) | 1514 | 1.10 |
| Ours v2 (tuned) | 1.3 / 3.3 | 1.1 / 2.3 | 0.2 / 0.3 | 0.0 | 0.3 | 1.91 | 2.05 (10/10) | 1514 | 0.93 |
| PathPlanner path2 (new-path, pre-release), as shipped | 2.8 / 6.1 | 3.8 / 11.5 | 3.4 / 4.7 | 1.6 | 0.0 | 1.78 | never | 3585 | 0.04 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | 2.5 / 5.9 | 1.4 / 5.2 | 1.7 / 1.8 | 0.0 | 0.0 | 2.79 | 3.11 (10/10) | 1832 | 0.04 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 1.2 / 2.8 | 0.9 / 1.9 | 0.2 / 0.3 | 0.0 | 0.3 | 2.08 | 2.22 (10/10) | 628 | 1.08 |

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
| heavy_robot | the S-curve with a 68 kg robot (MOI 6.8) and a 192 A current limit (20 % lower) | a heavier, weaker robot: who plans with the right robot and still settles cleanly |

Course-specific numbers: **wall clearance** = the closest the TRUE bumper square (0.85 m) comes to either wall over the run (the plant has no walls, so negative = through it); **miss** = the closest the robot's centre comes to the TRUE piece; **at piece** = true speed there; **keeps going** = runs at least 1 m/s at the piece. On tag_align_moving_goal every time and error is against the TRUE goal. intake_drive_through ends moving, so it has no settle time.

### Perfect pose

| course | winner, time settled s (every run settled, final error within 5 cm) | ours v2 as shipped | ours v2 tuned | course-specific: best / ours v2 / ours tuned |
|---|---|---|---|---|
| spin_while_driving | Ours v2 (as shipped) 2.81 | 2.81 | 2.81 | - |
| figure_eight | Ours v2 (as shipped) 3.70 | 3.70 | 3.70 | - |
| wall_hug | Ours v2 (as shipped) 2.88 | 2.88 | 2.88 | wall clearance worst run cm: best Ours v2, equal-push traction rule (robot file tractionRule) 15.0; ours 15.0 / 15.0 |
| tag_align_moving_goal | Ours v2 (as shipped, adaptive grip off) 1.95 | 2.03 | 2.03 | - |
| intake_drive_through | n/a (ends moving) | - | - | miss cm (keeps going in every run): best PathPlanner (kP 5) + hold, wheel COF 0.9 0.1 at 2.12 m/s; ours 0.3 at 1.99 m/s / 0.3 at 1.99 m/s |
| three_piece_chain | Ours v2 (as shipped, adaptive grip off) 4.72 | 4.90 | 4.90 | - |
| fast_long | Ours v2 (as shipped, adaptive grip off) 2.56 | 2.66 | 2.66 | - |
| heavy_robot | Ours v2 (as shipped) 2.05 | 2.05 | 2.05 | - |

Every method on the course-specific numbers (mean; wall clearance and speed at the piece: worst run = lowest):

| method | wall clearance cm, mean / worst | miss at piece cm, mean / worst | at piece m/s, mean / lowest | keeps going |
|---|---|---|---|---|
| Old path-editor FollowPath | 14.2 / 14.2 | 10.0 / 10.0 | 0.69 / 0.67 | 0/10 |
| BLine (README gains, r 0.30) | -79.1 / -79.5 | 0.2 / 0.5 | 2.83 / 2.74 | 10/10 |
| BLine (README gains, r 0.45 default) | -75.6 / -76.1 | 0.2 / 0.5 | 2.82 / 2.73 | 10/10 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | -75.6 / -76.1 | 0.2 / 0.5 | 2.82 / 2.73 | 10/10 |
| BLine (tuned) | -35.9 / -36.2 | 0.3 / 0.4 | 2.02 / 1.98 | 10/10 |
| BLine (tuned, with corner caps) | -77.5 / -77.7 | 0.3 / 0.4 | 2.02 / 2.01 | 10/10 |
| PathPlanner (kP 5) | 5.3 / 5.0 | 0.1 / 0.2 | 2.12 / 2.11 | 10/10 |
| PathPlanner (kP 5) + hold | 5.3 / 5.0 | 0.1 / 0.2 | 2.12 / 2.11 | 10/10 |
| PathPlanner (tuned) + hold | 13.9 / 13.7 | 0.2 / 0.3 | 2.05 / 2.04 | 10/10 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 15.0 / 15.0 | 0.1 / 0.2 | 2.12 / 2.11 | 10/10 |
| DriveToPose chain | 14.9 / 14.9 | 0.1 / 0.1 | 0.00 / 0.00 | 0/10 |
| Choreo + ChoreoLib follower (as shipped) | 11.7 / 11.7 | 10.0 / 10.0 | 2.07 / 2.06 | 10/10 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 10.0 / 9.9 | 0.1 / 0.1 | 0.00 / 0.00 | 0/10 |
| Ours v2 (as shipped) | 15.0 / 15.0 | 0.3 / 0.4 | 1.99 / 1.42 | 10/10 |
| Ours v2 (as shipped, adaptive grip off) | 15.0 / 15.0 | 0.3 / 0.4 | 2.05 / 2.05 | 10/10 |
| Ours v2 (tuned) | 15.0 / 15.0 | 0.3 / 0.4 | 1.99 / 1.42 | 10/10 |
| PathPlanner path2 (new-path, pre-release), as shipped | -4.5 / -7.5 | 12.8 / 13.0 | 0.00 / 0.00 | 0/10 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | -7.1 / -8.4 | 10.1 / 10.2 | 0.00 / 0.00 | 0/10 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 15.0 / 15.0 | 0.4 / 0.4 | 2.05 / 2.05 | 10/10 |

### Wheel odometry + gyro

| course | winner, time settled s (every run settled, final error within 5 cm) | ours v2 as shipped | ours v2 tuned | course-specific: best / ours v2 / ours tuned |
|---|---|---|---|---|
| spin_while_driving | nobody settles every run | 2.86 (2/10) | 2.86 (2/10) | - |
| figure_eight | nobody settles every run | 3.69 (5/10) | 3.69 (5/10) | - |
| wall_hug | nobody settles every run | never | never | wall clearance worst run cm: best DriveToPose chain 4.4; ours 1.7 / 1.7 |
| tag_align_moving_goal | nobody settles every run | 2.03 (9/10) | 2.03 (9/10) | - |
| intake_drive_through | n/a (ends moving) | - | - | miss cm (keeps going in every run): best BLine (tuned, with corner caps) 0.2 at 2.01 m/s; ours 0.3 at 1.96 m/s / 0.3 at 1.96 m/s |
| three_piece_chain | nobody settles every run | never | never | - |
| fast_long | nobody settles every run | 3.05 (2/10) | 3.05 (2/10) | - |
| heavy_robot | nobody settles every run | 2.05 (4/10) | 2.05 (4/10) | - |

Every method on the course-specific numbers (mean; wall clearance and speed at the piece: worst run = lowest):

| method | wall clearance cm, mean / worst | miss at piece cm, mean / worst | at piece m/s, mean / lowest | keeps going |
|---|---|---|---|---|
| Old path-editor FollowPath | 11.6 / 3.2 | 10.0 / 10.5 | 0.72 / 0.00 | 3/10 |
| BLine (README gains, r 0.30) | -79.8 / -89.1 | 0.3 / 0.5 | 2.84 / 2.76 | 10/10 |
| BLine (README gains, r 0.45 default) | -76.3 / -85.0 | 0.3 / 0.6 | 2.82 / 2.64 | 10/10 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | -76.3 / -85.0 | 0.3 / 0.6 | 2.82 / 2.64 | 10/10 |
| BLine (tuned) | -36.6 / -45.0 | 0.2 / 0.4 | 2.02 / 2.00 | 10/10 |
| BLine (tuned, with corner caps) | -78.5 / -87.9 | 0.2 / 0.4 | 2.01 / 1.97 | 10/10 |
| PathPlanner (kP 5) | 5.7 / -3.9 | 0.3 / 0.5 | 2.11 / 1.96 | 10/10 |
| PathPlanner (kP 5) + hold | 5.7 / -3.9 | 0.3 / 0.5 | 2.11 / 1.96 | 10/10 |
| PathPlanner (tuned) + hold | 10.3 / 0.9 | 0.3 / 0.6 | 2.04 / 1.88 | 10/10 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 11.3 / 2.5 | 0.3 / 0.5 | 2.11 / 1.96 | 10/10 |
| DriveToPose chain | 12.5 / 4.4 | 0.9 / 4.2 | 0.11 / 0.00 | 0/10 |
| Choreo + ChoreoLib follower (as shipped) | 8.9 / 0.3 | 10.0 / 10.5 | 2.06 / 1.70 | 10/10 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.0 / -1.1 | 1.2 / 3.6 | 0.09 / 0.00 | 0/10 |
| Ours v2 (as shipped) | 10.9 / 1.7 | 0.3 / 0.5 | 1.96 / 1.42 | 10/10 |
| Ours v2 (as shipped, adaptive grip off) | 10.9 / 1.7 | 0.3 / 0.5 | 2.02 / 1.80 | 10/10 |
| Ours v2 (tuned) | 10.9 / 1.7 | 0.3 / 0.5 | 1.96 / 1.42 | 10/10 |
| PathPlanner path2 (new-path, pre-release), as shipped | 3.1 / -4.3 | 17.3 / 22.8 | 0.00 / 0.00 | 0/10 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | -8.8 / -18.2 | 10.6 / 14.0 | 0.05 / 0.00 | 0/10 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 12.1 / 3.6 | 0.4 / 0.7 | 2.04 / 1.95 | 10/10 |

### Slip-aware odometry + IMU

| course | winner, time settled s (every run settled, final error within 5 cm) | ours v2 as shipped | ours v2 tuned | course-specific: best / ours v2 / ours tuned |
|---|---|---|---|---|
| spin_while_driving | nobody settles every run | 2.86 (2/10) | 2.86 (2/10) | - |
| figure_eight | nobody settles every run | 3.69 (4/10) | 3.69 (4/10) | - |
| wall_hug | nobody settles every run | never | never | wall clearance worst run cm: best DriveToPose chain 4.4; ours 1.7 / 1.7 |
| tag_align_moving_goal | nobody settles every run | 2.03 (9/10) | 2.03 (9/10) | - |
| intake_drive_through | n/a (ends moving) | - | - | miss cm (keeps going in every run): best BLine (tuned, with corner caps) 0.2 at 2.01 m/s; ours 0.3 at 1.96 m/s / 0.3 at 1.96 m/s |
| three_piece_chain | nobody settles every run | never | never | - |
| fast_long | nobody settles every run | 3.05 (2/10) | 3.05 (2/10) | - |
| heavy_robot | nobody settles every run | 2.05 (4/10) | 2.05 (4/10) | - |

Every method on the course-specific numbers (mean; wall clearance and speed at the piece: worst run = lowest):

| method | wall clearance cm, mean / worst | miss at piece cm, mean / worst | at piece m/s, mean / lowest | keeps going |
|---|---|---|---|---|
| Old path-editor FollowPath | 11.6 / 3.2 | 10.0 / 10.5 | 0.72 / 0.00 | 3/10 |
| BLine (README gains, r 0.30) | -79.8 / -89.1 | 0.3 / 0.5 | 2.84 / 2.76 | 10/10 |
| BLine (README gains, r 0.45 default) | -76.3 / -85.0 | 0.3 / 0.6 | 2.82 / 2.64 | 10/10 |
| BLine (README gains, r 0.45 + BLine-Web corner caps) | -76.3 / -85.0 | 0.3 / 0.6 | 2.82 / 2.64 | 10/10 |
| BLine (tuned) | -36.6 / -45.0 | 0.2 / 0.4 | 2.02 / 2.00 | 10/10 |
| BLine (tuned, with corner caps) | -78.5 / -87.9 | 0.2 / 0.4 | 2.01 / 1.97 | 10/10 |
| PathPlanner (kP 5) | 5.1 / -4.5 | 0.3 / 0.5 | 2.11 / 1.96 | 10/10 |
| PathPlanner (kP 5) + hold | 5.1 / -4.5 | 0.3 / 0.5 | 2.11 / 1.96 | 10/10 |
| PathPlanner (tuned) + hold | 10.4 / 0.6 | 0.3 / 0.6 | 2.04 / 1.88 | 10/10 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 11.3 / 2.5 | 0.3 / 0.5 | 2.11 / 1.96 | 10/10 |
| DriveToPose chain | 12.5 / 4.4 | 0.9 / 4.2 | 0.11 / 0.00 | 0/10 |
| Choreo + ChoreoLib follower (as shipped) | 8.9 / 0.3 | 10.0 / 10.5 | 2.06 / 1.70 | 10/10 |
| Choreo geometry + DriveToPose finish (hybrid prototype) | 8.0 / -1.1 | 1.2 / 3.6 | 0.09 / 0.00 | 0/10 |
| Ours v2 (as shipped) | 10.9 / 1.7 | 0.3 / 0.5 | 1.96 / 1.42 | 10/10 |
| Ours v2 (as shipped, adaptive grip off) | 10.9 / 1.7 | 0.3 / 0.5 | 2.02 / 1.80 | 10/10 |
| Ours v2 (tuned) | 10.9 / 1.7 | 0.3 / 0.5 | 1.96 / 1.42 | 10/10 |
| PathPlanner path2 (new-path, pre-release), as shipped | -0.7 / -12.0 | 16.7 / 22.6 | 0.00 / 0.00 | 0/10 |
| PathPlanner path2 (tuned, 6 m/s^2 cap) | -8.7 / -18.2 | 10.6 / 14.0 | 0.05 / 0.00 | 0/10 |
| Ours v2, equal-push traction rule (robot file tractionRule) | 12.1 / 3.6 | 0.4 / 0.7 | 2.04 / 1.95 | 10/10 |

## Full traction

Ours on the original nine courses with the path's acceleration cap raised from 6 to 11.2 m/s^2, so its own traction rule decides (speed cap 4 m/s), against PathPlanner (tuned) + hold, Choreo + ChoreoLib follower (as shipped), Choreo geometry + DriveToPose finish (hybrid prototype) at the shared limits (the main grid's rows). Each cell: time settled s (runs settled), mean wheel slip ms, mean end error cm.

### Wheel odometry + gyro

| course | Ours v2 (as shipped) (own traction) | Ours v2, equal-push traction rule (robot file tractionRule) (own traction) | Ours v2 (as shipped) (shared limits) | PathPlanner (tuned) + hold (shared limits) | Choreo + ChoreoLib follower (as shipped) (shared limits) | Choreo geometry + DriveToPose finish (hybrid prototype) (shared limits) |
|---|---|---|---|---|---|---|
| straight_3m | 1.25 (3/10), 1479 ms, 2.8 cm | 1.79 (4/10), 609 ms, 2.3 cm | 1.43 (6/10), 368 ms, 2.5 cm | 1.45 (4/10), 570 ms, 2.9 cm | 1.48 (5/10), 667 ms, 3.0 cm | 1.58 (5/10), 531 ms, 2.4 cm |
| s_curve | 1.88 (1/10), 2680 ms, 4.5 cm | 2.56 (4/10), 1007 ms, 4.1 cm | 2.04 (3/10), 1623 ms, 4.2 cm | 2.15 (3/10), 1605 ms, 4.8 cm | 2.18 (4/10), 1187 ms, 4.5 cm | 2.27 (4/10), 1178 ms, 4.1 cm |
| hairpin | never, 4539 ms, 4.0 cm | 3.38 (2/10), 1627 ms, 2.7 cm | never, 3461 ms, 8.2 cm | never, 3170 ms, 5.7 cm | 3.07 (7/10), 1834 ms, 1.8 cm | never, 1347 ms, 2.8 cm |
| slalom | 3.14 (1/10), 3212 ms, 6.7 cm | 3.34 (3/10), 1628 ms, 5.5 cm | 3.07 (2/10), 2657 ms, 5.5 cm | 2.67 (4/10), 3437 ms, 5.7 cm | 3.00 (4/10), 1965 ms, 5.6 cm | 3.13 (2/10), 1694 ms, 5.8 cm |
| long_auto | 4.66 (1/10), 2528 ms, 9.3 cm | 4.41 (3/10), 1007 ms, 8.1 cm | 4.45 (2/10), 1808 ms, 8.5 cm | 3.80 (1/10), 2150 ms, 8.6 cm | 4.00 (1/10), 1810 ms, 8.0 cm | 4.09 (2/10), 1596 ms, 8.1 cm |
| moving_start | never, 2897 ms, 5.2 cm | 2.60 (4/10), 1750 ms, 3.6 cm | never, 2054 ms, 4.1 cm | never, 2480 ms, 39.0 cm | never, 1986 ms, 19.9 cm | 2.09 (3/10), 1908 ms, 3.6 cm |
| hit_mid_path | never, 4446 ms, 61.8 cm | never, 2945 ms, 41.4 cm | never, 3677 ms, 53.8 cm | never, 3709 ms, 53.0 cm | never, 3614 ms, 98.2 cm | never, 3495 ms, 94.8 cm |
| low_grip_s_curve | 2.81 (4/10), 1469 ms, 6.3 cm | 2.57 (4/10), 3693 ms, 4.2 cm | 3.10 (4/10), 306 ms, 4.1 cm | never, 4811 ms, 9.7 cm | 2.18 (4/10), 4558 ms, 4.5 cm | 2.26 (3/10), 4157 ms, 4.2 cm |
| grip_drop_s_curve | never, 4543 ms, 25.2 cm | 2.56 (3/10), 2416 ms, 4.7 cm | never, 3386 ms, 10.2 cm | never, 3379 ms, 12.3 cm | 2.18 (1/10), 2960 ms, 5.5 cm | 2.26 (4/10), 2624 ms, 4.3 cm |

### Slip-aware odometry + IMU

| course | Ours v2 (as shipped) (own traction) | Ours v2, equal-push traction rule (robot file tractionRule) (own traction) | Ours v2 (as shipped) (shared limits) | PathPlanner (tuned) + hold (shared limits) | Choreo + ChoreoLib follower (as shipped) (shared limits) | Choreo geometry + DriveToPose finish (hybrid prototype) (shared limits) |
|---|---|---|---|---|---|---|
| straight_3m | 1.25 (4/10), 1454 ms, 3.0 cm | 1.79 (4/10), 609 ms, 2.3 cm | 1.43 (6/10), 368 ms, 2.5 cm | 1.45 (4/10), 570 ms, 2.9 cm | 1.48 (5/10), 667 ms, 3.0 cm | 1.58 (5/10), 531 ms, 2.4 cm |
| s_curve | 1.84 (2/10), 2649 ms, 4.5 cm | 2.56 (4/10), 1007 ms, 4.1 cm | 2.04 (3/10), 1623 ms, 4.2 cm | 2.15 (3/10), 1605 ms, 4.8 cm | 2.18 (4/10), 1187 ms, 4.5 cm | 2.27 (4/10), 1178 ms, 4.1 cm |
| hairpin | never, 4488 ms, 7.4 cm | 3.38 (2/10), 1627 ms, 2.7 cm | never, 3467 ms, 8.5 cm | never, 3183 ms, 5.4 cm | 3.07 (7/10), 1834 ms, 1.8 cm | never, 1347 ms, 2.8 cm |
| slalom | 3.14 (1/10), 3222 ms, 5.8 cm | 3.34 (3/10), 1628 ms, 5.5 cm | 3.07 (2/10), 2657 ms, 5.5 cm | 2.71 (2/10), 3535 ms, 6.0 cm | 3.00 (4/10), 1965 ms, 5.6 cm | 3.13 (2/10), 1694 ms, 5.8 cm |
| long_auto | 4.66 (1/10), 2502 ms, 8.8 cm | 4.41 (3/10), 1007 ms, 8.1 cm | 4.45 (2/10), 1808 ms, 8.5 cm | 3.80 (1/10), 2149 ms, 8.7 cm | 4.00 (1/10), 1810 ms, 8.0 cm | 4.09 (2/10), 1596 ms, 8.1 cm |
| moving_start | 1.71 (1/10), 2896 ms, 5.7 cm | 2.60 (4/10), 1750 ms, 3.6 cm | 1.86 (1/10), 2072 ms, 4.2 cm | never, 2798 ms, 25.8 cm | never, 2359 ms, 13.0 cm | 2.09 (4/10), 1907 ms, 3.6 cm |
| hit_mid_path | never, 5174 ms, 17.6 cm | 2.70 (1/10), 3159 ms, 6.4 cm | never, 4945 ms, 8.4 cm | never, 5208 ms, 14.6 cm | never, 5408 ms, 28.0 cm | never, 5679 ms, 8.8 cm |
| low_grip_s_curve | 2.81 (4/10), 1111 ms, 5.6 cm | 2.57 (4/10), 3693 ms, 4.2 cm | 3.10 (4/10), 306 ms, 4.1 cm | never, 6101 ms, 9.4 cm | 2.18 (4/10), 4558 ms, 4.5 cm | 2.26 (3/10), 4139 ms, 4.2 cm |
| grip_drop_s_curve | never, 5606 ms, 13.3 cm | 2.56 (3/10), 2416 ms, 4.7 cm | 2.23 (2/10), 4234 ms, 7.4 cm | 2.36 (1/10), 4500 ms, 7.0 cm | 2.18 (1/10), 2960 ms, 5.5 cm | 2.26 (4/10), 2624 ms, 4.3 cm |

### Perfect pose

| course | Ours v2 (as shipped) (own traction) | Ours v2, equal-push traction rule (robot file tractionRule) (own traction) | Ours v2 (as shipped) (shared limits) | PathPlanner (tuned) + hold (shared limits) | Choreo + ChoreoLib follower (as shipped) (shared limits) | Choreo geometry + DriveToPose finish (hybrid prototype) (shared limits) |
|---|---|---|---|---|---|---|
| straight_3m | 1.28 (10/10), 1471 ms, 0.0 cm | 1.79 (10/10), 614 ms, 0.1 cm | 1.43 (10/10), 337 ms, 0.1 cm | 1.48 (10/10), 467 ms, 1.0 cm | 1.48 (10/10), 625 ms, 1.6 cm | 1.58 (10/10), 514 ms, 0.1 cm |
| s_curve | 1.89 (10/10), 2767 ms, 0.0 cm | 2.57 (10/10), 1036 ms, 0.0 cm | 2.05 (10/10), 1655 ms, 0.1 cm | 2.13 (10/10), 1620 ms, 0.8 cm | 2.17 (10/10), 1147 ms, 1.6 cm | 2.28 (10/10), 1213 ms, 0.1 cm |
| hairpin | 2.44 (10/10), 4674 ms, 0.0 cm | 3.37 (10/10), 1679 ms, 0.1 cm | 2.60 (10/10), 3479 ms, 0.1 cm | 2.71 (10/10), 3240 ms, 0.5 cm | 3.07 (10/10), 1767 ms, 1.5 cm | 3.19 (10/10), 1340 ms, 0.1 cm |
| slalom | 2.56 (10/10), 3466 ms, 0.1 cm | 3.36 (10/10), 1713 ms, 0.0 cm | 2.68 (10/10), 2831 ms, 0.1 cm | 2.69 (10/10), 3711 ms, 1.0 cm | 2.99 (10/10), 1824 ms, 1.6 cm | 3.10 (10/10), 1780 ms, 0.1 cm |
| long_auto | 3.83 (10/10), 2591 ms, 0.1 cm | 4.42 (10/10), 1042 ms, 0.1 cm | 3.91 (10/10), 1829 ms, 0.1 cm | 3.88 (10/10), 2050 ms, 0.6 cm | 4.02 (2/10), 1792 ms, 1.6 cm | 4.09 (10/10), 1649 ms, 0.2 cm |
| moving_start | 1.75 (10/10), 2956 ms, 0.0 cm | 2.63 (10/10), 1794 ms, 0.0 cm | 1.87 (10/10), 2094 ms, 0.1 cm | 1.67 (10/10), 3444 ms, 0.8 cm | 1.93 (10/10), 3028 ms, 1.6 cm | 2.10 (10/10), 1932 ms, 0.1 cm |
| hit_mid_path | 2.27 (10/10), 6095 ms, 0.1 cm | 2.72 (10/10), 3249 ms, 0.1 cm | 2.18 (10/10), 4926 ms, 0.2 cm | 2.43 (10/10), 5951 ms, 0.8 cm | never, 5396 ms, 38.2 cm | 2.97 (10/10), 5714 ms, 0.1 cm |
| low_grip_s_curve | 2.77 (10/10), 1381 ms, 0.2 cm | 2.61 (10/10), 3945 ms, 0.0 cm | 3.01 (10/10), 865 ms, 0.2 cm | 2.37 (10/10), 6730 ms, 0.6 cm | 2.18 (6/10), 4594 ms, 1.9 cm | 2.26 (10/10), 4326 ms, 0.1 cm |
| grip_drop_s_curve | 2.58 (10/10), 6472 ms, 0.1 cm | 2.60 (10/10), 2659 ms, 0.0 cm | 2.22 (10/10), 4376 ms, 0.1 cm | 2.37 (10/10), 4775 ms, 0.6 cm | 2.18 (6/10), 2968 ms, 1.9 cm | 2.24 (10/10), 2780 ms, 0.1 cm |

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
- Choreo `long_auto.traj`: 145 samples, 3.907 s
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
- Shove: 1043 N for 0.3 s at 50% of the path, to the left of travel
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.42, 0.00); (1.25, 0.00): (0.83, -0.13), (1.67, 0.13); (2.50, 0.80): (2.08, 0.53), (2.92, 1.07); (3.75, 1.60): (3.33, 1.47), (4.17, 1.73); (5.00, 1.60): (4.58, 1.60), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), R 0 deg @t 1, (5.00, 1.60) (handoff 0.30 m; 0.45 m in the r 0.45 rows; the tuned rows' radius is in the sweeps)
- BLine-Web corner caps at r 0.45, per target (m/s): 2, 2.8, 2.8, 2.8, 2.8
- Choreo `s_curve.traj`: 79 samples, 2.105 s
- PathPlanner path2 (new-path, pre-release): the same waypoints as nodes (a pose waypoint where a heading is set), joined by distance transitions; it drives the straight segments between them
- DriveToPose chain goals: (1.25, 0.00) @0 deg, (2.50, 0.80) @0 deg, (3.75, 1.60) @0 deg, (5.00, 1.60) @0 deg
- Neutral path length 5.50 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**low_grip_s_curve**

- Waypoints (old path-editor control points; PathPlanner anchors; Choreo waypoints): (0.00, 0.00) @0 deg, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60) @0 deg
- Grip: mu 0.91 static / 0.79 kinetic
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
- Choreo `extra/spin_while_driving.traj`: 97 samples, 2.100 s
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
- Choreo `extra/three_piece_chain.traj`: 219 samples, 4.586 s
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
| across | 24.3 | 1.99 / 3.65 | 39 | 5.54 / 9.9 | 13 / 82 |
| behind_wall | 41.1 | 0.78 / 2.38 | 61 | 1.59 / 3.2 | 8 / 20 |
| moving_start | 24.9 | 1.10 / 3.10 | 59 | 2.18 / 5.4 | 18 / 78 |
| turn_at_goal | 26.6 | 0.58 / 1.91 | 52 | 2.12 / 4.4 | 4 / 15 |

### Perfect pose

**across**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 9.36 | 3.08 | never | 3.1 / 3.2 | 6.5 | 0 | 3540 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 9.36 | 3.08 | 3.51 (10/10) | 0.1 / 0.1 | 6.5 | 0 | 3543 |
| Ours (visibility graph + A* + quintic, our follower) | 9.25 (corners 9.19) | 3.31 | 3.45 (10/10) | 0.1 / 0.2 | 4.5 | 0 | 649 |

**behind_wall**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 6.33 | 2.37 | never | 3.1 / 3.3 | 0.0 | 4 | 3583 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 6.33 | 2.37 | 2.81 (10/10) | 0.1 / 0.1 | 0.0 | 4 | 3584 |
| Ours (visibility graph + A* + quintic, our follower) | 6.25 (corners 6.19) | 2.83 | 2.97 (10/10) | 0.1 / 0.2 | 5.6 | 0 | 799 |

**moving_start**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 9.48 | 2.72 | never | 3.5 / 3.7 | 0.7 | 0 | 2607 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 9.48 | 2.72 | 3.17 (10/10) | 0.1 / 0.1 | 0.7 | 0 | 2613 |
| Ours (visibility graph + A* + quintic, our follower) | 9.58 (corners 9.41) | 3.07 | 3.21 (10/10) | 0.1 / 0.2 | 6.2 | 0 | 707 |

**turn_at_goal**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 4.97 | 1.88 | never | 3.2 / 3.4 | 16.1 | 0 | 1797 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 4.97 | 1.88 | 2.33 (10/10) | 0.1 / 0.1 | 16.1 | 0 | 1799 |
| Ours (visibility graph + A* + quintic, our follower) | 4.91 (corners 4.82) | 2.20 | 2.35 (10/10) | 0.1 / 0.2 | 17.2 | 0 | 131 |

### Wheel odometry + gyro

**across**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 9.36 | 3.12 (4/10) | 3.24 (3/10) | 7.1 / 17.3 | 0.0 | 8 | 3163 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 9.36 | 3.12 (4/10) | never | 8.2 / 16.9 | 0.0 | 8 | 3164 |
| Ours (visibility graph + A* + quintic, our follower) | 9.25 (corners 9.19) | 3.34 (4/10) | 3.44 (1/10) | 7.1 / 16.4 | 0.0 | 20 | 630 |

**behind_wall**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 6.33 | never | never | 8.0 / 10.9 | 0.0 | 4 | 3169 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 6.33 | never | never | 10.6 / 12.8 | 0.0 | 4 | 3169 |
| Ours (visibility graph + A* + quintic, our follower) | 6.25 (corners 6.19) | 2.87 (8/10) | 2.96 (3/10) | 3.8 / 8.8 | 0.0 | 16 | 771 |

**moving_start**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 9.48 | never | never | 10.2 / 20.3 | 3.8 | 0 | 2398 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 9.48 | never | never | 12.4 / 24.0 | 3.8 | 0 | 2401 |
| Ours (visibility graph + A* + quintic, our follower) | 9.58 (corners 9.41) | 3.10 (7/10) | 3.25 (1/10) | 6.8 / 17.1 | 5.1 | 0 | 677 |

**turn_at_goal**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 4.97 | 1.98 (4/10) | never | 6.2 / 10.6 | 15.7 | 0 | 1650 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 4.97 | 1.98 (4/10) | never | 7.6 / 10.2 | 15.7 | 0 | 1651 |
| Ours (visibility graph + A* + quintic, our follower) | 4.91 (corners 4.82) | 2.23 (8/10) | 2.34 (3/10) | 3.8 / 9.0 | 16.7 | 0 | 143 |

### Slip-aware odometry + IMU

**across**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 9.36 | 3.13 (6/10) | never | 7.8 / 20.3 | 0.9 | 0 | 3442 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 9.36 | 3.13 (6/10) | 3.51 (1/10) | 8.5 / 21.8 | 0.9 | 0 | 3446 |
| Ours (visibility graph + A* + quintic, our follower) | 9.25 (corners 9.19) | 3.34 (4/10) | 3.44 (1/10) | 7.1 / 16.4 | 0.0 | 20 | 630 |

**behind_wall**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 6.33 | 2.42 (7/10) | 2.55 (2/10) | 4.7 / 12.6 | 0.0 | 12 | 3415 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 6.33 | 2.42 (7/10) | never | 6.0 / 12.3 | 0.0 | 12 | 3415 |
| Ours (visibility graph + A* + quintic, our follower) | 6.25 (corners 6.19) | 2.87 (8/10) | 2.96 (3/10) | 3.8 / 8.8 | 0.0 | 16 | 771 |

**moving_start**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 9.48 | 2.74 (7/10) | 2.89 (3/10) | 6.4 / 16.1 | 0.0 | 16 | 2546 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 9.48 | 2.74 (7/10) | never | 6.9 / 18.8 | 0.0 | 16 | 2548 |
| Ours (visibility graph + A* + quintic, our follower) | 9.58 (corners 9.41) | 3.10 (7/10) | 3.25 (1/10) | 6.8 / 17.1 | 5.1 | 0 | 677 |

**turn_at_goal**

| method | path m | t to 5 cm s | t settled s (n/N) | final error cm mean / worst | min bumper clearance cm (worst run) | contact ms (worst run) | wheel slip ms |
|---|---|---|---|---|---|---|---|
| PathPlanner pathfinding (AD*, kP 5) | 4.97 | 1.92 (7/10) | 2.05 (3/10) | 4.4 / 10.1 | 15.7 | 0 | 1737 |
| PathPlanner pathfinding (AD*, kP 5) + hold | 4.97 | 1.92 (7/10) | never | 5.5 / 9.9 | 15.7 | 0 | 1738 |
| Ours (visibility graph + A* + quintic, our follower) | 4.91 (corners 4.82) | 2.23 (8/10) | 2.34 (3/10) | 3.8 / 9.0 | 16.7 | 0 | 143 |

### Ours: bumper margin

The margin is how far beyond the bumpers' half-diagonal the obstacles are grown, and what the swept check demands of the planned path. Contact runs = runs (of the seeds) where the TRUE bumper square touched an obstacle or wall.

| route | pose | margin m | path m | t to 5 cm s | t settled s (n/N) | min clearance cm (worst run) | contact runs | contact ms (worst run) |
|---|---|---|---|---|---|---|---|---|
| across | Perfect pose | 0.05 | 9.25 | 3.31 | 3.45 (10/10) | 4.5 | 0/10 | 0 |
| across | Perfect pose | 0.10 | 9.40 | 3.64 | 3.78 (10/10) | 10.4 | 0/10 | 0 |
| across | Perfect pose | 0.15 | 12.05 | 4.03 | 4.17 (10/10) | 15.8 | 0/10 | 0 |
| across | Wheel odometry + gyro | 0.05 | 9.25 | 3.34 (4/10) | 3.44 (1/10) | 0.0 | 1/10 | 20 |
| across | Wheel odometry + gyro | 0.10 | 9.40 | 3.67 (4/10) | 3.79 (1/10) | 2.6 | 0/10 | 0 |
| across | Wheel odometry + gyro | 0.15 | 12.05 | 4.06 (4/10) | 4.20 (1/10) | 3.8 | 0/10 | 0 |
| across | Slip-aware odometry + IMU | 0.05 | 9.25 | 3.34 (4/10) | 3.44 (1/10) | 0.0 | 1/10 | 20 |
| across | Slip-aware odometry + IMU | 0.10 | 9.40 | 3.67 (4/10) | 3.79 (1/10) | 2.6 | 0/10 | 0 |
| across | Slip-aware odometry + IMU | 0.15 | 12.05 | 4.06 (4/10) | 4.20 (1/10) | 3.8 | 0/10 | 0 |
| behind_wall | Perfect pose | 0.05 | 6.25 | 2.83 | 2.97 (10/10) | 5.6 | 0/10 | 0 |
| behind_wall | Perfect pose | 0.10 | 16.15 | 6.21 | 6.35 (10/10) | 10.6 | 0/10 | 0 |
| behind_wall | Perfect pose | 0.15 | 19.05 | 6.65 | 6.78 (10/10) | 15.8 | 0/10 | 0 |
| behind_wall | Wheel odometry + gyro | 0.05 | 6.25 | 2.87 (8/10) | 2.96 (3/10) | 0.0 | 1/10 | 16 |
| behind_wall | Wheel odometry + gyro | 0.10 | 16.15 | 6.21 (9/10) | 6.33 (4/10) | 0.0 | 1/10 | 40 |
| behind_wall | Wheel odometry + gyro | 0.15 | 19.05 | 6.65 (9/10) | 6.77 (4/10) | 0.0 | 1/10 | 8 |
| behind_wall | Slip-aware odometry + IMU | 0.05 | 6.25 | 2.87 (8/10) | 2.96 (3/10) | 0.0 | 1/10 | 16 |
| behind_wall | Slip-aware odometry + IMU | 0.10 | 16.15 | 6.21 (9/10) | 6.33 (4/10) | 0.0 | 1/10 | 40 |
| behind_wall | Slip-aware odometry + IMU | 0.15 | 19.05 | 6.65 (9/10) | 6.77 (4/10) | 0.0 | 1/10 | 8 |
| moving_start | Perfect pose | 0.05 | 9.58 | 3.07 | 3.21 (10/10) | 6.2 | 0/10 | 0 |
| moving_start | Perfect pose | 0.10 | 13.75 | 5.41 | 5.55 (10/10) | 10.6 | 0/10 | 0 |
| moving_start | Perfect pose | 0.15 | 16.54 | 5.83 | 5.96 (10/10) | 15.8 | 0/10 | 0 |
| moving_start | Wheel odometry + gyro | 0.05 | 9.58 | 3.10 (7/10) | 3.25 (1/10) | 5.1 | 0/10 | 0 |
| moving_start | Wheel odometry + gyro | 0.10 | 13.75 | 5.42 (4/10) | 5.54 (3/10) | 0.0 | 1/10 | 24 |
| moving_start | Wheel odometry + gyro | 0.15 | 16.54 | 5.83 (4/10) | 5.95 (3/10) | 0.0 | 1/10 | 4 |
| moving_start | Slip-aware odometry + IMU | 0.05 | 9.58 | 3.10 (7/10) | 3.25 (1/10) | 5.1 | 0/10 | 0 |
| moving_start | Slip-aware odometry + IMU | 0.10 | 13.75 | 5.42 (4/10) | 5.54 (3/10) | 0.0 | 1/10 | 24 |
| moving_start | Slip-aware odometry + IMU | 0.15 | 16.54 | 5.83 (4/10) | 5.95 (3/10) | 0.0 | 1/10 | 4 |
| turn_at_goal | Perfect pose | 0.05 | 4.91 | 2.20 | 2.35 (10/10) | 17.2 | 0/10 | 0 |
| turn_at_goal | Perfect pose | 0.10 | 4.97 | 2.20 | 2.34 (10/10) | 22.2 | 0/10 | 0 |
| turn_at_goal | Perfect pose | 0.15 | 5.02 | 2.20 | 2.34 (10/10) | 27.2 | 0/10 | 0 |
| turn_at_goal | Wheel odometry + gyro | 0.05 | 4.91 | 2.23 (8/10) | 2.34 (3/10) | 16.7 | 0/10 | 0 |
| turn_at_goal | Wheel odometry + gyro | 0.10 | 4.97 | 2.23 (8/10) | 2.34 (3/10) | 21.6 | 0/10 | 0 |
| turn_at_goal | Wheel odometry + gyro | 0.15 | 5.02 | 2.23 (8/10) | 2.34 (3/10) | 26.6 | 0/10 | 0 |
| turn_at_goal | Slip-aware odometry + IMU | 0.05 | 4.91 | 2.23 (8/10) | 2.34 (3/10) | 16.7 | 0/10 | 0 |
| turn_at_goal | Slip-aware odometry + IMU | 0.10 | 4.97 | 2.23 (8/10) | 2.34 (3/10) | 21.6 | 0/10 | 0 |
| turn_at_goal | Slip-aware odometry + IMU | 0.15 | 5.02 | 2.23 (8/10) | 2.34 (3/10) | 26.6 | 0/10 | 0 |

<!-- pathfind:end -->

## Path speed: PathPlanner and ours

**Optimised paths, each tool's own optimiser, curves only** (rows (c) and (d) below; perfect pose, time settled,
10 seeds). PathPlanner's optimiser is a genetic algorithm on the Bezier handles that never moves an anchor
(`pp_optimizer.py`, ported from the GUI); ours, as it now ships, changes only the tangents. Ours optimised settles
first on 11 of the 14 targets: s_curve 2.06 s against 2.33 s, hairpin 2.88 against 3.30, slalom 2.80 against 2.89,
long_auto 3.89 against 4.01, neutral_three_piece 4.91 against 4.96, depot_and_back 3.45 against 3.57.
PathPlanner's PLAN is still shorter on 13 of the 14 (s_curve 2.04 s against 2.06), and it reaches 5 cm first on 11;
its time-sampled finish then takes longer to settle. PathPlanner's own optimiser gains little:
0-6 % per target (depot_and_back most), against 0-6 % for ours curves-only.

**Where ours loses here:**
- around_hub_trench: 3.33 s against 3.32 s. Our optimiser can't clear the field wall (5 cm) by reshaping the curve
  between the trench waypoints, so it leaves the path unchanged and says where a waypoint would help.
  PathPlanner's optimiser has no wall check there: the route's anchors are within its robot radius of the wall,
  which turns its field check off.
- moving_start: 1.93 s against 1.89 s (PathPlanner starts its plan from the measured speed).
- hit_mid_path: 2.99 s against 2.80 s, and ours optimised is slower than ours as drawn (2.22 s). The faster curve
  is closer to the limit when the shove comes. On low grip and the grip drop ours optimised still settles first
  (2.92 s and 2.53 s, against 3.55 s and 3.07 s), but also slower than ours as drawn (2.46 s and 2.39 s).
- With wheel odometry, settling every seed is rare for everyone; see the odometry tables.

The extra rows (labelled "extra") move waypoints: ours with `--move-waypoints`, and our search on PathPlanner's
anchors and on path2's waypoints. PathPlanner ships no optimiser that moves anchors, and none for path2.

<!-- ppcompare:start -->
_Generated by `python bench/run.py --pp-compare` (10 seeds per cell, 0 s wall)._

Plan = each tool's own figure for the path as drawn, and they are not measured the same way. Ours (`Path.totalTimeSeconds`) and PathPlanner 2027's (the trajectory's total time) are plans: a time-parameterised profile the follower then tracks. path2 has no plan: its figure is the GUI's 50 Hz simulation of its own follower and setpoint generator on an ideal robot, a simulated run that ends at the follower's end tolerance (10 cm as shipped). path2 is PathPlanner's new-path branch (af86ef2b), a pre-release: no PathPlannerLib release ships it yet. To 5 cm / settled: on the bench plant, from the start (settled = 2 cm, 1 deg, under 5 cm/s, held 0.25 s). sim - plan = time to 5 cm minus the plan. Cross-track is against the authored reference (the neutral curve through the same waypoints; path2 drives the polyline between them, so its cross-track there is the cut of its corners).

### 2026 routes

| route | legs | what |
|---|---|---|
| start_to_shot | (-4.7, -2.2, 0 deg) -> (-5.3, -2.05) -> (-5.9, -1.4, 32 deg) | from the starting line back to a scoring pose, turning to face the hub (1.5 m) |
| depot_and_back | (-4.7, 2.2, 0 deg) -> (-6.2, 2.45) -> (-7.5, 2, 180 deg); (-7.5, 2, 180 deg) -> (-6.7, 1.55) -> (-5.8, 1.3, -31 deg) | starting line to the depot (stop, intake facing the wall), then back to a scoring pose |
| neutral_three_piece | (-4.7, -3.45, 0 deg) -> (-3.645, -3.45) -> (-2.4, -3.2) -> (-1.2, -2.2, 90 deg) at 2 m/s; (-1.2, -2.2, 90 deg) -> (-1.2, -0.8) -> (-1.2, 0.8) -> (-1.2, 2.2, 90 deg) at 2 m/s; (-1.2, 2.2, 90 deg) -> (-2.4, 3.2) -> (-3.645, 3.45) -> (-4.9, 3.3) -> (-5.7, 2.2, -47 deg) | through the lower trench to the neutral zone, a sweep along the fuel facing +y, back through the upper trench to a scoring pose: three paths with rolling handoffs at 2 m/s |
| around_hub_trench | (-6, -0.9, 0 deg) -> (-5.2, -2.7) -> (-4.25, -3.45) -> (-3.04, -3.45) -> (-2.4, -2.6) -> (-2.2, -1) -> (-2.2, 1.2, 90 deg) | from behind the hub round it through the lower trench to the hub's neutral side |
| shot_to_tower | (-5.7, 2.2, -47 deg) -> (-6.6, 1) -> (-7.15, -0.07, 180 deg) | from a scoring pose to the tower for a level-1 climb, turning to face the wall |

### The optimisers, rows (c) and (d)

Each tool's own optimiser, as it ships, scored by that tool's own figure. (c) ours: `tools/optimise.mjs` at its default budget (3 s = 1500 plans per path file), seed 1, curves only: the tangent direction and scale at every waypoint change; waypoints and headings never move. (d) PathPlanner 2027: `pp_optimizer.py`, a port of the GUI's optimiser (`lib/util/path_optimizer.dart`): a genetic algorithm, 50 paths for 100 generations (4,551 plans per path file), that turns each waypoint's handles by up to 5 deg and changes their lengths by up to 0.1 m, anchors fixed; seeded. The extra rows move waypoints: ours with `--move-waypoints` (within 0.5 m, headings kept), and our search (`pp_objective.py`, a port of `optimiser.ts`) on PathPlanner 2027's anchors and handles and on path2's waypoints. PathPlanner has no optimiser that moves anchors, and none for path2. The bench has no obstacles, so a moved waypoint can cut into the hub, bumps or trench arms. Planned time before -> after, summed over the target's path files:

| target | ours, curves only | PathPlanner 2027, its own GA | extra: ours, --move-waypoints | extra: PathPlanner 2027, our search | extra: path2, our search |
|---|---|---|---|---|---|
| start_to_shot | 1.00 -> 1.00 (-0%) | 1.03 -> 1.03 (-1%) | 1.00 -> 0.98 (-2%) | 1.03 -> 1.01 (-2%) | 0.84 -> 0.82 (-2%) |
| depot_and_back | 2.73 -> 2.73 (-0%) | 3.29 -> 2.97 (-10%) | 2.73 -> 2.70 (-1%) | 3.29 -> 2.90 (-12%) | 2.88 -> 2.88 (+0%) |
| neutral_three_piece | 4.69 -> 4.61 (-2%) | 4.63 -> 4.57 (-1%) | 4.69 -> 4.40 (-6%) | 4.63 -> 4.46 (-4%) | 4.10 -> 4.02 (-2%) |
| around_hub_trench | 3.04 -> 3.04 (+0%) | 3.02 -> 3.01 (-0%) | 3.04 -> 2.97 (-2%) | 3.02 -> 2.73 (-10%) | 2.82 -> 2.74 (-3%) |
| shot_to_tower | 1.43 -> 1.43 (+0%) | 1.49 -> 1.44 (-3%) | 1.43 -> 1.42 (-0%) | 1.49 -> 1.43 (-3%) | 1.36 -> 1.36 (+0%) |
| straight_3m | 1.42 -> 1.42 (+0%) | 1.42 -> 1.41 (-0%) | 1.42 -> 1.42 (+0%) | 1.42 -> 1.42 (+0%) | 1.20 -> 1.20 (+0%) |
| s_curve | 2.04 -> 2.04 (+0%) | 2.04 -> 2.04 (-0%) | 2.04 -> 1.98 (-3%) | 2.04 -> 1.98 (-3%) | 1.82 -> 1.78 (-2%) |
| hairpin | 2.60 -> 2.54 (-2%) | 2.49 -> 2.49 (+0%) | 2.60 -> 2.58 (-1%) | 2.49 -> 2.44 (-2%) | 2.30 -> 2.26 (-2%) |
| slalom | 2.58 -> 2.55 (-1%) | 2.56 -> 2.55 (-1%) | 2.58 -> 2.42 (-6%) | 2.56 -> 2.42 (-6%) | 2.34 -> 2.20 (-6%) |
| long_auto | 3.77 -> 3.75 (-1%) | 3.74 -> 3.72 (-0%) | 3.77 -> 3.52 (-7%) | 3.74 -> 3.49 (-7%) | 3.66 -> 3.38 (-8%) |
| moving_start | 1.85 -> 1.84 (-0%) | 1.85 -> 1.84 (-0%) | 1.85 -> 1.75 (-5%) | 1.85 -> 1.75 (-5%) | 1.64 -> 1.54 (-6%) |
| hit_mid_path | 2.04 -> 2.04 (+0%) | 2.04 -> 2.04 (-0%) | 2.04 -> 1.98 (-3%) | 2.04 -> 1.98 (-3%) | 1.82 -> 1.78 (-2%) |
| low_grip_s_curve | 2.04 -> 2.04 (+0%) | 2.04 -> 2.04 (-0%) | 2.04 -> 1.98 (-3%) | 2.04 -> 1.98 (-3%) | 1.82 -> 1.78 (-2%) |
| grip_drop_s_curve | 2.04 -> 2.04 (+0%) | 2.04 -> 2.04 (-0%) | 2.04 -> 1.98 (-3%) | 2.04 -> 1.98 (-3%) | 1.82 -> 1.78 (-2%) |

### Perfect pose

All targets (the 2026 routes and the existing courses):

| method | mean plan s | mean to 5 cm s | vs ours to 5 cm | targets reached every seed | mean end error cm | mean wheel slip ms |
|---|---|---|---|---|---|---|
| PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.41 | 2.27 | +1% | 14/14 | 0.1 | 2785 |
| PathPlanner 2027 (Bezier), tuned + hold | 2.83 | 2.67 | +18% | 14/14 | 0.5 | 1221 |
| PathPlanner path2 (new-path, pre-release), as shipped | 2.17 | 2.17 | -10% | 3/14 | 6.1 | 4480 |
| PathPlanner path2, tuned (6 m/s^2 cap) | 3.81 | 3.15 | +47% | 14/14 | 1.7 | 3099 |
| Ours v2, as shipped | 2.38 | 2.25 | +0% | 14/14 | 0.1 | 2603 |
| Ours v2 after its optimiser (curves only, as it ships) | 2.36 | 2.25 | -0% | 14/14 | 0.1 | 2711 |
| PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.37 | 2.28 | +1% | 14/14 | 0.3 | 3070 |
| Ours v2 after its optimiser with --move-waypoints | 2.29 | 2.19 | -3% | 14/14 | 0.1 | 1639 |
| PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 2.29 | 2.15 | -5% | 14/14 | 0.4 | 1939 |
| PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 2.11 | 2.46 | -10% | 3/14 | 7.1 | 3625 |

**start_to_shot**: from the starting line back to a scoring pose, turning to face the hub (1.5 m) (5.3 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.03 | 0.88 | 1.33 | -0.15 | 0.1 / 0.1 | 0.0 | 3.4 | 1.3 / 3.1 | 547 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 1.20 | 1.06 | 1.32 | -0.15 | 0.4 / 1.3 | 0.1 | 1.3 | 0.9 / 2.0 | 9 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 0.84 | 0.71 | never | -0.13 | 1.4 / 2.4 | 1.7 | 0.0 | 3.6 / 8.9 | 2392 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 2.34 | 1.71 | 2.02 | -0.63 | 1.7 / 1.8 | 0.0 | 0.0 | 0.9 / 2.7 | 723 |
| (b) | Ours v2, as shipped | 1.01 | 0.88 | 1.02 | -0.12 | 0.1 / 0.2 | 0.3 | 0.2 | 0.9 / 1.8 | 844 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 1.00 | 0.88 | 1.02 | -0.12 | 0.1 / 0.2 | 0.2 | 0.2 | 0.6 / 1.1 | 841 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 1.03 | 0.88 | 1.32 | -0.15 | 0.1 / 0.1 | 0.0 | 3.4 | 1.4 / 3.3 | 722 |
| extra | Ours v2 after its optimiser with --move-waypoints | 0.98 | 0.86 | 1.00 | -0.12 | 0.1 / 0.2 | 0.3 | 0.1 | 10.6 / 20.3 | 550 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.01 | 0.86 | 1.31 | -0.15 | 0.1 / 0.1 | 0.0 | 3.4 | 9.2 / 18.1 | 499 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 0.82 | never | never | - | 8.0 / 8.3 | 1.1 | 0.0 | 2.7 / 13.0 | 1567 |

**depot_and_back**: starting line to the depot (stop, intake facing the wall), then back to a scoring pose (8.0 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 3.29 | 3.46 | 3.87 | +0.17 | 0.1 / 0.1 | 0.0 | 2.7 | 1.5 / 3.2 | 254 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.70 | 3.64 | 3.87 | -0.06 | 0.7 / 1.1 | 0.0 | 1.4 | 0.6 / 1.4 | 28 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.88 | 2.54 | never | -0.34 | 2.1 / 2.1 | 1.6 | 0.0 | 3.4 / 9.0 | 3035 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 5.30 | 4.42 | 4.74 | -0.88 | 1.7 / 1.8 | 0.0 | 0.0 | 3.3 / 9.8 | 1190 |
| (b) | Ours v2, as shipped | 2.73 | 2.75 | 3.53 | +0.02 | 0.2 / 0.3 | 0.2 | 0.0 | 2.8 / 6.2 | 1014 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 2.73 | 2.75 | 3.53 | +0.02 | 0.2 / 0.3 | 0.2 | 0.1 | 4.0 / 9.1 | 1252 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.97 | 3.15 | 3.59 | +0.18 | 0.1 / 0.1 | 0.0 | 3.2 | 6.1 / 15.6 | 844 |
| extra | Ours v2 after its optimiser with --move-waypoints | 2.70 | 2.72 | 3.50 | +0.02 | 0.2 / 0.3 | 0.2 | 0.3 | 17.3 / 39.8 | 943 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 2.90 | 3.10 | 3.53 | +0.20 | 0.1 / 0.1 | 0.0 | 3.1 | 19.7 / 45.9 | 586 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 2.88 | 2.54 | never | -0.34 | 2.1 / 2.1 | 1.6 | 0.0 | 3.4 / 9.0 | 3035 |

**neutral_three_piece**: through the lower trench to the neutral zone, a sweep along the fuel facing +y, back through the upper trench to a scoring pose: three paths with rolling handoffs at 2 m/s (15.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 4.63 | 4.54 | 5.00 | -0.08 | 0.1 / 0.1 | 0.0 | 3.5 | 1.8 / 4.0 | 3911 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 5.51 | 5.43 | 5.68 | -0.07 | 0.4 / 1.0 | 0.0 | 1.1 | 0.9 / 3.0 | 1093 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 4.10 | 4.03 | never | -0.07 | 2.2 / 2.5 | 1.6 | 0.7 | 8.5 / 26.1 | 8024 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 5.60 | 5.01 | 5.33 | -0.59 | 1.7 / 1.8 | 0.0 | 0.0 | 6.1 / 21.0 | 6268 |
| (b) | Ours v2, as shipped | 4.69 | 4.58 | 4.72 | -0.10 | 0.1 / 0.1 | 0.2 | 0.1 | 2.5 / 8.8 | 3420 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 4.61 | 4.63 | 4.77 | +0.02 | 0.1 / 0.2 | 0.3 | 0.2 | 6.7 / 21.8 | 3884 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 4.57 | 4.48 | 4.93 | -0.09 | 0.8 / 0.8 | 0.0 | 2.9 | 7.1 / 23.2 | 5190 |
| extra | Ours v2 after its optimiser with --move-waypoints | 4.40 | 4.53 | 4.67 | +0.13 | 0.1 / 0.2 | 0.3 | 0.2 | 29.8 / 54.8 | 3606 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 4.46 | 4.37 | 4.82 | -0.09 | 0.1 / 0.1 | 0.0 | 3.5 | 20.1 / 50.5 | 3516 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 4.02 | 4.04 (9/10) | never | +0.02 | 3.3 / 5.9 | 1.6 | 0.0 | 11.3 / 32.7 | 7266 |

**around_hub_trench**: from behind the hub round it through the lower trench to the hub's neutral side (11.8 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 3.02 | 2.86 | 3.32 | -0.16 | 0.1 / 0.1 | 0.0 | 3.5 | 1.2 / 3.5 | 2736 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.60 | 3.45 | 3.66 | -0.15 | 0.6 / 1.2 | 0.0 | 1.2 | 0.8 / 1.7 | 586 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.82 | never | never | - | 7.9 / 8.3 | 0.0 | 0.0 | 1.5 / 10.0 | 4821 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 4.38 | 3.80 | 4.11 | -0.58 | 1.7 / 1.8 | 0.0 | 0.0 | 4.1 / 11.3 | 4214 |
| (b) | Ours v2, as shipped | 3.04 | 2.90 | 3.04 | -0.14 | 0.1 / 0.2 | 0.1 | 0.2 | 3.2 / 7.0 | 2682 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 3.04 | 2.90 | 3.04 | -0.14 | 0.1 / 0.2 | 0.1 | 0.2 | 3.2 / 7.0 | 2682 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 3.01 | 2.86 | 3.30 | -0.15 | 0.9 / 0.9 | 0.0 | 2.5 | 2.2 / 5.6 | 2979 |
| extra | Ours v2 after its optimiser with --move-waypoints | 2.97 | 2.83 | 2.97 | -0.14 | 0.1 / 0.1 | 0.1 | 0.1 | 8.7 / 24.3 | 2768 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 2.73 | 2.58 | 3.03 | -0.16 | 0.1 / 0.1 | 0.0 | 3.6 | 28.8 / 52.8 | 3337 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 2.74 | never | never | - | 7.9 / 8.2 | 0.0 | 0.0 | 3.5 / 17.0 | 4755 |

**shot_to_tower**: from a scoring pose to the tower for a level-1 climb, turning to face the wall (6.3 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.49 | 1.33 | 1.78 | -0.15 | 0.1 / 0.1 | 0.0 | 3.4 | 1.8 / 3.0 | 478 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 1.73 | 1.58 | 1.84 | -0.15 | 0.4 / 1.2 | 0.0 | 1.3 | 0.6 / 1.2 | 18 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.36 | never | never | - | 7.0 / 7.4 | 1.6 | 0.0 | 0.7 / 4.2 | 1507 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 2.82 | 2.19 | 2.51 | -0.63 | 1.8 / 1.8 | 0.0 | 0.0 | 3.3 / 11.5 | 1154 |
| (b) | Ours v2, as shipped | 1.43 | 1.32 | 1.46 | -0.11 | 0.1 / 0.2 | 0.2 | 0.2 | 2.0 / 3.5 | 533 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 1.43 | 1.32 | 1.46 | -0.11 | 0.1 / 0.2 | 0.2 | 0.2 | 1.4 / 3.3 | 538 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 1.44 | 1.29 | 1.74 | -0.16 | 0.1 / 0.1 | 0.0 | 3.4 | 4.7 / 10.5 | 702 |
| extra | Ours v2 after its optimiser with --move-waypoints | 1.42 | 1.31 | 1.45 | -0.11 | 0.1 / 0.1 | 0.2 | 0.1 | 4.4 / 7.7 | 556 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.43 | 1.28 | 1.73 | -0.15 | 0.1 / 0.1 | 0.0 | 3.5 | 7.3 / 14.9 | 487 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.36 | never | never | - | 7.0 / 7.4 | 1.6 | 0.0 | 0.7 / 4.2 | 1507 |

**straight_3m**: 3 m straight, heading held at 0 (6.5 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.42 | 1.26 | 1.71 | -0.16 | 0.1 / 0.1 | 0.0 | 3.6 | 0.0 / 0.1 | 428 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 1.57 | 1.42 | 1.67 | -0.15 | 0.4 / 1.1 | 0.0 | 1.3 | 0.0 / 0.0 | 2 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.20 | never | never | - | 8.0 / 8.2 | 0.0 | 0.0 | 0.0 / 0.1 | 1613 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 2.86 | 2.20 | 2.52 | -0.66 | 1.7 / 1.8 | 0.0 | 0.0 | 0.0 / 0.1 | 57 |
| (b) | Ours v2, as shipped | 1.42 | 1.30 | 1.43 | -0.12 | 0.1 / 0.2 | 0.0 | 0.2 | 0.0 / 0.1 | 337 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 1.42 | 1.30 | 1.43 | -0.12 | 0.1 / 0.2 | 0.0 | 0.2 | 0.0 / 0.1 | 337 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 1.41 | 1.27 | 1.71 | -0.15 | 0.9 / 0.9 | 0.0 | 2.5 | 0.3 / 0.6 | 445 |
| extra | Ours v2 after its optimiser with --move-waypoints | 1.42 | 1.30 | 1.43 | -0.12 | 0.1 / 0.2 | 0.0 | 0.2 | 0.0 / 0.1 | 337 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.42 | 1.26 | 1.71 | -0.16 | 0.1 / 0.1 | 0.0 | 3.6 | 0.0 / 0.1 | 428 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.20 | never | never | - | 8.0 / 8.2 | 0.0 | 0.0 | 0.0 / 0.1 | 1613 |

**s_curve**: S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | 1.88 | 2.34 | -0.16 | 0.1 / 0.1 | 0.0 | 3.6 | 0.6 / 1.8 | 1599 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | 2.21 | 2.46 | -0.15 | 0.4 / 1.2 | 0.0 | 1.3 | 0.5 / 1.1 | 471 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.82 | 1.79 (9/10) | never | -0.03 | 3.9 / 5.4 | 1.7 | 0.0 | 2.6 / 6.0 | 3642 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.44 | 2.80 | 3.11 | -0.64 | 1.7 / 1.8 | 0.0 | 0.0 | 2.5 / 5.9 | 1853 |
| (b) | Ours v2, as shipped | 2.04 | 1.91 | 2.05 | -0.13 | 0.1 / 0.2 | 0.0 | 0.2 | 1.3 / 3.3 | 1655 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 2.04 | 1.91 | 2.05 | -0.13 | 0.1 / 0.2 | 0.0 | 0.2 | 1.3 / 2.9 | 1677 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.04 | 1.88 | 2.34 | -0.16 | 0.1 / 0.1 | 0.0 | 3.7 | 0.9 / 1.9 | 1877 |
| extra | Ours v2 after its optimiser with --move-waypoints | 1.98 | 1.85 | 1.99 | -0.13 | 0.1 / 0.1 | 0.0 | 0.1 | 21.9 / 38.2 | 502 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.98 | 1.83 | 2.27 | -0.15 | 0.9 / 1.0 | 0.0 | 2.5 | 19.5 / 36.1 | 804 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.78 | never | never | - | 8.0 / 8.3 | 0.1 | 0.0 | 5.5 / 21.6 | 2432 |

**hairpin**: 2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held (9.9 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.49 | 2.35 | 2.79 | -0.14 | 0.1 / 0.1 | 0.0 | 3.6 | 13.2 / 31.2 | 4960 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.27 | 3.12 | 3.30 | -0.15 | 0.9 / 1.2 | 0.0 | 1.3 | 0.9 / 1.5 | 427 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.30 | never | never | - | 8.0 / 8.3 | 0.1 | 0.0 | 4.4 / 22.5 | 5191 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.90 | 3.39 | 3.71 | -0.51 | 1.7 / 1.8 | 0.0 | 0.0 | 8.5 / 23.9 | 5423 |
| (b) | Ours v2, as shipped | 2.60 | 2.46 | 2.60 | -0.13 | 0.1 / 0.1 | 0.1 | 0.1 | 1.3 / 2.7 | 3479 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 2.54 | 2.41 | 2.55 | -0.13 | 0.1 / 0.2 | 0.1 | 0.2 | 1.6 / 4.5 | 3338 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.49 | 2.35 | 2.79 | -0.14 | 0.1 / 0.1 | 0.0 | 3.6 | 13.2 / 31.2 | 4960 |
| extra | Ours v2 after its optimiser with --move-waypoints | 2.58 | 2.44 | 2.58 | -0.13 | 0.1 / 0.2 | 0.1 | 0.2 | 1.7 / 4.4 | 3479 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 2.44 | 2.30 | 2.74 | -0.14 | 0.9 / 0.9 | 0.0 | 2.6 | 8.7 / 21.4 | 4928 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 2.26 | never | never | - | 8.0 / 8.3 | 0.1 | 0.0 | 3.0 / 17.3 | 5416 |

**slalom**: through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m (10.3 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.56 | 2.41 | 2.86 | -0.16 | 0.1 / 0.1 | 0.0 | 3.7 | 1.7 / 6.0 | 3889 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.25 | 3.10 | 3.32 | -0.15 | 0.6 / 1.2 | 0.0 | 1.3 | 0.7 / 1.5 | 1230 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.34 | never | never | - | 6.7 / 8.1 | 1.6 | 0.0 | 5.7 / 29.4 | 5932 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.92 | 3.28 | 3.60 | -0.64 | 1.8 / 1.8 | 0.0 | 0.0 | 4.4 / 8.8 | 3273 |
| (b) | Ours v2, as shipped | 2.58 | 2.44 | 2.58 | -0.14 | 0.1 / 0.2 | 0.0 | 0.2 | 1.7 / 3.4 | 3148 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 2.55 | 2.41 | 2.55 | -0.14 | 0.1 / 0.2 | 0.0 | 0.2 | 2.0 / 6.1 | 3367 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.55 | 2.39 | 2.85 | -0.16 | 0.1 / 0.1 | 0.0 | 3.7 | 2.4 / 8.2 | 4307 |
| extra | Ours v2 after its optimiser with --move-waypoints | 2.42 | 2.29 | 2.43 | -0.13 | 0.1 / 0.2 | 0.0 | 0.2 | 23.9 / 41.0 | 343 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 2.42 | 2.27 | 2.71 | -0.15 | 0.9 / 1.0 | 0.0 | 2.5 | 21.4 / 46.1 | 491 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 2.20 | never | never | - | 7.9 / 8.3 | 0.0 | 0.0 | 10.2 / 41.1 | 1695 |

**long_auto**: auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end (14.1 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 3.74 | 3.58 | 4.03 | -0.16 | 0.1 / 0.1 | 0.0 | 3.5 | 1.6 / 3.2 | 2205 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 4.31 | 4.16 | 4.38 | -0.15 | 0.6 / 1.2 | 0.0 | 1.2 | 0.7 / 1.6 | 368 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 3.66 | never | never | - | 7.9 / 8.2 | 0.0 | 0.0 | 4.0 / 22.9 | 3496 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 5.18 | 4.60 | 4.91 | -0.58 | 1.7 / 1.8 | 0.0 | 0.0 | 5.8 / 13.3 | 3092 |
| (b) | Ours v2, as shipped | 3.77 | 3.63 | 3.77 | -0.14 | 0.1 / 0.1 | 0.2 | 0.1 | 4.4 / 12.5 | 2004 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 3.75 | 3.61 | 3.75 | -0.14 | 0.1 / 0.2 | 0.3 | 0.2 | 4.7 / 11.2 | 2307 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 3.72 | 3.56 | 4.02 | -0.16 | 0.1 / 0.1 | 0.0 | 3.6 | 3.4 / 7.6 | 2586 |
| extra | Ours v2 after its optimiser with --move-waypoints | 3.52 | 3.38 | 3.52 | -0.14 | 0.1 / 0.2 | 0.2 | 0.2 | 32.8 / 55.8 | 1805 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 3.49 | 3.34 | 3.79 | -0.16 | 0.1 / 0.1 | 0.0 | 3.7 | 29.2 / 52.0 | 1876 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 3.38 | never | never | - | 7.9 / 8.1 | 0.0 | 0.0 | 10.1 / 48.1 | 3488 |

**moving_start**: starts moving 2 m/s sideways (+y), path heads +x then curves left (7.9 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.85 | 1.44 | 1.90 | -0.41 | 0.1 / 0.1 | 0.0 | 3.7 | 12.9 / 25.3 | 3068 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.10 | 1.65 | 1.92 | -0.44 | 0.3 / 1.1 | 0.0 | 1.3 | 9.5 / 24.1 | 2694 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.64 | never | never | - | 7.9 / 8.4 | 0.1 | 0.0 | 5.8 / 30.7 | 3552 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.26 | 2.44 | 2.75 | -0.82 | 1.7 / 1.8 | 0.0 | 0.0 | 10.0 / 29.1 | 3169 |
| (b) | Ours v2, as shipped | 1.85 | 1.74 | 1.87 | -0.11 | 0.1 / 0.2 | 0.0 | 0.2 | 8.6 / 23.3 | 2094 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 1.84 | 1.72 | 1.86 | -0.12 | 0.1 / 0.2 | 0.0 | 0.2 | 9.1 / 23.3 | 2274 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 1.84 | 1.44 | 1.89 | -0.41 | 0.1 / 0.1 | 0.0 | 3.6 | 13.5 / 25.9 | 3180 |
| extra | Ours v2 after its optimiser with --move-waypoints | 1.75 | 1.55 | 1.68 | -0.20 | 0.1 / 0.1 | 0.0 | 0.1 | 36.1 / 59.3 | 1547 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.75 | 1.34 | 1.80 | -0.41 | 0.1 / 0.1 | 0.0 | 3.8 | 36.8 / 58.4 | 2025 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.54 | never | never | - | 7.9 / 8.2 | 0.1 | 0.0 | 15.4 / 53.7 | 2428 |

**hit_mid_path**: the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | 2.21 | 2.51 | +0.17 | 0.2 / 0.2 | 0.2 | 10.4 | 20.2 / 49.1 | 5000 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | 2.16 | 2.62 | -0.20 | 0.1 / 0.1 | 0.1 | 12.1 | 16.8 / 42.6 | 5006 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.82 | never | never | - | 7.9 / 8.2 | 0.2 | 0.0 | 10.1 / 50.3 | 5467 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.44 | 2.87 | 3.19 | -0.57 | 1.7 / 1.8 | 0.0 | 0.0 | 11.7 / 38.2 | 4113 |
| (b) | Ours v2, as shipped | 2.04 | 1.97 | 2.13 | -0.07 | 0.1 / 0.2 | 0.2 | 0.1 | 20.1 / 47.6 | 4814 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 2.04 | 1.98 | 2.15 | -0.06 | 0.1 / 0.2 | 0.2 | 0.1 | 21.2 / 49.4 | 4918 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.04 | 2.21 | 2.51 | +0.17 | 0.2 / 0.2 | 0.2 | 10.2 | 20.9 / 50.6 | 5074 |
| extra | Ours v2 after its optimiser with --move-waypoints | 1.98 | 1.88 | 2.02 | -0.09 | 0.1 / 0.2 | 0.3 | 0.2 | 16.7 / 38.1 | 2712 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.98 | 1.87 | 2.29 | -0.11 | 0.9 / 0.9 | 0.0 | 2.2 | 14.4 / 35.5 | 3456 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.78 | never | never | - | 8.0 / 8.3 | 0.4 | 0.0 | 5.8 / 21.6 | 4410 |

**low_grip_s_curve**: the S-curve on mu 0.91 static / 0.79 kinetic (61 % of normal grip) (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | 1.82 | 2.63 | -0.22 | 0.1 / 0.1 | 0.0 | 19.0 | 4.4 / 10.2 | 5852 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | 2.21 | 2.47 | -0.15 | 0.5 / 1.3 | 0.0 | 1.5 | 0.5 / 1.0 | 3315 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.82 | 1.76 (1/10) | never | -0.06 | 7.4 / 7.6 | 1.3 | 34.8 | 9.6 / 35.5 | 8387 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.44 | 2.74 | 3.05 | -0.70 | 1.7 / 1.8 | 0.0 | 0.0 | 3.9 / 9.9 | 5665 |
| (b) | Ours v2, as shipped | 2.04 | 1.83 | 2.28 | -0.21 | 0.1 / 0.1 | 0.3 | 9.6 | 3.6 / 8.2 | 6036 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 2.04 | 1.84 | 2.30 | -0.20 | 0.1 / 0.1 | 0.2 | 10.5 | 4.1 / 9.1 | 6100 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.04 | 2.35 | 2.66 | +0.31 | 0.1 / 0.1 | 0.0 | 21.6 | 8.7 / 21.9 | 5982 |
| extra | Ours v2 after its optimiser with --move-waypoints | 1.98 | 1.86 | 2.00 | -0.12 | 0.2 / 0.3 | 0.1 | 0.3 | 21.9 / 38.2 | 2343 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.98 | 1.82 | 2.31 | -0.16 | 0.9 / 1.0 | 0.0 | 3.4 | 19.5 / 36.1 | 2886 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.78 | 1.68 | never | -0.10 | 7.6 / 7.9 | 0.8 | 21.1 | 11.5 / 21.5 | 6570 |

**grip_drop_s_curve**: the S-curve; halfway along, the floor drops from mu 1.5 / 1.3 to 0.91 / 0.79 (a worn carpet seam) (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | 1.81 | 2.61 | -0.23 | 0.1 / 0.1 | 0.0 | 17.8 | 3.3 / 9.5 | 4066 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | 2.21 | 2.47 | -0.15 | 0.5 / 1.3 | 0.0 | 1.5 | 0.5 / 1.1 | 1851 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.82 | never | never | - | 7.2 / 7.4 | 1.3 | 37.3 | 10.4 / 38.3 | 5665 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.44 | 2.69 | 3.01 | -0.75 | 1.7 / 1.8 | 0.0 | 0.0 | 3.7 / 11.6 | 3192 |
| (b) | Ours v2, as shipped | 2.04 | 1.81 | 2.22 | -0.23 | 0.1 / 0.1 | 0.3 | 8.0 | 2.1 / 6.2 | 4376 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 2.04 | 1.81 | 2.24 | -0.23 | 0.1 / 0.2 | 0.3 | 8.9 | 2.4 / 6.9 | 4432 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.04 | 1.81 | 2.63 | -0.22 | 0.1 / 0.1 | 0.0 | 20.0 | 3.5 / 10.0 | 4131 |
| extra | Ours v2 after its optimiser with --move-waypoints | 1.98 | 1.84 | 1.99 | -0.14 | 0.2 / 0.3 | 0.1 | 0.3 | 22.0 / 38.2 | 1456 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.98 | 1.82 | 2.31 | -0.16 | 0.9 / 1.0 | 0.0 | 3.4 | 19.5 / 36.1 | 1829 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.78 | 1.58 | never | -0.20 | 7.6 / 8.0 | 0.8 | 22.1 | 12.2 / 21.6 | 4567 |

### Wheel odometry + gyro

All targets (the 2026 routes and the existing courses):

| method | mean plan s | mean to 5 cm s | vs ours to 5 cm | targets reached every seed | mean end error cm | mean wheel slip ms |
|---|---|---|---|---|---|---|
| PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.41 | 2.31 | +2% | 5/14 | 13.6 | 2410 |
| PathPlanner 2027 (Bezier), tuned + hold | 2.83 | 2.81 | +19% | 5/14 | 9.3 | 989 |
| PathPlanner path2 (new-path, pre-release), as shipped | 2.17 | 2.43 | -7% | 0/14 | 14.6 | 3969 |
| PathPlanner path2, tuned (6 m/s^2 cap) | 3.81 | 3.06 | +45% | 2/14 | 10.3 | 2773 |
| Ours v2, as shipped | 2.38 | 2.31 | +0% | 1/14 | 8.6 | 2323 |
| Ours v2 after its optimiser (curves only, as it ships) | 2.36 | 2.27 | -0% | 1/14 | 8.9 | 2420 |
| PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.37 | 2.11 | -0% | 4/14 | 14.8 | 2643 |
| Ours v2 after its optimiser with --move-waypoints | 2.29 | 2.21 | -3% | 2/14 | 6.3 | 1601 |
| PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 2.29 | 2.21 | -2% | 4/14 | 11.3 | 1761 |
| PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 2.11 | 2.75 | -10% | 1/14 | 15.6 | 3376 |

**start_to_shot**: from the starting line back to a scoring pose, turning to face the hub (1.5 m) (5.3 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.03 | 0.89 | 1.33 (8/10) | -0.15 | 1.3 / 2.9 | 0.2 | 5.3 | 1.3 / 3.1 | 540 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 1.20 | 1.06 | 1.33 (8/10) | -0.15 | 1.4 / 3.8 | 0.2 | 3.6 | 0.9 / 2.0 | 13 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 0.84 | 0.79 (3/10) | never | -0.05 | 5.9 / 8.5 | 0.8 | 0.0 | 3.2 / 4.7 | 2054 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 2.34 | 1.67 | 2.03 (7/10) | -0.67 | 1.7 / 3.8 | 0.2 | 1.5 | 0.8 / 2.7 | 702 |
| (b) | Ours v2, as shipped | 1.01 | 0.89 | 1.02 (8/10) | -0.12 | 1.5 / 3.1 | 0.4 | 2.3 | 1.1 / 2.4 | 861 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 1.00 | 0.89 | 1.03 (8/10) | -0.11 | 1.5 / 3.4 | 0.4 | 1.9 | 0.7 / 1.8 | 846 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 1.03 | 0.88 | 1.32 (8/10) | -0.15 | 1.3 / 2.8 | 0.2 | 5.4 | 1.4 / 3.4 | 708 |
| extra | Ours v2 after its optimiser with --move-waypoints | 0.98 | 0.87 | 1.00 (8/10) | -0.11 | 1.2 / 3.4 | 0.4 | 2.2 | 10.6 / 20.3 | 545 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.01 | 0.86 | 1.30 (8/10) | -0.15 | 1.2 / 2.9 | 0.2 | 5.7 | 9.2 / 18.1 | 501 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 0.82 | never | never | - | 16.6 / 18.4 | 1.3 | 0.0 | 4.7 / 14.3 | 1361 |

**depot_and_back**: starting line to the depot (stop, intake facing the wall), then back to a scoring pose (8.0 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 3.29 | 3.46 | 3.87 (7/10) | +0.17 | 1.5 / 3.5 | 0.2 | 4.3 | 1.9 / 8.3 | 266 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.70 | 3.65 | 3.85 (7/10) | -0.05 | 1.6 / 3.0 | 0.2 | 3.0 | 1.2 / 6.5 | 31 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.88 | 2.59 (8/10) | never | -0.29 | 5.1 / 8.1 | 2.7 | 4.6 | 5.1 / 11.6 | 2990 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 5.30 | 4.48 | 4.71 (1/10) | -0.82 | 2.9 / 4.7 | 1.7 | 0.7 | 3.6 / 13.6 | 1323 |
| (b) | Ours v2, as shipped | 2.73 | 2.76 (9/10) | 3.55 (7/10) | +0.03 | 1.2 / 5.4 | 1.9 | 2.9 | 3.1 / 8.0 | 998 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 2.73 | 2.76 (9/10) | 3.56 (8/10) | +0.03 | 1.2 / 5.3 | 1.8 | 2.9 | 4.1 / 10.6 | 1215 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.97 | 3.15 | 3.57 (8/10) | +0.17 | 1.4 / 3.2 | 0.2 | 4.9 | 6.4 / 17.4 | 820 |
| extra | Ours v2 after its optimiser with --move-waypoints | 2.70 | 2.73 | 3.54 (8/10) | +0.03 | 1.3 / 4.8 | 1.8 | 2.5 | 17.2 / 39.7 | 985 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 2.90 | 3.11 | 3.53 (7/10) | +0.21 | 1.4 / 3.4 | 0.2 | 3.9 | 19.7 / 45.9 | 570 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 2.88 | 2.59 (8/10) | never | -0.29 | 5.1 / 8.1 | 2.7 | 4.6 | 5.1 / 11.6 | 2990 |

**neutral_three_piece**: through the lower trench to the neutral zone, a sweep along the fuel facing +y, back through the upper trench to a scoring pose: three paths with rolling handoffs at 2 m/s (15.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 4.63 | 4.58 (6/10) | never | -0.05 | 6.6 / 12.0 | 0.3 | 13.0 | 5.5 / 15.1 | 3754 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 5.51 | 5.47 (7/10) | 5.65 (2/10) | -0.03 | 5.6 / 11.0 | 0.3 | 9.9 | 4.1 / 16.0 | 1040 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 4.10 | 3.95 (8/10) | never | -0.15 | 10.7 / 22.1 | 1.9 | 19.9 | 9.2 / 26.2 | 7162 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 5.60 | 5.18 (4/10) | never | -0.42 | 6.6 / 12.3 | 0.3 | 7.6 | 6.8 / 23.8 | 5862 |
| (b) | Ours v2, as shipped | 4.69 | 4.64 (4/10) | never | -0.05 | 6.2 / 11.0 | 0.3 | 9.3 | 5.0 / 19.3 | 3290 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 4.61 | 4.58 (7/10) | 4.77 (3/10) | -0.03 | 6.5 / 16.4 | 0.3 | 14.5 | 8.0 / 25.1 | 3722 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 4.57 | never | never | - | 24.8 / 31.8 | 0.3 | 20.0 | 22.2 / 33.2 | 4776 |
| extra | Ours v2 after its optimiser with --move-waypoints | 4.40 | 4.58 (3/10) | 4.68 (1/10) | +0.18 | 7.5 / 16.4 | 0.3 | 12.6 | 22.6 / 70.5 | 3535 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 4.46 | 4.52 (1/10) | never | +0.06 | 11.1 / 16.7 | 0.3 | 10.2 | 14.8 / 66.1 | 3397 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 4.02 | 3.99 (3/10) | never | -0.03 | 8.0 / 16.3 | 1.4 | 12.8 | 11.4 / 37.9 | 6867 |

**around_hub_trench**: from behind the hub round it through the lower trench to the hub's neutral side (11.8 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 3.02 | never | never | - | 9.8 / 13.7 | 0.4 | 0.0 | 4.3 / 9.7 | 2646 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.60 | 3.50 (8/10) | never | -0.10 | 4.6 / 10.9 | 0.4 | 2.2 | 2.6 / 9.7 | 603 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.82 | 2.88 (1/10) | never | +0.06 | 9.8 / 18.8 | 0.5 | 0.0 | 7.6 / 18.2 | 4790 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 4.38 | never | never | - | 10.3 / 13.3 | 0.4 | 0.0 | 4.4 / 14.2 | 4105 |
| (b) | Ours v2, as shipped | 3.04 | 3.00 (2/10) | never | -0.04 | 6.5 / 13.0 | 0.5 | 0.0 | 3.4 / 13.8 | 2624 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 3.04 | 3.00 (2/10) | never | -0.04 | 6.5 / 13.0 | 0.5 | 0.0 | 3.4 / 13.8 | 2624 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 3.01 | 2.96 (5/10) | never | -0.05 | 8.3 / 13.0 | 0.5 | 0.0 | 3.7 / 12.4 | 2872 |
| extra | Ours v2 after its optimiser with --move-waypoints | 2.97 | 2.93 (3/10) | never | -0.04 | 6.2 / 12.5 | 0.5 | 0.0 | 6.4 / 30.2 | 2749 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 2.73 | never | never | - | 11.7 / 13.5 | 0.5 | 0.0 | 14.8 / 55.4 | 3088 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 2.74 | never | never | - | 12.6 / 20.4 | 0.5 | 0.0 | 8.1 / 27.6 | 4775 |

**shot_to_tower**: from a scoring pose to the tower for a level-1 climb, turning to face the wall (6.3 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.49 | 1.33 | 1.77 (4/10) | -0.15 | 2.2 / 5.3 | 0.7 | 7.8 | 1.8 / 3.7 | 451 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 1.73 | 1.59 | 1.87 (4/10) | -0.15 | 2.5 / 6.1 | 0.7 | 6.0 | 0.8 / 2.1 | 20 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.36 | never | never | - | 13.4 / 16.6 | 2.0 | 0.0 | 4.8 / 6.2 | 1456 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 2.82 | 2.15 (9/10) | 2.52 (6/10) | -0.67 | 2.6 / 6.7 | 0.7 | 3.3 | 3.1 / 11.4 | 1120 |
| (b) | Ours v2, as shipped | 1.43 | 1.32 (9/10) | 1.46 (6/10) | -0.11 | 2.3 / 5.6 | 0.9 | 4.6 | 1.8 / 3.6 | 554 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 1.43 | 1.32 (9/10) | 1.46 (6/10) | -0.11 | 2.3 / 5.7 | 0.9 | 4.6 | 1.2 / 3.3 | 555 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 1.44 | 1.29 | 1.73 (4/10) | -0.15 | 2.4 / 5.7 | 0.7 | 8.4 | 4.7 / 11.1 | 679 |
| extra | Ours v2 after its optimiser with --move-waypoints | 1.42 | 1.31 (9/10) | 1.45 (7/10) | -0.11 | 2.3 / 5.7 | 0.9 | 4.4 | 4.3 / 8.0 | 560 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.43 | 1.28 | 1.73 (4/10) | -0.15 | 2.3 / 5.2 | 0.7 | 8.0 | 7.4 / 15.0 | 477 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.36 | never | never | - | 13.4 / 16.6 | 2.0 | 0.0 | 4.8 / 6.2 | 1456 |

**straight_3m**: 3 m straight, heading held at 0 (6.5 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.42 | 1.26 | 1.72 (4/10) | -0.16 | 2.3 / 5.8 | 0.1 | 8.8 | 0.1 / 0.3 | 471 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 1.57 | 1.42 | 1.71 (5/10) | -0.15 | 2.7 / 6.5 | 0.1 | 6.6 | 0.1 / 0.3 | 32 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.20 | never | never | - | 14.1 / 19.2 | 0.1 | 0.0 | 0.1 / 0.3 | 1590 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 2.86 | 2.18 (9/10) | 2.53 (4/10) | -0.68 | 2.7 / 7.7 | 0.1 | 3.5 | 0.1 / 0.3 | 59 |
| (b) | Ours v2, as shipped | 1.42 | 1.30 (9/10) | 1.43 (6/10) | -0.12 | 2.5 / 6.4 | 0.1 | 5.0 | 0.1 / 0.3 | 368 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 1.42 | 1.30 (9/10) | 1.43 (6/10) | -0.12 | 2.5 / 6.4 | 0.1 | 5.0 | 0.1 / 0.3 | 368 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 1.41 | 1.27 | 1.71 (7/10) | -0.15 | 2.5 / 6.8 | 0.1 | 7.7 | 0.3 / 0.7 | 501 |
| extra | Ours v2 after its optimiser with --move-waypoints | 1.42 | 1.30 (9/10) | 1.43 (6/10) | -0.12 | 2.5 / 6.4 | 0.1 | 5.0 | 0.1 / 0.3 | 368 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.42 | 1.26 | 1.72 (4/10) | -0.16 | 2.3 / 5.8 | 0.1 | 8.8 | 0.1 / 0.3 | 471 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.20 | never | never | - | 14.1 / 19.2 | 0.1 | 0.0 | 0.1 / 0.3 | 1590 |

**s_curve**: S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | 1.87 (9/10) | 2.34 (4/10) | -0.17 | 4.2 / 10.2 | 0.1 | 12.5 | 1.5 / 4.6 | 1565 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | 2.20 (9/10) | 2.44 (4/10) | -0.16 | 4.1 / 10.6 | 0.1 | 10.2 | 1.3 / 4.5 | 483 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.82 | 1.79 (2/10) | 1.86 (1/10) | -0.03 | 9.1 / 17.7 | 1.1 | 0.0 | 2.6 / 6.4 | 3287 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.44 | 2.62 (9/10) | 3.11 (4/10) | -0.82 | 4.3 / 10.7 | 0.1 | 10.2 | 3.6 / 9.8 | 1694 |
| (b) | Ours v2, as shipped | 2.04 | 1.89 (8/10) | 2.04 (3/10) | -0.15 | 4.2 / 10.7 | 0.0 | 9.0 | 2.1 / 7.8 | 1623 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 2.04 | 1.90 (8/10) | 2.04 (3/10) | -0.15 | 4.4 / 10.9 | 0.0 | 8.9 | 2.3 / 8.0 | 1623 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.04 | 1.86 (9/10) | 2.33 (4/10) | -0.18 | 4.5 / 10.6 | 0.1 | 13.6 | 2.1 / 6.4 | 1764 |
| extra | Ours v2 after its optimiser with --move-waypoints | 1.98 | 1.84 (8/10) | 1.99 (3/10) | -0.14 | 4.1 / 10.8 | 0.1 | 8.5 | 19.7 / 38.5 | 502 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.98 | 1.83 (9/10) | 2.27 (3/10) | -0.15 | 4.2 / 11.4 | 0.1 | 11.1 | 18.5 / 36.5 | 796 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.78 | never | never | - | 13.7 / 23.2 | 0.1 | 0.0 | 6.6 / 24.5 | 2391 |

**hairpin**: 2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held (9.9 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.49 | never | never | - | 36.5 / 41.2 | 0.1 | 0.0 | 20.0 / 26.9 | 3968 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.27 | 3.17 | 3.28 (4/10) | -0.11 | 2.6 / 4.2 | 0.1 | 0.0 | 1.6 / 7.0 | 472 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.30 | never | never | - | 13.7 / 16.8 | 0.1 | 0.0 | 8.9 / 19.8 | 5107 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.90 | never | never | - | 19.1 / 22.2 | 0.1 | 0.0 | 11.4 / 29.9 | 5047 |
| (b) | Ours v2, as shipped | 2.60 | never | never | - | 8.2 / 9.9 | 0.1 | 0.0 | 3.1 / 7.2 | 3461 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 2.54 | never | never | - | 9.6 / 11.1 | 0.2 | 0.0 | 3.4 / 6.8 | 3298 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.49 | never | never | - | 36.5 / 41.2 | 0.1 | 0.0 | 20.0 / 26.9 | 3968 |
| extra | Ours v2 after its optimiser with --move-waypoints | 2.58 | never | never | - | 8.4 / 9.7 | 0.1 | 0.0 | 3.2 / 6.9 | 3445 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 2.44 | never | never | - | 36.3 / 42.2 | 0.1 | 0.0 | 19.0 / 26.5 | 4096 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 2.26 | never | never | - | 16.0 / 18.8 | 0.0 | 0.0 | 9.8 / 23.6 | 5419 |

**slalom**: through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m (10.3 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.56 | 2.39 (9/10) | never | -0.17 | 6.9 / 14.6 | 0.1 | 17.7 | 3.2 / 9.6 | 3520 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 3.25 | 3.08 (8/10) | 3.36 (4/10) | -0.17 | 5.7 / 13.7 | 0.1 | 13.4 | 1.5 / 4.9 | 1226 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 2.34 | 2.29 (1/10) | 2.42 (1/10) | -0.05 | 10.6 / 22.9 | 0.2 | 0.4 | 4.9 / 22.0 | 5085 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.92 | 3.04 (9/10) | 3.59 (3/10) | -0.88 | 5.9 / 14.7 | 0.1 | 14.3 | 5.3 / 11.9 | 2918 |
| (b) | Ours v2, as shipped | 2.58 | 2.43 (8/10) | 2.57 (2/10) | -0.15 | 5.6 / 14.6 | 0.1 | 11.4 | 2.0 / 7.5 | 2944 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 2.55 | 2.40 (8/10) | 2.55 (3/10) | -0.15 | 5.7 / 13.8 | 0.1 | 11.7 | 2.1 / 7.2 | 3201 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.55 | 2.36 (9/10) | 2.85 (2/10) | -0.19 | 6.3 / 15.3 | 0.1 | 18.1 | 2.5 / 9.6 | 3858 |
| extra | Ours v2 after its optimiser with --move-waypoints | 2.42 | 2.27 (8/10) | 2.42 (3/10) | -0.14 | 5.6 / 14.5 | 0.1 | 11.8 | 21.5 / 41.5 | 363 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 2.42 | 2.27 (9/10) | 2.71 (2/10) | -0.15 | 5.6 / 14.9 | 0.1 | 14.2 | 20.3 / 46.3 | 499 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 2.20 | 2.16 (1/10) | never | -0.04 | 14.1 / 27.0 | 0.1 | 0.0 | 12.0 / 40.9 | 1660 |

**long_auto**: auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end (14.1 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 3.74 | 3.61 (4/10) | never | -0.12 | 9.3 / 21.1 | 0.9 | 4.3 | 7.3 / 20.9 | 2190 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 4.31 | 4.17 (4/10) | 4.45 (2/10) | -0.14 | 8.2 / 19.4 | 0.9 | 4.3 | 6.8 / 19.5 | 438 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 3.66 | 3.62 (2/10) | 3.72 (1/10) | -0.04 | 9.9 / 24.0 | 0.9 | 2.7 | 9.6 / 35.5 | 3586 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 5.18 | 4.58 (4/10) | 4.88 (1/10) | -0.60 | 8.9 / 20.9 | 0.9 | 3.4 | 9.0 / 25.6 | 3040 |
| (b) | Ours v2, as shipped | 3.77 | 3.65 (4/10) | 3.77 (1/10) | -0.12 | 8.5 / 20.1 | 0.9 | 2.0 | 8.3 / 21.5 | 1984 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 3.75 | 3.64 (4/10) | 3.73 (1/10) | -0.11 | 8.8 / 20.5 | 0.9 | 1.5 | 8.5 / 20.6 | 2298 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 3.72 | 3.60 (4/10) | 4.02 (1/10) | -0.12 | 9.4 / 21.5 | 0.9 | 4.9 | 8.0 / 21.5 | 2550 |
| extra | Ours v2 after its optimiser with --move-waypoints | 3.52 | 3.41 (4/10) | 3.51 (1/10) | -0.11 | 8.5 / 19.4 | 1.0 | 1.9 | 24.6 / 63.8 | 1823 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 3.49 | 3.36 (4/10) | 3.80 (1/10) | -0.13 | 9.0 / 20.6 | 0.9 | 5.1 | 22.4 / 59.0 | 1832 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 3.38 | 3.36 (2/10) | 3.48 (1/10) | -0.02 | 10.0 / 24.7 | 1.0 | 3.3 | 16.9 / 61.4 | 3418 |

**moving_start**: starts moving 2 m/s sideways (+y), path heads +x then curves left (7.9 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 1.85 | never | never | - | 34.1 / 37.9 | 0.1 | 0.0 | 33.4 / 38.7 | 2478 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.10 | never | never | - | 35.6 / 38.6 | 0.1 | 0.0 | 34.6 / 39.6 | 1923 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.64 | 1.56 (3/10) | never | -0.08 | 7.6 / 13.8 | 0.0 | 3.3 | 11.4 / 30.9 | 3386 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.26 | 2.20 (5/10) | never | -1.06 | 6.5 / 10.1 | 0.1 | 8.0 | 11.2 / 29.3 | 2893 |
| (b) | Ours v2, as shipped | 1.85 | 1.71 (8/10) | never | -0.13 | 4.1 / 9.9 | 0.1 | 6.2 | 8.9 / 23.3 | 2054 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 1.84 | 1.71 (8/10) | 1.84 (1/10) | -0.14 | 3.8 / 10.0 | 0.1 | 5.9 | 9.3 / 23.3 | 2234 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 1.84 | never | never | - | 30.9 / 34.9 | 0.1 | 11.4 | 30.5 / 37.1 | 2655 |
| extra | Ours v2 after its optimiser with --move-waypoints | 1.75 | 1.55 (8/10) | 1.68 (3/10) | -0.20 | 3.8 / 10.1 | 0.1 | 5.5 | 33.2 / 61.2 | 1534 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.75 | never | never | - | 20.1 / 25.8 | 0.1 | 6.5 | 26.6 / 72.0 | 1798 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.54 | never | never | - | 12.8 / 18.9 | 0.1 | 0.7 | 20.6 / 63.7 | 2268 |

**hit_mid_path**: the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | never | never | - | 52.9 / 56.7 | 0.1 | 0.0 | 44.7 / 50.5 | 3650 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | never | never | - | 46.8 / 50.1 | 0.1 | 0.0 | 39.2 / 44.8 | 2506 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.82 | never | never | - | 56.4 / 59.3 | 1.4 | 0.0 | 44.0 / 49.6 | 4767 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.44 | never | never | - | 48.6 / 58.2 | 0.1 | 0.0 | 42.1 / 56.5 | 3192 |
| (b) | Ours v2, as shipped | 2.04 | never | never | - | 51.6 / 58.0 | 0.3 | 0.0 | 43.9 / 49.7 | 3596 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 2.04 | never | never | - | 52.7 / 58.0 | 0.3 | 0.0 | 44.8 / 49.6 | 3669 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.04 | never | never | - | 54.7 / 58.6 | 0.1 | 0.0 | 46.1 / 52.2 | 3680 |
| extra | Ours v2 after its optimiser with --move-waypoints | 1.98 | never | never | - | 28.9 / 32.1 | 0.2 | 0.0 | 23.0 / 38.2 | 2217 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.98 | never | never | - | 44.6 / 46.8 | 0.1 | 0.0 | 33.8 / 41.0 | 2514 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.78 | never | never | - | 44.5 / 49.7 | 0.6 | 0.0 | 28.5 / 31.9 | 4080 |

**low_grip_s_curve**: the S-curve on mu 0.91 static / 0.79 kinetic (61 % of normal grip) (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | 1.83 | never | -0.21 | 9.8 / 18.7 | 0.1 | 21.9 | 4.8 / 9.8 | 4858 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | 2.21 (9/10) | 2.43 (4/10) | -0.15 | 4.2 / 10.5 | 0.1 | 10.0 | 1.2 / 4.4 | 3210 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.82 | never | never | - | 13.9 / 21.2 | 1.3 | 0.0 | 3.2 / 10.4 | 5863 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.44 | 2.47 (9/10) | never | -0.97 | 8.2 / 17.0 | 0.1 | 15.9 | 8.1 / 18.1 | 4310 |
| (b) | Ours v2, as shipped | 2.04 | 1.84 (8/10) | never | -0.20 | 8.0 / 14.2 | 0.3 | 14.2 | 5.9 / 12.5 | 4781 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 2.04 | 1.84 (4/10) | never | -0.20 | 8.9 / 14.0 | 0.3 | 13.8 | 7.3 / 14.4 | 4779 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.04 | 1.82 (9/10) | never | -0.22 | 10.8 / 18.9 | 0.1 | 22.2 | 7.0 / 13.7 | 4810 |
| extra | Ours v2 after its optimiser with --move-waypoints | 1.98 | 1.86 (8/10) | 1.99 (2/10) | -0.12 | 4.3 / 11.7 | 0.1 | 7.7 | 19.7 / 38.6 | 2359 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.98 | 1.83 (9/10) | 2.28 (3/10) | -0.15 | 4.2 / 11.1 | 0.1 | 11.2 | 18.5 / 36.5 | 2809 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.78 | never | never | - | 29.4 / 37.4 | 0.9 | 0.0 | 14.1 / 29.5 | 5135 |

**grip_drop_s_curve**: the S-curve; halfway along, the floor drops from mu 1.5 / 1.3 to 0.91 / 0.79 (a worn carpet seam) (8.6 s simulated)

| row | method | plan s | to 5 cm s | settled s | sim - plan s | end error cm mean / worst | end yaw worst deg | overshoot worst cm | cross-track RMS / max cm | wheel slip ms |
|---|---|---|---|---|---|---|---|---|---|---|
| (a) | PathPlanner 2027 (Bezier), kP 5 + hold, as shipped | 2.04 | 1.90 (1/10) | never | -0.14 | 12.9 / 22.4 | 0.1 | 23.2 | 11.4 / 25.3 | 3380 |
| (a) | PathPlanner 2027 (Bezier), tuned + hold | 2.36 | 2.19 (9/10) | 2.46 (3/10) | -0.17 | 4.7 / 12.1 | 0.1 | 11.7 | 1.2 / 4.5 | 1848 |
| (a) | PathPlanner path2 (new-path, pre-release), as shipped | 1.82 | never | never | - | 24.6 / 29.7 | 1.3 | 24.3 | 22.2 / 29.7 | 4448 |
| (a) | PathPlanner path2, tuned (6 m/s^2 cap) | 3.44 | never | never | - | 16.1 / 23.2 | 0.1 | 16.9 | 14.0 / 23.2 | 2564 |
| (b) | Ours v2, as shipped | 2.04 | never | never | - | 10.2 / 17.5 | 0.2 | 14.8 | 9.0 / 17.5 | 3389 |
| (c) | Ours v2 after its optimiser (curves only, as it ships) | 2.04 | 1.93 (2/10) | never | -0.11 | 10.0 / 17.4 | 0.3 | 15.0 | 8.4 / 17.4 | 3443 |
| (d) | PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.04 | 1.91 (1/10) | never | -0.13 | 13.9 / 23.5 | 0.1 | 23.9 | 12.3 / 26.4 | 3363 |
| extra | Ours v2 after its optimiser with --move-waypoints | 1.98 | 1.83 (9/10) | 1.99 (4/10) | -0.15 | 4.3 / 10.5 | 0.1 | 10.1 | 20.9 / 38.5 | 1435 |
| extra | PathPlanner 2027, anchors + handles moved by OUR search (not PathPlanner's), + hold | 1.98 | 1.81 (9/10) | 2.28 (4/10) | -0.17 | 4.5 / 11.1 | 0.1 | 13.6 | 18.6 / 36.5 | 1802 |
| extra | PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.78 | 1.63 | 1.80 (1/10) | -0.15 | 8.4 / 14.2 | 0.9 | 13.6 | 12.9 / 24.5 | 3851 |

<!-- ppcompare:end -->

## Planning grip: ours at mu 0.7 to 1.15

_Not re-run since the optimiser changed: the optimised rows below come from the optimiser that also moved
waypoints, and "PathPlanner optimised" is our search on its anchors and handles (now the extra row above), not
PathPlanner's own optimiser. Re-run with `python bench/run.py --mu-sweep` before relying on them._

<!-- musweep:start -->
_Generated by `python bench/run.py --mu-sweep` (10 seeds per cell, 0 s wall)._

Ours with the robot file's grip at mu 1.5, 1.25, 1, 0.7 (1.5 as shipped), as drawn and optimised (the optimiser re-run per mu, default budget, seed 1). The output limiter takes the same traction limit as the plan. The plant's grip is mu 1.5 static / 1.3 kinetic (less on the low-grip course, and from halfway on the grip-drop course). Means over all --pp-compare targets; PathPlanner's rows are from the --pp-compare run.

### Perfect pose: all targets

| method | to 5 cm s | settled s (targets settled every seed) | end error cm | worst overshoot cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 2.25 | 2.48 (14/14) | 0.1 | 9.6 | 2603 |
| Ours v2, planning mu 1.25 | 2.28 | 2.49 (14/14) | 0.1 | 4.7 | 2419 |
| Ours v2, planning mu 1 | 2.35 | 2.53 (14/14) | 0.1 | 0.4 | 2100 |
| Ours v2, planning mu 0.7 | 2.58 | 2.81 (14/14) | 0.1 | 27.9 | 998 |
| Ours v2 after its optimiser (curves only, as it ships) | 2.25 | 2.48 (14/14) | 0.1 | 10.5 | 2711 |
| Ours v2 optimised, planning mu 1.25 | 2.26 | 2.48 (14/14) | 0.1 | 8.2 | 2579 |
| Ours v2 optimised, planning mu 1 | 2.31 | 2.51 (14/14) | 0.1 | 3.5 | 2317 |
| Ours v2 optimised, planning mu 0.7 | 2.55 | 2.75 (14/14) | 0.1 | 34.8 | 1153 |
| PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.28 | 2.73 (14/14) | 0.3 | 21.6 | 3070 |
| PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 2.46 | nan (0/14) | 7.1 | 22.1 | 3625 |

### Wheel odometry + gyro: all targets

| method | to 5 cm s | settled s (targets settled every seed) | end error cm | worst overshoot cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 2.31 | 2.26 (0/14) | 8.6 | 14.8 | 2323 |
| Ours v2, planning mu 1.25 | 2.33 | 2.51 (0/14) | 8.0 | 13.0 | 2189 |
| Ours v2, planning mu 1 | 2.38 | 2.51 (0/14) | 7.3 | 11.0 | 2018 |
| Ours v2, planning mu 0.7 | 2.58 | 2.77 (0/14) | 6.6 | 13.3 | 934 |
| Ours v2 after its optimiser (curves only, as it ships) | 2.27 | 2.49 (0/14) | 8.9 | 15.0 | 2420 |
| Ours v2 optimised, planning mu 1.25 | 2.29 | 2.34 (0/14) | 8.4 | 14.2 | 2335 |
| Ours v2 optimised, planning mu 1 | 2.35 | 2.47 (0/14) | 8.4 | 11.7 | 2200 |
| Ours v2 optimised, planning mu 0.7 | 2.56 | 2.74 (0/14) | 10.0 | 34.5 | 1113 |
| PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.11 | 2.50 (0/14) | 14.8 | 23.9 | 2643 |
| PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 2.75 | 2.64 (0/14) | 15.6 | 13.6 | 3376 |

### Slip-aware odometry + IMU: the bench courses

| method | to 5 cm s | settled s (targets settled every seed) | end error cm | worst overshoot cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 2.28 | 2.60 (0/9) | 5.9 | 11.4 | 2387 |
| Ours v2, planning mu 1.25 | 2.31 | 2.61 (0/9) | 5.5 | 11.2 | 2237 |
| Ours v2, planning mu 1 | 2.27 | 2.42 (0/9) | 5.0 | 14.0 | 2322 |
| Ours v2, planning mu 0.7 | 2.45 | 2.61 (0/9) | 4.7 | 28.6 | 1332 |
| Ours v2 after its optimiser (curves only, as it ships) | 2.27 | 2.57 (0/9) | 6.1 | 11.7 | 2462 |
| Ours v2 optimised, planning mu 1.25 | 2.23 | 2.60 (0/9) | 5.9 | 11.5 | 2453 |
| Ours v2 optimised, planning mu 1 | 2.24 | 2.36 (0/9) | 5.4 | 11.7 | 2503 |
| Ours v2 optimised, planning mu 0.7 | 2.44 | 2.51 (0/9) | 4.8 | 34.5 | 1557 |
| PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.14 | 2.72 (0/9) | 8.2 | 27.1 | 3478 |
| PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 2.07 | 2.24 (0/9) | 16.6 | 21.3 | 3448 |

**low_grip_s_curve**, Perfect pose: the S-curve on mu 0.91 static / 0.79 kinetic (61 % of normal grip)

| method | to 5 cm s | settled s | end error cm mean / worst | overshoot worst cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 1.83 | 2.28 | 0.1 / 0.1 | 9.6 | 6036 |
| Ours v2, planning mu 1.25 | 1.86 | 2.19 | 0.1 / 0.2 | 4.7 | 5737 |
| Ours v2, planning mu 1 | 1.97 | 2.11 | 0.2 / 0.3 | 0.3 | 4623 |
| Ours v2, planning mu 0.7 | 2.12 | 2.27 | 0.2 / 0.3 | 0.4 | 3395 |
| Ours v2 after its optimiser (curves only, as it ships) | 1.84 | 2.30 | 0.1 / 0.1 | 10.5 | 6100 |
| Ours v2 optimised, planning mu 1.25 | 1.84 | 2.26 | 0.1 / 0.1 | 8.2 | 5997 |
| Ours v2 optimised, planning mu 1 | 1.90 | 2.06 | 0.2 / 0.3 | 0.4 | 5184 |
| Ours v2 optimised, planning mu 0.7 | 1.99 | 2.13 | 0.2 / 0.3 | 0.3 | 3723 |
| PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 2.35 | 2.66 | 0.1 / 0.1 | 21.6 | 5982 |
| PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.68 | never | 7.6 / 7.9 | 21.1 | 6570 |

**low_grip_s_curve**, Wheel odometry + gyro: the S-curve on mu 0.91 static / 0.79 kinetic (61 % of normal grip)

| method | to 5 cm s | settled s | end error cm mean / worst | overshoot worst cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 1.84 (8/10) | never | 8.0 / 14.2 | 14.2 | 4781 |
| Ours v2, planning mu 1.25 | 1.88 (8/10) | never | 6.6 / 12.3 | 12.3 | 4628 |
| Ours v2, planning mu 1 | 1.96 (8/10) | never | 4.7 / 11.6 | 8.3 | 4402 |
| Ours v2, planning mu 0.7 | 2.12 (8/10) | 2.25 (3/10) | 4.1 / 10.8 | 8.4 | 3352 |
| Ours v2 after its optimiser (curves only, as it ships) | 1.84 (4/10) | never | 8.9 / 14.0 | 13.8 | 4779 |
| Ours v2 optimised, planning mu 1.25 | 1.86 (6/10) | never | 8.4 / 13.6 | 13.4 | 4770 |
| Ours v2 optimised, planning mu 1 | 1.91 (8/10) | never | 6.3 / 10.4 | 9.9 | 4798 |
| Ours v2 optimised, planning mu 0.7 | 1.99 (8/10) | 2.13 (4/10) | 4.1 / 10.4 | 8.7 | 3673 |
| PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 1.82 (9/10) | never | 10.8 / 18.9 | 22.2 | 4810 |
| PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | never | never | 29.4 / 37.4 | 0.0 | 5135 |

**low_grip_s_curve**, Slip-aware odometry + IMU: the S-curve on mu 0.91 static / 0.79 kinetic (61 % of normal grip)

| method | to 5 cm s | settled s | end error cm mean / worst | overshoot worst cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 2.94 (8/10) | 3.10 (4/10) | 4.1 / 10.5 | 9.3 | 306 |
| Ours v2, planning mu 1.25 | 2.71 (8/10) | 2.89 (4/10) | 4.1 / 10.0 | 9.3 | 462 |
| Ours v2, planning mu 1 | 2.25 (6/10) | 2.74 (2/10) | 5.3 / 10.1 | 7.9 | 2816 |
| Ours v2, planning mu 0.7 | 2.12 (8/10) | 2.25 (3/10) | 4.1 / 10.8 | 8.4 | 3352 |
| Ours v2 after its optimiser (curves only, as it ships) | 2.94 (8/10) | 3.10 (4/10) | 4.1 / 10.5 | 9.2 | 313 |
| Ours v2 optimised, planning mu 1.25 | 2.70 (8/10) | 2.88 (4/10) | 4.1 / 10.1 | 9.2 | 557 |
| Ours v2 optimised, planning mu 1 | 2.33 (4/10) | 2.69 (2/10) | 7.3 / 11.5 | 10.7 | 3119 |
| Ours v2 optimised, planning mu 0.7 | 1.99 (8/10) | 2.13 (4/10) | 4.1 / 10.4 | 8.7 | 3673 |
| PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 1.91 (7/10) | never | 9.3 / 13.3 | 26.0 | 5858 |
| PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | never | never | 23.5 / 30.4 | 0.0 | 5389 |

**grip_drop_s_curve**, Perfect pose: the S-curve; halfway along, the floor drops from mu 1.5 / 1.3 to 0.91 / 0.79 (a worn carpet seam)

| method | to 5 cm s | settled s | end error cm mean / worst | overshoot worst cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 1.81 | 2.22 | 0.1 / 0.1 | 8.0 | 4376 |
| Ours v2, planning mu 1.25 | 1.84 | 2.13 | 0.1 / 0.2 | 3.1 | 4120 |
| Ours v2, planning mu 1 | 1.95 | 2.08 | 0.2 / 0.3 | 0.3 | 3202 |
| Ours v2, planning mu 0.7 | 2.11 | 2.25 | 0.2 / 0.3 | 0.3 | 1874 |
| Ours v2 after its optimiser (curves only, as it ships) | 1.81 | 2.24 | 0.1 / 0.2 | 8.9 | 4432 |
| Ours v2 optimised, planning mu 1.25 | 1.82 | 2.19 | 0.1 / 0.1 | 6.5 | 4407 |
| Ours v2 optimised, planning mu 1 | 1.89 | 2.05 | 0.2 / 0.3 | 0.3 | 3465 |
| Ours v2 optimised, planning mu 0.7 | 1.99 | 2.15 | 0.2 / 0.3 | 0.3 | 2204 |
| PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 1.81 | 2.63 | 0.1 / 0.1 | 20.0 | 4131 |
| PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.58 | never | 7.6 / 8.0 | 22.1 | 4567 |

**grip_drop_s_curve**, Wheel odometry + gyro: the S-curve; halfway along, the floor drops from mu 1.5 / 1.3 to 0.91 / 0.79 (a worn carpet seam)

| method | to 5 cm s | settled s | end error cm mean / worst | overshoot worst cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | never | never | 10.2 / 17.5 | 14.8 | 3389 |
| Ours v2, planning mu 1.25 | 1.94 (1/10) | never | 8.3 / 15.3 | 13.0 | 3299 |
| Ours v2, planning mu 1 | 1.95 (9/10) | 2.10 (3/10) | 4.8 / 11.1 | 10.3 | 3061 |
| Ours v2, planning mu 0.7 | 2.10 (9/10) | 2.25 (4/10) | 4.3 / 10.7 | 10.4 | 1822 |
| Ours v2 after its optimiser (curves only, as it ships) | 1.93 (2/10) | never | 10.0 / 17.4 | 15.0 | 3443 |
| Ours v2 optimised, planning mu 1.25 | 1.93 (2/10) | never | 9.0 / 16.3 | 14.2 | 3478 |
| Ours v2 optimised, planning mu 1 | 1.89 (9/10) | 2.03 (1/10) | 5.3 / 12.3 | 11.7 | 3295 |
| Ours v2 optimised, planning mu 0.7 | 1.98 (9/10) | 2.13 (4/10) | 4.4 / 10.9 | 10.5 | 2122 |
| PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 1.91 (1/10) | never | 13.9 / 23.5 | 23.9 | 3363 |
| PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.63 | 1.80 (1/10) | 8.4 / 14.2 | 13.6 | 3851 |

**grip_drop_s_curve**, Slip-aware odometry + IMU: the S-curve; halfway along, the floor drops from mu 1.5 / 1.3 to 0.91 / 0.79 (a worn carpet seam)

| method | to 5 cm s | settled s | end error cm mean / worst | overshoot worst cm | wheel slip ms |
|---|---|---|---|---|---|
| Ours v2, as shipped | 1.82 (8/10) | 2.23 (2/10) | 7.4 / 14.8 | 11.1 | 4234 |
| Ours v2, planning mu 1.25 | 1.89 (6/10) | never | 6.8 / 13.7 | 9.5 | 3718 |
| Ours v2, planning mu 1 | 1.95 (9/10) | 2.10 (3/10) | 4.8 / 11.1 | 10.3 | 3064 |
| Ours v2, planning mu 0.7 | 2.10 (9/10) | 2.25 (4/10) | 4.3 / 10.7 | 10.4 | 1822 |
| Ours v2 after its optimiser (curves only, as it ships) | 1.82 (8/10) | 2.26 (1/10) | 8.5 / 15.3 | 10.9 | 4294 |
| Ours v2 optimised, planning mu 1.25 | 1.86 (7/10) | never | 9.6 / 13.7 | 8.3 | 4172 |
| Ours v2 optimised, planning mu 1 | 1.89 (9/10) | 2.03 (1/10) | 5.3 / 12.3 | 11.7 | 3295 |
| Ours v2 optimised, planning mu 0.7 | 1.98 (9/10) | 2.13 (4/10) | 4.4 / 10.9 | 10.5 | 2122 |
| PathPlanner 2027 after PathPlanner's own optimiser (handles only), + hold | 1.81 (8/10) | 2.62 (3/10) | 6.0 / 14.2 | 27.1 | 4054 |
| PathPlanner path2, waypoints moved by OUR search (path2 has no optimiser) | 1.59 (9/10) | never | 8.8 / 14.1 | 21.3 | 4496 |

Planned time before -> after the optimiser, summed over the target's path files:

| target | mu 1.25 | mu 1 | mu 0.7 |
|---|---|---|---|
| start_to_shot | 1.01 -> 1.01 | 1.02 -> 1.01 | 1.08 -> 1.07 |
| depot_and_back | 2.74 -> 2.73 | 2.76 -> 2.76 | 2.90 -> 2.89 |
| neutral_three_piece | 4.74 -> 4.62 | 4.86 -> 4.64 | 5.17 -> 4.83 |
| around_hub_trench | 3.07 -> 3.07 | 3.17 -> 3.17 | 3.43 -> 3.43 |
| shot_to_tower | 1.43 -> 1.43 | 1.43 -> 1.43 | 1.49 -> 1.49 |
| straight_3m | 1.42 -> 1.42 | 1.42 -> 1.42 | 1.42 -> 1.42 |
| s_curve | 2.05 -> 2.04 | 2.09 -> 2.05 | 2.22 -> 2.09 |
| hairpin | 2.67 -> 2.61 | 2.80 -> 2.73 | 3.07 -> 2.98 |
| slalom | 2.63 -> 2.57 | 2.76 -> 2.65 | 3.04 -> 2.90 |
| long_auto | 3.80 -> 3.76 | 3.87 -> 3.78 | 4.09 -> 3.97 |
| moving_start | 1.85 -> 1.84 | 1.85 -> 1.85 | 1.91 -> 1.86 |
| hit_mid_path | 2.05 -> 2.04 | 2.09 -> 2.05 | 2.22 -> 2.09 |
| low_grip_s_curve | 2.05 -> 2.04 | 2.09 -> 2.05 | 2.22 -> 2.09 |
| grip_drop_s_curve | 2.05 -> 2.04 | 2.09 -> 2.05 | 2.22 -> 2.09 |

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
