# Path-following bench: results (phase 1 baseline)

How to run: `python bench/run.py --both` (about 3 minutes on 15 processes; results in `bench/out/results.json`,
gitignored). `--quick` is a 2-seed smoke test on 3 courses, `--perfect-pose` runs the true pose only,
`--md-only` rebuilds the generated part of this file from `results.json`. Tests: `python -m pytest -q bench/tests`.

What runs: every method drives the team's `alignsim` swerve plant (500 Hz stick-slip tyres, Kraken X60 FOC curve,
steering lag, 250 Hz odometry thread, 50 Hz loop with 1 ms jitter, 2 ms actuation delay) along 8 courses, 10 noise
seeds each. Each seed draws a wheel-radius error (1 % sigma) and a gyro scale error (0.2 % sigma) as in the
drive-to-pose race. Every method gets the same limits: 4.0 m/s, 6 m/s^2, 540 deg/s, 720 deg/s^2. Two pose sources:
wheel odometry + gyro (no vision; the main condition) and the perfect pose (the control law alone). All accuracy
numbers are against the TRUE plant pose. "Ours v2" is not built yet (slot in `bench/followers/ours_v2.py`).

## Summary

The numbers below are from the run recorded in the generated section (10 seeds per cell).

**With a perfect pose (the control laws alone):**
- **PathPlanner + a final hold is the strongest competitor.** As configured (kP 5, wheel COF 1.15 = the plant's
  static mu) it settles in every run on every course, for example 1.71 s on the 3 m straight, 2.34 s on the
  S-curve and 4.07 s on the 12 m route. It tracks the S-curve at 0.8 cm RMS. But its generator plans corners right
  at the friction limit, so it slides on every curve (1.9-7.0 s of summed wheel slip at normal grip). On the
  hairpin it runs wide
  (18.7 cm RMS, 40.8 cm worst). With COF 0.9 the hairpin drops to 0.8 cm RMS. The tuned set (kP 10 / 10, COF 0.7,
  the sweep's edge) holds the S-curve, hairpin, slalom and 12 m route to 0.4-0.8 cm RMS. It settles in 1.49-4.06 s.
- **Without the hold, PathPlanner never settles at the 2 cm bar.** The command ends when its clock runs out:
  3.1-3.4 cm past the goal on the smooth courses, 21 cm on the hairpin, 32 cm after the hit and 88 cm at low grip.
- **The old path-editor FollowPath tracks curves at 4.6-7.3 cm RMS.** The look-ahead cuts the inside of the
  hairpin by up to 13.3 cm. At normal grip it ends 3.0-3.5 cm past the goal and then idles (10.7 cm at low grip).
  It settled in 11 of 80 runs, 10 of them on the hit course. It doesn't close the final heading: 2.3 deg off at
  the end of the 12 m route.
- **BLine with the README gains overshoots every stop.** On the 3 m straight it ends 41.5 cm past the goal. On curves
  at 4 m/s it can't turn inside its 6 m/s^2 limit (no corner slow-downs without BLine-Web's velocity constraints, see
  caveats). It misses handoff circles and loops back: worst cross-track 38-99 cm on the S-curve, slalom, hairpin and
  12 m route. Tuned (kP 2, cross-track 2, 1 cm tolerance, 1.2 m handoff) it ends 0.8 cm off and settles in 75 of 80
  runs. It settles 0.4-1.3 s later than PathPlanner + hold on six courses (2.67 s vs 1.71 s on the straight,
  3.17 s vs 2.34 s on the S-curve). It is quicker after the hit (2.30 s vs 2.79 s) and at low grip (3.13 s vs
  3.35 s). It still runs 41 cm RMS / 97 cm worst on the hairpin.
- **The DriveToPose chain is the naive baseline.** It is exact at the end (0.1 cm on seven courses, settles in every
  run), but it stops at every waypoint: 7.95 s on the hairpin against 3.26 s for PathPlanner + hold.

**With wheel odometry (no vision), the odometry error sets the floor:**
- Even methods that never slip end about 2-8 cm off on average (worst up to 20 cm on the 12 m route), because of
  the 1 % wheel-radius error.
  So few runs meet the 2 cm settle bar. The best is the DriveToPose chain, which settled 30 of 80 runs.
- **Slip turns into odometry error.** PathPlanner as configured slides on the hairpin and ends 50-53 cm off (tuned:
  3.3 cm). On the moving start every PathPlanner variant ends 56-60 cm off. The trajectory carries the measured
  2 m/s sideways only in its first state and then steps to the along-path velocity. The wheels slide for about
  2.5 s (summed over the four wheels) and the odometry over-reads. With a perfect pose, PathPlanner + hold ends
  those runs 0.1 cm off.
- **The shove on the hit course is invisible to wheel odometry.** The wheels slide sideways, so odometry misses
  33-54 cm, and every method finishes 33-56 cm off. With a perfect pose the position-based followers recover:
  the old FollowPath ends 1.6 cm off, tuned BLine 0.8 cm, PathPlanner + hold 0.1 cm.
- **Low grip (mu 0.70 / 0.61):** PathPlanner keeps planning with COF 1.15 and ends 27-31 cm off. The old FollowPath
  ends 12 cm off (its limiter assumes MU 1.0). Tuned BLine ends 5.0 cm off, the chain 4.8 cm and tuned PathPlanner
  7.6 cm.

**Path preparation (Python wall-clock, relative only):** BLine needs under 0.1 ms and the chain about 0.01 ms.
PathPlanner's path and trajectory take 2-9 ms. The old path-editor's 1,000-samples-per-metre spline table and
velocity profile take about 50-420 ms per path.

**For ours v2, the bar is PathPlanner + hold (tuned) with a perfect pose:** sub-centimetre tracking on smooth
courses and settling 1.5-4.1 s. The gaps to beat are its slides on a moving start (17 cm RMS) and after a hit
(20 cm RMS), and with odometry its final error when it slips. Every method needs vision, or slip-aware odometry,
to get under the 2-8 cm odometry floor.

<!-- generated:start -->
_Generated by `python bench/run.py --both` (10 seeds, 193 s wall)._

## Tuning sweeps

Score: settled fraction first, then mean(settle time or sim time) + 10 s per metre of RMS cross-track.

**bline** sweep (150 variants, perfect pose, seeds [100, 101, 102], all courses). Chosen: `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 1.2}`. Best five:

| options | settled fraction | score s |
|---|---|---|
| `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 1.2}` | 1.00 | 5.06 |
| `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 0.9}` | 1.00 | 5.12 |
| `{'kp': 2.0, 'kp_ct': 4.0, 'tol': 0.01, 'radius': 0.9}` | 1.00 | 5.22 |
| `{'kp': 2.0, 'kp_ct': 2.0, 'tol': 0.01, 'radius': 0.6}` | 1.00 | 5.34 |
| `{'kp': 2.0, 'kp_ct': 4.0, 'tol': 0.01, 'radius': 0.6}` | 1.00 | 5.37 |

**pathplanner** sweep (24 variants, perfect pose, seeds [100, 101, 102], all courses). Chosen: `{'kp': 10.0, 'kp_rot': 10.0, 'hold': True, 'cof': 0.7}`. Best five:

| options | settled fraction | score s |
|---|---|---|
| `{'kp': 10.0, 'kp_rot': 10.0, 'hold': True, 'cof': 0.7}` | 1.00 | 3.41 |
| `{'kp': 10.0, 'kp_rot': 5.0, 'hold': True, 'cof': 0.7}` | 1.00 | 3.41 |
| `{'kp': 7.0, 'kp_rot': 10.0, 'hold': True, 'cof': 0.7}` | 1.00 | 3.41 |
| `{'kp': 7.0, 'kp_rot': 5.0, 'hold': True, 'cof': 0.7}` | 1.00 | 3.41 |
| `{'kp': 5.0, 'kp_rot': 10.0, 'hold': True, 'cof': 0.7}` | 1.00 | 3.44 |

## Per-course tables

Accuracy is always against the TRUE plant pose. Cross-track is measured from the start until the robot first reaches the end of the path. Times are means over the runs that got there; 'settled' = within 2 cm and 1 deg of the end pose, true speed under 5 cm/s, held 0.25 s. Wheel slip = milliseconds of slip summed over the four wheels. Path prep = median wall-clock of the method's own preparation in Python (relative only).

### Wheel odometry + gyro (no vision): the main condition

| method | runs settled | courses settled every seed | mean RMS cross-track cm | worst cross-track cm | mean final error cm | mean wheel slip ms |
|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 10/80 | 0/8 | 9.0 | 43.4 | 10.7 | 1334 |
| BLine (README gains, r 0.30) | 3/80 | 0/8 | 25.5 | 104.3 | 23.8 | 2246 |
| BLine (README gains, r 0.45 default) | 10/80 | 0/8 | 22.8 | 104.2 | 20.1 | 2001 |
| BLine (README gains, curve targets, r 0.45) | 10/80 | 0/8 | 23.3 | 104.2 | 20.6 | 2024 |
| BLine (tuned) | 19/80 | 0/8 | 20.1 | 101.8 | 9.9 | 952 |
| PathPlanner (kP 5) | 3/80 | 0/8 | 15.4 | 60.7 | 27.0 | 3158 |
| PathPlanner (kP 5) + hold | 10/80 | 0/8 | 15.4 | 60.7 | 26.6 | 3173 |
| PathPlanner (tuned) + hold | 12/80 | 0/8 | 12.8 | 63.8 | 17.6 | 2415 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 13/80 | 0/8 | 13.1 | 60.7 | 18.7 | 2910 |
| DriveToPose chain | 30/80 | 0/8 | 7.1 | 58.7 | 7.9 | 1791 |

**straight_3m**: 3 m straight, heading held at 0 (6.5 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.1 / 0.3 | 0.1 / 0.3 | 4.1 / 8.7 | 0.1 | 8.7 | 1.36 | 1.53 (1/10) | 89 | 150.84 |
| BLine (README gains, r 0.30) | 0.1 / 0.3 | 0.1 / 0.3 | 38.2 / 47.4 | 0.1 | 69.4 | 1.13 | 2.27 (1/10) | 954 | 0.03 |
| BLine (README gains, r 0.45 default) | 0.1 / 0.3 | 0.1 / 0.3 | 38.2 / 47.4 | 0.1 | 69.4 | 1.13 | 2.27 (1/10) | 954 | 0.03 |
| BLine (README gains, curve targets, r 0.45) | 0.1 / 0.3 | 0.1 / 0.3 | 38.2 / 47.4 | 0.1 | 69.4 | 1.13 | 2.27 (1/10) | 954 | 0.03 |
| BLine (tuned) | 0.1 / 0.3 | 0.1 / 0.3 | 2.5 / 6.8 | 0.1 | 4.5 | 2.18 (9/10) | 2.67 (7/10) | 0 | 0.03 |
| PathPlanner (kP 5) | 0.1 / 0.3 | 0.1 / 0.3 | 3.9 / 8.3 | 0.1 | 8.5 | 1.26 | 1.46 (1/10) | 0 | 2.39 |
| PathPlanner (kP 5) + hold | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.1 | 8.5 | 1.26 | 1.71 (4/10) | 0 | 2.39 |
| PathPlanner (tuned) + hold | 0.1 / 0.3 | 0.1 / 0.3 | 2.7 / 6.1 | 0.1 | 6.9 | 1.28 | 1.45 (4/10) | 42 | 2.44 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.1 | 8.5 | 1.26 | 1.71 (4/10) | 0 | 2.44 |
| DriveToPose chain | 0.1 / 0.3 | 0.1 / 0.3 | 2.3 / 5.9 | 0.0 | 5.2 | 1.33 (9/10) | 1.52 (5/10) | 0 | 0.01 |

**s_curve**: S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.5 / 12.0 | 4.5 / 12.0 | 5.4 / 12.7 | 0.1 | 12.4 | 2.24 (9/10) | 2.42 (1/10) | 239 | 146.87 |
| BLine (README gains, r 0.30) | 13.0 / 37.3 | 14.1 / 37.4 | 6.7 / 15.1 | 0.1 | 96.7 | 3.01 (8/10) | never | 346 | 0.04 |
| BLine (README gains, r 0.45 default) | 8.2 / 27.0 | 9.0 / 27.0 | 5.9 / 13.2 | 0.1 | 91.5 | 2.93 (9/10) | never | 282 | 0.03 |
| BLine (README gains, curve targets, r 0.45) | 8.2 / 27.0 | 9.0 / 27.0 | 5.9 / 13.2 | 0.1 | 91.5 | 2.93 (9/10) | never | 282 | 0.03 |
| BLine (tuned) | 6.6 / 16.6 | 4.4 / 15.9 | 4.2 / 11.3 | 0.1 | 8.2 | 2.61 (8/10) | 3.15 (3/10) | 0 | 0.03 |
| PathPlanner (kP 5) | 1.6 / 3.9 | 1.6 / 3.8 | 5.8 / 13.1 | 0.1 | 12.8 | 1.88 (9/10) | 2.09 (1/10) | 1688 | 4.58 |
| PathPlanner (kP 5) + hold | 1.6 / 3.9 | 1.6 / 3.8 | 4.3 / 10.1 | 0.1 | 12.8 | 1.88 (9/10) | 2.34 (4/10) | 1688 | 3.98 |
| PathPlanner (tuned) + hold | 1.3 / 4.3 | 1.2 / 4.2 | 4.4 / 11.1 | 0.1 | 10.8 | 1.98 (9/10) | 2.28 (4/10) | 963 | 4.04 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.3 / 3.9 | 1.2 / 3.8 | 4.2 / 10.0 | 0.1 | 12.3 | 1.91 (9/10) | 2.37 (4/10) | 1411 | 4.44 |
| DriveToPose chain | 2.9 / 8.7 | 0.8 / 3.3 | 4.1 / 10.5 | 0.0 | 8.9 | 4.04 (8/10) | 4.20 (4/10) | 0 | 0.01 |

**hairpin**: 2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held (9.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 7.0 / 18.4 | 7.0 / 18.4 | 1.8 / 3.8 | 0.1 | 0.8 | 2.85 | 3.01 (7/10) | 1395 | 417.81 |
| BLine (README gains, r 0.30) | 45.5 / 104.3 | 45.9 / 107.5 | 7.8 / 9.9 | 0.1 | 8.8 | 6.54 (9/10) | never | 451 | 0.05 |
| BLine (README gains, r 0.45 default) | 47.0 / 104.2 | 47.3 / 107.4 | 1.7 / 4.5 | 0.1 | 53.7 | 5.02 | 5.44 (7/10) | 0 | 0.04 |
| BLine (README gains, curve targets, r 0.45) | 47.0 / 104.2 | 47.3 / 107.4 | 1.7 / 4.5 | 0.1 | 53.7 | 5.02 | 5.44 (7/10) | 0 | 0.04 |
| BLine (tuned) | 40.0 / 101.8 | 40.9 / 104.5 | 3.8 / 5.8 | 0.1 | 0.0 | 4.33 (9/10) | never | 0 | 0.04 |
| PathPlanner (kP 5) | 13.5 / 24.1 | 13.5 / 24.6 | 49.7 / 55.1 | 0.1 | 0.0 | never | never | 4220 | 4.97 |
| PathPlanner (kP 5) + hold | 13.5 / 24.1 | 13.5 / 24.6 | 52.7 / 57.8 | 0.1 | 0.0 | never | never | 4220 | 4.65 |
| PathPlanner (tuned) + hold | 1.6 / 6.6 | 1.6 / 6.5 | 3.3 / 4.4 | 0.1 | 0.0 | 2.79 | never | 1258 | 4.97 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.8 / 7.0 | 1.9 / 6.9 | 5.7 / 6.8 | 0.1 | 0.0 | 2.65 | never | 3500 | 5.82 |
| DriveToPose chain | 2.1 / 9.2 | 1.6 / 6.2 | 1.7 / 3.8 | 0.0 | 0.0 | 7.61 | 7.81 (8/10) | 0 | 0.01 |

**slalom**: through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m (10.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.2 / 13.3 | 5.2 / 13.3 | 6.5 / 15.7 | 0.1 | 15.4 | 3.09 (9/10) | 3.26 (1/10) | 146 | 141.88 |
| BLine (README gains, r 0.30) | 37.0 / 82.5 | 36.2 / 78.7 | 28.7 / 43.0 | 0.1 | 42.2 | 4.06 | 4.61 (1/10) | 778 | 0.03 |
| BLine (README gains, r 0.45 default) | 29.7 / 69.0 | 29.8 / 64.4 | 28.8 / 40.4 | 0.1 | 38.8 | 3.85 | never | 762 | 0.03 |
| BLine (README gains, curve targets, r 0.45) | 29.7 / 69.0 | 29.8 / 64.4 | 28.8 / 40.4 | 0.1 | 38.8 | 3.85 | never | 762 | 0.03 |
| BLine (tuned) | 10.3 / 28.4 | 7.1 / 25.6 | 5.6 / 14.8 | 0.1 | 11.2 | 3.10 (8/10) | 3.65 (2/10) | 0 | 0.03 |
| PathPlanner (kP 5) | 4.1 / 11.8 | 4.1 / 11.8 | 7.3 / 16.1 | 0.1 | 16.1 | 2.46 (9/10) | never | 3810 | 5.90 |
| PathPlanner (kP 5) + hold | 4.1 / 11.8 | 4.1 / 11.8 | 5.9 / 13.1 | 0.1 | 16.1 | 2.46 (9/10) | 2.92 (2/10) | 3810 | 5.05 |
| PathPlanner (tuned) + hold | 1.7 / 5.8 | 1.7 / 5.8 | 6.3 / 13.8 | 0.1 | 14.8 | 2.70 (9/10) | 3.01 (1/10) | 3553 | 5.04 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.9 / 6.9 | 1.8 / 6.9 | 5.8 / 14.0 | 0.1 | 16.8 | 2.55 (9/10) | 3.02 (4/10) | 3657 | 5.40 |
| DriveToPose chain | 4.2 / 11.3 | 1.2 / 4.0 | 5.4 / 14.0 | 0.0 | 12.1 | 4.68 (8/10) | 4.84 (3/10) | 0 | 0.01 |

**long_auto**: auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end (14.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 6.1 / 25.4 | 6.1 / 25.4 | 8.4 / 18.9 | 3.0 | 5.8 | 4.11 (4/10) | never | 452 | 158.95 |
| BLine (README gains, r 0.30) | 33.0 / 99.5 | 37.3 / 111.8 | 17.8 / 37.7 | 1.8 | 62.9 | 5.73 (5/10) | 6.62 (1/10) | 465 | 0.04 |
| BLine (README gains, r 0.45 default) | 31.7 / 90.8 | 34.8 / 101.9 | 8.7 / 20.0 | 1.2 | 51.8 | 5.71 (4/10) | 5.98 (1/10) | 149 | 0.04 |
| BLine (README gains, curve targets, r 0.45) | 35.5 / 99.2 | 36.3 / 102.8 | 12.3 / 24.9 | 2.5 | 39.6 | 5.02 (5/10) | 5.46 (1/10) | 334 | 0.03 |
| BLine (tuned) | 23.4 / 70.3 | 25.8 / 79.9 | 8.5 / 20.0 | 1.5 | 0.9 | 3.93 (4/10) | never | 27 | 0.03 |
| PathPlanner (kP 5) | 5.8 / 21.7 | 5.8 / 21.7 | 9.8 / 22.1 | 0.9 | 1.1 | 3.74 (4/10) | 3.80 (1/10) | 2330 | 8.86 |
| PathPlanner (kP 5) + hold | 5.8 / 21.7 | 5.8 / 21.7 | 11.6 / 22.9 | 0.9 | 1.1 | 3.74 (4/10) | never | 2330 | 8.68 |
| PathPlanner (tuned) + hold | 4.7 / 20.1 | 4.7 / 20.0 | 8.1 / 20.0 | 0.9 | 4.3 | 3.84 (4/10) | 4.01 (3/10) | 1001 | 8.83 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 4.8 / 20.8 | 4.8 / 20.8 | 8.7 / 20.7 | 0.9 | 5.8 | 3.72 (4/10) | 4.15 (1/10) | 1715 | 8.67 |
| DriveToPose chain | 8.1 / 31.5 | 4.6 / 18.9 | 8.0 / 17.8 | 0.9 | 5.0 | 6.54 (4/10) | 6.64 (3/10) | 2 | 0.00 |

**moving_start**: starts moving 2 m/s sideways (+y), path heads +x then curves left (7.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 13.0 / 27.7 | 13.0 / 27.7 | 5.9 / 11.3 | 0.1 | 10.7 | 1.88 (9/10) | never | 1544 | 94.90 |
| BLine (README gains, r 0.30) | 30.8 / 59.4 | 31.9 / 66.2 | 29.4 / 35.3 | 0.1 | 35.2 | 3.44 | never | 840 | 0.03 |
| BLine (README gains, r 0.45 default) | 33.1 / 54.8 | 34.2 / 51.5 | 23.7 / 39.3 | 0.1 | 55.8 | 2.14 | never | 587 | 0.03 |
| BLine (README gains, curve targets, r 0.45) | 33.1 / 54.8 | 34.2 / 51.5 | 23.7 / 39.3 | 0.1 | 55.8 | 2.14 | never | 587 | 0.03 |
| BLine (tuned) | 34.9 / 56.2 | 33.7 / 52.2 | 3.7 / 8.5 | 0.1 | 6.0 | 2.09 (9/10) | 2.37 (4/10) | 0 | 0.03 |
| PathPlanner (kP 5) | 46.8 / 60.7 | 46.8 / 60.7 | 56.8 / 59.8 | 0.1 | 12.0 | never | never | 2514 | 3.16 |
| PathPlanner (kP 5) + hold | 46.8 / 60.7 | 46.8 / 60.7 | 56.5 / 59.9 | 0.1 | 12.0 | never | never | 2514 | 3.20 |
| PathPlanner (tuned) + hold | 49.3 / 63.8 | 49.2 / 63.7 | 60.4 / 62.9 | 0.1 | 13.3 | never | never | 2317 | 3.27 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 46.8 / 60.7 | 46.8 / 60.6 | 56.5 / 59.9 | 0.1 | 12.1 | never | never | 2506 | 3.24 |
| DriveToPose chain | 21.1 / 58.7 | 19.5 / 54.8 | 3.8 / 8.3 | 0.0 | 7.6 | 3.65 (9/10) | 3.86 (4/10) | 0 | 0.01 |

**hit_mid_path**: the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 31.2 / 43.4 | 31.2 / 43.4 | 41.8 / 48.6 | 0.1 | 1.7 | never | never | 2182 | 83.69 |
| BLine (README gains, r 0.30) | 32.0 / 80.7 | 33.8 / 81.9 | 55.6 / 65.4 | 0.1 | 72.7 | never | never | 2989 | 0.03 |
| BLine (README gains, r 0.45 default) | 25.2 / 64.9 | 26.9 / 65.8 | 49.1 / 55.4 | 0.1 | 70.1 | never | never | 2838 | 0.03 |
| BLine (README gains, curve targets, r 0.45) | 25.2 / 64.9 | 26.9 / 65.8 | 49.1 / 55.4 | 0.1 | 70.1 | never | never | 2838 | 0.03 |
| BLine (tuned) | 38.7 / 44.5 | 39.4 / 44.9 | 46.0 / 48.8 | 0.1 | 0.0 | never | never | 2373 | 0.03 |
| PathPlanner (kP 5) | 41.6 / 51.7 | 41.6 / 51.7 | 51.8 / 53.6 | 0.1 | 1.3 | never | never | 3722 | 4.28 |
| PathPlanner (kP 5) + hold | 41.5 / 51.7 | 41.5 / 51.7 | 52.4 / 54.9 | 0.1 | 1.3 | never | never | 3722 | 3.92 |
| PathPlanner (tuned) + hold | 39.9 / 47.5 | 39.8 / 47.5 | 48.1 / 50.0 | 0.1 | 0.0 | never | never | 3066 | 4.08 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 40.2 / 50.1 | 40.1 / 50.1 | 50.7 / 52.8 | 0.1 | 1.0 | never | never | 3491 | 3.98 |
| DriveToPose chain | 15.0 / 29.8 | 16.8 / 32.8 | 33.3 / 41.6 | 0.0 | 0.0 | never | never | 1931 | 0.00 |

**low_grip_s_curve**: the S-curve on mu 0.70 static / 0.61 kinetic (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 11.6 | 4.6 / 11.6 | 12.0 / 20.2 | 0.1 | 20.0 | 2.23 | never | 4626 | 94.24 |
| BLine (README gains, r 0.30) | 12.2 / 36.6 | 13.4 / 36.6 | 6.4 / 15.6 | 0.1 | 97.2 | 3.06 (8/10) | never | 11148 | 0.03 |
| BLine (README gains, r 0.45 default) | 7.5 / 25.4 | 8.3 / 25.5 | 5.0 / 13.0 | 0.1 | 92.1 | 3.01 (8/10) | 3.14 (1/10) | 10434 | 0.03 |
| BLine (README gains, curve targets, r 0.45) | 7.5 / 25.4 | 8.3 / 25.5 | 5.0 / 13.0 | 0.1 | 92.1 | 3.01 (8/10) | 3.14 (1/10) | 10434 | 0.03 |
| BLine (tuned) | 7.1 / 17.7 | 5.0 / 16.9 | 5.0 / 14.0 | 0.1 | 5.6 | 2.77 (7/10) | 3.19 (3/10) | 5220 | 0.03 |
| PathPlanner (kP 5) | 10.1 / 19.8 | 10.0 / 19.7 | 30.5 / 39.4 | 0.1 | 39.3 | 1.83 (8/10) | never | 6984 | 4.30 |
| PathPlanner (kP 5) + hold | 10.1 / 19.8 | 10.0 / 19.7 | 27.4 / 36.5 | 0.1 | 39.2 | 1.83 (8/10) | never | 7102 | 3.89 |
| PathPlanner (tuned) + hold | 3.7 / 9.2 | 3.6 / 9.1 | 7.6 / 14.0 | 0.1 | 14.0 | 2.00 | never | 7119 | 4.84 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 7.7 / 15.8 | 7.6 / 15.6 | 16.1 / 24.8 | 0.1 | 27.5 | 1.89 (9/10) | never | 7003 | 4.10 |
| DriveToPose chain | 3.1 / 9.6 | 1.1 / 4.1 | 4.8 / 13.2 | 0.0 | 4.8 | 4.00 (7/10) | 4.13 (3/10) | 12394 | 0.01 |

### Perfect pose (the control law alone)

| method | runs settled | courses settled every seed | mean RMS cross-track cm | worst cross-track cm | mean final error cm | mean wheel slip ms |
|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 11/80 | 1/8 | 6.7 | 37.4 | 4.0 | 1485 |
| BLine (README gains, r 0.30) | 11/80 | 0/8 | 26.3 | 98.9 | 19.5 | 2437 |
| BLine (README gains, r 0.45 default) | 6/80 | 0/8 | 23.1 | 98.7 | 14.4 | 2162 |
| BLine (README gains, curve targets, r 0.45) | 6/80 | 0/8 | 23.6 | 98.7 | 14.4 | 2190 |
| BLine (tuned) | 75/80 | 7/8 | 17.6 | 96.8 | 0.8 | 1068 |
| PathPlanner (kP 5) | 0/80 | 0/8 | 10.3 | 53.4 | 19.7 | 4325 |
| PathPlanner (kP 5) + hold | 80/80 | 8/8 | 10.3 | 53.4 | 0.1 | 4807 |
| PathPlanner (tuned) + hold | 80/80 | 8/8 | 6.7 | 46.4 | 0.6 | 4211 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 80/80 | 8/8 | 7.3 | 50.0 | 0.1 | 3923 |
| DriveToPose chain | 80/80 | 8/8 | 5.9 | 58.6 | 0.3 | 1923 |

**straight_3m**: 3 m straight, heading held at 0 (6.5 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 0.0 / 0.1 | 0.0 / 0.0 | 3.4 / 3.5 | 0.0 | 3.5 | 1.35 | never | 83 | 51.16 |
| BLine (README gains, r 0.30) | 0.0 / 0.1 | 0.0 / 0.1 | 41.5 / 42.8 | 0.0 | 42.8 | 1.13 | never | 1059 | 0.03 |
| BLine (README gains, r 0.45 default) | 0.0 / 0.1 | 0.0 / 0.1 | 41.5 / 42.8 | 0.0 | 42.8 | 1.13 | never | 1059 | 0.03 |
| BLine (README gains, curve targets, r 0.45) | 0.0 / 0.1 | 0.0 / 0.1 | 41.5 / 42.8 | 0.0 | 42.8 | 1.13 | never | 1059 | 0.03 |
| BLine (tuned) | 0.0 / 0.1 | 0.0 / 0.1 | 0.8 / 0.9 | 0.0 | 0.0 | 2.20 | 2.67 (10/10) | 0 | 0.03 |
| PathPlanner (kP 5) | 0.0 / 0.1 | 0.0 / 0.1 | 3.2 / 3.4 | 0.0 | 3.4 | 1.26 | never | 0 | 2.48 |
| PathPlanner (kP 5) + hold | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.1 | 0.0 | 3.4 | 1.26 | 1.71 (10/10) | 0 | 2.31 |
| PathPlanner (tuned) + hold | 0.0 / 0.1 | 0.0 / 0.0 | 0.8 / 1.5 | 0.0 | 1.6 | 1.28 | 1.49 (10/10) | 0 | 2.40 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.0 / 0.1 | 0.0 / 0.1 | 0.1 / 0.1 | 0.0 | 3.4 | 1.26 | 1.71 (10/10) | 0 | 2.89 |
| DriveToPose chain | 0.1 / 0.1 | 0.0 / 0.1 | 0.1 / 0.1 | 0.0 | 0.0 | 1.34 | 1.55 (10/10) | 0 | 0.00 |

**s_curve**: S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.6 / 9.1 | 4.6 / 9.1 | 3.3 / 3.4 | 0.0 | 3.4 | 2.24 | never | 254 | 105.45 |
| BLine (README gains, r 0.30) | 12.8 / 37.6 | 14.0 / 37.7 | 5.7 / 6.3 | 0.0 | 89.9 | 2.99 | never | 365 | 0.04 |
| BLine (README gains, r 0.45 default) | 8.4 / 26.6 | 9.4 / 26.6 | 4.2 / 4.5 | 0.0 | 84.9 | 2.93 | never | 299 | 0.04 |
| BLine (README gains, curve targets, r 0.45) | 8.4 / 26.6 | 9.4 / 26.6 | 4.2 / 4.5 | 0.0 | 84.9 | 2.93 | never | 299 | 0.04 |
| BLine (tuned) | 6.8 / 15.6 | 4.4 / 15.0 | 0.8 / 0.9 | 0.0 | 0.0 | 2.70 | 3.17 (10/10) | 0 | 0.04 |
| PathPlanner (kP 5) | 0.8 / 2.5 | 0.8 / 2.6 | 3.3 / 3.5 | 0.0 | 3.6 | 1.89 | never | 1938 | 5.65 |
| PathPlanner (kP 5) + hold | 0.8 / 2.5 | 0.8 / 2.6 | 0.1 / 0.1 | 0.0 | 3.6 | 1.89 | 2.34 (10/10) | 1938 | 3.95 |
| PathPlanner (tuned) + hold | 0.4 / 1.1 | 0.3 / 1.1 | 0.4 / 1.5 | 0.0 | 1.6 | 1.99 | 2.25 (10/10) | 1066 | 5.65 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.7 / 1.8 | 0.6 / 1.8 | 0.1 / 0.1 | 0.0 | 3.5 | 1.92 | 2.36 (10/10) | 1544 | 4.23 |
| DriveToPose chain | 2.8 / 6.0 | 0.2 / 0.4 | 0.1 / 0.1 | 0.0 | 0.0 | 4.08 | 4.28 (10/10) | 0 | 0.01 |

**hairpin**: 2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held (9.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 7.3 / 13.3 | 7.3 / 13.3 | 3.4 / 3.5 | 0.0 | 3.5 | 2.82 | never | 1429 | 126.42 |
| BLine (README gains, r 0.30) | 45.4 / 98.9 | 45.8 / 102.1 | 9.8 / 11.5 | 0.0 | 11.5 | 6.57 | never | 472 | 0.05 |
| BLine (README gains, r 0.45 default) | 47.2 / 98.7 | 47.4 / 101.8 | 2.3 / 2.4 | 0.0 | 54.7 | 5.40 | never | 0 | 0.05 |
| BLine (README gains, curve targets, r 0.45) | 47.2 / 98.7 | 47.4 / 101.8 | 2.3 / 2.4 | 0.0 | 54.7 | 5.40 | never | 0 | 0.05 |
| BLine (tuned) | 40.9 / 96.8 | 41.8 / 99.3 | 0.8 / 0.9 | 0.0 | 0.0 | 4.06 | 4.53 (10/10) | 0 | 0.04 |
| PathPlanner (kP 5) | 18.7 / 40.8 | 18.8 / 40.9 | 21.2 / 22.1 | 0.0 | 21.9 | 2.44 | never | 6552 | 5.15 |
| PathPlanner (kP 5) + hold | 18.7 / 40.8 | 18.8 / 40.9 | 0.1 / 0.1 | 0.0 | 21.6 | 2.44 | 3.26 (10/10) | 6950 | 7.60 |
| PathPlanner (tuned) + hold | 0.8 / 1.5 | 0.6 / 1.4 | 1.2 / 1.5 | 0.0 | 1.6 | 2.74 | 2.90 (10/10) | 1278 | 5.04 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 0.8 / 1.9 | 0.7 / 1.8 | 0.1 / 0.1 | 0.0 | 3.4 | 2.57 | 3.02 (10/10) | 3560 | 4.91 |
| DriveToPose chain | 1.2 / 3.2 | 0.1 / 0.3 | 0.1 / 0.1 | 0.0 | 0.0 | 7.74 | 7.95 (10/10) | 0 | 0.01 |

**slalom**: through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m (10.3 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.5 / 10.2 | 5.5 / 10.2 | 3.3 / 3.5 | 0.0 | 3.5 | 3.08 | 3.26 (1/10) | 168 | 108.13 |
| BLine (README gains, r 0.30) | 40.3 / 83.4 | 40.9 / 80.8 | 13.3 / 31.0 | 0.0 | 40.9 | 4.29 | 4.57 (3/10) | 362 | 0.04 |
| BLine (README gains, r 0.45 default) | 30.0 / 67.8 | 30.0 / 63.2 | 29.0 / 30.6 | 0.0 | 28.9 | 3.87 | never | 854 | 0.04 |
| BLine (README gains, curve targets, r 0.45) | 30.0 / 67.8 | 30.0 / 63.2 | 29.0 / 30.6 | 0.0 | 28.9 | 3.87 | never | 854 | 0.03 |
| BLine (tuned) | 11.0 / 28.7 | 7.5 / 25.8 | 0.8 / 0.9 | 0.0 | 0.0 | 3.19 | 3.66 (10/10) | 0 | 0.03 |
| PathPlanner (kP 5) | 5.3 / 12.3 | 5.3 / 12.4 | 3.4 / 3.6 | 0.0 | 3.6 | 2.47 | never | 5242 | 5.42 |
| PathPlanner (kP 5) + hold | 5.3 / 12.3 | 5.3 / 12.4 | 0.1 / 0.1 | 0.0 | 3.6 | 2.47 | 2.93 (10/10) | 5242 | 5.27 |
| PathPlanner (tuned) + hold | 0.4 / 1.3 | 0.3 / 1.2 | 0.4 / 1.4 | 0.0 | 1.6 | 2.72 | 2.98 (10/10) | 3763 | 5.22 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.0 / 3.1 | 1.1 / 3.4 | 0.1 / 0.1 | 0.0 | 3.4 | 2.57 | 3.02 (10/10) | 4203 | 5.20 |
| DriveToPose chain | 4.2 / 8.7 | 0.3 / 0.7 | 0.1 / 0.1 | 0.0 | 0.0 | 4.74 | 4.94 (10/10) | 0 | 0.00 |

**long_auto**: auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end (14.1 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 5.0 / 10.8 | 5.0 / 10.8 | 3.0 / 3.2 | 2.3 | 3.2 | 4.11 | never | 466 | 177.89 |
| BLine (README gains, r 0.30) | 32.2 / 86.2 | 36.5 / 100.5 | 1.8 / 2.4 | 0.4 | 64.0 | 5.55 | 6.65 (7/10) | 630 | 0.03 |
| BLine (README gains, r 0.45 default) | 31.2 / 76.8 | 34.3 / 89.8 | 2.2 / 2.4 | 0.5 | 51.4 | 5.10 | never | 206 | 0.03 |
| BLine (README gains, curve targets, r 0.45) | 35.2 / 84.7 | 35.9 / 88.8 | 2.2 / 2.3 | 1.5 | 37.0 | 5.05 | never | 427 | 0.04 |
| BLine (tuned) | 23.1 / 56.0 | 25.7 / 65.2 | 0.9 / 0.9 | 1.7 | 0.0 | 3.93 | 4.46 (5/10) | 50 | 0.03 |
| PathPlanner (kP 5) | 1.8 / 5.2 | 1.9 / 5.4 | 3.1 / 3.3 | 0.0 | 3.3 | 3.63 | never | 2550 | 8.45 |
| PathPlanner (kP 5) + hold | 1.8 / 5.2 | 1.9 / 5.4 | 0.1 / 0.1 | 0.0 | 3.3 | 3.63 | 4.07 (10/10) | 2550 | 8.63 |
| PathPlanner (tuned) + hold | 0.7 / 1.6 | 0.7 / 1.6 | 0.5 / 1.4 | 0.0 | 1.5 | 3.82 | 4.06 (10/10) | 857 | 8.62 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 1.3 / 2.9 | 1.3 / 2.9 | 0.1 / 0.1 | 0.0 | 3.4 | 3.70 | 4.14 (10/10) | 1817 | 8.83 |
| DriveToPose chain | 6.9 / 21.3 | 1.1 / 3.3 | 0.1 / 0.4 | 0.2 | 0.4 | 6.40 | 6.55 (10/10) | 0 | 0.00 |

**moving_start**: starts moving 2 m/s sideways (+y), path heads +x then curves left (7.9 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 12.0 / 27.7 | 12.0 / 27.7 | 3.4 / 3.5 | 0.0 | 3.5 | 1.90 | never | 1584 | 72.65 |
| BLine (README gains, r 0.30) | 31.3 / 58.4 | 32.4 / 66.0 | 28.5 / 30.3 | 0.0 | 30.1 | 3.48 | never | 866 | 0.03 |
| BLine (README gains, r 0.45 default) | 33.4 / 54.6 | 34.5 / 51.3 | 18.5 / 36.2 | 0.0 | 54.3 | 2.08 | 3.06 (5/10) | 514 | 0.03 |
| BLine (README gains, curve targets, r 0.45) | 33.4 / 54.6 | 34.5 / 51.3 | 18.5 / 36.2 | 0.0 | 54.3 | 2.08 | 3.06 (5/10) | 514 | 0.03 |
| BLine (tuned) | 35.1 / 55.9 | 33.9 / 51.9 | 0.8 / 0.8 | 0.0 | 4.3 | 2.06 | 2.44 (10/10) | 0 | 0.03 |
| PathPlanner (kP 5) | 20.0 / 33.5 | 20.0 / 33.5 | 3.2 / 3.4 | 0.0 | 3.5 | 1.44 | never | 4275 | 3.62 |
| PathPlanner (kP 5) + hold | 20.0 / 33.5 | 20.0 / 33.5 | 0.1 / 0.1 | 0.0 | 3.5 | 1.44 | 1.89 (10/10) | 4275 | 3.33 |
| PathPlanner (tuned) + hold | 17.1 / 31.7 | 17.0 / 31.7 | 0.2 / 0.2 | 0.0 | 9.5 | 1.39 | 1.81 (10/10) | 5893 | 3.27 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 20.0 / 33.5 | 20.0 / 33.5 | 0.1 / 0.1 | 0.0 | 3.5 | 1.45 | 1.89 (10/10) | 4224 | 3.80 |
| DriveToPose chain | 21.5 / 58.6 | 19.9 / 54.7 | 0.1 / 0.1 | 0.0 | 0.0 | 3.70 | 3.90 (10/10) | 0 | 0.00 |

**hit_mid_path**: the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 14.2 / 37.4 | 14.2 / 37.4 | 1.6 / 2.0 | 0.0 | 1.8 | 2.31 | 2.49 (10/10) | 3178 | 83.88 |
| BLine (README gains, r 0.30) | 34.5 / 74.5 | 36.4 / 78.1 | 37.1 / 42.6 | 0.0 | 67.3 | 4.32 | 5.46 (1/10) | 4037 | 0.04 |
| BLine (README gains, r 0.45 default) | 25.1 / 63.0 | 26.9 / 66.0 | 2.1 / 2.3 | 0.0 | 80.8 | 3.12 | 3.73 (1/10) | 3318 | 0.03 |
| BLine (README gains, curve targets, r 0.45) | 25.1 / 63.0 | 26.9 / 66.0 | 2.1 / 2.3 | 0.0 | 80.8 | 3.12 | 3.73 (1/10) | 3318 | 0.03 |
| BLine (tuned) | 17.5 / 38.2 | 18.9 / 43.4 | 0.8 / 0.8 | 0.0 | 1.6 | 2.08 | 2.30 (10/10) | 2978 | 0.03 |
| PathPlanner (kP 5) | 22.4 / 53.4 | 22.5 / 53.4 | 32.0 / 32.5 | 0.0 | 32.6 | never | never | 5517 | 4.03 |
| PathPlanner (kP 5) + hold | 22.4 / 53.4 | 22.5 / 53.4 | 0.1 / 0.1 | 0.0 | 31.1 | 2.49 | 2.79 (10/10) | 5856 | 4.23 |
| PathPlanner (tuned) + hold | 20.3 / 46.4 | 20.3 / 46.5 | 1.0 / 1.9 | 0.0 | 20.4 | 2.51 | 2.70 (10/10) | 6602 | 3.76 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 21.5 / 50.0 | 21.5 / 50.0 | 0.1 / 0.1 | 0.0 | 24.4 | 2.44 | 2.75 (10/10) | 5578 | 3.91 |
| DriveToPose chain | 8.2 / 29.2 | 8.8 / 31.5 | 0.1 / 0.1 | 0.0 | 0.0 | 4.18 | 4.39 (10/10) | 2293 | 0.00 |

**low_grip_s_curve**: the S-curve on mu 0.70 static / 0.61 kinetic (8.6 s simulated)

| method | cross-track vs authored path, RMS / max cm | vs own geometry RMS / max cm | final error cm, mean / worst | final yaw worst deg | overshoot worst cm | t to 5 cm s | t settled s (n/N) | wheel slip ms | path prep ms |
|---|---|---|---|---|---|---|---|---|---|
| Old path-editor FollowPath | 4.5 / 8.8 | 4.5 / 8.8 | 10.7 / 10.9 | 0.0 | 10.9 | 2.23 | never | 4717 | 89.35 |
| BLine (README gains, r 0.30) | 14.0 / 39.9 | 15.2 / 39.9 | 18.5 / 19.3 | 0.0 | 96.5 | 3.06 | never | 11706 | 0.03 |
| BLine (README gains, r 0.45 default) | 9.3 / 28.6 | 10.3 / 28.6 | 15.1 / 15.6 | 0.0 | 91.7 | 2.99 | never | 11048 | 0.03 |
| BLine (README gains, curve targets, r 0.45) | 9.3 / 28.6 | 10.3 / 28.6 | 15.1 / 15.6 | 0.0 | 91.7 | 2.99 | never | 11048 | 0.03 |
| BLine (tuned) | 6.5 / 15.5 | 4.3 / 14.8 | 0.8 / 0.8 | 0.0 | 0.0 | 2.66 | 3.13 (10/10) | 5515 | 0.03 |
| PathPlanner (kP 5) | 13.5 / 36.8 | 13.6 / 36.8 | 88.0 / 88.5 | 0.0 | 84.8 | never | never | 8528 | 3.93 |
| PathPlanner (kP 5) + hold | 13.5 / 36.8 | 13.6 / 36.8 | 0.1 / 0.1 | 0.0 | 83.5 | 2.91 | 3.35 (10/10) | 11646 | 3.83 |
| PathPlanner (tuned) + hold | 13.7 / 35.2 | 13.7 / 35.1 | 0.2 / 0.2 | 0.0 | 58.4 | 3.41 | 3.77 (10/10) | 14230 | 4.30 |
| PathPlanner (kP 5) + hold, wheel COF 0.9 | 12.8 / 33.3 | 12.8 / 33.3 | 0.1 / 0.1 | 0.0 | 61.7 | 2.84 | 3.14 (10/10) | 10456 | 3.75 |
| DriveToPose chain | 2.7 / 6.0 | 0.6 / 1.4 | 1.6 / 1.7 | 0.0 | 1.7 | 3.80 | 3.94 (10/10) | 13094 | 0.00 |

## Geometry each method authored

**straight_3m**

- Waypoints (old path-editor control points; PathPlanner anchors): (0.00, 0.00) @0 deg, (3.00, 0.00) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (1.00, 0.00); (3.00, 0.00): (2.00, 0.00), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, R 0 deg @t 1, (3.00, 0.00) (handoff 0.30 m; 0.45 m in the r 0.45 row; the tuned row's radius is in the sweep)
- BLine curve targets (2 targets, handoff 0.45 m): (0.00, 0.00), (3.00, 0.00)
- DriveToPose chain goals: (3.00, 0.00) @0 deg
- Neutral path length 3.00 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**s_curve**

- Waypoints (old path-editor control points; PathPlanner anchors): (0.00, 0.00) @0 deg, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.42, 0.00); (1.25, 0.00): (0.83, -0.13), (1.67, 0.13); (2.50, 0.80): (2.08, 0.53), (2.92, 1.07); (3.75, 1.60): (3.33, 1.47), (4.17, 1.73); (5.00, 1.60): (4.58, 1.60), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), R 0 deg @t 1, (5.00, 1.60) (handoff 0.30 m; 0.45 m in the r 0.45 row; the tuned row's radius is in the sweep)
- BLine curve targets (5 targets, handoff 0.45 m): (0.00, 0.00), (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60)
- DriveToPose chain goals: (1.25, 0.00) @0 deg, (2.50, 0.80) @0 deg, (3.75, 1.60) @0 deg, (5.00, 1.60) @0 deg
- Neutral path length 5.50 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**hairpin**

- Waypoints (old path-editor control points; PathPlanner anchors): (0.00, 0.00) @0 deg, (1.00, 0.00), (2.00, 0.00), (2.50, 0.13), (2.87, 0.50), (3.00, 1.00), (2.87, 1.50), (2.50, 1.87), (2.00, 2.00), (1.00, 2.00), (0.00, 2.00) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.33, 0.00); (1.00, 0.00): (0.67, 0.00), (1.33, 0.00); (2.00, 0.00): (1.75, -0.02), (2.25, 0.02); (2.50, 0.13): (2.36, 0.05), (2.64, 0.22); (2.87, 0.50): (2.78, 0.36), (2.95, 0.64); (3.00, 1.00): (3.00, 0.83), (3.00, 1.17); (2.87, 1.50): (2.95, 1.36), (2.78, 1.64); (2.50, 1.87): (2.64, 1.78), (2.36, 1.95); (2.00, 2.00): (2.25, 1.98), (1.75, 2.02); (1.00, 2.00): (1.33, 2.00), (0.67, 2.00); (0.00, 2.00): (0.33, 2.00), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.00, 0.00), (2.00, 0.00), (2.50, 0.13), (2.87, 0.50), (3.00, 1.00), (2.87, 1.50), (2.50, 1.87), (2.00, 2.00), (1.00, 2.00), R 0 deg @t 1, (0.00, 2.00) (handoff 0.30 m; 0.45 m in the r 0.45 row; the tuned row's radius is in the sweep)
- BLine curve targets (11 targets, handoff 0.45 m): (0.00, 0.00), (1.00, 0.00), (2.00, 0.00), (2.50, 0.13), (2.87, 0.50), (3.00, 1.00), (2.87, 1.50), (2.50, 1.87), (2.00, 2.00), (1.00, 2.00), (0.00, 2.00)
- DriveToPose chain goals: (1.00, 0.00) @0 deg, (2.00, 0.00) @0 deg, (2.50, 0.13) @0 deg, (2.87, 0.50) @0 deg, (3.00, 1.00) @0 deg, (2.87, 1.50) @0 deg, (2.50, 1.87) @0 deg, (2.00, 2.00) @0 deg, (1.00, 2.00) @0 deg, (0.00, 2.00) @0 deg
- Neutral path length 7.14 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**slalom**

- Waypoints (old path-editor control points; PathPlanner anchors): (0.00, 0.00) @0 deg, (1.75, 0.40), (3.50, -0.40), (5.25, 0.40), (7.00, 0.00) @0 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.58, 0.13); (1.75, 0.40): (1.17, 0.47), (2.33, 0.33); (3.50, -0.40): (2.92, -0.40), (4.08, -0.40); (5.25, 0.40): (4.67, 0.33), (5.83, 0.47); (7.00, 0.00): (6.42, 0.13), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.75, 0.40), (3.50, -0.40), (5.25, 0.40), R 0 deg @t 1, (7.00, 0.00) (handoff 0.30 m; 0.45 m in the r 0.45 row; the tuned row's radius is in the sweep)
- BLine curve targets (5 targets, handoff 0.45 m): (0.00, 0.00), (1.75, 0.40), (3.50, -0.40), (5.25, 0.40), (7.00, 0.00)
- DriveToPose chain goals: (1.75, 0.40) @0 deg, (3.50, -0.40) @0 deg, (5.25, 0.40) @0 deg, (7.00, 0.00) @0 deg
- Neutral path length 7.51 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**long_auto**

- Waypoints (old path-editor control points; PathPlanner anchors): (0.00, 0.00) @0 deg, (3.00, 0.50), (5.50, 2.00) @90 deg, (7.50, 3.20), (9.00, 2.20), (9.30, 0.30) @180 deg
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (1.00, 0.17); (3.00, 0.50): (2.08, 0.17), (3.92, 0.83); (5.50, 2.00): (4.75, 1.55), (6.25, 2.45); (7.50, 3.20): (6.92, 3.17), (8.08, 3.23); (9.00, 2.20): (8.70, 2.68), (9.30, 1.72); (9.30, 0.30): (9.20, 0.93), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (3.00, 0.50), R 90 deg @t 1, (5.50, 2.00), (7.50, 3.20), (9.00, 2.20), R 180 deg @t 1, (9.30, 0.30) (handoff 0.30 m; 0.45 m in the r 0.45 row; the tuned row's radius is in the sweep)
- BLine curve targets (7 targets, handoff 0.45 m): (0.00, 0.00), (3.00, 0.50), (5.50, 2.00), (7.50, 3.20), (8.28, 2.95), (9.00, 2.20), (9.30, 0.30)
- DriveToPose chain goals: (3.00, 0.50) @45 deg, (5.50, 2.00) @90 deg, (7.50, 3.20) @120 deg, (9.00, 2.20) @150 deg, (9.30, 0.30) @180 deg
- Neutral path length 12.14 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**moving_start**

- Waypoints (old path-editor control points; PathPlanner anchors): (0.00, 0.00) @0 deg, (1.50, 0.40), (2.60, 1.40), (3.00, 3.00) @0 deg
- Start velocity (field): (0.0, 2.0, 0.0)
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.50, 0.13); (1.50, 0.40): (1.07, 0.17), (1.93, 0.63); (2.60, 1.40): (2.35, 0.97), (2.85, 1.83); (3.00, 3.00): (2.87, 2.47), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.50, 0.40), (2.60, 1.40), R 0 deg @t 1, (3.00, 3.00) (handoff 0.30 m; 0.45 m in the r 0.45 row; the tuned row's radius is in the sweep)
- BLine curve targets (4 targets, handoff 0.45 m): (0.00, 0.00), (1.50, 0.40), (2.60, 1.40), (3.00, 3.00)
- DriveToPose chain goals: (1.50, 0.40) @0 deg, (2.60, 1.40) @0 deg, (3.00, 3.00) @0 deg
- Neutral path length 4.72 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**hit_mid_path**

- Waypoints (old path-editor control points; PathPlanner anchors): (0.00, 0.00) @0 deg, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60) @0 deg
- Shove: 800 N for 0.3 s at 50% of the path, to the left of travel
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.42, 0.00); (1.25, 0.00): (0.83, -0.13), (1.67, 0.13); (2.50, 0.80): (2.08, 0.53), (2.92, 1.07); (3.75, 1.60): (3.33, 1.47), (4.17, 1.73); (5.00, 1.60): (4.58, 1.60), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), R 0 deg @t 1, (5.00, 1.60) (handoff 0.30 m; 0.45 m in the r 0.45 row; the tuned row's radius is in the sweep)
- BLine curve targets (5 targets, handoff 0.45 m): (0.00, 0.00), (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60)
- DriveToPose chain goals: (1.25, 0.00) @0 deg, (2.50, 0.80) @0 deg, (3.75, 1.60) @0 deg, (5.00, 1.60) @0 deg
- Neutral path length 5.50 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

**low_grip_s_curve**

- Waypoints (old path-editor control points; PathPlanner anchors): (0.00, 0.00) @0 deg, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60) @0 deg
- Grip: mu 0.7 static / 0.61 kinetic
- PathPlanner Bezier handles (prev, next): (0.00, 0.00): -, (0.42, 0.00); (1.25, 0.00): (0.83, -0.13), (1.67, 0.13); (2.50, 0.80): (2.08, 0.53), (2.92, 1.07); (3.75, 1.60): (3.33, 1.47), (4.17, 1.73); (5.00, 1.60): (4.58, 1.60), -
- BLine (waypoints): (0.00, 0.00), R 0 deg @t 0, (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), R 0 deg @t 1, (5.00, 1.60) (handoff 0.30 m; 0.45 m in the r 0.45 row; the tuned row's radius is in the sweep)
- BLine curve targets (5 targets, handoff 0.45 m): (0.00, 0.00), (1.25, 0.00), (2.50, 0.80), (3.75, 1.60), (5.00, 1.60)
- DriveToPose chain goals: (1.25, 0.00) @0 deg, (2.50, 0.80) @0 deg, (3.75, 1.60) @0 deg, (5.00, 1.60) @0 deg
- Neutral path length 5.50 m; old path-editor and PathPlanner author the same cubics (to_catmull / to_bezier); sampled geometry of every method is in results.json

<!-- generated:end -->

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
  time) plus 10 s per metre of RMS cross-track. Both choices landed on an edge of their grid: BLine on the largest
  handoff radius (1.2 m), PathPlanner on the lowest COF (0.7) and the highest kP (10). A wider grid might do better.
  The tuned rows are this sim's best, not the authors' recommendations.

**Port approximations**
- **BLine.**
  - Swerve only. Handoff mode is RADIUS, the library default; PROGRESS is not ported.
  - Not ported: events, rotation override, flip/mirror, rolling ends (min velocity 0), per-element constraint ranges.
  - Not ported: BLine-Web's automatic velocity constraints and automatic handoff radii. These are what slow a BLine
    path for corners, so the corner results above are BLine with every segment at 4 m/s.
  - Gains are P-only, as in the README. End tolerances are 3 cm / 2 deg from the BLine-Web defaults.
  - The "curve targets" row uses BLine-Web's curve-to-targets (RDP 0.18 m, 0.35 m spacing, 0.45 m handoff). It only
    adds a target on the 12 m route ((8.28, 2.95)), so elsewhere it equals the r 0.45 row.
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
- **Courses.** The neutral curve is the old editor's Catmull-Rom through the waypoints. PathPlanner gets Bezier
  handles that reproduce those exact cubics (checked in `tests/test_courses.py`). Catmull-Rom tightens the hairpin
  to 0.75 m at the joins.

## Sources

| What | Repository, branch, commit | Files ported |
|---|---|---|
| BLine follower | github.com/edanliahovetsky/BLine-Lib, `wpilib-2027`, 44022501faa981e5cbb9ea9492d8ac4bc67acdc5 (2026-10-15, "2027 beta.3") | `following/Follower`, `TranslationProgress`, `RotationProgress`, `TranslationGuidance`, `HolonomicController`, `HolonomicRateLimiter`; `path/PreparedPath`, `Path` (waypoint expansion); README gains |
| BLine defaults, curve tool | github.com/edanliahovetsky/BLine-Web, `bline-2027`, 57127fd43ae5d8f650160a68bf37afb2bc0b49c1 | `core/config/projectConfig.ts` (0.45 m handoff, 3 cm / 2 deg tolerances), `core/pathProfile/curveProfile.ts` |
| PathPlanner | github.com/mjansen4857/pathplanner, `2027`, 7fbd7980c5f81874d56a5b64c39fb8d8c3037352 (2026-10-03) | `path/PathPlannerPath`, `trajectory/PathPlannerTrajectory`, `PathPlannerTrajectoryState`, `config/RobotConfig`, `ModuleConfig`, `util/GeometryUtil`, `controllers/PPHolonomicDriveController`, `commands/FollowPathCommand`; Kraken X60 FOC numbers from `lib/trajectory/dc_motor.dart` |
| Old path-editor | github.com/Hemlock5712/path-editor, `master`, 29d7ada471fbee71af8325ad7fd842697d3dc83e (2026-04-25) | `public/java/commands/FollowPath.java`, `utils/path/SplinePath`, `CubicSegment`, `VelocityProfile`, `VelocityConstraints`, `RotationSupplier` |
| DriveToPose, AccelerationLimiter | this repo, `bench/reference/DriveToPose.java`, `AccelerationLimiter.java` (via `race_drive_to_pose.py`) | `execute`, `isFinished`, `limit` |
| Plant | this repo, `bench/alignsim` (unchanged) | |
