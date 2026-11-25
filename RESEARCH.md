# Control research: what industry does, and what it means for this library

Written 2026-11-25. Two questions: is the follower in `lib/` in line with best practice outside FRC, and what does a
faster controller (Systemcore) make worth doing?

**How far to trust this.** The research could search the web but could not open arxiv.org, Chief Delphi, the WPILib,
PathPlanner, Choreo or CTRE doc sites, or ssl.robocup.org. Papers below were checked against their abstracts or search
summaries, not read in full; GitHub sources were read directly. Nothing here was measured on our robot or bench. No
source gives a solver timing on Systemcore-class hardware, so every compute figure is from another machine. Each
recommendation should go through `bench/` before it goes on a robot, as PLAN.md already requires.

## Where the current design already matches

| What we do | What the literature says |
| --- | --- |
| Follow the path by arc length, not by the clock | Path following removes a performance limit that time tracking has (Aguiar, Hespanha & Kokotović, IEEE TAC 2005). A swerve base is close to fully actuated, so the practical gain is at the traction limit: a late robot slows its progress instead of cutting the corner. |
| Forward and backward pass over a grip circle and the motor curve | Still the mainstream in autonomous racing (Heilmeier et al., Vehicle System Dynamics 2020, on the Roborace car; reused by ForzaETH). Full minimum-time optimal control gained about 1.4 % on a whole racing lap in one survey, and most of that was the racing line. TOPP-RA (Pham & Pham, T-RO 2018) solves the same problem; it would not be faster or better here. |
| Project, track progress, hold a friction limit | This is Model Predictive Contouring Control (Lam et al. 2010; Liniger et al. 2015) done by hand, without the preview. |
| Hand over to a drive-to-pose law at the end | Choreo's and PathPlanner's own docs say their followers do not guarantee the end pose; 6328 and 254 both finish with a separate profiled controller seeded from the measured closing speed. |
| P gains of 5 /s | FRC teams publish 5 to 10 /s (254: 5, 6328: 8, Choreo's example: 10). |

So the structure is sound. The gaps are in four places, in the order the evidence supports.

## Measured on the bench since (2026-11-25)

Items 1 to 3 below were tried on `bench/`; the numbers are in bench/RESULTS.md.

- **250 Hz loop (item 1):** 3 % faster with no code change. Predicting the pose over the delay was not tried.
- **Acceleration feed-forward (item 2):** 4 % faster at 50 Hz, 7 % with 250 Hz. Tracking error did not improve,
  and wheel-odometry drift was a centimetre worse, so it is a time gain, not an accuracy one. The follower now
  reports the acceleration (`commandedAcceleration()`); sending it is the robot code's job.
- **Moving goal (item 3):** the bench showed the real fault was not how the path is re-planned but that the goal
  was followed raw. With 2 cm of camera noise the tag align never settled. Averaging the estimates (`bindGoal(goal, 0.5)`,
  opt-in) fixed it, in about 30 lines. The state-to-state generator was therefore **not** built: the existing re-plan plus the
  averaged goal passes every case the bench has, and nothing measured yet says a generator would do better. It
  stays the right idea for a goal that really moves (a rolling piece, a moving robot), which the bench cannot
  test yet.

## What to change, ranked

### 1. Run faster and predict over the delay (strong evidence, small change)

Racing stacks compute control from the state *predicted* one delay ahead, using the commands already sent (Liniger
2015; KIT's Formula Student stack keeps a chain of past inputs in the state). We do half of that: `followPath` reads
the plan `latencySeconds` ahead, but the measured pose is used as it is.

- Predict the pose forward by the latency from the buffered commands before projecting onto the path.
- Run `step()` at the odometry rate (CTRE and 6328 run odometry at 250 Hz on CAN FD) rather than 50 Hz. PREEMPT_RT on
  a Raspberry Pi 5 holds worst-case latency under about 225 µs in one study, enough for a 250 Hz loop.
  `PathFollower.step()` is about 2 µs, so compute is not the limit; `NOMINAL_LOOP_SECONDS = 0.02` and the event
  lead assume 50 Hz and would need to follow the real period.

### 2. Send acceleration, not only velocity (strong by analogy, small change)

The follower returns a velocity (`FieldSpeeds`) and leaves the rest to the modules' velocity loops. Everyone who
measures tracking error feeds the model forward:

- 6328 and PathPlanner pass per-module force as a torque-current feedforward; CTRE's swerve API has
  `withWheelForceFeedforwardsX/Y` for exactly this; 254 added acceleration and `v²κ` terms to their follower.
- On 1:10 race cars a model- and acceleration-aware pursuit controller cut lateral error about four times against pure
  pursuit with no MPC at all (Becker et al., the MAP controller). The gain came from the feedforward and low latency.

The plan already knows the acceleration at every `s` (along the path and `v²κ` across it). Returning it next to the
velocity, and letting the adapter turn it into wheel forces, is the change. There is no swerve-specific measurement of
the benefit, so the bench has to give the number.

### 3. A moving goal wants a trajectory regenerated every loop, not a re-planned path (strong evidence, medium change)

Today a goal that moves 2 cm rebuilds the tail of the path as a new quintic with a new profile (`rebuildTail`), and the
last 0.3 m is the `sqrt(2a(d − vτ))` stopping law. Both communities that chase moving targets do something simpler:

- **RoboCup Small Size League** regenerates a closed-form, near-time-optimal bang-bang trajectory from the robot's
  current state to the target every cycle (Purwin & D'Andrea 2006; TIGERs Mannheim). TIGERs aim at a "virtual
  destination" past the intercept point so the robot arrives on time and still moving.
- **Robot arms** use jerk-limited online trajectory generation to a full target state, including a target velocity
  (Ruckig, Berscheid & Kröger, RSS 2021: about 20 µs mean for 7 axes).
- 6328's `DriveToPose` is the FRC version of the same idea: a profile on the distance to the goal, rebuilt every loop
  from the last setpoint, so the goal is free to move.

What that means here: in free space, drive to a moving goal with a state-to-state generator fed by a *filtered* goal
(position and velocity, timestamped and extrapolated over the camera delay), and arrive at the goal's velocity. Keep
path re-planning (`OnTheFly`) for when geometry matters: obstacles, or a set approach direction. This would also
shrink `PathFollower`: `rebuildTail`, the event re-spacing, and the 2 cm / 2° / 0.15 m thresholds would go. The bench
already has the cases to judge it (`tag_align_moving_goal`, `intake_drive_through`, `h2h.py`'s goal jump); the current
pair of mechanisms passes them, so a replacement has to match that first.

### 4. Estimation: use the fourth module, and handle late camera frames properly (moderate evidence)

- Four modules over-determine a three-number chassis velocity, so the least-squares residual is a free slip detector.
  The standard use is to down-weight the slipping wheel in odometry. (The slip package removed on 2026-10-23 did
  more than this; the residual gate is the small part worth a second look.)
- Camera frames arrive late. 6328 keeps a 2 s pose buffer, rewinds to the frame's timestamp, corrects and replays;
  fixed-lag smoothers do the same more generally. No study showed a factor graph beating a well-built delay-aware
  filter for this sensor set.

### 5. Plan for steering rate (moderate evidence)

A forward-backward pass assumes the wheels point where they need to at once. Steerable-wheel robot papers plan with a
steering-rate limit instead of patching it at run time (Sorour et al.; Connette et al.). In our terms: a cap on how
fast the path's direction may change at low speed (the start of a path and at cusps). Jerk limits make the offline
problem non-convex; the usual answer is to handle jerk in the online layer (item 3).

### 6. Later, and only if 1 to 3 leave something on the table

- **Force allocation by QP** with a friction circle per wheel and load transfer, torque commands to the motors (Tech
  United's platform; the 4WID-4WIS vehicle literature). It is the principled traction control, but it needs vertical
  load and friction estimates, and nothing shows a gain at our scale.
- **MPCC with acados + HPIPM** at 50 to 100 Hz above a fast tracking loop. Proven on cars (about 6.6 ms for 20 steps
  on a Jetson TX2). On a holonomic base that can already push in any direction, the gain over items 1 and 2 is
  unproven, and it would replace the follower rather than simplify it.
- **Iterative learning** for the paths an auto repeats (Kapania & Gerdes 2019). Cheap, but it cannot help once the
  tyres are at their limit.

## Not worth it for a three-second, traction-limited move

Full minimum-time optimal control in place of the two passes; TOPP-RA for its own sake; tube MPC; factor-graph
smoothing in place of a delay-aware filter; jerk-constrained offline planning; image-based visual servoing; learned
residual dynamics before items 1 to 3 are done.

## Where sources disagree

- PathPlanner's setpoint generator says not to discretize chassis speeds before it; 6328 discretizes before its own.
- No published real-robot comparison of time-based against arc-length followers exists; BLine's numbers are its own
  simulation. Our bench is the only head-to-head we have.
- The `sqrt(2ad)` stopping law does not appear in any top team's published code; they use trapezoid profiles. That is
  not evidence against it (ours wins our own race), but it is ours to defend.

## Sources

Read directly (GitHub): Mechanical Advantage `RobotCode2025Public` (`Drive.java`, `DriveTrajectory.java`,
`DriveToPose.java`, `RobotState.java`); Team 254 `FRC-2024-Public` and `FRC-2025-Public` (`AutoAlignToPoseCommand`,
`PathfindingAutoAlignCommand`); PathPlanner (`PathPlannerTrajectory.java`, `PPHolonomicDriveController.java`, the
setpoint generator and pathfinding docs); Choreo (`swerve_trajectory_generator.cpp`, ChoreoLib docs); WPILib
`ChassisSpeeds.java`.

From abstracts or search summaries:
Aguiar, Hespanha & Kokotović, IEEE TAC 50(2), 2005 ·
Lam, Manzie & Good, CDC 2010 ·
Liniger, Domahidi & Morari, OCAM 2015 (arXiv 1711.07300) ·
Becker et al., MAP controller (arXiv 2209.04346) ·
Kapania & Gerdes, ACC 2019 (arXiv 1902.00611) ·
Berscheid & Kröger, Ruckig, RSS 2021 (arXiv 2105.04830) ·
Purwin & D'Andrea, Robotics and Autonomous Systems 54(1), 2006 ·
TIGERs Mannheim extended team description 2023 ·
Heilmeier et al., Vehicle System Dynamics 58(10), 2020 ·
Betz et al., autonomous racing survey (arXiv 2202.07008) ·
Pham & Pham, TOPP-RA, IEEE T-RO 34(3), 2018 (arXiv 1707.07239) ·
Verscheure et al., IEEE TAC 54(10), 2009 ·
Connette et al., CDC 2008 · Sorour et al., Robotics and Autonomous Systems 2019 ·
Lu et al., 4WID-4WIS control allocation (arXiv 2403.12487) ·
Tech United Eindhoven team description papers 2021 and 2023 ·
Verschueren et al., acados (arXiv 1910.13753) · Frison & Diehl, HPIPM (arXiv 2003.02547) ·
acados MPCC on a Jetson TX2 (arXiv 1908.10609) ·
Kabzan et al., AMZ Driverless, JFR 2020 (arXiv 1905.05150) · KIT19d (arXiv 2010.02828) ·
PREEMPT_RT on a Raspberry Pi 5 (arXiv 2604.19275).
