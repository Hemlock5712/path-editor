# To do

Written 2026-11-25 from a cloud session that had no WPILib 2027 install, no JDK 25 and a restricted network. The
items under "Check" are things that session changed but could not compile or run; do those first, on a machine
with the full toolchain. Delete each item when it is done, and this file when it is empty.

## Check (changed, not compiled or run)

- [ ] `lib/gradlew -p lib check`. The WPILib adapters (`lib/src/wpilib`), their tests (`lib/src/wpilibTest`) and
      the end-to-end suites (`lib/src/e2eTest`) were edited by hand when Commands v2 was removed: the two `...V2`
      command classes deleted, the `V2` factories cut from `Pathfinding`, the v2 event bindings cut from
      `PathLibrary`, the `V2` branches cut from `EndToEndRun` and `PathfindEndToEndTest`, and
      `PathfindCommand.handOverCommand()` removed. The build uses `-Xlint:all -Werror`, so a stray unused variable
      there (for example `fw` or `sub` in a lambda) may fail it.
- [ ] `node tools/e2e.mjs`, then commit the regenerated `e2e/RESULTS.md` and `e2e/PATHFIND.md`. Both still show
      V2 rows. Check the counts in `lib/README.md` ("End-to-end" table, "65 of 66") against the new reports.
- [ ] `PerformanceTest.coldStartInAFreshJvm` and `PlannerPerformanceTest.coldInAFreshJvm` on JDK 25. They failed
      in the cloud session only because its JVM prints a "Picked up JAVA_TOOL_OPTIONS" line they try to parse; the
      other 188 core tests passed there on JDK 21.

## Finish the Commands v2 removal

- [ ] Remove the `Framework` enum, now one value (`V3`), from `EndToEndRun`, `PathfindEndToEndTest`,
      `PathEndToEndTest` and `Report` (about 40 uses: constructor arguments, `for (Framework fw : ...)` loops, the
      report's "Framework" column and labels). Left in because it could not be compiled.
- [ ] `PathEndToEndTest.load()` still initialises the HAL and enables the driver station, which only Commands v2
      needed. Remove it if the v3 suite passes without.
- [ ] Decide whether `Pathfinding` (the factory) and `PathfindCommand` should be one file. `OnTheFly` has to stay
      separate: the bench runs it without WPILib.

## Build next

- [ ] **Send the feed-forward.** `PathFollower.commandedAcceleration()` exists and is tested, but
      `FollowPathCommand` and `PathfindCommand` only send a velocity. Give them a second output (acceleration, or
      wheel forces) and use it in the robot code. Bench: 4 % faster at 50 Hz, 7 % at 250 Hz (bench/RESULTS.md).
- [ ] **250 Hz.** The follower uses the real time step, but `PathFollower.NOMINAL_LOOP_SECONDS` (0.02) is still the
      first step's length and the floor of the read-ahead, and the end-to-end `SimDrive` and its tolerances assume
      50 Hz. Run the e2e suite at 250 Hz once the robot code does.
- [ ] **Goal averaging on on-the-fly paths.** `FollowPathCommand.withMovingGoal(goal, seconds)` exists (not
      compiled, see above); `OnTheFly.withMovingGoal` and `PathfindCommand.withMovingGoal` still bind the goal raw.
- [ ] **Bench, feed-forward on the harder courses.** `sim.py` reads `ff_forces` from the follower it is given, and
      the `MovingEnd` and `Chain` wrappers don't pass it on, so `OursV2(ff=True)` has no effect on
      `tag_align_moving_goal`, `intake_drive_through` or `three_piece_chain`.
- [ ] **Bench, a noisy goal as a real course.** The noisy-goal numbers in bench/RESULTS.md came from a throwaway
      script that re-sent the goal with noise every loop. Make it an option of `MovingEnd` so it runs with the rest.
      `PathFollowerTest.aNoisyGoalIsAveragedSoTheRobotStillSettlesOnIt` covers the library side.
- [ ] **Bench, a goal that really moves** (a rolling piece at 0.5 to 1 m/s). The goal average lags a moving goal by
      its speed times 0.5 s. This is the case that would justify the state-to-state generator in RESEARCH.md item 3.
- [ ] **Predict the pose over the latency** (RESEARCH.md item 1, second half). Not tried.
- [ ] **Slip gate** (RESEARCH.md item 4). Waiting for a real robot.
- [ ] The editor's TypeScript follower (`editor/core/src/follow.ts`) has neither the goal average nor
      `commandedAcceleration`. It never had the moving goal, so the sim preview is unaffected; port them only if
      the editor needs them.

## Precision-Drive on this library

Compared 2026-11-25 (Precision-Drive `main` at cce4239 and its `precision-2-DriveToTag` branch). What overlaps and
what does not:

- [ ] **`DriveToPose` could be this library's finish law.** `PathFollower.driveToGoal` is the same law
      (`min(top speed, sqrt(2a(d - v t)), k d)` into the same limiter). A two-waypoint path, or an `OnTheFly` plan,
      already ends in it. Differences to settle first: Precision-Drive brakes at a fixed 6 m/s² with a 6 /s ease-in
      because "braking harder costs tag accuracy"; the library brakes at the traction limit with 12 /s. Bench the
      two settings on `tag_align_moving_goal` with wheel odometry before switching.
- [ ] **`AccelerationLimiter` and `OutputLimiter` are the same idea with two different formulas.** The robot's
      takes `hypot(|a|, alpha x base radius)` and caps the whole acceleration vector by the motors; the library's
      takes `|a| + alpha x I/(m r)` and caps only the part along the travel. Pick one and use it in both
      (teleop needs it too), or have `TeleopDrive` call the library's.
- [ ] **`TagTracker` + `WheelOdometry` have no equivalent here and should stay.** They are the goal supplier:
      `DriveToTag` becomes `otf.to(...)` or a drawn path with `.withMovingGoal(() -> tracker.getFieldGoal(tagToGoal))`
      (raw, no averaging: the tracker already filters and its goal must move with pose corrections).
      `DriveToTag` finishes only once locked; the library has no such gate, so keep that check in the command.
- [ ] **Drive output.** Precision-Drive sends `ApplyFieldVelocity` with `DriveRequestType.Velocity` and voltage
      closed-loop drive gains (kP 0.2, kV 0.124). Sending the feed-forward (above) means adding the wheel forces to
      that request, and probably moving the drive loop to torque current.

## Research to confirm with open web access

The research in RESEARCH.md could not open arxiv.org, Chief Delphi, docs.wpilib.org, pathplanner.dev,
choreo.autos, CTRE's doc sites or ssl.robocup.org. Papers were checked from abstracts and search summaries only.

- [ ] **CTRE Phoenix 6 swerve.** Exact names and units of the wheel force feed-forward inputs
      (`withWheelForceFeedforwardsX/Y` on `ApplyRobotSpeeds` / `ApplyFieldSpeeds`?), whether they need the
      torque-current drive request type, and the odometry rate on Systemcore. Decides whether sending the
      feed-forward is an adapter change or a swerve rewrite.
- [ ] **Systemcore.** Its real CPU, whether the kernel is PREEMPT_RT, and whether WPILib 2027 runs the main loop
      at 250 Hz. RESEARCH.md assumes roughly a Raspberry Pi CM5.
- [ ] **Team practice that could not be sourced:** Team 254's 2024 traction control, Team 1690's skid detection,
      and anything from the 2026 season (6328's `RobotCode2026Public` was not read).
- [ ] **Read in full, not from the abstract:** Purwin & D'Andrea 2006 and the TIGERs Mannheim 2023 team
      description (the bang-bang moving-target method), Berscheid & Kröger 2021 (Ruckig), Becker et al. (the MAP
      controller, source of the "four times lower lateral error" figure), Liniger et al. 2015 (delay prediction).
- [ ] **Three sources with 2026 dates the session could not open at all:** arXiv 2604.19275 (PREEMPT_RT on a
      Raspberry Pi 5), arXiv 2610.12196 (MAP2) and the RSS 2026 swerve MPC paper. Confirm they exist and say what
      RESEARCH.md says they say.
