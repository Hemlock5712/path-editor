# To do

Written 2026-11-25 from a cloud session that had no WPILib 2027 install, no JDK 25 and a restricted network. Its
changes were compiled and run on 2026-11-25 with WPILib 2027 alpha-7 and JDK 25 (`check` and the end-to-end suite
pass). Delete each item when it is done, and this file when it is empty.

## Finish the Commands v2 removal

- [ ] Decide whether `Pathfinding` (the factory) and `PathfindCommand` should be one file. `OnTheFly` has to stay
      separate: the bench runs it without WPILib.

## Build next

- [ ] **Send the feed-forward: the robot's side.** The library's side is done (2026-11-26):
      `Pathfinding.withWheelForces(modules, drive)` and `FollowPathCommand.withWheelForces` send each wheel's push
      with the velocity (lib/README.md, "Feed-forward"). Still to do in the robot code: a three-argument
      `driveFieldVelocity` on the drivetrain, and a check on the real robot that the forces do something (see
      "Drive output" below). Bench: 4 % faster at 50 Hz, 7 % at 250 Hz (bench/RESULTS.md).
- [ ] **250 Hz.** The follower uses the real time step, but `PathFollower.NOMINAL_LOOP_SECONDS` (0.02) is still the
      first step's length and the floor of the read-ahead, and the end-to-end `SimDrive` and its tolerances assume
      50 Hz. Run the e2e suite at 250 Hz once the robot code does.
- [ ] **Goal averaging on on-the-fly paths.** `FollowPathCommand.withMovingGoal(goal, seconds)` exists (not
      compiled, see above); `OnTheFly.withMovingGoal` and `PathfindCommand.withMovingGoal` still bind the goal raw.
- [ ] **Bench, feed-forward on the harder courses.** `sim.py` reads `ff_forces` from the follower it is given, and
      the `MovingEnd` and `Chain` wrappers don't pass it on, so `OursV2(ff=True)` has no effect on
      `tag_align_moving_goal`, `intake_drive_through` or `three_piece_chain`. The on-the-fly bench op (`otf`)
      doesn't take `"modules"` either.
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

## Bring Precision-Drive's parts into this library

Decided 2026-11-25: the library does everything about moving the robot; robot code keeps the hardware, the
cameras and the driver. Precision-Drive's pieces move here as they are needed, and the Swerve School videos that
show its code get redone to match. Both were read at Precision-Drive `main` cce4239 and `precision-2-DriveToTag`.

Where the two repos do the same job, each difference was checked on the bench (12 matched courses, 6 seeds):

| Same job | Precision-Drive | This library | Which is right |
|---|---|---|---|
| Field origin and red flip | blue corner; `(L - x, W - y, heading + 180)` | centre; `(-x, -y, heading + 180)` | **The library's origin, for WPILib 2027** (allwpilib PR 9590, merged 2026-11-07: centre origin, +X towards blue, which the library and editor now use) |
| Motor limit when speeding up | scales the whole acceleration vector | caps only the part along the travel | **The library's.** Precision-Drive's rule: 2 % slower, cross-track 5.6 cm against 4.1 |
| Sharing grip between driving and spin | `hypot(a, alpha x base radius)` | `a + alpha x I / (m r)` | **Either.** No measurable difference (31.72 s against 31.83 s, same slip). Keep the library's: it is the worst wheel's case and the planner uses the same one |
| Finish: brake and ease-in | 6 m/s² and 6 /s | the traction limit and 12 /s | **The library's on a path planned to the limit** (Precision-Drive's is 14 % slower to settle on the straight and no more accurate). On a move planned at 6 m/s², as `DriveToPose` always is, Precision-Drive's has no overshoot and a third of the wheel slip: set the path's acceleration, not the finish law |
| A stray goal estimate | ignored unless 3 in a row agree | restarted on the first | **Precision-Drive's.** Adopted in `PathFollower`'s goal average |
| Filtering a tag's position | a Kalman filter per axis, noise growing with distance, in the odometry frame | an equal-weight average | **Precision-Drive's.** The library's average is only for a supplier with no filter of its own |
| Finished | in tolerance and slower than 5 cm/s, at once; 2° | the same held for 0.1 s; 1° | **The library's** (the hold stops a robot passing through the tolerance from finishing) |

What to do:

- [ ] **Field direction: the robot's side.** The editor, the fixtures and the library are in the WPILib 2027
      frame (centre origin, +X towards blue) since 2026-11-26, and older files are turned when read (lib/README.md,
      "The field frame and older files"). Still to do:
      - The build is on 2027.0.0-alpha-7, which has the old corner origin, as does alpha-8. Move `lib/build.gradle`
        to the first WPILib release with allwpilib PR 9590, and until then don't drive without converting the
        pose: `(L/2 - x, W/2 - y, heading + 180°)` from the old corner frame.
      - Precision-Drive's `AllianceFlip` and its "blue corner" comments need the same update.
      - The end-to-end authoring spec (`editor/app/e2e-author/author.spec.ts`) and the editor's UI tests still
        draw their paths at x < 0, now red's half. They don't depend on the side; move them if that confuses.
- [ ] **One limiter.** Make the library's `OutputLimiter` public (it is package-private) with a
      `limit(current, wanted, dt)` for `TeleopDrive`, built from `robot.json`. Retire `AccelerationLimiter`.
      Its `MODEL_CURRENT` (150 A) becomes `statorCurrentLimitAmps`; its hand-calibration notes for mu and the
      centre of mass height belong in the robot file's documentation.
- [ ] **One drive-to-pose.** A command that runs the follower's finish law from anywhere (`Pathfinding.to` already
      does on an empty field). Retire `DriveToPose`; keep its settable tolerance (`withTolerance`).
- [ ] **Tag tracking: the filter here, the camera there.** Move `TagTracker`'s filter (per-axis Kalman, jump
      rule, lock after 3 frames, the odometry-frame goal) into the library with no Limelight in it: in goes "tag
      seen at this pose, this far away, with the robot's wheel-odometry pose at capture", out comes the goal.
      The Limelight reading, `CameraCorrections`, frame age and `WheelOdometry` stay in robot code.
      `DriveToTag` becomes `.withMovingGoal(() -> tracker.getFieldGoal(tagToGoal))` (raw, no averaging).
- [ ] **A lock gate.** `DriveToTag` finishes only once the tag is locked. Give the follow commands a way to say
      "don't finish until this is true".
- [ ] **Drive output.** Precision-Drive sends `ApplyFieldVelocity` with `DriveRequestType.Velocity` and voltage
      drive gains (kP 0.2, kV 0.124). Sending the feed-forward means adding
      `.withWheelForceFeedforwardsX/Y(double[])` to that request: newtons, field-relative, one per module in the
      drivetrain's order (read in the Phoenix 6 26.70.0-alpha-2 source). Open:
      - Whether the forces are used with voltage drive gains at all. The module code is native in this version
        and could not be read. Test on the robot: log the drive motors' output with the forces on and off.
      - Torque current is `kDriveClosedLoopOutput = TorqueCurrentFOC` in `TunerConstants` with the drive gains
        retuned in amps; the request stays `Velocity`. It needs Phoenix Pro. The bench has not simulated it.
      - CTRE's comment says the forces "should include friction". Ours don't: check the drive kS covers it.
- [ ] **Packaging.** Until the first event, the robot project includes `lib/` as source (a Gradle composite build
      or a submodule). After it, publish a vendordep: `lib/build.gradle` has no publishing today, and the WPILib
      2027 vendordep format was not checked.
- [ ] **Swerve School.** The videos copy their on-screen code from Precision-Drive (`precision-1-DriveToPose`,
      `precision-2-DriveToTag`). List which chapters show `AccelerationLimiter`, `DriveToPose`, `AllianceFlip` and
      `TagTracker`, and redo those once the code above has moved.

## Research to confirm with open web access

The research in RESEARCH.md could not open arxiv.org, Chief Delphi, docs.wpilib.org, pathplanner.dev,
choreo.autos, CTRE's doc sites or ssl.robocup.org. Papers were checked from abstracts and search summaries only.

- [ ] **CTRE Phoenix 6 swerve.** The odometry rate on Systemcore, and how the native module code applies the
      wheel force feed-forward with voltage and with torque-current drive gains. (The names and units are settled:
      see "Drive output" above. It was an adapter change.)
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
