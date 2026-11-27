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
- [ ] **Feed-forward at the tipping limit.** Sharing the push by weight fixed the extra wheel slip on ordinary
      paths, and on paths planned to 95 % of the tipping limit it is worth 1.4 % where equal shares gave 4 %
      (bench/RESULTS.md, "How the push is shared"). To try: `easeInPerSecond` 24 when the feed-forward is sent
      (a third of the gap); a limiter budget nearer the plan's share while it is sent, so a launch isn't driven
      at 100 % of the limit; and the 250 Hz rows again. The lever that did work is the robot's, not the
      library's: a softer drive kP once the feed-forward is sent (3 % at the limit with less slip; RESULTS.md,
      "A softer drive velocity loop"). Try it on the robot with the feed-forward on, never without.
- [ ] **Teleop feed-forward.** `DriveLimiter.wheelForces` gives it; a scratch run says it cuts wheel slip by a
      third to a half below the limit (RESULTS.md, "Teleop"). It ran the Python port of AccelerationLimiter, not
      `DriveLimiter`: make it a bench course that runs the library's.
- [ ] **WheelForces' assumptions on the real robot** (its class comment lists them): module positions measured
      from the centre of mass, a stiff frame, the lean following the command at once, rotor inertia left out.
      Measure where the centre of mass is, front to back and side to side, not only its height.
- [ ] **Bench, feed-forward on the fly.** The on-the-fly bench op (`otf`) doesn't take `"modules"`, so
      `bench/h2h.py` still runs ours with the velocity alone.
- [ ] **Goal averaging across a re-plan.** `OnTheFly.withMovingGoal(goal, seconds)` starts its average again at
      each re-plan (each plan has a new follower), so the first estimate after one is followed raw.
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

## The robot's side (Precision-Drive, branch `library-drive`, cut from `precision-5-trust` 2026-11-26)

Done there, compiled, unit-tested (37 tests) and run in the CTRE simulator (`./gradlew simulateJavaAgent`), not
on a robot: the library compiled in from source (`build.gradle`, `-PpathLib=`), `deploy/paths/robot.json` (the
numbers from `AccelerationLimiter`), `CenterFrame` (the pose conversion until WPILib moves the origin),
`DriveMechanism.driveFieldVelocity` with the wheel forces and a switch (the driver's Start button; logged as
`Drivetrain/WheelForcesSent`), `TeleopDrive` on `DriveLimiter`, `DriveToPose` and `DriveToTag` on
`Pathfinding`, and `TagTracker` handing its sightings to `TagGoal`.

- [ ] **The finish's ease-in, 12 per second, twitched in the CTRE simulator** with Precision-Drive's (example)
      drive gains: after arriving the robot moved at up to 0.4 m/s and finished 0.6 s late. 8 twitched half as
      much and finished soonest; 6 (the old DriveToPose's) least, but arrives later. Precision-Drive sets 8
      (`DriveMechanism`). 12 was picked on the Python plant, whose velocity loop is stiffer. Tune on the robot,
      and reconsider the library's default with what it shows.
- [ ] **Two `DriveToPose`s in a row lose about a quarter second at the corner** against the old command: the
      library's finish waits for the robot to be under 5 cm/s for 0.1 s, the old one moved on the moment it was
      inside the tolerance. The legs themselves take the same time (1.16 s and 1.13 s for the 1.5 m quarter
      turn). An auto that shouldn't stop at a corner wants one path, or a rolling end, not two commands.
- [ ] **`DriveToPose`'s limit is 9 m/s², both ways** (Joe, 2026-11-26: the same rate speeding up and slowing
      down, and 6 was too safe). `robot.json`'s `onTheFlyMaxMetersPerSecondSquared`; bench/RESULTS.md, "How
      hard to plan", is why 9. In the CTRE simulator "Drive To Pose" (3 m, then 1.5 m with a quarter turn) holds
      its goals at 1.16 s and 2.51 s; the old command (11.2 speeding up, 6 braking) at 1.26 s and 2.48 s, and
      the library at 6 at 1.32 s and 2.70 s. Check on the robot that it still stops on a tag at 9: the old
      comment said braking harder than 6 cost accuracy there. The second leg (a quarter turn in 1.5 m) is still
      0.13 s behind the old one; not looked into.
- [ ] **The library's own default is still 6 m/s²** when a robot file has no `onTheFlyMaxMetersPerSecondSquared`
      (the bench's on-the-fly comparison and the plan fixtures are at 6). The editor may not keep the key when it
      rewrites `robot.json`: check, and add it to the editor's robot settings.
- [ ] **Rails and the intake assist are the library's** (2026-11-26: `follow/RailLaw`, `RailFollower`,
      `PieceAssist`; `wpilib/RailDrive`; lib/README.md, "Rails and the intake assist"). Precision-Drive's `Rail`
      is now only the line and its alliance flip, `PieceAssist` is gone, and `DriveOnRail` and `IntakeAssist`
      hand the sticks to `RailDrive`. Its `RailTest` and `PieceAssistTest` (27 drives, 1 cm tolerances) pass
      unchanged in what they ask, on the library's law with the robot file's limits and an ease-in of 8. Not
      done: run on a robot or in the CTRE simulator (it needs held buttons), and no bench course exercises a
      rail, so its wheel slip has never been measured.
- [ ] **Set the rail's pull-in with a driver** (built 2026-11-27; lib/README.md, "How much the rail takes from
      the driver"). Precision-Drive's `DriveTuning.RAIL_PULL_IN` is 0.5, a starting point with nothing measured
      behind it. Also mine and unreviewed by a driver: the 0.5 m/s it still settles at with the stick released
      (`RailLaw.SETTLE_METERS_PER_SECOND`), and that a push away from the line is ignored. The pull still comes
      on in one or two loops (the limiter's rate), now to a much lower speed.
- [ ] **The piece nudge comes on in one or two loops** (the limiter's rate, 12.8 m/s², though only to about 0.4
      m/s for a piece 25 cm off). Not smoothed: ask Joe whether drivers feel it before adding anything.
- [ ] **The piece filter's numbers are guesses to measure on the robot** (`follow/PieceFilter`, done
      2026-11-26; bench/RESULTS.md, "Game pieces"): the camera's angle noise (0.3°: watch a still piece and take
      the spread), and whether the range is off the same way photo after photo (a wrong piece height or camera
      tilt; the filter allows half a metre for it). Two pieces in one line of sight less than half a metre apart
      are taken for one. Precision-Drive's `PieceTracker` hands its photos to it.
- [ ] **`build.gradle`'s library path** points into this repository's worktree
      (`../path-editor/.claude/worktrees/pensive-einstein/lib`). Once this branch is merged it becomes
      `../path-editor/lib`.
- [ ] **On the robot, in this order:** the forces do something with the voltage drive gains (Start flips them;
      compare the drive motors' output); wheel slip and odometry drift with them on and off; a lower drive kP with
      them on (bench/RESULTS.md, "A softer drive velocity loop").
- [ ] **The library's telemetry is in its own frame.** `Path/<name>/Reference`, `Goal` and `Plan` are written from
      the middle of the field. On alpha-7, where the robot's pose is from the blue corner, they land mirrored
      against it in AdvantageScope, so a student can't overlay the plan on the robot. Convert them on the way out
      while `CenterFrame` exists, or leave it until WPILib moves the origin. `Plan` is also a plain list of
      numbers, which AdvantageScope doesn't draw. (Found reviewing lesson 2's script, 2026-11-27.)
- [ ] **Teleop on red.** `TeleopDrive` now turns the sticks into field directions itself and sends
      `ApplyFieldVelocity`; it was `FieldCentric` with CTRE's operator perspective. Checked by reading, not run
      as red in the simulator.

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
        to the first WPILib release with allwpilib PR 9590, and until then don't drive without converting
        everything that crosses between the robot and the library, from the old corner frame: every pose in
        (the measured pose, a moving goal, a `toTarget` goal) as `(L/2 - x, W/2 - y, heading + 180°)`, and every
        velocity, in and out, as `(-vx, -vy, omega)`. Converting the pose alone drives the robot away from
        the path.
      - Precision-Drive's `AllianceFlip` and its "blue corner" comments need the same update.
      - The end-to-end authoring spec (`editor/app/e2e-author/author.spec.ts`) and the editor's UI tests still
        draw their paths at x < 0, now red's half. They don't depend on the side; move them if that confuses.
The library's side of the next three is done (2026-11-26; lib/README.md, "Driving to a pose, and to a tag";
`DriveToPoseTest` drives each one). What is left is in Precision-Drive, which has not been touched:

- [ ] **One limiter: the robot's side.** `TeleopDrive` uses `DriveLimiter` (the library's `OutputLimiter`, now
      public) built from `robot.json`; retire `AccelerationLimiter`. Its `MODEL_CURRENT` (150 A) becomes
      `statorCurrentLimitAmps`; its hand-calibration notes for mu and the centre of mass height belong in the
      robot file's documentation. `DriveLimiter` drives to the planned traction limit (the 0.9 share), as
      `AccelerationLimiter` did.
- [ ] **One drive-to-pose: the robot's side.** Replace `DriveToPose` with `otf.to(...)` / `otf.toTarget(...)`
      and `.withTolerance(...)`; `DriveOnRail` and `IntakeAssist` extend or copy `DriveToPose` and were not read.
- [ ] **Tag tracking: the robot's side.** `TagTracker` keeps the Limelight reading, `CameraCorrections`, the
      frame-age and ambiguity checks, `WheelOdometry` and its logging, and hands each sighting to a `TagGoal`.
      `DriveToTag` becomes `otf.toTarget(...).withMovingGoal(...).finishWhen(tagGoal::hasLock)`.
      The filter's noise numbers are Precision-Drive's at `precision-2-DriveToTag`; `precision-5-trust` measures
      them per camera (`CameraNoise`, `NoiseFit`) and that has not been brought over.
- [ ] **Lessons 3 to 5** (`Rail`, `DriveOnRail`, `IntakeAssist`, `PieceTracker`, `PieceAssist`, the camera trust
      work) were written after the comparison above and have not been compared with the library at all.
- [ ] **Drive output.** Precision-Drive sends `ApplyFieldVelocity` with `DriveRequestType.Velocity` and voltage
      drive gains (kP 0.2, kV 0.124). Sending the feed-forward means adding
      `.withWheelForceFeedforwardsX/Y(double[])` to that request: newtons, field-relative, one per module in the
      drivetrain's order (read in the Phoenix 6 26.70.0-alpha-2 source). Open:
      - Whether the forces are used with voltage drive gains at all. The module code is native in this version
        and could not be read. Test on the robot: log the drive motors' output with the forces on and off.
      - Torque current is `kDriveClosedLoopOutput = TorqueCurrentFOC` in `TunerConstants` with the drive gains
        retuned in amps; the request stays `Velocity`. It needs Phoenix Pro. The bench has not simulated it.
      - CTRE's comment says the forces "should include friction". Ours don't: check the drive kS covers it.
- [ ] **Packaging: the vendordep is published** (2027.0.0-alpha-2, 2026-11-28, GitHub Pages from the `gh-pages`
      branch; lib/README.md, "The vendordep"), and Precision-Drive's `library-drive` builds from
      `vendordeps/Myelin.json` (`-PpathLib=<lib folder>` still builds it from source while changing the
      library). Publishing a new version is by hand. Not checked: whether a vendordep can say it needs
      Commands v3 (GradleRIO alpha-7's handling of a `requires` entry), and installing it through the WPILib
      VS Code "Manage Vendor Libraries" command rather than by copying the file.
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
