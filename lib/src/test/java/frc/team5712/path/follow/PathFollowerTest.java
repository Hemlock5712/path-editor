package frc.team5712.path.follow;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.Angles;
import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.GeometrySample;
import frc.team5712.path.core.HeadingProfile;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathEvent;
import frc.team5712.path.core.PathGeometry;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.PathSpec;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.RotationTarget;
import frc.team5712.path.core.VelocityProfile;
import frc.team5712.path.core.Waypoint;
import frc.team5712.path.tools.FixturePaths;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

class PathFollowerTest {
  static final RobotConfig ROBOT = RobotConfig.DEFAULT;
  static final FollowerConfig CFG = FollowerConfig.defaults(ROBOT);
  static final double DT = 0.02;

  /** Everything a closed-loop run produced. */
  record Run(List<Pose> trace, double seconds, boolean finished, double worstCrossTrack, PathFollower follower) {}

  /** Starts at the path's start pose (in {@code frame}) and steps until finished or out of time. */
  static Run run(Path path, PathFollower f, FieldFrame frame, double maxSeconds, Hook hook) {
    PathSample start = path.sample(0);
    Pose startPose = frame.fromBlue(new Pose(start.xMeters(), start.yMeters(), start.headingRadians()));
    KinematicPlant plant = new KinematicPlant(startPose, new FieldSpeeds(0, 0, 0));
    f.start(0.0, plant.pose(), plant.velocity(), frame);
    List<Pose> trace = new ArrayList<>();
    double worst = 0;
    double t = 0;
    while (t < maxSeconds && !f.isFinished()) {
      t += DT;
      hook.before(t, plant, f);
      FieldSpeeds cmd = f.step(t, plant.pose(), plant.velocity());
      worst = Math.max(worst, Math.abs(f.crossTrackMeters()));
      plant.step(cmd, DT);
      trace.add(plant.pose());
    }
    return new Run(trace, t, f.isFinished(), worst, f);
  }

  interface Hook {
    void before(double t, KinematicPlant plant, PathFollower f);
  }

  static final Hook NONE = (t, p, f) -> {};

  static Run run(PathSpec spec) {
    Path path = Path.build(spec, ROBOT);
    return run(path, new PathFollower(path, CFG), FieldFrame.BLUE, 20, NONE);
  }

  // ------------------------------------------------------------------------------ closed loop

  @Test
  void drivesEveryFixturePathAndStopsOnTheGoal() {
    for (var e : FixturePaths.all().entrySet()) {
      PathSpec spec = e.getValue();
      if (spec.endMetersPerSecond() > 0) {
        spec = spec.withEndSpeeds(spec.startMetersPerSecond(), 0.0);
      }
      Path path = Path.build(spec, ROBOT);
      Run r = run(path, new PathFollower(path, CFG), FieldFrame.BLUE, 20, NONE);
      assertTrue(r.finished(), e.getKey() + " did not finish");
      Pose end = r.trace().get(r.trace().size() - 1);
      PathSample goal = path.sample(path.lengthMeters());
      double miss = Math.hypot(end.xMeters() - goal.xMeters(), end.yMeters() - goal.yMeters());
      assertTrue(miss < CFG.positionToleranceMeters(), e.getKey() + " ends " + miss + " m off");
      assertTrue(Math.abs(Angles.wrap(end.headingRadians() - goal.headingRadians())) < CFG.headingToleranceRadians());
      assertTrue(r.worstCrossTrack() < 0.05, e.getKey() + " strays " + r.worstCrossTrack() + " m");
      // finishing takes the plan plus the settle, not much more
      assertTrue(r.seconds() < path.totalTimeSeconds() + 0.8,
          e.getKey() + " took " + r.seconds() + " s for a " + path.totalTimeSeconds() + " s plan");
    }
  }

  @Test
  void noLatencyStillStartsAndFinishes() {
    // latency 0 used to read the plan where the robot stands, 0 m/s at the start, so it never moved
    FollowerConfig noLead = CFG.toBuilder().latencySeconds(0.0).build();
    for (PathSpec spec : List.of(FixturePaths.straight(), FixturePaths.sCurve())) {
      Path path = Path.build(spec, ROBOT);
      Run r = run(path, new PathFollower(path, noLead), FieldFrame.BLUE, 20, NONE);
      assertTrue(r.finished(), "did not finish");
      Pose end = r.trace().get(r.trace().size() - 1);
      PathSample goal = path.sample(path.lengthMeters());
      assertTrue(Math.hypot(end.xMeters() - goal.xMeters(), end.yMeters() - goal.yMeters()) < CFG.positionToleranceMeters());
    }
  }

  @Test
  void reportsHowItTracksThePlan() {
    Path path = Path.build(FixturePaths.sCurve(), ROBOT);
    PathFollower follower = new PathFollower(path, CFG);
    assertTrue(Double.isNaN(follower.alongTrackMeters()), "nothing before the start");
    double[] worst = new double[3];
    List<Double> along = new ArrayList<>();
    Run r = run(path, follower, FieldFrame.BLUE, 20, (t, plant, f) -> {
      if (f.phase() != PathFollower.Phase.TRACKING || Double.isNaN(f.headingErrorRadians())) {
        return; // (before the first step there is nothing to report yet)
      }
      along.add(f.alongTrackMeters());
      worst[0] = Math.max(worst[0], Math.abs(f.headingErrorRadians()));
      worst[1] = Math.max(worst[1], Math.abs(f.measuredMetersPerSecond() - f.plannedMetersPerSecond()));
      worst[2] = Math.max(worst[2], f.plannedMetersPerSecond());
      assertEquals(f.elapsedSeconds(), t, 0.05);
    });
    assertTrue(r.finished());
    assertTrue(along.size() > 50);
    // it keeps the plan's heading and speed closely, and stays within a stride of the plan's schedule
    assertTrue(worst[0] < 0.05, "heading error " + worst[0]);
    assertTrue(worst[1] < 0.8, "speed error " + worst[1]);
    assertTrue(worst[2] <= FixturePaths.sCurve().maxMetersPerSecond() + 1e-9);
    assertTrue(along.stream().allMatch(a -> Math.abs(a) < 0.6), "along-track " + along);
  }

  @Test
  void recoversFromAShove() {
    Path path = Path.build(FixturePaths.sCurve(), ROBOT);
    Run r = run(path, new PathFollower(path, CFG), FieldFrame.BLUE, 20,
        (t, plant, f) -> {
          if (Math.abs(t - 1.0) < 1e-9) {
            plant.displace(0.0, 0.4); // 40 cm sideways in one loop
          }
        });
    assertTrue(r.finished());
    assertTrue(r.worstCrossTrack() > 0.2, "the shove happened");
    Pose end = r.trace().get(r.trace().size() - 1);
    assertEquals(4.0, end.xMeters(), 0.02);
    assertEquals(1.0, end.yMeters(), 0.02);
  }

  @Test
  void knockedFarOffItBuildsAPieceBackOntoThePath() {
    Path path = Path.build(FixturePaths.sCurve(), ROBOT);
    List<String> fired = new ArrayList<>();
    PathFollower follower = new PathFollower(path, CFG).addEvent("half", path.lengthMeters() / 2, () -> {});
    follower.onEvent(fired::add);
    Run r = run(path, follower, FieldFrame.BLUE, 20,
        (t, plant, f) -> {
          if (Math.abs(t - 1.0) < 1e-9) {
            plant.displace(0.0, 0.7); // 70 cm sideways in one loop
          }
        });
    assertTrue(r.finished());
    assertTrue(follower.rejoinCount() >= 1, "rejoined");
    assertTrue(follower.lastRejoinMicros() > 0.0);
    Pose end = r.trace().get(r.trace().size() - 1);
    assertEquals(4.0, end.xMeters(), 0.02);
    assertEquals(1.0, end.yMeters(), 0.02);
    assertEquals(List.of("half"), fired, "the event fires once, on the way");

    // switched off, the same knock is recovered by the cross-track law alone
    PathFollower plain = new PathFollower(path, CFG.toBuilder().rejoinAboveMeters(Double.POSITIVE_INFINITY).build());
    Run r2 = run(path, plain, FieldFrame.BLUE, 20,
        (t, plant, f) -> {
          if (Math.abs(t - 1.0) < 1e-9) {
            plant.displace(0.0, 0.7);
          }
        });
    assertTrue(r2.finished());
    assertEquals(0, plain.rejoinCount());
  }

  @Test
  void aRejoinNeverSkipsACusp() {
    // out to x = 2, back and up: knocked sideways on the way out, the robot must still go out to the tip
    PathSpec spec = PathSpec.of(List.of(Waypoint.at(0.0, 0.0, 0.0), Waypoint.at(2.0, 0.0, 0.0),
            Waypoint.at(1.2, 0.05, Math.PI), Waypoint.at(0.0, 0.6)))
        .withRotationTargets(List.of(RotationTarget.atWaypoint(0, 0.0), RotationTarget.atWaypoint(3, 0.0)))
        .withLimits(4.0, 6.0);
    Path path = Path.build(spec, ROBOT);
    PathSample tip = path.sample(path.cusps()[0]);
    PathFollower follower = new PathFollower(path, CFG);
    Run r = run(path, follower, FieldFrame.BLUE, 20,
        (t, plant, f) -> {
          if (Math.abs(t - 0.4) < 1e-9) {
            plant.displace(0.0, -0.7);
          }
        });
    assertTrue(r.finished());
    assertTrue(follower.rejoinCount() >= 1, "rejoined");
    double furthest = Double.NEGATIVE_INFINITY;
    for (Pose q : r.trace()) {
      furthest = Math.max(furthest, q.xMeters() - tip.xMeters());
    }
    assertTrue(furthest > -0.1, "got to the tip: " + furthest);
    Pose end = r.trace().get(r.trace().size() - 1);
    assertEquals(0.0, end.xMeters(), 0.02);
    assertEquals(0.6, end.yMeters(), 0.02);
  }

  @Test
  void movingAtTheStartReplansOnlyTheSpeeds() {
    Path path = Path.build(FixturePaths.sCurve(), ROBOT);
    PathSample s0 = path.sample(0);
    Pose p = new Pose(s0.xMeters(), s0.yMeters(), s0.headingRadians());
    PathFollower f = new PathFollower(path, CFG.toBuilder().replanStartFromMeasuredVelocity(true).build());
    long geometries = PathGeometry.buildCount();
    f.start(0.0, p, new FieldSpeeds(1.0, 1.5, 0.0), FieldFrame.BLUE);
    assertEquals(geometries, PathGeometry.buildCount(), "the curve stays as drawn");
    assertTrue(f.activePath().geometry() == path.geometry());
    assertTrue(f.activePath().sample(0).metersPerSecond() > 1.0, "planned from the measured speed");
    assertTrue(f.lastStartReplanMicros() > 0.0);
    // moving backwards along the path: from rest, as without the option
    f.start(0.0, p, new FieldSpeeds(-1.0, 1.5, 0.0), FieldFrame.BLUE);
    assertTrue(f.activePath() == path);
    assertTrue(Double.isNaN(f.lastStartReplanMicros()));
    PathFollower off = new PathFollower(path, CFG.toBuilder().replanStartFromMeasuredVelocity(false).build());
    off.start(0.0, p, new FieldSpeeds(1.0, 1.5, 0.0), FieldFrame.BLUE);
    assertTrue(off.activePath() == path, "switched off");
    // on by default, and a no-op at rest
    PathFollower byDefault = new PathFollower(path, CFG);
    byDefault.start(0.0, p, new FieldSpeeds(1.0, 1.5, 0.0), FieldFrame.BLUE);
    assertTrue(byDefault.activePath() != path, "on by default");
    byDefault.start(0.0, p, new FieldSpeeds(0, 0, 0), FieldFrame.BLUE);
    assertTrue(byDefault.activePath() == path, "nothing to do at rest");
  }

  @Test
  void aPathThatPlansItsOwnStartSpeedIsLeftAlone() {
    // tangent scales starts at 1.2 m/s: a rolling hand-over or an on-the-fly path already plans the speed
    Path path = Path.build(FixturePaths.tangentScales(), ROBOT);
    PathSample s0 = path.sample(0);
    PathFollower f = new PathFollower(path, CFG);
    f.start(0.0, new Pose(s0.xMeters(), s0.yMeters(), s0.headingRadians()),
        new FieldSpeeds(1.3 * s0.tangentX(), 1.3 * s0.tangentY(), 0.0), FieldFrame.BLUE);
    assertTrue(f.activePath() == path);
    assertTrue(Double.isNaN(f.lastStartReplanMicros()));
  }

  @Test
  void coastingFromTeleopTheRePlannedStartArrivesSooner() {
    // driven in teleop at 2 m/s along the path, then the auto command starts a path planned from rest
    Path path = Path.build(FixturePaths.sCurve(), ROBOT);
    PathSample s0 = path.sample(0);
    FieldSpeeds coasting = new FieldSpeeds(2.0 * s0.tangentX(), 2.0 * s0.tangentY(), 0.0);
    double[] seconds = new double[2];
    double[] worst = new double[2];
    for (int k = 0; k < 2; k++) {
      PathFollower f = new PathFollower(path, CFG.toBuilder().replanStartFromMeasuredVelocity(k == 1).build());
      KinematicPlant plant = new KinematicPlant(new Pose(s0.xMeters(), s0.yMeters(), s0.headingRadians()), coasting);
      f.start(0.0, plant.pose(), plant.velocity(), FieldFrame.BLUE);
      double t = 0;
      while (t < 20 && !f.isFinished()) {
        t += DT;
        FieldSpeeds cmd = f.step(t, plant.pose(), plant.velocity());
        worst[k] = Math.max(worst[k], Math.abs(f.crossTrackMeters()));
        plant.step(cmd, DT);
      }
      assertTrue(f.isFinished());
      seconds[k] = t;
    }
    System.out.printf("coasting at 2 m/s into a from-rest S-curve: %.2f s planned from rest, %.2f s re-planned "
        + "(worst cross-track %.3f / %.3f m)%n", seconds[0], seconds[1], worst[0], worst[1]);
    assertTrue(seconds[1] < seconds[0], "re-planned " + seconds[1] + " s, from rest " + seconds[0] + " s");
  }

  @Test
  void replanningTheStartIsWellUnderAMillisecondWarm() {
    Path path = Path.build(FixturePaths.autoRoute().withEndSpeeds(0, 0), ROBOT);
    PathFollower f = new PathFollower(path, CFG.toBuilder().replanStartFromMeasuredVelocity(true).build());
    PathSample s0 = path.sample(0);
    Pose p = new Pose(s0.xMeters(), s0.yMeters(), s0.headingRadians());
    FieldSpeeds v = new FieldSpeeds(2.0 * s0.tangentX(), 2.0 * s0.tangentY(), 0.0);
    double best = Double.POSITIVE_INFINITY;
    double total = 0;
    int n = 0;
    for (int rep = 0; rep < 400; rep++) {
      f.start(0, p, v, FieldFrame.BLUE);
      if (rep >= 200) {
        best = Math.min(best, f.lastStartReplanMicros());
        total += f.lastStartReplanMicros();
        n++;
      }
    }
    System.out.printf("start re-plan of the %.1f m route from 2 m/s, warm: mean %.0f us, best %.0f us%n",
        path.lengthMeters(), total / n, best);
    assertTrue(total / n < 1000.0, "mean start re-plan " + total / n + " us");
  }

  @Test
  void aBigShoveReprojectsGlobally() {
    Path path = Path.build(FixturePaths.hairpin(), ROBOT);
    // hairpin legs at y = -1 and y = +1: carried from the first leg to the second, the robot is ahead
    Run r = run(path, new PathFollower(path, CFG), FieldFrame.BLUE, 20,
        (t, plant, f) -> {
          if (Math.abs(t - 0.4) < 1e-9) {
            plant.displace(0.0, 2.0);
          }
        });
    assertTrue(r.finished());
  }

  @Test
  void terminalPhaseTakesOverNearTheEnd() {
    Path path = Path.build(FixturePaths.straight(), ROBOT);
    List<PathFollower.Phase> phases = new ArrayList<>();
    run(path, new PathFollower(path, CFG), FieldFrame.BLUE, 10, (t, p, f) -> phases.add(f.phase()));
    int firstTerminal = phases.indexOf(PathFollower.Phase.TERMINAL);
    assertTrue(firstTerminal > 0);
    assertTrue(phases.subList(firstTerminal, phases.size()).stream().allMatch(p -> p == PathFollower.Phase.TERMINAL));
  }

  @Test
  void aLoopStartsAtItsStartNotItsEnd() {
    List<Waypoint> loop = new ArrayList<>();
    for (int i = 0; i <= 8; i++) {
      double a = i * Math.PI / 4;
      loop.add(Waypoint.at(Math.cos(a) * 1.5, Math.sin(a) * 1.5));
    }
    Path path = Path.build(PathSpec.of(loop).withLimits(3, 5), ROBOT);
    PathFollower f = new PathFollower(path, CFG);
    f.start(0, new Pose(1.5, 0, 0), new FieldSpeeds(0, 0, 0), FieldFrame.BLUE);
    assertTrue(f.progressMeters() < 0.1, "starts at s = " + f.progressMeters());
  }

  @Test
  void noPoseMeansBrakeNotCrash() {
    Path path = Path.build(FixturePaths.straight(), ROBOT);
    PathFollower f = new PathFollower(path, CFG);
    f.start(0, new Pose(-1.5, 0, 0), new FieldSpeeds(2, 0, 0), FieldFrame.BLUE);
    FieldSpeeds out = f.step(0.02, new Pose(Double.NaN, 0, 0), new FieldSpeeds(2, 0, 0));
    assertTrue(out.vxMetersPerSecond() < 2 && out.vxMetersPerSecond() > 0, "brakes within the limits");
  }

  @Test
  void stepBeforeStartIsAnError() {
    Path path = Path.build(FixturePaths.straight(), ROBOT);
    assertThrows(IllegalStateException.class,
        () -> new PathFollower(path, CFG).step(0, new Pose(0, 0, 0), new FieldSpeeds(0, 0, 0)));
  }

  // ------------------------------------------------------------------------------ zero path work

  @ParameterizedTest
  @EnumSource(FieldFrame.class)
  void startingDoesNoPathWork(FieldFrame frame) {
    Path path = Path.build(FixturePaths.autoRoute(), ROBOT); // robot init
    PathFollower f = new PathFollower(path, CFG);
    long geometries = PathGeometry.buildCount();
    long profiles = VelocityProfile.buildCount();
    // auto begins: start and run a second of steps, on either alliance
    PathSample s0 = path.sample(0);
    Pose start = frame.fromBlue(new Pose(s0.xMeters(), s0.yMeters(), s0.headingRadians()));
    f.start(0.0, start, new FieldSpeeds(0, 0, 0), frame);
    for (int i = 1; i <= 50; i++) {
      f.step(i * DT, start, new FieldSpeeds(0, 0, 0));
    }
    assertEquals(geometries, PathGeometry.buildCount(), "no geometry built");
    assertEquals(profiles, VelocityProfile.buildCount(), "no profile built");
    assertTrue(f.activePath() == path, "follows the very object built at init");
  }

  // ------------------------------------------------------------------------------ flip

  @ParameterizedTest
  @EnumSource(value = FieldFrame.class, names = {"RED_HALF_TURN", "RED_MIRROR"})
  void onRedTheRunIsTheFlippedBlueRun(FieldFrame frame) {
    Path path = Path.build(FixturePaths.autoRoute().withEndSpeeds(0, 0), ROBOT);
    Run blue = run(path, new PathFollower(path, CFG), FieldFrame.BLUE, 20, NONE);
    Run red = run(path, new PathFollower(path, CFG), frame, 20, NONE);
    assertEquals(blue.trace().size(), red.trace().size());
    for (int i = 0; i < blue.trace().size(); i++) {
      Pose expected = frame.fromBlue(blue.trace().get(i));
      Pose got = red.trace().get(i);
      assertEquals(expected.xMeters(), got.xMeters(), 1e-9);
      assertEquals(expected.yMeters(), got.yMeters(), 1e-9);
      assertEquals(0.0, Angles.wrap(expected.headingRadians() - got.headingRadians()), 1e-9);
    }
  }

  // ------------------------------------------------------------------------------ events

  @Test
  void eventsFireInOrderAsTheyArePassed() {
    Path path = Path.build(FixturePaths.sCurve(), ROBOT);
    PathFollower f = new PathFollower(path, CFG);
    List<String> fired = new ArrayList<>();
    List<Double> where = new ArrayList<>();
    f.addEvent("c", 6.0, () -> where.add(f.progressMeters()))
        .addEvent("a", 1.0, () -> where.add(f.progressMeters()))
        .addEventAtWaypoint("b", 1.5, () -> where.add(f.progressMeters()))
        .onEvent(fired::add);
    run(path, f, FieldFrame.BLUE, 20, NONE);
    assertEquals(List.of("a", "b", "c"), fired);
    // on the loop nearest to passing: within half a loop of travel (4 m/s · 10 ms = 4 cm), either side
    assertEquals(1.0, where.get(0), 0.045, "fired as it passed, at " + where.get(0));
    assertEquals(6.0, where.get(2), 0.045, "fired as it passed, at " + where.get(2));
  }

  @Test
  void pathFileEventsFireWithCodeEvents() {
    PathSpec spec = FixturePaths.sCurve()
        .withEvents(List.of(PathEvent.atMeters("fileLate", 6.0), PathEvent.atWaypoint("fileEarly", 0.5)));
    Path path = Path.build(spec, ROBOT);
    PathFollower f = new PathFollower(path, CFG);
    assertEquals(List.of("fileEarly", "fileLate"), f.eventNames(), "file events are registered, in path order");
    List<String> fired = new ArrayList<>();
    List<Double> where = new ArrayList<>();
    f.addEvent("code", 3.0, () -> {}).onEvent(name -> {
      fired.add(name);
      where.add(f.progressMeters());
    });
    assertEquals(path.geometry().sAtWaypoint(0.5), f.eventMeters()[0], 0.0);
    run(path, f, FieldFrame.RED_HALF_TURN, 20, NONE);
    assertEquals(List.of("fileEarly", "code", "fileLate"), fired);
    assertTrue(Math.abs(where.get(0) - path.geometry().sAtWaypoint(0.5)) < 0.2, "fired as it passed: " + where);
    assertEquals(6.0, where.get(2), 0.045, "fired as it passed: " + where);
  }

  /** Plans spin a little under the follower's own limits (540°/s, 720°/s²), as the e2e suite's robot does. */
  static final RobotConfig SPIN_ROBOT = RobotConfig.DEFAULT.withMaxAngularRadPerSec(7.5).withMaxAngularRadPerSecSquared(10.0);

  @Test
  void aStepInStartsOwnLoopPlansForAWholeLoop() {
    // the adapters start and step in the same loop, at the same time: that first command must not be a
    // millisecond's worth of acceleration (it made every path start a loop late)
    Path path = Path.build(FixturePaths.straight(), ROBOT);
    PathSample s0 = path.sample(0);
    Pose p = new Pose(s0.xMeters(), s0.yMeters(), s0.headingRadians());
    FieldSpeeds still = new FieldSpeeds(0, 0, 0);
    PathFollower same = new PathFollower(path, CFG);
    same.start(1.0, p, still, FieldFrame.BLUE);
    FieldSpeeds first = same.step(1.0, p, still);
    PathFollower next = new PathFollower(path, CFG);
    next.start(1.0, p, still, FieldFrame.BLUE);
    FieldSpeeds aLoopLater = next.step(1.0 + DT, p, still);
    assertEquals(aLoopLater.vxMetersPerSecond(), first.vxMetersPerSecond(), 1e-12);
    assertTrue(first.vxMetersPerSecond() > 0.05, "moves off the line at once: " + first);
  }

  @Test
  void aTurnOnTheSpotFollowsItsPlanAndItsEventsFireOnTheTurn() {
    PathSpec spec = FixturePaths.pureRotation().withEvents(List.of(PathEvent.atWaypoint("half", 0.5)));
    Path path = Path.build(spec, SPIN_ROBOT);
    PathFollower f = new PathFollower(path, FollowerConfig.defaults(SPIN_ROBOT));
    List<Double> headingAtEvent = new ArrayList<>();
    double[] heading = {0};
    f.onEvent(name -> headingAtEvent.add(heading[0]));
    Run r = run(path, f, FieldFrame.BLUE, 5, (t, plant, ff) -> heading[0] = plant.pose().headingRadians());
    assertTrue(r.finished());
    assertTrue(r.seconds() < path.totalTimeSeconds() + 0.3, "took " + r.seconds() + " s for a " + path.totalTimeSeconds() + " s plan");
    // halfway through the turn, not all at once at the start (progress used to come from a 1 mm projection)
    assertEquals(45.0, Math.toDegrees(headingAtEvent.get(0)), 8.0);
    for (Pose q : r.trace()) {
      assertTrue(Math.hypot(q.xMeters() - 1.0, q.yMeters() + 0.5) < 0.01, "stays on the spot: " + q);
    }
  }

  @Test
  void aShortHopWithABigTurnIsFollowedAlongItsPlan() {
    // 0.2 m with an 80° turn is inside the terminal radius from the start; it used to be driven by the
    // DriveToPose law alone, at three times the planned speed, while the turn lagged
    Path path = Path.build(FixturePaths.shortHop(), SPIN_ROBOT);
    PathFollower f = new PathFollower(path, FollowerConfig.defaults(SPIN_ROBOT));
    double planPeak = 0;
    for (double s = 0; s <= path.lengthMeters(); s += 0.002) {
      planPeak = Math.max(planPeak, path.sample(s).metersPerSecond());
    }
    Run r = run(path, f, FieldFrame.BLUE, 5, NONE);
    double peak = 0;
    for (int i = 1; i < r.trace().size(); i++) {
      Pose a = r.trace().get(i - 1);
      Pose b = r.trace().get(i);
      peak = Math.max(peak, Math.hypot(b.xMeters() - a.xMeters(), b.yMeters() - a.yMeters()) / DT);
    }
    assertTrue(r.finished());
    // 1.28x here (it was 3.5x): the last 2° are the DriveToPose law's, which closes the last centimetres
    // a little faster than the plan's crawl. That speed (0.32 m/s) doesn't depend on the plan; the plan
    // turns at 90 % of the robot's spin cap, so its own peak is 0.26 m/s (0.27 when it turned at all of it)
    assertTrue(peak < 1.35 * planPeak, "peak " + peak + " m/s against the plan's " + planPeak);
    assertTrue(r.seconds() < path.totalTimeSeconds() + 0.35, "took " + r.seconds() + " s for a " + path.totalTimeSeconds() + " s plan");
  }

  @Test
  void eventListenersAddUp() {
    Path path = Path.build(FixturePaths.straight(), ROBOT);
    List<String> a = new ArrayList<>();
    List<String> b = new ArrayList<>();
    PathFollower f = new PathFollower(path, CFG).addEvent("x", 1.0, () -> {}).onEvent(a::add).onEvent(b::add);
    run(path, f, FieldFrame.BLUE, 10, NONE);
    assertEquals(List.of("x"), a);
    assertEquals(List.of("x"), b);
  }

  @Test
  void eventsNotReachedFireOnANormalFinish() {
    Path path = Path.build(FixturePaths.straight(), ROBOT);
    PathFollower f = new PathFollower(path, CFG);
    List<String> fired = new ArrayList<>();
    List<PathFollower.Phase> phaseAtFire = new ArrayList<>();
    f.addEvent("at end", path.lengthMeters(), () -> phaseAtFire.add(f.phase())).onEvent(fired::add);
    Run r = run(path, f, FieldFrame.BLUE, 10, (t, p, ff) -> {
      if (ff.phase() == PathFollower.Phase.TERMINAL) {
        p.displace(0, 0); // the robot stops a few mm short, so it never 'passes' the end
      }
    });
    assertTrue(r.finished());
    assertEquals(List.of("at end"), fired);
  }

  @Test
  void interruptCancelsWhatHasNotFired() {
    Path path = Path.build(FixturePaths.sCurve(), ROBOT);
    PathFollower f = new PathFollower(path, CFG);
    List<String> fired = new ArrayList<>();
    f.addEvent("early", 0.5, () -> {}).addEvent("late", 8.0, () -> {}).onEvent(fired::add);
    Run r = run(path, f, FieldFrame.BLUE, 20, (t, p, ff) -> {
      if (Math.abs(t - 1.0) < 1e-9) {
        ff.cancel();
      }
    });
    assertFalse(r.finished());
    assertEquals(PathFollower.Phase.CANCELLED, f.phase());
    assertEquals(List.of("early"), fired);
    FieldSpeeds after = f.step(5.0, new Pose(0, 0, 0), new FieldSpeeds(0, 0, 0));
    assertEquals(new FieldSpeeds(0, 0, 0), after);
  }

  @Test
  void restartingResetsEvents() {
    Path path = Path.build(FixturePaths.straight(), ROBOT);
    PathFollower f = new PathFollower(path, CFG);
    List<String> fired = new ArrayList<>();
    f.addEvent("half", 1.5, () -> {}).onEvent(fired::add);
    run(path, f, FieldFrame.BLUE, 10, NONE);
    run(path, f, FieldFrame.BLUE, 10, NONE);
    assertEquals(List.of("half", "half"), fired);
  }

  // ------------------------------------------------------------------------------ rolling end

  @Test
  void rollingEndPassesTheEndAndHandsItsVelocityOn() {
    PathSpec first = PathSpec.of(List.of(Waypoint.at(-2, 0, 0), Waypoint.at(0, 0, 0)))
        .withLimits(4, 6).withEndSpeeds(0, 1.5);
    PathSpec second = PathSpec.of(List.of(Waypoint.at(0, 0, 0), Waypoint.at(2, 1, 0)))
        .withLimits(4, 6).withEndSpeeds(1.5, 0);
    Path a = Path.build(first, ROBOT);
    Path b = Path.build(second, ROBOT);
    PathFollower fa = new PathFollower(a, CFG);
    Run ra = run(a, fa, FieldFrame.BLUE, 10, NONE);
    assertTrue(ra.finished());
    Pose atEnd = ra.trace().get(ra.trace().size() - 1);
    assertTrue(atEnd.xMeters() >= -0.1, "finished as it passed the end, at x = " + atEnd.xMeters());
    FieldSpeeds handed = fa.endCommand();
    assertEquals(1.5, handed.vxMetersPerSecond(), 0.15);

    PathFollower fb = new PathFollower(b, CFG);
    fb.start(ra.seconds(), atEnd, handed, FieldFrame.BLUE, handed);
    FieldSpeeds firstOut = fb.step(ra.seconds() + DT, atEnd, handed);
    double jump = Math.hypot(firstOut.vxMetersPerSecond() - handed.vxMetersPerSecond(),
        firstOut.vyMetersPerSecond() - handed.vyMetersPerSecond());
    assertTrue(jump <= PathFollower.limiterBudget(CFG) * DT + 1e-9,
        "no drop at the hand-off: the second path starts from the handed velocity");
    assertTrue(firstOut.vxMetersPerSecond() > 1.0);
  }

  // ------------------------------------------------------------------------------ moving goal

  @Test
  void movingGoalRebuildsOnlyTheTailWithASmoothJoin() {
    Path path = Path.build(FixturePaths.sCurve(), ROBOT);
    PathFollower f = new PathFollower(path, CFG);
    Pose[] goal = {null};
    f.bindGoal(() -> Optional.ofNullable(goal[0]));
    double[] join = new double[5];
    Run r = run(path, f, FieldFrame.BLUE, 20, (t, plant, ff) -> {
      if (Math.abs(t - 1.0) < 1e-9) {
        PathSample at = path.sample(ff.progressMeters());
        join[0] = at.xMeters();
        join[1] = at.yMeters();
        join[2] = Math.atan2(at.tangentY(), at.tangentX());
        join[3] = at.curvature();
        goal[0] = new Pose(4.3, 1.4, 0.3); // the target moved 50 cm and turned
      }
      if (Math.abs(t - 1.5) < 1e-9) {
        goal[0] = null; // lost sight of it: hold the last estimate
      }
    });
    assertEquals(1, f.tailRebuildCount());
    Path tail = f.activePath();
    assertTrue(tail != path);
    GeometrySample start = tail.geometry().sample(0);
    assertEquals(join[0], start.xMeters(), 1e-9);
    assertEquals(join[1], start.yMeters(), 1e-9);
    assertEquals(0.0, Angles.wrap(join[2] - start.tangentRadians()), 1e-9);
    assertEquals(join[3], start.curvature(), 1e-6);
    assertTrue(r.finished());
    Pose end = r.trace().get(r.trace().size() - 1);
    assertEquals(4.3, end.xMeters(), 0.02);
    assertEquals(1.4, end.yMeters(), 0.02);
    assertEquals(0.3, end.headingRadians(), Math.toRadians(1));
    System.out.printf("tail rebuild (cold in this JVM): %.0f us%n", f.lastTailRebuildMicros());
  }

  @Test
  void tailRebuildIsUnderAMillisecondWarm() {
    Path path = Path.build(FixturePaths.autoRoute().withEndSpeeds(0, 0), ROBOT);
    PathFollower f = new PathFollower(path, CFG);
    Pose[] goal = {null};
    f.bindGoal(() -> Optional.ofNullable(goal[0]));
    PathSample s0 = path.sample(0);
    Pose p = new Pose(s0.xMeters(), s0.yMeters(), s0.headingRadians());
    double best = Double.POSITIVE_INFINITY;
    double total = 0;
    int n = 0;
    for (int rep = 0; rep < 400; rep++) {
      f.start(0, p, new FieldSpeeds(0, 0, 0), FieldFrame.BLUE);
      goal[0] = new Pose(-5.44 + 0.001 * (rep % 50), 3.0 + 0.05, Math.PI); // a move every start
      f.step(DT, p, new FieldSpeeds(0, 0, 0));
      if (rep >= 200) {
        best = Math.min(best, f.lastTailRebuildMicros());
        total += f.lastTailRebuildMicros();
        n++;
      }
    }
    System.out.printf("tail rebuild of a %.1f m tail, warm: mean %.0f us, best %.0f us%n",
        f.activePath().lengthMeters(), total / n, best);
    assertTrue(total / n < 1000.0, "mean tail rebuild " + total / n + " us");
  }

  @Test
  void smallGoalJitterDoesNotRebuild() {
    Path path = Path.build(FixturePaths.sCurve(), ROBOT);
    PathFollower f = new PathFollower(path, CFG);
    f.bindGoal(() -> Optional.of(new Pose(4.0 + 0.005, 1.0, Math.PI / 2 + 0.01))); // 5 mm, 0.6 deg
    run(path, f, FieldFrame.BLUE, 20, NONE);
    assertEquals(0, f.tailRebuildCount());
  }

  // ------------------------------------------------------------------------------ overrides

  // ------------------------------------------------------------------------------ telemetry

  @Test
  void telemetryDescribesTheRun() {
    Path path = Path.build(FixturePaths.sCurve(), ROBOT);
    PathFollower f = new PathFollower(path, CFG);
    run(path, f, FieldFrame.BLUE, 1.0, NONE);
    assertTrue(f.progressMeters() > 1.0);
    assertEquals(path.lengthMeters() - f.progressMeters(), f.remainingMeters(), 1e-12);
    Pose ref = f.reference();
    PathSample here = path.sample(f.progressMeters());
    assertEquals(here.xMeters(), ref.xMeters(), 1e-9);
    assertEquals(new Pose(4, 1, 0).xMeters(), f.goal().xMeters(), 1e-6);
    assertEquals(FieldFrame.BLUE, f.frame());
  }

  @Test
  void headingProfileIsFollowed() {
    PathSpec spec = FixturePaths.sCurve();
    Path path = Path.build(spec, ROBOT);
    HeadingProfile h = path.heading();
    PathFollower f = new PathFollower(path, CFG);
    double[] worst = {0};
    run(path, f, FieldFrame.BLUE, 20, (t, plant, ff) -> {
      if (ff.phase() == PathFollower.Phase.TRACKING && t > 0.2) {
        double err = Angles.wrap(h.sample(ff.progressMeters()).radians() - plant.heading);
        worst[0] = Math.max(worst[0], Math.abs(err));
      }
    });
    assertTrue(worst[0] < Math.toRadians(5), "worst heading lag " + Math.toDegrees(worst[0]) + " deg");
  }

  // ------------------------------------------------------------------------------ adaptive grip

  /** A scripted slip signal. */
  static final class FakeSlip implements frc.team5712.path.slip.SlipSignal {
    double confidence;
    double achieved;
    double gap = Double.NaN;

    @Override
    public double confidence() {
      return confidence;
    }

    @Override
    public double achievedAccelMetersPerSecondSquared() {
      return achieved;
    }

    @Override
    public double imuGapMetersPerSecondSquared() {
      return gap;
    }
  }

  @Test
  void quietSlipSignalChangesNothing() {
    Path path = Path.build(FixturePaths.sCurve(), ROBOT);
    Run plain = run(path, new PathFollower(path, CFG), FieldFrame.BLUE, 20, NONE);
    FakeSlip quiet = new FakeSlip();
    quiet.confidence = 0.25; // below the threshold: noise, not slip
    quiet.achieved = 3.0;
    Run withSignal = run(path, new PathFollower(path, CFG).withSlipSignal(quiet), FieldFrame.BLUE, 20, NONE);
    assertEquals(plain.trace().size(), withSignal.trace().size());
    for (int i = 0; i < plain.trace().size(); i++) {
      assertEquals(plain.trace().get(i), withSignal.trace().get(i));
    }
  }

  @Test
  void slipLowersTheGripThenItRecovers() {
    Path path = Path.build(FixturePaths.autoRoute().withEndSpeeds(0, 0), ROBOT);
    FakeSlip slip = new FakeSlip();
    // a share no command reaches: every slip counts as the floor's
    FollowerConfig counts = CFG.toBuilder().expectedSlipShare(100.0).build();
    PathFollower f = new PathFollower(path, counts).withSlipSignal(slip);
    double[] muDuring = {Double.NaN};
    double[] worstAccel = {0};
    FieldSpeeds[] last = {new FieldSpeeds(0, 0, 0)};
    Run r = run(path, f, FieldFrame.BLUE, 30, (t, plant, ff) -> {
      boolean slipping = t > 1.0 && t < 1.5;
      slip.confidence = slipping ? 0.9 : 0.0;
      slip.achieved = 5.0; // m/s²: what the slick floor gave
      if (Math.abs(t - 1.4) < 1e-9) {
        muDuring[0] = ff.slipFrictionCoefficient();
      }
      if (t > 1.1 && t < 1.5) {
        FieldSpeeds v = plant.velocity();
        double a = Math.hypot(v.vxMetersPerSecond() - last[0].vxMetersPerSecond(),
            v.vyMetersPerSecond() - last[0].vyMetersPerSecond()) / DT;
        worstAccel[0] = Math.max(worstAccel[0], a);
      }
      last[0] = plant.velocity();
    });
    double expected = 5.0 / RobotConfig.GRAVITY * (1 - CFG.slipMargin());
    assertEquals(expected, muDuring[0], 1e-9);
    // speeding up is held to the lowered grip; braking may still use slipBrakeFriction
    assertTrue(worstAccel[0] <= CFG.slipBrakeFriction() * RobotConfig.GRAVITY + 1e-6, "accel " + worstAccel[0]);
    assertTrue(r.finished());
    assertTrue(Double.isInfinite(f.slipFrictionCoefficient()) || f.slipFrictionCoefficient() > muDuring[0],
        "climbs back after the slip");
  }

  @Test
  void slipWhileAskingForMoreThanTheWheelSlipOnsetIsTheTreadsOwn() {
    // A launch at 9 m/s² asks for more than the light wheels hold (7.2 m/s² for the default robot), so
    // they slip on a good floor: adaptive grip leaves the grip alone. The same slip on a gentle path,
    // asking for 2 m/s², says the floor is slick.
    FakeSlip slip = new FakeSlip();
    double[] lowest = {Double.POSITIVE_INFINITY};
    double[] achieved = {9.0}; // on its own floor the robot gets what the plan asks for
    Hook slipping = (t, plant, ff) -> {
      slip.confidence = t > 0.1 && t < 0.4 ? 0.9 : 0.0;
      slip.achieved = achieved[0];
      lowest[0] = Math.min(lowest[0], ff.slipFrictionCoefficient());
    };
    RobotConfig tipOrGrip = ROBOT; // a plan that reaches 9
    FollowerConfig tipCfg = FollowerConfig.defaults(tipOrGrip);
    Path hard = Path.build(FixturePaths.straight().withLimits(4.0, 9.0), tipOrGrip);
    run(hard, new PathFollower(hard, tipCfg).withSlipSignal(slip), FieldFrame.BLUE, 20, slipping);
    assertEquals(Double.POSITIVE_INFINITY, lowest[0], "a hard launch's slip is expected");
    lowest[0] = Double.POSITIVE_INFINITY;
    Path gentle = Path.build(FixturePaths.straight().withLimits(4.0, 2.0), ROBOT);
    run(gentle, new PathFollower(gentle, CFG).withSlipSignal(slip), FieldFrame.BLUE, 20, slipping);
    assertTrue(lowest[0] < 1.0, "a gentle launch's slip lowers the grip: " + lowest[0]);
    // the hard launch again, the wheels spinning up 5 m/s² faster than the chassis and the chassis short of the ask:
    // all of them slide
    slip.gap = 5.0;
    achieved[0] = 7.5;
    lowest[0] = Double.POSITIVE_INFINITY;
    run(hard, new PathFollower(hard, tipCfg).withSlipSignal(slip), FieldFrame.BLUE, 20, slipping);
    assertTrue(lowest[0] < 1.0, "a slide lowers the grip whatever the robot asked for: " + lowest[0]);
    // ...but with the chassis getting all it asked for, only the light wheels spin: the tread's own
    achieved[0] = 11.0;
    lowest[0] = Double.POSITIVE_INFINITY;
    run(hard, new PathFollower(hard, tipCfg).withSlipSignal(slip), FieldFrame.BLUE, 20, slipping);
    assertEquals(Double.POSITIVE_INFINITY, lowest[0], "light wheels spinning while the robot gets its ask");
    // the hard launch again, no gap, but the robot gets only 5 of the 9 it asked for: the floor is giving out
    slip.gap = 0.0;
    achieved[0] = 5.0;
    lowest[0] = Double.POSITIVE_INFINITY;
    run(hard, new PathFollower(hard, tipCfg).withSlipSignal(slip), FieldFrame.BLUE, 20, slipping);
    assertTrue(lowest[0] < 1.0, "falling well short of the ask lowers the grip: " + lowest[0]);
  }

  /**
   * A slip signal that reads the plant like the bench IMU: the chassis acceleration, one loop late and low-passed
   * (30 ms), times the share the floor delivers, with seeded noise; the wheels-against-IMU gap is noisy too.
   */
  static final class LaggedSlip implements frc.team5712.path.slip.SlipSignal {
    final java.util.Random noise = new java.util.Random(5712);
    double confidence;
    double share = 1.0; // the share of the chassis acceleration the floor delivers
    double cap = Double.POSITIVE_INFINITY; // the most the floor gives, m/s²
    double gapMean = 1.0;
    private double achieved;
    private double gap;
    private FieldSpeeds last = new FieldSpeeds(0, 0, 0);

    /** One loop: read the plant's velocity (the command it applied, a loop late). */
    void update(KinematicPlant plant) {
      FieldSpeeds v = plant.velocity();
      double a = Math.hypot(v.vxMetersPerSecond() - last.vxMetersPerSecond(),
          v.vyMetersPerSecond() - last.vyMetersPerSecond()) / DT;
      last = v;
      double felt = Math.min(cap, share * a) + 0.25 * noise.nextGaussian();
      achieved += (Math.abs(felt) - achieved) * DT / (0.03 + DT);
      gap = Math.max(0.0, gapMean + 0.3 * noise.nextGaussian());
    }

    @Override
    public double confidence() {
      return confidence;
    }

    @Override
    public double achievedAccelMetersPerSecondSquared() {
      return achieved;
    }

    @Override
    public double imuGapMetersPerSecondSquared() {
      return gap;
    }
  }

  /** A launch at the robot's full traction, slipping from 0.06 s until it reaches top speed. */
  private static double lowestGripOnALaunch(LaggedSlip slip, double slipUntil) {
    Path launch = Path.build(FixturePaths.straight().withLimits(4.0, ROBOT.tractionMetersPerSecondSquared()), ROBOT);
    double[] lowest = {Double.POSITIVE_INFINITY};
    run(launch, new PathFollower(launch, CFG).withSlipSignal(slip), FieldFrame.BLUE, 20, (t, plant, ff) -> {
      slip.update(plant);
      slip.confidence = t > 0.06 && t < slipUntil ? 0.9 : 0.0;
      lowest[0] = Math.min(lowest[0], ff.slipFrictionCoefficient());
    });
    return lowest[0];
  }

  @Test
  void aModeratelySlickFloorIsCaughtWithoutAGap() {
    // The launch asks for the robot's full traction; the floor gives 85 % of it with the wheels and the IMU
    // agreeing (no slide). The loaded wheels can't be making up for the light ones: the floor is giving out.
    LaggedSlip slip = new LaggedSlip();
    slip.share = 0.85;
    double lowest = lowestGripOnALaunch(slip, 0.45);
    assertTrue(lowest < ROBOT.frictionCoefficient(), "85 % of the ask lowers the grip: " + lowest);
    // what it measured is about what the floor gave, less the margin
    double gave = 0.85 * ROBOT.tractionMetersPerSecondSquared() / RobotConfig.GRAVITY;
    assertTrue(lowest > 0.7 * gave * (1 - CFG.slipMargin()), "not far below what the floor gave: " + lowest);
    // ...and the same launch getting all it asks for is the tread's own slip
    LaggedSlip good = new LaggedSlip();
    assertEquals(Double.POSITIVE_INFINITY, lowestGripOnALaunch(good, 0.45), "a good floor's launch");
  }

  @Test
  void aGripDropInsideARunningSpellIsCaught() {
    // A hard launch (9 m/s², over the wheel-slip onset) slips the light wheels on a good floor, and the robot gets
    // what it asks for; 0.25 s in, still slipping, the floor drops to 5 m/s². The spell never ends, so only what
    // the robot gets now (not the best of the spell) can show it.
    Path hard = Path.build(FixturePaths.straight().withLimits(4.0, 9.0), ROBOT);
    LaggedSlip slip = new LaggedSlip();
    double[] lowest = {Double.POSITIVE_INFINITY};
    double[] loweredAt = {Double.NaN};
    run(hard, new PathFollower(hard, CFG).withSlipSignal(slip), FieldFrame.BLUE, 20, (t, plant, ff) -> {
      slip.cap = t > 0.25 ? 5.0 : Double.POSITIVE_INFINITY;
      slip.update(plant);
      slip.confidence = t > 0.06 && t < 0.6 ? 0.9 : 0.0;
      double mu = ff.slipFrictionCoefficient();
      if (Double.isFinite(mu) && Double.isNaN(loweredAt[0])) {
        loweredAt[0] = t;
      }
      lowest[0] = Math.min(lowest[0], mu);
    });
    assertTrue(loweredAt[0] > 0.25, "not before the drop: " + loweredAt[0]);
    assertTrue(loweredAt[0] < 0.45, "within 0.2 s of the drop: " + loweredAt[0]);
    // to near what the floor gives, less the margin (once lowered, the profile slows and asks the floor for less, so
    // the estimate is what the IMU had settled to by then)
    double gives = 5.0 / RobotConfig.GRAVITY * (1 - CFG.slipMargin());
    assertTrue(lowest[0] >= gives - 0.05 && lowest[0] < 1.4 * gives, "near what the floor gives: " + lowest[0]);
  }

  @Test
  void slipStartingAtCruiseOnAGoodFloorKeepsTheGrip() {
    // Cruising at 3 m/s the robot asks for (almost) nothing, and gets it; the detector flags a spell anyway (a
    // knock, a seam). Nothing was asked of the floor, so it says nothing about the floor.
    Path cruise = Path.build(PathSpec.of(List.of(Waypoint.at(-4.0, 0.0, 0.0), Waypoint.at(4.0, 0.0, 0.0)))
        .withLimits(3.0, 6.0), ROBOT);
    LaggedSlip slip = new LaggedSlip();
    double[] lowest = {Double.POSITIVE_INFINITY};
    run(cruise, new PathFollower(cruise, CFG).withSlipSignal(slip), FieldFrame.BLUE, 20, (t, plant, ff) -> {
      slip.update(plant);
      slip.confidence = t > 1.2 && t < 1.5 ? 0.9 : 0.0;
      lowest[0] = Math.min(lowest[0], ff.slipFrictionCoefficient());
    });
    assertEquals(Double.POSITIVE_INFINITY, lowest[0], "slip at cruise on a good floor");
  }

  @Test
  void slipSlowsTheRunAndCanBeTurnedOff() {
    Path path = Path.build(FixturePaths.sCurve(), ROBOT);
    FakeSlip slip = new FakeSlip();
    slip.confidence = 1.0;
    slip.achieved = 3.0;
    double plain = run(path, new PathFollower(path, CFG), FieldFrame.BLUE, 20, NONE).seconds();
    double slowed = run(path, new PathFollower(path, CFG).withSlipSignal(slip), FieldFrame.BLUE, 20, NONE).seconds();
    FollowerConfig off = CFG.toBuilder().adaptiveGrip(false).build();
    double ignored = run(path, new PathFollower(path, off).withSlipSignal(slip), FieldFrame.BLUE, 20, NONE).seconds();
    assertTrue(slowed > plain + 0.2, plain + " s vs " + slowed + " s on a slippery floor");
    assertEquals(plain, ignored, 1e-12);
  }
}
