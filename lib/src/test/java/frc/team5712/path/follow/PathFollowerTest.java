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
import frc.team5712.path.core.PathGeometry;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.PathSpec;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
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
    assertTrue(where.get(0) >= 1.0 && where.get(0) < 1.2, "fired as it passed, at " + where.get(0));
    assertTrue(where.get(2) >= 6.0 && where.get(2) < 6.2);
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
    assertTrue(jump <= CFG.limiterFrictionCoefficient() * RobotConfig.GRAVITY * DT + 1e-9,
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

  @Test
  void lateralOffsetShiftsTheTrack() {
    Path path = Path.build(FixturePaths.straight(), ROBOT);
    PathFollower f = new PathFollower(path, CFG).lateralOffset(() -> 0.2);
    Run r = run(path, f, FieldFrame.BLUE, 10, NONE);
    assertTrue(r.finished());
    Pose end = r.trace().get(r.trace().size() - 1);
    assertEquals(0.2, end.yMeters(), 0.02, "ends 20 cm to the left of the goal");
    Pose mid = r.trace().get(r.trace().size() / 2);
    assertEquals(0.2, mid.yMeters(), 0.03);
  }

  @Test
  void alongTrackScaleSlowsTheRobot() {
    Path path = Path.build(FixturePaths.sCurve(), ROBOT);
    double full = run(path, new PathFollower(path, CFG), FieldFrame.BLUE, 20, NONE).seconds();
    double half = run(path, new PathFollower(path, CFG).alongTrackScale(() -> 0.5), FieldFrame.BLUE, 20, NONE).seconds();
    assertTrue(half > 1.5 * full - 0.5, full + " s vs " + half + " s");
  }

  @Test
  void bypassOverrideTakesEffectAtOnceAndRespectRamps() {
    Path path = Path.build(FixturePaths.straight(), ROBOT);
    PathFollower bypass = new PathFollower(path, CFG).overrideY(() -> 1.0, OverrideMode.BYPASS);
    bypass.start(0, new Pose(-1.5, 0, 0), new FieldSpeeds(0, 0, 0), FieldFrame.BLUE);
    assertEquals(1.0, bypass.step(DT, new Pose(-1.5, 0, 0), new FieldSpeeds(0, 0, 0)).vyMetersPerSecond(), 0.0);

    PathFollower respect = new PathFollower(path, CFG).overrideY(() -> 1.0, OverrideMode.RESPECT_LIMITS);
    respect.start(0, new Pose(-1.5, 0, 0), new FieldSpeeds(0, 0, 0), FieldFrame.BLUE);
    double vy = respect.step(DT, new Pose(-1.5, 0, 0), new FieldSpeeds(0, 0, 0)).vyMetersPerSecond();
    assertTrue(vy > 0 && vy <= CFG.limiterFrictionCoefficient() * RobotConfig.GRAVITY * DT + 1e-9, "ramps: " + vy);

    PathFollower off = new PathFollower(path, CFG).overrideOmega(() -> Double.NaN, OverrideMode.BYPASS);
    off.start(0, new Pose(-1.5, 0, 0), new FieldSpeeds(0, 0, 0), FieldFrame.BLUE);
    assertEquals(0.0, off.step(DT, new Pose(-1.5, 0, 0), new FieldSpeeds(0, 0, 0)).omegaRadPerSec(), 1e-12,
        "NaN means no override");
  }

  @Test
  void overridesAreInTheRobotsFrame() {
    Path path = Path.build(FixturePaths.straight(), ROBOT);
    PathFollower f = new PathFollower(path, CFG).overrideX(() -> 0.5, OverrideMode.BYPASS);
    f.start(0, new Pose(1.5, 0, Math.PI), new FieldSpeeds(0, 0, 0), FieldFrame.RED_HALF_TURN);
    assertEquals(0.5, f.step(DT, new Pose(1.5, 0, Math.PI), new FieldSpeeds(0, 0, 0)).vxMetersPerSecond(), 1e-12);
  }

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
}
