package frc.team5712.path.follow;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.Vec2;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import org.junit.jupiter.api.Test;

/**
 * The rail and the piece assist on the simplest robot (it does last loop's command). The thorough drives, with
 * the robot's own numbers, are Precision-Drive's RailTest and PieceAssistTest.
 */
class RailFollowerTest {
  private static final RobotConfig ROBOT = RobotConfig.DEFAULT;
  private static final FollowerConfig CFG = FollowerConfig.defaults(ROBOT);
  private static final double DT = 0.02;
  private static final double MAX = 4.5;
  // along field x at y = 1, holding heading 0
  private static final Rail RAIL = Rail.through(0.0, 1.0, 0.0).withHeading(0.0);

  @Test
  void theLimitsComeFromTheRobotFile() {
    RailLaw law = new RailLaw(CFG);
    double traction = ROBOT.tractionMetersPerSecondSquared();
    assertEquals(RailLaw.ACROSS_SHARE * traction, law.acrossBrakeMetersPerSecondSquared(), 1e-12);
    // braking onto the rail and the driver's change along it together stay inside the one grip budget
    assertTrue(Math.hypot(law.acrossBrakeMetersPerSecondSquared(), law.alongAccelMetersPerSecondSquared()) < traction);
    RobotConfig slick = ROBOT.withFrictionCoefficient(0.6);
    assertTrue(new RailLaw(FollowerConfig.defaults(slick)).acrossBrakeMetersPerSecondSquared()
        < law.acrossBrakeMetersPerSecondSquared(), "a robot with less grip brakes onto the rail more gently");
  }

  /** From 80 cm off and turned, at full stick: onto the line, squared up, no more than 1 cm past it. */
  @Test
  void pullsOntoTheRailWhileTheDriverDrives() {
    RailFollower f = new RailFollower(CFG);
    KinematicPlant plant = new KinematicPlant(new Pose(0.0, 0.2, 0.6), new FieldSpeeds(0, 0, 0));
    f.reset(plant.velocity());
    FieldSpeeds stick = new FieldSpeeds(MAX, 0.0, 0.0);
    double past = 0.0;
    for (int i = 0; i < 150; i++) {
      FieldSpeeds cmd = f.step(RAIL, plant.pose(), stick, MAX, DT);
      assertTrue(Math.hypot(cmd.vxMetersPerSecond(), cmd.vyMetersPerSecond()) <= MAX + 1e-9);
      plant.step(cmd, DT);
      past = Math.max(past, plant.y - 1.0);
    }
    assertEquals(1.0, plant.y, 0.01, "on the rail");
    assertEquals(0.0, plant.heading, Math.toRadians(1.0), "squared up");
    assertTrue(past < 0.01, "crossed the rail by " + past);
    assertTrue(plant.x > 3.0, "and still got somewhere: " + plant.x);
  }

  /** How a run onto the rail went. */
  private record Capture(double peakAcross, double peakAcrossAccel, double mostTurnedDeg, double secondsToLine,
      double metersAlongToLine, double alongKept, double pastLine) {}

  /**
   * A rail along x at y = 0. The robot starts {@code off} metres to its left, already moving along it at {@code
   * speed}, and the driver holds {@code stick}.
   */
  private static Capture capture(Rail rail, double off, double speed, FieldSpeeds stick) {
    RailFollower f = new RailFollower(CFG);
    KinematicPlant plant = new KinematicPlant(new Pose(0.0, off, 0.0), new FieldSpeeds(speed, 0, 0));
    f.reset(plant.velocity());
    double peak = 0.0;
    double peakAccel = 0.0;
    double turned = 0.0;
    double lastAcross = 0.0;
    double seconds = Double.NaN;
    double meters = Double.NaN;
    double alongSum = 0.0;
    int n = 0;
    double past = 0.0;
    for (int i = 0; i < 600; i++) {
      FieldSpeeds cmd = f.step(rail, plant.pose(), stick, MAX, DT);
      peak = Math.max(peak, Math.abs(cmd.vyMetersPerSecond()));
      peakAccel = Math.max(peakAccel, Math.abs(cmd.vyMetersPerSecond() - lastAcross) / DT);
      lastAcross = cmd.vyMetersPerSecond();
      if (Math.abs(cmd.vxMetersPerSecond()) > 0.2) {
        turned = Math.max(turned, Math.toDegrees(Math.abs(Math.atan2(cmd.vyMetersPerSecond(), cmd.vxMetersPerSecond()))));
      }
      if (Double.isNaN(seconds)) {
        alongSum += cmd.vxMetersPerSecond();
        n++;
      }
      plant.step(cmd, DT);
      past = Math.max(past, -Math.signum(off) * plant.y);
      if (Double.isNaN(seconds) && Math.abs(plant.y) < 0.02) {
        seconds = (i + 1) * DT;
        meters = plant.x;
      }
    }
    double wanted = stick.vxMetersPerSecond();
    return new Capture(peak, peakAccel, turned, seconds, meters, Math.abs(wanted) < 1e-9 ? Double.NaN : alongSum / n / wanted,
        past);
  }

  /**
   * The pull-in share: the rail pulls at that share of the driver's speed along it, so the robot slides on at a
   * fixed angle and joins the line about (how far off ÷ share) further along. The driver picks where it joins.
   */
  @Test
  void thePullInIsAShareOfTheDriversSpeedAlongTheRail() {
    Rail rail = Rail.through(0.0, 0.0, 0.0).withHeading(0.0);
    FieldSpeeds along = new FieldSpeeds(3.0, 0.0, 0.0);
    System.out.println("    rail, 1.2 m off, cruising and holding 3 m/s along it: peak speed across m/s / hardest change across "
        + "m/s² / most the travel is turned deg / s to the line / m along to the line / share of the driver's speed kept");
    for (double share : new double[] {Double.POSITIVE_INFINITY, 1.0, 0.5, 0.3, 0.2}) {
      Capture c = capture(Double.isFinite(share) ? rail.withPullIn(share) : rail, 1.2, 3.0, along);
      System.out.printf("    pull-in %-9s %5.2f / %5.1f / %5.1f / %5.2f / %5.2f / %4.2f%n",
          Double.isFinite(share) ? String.valueOf(share) : "no cap", c.peakAcross(), c.peakAcrossAccel(), c.mostTurnedDeg(),
          c.secondsToLine(), c.metersAlongToLine(), c.alongKept());
      assertTrue(c.pastLine() < 0.01, "never more than 1 cm past the line, at " + share + ": " + c.pastLine());
      if (Double.isFinite(share)) {
        assertTrue(c.peakAcross() <= share * 3.0 + 1e-9, "the pull never passes " + share + " of 3 m/s: " + c.peakAcross());
        if (share <= 0.5) {
          // (at a share of 1 and this speed the two together would pass top speed, so the along speed gives way)
          assertEquals(Math.toDegrees(Math.atan(share)), c.mostTurnedDeg(), 0.5, "it slides on at atan(share)");
        }
        // the straight slide would join after 1.2 / share metres; speeding up to it and easing in add a little
        assertTrue(c.metersAlongToLine() > 1.2 / share && c.metersAlongToLine() < 1.2 / share + 1.2,
            "joined after " + c.metersAlongToLine() + " m at " + share);
      }
    }
  }

  @Test
  void theDriversOwnPushTowardTheLineCountsInFullAndAwayFromItNotAtAll() {
    Rail rail = Rail.through(0.0, 0.0, 0.0).withHeading(0.0).withPullIn(0.3);
    Capture cruising = capture(rail, 1.2, 3.0, new FieldSpeeds(3.0, 0.0, 0.0));
    // the robot is on the rail's left (+y): pushing right (-y) is toward the line
    Capture steering = capture(rail, 1.2, 3.0, new FieldSpeeds(3.0, -2.5, 0.0));
    assertTrue(steering.peakAcross() > 2.0, "the driver's own 2.5 m/s toward the line gets through: " + steering.peakAcross());
    assertTrue(steering.secondsToLine() < 0.6 * cruising.secondsToLine(),
        "so they get there sooner: " + steering.secondsToLine() + " s against " + cruising.secondsToLine());
    assertTrue(steering.pastLine() < 0.01, "and still don't cross it: " + steering.pastLine());
    Capture fighting = capture(rail, 1.2, 3.0, new FieldSpeeds(3.0, 2.5, 0.0));
    assertEquals(cruising.secondsToLine(), fighting.secondsToLine(), 0.1, "pushing away changes nothing: it is a rail");
  }

  @Test
  void withTheStickReleasedTheRobotStillSettlesOntoTheLine() {
    Rail rail = Rail.through(0.0, 0.0, 0.0).withHeading(0.0).withPullIn(0.3);
    Capture still = capture(rail, 0.4, 0.0, new FieldSpeeds(0.0, 0.0, 0.0));
    assertTrue(still.peakAcross() <= RailLaw.SETTLE_METERS_PER_SECOND + 1e-9, "at the settle speed, no faster: " + still.peakAcross());
    assertTrue(still.secondsToLine() < 1.2, "on the line in " + still.secondsToLine() + " s");
    // on the line, a push straight across it does nothing
    Capture across = capture(rail, 0.0, 0.0, new FieldSpeeds(0.0, MAX, 0.0));
    assertTrue(across.peakAcross() < 0.01, "on the rail the driver cannot push across it");
  }

  @Test
  void stopsAtTheEndOfTheRail() {
    RailFollower f = new RailFollower(CFG);
    Rail platform = RAIL.between(0.0, 2.0);
    KinematicPlant plant = new KinematicPlant(new Pose(0.0, 1.0, 0.0), new FieldSpeeds(0, 0, 0));
    f.reset(plant.velocity());
    double furthest = 0.0;
    for (int i = 0; i < 200; i++) {
      plant.step(f.step(platform, plant.pose(), new FieldSpeeds(MAX, 0.0, 0.0), MAX, DT), DT);
      furthest = Math.max(furthest, plant.x);
    }
    assertEquals(2.0, plant.x, 0.01, "stopped at the end with the stick still forward");
    assertTrue(furthest < 2.01, "and never more than 1 cm past it: " + furthest);
  }

  /**
   * Holding the rail with the stick released and a pose that wobbles by a few millimetres: the commands across
   * the rail follow the wobble, and a feed-forward made from their differences would kick the robot. The
   * rail's is made from its plan, and asks for next to nothing.
   */
  @Test
  void theFeedForwardIsQuietWhileHoldingTheRail() {
    RailFollower f = new RailFollower(CFG);
    f.reset(new FieldSpeeds(0, 0, 0));
    Random wobble = new Random(4);
    FieldSpeeds last = new FieldSpeeds(0, 0, 0);
    double fromSteps = 0.0;
    double fromPlan = 0.0;
    for (int i = 0; i < 300; i++) {
      Pose seen = new Pose(2.0 + 0.004 * wobble.nextGaussian(), 1.0 + 0.004 * wobble.nextGaussian(),
          Math.toRadians(0.4) * wobble.nextGaussian());
      FieldSpeeds cmd = f.step(RAIL, seen, new FieldSpeeds(0, 0, 0), MAX, DT);
      FieldSpeeds acc = f.commandedAcceleration();
      fromSteps = Math.max(fromSteps, Math.hypot(cmd.vxMetersPerSecond() - last.vxMetersPerSecond(),
          cmd.vyMetersPerSecond() - last.vyMetersPerSecond()) / DT);
      fromPlan = Math.max(fromPlan, Math.hypot(acc.vxMetersPerSecond(), acc.vyMetersPerSecond()));
      last = cmd;
    }
    assertTrue(fromSteps > 1.0, "the commands do wobble: " + fromSteps + " m/s² between loops");
    assertTrue(fromPlan < 0.25 * fromSteps && fromPlan < 2.0, "the feed-forward doesn't: " + fromPlan);
  }

  /** On the rail, the driver's own speeding up is fed forward in full: it is the plan there. */
  @Test
  void theDriversAccelerationAlongTheRailIsFedForward() {
    RailFollower f = new RailFollower(CFG);
    f.reset(new FieldSpeeds(0, 0, 0));
    FieldSpeeds cmd = f.step(RAIL, new Pose(0.0, 1.0, 0.0), new FieldSpeeds(MAX, 0.0, 0.0), MAX, DT);
    FieldSpeeds acc = f.commandedAcceleration();
    assertEquals(cmd.vxMetersPerSecond() / DT, acc.vxMetersPerSecond(), 1e-9);
    assertEquals(0.0, acc.vyMetersPerSecond(), 1e-9);
    assertTrue(acc.vxMetersPerSecond() <= ROBOT.tractionMetersPerSecondSquared() + 1e-9);
  }

  @Test
  void thePieceAssistOnlyHelpsWithThePieceTheDriverIsHeadingFor() {
    Vec2 intake = new Vec2(0.0, 0.0);
    FieldSpeeds ahead = new FieldSpeeds(3.0, 0.0, 0.0);
    Vec2 tenOff = new Vec2(2.0, 2.0 * Math.tan(Math.toRadians(10)));
    Vec2 fortyOff = new Vec2(1.0, Math.tan(Math.toRadians(40)));
    Vec2 behind = new Vec2(-1.0, 0.0);
    assertEquals(Optional.of(tenOff), PieceAssist.choose(List.of(fortyOff, behind, tenOff), intake, ahead));
    assertTrue(PieceAssist.choose(List.of(fortyOff, behind), intake, ahead).isEmpty());
    assertTrue(PieceAssist.choose(List.of(tenOff), intake, new FieldSpeeds(0.1, 0.0, 0.0)).isEmpty(),
        "no stick, no help");
  }

  /** A piece 25 cm to the left and 2 m ahead: slide left at 25/200 of the speed, times the eagerness. */
  @Test
  void theNudgeAimsTheDriversVelocityAtThePiece() {
    FieldSpeeds stick = new FieldSpeeds(3.0, 0.0, 0.0);
    FieldSpeeds nudge = PieceAssist.nudge(stick, new Vec2(0, 0), new Vec2(2.0, 0.25));
    assertEquals(0.0, nudge.vxMetersPerSecond(), 1e-12, "only sideways to the way the driver is pushing");
    assertEquals(PieceAssist.EAGERNESS * 0.25 / 2.0 * 3.0, nudge.vyMetersPerSecond(), 1e-12);
    // half the stick, half the nudge; no stick, none
    assertEquals(nudge.vyMetersPerSecond() / 2, PieceAssist.nudge(new FieldSpeeds(1.5, 0, 0), new Vec2(0, 0),
        new Vec2(2.0, 0.25)).vyMetersPerSecond(), 1e-12);
    assertEquals(0.0, PieceAssist.nudge(new FieldSpeeds(0, 0, 0), new Vec2(0, 0), new Vec2(2.0, 0.25)).vyMetersPerSecond());
    // never more than the cap, however close or far to the side
    FieldSpeeds big = PieceAssist.nudge(stick, new Vec2(0, 0), new Vec2(0.3, -0.5));
    assertEquals(-PieceAssist.ASSIST_RATIO * 3.0, big.vyMetersPerSecond(), 1e-12);
    // and the robot keeps the driver's speed: the nudge turns the velocity, it doesn't add to it
    RailFollower f = new RailFollower(CFG);
    f.reset(stick);
    f.step(stick, nudge, DT);
    assertEquals(3.0, Math.hypot(f.wanted().vxMetersPerSecond(), f.wanted().vyMetersPerSecond()), 1e-9);
  }
}
