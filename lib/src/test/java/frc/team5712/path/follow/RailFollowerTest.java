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
    assertEquals(PieceAssist.ASSIST_RATIO * 3.0, PieceAssist.maxCrossSpeed(ahead), 1e-12);
    Rail rail = PieceAssist.railThrough(tenOff, ahead);
    assertEquals(0.0, rail.directionRadians(), 1e-12, "the rail runs the way the driver is pushing");
    assertTrue(!rail.holdsHeading(), "and the driver still turns");
  }
}
