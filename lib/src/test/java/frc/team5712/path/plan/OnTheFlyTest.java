package frc.team5712.path.plan;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.sim.SimDrivetrain;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.Test;

/** On-the-fly paths on the 2026 field, on the noise-free simulated drivetrain (the WPILib runs are in e2eTest). */
class OnTheFlyTest {
  static final double LOOP = 0.02;

  static FieldMap field2026() {
    try {
      java.nio.file.Path f = java.nio.file.Path.of(System.getProperty("fixtures.dir", "../fixtures"), "plan", "field_2026.json");
      return FieldMap.read(f);
    } catch (IOException e) {
      throw new IllegalStateException(e);
    }
  }

  static final Pathfinder PLANNER = new Pathfinder(field2026(), RobotConfig.DEFAULT, PlannerConfig.DEFAULT);
  static final FollowerConfig CONFIG = FollowerConfig.defaults(RobotConfig.DEFAULT).toBuilder()
      .latencySeconds(SimDrivetrain.responseDelaySeconds(0.02, 0.04)).build();

  /** A random pose whose robot centre is free with 10 cm to spare. */
  static Pose randomFree(Random r) {
    while (true) {
      double x = (r.nextDouble() - 0.5) * 15.5;
      double y = (r.nextDouble() - 0.5) * 7.0;
      double h = (r.nextDouble() * 2 - 1) * Math.PI;
      if (PLANNER.isFree(x, y) && PLANNER.clearance(x, y, h) > 0.15) {
        return new Pose(x, y, h);
      }
    }
  }

  record Run(boolean arrived, double endError, double endHeadingDeg, double minClearance, int replans,
      double maxPlanMicros, double seconds, double maxCrossTrack) {}

  /** Plans from start at rest and drives; from shoveAt for shoveSeconds a push adds (dvx, dvy) every loop. */
  static Run drive(Pose start, Pose goal, double shoveAt, double shoveSeconds, double dvx, double dvy) {
    SimDrivetrain sim = new SimDrivetrain(start, new FieldSpeeds(0, 0, 0), 0.02, 0.04, 0.001);
    OnTheFly otf = new OnTheFly(PLANNER, CONFIG, () -> goal);
    double minClear = Double.POSITIVE_INFINITY;
    double maxCt = 0;
    boolean ok = otf.start(0.0, start, new FieldSpeeds(0, 0, 0));
    assertTrue(ok, () -> "no route: " + otf.failure());
    double t = 0;
    for (int k = 0; k < 2000; k++) {
      t = k * LOOP;
      for (int j = 1; j <= 4; j++) { // the true pose every 5 ms
        sim.advanceTo(t - LOOP + j * LOOP / 4);
        Pose p = sim.pose();
        minClear = Math.min(minClear, PLANNER.clearance(p.xMeters(), p.yMeters(), p.headingRadians()));
      }
      if (t >= shoveAt && t < shoveAt + shoveSeconds) {
        sim.shove(dvx, dvy);
      }
      Pose p = sim.pose();
      sim.command(otf.step(t, p, sim.velocity()));
      if (otf.follower() != null) {
        maxCt = Math.max(maxCt, Math.abs(otf.follower().crossTrackMeters()));
      }
      if (otf.isFinished()) {
        sim.command(new FieldSpeeds(0, 0, 0));
        break;
      }
    }
    sim.advanceTo(t + 0.5);
    Pose end = sim.pose();
    return new Run(otf.isFinished(), Math.hypot(end.xMeters() - goal.xMeters(), end.yMeters() - goal.yMeters()),
        Math.toDegrees(Math.abs(Math.IEEEremainder(end.headingRadians() - goal.headingRadians(), 2 * Math.PI))),
        minClear, otf.replans(), otf.maxPlanMicros(), t, maxCt);
  }

  @Test
  void randomRoutesOnThe2026FieldArriveWithoutTouchingAnything() {
    Random r = new Random(2026);
    List<String> bad = new ArrayList<>();
    double worst = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 30; i++) {
      Pose s = randomFree(r);
      Pose g = randomFree(r);
      Run run = drive(s, g, Double.POSITIVE_INFINITY, 0, 0, 0);
      worst = Math.min(worst, run.minClearance);
      if (!run.arrived || run.endError > 0.03 || run.endHeadingDeg > 1.5 || run.minClearance <= 0) {
        bad.add(i + ": " + s + " -> " + g + " " + run);
      }
    }
    System.out.printf("on-the-fly, 30 routes: worst bumper clearance %.3f m%n", worst);
    assertEquals(List.of(), bad);
  }

  @Test
  void aHardShoveIsRecoveredByPlanningAgain() {
    Pose s = new Pose(-6.0, -1.5, 0.0);
    Pose g = new Pose(1.0, -1.5, Math.PI / 2); // through the lower trench
    // a defender pushes the robot sideways (away from the bump) for 0.4 s on the way to the trench
    Run run = drive(s, g, 0.8, 0.4, -0.85, -0.53);
    System.out.println("shove: " + run);
    assertTrue(run.replans >= 1, run.toString());
    assertTrue(run.arrived && run.endError < 0.03, run.toString());
  }

  @Test
  void aStartInsideAnObstacleIsSnappedOut() {
    Pose s = new Pose(-3.4, 0.3, 0.0); // on the blue hub
    PlanResult r = PLANNER.plan(s, new FieldSpeeds(0, 0, 0), new Pose(-6.0, 2.0, 0.0));
    assertTrue(r.ok() && r.startSnapped(), r.toString());
  }

  @Test
  void aHandOverArrivesAlongTheGivenDirection() {
    OnTheFly otf = new OnTheFly(PLANNER, CONFIG, () -> new Pose(1.0, 3.25, 0.0)).arriving(1.5, 0.0);
    assertTrue(otf.start(0.0, new Pose(-6.0, 2.0, 0.0), new FieldSpeeds(0, 0, 0)));
    var path = otf.lastPlan().path();
    var end = path.sample(path.lengthMeters());
    assertEquals(1.0, end.tangentX(), 1e-6);
    assertEquals(1.5, end.metersPerSecond(), 1e-9);
    assertTrue(otf.rollingEnd());
  }

  @Test
  void theFieldFileIsTheEditorsFormat() throws IOException {
    String text = Files.readString(java.nio.file.Path.of(System.getProperty("fixtures.dir", "../fixtures"), "plan",
        "field_2026.json"));
    assertTrue(text.contains("\"length_m\"") && text.contains("\"obstacles\""));
    assertEquals(8, field2026().obstacles().size());
  }
}
