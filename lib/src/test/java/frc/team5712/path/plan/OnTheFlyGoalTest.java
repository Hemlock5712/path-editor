package frc.team5712.path.plan;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Pose;
import frc.team5712.path.sim.SimDrivetrain;
import java.util.Optional;
import java.util.Random;
import org.junit.jupiter.api.Test;

/** An on-the-fly path whose goal comes raw from a camera, and one that may not finish yet. */
class OnTheFlyGoalTest {
  private static final Pose START = new Pose(-6.0, -2.5, 0.0);
  private static final Pose GOAL = new Pose(-3.5, -1.8, 0.4);

  /** Drives until finished or out of time; returns {seconds, end error in metres, 1 if finished}. */
  private static double[] drive(OnTheFly otf, double maxSeconds, java.util.function.DoubleConsumer everyLoop) {
    SimDrivetrain sim = new SimDrivetrain(START, new FieldSpeeds(0, 0, 0), 0.02, 0.04, 0.001);
    assertTrue(otf.start(0.0, START, new FieldSpeeds(0, 0, 0)), () -> "no route: " + otf.failure());
    double t = 0;
    while (t < maxSeconds && !otf.isFinished()) {
      t += OnTheFlyTest.LOOP;
      sim.advanceTo(t);
      everyLoop.accept(t);
      sim.command(otf.step(t, sim.pose(), sim.velocity()));
    }
    sim.command(new FieldSpeeds(0, 0, 0));
    sim.advanceTo(t + 0.3);
    Pose p = sim.pose();
    return new double[] {t, Math.hypot(p.xMeters() - GOAL.xMeters(), p.yMeters() - GOAL.yMeters()),
        otf.isFinished() ? 1 : 0};
  }

  /** The goal re-sent every loop with 2 cm of noise: averaged, the robot settles on the true goal. */
  @Test
  void aNoisyGoalIsAveraged() {
    Random noise = new Random(11);
    OnTheFly otf = new OnTheFly(OnTheFlyTest.PLANNER, OnTheFlyTest.CONFIG, () -> GOAL)
        .withMovingGoal(() -> Optional.of(new Pose(GOAL.xMeters() + 0.02 * noise.nextGaussian(),
            GOAL.yMeters() + 0.02 * noise.nextGaussian(), GOAL.headingRadians())), 0.5);
    double[] r = drive(otf, 10.0, t -> {});
    assertEquals(1.0, r[2], "it settles on a goal that jitters (" + r[0] + " s)");
    assertEquals(0.0, r[1], 0.02);
  }

  @Test
  void theFinishWaitsForItsGate() {
    boolean[] ready = {false};
    OnTheFly otf = new OnTheFly(OnTheFlyTest.PLANNER, OnTheFlyTest.CONFIG, () -> GOAL).finishWhen(() -> ready[0]);
    double[] held = drive(otf, 6.0, t -> {});
    assertEquals(0.0, held[2], "not finished: the gate is shut");
    assertEquals(0.0, held[1], 0.02, "but on the goal");
    OnTheFly again = new OnTheFly(OnTheFlyTest.PLANNER, OnTheFlyTest.CONFIG, () -> GOAL).finishWhen(() -> ready[0]);
    double[] r = drive(again, 10.0, t -> ready[0] = t >= 5.0);
    assertEquals(1.0, r[2]);
    assertEquals(5.0, r[0], 0.1, "finished when the gate opened");
    assertFalse(r[1] > 0.02);
  }
}
