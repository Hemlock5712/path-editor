package frc.team5712.path.follow;

import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.tools.FixturePaths;
import org.junit.jupiter.api.Test;

/** Per-loop cost of {@link PathFollower#step}, cold and warm, and of start(). Loose assertions. */
class FollowerPerformanceTest {
  @Test
  void stepTimes() {
    Path path = Path.build(FixturePaths.autoRoute().withEndSpeeds(0, 0), RobotConfig.DEFAULT);
    PathFollower f = new PathFollower(path, FollowerConfig.defaults(RobotConfig.DEFAULT));
    PathSample s0 = path.sample(0);
    Pose start = FieldFrame.RED_HALF_TURN.fromBlue(new Pose(s0.xMeters(), s0.yMeters(), s0.headingRadians()));

    long t0 = System.nanoTime();
    f.start(0, start, new FieldSpeeds(0, 0, 0), FieldFrame.RED_HALF_TURN);
    double startColdUs = (System.nanoTime() - t0) / 1e3;
    long t1 = System.nanoTime();
    f.step(0.02, start, new FieldSpeeds(0, 0, 0));
    double firstStepUs = (System.nanoTime() - t1) / 1e3;

    // warm up on full closed-loop runs, then time a run's worth of steps
    double totalUs = 0;
    int steps = 0;
    double worstUs = 0;
    for (int rep = 0; rep < 60; rep++) {
      KinematicPlant plant = new KinematicPlant(start, new FieldSpeeds(0, 0, 0));
      long ts = System.nanoTime();
      f.start(0, plant.pose(), plant.velocity(), FieldFrame.RED_HALF_TURN);
      double startUs = (System.nanoTime() - ts) / 1e3;
      double t = 0;
      while (!f.isFinished() && t < 15) {
        t += 0.02;
        long a = System.nanoTime();
        FieldSpeeds cmd = f.step(t, plant.pose(), plant.velocity());
        double us = (System.nanoTime() - a) / 1e3;
        plant.step(cmd, 0.02);
        if (rep >= 40) {
          totalUs += us;
          worstUs = Math.max(worstUs, us);
          steps++;
        }
      }
      if (rep == 59) {
        System.out.printf("start() warm %.1f us%n", startUs);
      }
    }
    double meanUs = totalUs / steps;
    System.out.printf(
        "follower on the 16 m route, red: start() cold %.0f us, first step %.0f us; "
            + "warm step mean %.1f us, worst %.1f us over %d steps%n",
        startColdUs, firstStepUs, meanUs, worstUs, steps);
    assertTrue(meanUs < 200.0, "mean step " + meanUs + " us"); // the plan's budget is 0.2 ms
  }
}
