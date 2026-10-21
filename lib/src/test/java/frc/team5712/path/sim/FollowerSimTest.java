package frc.team5712.path.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.tools.FixturePaths;
import frc.team5712.path.tools.FollowFixtureGen;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Map;
import org.junit.jupiter.api.Test;

class FollowerSimTest {
  @Test
  void aCommandArrivesOneLatencyLateThenTheVelocityLags() {
    SimDrivetrain d = new SimDrivetrain(new Pose(0, 0, 0), new FieldSpeeds(0, 0, 0), 0.02, 0.04, 0.001);
    d.command(new FieldSpeeds(1.0, 0.0, 0.0));
    d.advanceTo(0.02);
    assertEquals(0.0, d.velocity().vxMetersPerSecond(), 1e-12, "nothing before the latency");
    d.advanceTo(0.06);
    assertEquals(1.0 - Math.exp(-1.0), d.velocity().vxMetersPerSecond(), 1e-9, "one time constant later");
    d.advanceTo(1.0);
    assertEquals(1.0, d.velocity().vxMetersPerSecond(), 1e-6);
    d.shove(0.0, 2.0);
    assertEquals(2.0, d.velocity().vyMetersPerSecond(), 0.0);
    assertEquals(SimDrivetrain.responseDelaySeconds(0.02, 0.04), 0.07, 1e-12);
  }

  @Test
  void theFollowerFinishesOnTheGoalInTheSim() {
    Path path = Path.build(FixturePaths.sCurve(), RobotConfig.DEFAULT);
    FollowerConfig cfg = FollowerConfig.builder(RobotConfig.DEFAULT).latencySeconds(0.07).build();
    FollowerSim.Trace tr = FollowerSim.run(path, cfg, FollowerSim.Drive.DEFAULT, FieldFrame.BLUE, null, 0.5, 20);
    assertTrue(tr.finishLoop > 0);
    PathSample end = path.sample(path.lengthMeters());
    int last = tr.t.size() - 1;
    assertEquals(end.xMeters(), tr.x.get(last), 0.02);
    assertEquals(end.yMeters(), tr.y.get(last), 0.02);
    assertEquals(tr.finishLoop * FollowerSim.LOOP_SECONDS + 0.5, tr.t.get(last), 1e-9);
    // finishes within about a second of the plan
    assertTrue(tr.finishLoop * FollowerSim.LOOP_SECONDS < path.totalTimeSeconds() + 1.0);
  }

  @Test
  void aShoveIsRecoveredFrom() {
    Path path = Path.build(FixturePaths.sCurve(), RobotConfig.DEFAULT);
    FollowerConfig cfg = FollowerConfig.builder(RobotConfig.DEFAULT).latencySeconds(0.07).build();
    FollowerSim.Trace calm = FollowerSim.run(path, cfg, FollowerSim.Drive.DEFAULT, FieldFrame.BLUE, null, 0.5, 20);
    FollowerSim.Trace hit = FollowerSim.run(path, cfg, FollowerSim.Drive.DEFAULT, FieldFrame.BLUE,
        new FollowerSim.Shove(1.0, 0.0, 3.0), 0.5, 20);
    double worstCalm = calm.crossTrack.stream().mapToDouble(Math::abs).max().orElse(0);
    double worstHit = hit.crossTrack.stream().mapToDouble(Math::abs).max().orElse(0);
    assertTrue(worstHit > worstCalm + 0.03, worstCalm + " vs " + worstHit);
    assertTrue(hit.finishLoop > 0, "recovered and finished");
  }

  /** The committed golden traces must be what this code writes; run `lib/gradlew -p lib fixtures` if not. */
  @Test
  void committedFollowFixturesAreUpToDate() throws IOException {
    java.nio.file.Path dir = java.nio.file.Path.of(System.getProperty("fixtures.dir", "../fixtures"), FollowFixtureGen.DIR);
    for (Map.Entry<String, FollowFixtureGen.Case> e : FollowFixtureGen.cases().entrySet()) {
      String committed = Files.readString(dir.resolve(e.getKey() + ".json"), StandardCharsets.UTF_8);
      assertEquals(FollowFixtureGen.fixture(e.getValue()), committed.replace("\r\n", "\n"),
          "fixtures/follow/" + e.getKey() + ".json is stale: run lib/gradlew -p lib fixtures");
    }
  }
}
