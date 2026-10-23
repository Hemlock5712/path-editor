package frc.team5712.path.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.tools.FixturePaths;
import java.util.List;
import org.junit.jupiter.api.Test;

class PathTest {
  private static final Path PATH = Path.build(FixturePaths.autoRoute(), RobotConfig.DEFAULT);

  @Test
  void sampleAgreesWithItsParts() {
    for (double s = 0; s <= PATH.lengthMeters(); s += 0.77) {
      PathSample p = PATH.sample(s);
      GeometrySample g = PATH.geometry().sample(s);
      HeadingSample h = PATH.heading().sample(s);
      assertEquals(g.xMeters(), p.xMeters(), 0.0);
      assertEquals(g.curvature(), p.curvature(), 0.0);
      assertEquals(h.radians(), p.headingRadians(), 0.0);
      assertEquals(PATH.profile().velocityAt(s), p.metersPerSecond(), 0.0);
      assertEquals(PATH.profile().timeAt(s), p.seconds(), 0.0);
    }
  }

  @Test
  void sampleByTimeRunsStartToEnd() {
    PathSample start = PATH.sampleAtTime(0);
    PathSample end = PATH.sampleAtTime(PATH.totalTimeSeconds());
    assertEquals(-5.2, start.xMeters(), 1e-9);
    assertEquals(-5.44, end.xMeters(), 1e-4);
    assertEquals(3.0, end.yMeters(), 1e-4);
    PathSample mid = PATH.sampleAtTime(3.0);
    assertEquals(3.0, mid.seconds(), 1e-9);
  }

  @Test
  void fieldSpeedsPointAlongTheTangent() {
    PathSample p = PATH.sample(6.0);
    FieldSpeeds v = p.fieldSpeeds();
    assertEquals(p.metersPerSecond(), Math.hypot(v.vxMetersPerSecond(), v.vyMetersPerSecond()), 1e-12);
    assertEquals(p.metersPerSecond() * p.headingRadiansPerMeter(), v.omegaRadPerSec(), 1e-12);
    assertEquals(new Pose(p.xMeters(), p.yMeters(), p.headingRadians()), p.pose());
  }

  @Test
  void specValidation() {
    List<Waypoint> two = List.of(Waypoint.at(0, 0), Waypoint.at(1, 0));
    assertThrows(IllegalArgumentException.class, () -> PathSpec.of(List.of(Waypoint.at(0, 0))));
    assertThrows(IllegalArgumentException.class, () -> PathSpec.of(two).withLimits(0, 1));
    assertThrows(IllegalArgumentException.class, () -> PathSpec.of(two).withEndSpeeds(-1, 0));
    assertThrows(IllegalArgumentException.class, () -> new ConstraintZone(1, 1, 1, 1));
    assertThrows(IllegalArgumentException.class, () -> new FacePointZone(2, 1, Vec2.ZERO, 0.5));
    assertThrows(IllegalArgumentException.class, () -> new RotationTarget(1, 1, 0));
    assertThrows(IllegalArgumentException.class, () -> new Waypoint(0, 0, Double.NaN, 0));
  }
}
