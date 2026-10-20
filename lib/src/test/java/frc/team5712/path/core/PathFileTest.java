package frc.team5712.path.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.tools.FixturePaths;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class PathFileTest {
  static String example() throws IOException {
    try (InputStream in = PathFileTest.class.getResourceAsStream("/example.path.json")) {
      return new String(in.readAllBytes(), StandardCharsets.UTF_8);
    }
  }

  @Test
  void exampleReadsAsTheAutoRoute() throws IOException {
    PathSpec read = PathFile.read(example());
    PathSpec expected = FixturePaths.autoRoute();
    assertEquals(expected.waypoints().size(), read.waypoints().size());
    for (int i = 0; i < read.waypoints().size(); i++) {
      Waypoint a = expected.waypoints().get(i);
      Waypoint b = read.waypoints().get(i);
      assertEquals(a.xMeters(), b.xMeters(), 0.0);
      assertEquals(a.yMeters(), b.yMeters(), 0.0);
      assertEquals(a.hasTangent(), b.hasTangent());
      if (a.hasTangent()) {
        assertEquals(a.tangentRadians(), b.tangentRadians(), 1e-15);
      }
    }
    assertEquals(4, read.rotationTargets().size());
    assertEquals(15.0, read.rotationTargets().get(3).meters(), 0.0);
    assertEquals(Math.PI / 2, read.rotationTargets().get(1).headingRadians(), 1e-15);
    assertEquals(new Vec2(4, 0), read.facePointZones().get(0).point());
    assertEquals(Double.POSITIVE_INFINITY, read.constraintZones().get(0).maxMetersPerSecondSquared());
    assertEquals(0.5, read.endMetersPerSecond(), 0.0);
    assertEquals(
        Path.build(expected, RobotConfig.DEFAULT).totalTimeSeconds(),
        Path.build(read, RobotConfig.DEFAULT).totalTimeSeconds(),
        1e-12);
  }

  @Test
  void writingTheExampleBackGivesTheSameText() throws IOException {
    String text = example().replace("\r\n", "\n");
    assertEquals(text, PathFile.write(PathFile.read(text)));
  }

  @Test
  void everyFixturePathRoundTrips() {
    for (PathSpec spec : FixturePaths.all().values()) {
      String once = PathFile.write(spec);
      String twice = PathFile.write(PathFile.read(once));
      assertEquals(once, twice);
    }
  }

  @Test
  void optionalKeysAreLeftOutAndDefaulted() {
    String text =
        """
        { "schema": 1, "waypoints": [ { "x": 0, "y": 0 }, { "x": 2, "y": 1, "tangentScale": 1.3 } ],
          "editorOnly": { "colour": "red" } }
        """;
    PathSpec spec = PathFile.read(text);
    assertTrue(spec.rotationTargets().isEmpty() && spec.constraintZones().isEmpty());
    assertEquals(Double.POSITIVE_INFINITY, spec.maxMetersPerSecond());
    assertEquals(0.0, spec.startMetersPerSecond(), 0.0);
    assertEquals(1.3, spec.waypoints().get(1).tangentScale(), 0.0);
    String written = PathFile.write(spec);
    assertTrue(!written.contains("maxMetersPerSecond") && !written.contains("tangentDegrees"));
    assertTrue(written.contains("\"tangentScale\": 1.3"));
    assertTrue(!written.contains("editorOnly"), "unknown keys are dropped, not carried");
  }

  @Test
  void keysComeOutInAStableOrder() {
    String written = PathFile.write(FixturePaths.autoRoute());
    String[] order = {
      "\"schema\"", "\"waypoints\"", "\"rotationTargets\"", "\"facePointZones\"", "\"constraintZones\"",
      "\"maxMetersPerSecond\": 4", "\"maxMetersPerSecondSquared\"", "\"startMetersPerSecond\"",
      "\"endMetersPerSecond\""
    };
    int at = -1;
    for (String key : order) {
      int next = written.indexOf(key);
      assertTrue(next > at, key + " out of order");
      at = next;
    }
  }

  @Test
  void numbersAreWrittenToFiveDecimals() {
    PathSpec spec =
        PathSpec.of(List.of(Waypoint.at(1.0 / 3.0, -2.0 / 3.0), Waypoint.at(1e-7, 123.456789012)));
    String written = PathFile.write(spec);
    assertTrue(written.contains("{ \"x\": 0.33333, \"y\": -0.66667 }"), written);
    assertTrue(written.contains("{ \"x\": 0, \"y\": 123.45679 }"), written);
  }

  @Test
  void rejectsWrongSchemaAndMissingFields() {
    assertThrows(IllegalArgumentException.class, () -> PathFile.read("{ \"waypoints\": [] }"));
    assertThrows(
        IllegalArgumentException.class,
        () -> PathFile.read("{ \"schema\": 2, \"waypoints\": [ { \"x\": 0, \"y\": 0 }, { \"x\": 1, \"y\": 0 } ] }"));
    assertThrows(
        IllegalArgumentException.class,
        () -> PathFile.read("{ \"schema\": 1, \"waypoints\": [ { \"x\": 0 }, { \"x\": 1, \"y\": 0 } ] }"));
    assertThrows(
        IllegalArgumentException.class,
        () -> PathFile.read("{ \"schema\": 1, \"waypoints\": [ { \"x\": \"0\", \"y\": 0 } ] }"));
    assertThrows(IllegalArgumentException.class, () -> PathFile.read("[1, 2]"));
  }

  @Test
  void readsAndWritesFiles(@TempDir java.nio.file.Path dir) throws IOException {
    java.nio.file.Path file = dir.resolve("s.path.json");
    PathFile.write(FixturePaths.sCurve(), file);
    assertEquals(PathFile.write(FixturePaths.sCurve()), Files.readString(file));
    assertEquals(4, PathFile.read(file).waypoints().size());
  }

  @Test
  void robotConfigRoundTrips() {
    RobotConfig r = RobotConfig.DEFAULT.withMaxWheelMetersPerSecond(4.2).withFrictionCoefficient(0.9);
    Map<String, Object> tree = RobotFile.toJson(r);
    assertEquals(r, RobotFile.read(Json.write(tree, Json::exact)));
    assertEquals(RobotConfig.DEFAULT, RobotFile.read("{}"));
    RobotConfig partial = RobotFile.read("{ \"massKg\": 50, \"motor\": { \"stallCurrentAmps\": 400 } }");
    assertEquals(50.0, partial.massKg(), 0.0);
    assertEquals(400.0, partial.driveMotor().stallCurrentAmps(), 0.0);
    assertEquals(Motor.KRAKEN_X60_FOC.stallTorqueNewtonMeters(), partial.driveMotor().stallTorqueNewtonMeters(), 0.0);
  }
}
