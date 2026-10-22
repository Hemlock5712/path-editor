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
      "\"schema\"", "\"headingMode\"", "\"waypoints\"", "\"rotationTargets\"", "\"facePointZones\"", "\"constraintZones\"",
      "\"events\"", "\"maxMetersPerSecond\": 4", "\"maxMetersPerSecondSquared\"", "\"startMetersPerSecond\"",
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
        () -> PathFile.read("{ \"schema\": 4, \"waypoints\": [ { \"x\": 0, \"y\": 0 }, { \"x\": 1, \"y\": 0 } ] }"));
    assertThrows(
        IllegalArgumentException.class,
        () -> PathFile.read("{ \"schema\": 1.5, \"waypoints\": [ { \"x\": 0, \"y\": 0 }, { \"x\": 1, \"y\": 0 } ] }"));
    assertThrows(
        IllegalArgumentException.class,
        () -> PathFile.read("{ \"schema\": 1, \"waypoints\": [ { \"x\": 0 }, { \"x\": 1, \"y\": 0 } ] }"));
    assertThrows(
        IllegalArgumentException.class,
        () -> PathFile.read("{ \"schema\": 1, \"waypoints\": [ { \"x\": \"0\", \"y\": 0 } ] }"));
    assertThrows(IllegalArgumentException.class, () -> PathFile.read("[1, 2]"));
  }

  static String exampleV1() throws IOException {
    try (InputStream in = PathFileTest.class.getResourceAsStream("/example.v1.path.json")) {
      return new String(in.readAllBytes(), StandardCharsets.UTF_8);
    }
  }

  @Test
  void schemaOneFilesStillRead() throws IOException {
    PathSpec v1 = PathFile.read(exampleV1());
    PathSpec v2 = PathFile.read(example());
    assertTrue(v1.events().isEmpty());
    assertEquals(v2.waypoints(), v1.waypoints());
    assertEquals(v2.rotationTargets(), v1.rotationTargets());
    String written = PathFile.write(v1);
    assertTrue(written.startsWith("{\n  \"schema\": 3,\n  \"headingMode\": \"faceTravel\","), "writes the current schema");
    assertTrue(written.contains("\"events\": []"), written);
  }

  @Test
  void eventsReadInFileOrder() throws IOException {
    List<PathEvent> events = PathFile.read(example()).events();
    assertEquals(4, events.size());
    assertEquals(PathEvent.atWaypoint("intakeDown", 0.5), events.get(0));
    assertEquals(PathEvent.atMeters("spinUp", 6.25), events.get(1));
    assertEquals("score \"L4\"", events.get(3).name());
    assertEquals(15.98765, events.get(3).meters(), 0.0);
    PathSpec spec = PathFile.read(example());
    Path path = Path.build(spec, RobotConfig.DEFAULT);
    assertEquals(path.geometry().sAtWaypoint(0.5), events.get(0).sMeters(path.geometry()), 0.0);
    assertEquals(6.25, events.get(1).sMeters(path.geometry()), 0.0);
  }

  @Test
  void rejectsBadEvents() {
    String head = "{ \"schema\": 2, \"waypoints\": [ { \"x\": 0, \"y\": 0 }, { \"x\": 1, \"y\": 0 } ], \"events\": ";
    assertThrows(IllegalArgumentException.class, () -> PathFile.read(head + "[ { \"atMeters\": 1 } ] }"));
    assertThrows(IllegalArgumentException.class, () -> PathFile.read(head + "[ { \"name\": 3, \"atMeters\": 1 } ] }"));
    assertThrows(IllegalArgumentException.class, () -> PathFile.read(head + "[ { \"name\": \" \", \"atMeters\": 1 } ] }"));
    assertThrows(IllegalArgumentException.class, () -> PathFile.read(head + "[ { \"name\": \"a\" } ] }"));
    assertThrows(IllegalArgumentException.class,
        () -> PathFile.read(head + "[ { \"name\": \"a\", \"atMeters\": 1, \"atWaypoint\": 0 } ] }"));
    assertThrows(IllegalArgumentException.class, () -> PathFile.read(head + "{ \"name\": \"a\" } }"));
  }

  /** The same specs PathFileSamples gives the TS port: each file reads back to the same text. */
  @Test
  void everySampleFileRoundTrips() {
    int events = 0;
    for (PathSpec spec : frc.team5712.path.tools.PathFileSamples.all()) {
      String once = PathFile.write(spec);
      PathSpec read = PathFile.read(once);
      assertEquals(once, PathFile.write(read));
      assertEquals(spec.events().size(), read.events().size());
      for (int i = 0; i < spec.events().size(); i++) {
        assertEquals(spec.events().get(i).name(), read.events().get(i).name());
        assertEquals(spec.events().get(i).byWaypoint(), read.events().get(i).byWaypoint());
      }
      events += spec.events().size();
    }
    assertTrue(events > 20, "the samples exercise events: " + events);
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
    RobotConfig tall = RobotConfig.DEFAULT.withCenterOfMassHeightMeters(0.3).withTipDistanceMeters(0.254)
        .withTractionRule(RobotConfig.TractionRule.EQUAL_PUSH);
    assertEquals(tall, RobotFile.read(Json.write(RobotFile.toJson(tall), Json::exact)));
    assertEquals(false, RobotFile.toJson(RobotConfig.DEFAULT).containsKey("tipDistanceMeters"), "a square base writes none");
    RobotConfig partial = RobotFile.read("{ \"massKg\": 50, \"motor\": { \"stallCurrentAmps\": 400 } }");
    assertEquals(50.0, partial.massKg(), 0.0);
    assertEquals(400.0, partial.driveMotor().stallCurrentAmps(), 0.0);
    assertEquals(Motor.KRAKEN_X60_FOC.stallTorqueNewtonMeters(), partial.driveMotor().stallTorqueNewtonMeters(), 0.0);
  }

  static String exampleV2() throws IOException {
    try (InputStream in = PathFileTest.class.getResourceAsStream("/example.v2.path.json")) {
      return new String(in.readAllBytes(), StandardCharsets.UTF_8);
    }
  }

  @Test
  void headingModesByFileSchema() throws IOException {
    assertEquals(HeadingMode.FACE_TRAVEL, PathFile.read(exampleV1()).headingMode(), "schema 1 did face travel");
    assertEquals(HeadingMode.FACE_TRAVEL, PathFile.read(exampleV2()).headingMode(), "schema 2 did face travel");
    assertEquals(HeadingMode.HOLD, PathFile.read(example()).headingMode());
    String bare = "{ \"schema\": 3, \"waypoints\": [ { \"x\": 0, \"y\": 0 }, { \"x\": 1, \"y\": 0 } ] }";
    assertEquals(HeadingMode.HOLD, PathFile.read(bare).headingMode());
    assertThrows(IllegalArgumentException.class, () -> PathFile.read(bare.replace("\"schema\": 3,", "\"schema\": 3, \"headingMode\": \"spin\",")));
  }

  @Test
  void waypointHeadingsRoundTripAndBecomeTargets() {
    PathSpec spec = PathSpec.of(List.of(new Waypoint(0, 0, Double.NaN, 1.0, Math.PI / 2), Waypoint.at(2, 1),
        new Waypoint(4, 0, Double.NaN, 1.0, -Math.PI / 4)));
    String text = PathFile.write(spec);
    assertTrue(text.contains("{ \"x\": 0, \"y\": 0, \"headingDegrees\": 90 }"), text);
    assertTrue(text.contains("{ \"x\": 2, \"y\": 1 }"), text);
    assertEquals(text, PathFile.write(PathFile.read(text)));
    Path path = Path.build(spec, RobotConfig.DEFAULT);
    assertEquals(2, path.spec().rotationTargets().size());
    assertEquals(Math.PI / 2, path.sample(0).headingRadians(), 1e-12);
    assertEquals(-Math.PI / 4, path.sample(path.lengthMeters()).headingRadians(), 1e-12);
  }

  @Test
  void holdKeepsTheStartDirectionWhereFaceTravelTurns() {
    List<Waypoint> curve = List.of(Waypoint.at(0, 0), Waypoint.at(2, 1), Waypoint.at(4, 0));
    Path hold = Path.build(PathSpec.of(curve), RobotConfig.DEFAULT);
    Path face = Path.build(PathSpec.of(curve).withHeadingMode(HeadingMode.FACE_TRAVEL), RobotConfig.DEFAULT);
    double mid = hold.lengthMeters() / 2;
    assertEquals(hold.sample(0).headingRadians(), hold.sample(mid).headingRadians(), 0.0);
    PathSample m = face.sample(mid);
    assertEquals(Math.atan2(m.tangentY(), m.tangentX()), m.headingRadians(), 1e-12);
  }
}
