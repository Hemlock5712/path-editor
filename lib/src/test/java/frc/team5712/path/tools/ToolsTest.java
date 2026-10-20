package frc.team5712.path.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.Json;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.VelocityProfile;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class ToolsTest {
  @Test
  void stationsCoverThePathEvenly() {
    double[] s = ProfileSamples.stations(1.0, 0.3);
    assertEquals(5, s.length);
    assertEquals(0.9, s[3], 1e-15);
    assertEquals(1.0, s[4], 0.0);
    assertEquals(3, ProfileSamples.stations(0.1, 0.05).length); // an exact multiple adds no sliver
  }

  @Test
  void cliProfilesAFile(@TempDir Path dir) throws IOException {
    Path path = dir.resolve("s.path.json");
    PathFile.write(FixturePaths.sCurve(), path);
    Path robot = dir.resolve("robot.json");
    Files.writeString(robot, "{ \"frictionCoefficient\": 0.5 }");
    Map<?, ?> out =
        (Map<?, ?>) Json.parse(ProfileCli.run(new String[] {path.toString(), "--robot", robot.toString(), "--step", "0.1"}));
    Map<?, ?> samples = (Map<?, ?>) out.get("samples");
    assertEquals(ProfileSamples.COLUMNS, List.copyOf(samples.keySet()));
    List<?> s = (List<?>) samples.get("s");
    assertEquals(0.1, (Double) s.get(1), 1e-15);
    double length = (Double) out.get("lengthMeters");
    assertEquals(length, (Double) s.get(s.size() - 1), 0.0);
    Map<?, ?> robotUsed = (Map<?, ?>) ((Map<?, ?>) out.get("inputs")).get("robot");
    assertEquals(0.5, (Double) robotUsed.get("frictionCoefficient"), 0.0);
    // less grip than the default, so slower than the default profile
    Map<?, ?> normal = (Map<?, ?>) Json.parse(ProfileCli.run(new String[] {path.toString()}));
    assertTrue((Double) out.get("totalTimeSeconds") > (Double) normal.get("totalTimeSeconds"));
  }

  @Test
  void cliRejectsBadArguments() {
    assertThrows(IllegalArgumentException.class, () -> ProfileCli.run(new String[] {}));
    assertThrows(IllegalArgumentException.class, () -> ProfileCli.run(new String[] {"a", "b"}));
    assertThrows(IllegalArgumentException.class, () -> ProfileCli.run(new String[] {"a", "--step"}));
    assertThrows(IllegalArgumentException.class, () -> ProfileCli.run(new String[] {"a", "--grid", "0"}));
  }

  /** The committed golden vectors must be what this code writes; run `fixtures` if this fails. */
  @Test
  void committedFixturesAreUpToDate() throws IOException {
    Path dir = Path.of(System.getProperty("fixtures.dir", "../fixtures"));
    for (var e : FixturePaths.all().entrySet()) {
      Map<String, Object> pathTree =
          PathFile.asMap(Json.parse(Json.write(PathFile.toJson(e.getValue()), Json::exact)), "path");
      String expected =
          Json.write(
              ProfileSamples.toJson(
                  pathTree, RobotConfig.DEFAULT, VelocityProfile.DEFAULT_GRID_METERS, FixtureGen.STEP_METERS),
              Json::exact);
      String committed = Files.readString(dir.resolve(e.getKey() + ".json"), StandardCharsets.UTF_8);
      assertEquals(expected, committed.replace("\r\n", "\n"), e.getKey() + ".json is stale: run lib/gradlew -p lib fixtures");
    }
  }
}
