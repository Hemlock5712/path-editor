package frc.team5712.path.tools;

import frc.team5712.path.core.Json;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.core.PathSpec;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.VelocityProfile;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Writes the golden vectors, {@code fixtures/<name>.json}, that the TypeScript port must reproduce
 * to 1e-9. Each path goes through its JSON tree first and is built from that tree, which is stored
 * in the fixture as is, so both languages start from the same doubles. Numbers are written in full
 * (shortest round-trip) precision.
 *
 * <p>It also writes {@code fixtures/files/path_files.json}: the path-file text of each
 * {@link PathFileSamples} spec, which the TS writer must print character for character.
 *
 * <p>Run from the repo root: {@code lib/gradlew -p lib fixtures} (writes {@code fixtures/}).
 */
public final class FixtureGen {
  /** Fixture sample spacing, metres. */
  public static final double STEP_METERS = 0.05;

  private FixtureGen() {}

  public static void main(String[] args) throws IOException {
    java.nio.file.Path out = java.nio.file.Path.of(args.length > 0 ? args[0] : "fixtures");
    Files.createDirectories(out);
    for (Map.Entry<String, PathSpec> e : FixturePaths.all().entrySet()) {
      write(out, e.getKey(), e.getValue(), RobotConfig.DEFAULT);
    }
    for (Map.Entry<String, PathSpec> e : FixturePaths.spinLimited().entrySet()) {
      write(out, e.getKey(), e.getValue(), FixturePaths.SPIN_LIMITED_ROBOT);
    }
    java.nio.file.Path files = out.resolve(PATH_FILES);
    Files.createDirectories(files.getParent());
    Files.writeString(files, pathFiles(), StandardCharsets.UTF_8);
    System.out.println("wrote " + files);
    FollowFixtureGen.writeAll(out); // the closed-loop follower traces, fixtures/follow
  }

  /** Where the path-file round-trip fixture goes, relative to the fixtures folder. */
  public static final String PATH_FILES = "files/path_files.json";

  /** The path-file round-trip fixture: {@code {"seed", "files": [{"text"}, ...]}}. */
  public static String pathFiles() {
    Map<String, Object> root = new LinkedHashMap<>();
    root.put("seed", PathFileSamples.SEED);
    List<Object> texts = new ArrayList<>();
    for (PathSpec spec : PathFileSamples.all()) {
      Map<String, Object> m = new LinkedHashMap<>();
      m.put("text", PathFile.write(spec));
      texts.add(m);
    }
    root.put("files", texts);
    return Json.write(root, Json::exact);
  }

  /** The fixture text for one path and robot. */
  public static String fixture(PathSpec spec, RobotConfig robot) {
    Map<String, Object> pathTree = PathFile.asMap(Json.parse(Json.write(PathFile.toJson(spec), Json::exact)), "path");
    return Json.write(ProfileSamples.toJson(pathTree, robot, VelocityProfile.DEFAULT_GRID_METERS, STEP_METERS),
        Json::exact);
  }

  private static void write(java.nio.file.Path out, String name, PathSpec spec, RobotConfig robot)
      throws IOException {
    java.nio.file.Path file = out.resolve(name + ".json");
    Files.writeString(file, fixture(spec, robot), StandardCharsets.UTF_8);
    System.out.println("wrote " + file);
  }
}
