package frc.team5712.path.tools;

import frc.team5712.path.core.Json;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.RobotFile;
import frc.team5712.path.core.VelocityProfile;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.NoSuchFileException;
import java.util.Map;

/**
 * Profiles one path file and prints the samples as JSON (see {@link ProfileSamples}) on stdout, so
 * the Python bench can plot and compare our geometry.
 *
 * <pre>
 * lib/gradlew -p lib jar
 * java -cp lib/build/libs/pathcore.jar frc.team5712.path.tools.ProfileCli my.path.json \
 *     [--robot robot.json] [--step 0.05] [--grid 0.03]
 * </pre>
 *
 * Exit code 2 and a message on stderr for bad arguments, a missing or bad file, or a path that
 * can't be driven.
 */
public final class ProfileCli {
  private ProfileCli() {}

  public static void main(String[] args) {
    try {
      System.out.print(run(args));
    } catch (NoSuchFileException e) {
      System.err.println("ProfileCli: no such file: " + e.getMessage());
      System.exit(2);
    } catch (IllegalArgumentException | IllegalStateException | IOException e) {
      System.err.println("ProfileCli: " + e.getMessage());
      System.exit(2);
    }
  }

  /** Does the work and returns the JSON text (the testable part of main). */
  public static String run(String[] args) throws IOException {
    String pathFile = null;
    String robotFile = null;
    double step = FixtureGen.STEP_METERS;
    double grid = VelocityProfile.DEFAULT_GRID_METERS;
    for (int i = 0; i < args.length; i++) {
      switch (args[i]) {
        case "--robot" -> robotFile = value(args, ++i);
        case "--step" -> step = Double.parseDouble(value(args, ++i));
        case "--grid" -> grid = Double.parseDouble(value(args, ++i));
        case "-h", "--help" -> {
          usage(System.err);
          return "";
        }
        default -> {
          if (pathFile != null) {
            throw new IllegalArgumentException("one path file at a time (got " + args[i] + ")");
          }
          pathFile = args[i];
        }
      }
    }
    if (pathFile == null) {
      usage(System.err);
      throw new IllegalArgumentException("no path file given");
    }
    if (!(step > 0.0) || !(grid > 0.0)) {
      throw new IllegalArgumentException("--step and --grid must be positive");
    }
    Map<String, Object> pathTree = PathFile.asMap(Json.parse(read(pathFile)), "path");
    RobotConfig robot = robotFile == null ? RobotConfig.DEFAULT : RobotFile.read(read(robotFile));
    return Json.write(ProfileSamples.toJson(pathTree, robot, grid, step), Json::exact);
  }

  private static String value(String[] args, int i) {
    if (i >= args.length) {
      throw new IllegalArgumentException(args[i - 1] + " needs a value");
    }
    return args[i];
  }

  private static String read(String file) throws IOException {
    return Files.readString(java.nio.file.Path.of(file), StandardCharsets.UTF_8);
  }

  private static void usage(PrintStream out) {
    out.println(
        "usage: ProfileCli <path.json> [--robot robot.json] [--step metres] [--grid metres]");
  }
}
