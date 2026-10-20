package frc.team5712.path.wpilib;

import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.RobotFile;
import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Collections;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.TreeMap;
import org.wpilib.system.Filesystem;

/**
 * Every path in {@code deploy/paths}, built once in robot init. Starting a path later does no path
 * work: it follows the object built here, on either alliance.
 *
 * <pre>
 * // Robot constructor
 * PathLibrary paths = PathLibrary.loadAll();
 * Path toReef = paths.get("ToReef");
 * </pre>
 */
public final class PathLibrary {
  public static final String SUFFIX = ".path.json";

  private final Map<String, Path> paths;
  private final RobotConfig robot;
  private final double buildMillis;

  private PathLibrary(Map<String, Path> paths, RobotConfig robot, double buildMillis) {
    this.paths = Collections.unmodifiableMap(paths);
    this.robot = robot;
    this.buildMillis = buildMillis;
  }

  /**
   * Loads {@code deploy/paths/*.path.json}, with the robot from {@code deploy/paths/robot.json} if
   * present (else {@link RobotConfig#DEFAULT}).
   */
  public static PathLibrary loadAll() {
    File dir = new File(Filesystem.getDeployDirectory(), "paths");
    File robotFile = new File(dir, "robot.json");
    RobotConfig robot = robotFile.isFile() ? RobotFile.read(read(robotFile)) : RobotConfig.DEFAULT;
    return loadAll(dir, robot);
  }

  /** Loads and builds every {@code *.path.json} in {@code dir}. A bad file fails loudly, at boot. */
  public static PathLibrary loadAll(File dir, RobotConfig robot) {
    long t0 = System.nanoTime();
    Map<String, Path> built = new TreeMap<>();
    File[] files = dir.listFiles((d, name) -> name.endsWith(SUFFIX));
    if (files != null) {
      for (File f : files) {
        String name = f.getName().substring(0, f.getName().length() - SUFFIX.length());
        try {
          built.put(name, Path.build(PathFile.read(read(f)), robot));
        } catch (RuntimeException e) {
          throw new IllegalStateException("path " + f + ": " + e.getMessage(), e);
        }
      }
    }
    return new PathLibrary(built, robot, (System.nanoTime() - t0) / 1e6);
  }

  private static String read(File f) {
    try {
      return Files.readString(f.toPath(), StandardCharsets.UTF_8);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  /** The path built from {@code <name>.path.json}. */
  public Path get(String name) {
    Path p = paths.get(name);
    if (p == null) {
      throw new NoSuchElementException("no path named " + name + " (have " + paths.keySet() + ")");
    }
    return p;
  }

  public Map<String, Path> all() {
    return paths;
  }

  public RobotConfig robot() {
    return robot;
  }

  /** How long loading and building took, milliseconds. */
  public double buildMillis() {
    return buildMillis;
  }
}
