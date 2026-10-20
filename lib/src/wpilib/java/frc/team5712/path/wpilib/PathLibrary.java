package frc.team5712.path.wpilib;

import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathEvent;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.RobotFile;
import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;
import org.wpilib.system.Filesystem;

/**
 * Every path in {@code deploy/paths}, built once in robot init. Starting a path later does no path
 * work: it follows the object built here, on either alliance.
 *
 * <pre>
 * // Robot constructor
 * PathLibrary paths = PathLibrary.loadAll();
 * Path toReef = paths.get("ToReef");
 *
 * // Events authored in the editor fire by name; bind each name to a command once, for every path
 * paths.bindEvent("intakeDown", intake.lower())
 *      .bindEvent("shoot", shooter.shoot());
 * Command go = new FollowPathCommand("ToReef", drive, follower, ...).withEvents(paths);
 * </pre>
 *
 * {@link #unboundEventNames} lists the names the path files use that have no command yet, which is
 * where a typo between the editor and the code shows up.
 */
public final class PathLibrary {
  public static final String SUFFIX = ".path.json";

  private final Map<String, Path> paths;
  private final RobotConfig robot;
  private final double buildMillis;
  private final Map<String, org.wpilib.command3.Command> eventCommands = new HashMap<>();
  private final Map<String, org.wpilib.command2.Command> eventCommandsV2 = new HashMap<>();

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

  /** Runs {@code command} (Commands v3) whenever a path followed with {@code withEvents(this)} fires {@code eventName}. */
  public PathLibrary bindEvent(String eventName, org.wpilib.command3.Command command) {
    eventCommands.put(eventName, command);
    return this;
  }

  /** The same for Commands v2 ({@link FollowPathCommandV2}). */
  public PathLibrary bindEvent(String eventName, org.wpilib.command2.Command command) {
    eventCommandsV2.put(eventName, command);
    return this;
  }

  /** The Commands v3 command bound to an event name, if any. */
  public Optional<org.wpilib.command3.Command> eventCommand(String eventName) {
    return Optional.ofNullable(eventCommands.get(eventName));
  }

  /** The Commands v2 command bound to an event name, if any. */
  public Optional<org.wpilib.command2.Command> eventCommandV2(String eventName) {
    return Optional.ofNullable(eventCommandsV2.get(eventName));
  }

  /** Every event name the loaded path files use, sorted. */
  public Set<String> eventNames() {
    Set<String> names = new TreeSet<>();
    for (Path p : paths.values()) {
      for (PathEvent e : p.spec().events()) {
        names.add(e.name());
      }
    }
    return names;
  }

  /** Event names in the path files with no command bound in either framework, sorted. */
  public Set<String> unboundEventNames() {
    Set<String> names = eventNames();
    names.removeAll(eventCommands.keySet());
    names.removeAll(eventCommandsV2.keySet());
    return names;
  }

  /** How long loading and building took, milliseconds. */
  public double buildMillis() {
    return buildMillis;
  }
}
