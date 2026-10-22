package frc.team5712.path.wpilib;

import frc.team5712.path.core.Json;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathEvent;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.RobotFile;
import frc.team5712.path.plan.FieldMap;
import frc.team5712.path.plan.Pathfinder;
import frc.team5712.path.plan.PlannerConfig;
import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
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
 *
 * <p>It also loads {@code field.json} (the field's size and obstacles, as the editor writes it) and builds
 * the {@link Pathfinder}'s visibility graph, once, for on-the-fly paths ({@link Pathfinding}); and the
 * editor's named points ({@code .pathproject.json}), so a path can be planned to one by name. A bad
 * field.json fails loudly here, at boot. Without one the field is empty (walls only), at the 2026 size.
 */
public final class PathLibrary {
  public static final String SUFFIX = ".path.json";

  public static final String FIELD_FILE = "field.json";
  public static final String PROJECT_FILE = ".pathproject.json";
  /** The 2026 field's size (the editor's default), for a deploy folder without field.json. */
  public static final double DEFAULT_FIELD_LENGTH = 16.54;
  public static final double DEFAULT_FIELD_WIDTH = 8.07;

  private final Map<String, Path> paths;
  private final RobotConfig robot;
  private final double buildMillis;
  private Pathfinder pathfinder;
  private boolean fieldFromFile;
  private Map<String, double[]> namedPoints = Map.of();
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
    return loadAll(new File(Filesystem.getDeployDirectory(), "paths"));
  }

  /**
   * Loads every {@code *.path.json} in {@code dir}, with the robot from {@code dir/robot.json} if
   * present (else {@link RobotConfig#DEFAULT}): what {@link #loadAll()} does with the deploy folder.
   */
  public static PathLibrary loadAll(File dir) {
    File robotFile = new File(dir, "robot.json");
    RobotConfig robot = robotFile.isFile() ? RobotFile.read(read(robotFile)) : RobotConfig.DEFAULT;
    PathLibrary lib = loadAll(dir, robot);
    long t0 = System.nanoTime();
    PlannerConfig planner = robotFile.isFile()
        ? PlannerConfig.fromRobotFile(PathFile.asMap(Json.parse(read(robotFile)), "robot.json"))
        : PlannerConfig.DEFAULT;
    lib.loadField(new File(dir, FIELD_FILE), planner);
    lib.loadNamedPoints(new File(dir, PROJECT_FILE));
    lib.fieldMillis = (System.nanoTime() - t0) / 1e6;
    return lib;
  }

  private double fieldMillis;
  private double warmUpMillis;
  /** Plans made at boot so the first real one isn't slowed by the JIT. */
  static final int WARM_UP_QUERIES = 40;

  /** How long the boot warm-up plans took, milliseconds (part of {@link #fieldMillis}). */
  public double warmUpMillis() {
    return warmUpMillis;
  }

  /**
   * Reads {@code file} (field.json) if it exists, else an empty 2026-size field, and builds the pathfinder
   * for this robot with {@code planner}'s bumpers and margin. A bad file throws, naming it.
   */
  public PathLibrary loadField(File file, PlannerConfig planner) {
    FieldMap field;
    fieldFromFile = file.isFile();
    if (fieldFromFile) {
      try {
        field = FieldMap.fromJson(PathFile.asMap(Json.parse(read(file)), "field file"));
      } catch (RuntimeException e) {
        throw new IllegalStateException("field " + file + ": " + e.getMessage(), e);
      }
    } else {
      field = new FieldMap(DEFAULT_FIELD_LENGTH, DEFAULT_FIELD_WIDTH, List.of(), List.of());
    }
    pathfinder = new Pathfinder(field, robot, planner);
    warmUpMillis = pathfinder.warmUp(WARM_UP_QUERIES);
    return this;
  }

  /** The editor's named points ({@code namedPoints} in .pathproject.json), blue frame; none if absent. */
  private void loadNamedPoints(File file) {
    if (!file.isFile()) {
      return;
    }
    Map<String, double[]> points = new LinkedHashMap<>();
    try {
      Object list = PathFile.asMap(Json.parse(read(file)), "project file").get("namedPoints");
      if (list instanceof List<?> l) {
        for (Object o : l) {
          Map<String, Object> m = PathFile.asMap(o, "named point");
          points.put(String.valueOf(m.get("name")), new double[] {
              PathFile.number(m, "x", Double.NaN), PathFile.number(m, "y", Double.NaN)});
        }
      }
    } catch (RuntimeException e) {
      throw new IllegalStateException("named points in " + file + ": " + e.getMessage(), e);
    }
    namedPoints = Collections.unmodifiableMap(points);
  }

  /** The pathfinder over field.json's obstacles, its graph built at load. */
  public Pathfinder pathfinder() {
    if (pathfinder == null) {
      loadField(new File("-"), PlannerConfig.DEFAULT); // a library built from a robot alone: an empty field
    }
    return pathfinder;
  }

  /** True when the obstacles came from a field.json (false: an empty field). */
  public boolean hasFieldFile() {
    return fieldFromFile;
  }

  /** The editor's named points by name, {x, y} in the blue frame. */
  public Map<String, double[]> namedPoints() {
    return namedPoints;
  }

  /** A named point, {x, y} in the blue frame. */
  public double[] namedPoint(String name) {
    double[] p = namedPoints.get(name);
    if (p == null) {
      throw new NoSuchElementException("no named point " + name + " (have " + namedPoints.keySet() + ")");
    }
    return p;
  }

  /** How long reading field.json and building the visibility graph took, milliseconds. */
  public double fieldMillis() {
    return fieldMillis;
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
