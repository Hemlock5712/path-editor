package frc.team5712.path.core;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * The robot's outline seen from above, as a rectangle around its centre: how far it reaches to the front
 * (along its heading), the back, the left and the right. {@link #BUMPERS} is the bumpers alone; any other
 * is the bumpers plus a mechanism that sticks out (an intake, an arm), named in {@code robot.json}:
 *
 * <pre>
 * "footprints": [
 *   { "name": "intakeOut", "frontMeters": 0.3, "retractSeconds": 0.4 },
 *   { "name": "armUp", "frontMeters": 0.15, "backMeters": 0.1 }
 * ]
 * </pre>
 *
 * Each side is how much further than the bumpers it reaches (absent = 0). A path event can switch the
 * footprint ({@link PathEvent#footprint}); the editor checks the outline in force along the path, and the
 * pathfinder plans with whichever one it's given.
 *
 * @param name what events and {@code PathSpec#startFootprint} call it
 * @param retractSeconds how long the mechanism takes to come back in: after an event switches away from
 *     this footprint, the checks keep it this long
 */
public record Footprint(
    String name,
    double frontMeters,
    double backMeters,
    double leftMeters,
    double rightMeters,
    double retractSeconds) {

  /** The bumpers alone: every path starts with them unless it says otherwise. */
  public static final String BUMPERS = "bumpers";

  public Footprint {
    if (name == null || name.isBlank()) {
      throw new IllegalArgumentException("a footprint needs a name");
    }
    if (!(frontMeters > -backMeters) || !(leftMeters > -rightMeters) || !Double.isFinite(frontMeters + backMeters)
        || !Double.isFinite(leftMeters + rightMeters)) {
      throw new IllegalArgumentException("footprint " + name + " must have a positive, finite size");
    }
    if (!(retractSeconds >= 0) || !Double.isFinite(retractSeconds)) {
      throw new IllegalArgumentException("footprint " + name + ": retractSeconds must be >= 0");
    }
  }

  /** The bumpers: a length × width rectangle centred on the robot. */
  public static Footprint bumpers(double lengthMeters, double widthMeters) {
    return new Footprint(BUMPERS, lengthMeters / 2, lengthMeters / 2, widthMeters / 2, widthMeters / 2, 0.0);
  }

  /** How far the outline reaches from the robot's centre in any heading: the far corner. */
  public double reachMeters() {
    return Math.hypot(2 * Math.max(frontMeters, backMeters), 2 * Math.max(leftMeters, rightMeters)) / 2;
  }

  /** The smallest rectangle around both (each side the further of the two), named after this one. */
  public Footprint union(Footprint other) {
    return new Footprint(name, Math.max(frontMeters, other.frontMeters), Math.max(backMeters, other.backMeters),
        Math.max(leftMeters, other.leftMeters), Math.max(rightMeters, other.rightMeters),
        Math.max(retractSeconds, other.retractSeconds));
  }

  /**
   * The robot file's footprints: {@link #bumpers} first (from {@code bumperLengthMeters} and {@code
   * bumperWidthMeters}, 0.85 m if absent), then each of {@code footprints} as the bumpers plus its extensions.
   * Names must be unique and none may be {@code "bumpers"}.
   */
  public static List<Footprint> fromRobotFile(Map<String, Object> robot) {
    Footprint base = bumpers(PathFile.number(robot, "bumperLengthMeters", 0.85),
        PathFile.number(robot, "bumperWidthMeters", 0.85));
    List<Footprint> out = new ArrayList<>(List.of(base));
    Set<String> names = new HashSet<>(Set.of(BUMPERS));
    for (Object o : PathFile.list(robot, "footprints")) {
      Map<String, Object> m = PathFile.asMap(o, "footprint");
      if (!(m.get("name") instanceof String name)) {
        throw new IllegalArgumentException("a footprint needs a 'name' string");
      }
      if (!names.add(name)) {
        throw new IllegalArgumentException("two footprints are called " + name
            + (name.equals(BUMPERS) ? " (\"bumpers\" is the bumpers alone)" : ""));
      }
      out.add(extended(base, name, extension(m, "frontMeters"), extension(m, "backMeters"),
          extension(m, "leftMeters"), extension(m, "rightMeters"), extension(m, "retractSeconds")));
    }
    return List.copyOf(out);
  }

  /** The bumpers plus a mechanism reaching this much further on each side. */
  public static Footprint extended(Footprint bumpers, String name, double front, double back, double left,
      double right, double retractSeconds) {
    return new Footprint(name, bumpers.frontMeters + front, bumpers.backMeters + back, bumpers.leftMeters + left,
        bumpers.rightMeters + right, retractSeconds);
  }

  private static double extension(Map<String, Object> m, String key) {
    double v = PathFile.number(m, key, 0.0);
    if (!(v >= 0) || !Double.isFinite(v)) {
      throw new IllegalArgumentException("footprint " + m.get("name") + ": '" + key + "' must be >= 0");
    }
    return v;
  }
}
