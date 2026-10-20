package frc.team5712.path.tools;

import frc.team5712.path.core.ConstraintZone;
import frc.team5712.path.core.FacePointZone;
import frc.team5712.path.core.PathSpec;
import frc.team5712.path.core.RotationTarget;
import frc.team5712.path.core.Vec2;
import frc.team5712.path.core.Waypoint;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The paths the golden vectors are made from, also used by the tests. Field coordinates have the
 * origin at the centre (WPILib 2027), so x runs about ±8.3 m and y about ±4 m.
 */
public final class FixturePaths {
  private FixturePaths() {}

  private static double deg(double degrees) {
    return Math.toRadians(degrees);
  }

  /** A 3 m straight line, facing the direction of travel. */
  public static PathSpec straight() {
    return PathSpec.of(List.of(Waypoint.at(-1.5, 0.0, 0.0), Waypoint.at(1.5, 0.0, 0.0)))
        .withLimits(4.0, 6.0);
  }

  /** An S-curve with a heading change in the middle. */
  public static PathSpec sCurve() {
    return PathSpec.of(
            List.of(
                Waypoint.at(-4.0, -1.0, 0.0),
                Waypoint.at(-1.0, 0.5),
                Waypoint.at(2.0, -0.5),
                Waypoint.at(4.0, 1.0, 0.0)))
        .withRotationTargets(
            List.of(RotationTarget.atWaypoint(0.0, 0.0), RotationTarget.atWaypoint(3.0, deg(90))))
        .withLimits(4.0, 6.0);
  }

  /** A tight 180° turn, holding one heading throughout. */
  public static PathSpec hairpin() {
    return PathSpec.of(
            List.of(
                Waypoint.at(-3.0, -1.0, 0.0),
                Waypoint.at(0.0, -1.0),
                Waypoint.at(1.0, 0.0),
                Waypoint.at(0.0, 1.0),
                Waypoint.at(-3.0, 1.0, deg(180))))
        .withRotationTargets(List.of(RotationTarget.atWaypoint(0.0, deg(-90))))
        .withLimits(4.0, 6.0);
  }

  /** A slalom through five gates with a slow zone. */
  public static PathSpec slalom() {
    return PathSpec.of(
            List.of(
                Waypoint.at(-5.0, 0.0, 0.0),
                Waypoint.at(-3.0, 1.0),
                Waypoint.at(-1.0, -1.0),
                Waypoint.at(1.0, 1.0),
                Waypoint.at(3.0, -1.0),
                Waypoint.at(5.0, 0.0, 0.0)))
        .withConstraintZones(List.of(new ConstraintZone(2.5, 3.5, 2.0, 3.0)))
        .withLimits(4.5, 7.0);
  }

  /**
   * A 16 m auto route with 10 waypoints, rotation targets, a face-point zone, a slow zone and a
   * rolling end. This is also {@code example.path.json} and the performance test's path.
   */
  public static PathSpec autoRoute() {
    return PathSpec.of(
            List.of(
                Waypoint.at(-5.2, -2.5, 0.0),
                Waypoint.at(-3.6, -1.5),
                Waypoint.at(-2.0, -2.4),
                Waypoint.at(-0.4, -1.2),
                Waypoint.at(0.8, 0.5),
                Waypoint.at(0.0, 2.2),
                Waypoint.at(-1.6, 2.6),
                Waypoint.at(-3.2, 1.6),
                Waypoint.at(-4.4, 2.5),
                Waypoint.at(-5.44, 3.0, deg(180))))
        .withRotationTargets(
            List.of(
                RotationTarget.atWaypoint(0.0, 0.0),
                RotationTarget.atWaypoint(3.5, deg(90)),
                RotationTarget.atWaypoint(6.0, deg(180)),
                RotationTarget.atMeters(15.0, deg(180))))
        .withFacePointZones(List.of(new FacePointZone(4.0, 5.5, new Vec2(4.0, 0.0), 0.5)))
        .withConstraintZones(
            List.of(new ConstraintZone(7.0, 8.0, 2.0, Double.POSITIVE_INFINITY)))
        .withLimits(4.0, 6.0)
        .withEndSpeeds(0.0, 0.5);
  }

  /** Automatic end tangents, uneven spacing, tangent scales and a moving start. */
  public static PathSpec tangentScales() {
    return PathSpec.of(
            List.of(
                Waypoint.at(-2.0, -2.0),
                Waypoint.at(-1.5, -1.7).withTangentScale(0.7),
                Waypoint.at(1.5, -0.5),
                new Waypoint(2.0, 1.5, deg(100), 1.4),
                Waypoint.at(0.5, 2.5)))
        .withEndSpeeds(1.2, 0.0)
        .withLimits(3.5, 5.0);
  }

  /** All fixture paths by name, in a fixed order. */
  public static Map<String, PathSpec> all() {
    Map<String, PathSpec> m = new LinkedHashMap<>();
    m.put("straight", straight());
    m.put("s_curve", sCurve());
    m.put("hairpin", hairpin());
    m.put("slalom", slalom());
    m.put("auto_route", autoRoute());
    m.put("tangent_scales", tangentScales());
    return m;
  }
}
