package frc.team5712.path.tools;

import frc.team5712.path.core.ConstraintZone;
import frc.team5712.path.core.FacePointZone;
import frc.team5712.path.core.HeadingMode;
import frc.team5712.path.core.PathSpec;
import frc.team5712.path.core.RobotConfig;
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

  /**
   * The robot for the spin-limited fixtures: the default with the bench's shared spin limits, 540°/s and
   * 720°/s².
   */
  public static final RobotConfig SPIN_LIMITED_ROBOT =
      RobotConfig.DEFAULT.withMaxAngularRadPerSec(deg(540)).withMaxAngularRadPerSecSquared(deg(720));

  /** A 0.2 m hop that turns 80°: the turn, not the distance, sets the time. */
  public static PathSpec shortHop() {
    return PathSpec.of(List.of(Waypoint.at(0.0, 0.0), Waypoint.at(0.2, 0.0)))
        .withRotationTargets(List.of(RotationTarget.atWaypoint(0, 0.0), RotationTarget.atWaypoint(1, deg(80))))
        .withLimits(4.0, 6.0);
  }

  /** A 90° turn on the spot (both waypoints at the same place). */
  public static PathSpec pureRotation() {
    return PathSpec.of(List.of(Waypoint.at(1.0, -0.5), Waypoint.at(1.0, -0.5)))
        .withRotationTargets(List.of(RotationTarget.atWaypoint(0, 0.0), RotationTarget.atWaypoint(1, deg(90))))
        .withLimits(4.0, 6.0);
  }

  /** Fixtures planned with {@link #SPIN_LIMITED_ROBOT}, by name. */
  public static Map<String, PathSpec> spinLimited() {
    Map<String, PathSpec> m = new LinkedHashMap<>();
    m.put("short_hop", shortHop());
    m.put("pure_rotation", pureRotation());
    return m;
  }

  /**
   * A real auto path (Joe's, 2026) whose end tangent points back against the approach: the curve runs 17 cm
   * past the end, stops and comes back. One cusp, which the profile must stop at and the follower must
   * reverse at.
   */
  public static PathSpec joeAutoCusp() {
    return PathSpec.of(List.of(
            new Waypoint(-3.89558, 3.3918, deg(3.54), 0.89, deg(-90)),
            new Waypoint(-0.5, 2.75, deg(-48.92), 1.19, deg(-90)),
            new Waypoint(-0.50378, 0.48882, deg(173.11), 0.44, deg(-90)),
            new Waypoint(-2.12985, 3.35891, deg(151.77), 1.1, deg(0)),
            new Waypoint(-3.91553, 3.39881, deg(18.87), 0.46, deg(0))))
        .withHeadingMode(HeadingMode.HOLD)
        .withLimits(4.0, 6.0);
  }

  /** Fixture paths that double back (planned with the default robot). */
  public static Map<String, PathSpec> cusped() {
    Map<String, PathSpec> m = new LinkedHashMap<>();
    m.put("joe_auto_cusp", joeAutoCusp());
    return m;
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
