package frc.team5712.path.core;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Reads and writes {@code *.path.json}. The writer prints every number to 5 decimals with keys in a
 * fixed order, so a small edit makes a small git diff. Angles are in degrees in the file and radians
 * everywhere else.
 *
 * <pre>
 * {
 *   "schema": 1,
 *   "waypoints": [
 *     { "x": -6, "y": -2, "tangentDegrees": 0 },
 *     { "x": -3, "y": -1, "tangentScale": 1.2 }
 *   ],
 *   "rotationTargets": [
 *     { "waypoint": 0, "degrees": 0 },
 *     { "meters": 2.5, "degrees": 90 }
 *   ],
 *   "facePointZones": [
 *     { "fromWaypoint": 1, "toWaypoint": 2, "x": 0, "y": 0, "blendMeters": 0.5 }
 *   ],
 *   "constraintZones": [
 *     { "fromWaypoint": 1.5, "toWaypoint": 2, "maxMetersPerSecond": 1.5 }
 *   ],
 *   "maxMetersPerSecond": 4,
 *   "maxMetersPerSecondSquared": 6,
 *   "startMetersPerSecond": 0,
 *   "endMetersPerSecond": 0
 * }
 * </pre>
 *
 * Optional keys: {@code tangentDegrees}, {@code tangentScale} (default 1), a zone's two limits and
 * the path-wide maxima (absent = no cap). Unknown keys are ignored, so editor-only data can't break
 * the robot.
 */
public final class PathFile {
  /** The schema this code writes and reads. */
  public static final int SCHEMA = 1;

  private static final double DEGREES = Math.PI / 180.0;

  private PathFile() {}

  public static PathSpec read(java.nio.file.Path file) throws IOException {
    return read(Files.readString(file, StandardCharsets.UTF_8));
  }

  public static PathSpec read(String json) {
    return fromJson(asMap(Json.parse(json), "path"));
  }

  public static String write(PathSpec spec) {
    return Json.write(toJson(spec), Json::fixed5);
  }

  public static void write(PathSpec spec, java.nio.file.Path file) throws IOException {
    Files.writeString(file, write(spec), StandardCharsets.UTF_8);
  }

  // ------------------------------------------------------------------------------ to JSON

  /** The file's JSON tree, keys in file order. */
  public static Map<String, Object> toJson(PathSpec spec) {
    Map<String, Object> root = new LinkedHashMap<>();
    root.put("schema", SCHEMA);

    List<Object> waypoints = new ArrayList<>();
    for (Waypoint w : spec.waypoints()) {
      Map<String, Object> m = new LinkedHashMap<>();
      m.put("x", w.xMeters());
      m.put("y", w.yMeters());
      if (w.hasTangent()) {
        m.put("tangentDegrees", w.tangentRadians() / DEGREES);
      }
      if (w.tangentScale() != 1.0) {
        m.put("tangentScale", w.tangentScale());
      }
      waypoints.add(m);
    }
    root.put("waypoints", waypoints);

    List<Object> targets = new ArrayList<>();
    for (RotationTarget t : spec.rotationTargets()) {
      Map<String, Object> m = new LinkedHashMap<>();
      if (t.byWaypoint()) {
        m.put("waypoint", t.waypoint());
      } else {
        m.put("meters", t.meters());
      }
      m.put("degrees", t.headingRadians() / DEGREES);
      targets.add(m);
    }
    root.put("rotationTargets", targets);

    List<Object> faces = new ArrayList<>();
    for (FacePointZone z : spec.facePointZones()) {
      Map<String, Object> m = new LinkedHashMap<>();
      m.put("fromWaypoint", z.fromWaypoint());
      m.put("toWaypoint", z.toWaypoint());
      m.put("x", z.point().x());
      m.put("y", z.point().y());
      m.put("blendMeters", z.blendMeters());
      faces.add(m);
    }
    root.put("facePointZones", faces);

    List<Object> zones = new ArrayList<>();
    for (ConstraintZone z : spec.constraintZones()) {
      Map<String, Object> m = new LinkedHashMap<>();
      m.put("fromWaypoint", z.fromWaypoint());
      m.put("toWaypoint", z.toWaypoint());
      putIfFinite(m, "maxMetersPerSecond", z.maxMetersPerSecond());
      putIfFinite(m, "maxMetersPerSecondSquared", z.maxMetersPerSecondSquared());
      zones.add(m);
    }
    root.put("constraintZones", zones);

    putIfFinite(root, "maxMetersPerSecond", spec.maxMetersPerSecond());
    putIfFinite(root, "maxMetersPerSecondSquared", spec.maxMetersPerSecondSquared());
    root.put("startMetersPerSecond", spec.startMetersPerSecond());
    root.put("endMetersPerSecond", spec.endMetersPerSecond());
    return root;
  }

  private static void putIfFinite(Map<String, Object> m, String key, double v) {
    if (Double.isFinite(v)) {
      m.put(key, v);
    }
  }

  // ------------------------------------------------------------------------------ from JSON

  /** Builds a spec from a parsed file. */
  public static PathSpec fromJson(Map<String, Object> root) {
    double schema = number(root, "schema", Double.NaN);
    if (schema != SCHEMA) {
      throw new IllegalArgumentException(
          Double.isNaN(schema)
              ? "not a path file: no schema"
              : "path file schema " + schema + " (this code reads " + SCHEMA + ")");
    }

    List<Waypoint> waypoints = new ArrayList<>();
    for (Object o : list(root, "waypoints")) {
      Map<String, Object> m = asMap(o, "waypoint");
      double tangent = number(m, "tangentDegrees", Double.NaN);
      waypoints.add(
          new Waypoint(
              required(m, "x"),
              required(m, "y"),
              Double.isNaN(tangent) ? Double.NaN : tangent * DEGREES,
              number(m, "tangentScale", 1.0)));
    }

    List<RotationTarget> targets = new ArrayList<>();
    for (Object o : list(root, "rotationTargets")) {
      Map<String, Object> m = asMap(o, "rotation target");
      targets.add(
          new RotationTarget(
              number(m, "waypoint", Double.NaN),
              number(m, "meters", Double.NaN),
              required(m, "degrees") * DEGREES));
    }

    List<FacePointZone> faces = new ArrayList<>();
    for (Object o : list(root, "facePointZones")) {
      Map<String, Object> m = asMap(o, "face-point zone");
      faces.add(
          new FacePointZone(
              required(m, "fromWaypoint"),
              required(m, "toWaypoint"),
              new Vec2(required(m, "x"), required(m, "y")),
              number(m, "blendMeters", FacePointZone.DEFAULT_BLEND_METERS)));
    }

    List<ConstraintZone> zones = new ArrayList<>();
    for (Object o : list(root, "constraintZones")) {
      Map<String, Object> m = asMap(o, "constraint zone");
      zones.add(
          new ConstraintZone(
              required(m, "fromWaypoint"),
              required(m, "toWaypoint"),
              number(m, "maxMetersPerSecond", Double.POSITIVE_INFINITY),
              number(m, "maxMetersPerSecondSquared", Double.POSITIVE_INFINITY)));
    }

    return new PathSpec(
        waypoints,
        targets,
        faces,
        zones,
        number(root, "maxMetersPerSecond", Double.POSITIVE_INFINITY),
        number(root, "maxMetersPerSecondSquared", Double.POSITIVE_INFINITY),
        number(root, "startMetersPerSecond", 0.0),
        number(root, "endMetersPerSecond", 0.0));
  }

  // ------------------------------------------------------------------------------ helpers

  /** Casts a parsed JSON value to an object, or says what was expected. */
  @SuppressWarnings("unchecked")
  public static Map<String, Object> asMap(Object o, String what) {
    if (!(o instanceof Map)) {
      throw new IllegalArgumentException(what + " must be a JSON object");
    }
    return (Map<String, Object>) o;
  }

  static List<?> list(Map<String, Object> m, String key) {
    Object v = m.get(key);
    if (v == null) {
      return List.of();
    }
    if (!(v instanceof List<?> l)) {
      throw new IllegalArgumentException("'" + key + "' must be a list");
    }
    return l;
  }

  static double number(Map<String, Object> m, String key, double fallback) {
    Object v = m.get(key);
    if (v == null) {
      return fallback;
    }
    if (!(v instanceof Number n)) {
      throw new IllegalArgumentException("'" + key + "' must be a number");
    }
    return n.doubleValue();
  }

  static double required(Map<String, Object> m, String key) {
    double v = number(m, key, Double.NaN);
    if (Double.isNaN(v)) {
      throw new IllegalArgumentException("missing '" + key + "'");
    }
    return v;
  }
}
