package frc.team5712.path.plan;

import frc.team5712.path.core.Json;
import frc.team5712.path.core.PathFile;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * The static field for pathfinding: its size and its obstacles, in metres in the WPILib 2027 field frame
 * (origin at the field centre, +X towards the blue alliance wall). Read from {@code deploy/paths/field.json}:
 *
 * <pre>
 * {"schema": 2,
 *  "field": {"length_m": 16.54, "width_m": 8.02},
 *  "obstacles": [{"name": "pillar", "polygon": [[x, y], ...]}, ...]}
 * </pre>
 *
 * Other keys are ignored. Each polygon is replaced by its convex hull. A file without {@code "schema": 2} was
 * written when +X pointed towards red, so its obstacles are turned half a turn about the centre when read.
 */
public record FieldMap(double lengthMeters, double widthMeters, List<String> names, List<ConvexPolygon> obstacles) {
  public FieldMap {
    if (!(lengthMeters > 0) || !(widthMeters > 0)) {
      throw new IllegalArgumentException("field size must be positive");
    }
    names = List.copyOf(names);
    obstacles = List.copyOf(obstacles);
    if (names.size() != obstacles.size()) {
      throw new IllegalArgumentException("one name per obstacle");
    }
  }

  public static FieldMap read(java.nio.file.Path file) throws IOException {
    return fromJson(PathFile.asMap(Json.parse(Files.readString(file, StandardCharsets.UTF_8)), "field file"));
  }

  /** The field.json schema in the WPILib 2027 frame: centre origin, +X towards the blue alliance wall. */
  public static final int SCHEMA = 2;

  public static FieldMap fromJson(Map<String, Object> root) {
    Map<String, Object> field = PathFile.asMap(root.get("field"), "'field'");
    double length = number(field.get("length_m"), "field length_m");
    double width = number(field.get("width_m"), "field width_m");
    // schema 2 is the WPILib 2027 frame (+X towards blue); a file without it is turned half a turn
    boolean old = PathFile.number(root, "schema", 1.0) < SCHEMA;
    List<String> names = new ArrayList<>();
    List<ConvexPolygon> polys = new ArrayList<>();
    Object obs = root.get("obstacles");
    if (obs != null) {
      if (!(obs instanceof List<?> list)) {
        throw new IllegalArgumentException("'obstacles' must be a list");
      }
      for (Object o : list) {
        Map<String, Object> m = PathFile.asMap(o, "obstacle");
        if (!(m.get("polygon") instanceof List<?> pts)) {
          throw new IllegalArgumentException("an obstacle needs a 'polygon' list");
        }
        double[][] p = new double[pts.size()][];
        for (int i = 0; i < pts.size(); i++) {
          if (!(pts.get(i) instanceof List<?> xy) || xy.size() != 2) {
            throw new IllegalArgumentException("polygon points must be [x, y]");
          }
          double x = number(xy.get(0), "x");
          double y = number(xy.get(1), "y");
          p[i] = old ? new double[] {PathFile.turnedMeters(x), PathFile.turnedMeters(y)} : new double[] {x, y};
        }
        polys.add(ConvexPolygon.of(p));
        names.add(m.get("name") == null ? "obstacle " + polys.size() : m.get("name").toString());
      }
    }
    return new FieldMap(length, width, names, polys);
  }

  private static double number(Object v, String what) {
    if (!(v instanceof Number n) || !Double.isFinite(n.doubleValue())) {
      throw new IllegalArgumentException(what + " must be a number");
    }
    return n.doubleValue();
  }
}
