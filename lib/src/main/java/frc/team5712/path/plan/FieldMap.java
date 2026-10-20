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
 * The static field for pathfinding: its size and its obstacles, in metres with the origin at the field
 * centre (WPILib 2027). Read from {@code deploy/paths/field.json}:
 *
 * <pre>
 * {"field": {"length_m": 16.54, "width_m": 8.02},
 *  "obstacles": [{"name": "pillar", "polygon": [[x, y], ...]}, ...]}
 * </pre>
 *
 * Other keys are ignored. Each polygon is replaced by its convex hull.
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

  public static FieldMap fromJson(Map<String, Object> root) {
    Map<String, Object> field = PathFile.asMap(root.get("field"), "'field'");
    double length = number(field.get("length_m"), "field length_m");
    double width = number(field.get("width_m"), "field width_m");
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
          p[i] = new double[] {number(xy.get(0), "x"), number(xy.get(1), "y")};
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
