package frc.team5712.path.e2e;

import frc.team5712.path.core.Json;
import frc.team5712.path.core.PathFile;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;

/**
 * e2e/out/preview.json: what the editor's preview said about each path (written by the authoring
 * step, editor/app/e2e-author), and which paths are chained by a rolling end.
 */
record Preview(List<List<String>> chains, List<PathPreview> paths) {
  record EventPreview(String name, double s, double t) {}

  record PathPreview(String name, double lengthMeters, double totalTimeSeconds, double peakMetersPerSecond,
      List<EventPreview> events) {}

  PathPreview path(String name) {
    return paths.stream().filter(p -> p.name().equals(name)).findFirst()
        .orElseThrow(() -> new NoSuchElementException("no preview for " + name));
  }

  static Preview read(java.nio.file.Path file) throws IOException {
    Map<String, Object> root = PathFile.asMap(Json.parse(Files.readString(file, StandardCharsets.UTF_8)), "preview");
    List<List<String>> chains = new ArrayList<>();
    for (Object c : (List<?>) root.get("chains")) {
      chains.add(((List<?>) c).stream().map(Object::toString).toList());
    }
    List<PathPreview> paths = new ArrayList<>();
    for (Object o : (List<?>) root.get("paths")) {
      Map<String, Object> m = PathFile.asMap(o, "path preview");
      List<EventPreview> events = new ArrayList<>();
      for (Object e : (List<?>) m.get("events")) {
        Map<String, Object> em = PathFile.asMap(e, "event preview");
        events.add(new EventPreview((String) em.get("name"), num(em, "s"), num(em, "t")));
      }
      paths.add(new PathPreview((String) m.get("name"), num(m, "lengthMeters"), num(m, "totalTimeSeconds"),
          num(m, "peakMetersPerSecond"), events));
    }
    return new Preview(chains, paths);
  }

  private static double num(Map<String, Object> m, String key) {
    return ((Number) m.get(key)).doubleValue();
  }
}
