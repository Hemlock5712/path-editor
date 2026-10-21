package frc.team5712.path.tools;

import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.RobotFile;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A profiled path sampled every {@code stepMeters} along it, as a JSON tree: the inputs, a summary
 * and one column per quantity. Shared by {@link FixtureGen} (golden vectors) and {@link ProfileCli}
 * (the bench).
 *
 * <pre>
 * { "inputs": { "path": {...}, "robot": {...}, "gridMeters": 0.03, "stepMeters": 0.05 },
 *   "lengthMeters": 16.4, "totalTimeSeconds": 8.0,
 *   "samples": { "s": [...], "x": [...], "y": [...], "tangentX": [...], "tangentY": [...],
 *                "curvature": [...], "dCurvatureDs": [...], "heading": [...],
 *                "headingPerMeter": [...], "velocity": [...], "acceleration": [...], "time": [...] } }
 * </pre>
 *
 * Units: metres, radians, seconds. {@code heading} is the planned robot heading; {@code
 * headingPerMeter} is dθ/ds, so the planned spin is {@code velocity × headingPerMeter}.
 */
public final class ProfileSamples {
  /** Column names, in output order. */
  public static final List<String> COLUMNS =
      List.of(
          "s", "x", "y", "tangentX", "tangentY", "curvature", "dCurvatureDs",
          "heading", "headingPerMeter", "velocity", "acceleration", "time");

  private ProfileSamples() {}

  /** Distances to sample at: 0, step, 2·step, ... and the end. */
  public static double[] stations(double lengthMeters, double stepMeters) {
    int n = (int) Math.ceil(lengthMeters / stepMeters - 1e-9);
    double[] s = new double[n + 1];
    for (int i = 0; i < n; i++) {
      s[i] = i * stepMeters;
    }
    s[n] = lengthMeters;
    return s;
  }

  /**
   * Builds the path described by a parsed path file and samples it. The tree is copied into
   * {@code inputs.path} as given, so a reader rebuilds from exactly the same numbers.
   */
  public static Map<String, Object> toJson(
      Map<String, Object> pathTree, RobotConfig robot, double gridMeters, double stepMeters) {
    Path path = Path.build(PathFile.fromJson(pathTree), robot, gridMeters);

    Map<String, Object> inputs = new LinkedHashMap<>();
    inputs.put("path", pathTree);
    inputs.put("robot", RobotFile.toJson(robot));
    inputs.put("gridMeters", gridMeters);
    inputs.put("stepMeters", stepMeters);

    List<List<Object>> columns = new ArrayList<>();
    for (int j = 0; j < COLUMNS.size(); j++) {
      columns.add(new ArrayList<>());
    }
    for (double s : stations(path.lengthMeters(), stepMeters)) {
      PathSample p = path.sample(s);
      double[] row = {
        p.s(), p.xMeters(), p.yMeters(), p.tangentX(), p.tangentY(), p.curvature(),
        p.dCurvatureDs(), p.headingRadians(), p.headingRadiansPerMeter(), p.metersPerSecond(),
        p.metersPerSecondSquared(), p.seconds()
      };
      for (int j = 0; j < row.length; j++) {
        columns.get(j).add(row[j]);
      }
    }
    Map<String, Object> samples = new LinkedHashMap<>();
    for (int j = 0; j < COLUMNS.size(); j++) {
      samples.put(COLUMNS.get(j), columns.get(j));
    }

    Map<String, Object> root = new LinkedHashMap<>();
    root.put("inputs", inputs);
    root.put("lengthMeters", path.lengthMeters());
    root.put("totalTimeSeconds", path.totalTimeSeconds());
    double[] cusps = path.cusps();
    if (cusps.length > 0) { // only paths that double back carry the key, so the others' fixtures stay as they were
      List<Object> c = new ArrayList<>();
      for (double s : cusps) {
        c.add(s);
      }
      root.put("cusps", c);
    }
    root.put("samples", samples);
    return root;
  }
}
