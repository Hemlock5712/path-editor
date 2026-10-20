package frc.team5712.path.tools;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Json;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.plan.FieldMap;
import frc.team5712.path.plan.Pathfinder;
import frc.team5712.path.plan.PlanResult;
import frc.team5712.path.plan.PlannerConfig;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Plans once in a fresh JVM (the cold time), then {@code --repeat} more times (warm), and prints JSON.
 *
 * <pre>
 * java -cp lib/build/libs/pathcore.jar frc.team5712.path.tools.PlanCli field.json \
 *     startX startY startDeg goalX goalY goalDeg [vx vy] [--repeat 2000]
 * </pre>
 */
public final class PlanCli {
  private PlanCli() {}

  public static void main(String[] args) throws Exception {
    List<String> a = new ArrayList<>(List.of(args));
    int repeat = 2000;
    int r = a.indexOf("--repeat");
    if (r >= 0) {
      repeat = Integer.parseInt(a.get(r + 1));
      a.remove(r + 1);
      a.remove(r);
    }
    if (a.size() != 7 && a.size() != 9) {
      System.err.println("usage: PlanCli field.json sx sy sdeg gx gy gdeg [vx vy] [--repeat N]");
      System.exit(2);
    }
    long t0 = System.nanoTime();
    FieldMap field = FieldMap.read(java.nio.file.Path.of(a.get(0)));
    Pathfinder pf = new Pathfinder(field, RobotConfig.DEFAULT, PlannerConfig.DEFAULT);
    double bootMs = (System.nanoTime() - t0) / 1e6;
    Pose start = new Pose(d(a, 1), d(a, 2), Math.toRadians(d(a, 3)));
    Pose goal = new Pose(d(a, 4), d(a, 5), Math.toRadians(d(a, 6)));
    FieldSpeeds v = a.size() == 9 ? new FieldSpeeds(d(a, 7), d(a, 8), 0) : new FieldSpeeds(0, 0, 0);
    long t1 = System.nanoTime();
    PlanResult first = pf.plan(start, v, goal);
    double coldMs = (System.nanoTime() - t1) / 1e6;
    double[] warm = new double[repeat];
    for (int i = 0; i < repeat; i++) {
      long t = System.nanoTime();
      pf.plan(start, v, goal);
      warm[i] = (System.nanoTime() - t) / 1e6;
    }
    java.util.Arrays.sort(warm);
    Map<String, Object> out = new LinkedHashMap<>();
    out.put("ok", first.ok());
    out.put("bootMs", bootMs);
    out.put("graphNodes", (double) pf.nodeCount());
    out.put("coldMs", coldMs);
    if (repeat > 0) {
      out.put("warmMedianMs", warm[repeat / 2]);
      out.put("warmBestMs", warm[0]);
      out.put("warmP95Ms", warm[(int) (repeat * 0.95)]);
      double sum = 0;
      for (int i = 0; i < repeat; i++) {
        sum += warm[i];
      }
      out.put("warmMeanMs", sum / repeat);
    }
    BenchServer.describe(first, out);
    System.out.println(Json.write(out, Json::exact).trim());
  }

  private static double d(List<String> a, int i) {
    return Double.parseDouble(a.get(i).replace(',', '.').toLowerCase(Locale.ROOT));
  }
}
