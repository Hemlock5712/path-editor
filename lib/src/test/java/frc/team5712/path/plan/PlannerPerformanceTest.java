package frc.team5712.path.plan;

import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.Locale;
import org.junit.jupiter.api.Test;

/**
 * Pathfinding timings on the bench field (six obstacles), from (-4, -1.5) to (3.8, 1.4): about 9.3 m of
 * route. The PLAN.md target is under 5 ms for search + smoothing + profile. Prints the numbers; the
 * assertions are loose so a busy machine doesn't fail the build. The cold query runs in a child JVM.
 */
class PlannerPerformanceTest {
  static final Pose START = new Pose(-4.0, -1.5, 0);
  static final Pose GOAL = new Pose(3.8, 1.4, 0);
  static final FieldSpeeds STOPPED = new FieldSpeeds(0, 0, 0);

  /** Child JVM: boot (grow + graph), first query, second query, in milliseconds. */
  public static void main(String[] args) {
    long t0 = System.nanoTime();
    Pathfinder pf = new Pathfinder(PathfinderTest.fieldV1(), RobotConfig.DEFAULT, PlannerConfig.DEFAULT);
    long t1 = System.nanoTime();
    pf.plan(START, STOPPED, GOAL);
    long t2 = System.nanoTime();
    pf.plan(START, STOPPED, GOAL);
    long t3 = System.nanoTime();
    System.out.printf(Locale.ROOT, "%.3f %.3f %.3f%n", (t1 - t0) / 1e6, (t2 - t1) / 1e6, (t3 - t2) / 1e6);
  }

  @Test
  void coldInAFreshJvm() throws IOException, InterruptedException {
    String javaExe = java.nio.file.Path.of(System.getProperty("java.home"), "bin", "java").toString();
    Process p = new ProcessBuilder(javaExe, "-cp", System.getProperty("java.class.path"),
        PlannerPerformanceTest.class.getName()).redirectErrorStream(true).start();
    String out = new String(p.getInputStream().readAllBytes(), StandardCharsets.UTF_8).trim();
    assertTrue(p.waitFor() == 0, out);
    String[] ms = out.split(" ");
    System.out.printf("pathfinder, fresh JVM: boot (grow + graph) %s ms, first query %s ms, second %s ms%n",
        ms[0], ms[1], ms[2]);
    assertTrue(Double.parseDouble(ms[1]) < 2000.0);
  }

  @Test
  void warmQuery() {
    Pathfinder pf = new Pathfinder(PathfinderTest.fieldV1(), RobotConfig.DEFAULT, PlannerConfig.DEFAULT);
    for (int i = 0; i < 3000; i++) {
      pf.plan(START, STOPPED, GOAL);
    }
    int reps = 1000;
    double[] total = new double[reps];
    double[] search = new double[reps];
    double[] smooth = new double[reps];
    double[] profile = new double[reps];
    PlanResult r = null;
    for (int i = 0; i < reps; i++) {
      r = pf.plan(START, STOPPED, GOAL);
      total[i] = r.totalMicros() / 1e3;
      search[i] = r.searchMicros() / 1e3;
      smooth[i] = r.smoothMicros() / 1e3;
      profile[i] = r.profileMicros() / 1e3;
    }
    System.out.printf(Locale.ROOT,
        "pathfinder, warm (%d nodes, %.2f m route, %.2f m path): total median %.3f ms (best %.3f, p95 %.3f); "
            + "search %.3f, smoothing %.3f, profile %.3f ms median%n",
        pf.nodeCount(), r.routeLengthMeters(), r.path().lengthMeters(), median(total), min(total), p95(total),
        median(search), median(smooth), median(profile));
    assertTrue(median(total) < 25.0, "warm query " + median(total) + " ms");
  }

  static double median(double[] a) {
    double[] b = a.clone();
    Arrays.sort(b);
    return b[b.length / 2];
  }

  static double min(double[] a) {
    return Arrays.stream(a).min().orElse(Double.NaN);
  }

  static double p95(double[] a) {
    double[] b = a.clone();
    Arrays.sort(b);
    return b[(int) (b.length * 0.95)];
  }
}
