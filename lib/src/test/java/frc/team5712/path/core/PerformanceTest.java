package frc.team5712.path.core;

import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.tools.FixturePaths;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import org.junit.jupiter.api.Test;

/**
 * Timings on the 16 m, 10-waypoint auto route. Prints the numbers; the assertions are loose so a
 * busy machine doesn't fail the build. The first build in a fresh JVM pays for class loading and
 * the interpreter, which is what the robot sees at boot, so that one runs in a child JVM.
 */
class PerformanceTest {
  /** Entry point for the child JVM: one cold build, then a second, printed in milliseconds. */
  public static void main(String[] args) {
    long t0 = System.nanoTime();
    Path.build(FixturePaths.autoRoute(), RobotConfig.DEFAULT);
    long t1 = System.nanoTime();
    Path.build(FixturePaths.autoRoute(), RobotConfig.DEFAULT);
    long t2 = System.nanoTime();
    System.out.printf(java.util.Locale.ROOT, "%.3f %.3f%n", (t1 - t0) / 1e6, (t2 - t1) / 1e6);
  }

  @Test
  void coldStartInAFreshJvm() throws IOException, InterruptedException {
    String javaExe = java.nio.file.Path.of(System.getProperty("java.home"), "bin", "java").toString();
    Process p =
        new ProcessBuilder(javaExe, "-cp", System.getProperty("java.class.path"), PerformanceTest.class.getName())
            .redirectErrorStream(true)
            .start();
    String out = new String(p.getInputStream().readAllBytes(), StandardCharsets.UTF_8).trim();
    assertTrue(p.waitFor() == 0, out);
    String[] ms = out.split(" ");
    System.out.printf("fresh JVM: first build %s ms (class loading + interpreter), second %s ms%n", ms[0], ms[1]);
    assertTrue(Double.parseDouble(ms[0]) < 2000.0);
  }

  @Test
  void buildAndLookupTimes() {
    PathSpec spec = FixturePaths.autoRoute();

    long t0 = System.nanoTime();
    Path cold = Path.build(spec, RobotConfig.DEFAULT);
    double coldMs = (System.nanoTime() - t0) / 1e6;

    long t1 = System.nanoTime();
    Path.build(spec, RobotConfig.DEFAULT);
    double secondMs = (System.nanoTime() - t1) / 1e6;

    for (int i = 0; i < 3000; i++) {
      Path.build(spec, RobotConfig.DEFAULT);
    }
    int reps = 500;
    double best = Double.POSITIVE_INFINITY;
    double total = 0;
    for (int i = 0; i < reps; i++) {
      long t = System.nanoTime();
      Path.build(spec, RobotConfig.DEFAULT);
      double ms = (System.nanoTime() - t) / 1e6;
      best = Math.min(best, ms);
      total += ms;
    }
    double warmMs = total / reps;

    // Per-loop lookups: a full sample, and a windowed projection
    double sink = 0;
    for (int i = 0; i < 200_000; i++) {
      sink += cold.sample((i % 1600) * 0.01).metersPerSecond();
      sink += cold.geometry().project(0.3, 0.1 + (i % 10) * 0.01, 9.0, 0.5).s();
    }
    int lookups = 200_000;
    long t2 = System.nanoTime();
    for (int i = 0; i < lookups; i++) {
      sink += cold.sample((i % 1600) * 0.01).metersPerSecond();
    }
    double sampleUs = (System.nanoTime() - t2) / 1e3 / lookups;
    long t3 = System.nanoTime();
    for (int i = 0; i < lookups; i++) {
      sink += cold.geometry().project(0.3 + (i % 10) * 0.01, 0.1, 9.0, 0.5).s();
    }
    double projectUs = (System.nanoTime() - t3) / 1e3 / lookups;

    System.out.printf(
        "auto route %.1f m, %d table pieces, %d profile points:%n"
            + "  build cold %.2f ms, second %.2f ms, warm mean %.3f ms (best %.3f ms)%n"
            + "  sample(s) %.3f us, project (±0.5 m window) %.3f us   [%s]%n",
        cold.lengthMeters(),
        cold.geometry().tablePieceCount(),
        cold.profile().gridPointCount(),
        coldMs,
        secondMs,
        warmMs,
        best,
        sampleUs,
        projectUs,
        sink > 0 ? "ok" : "?");

    assertTrue(warmMs < 5.0, "warm build " + warmMs + " ms");
    assertTrue(sampleUs < 20.0, "sample " + sampleUs + " us");
    assertTrue(projectUs < 50.0, "project " + projectUs + " us");
  }
}
