package frc.team5712.path.e2e;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.e2e.EndToEndRun.EventResult;
import frc.team5712.path.e2e.EndToEndRun.Framework;
import frc.team5712.path.wpilib.PathLibrary;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.DynamicTest;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestFactory;
import org.wpilib.command2.Commands;
import org.wpilib.command3.Command;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;

/**
 * Phase 6: every path authored in the editor, start to finish on the "robot".
 *
 * <ol>
 *   <li>The editor (Playwright, editor/app/e2e-author) authored the paths and saved them to
 *       e2e/out/robot-project, with its preview numbers in e2e/out/preview.json.
 *   <li>Here {@link PathLibrary#loadAll(File)} loads that folder as the robot loads deploy/paths, and every
 *       event name is bound to a command that records the true pose when it runs.
 *   <li>Each path (a rolling chain counts as one) runs through Commands v3 and v2, on blue and on both red
 *       flips, on {@link SimDrive} at 50 Hz: once to the end, and once cancelled halfway.
 * </ol>
 *
 * The table goes to e2e/RESULTS.md. Run everything with {@code node tools/e2e.mjs} from the repo root.
 */
class PathEndToEndTest {
  private static final Path DIR = Path.of(System.getProperty("e2e.dir", "../e2e/out"));
  private static final Path REPORT = Path.of(System.getProperty("e2e.report", "../e2e/RESULTS.md"));
  private static final FieldFrame[] FRAMES = {FieldFrame.BLUE, FieldFrame.RED_HALF_TURN, FieldFrame.RED_MIRROR};

  private static PathLibrary library;
  private static Preview preview;
  /** The run the recording event commands report to. */
  private static EndToEndRun current;
  private static final List<Row> rows = new ArrayList<>();

  record Row(String scenario, Framework framework, FieldFrame frame, EndToEndRun full, EndToEndRun interrupted) {
    boolean pass() {
      return full.failures.isEmpty() && interrupted.failures.isEmpty();
    }
  }

  @BeforeAll
  static void load() throws IOException {
    File paths = DIR.resolve("robot-project/src/main/deploy/paths").toFile();
    assertTrue(paths.isDirectory(), paths + " is missing: author the suite first (node tools/e2e.mjs)");
    library = PathLibrary.loadAll(paths);
    preview = Preview.read(DIR.resolve("preview.json"));
    for (String name : library.eventNames()) {
      library.bindEvent(name, Command.noRequirements(co -> current.record(name)).named("record " + name));
      library.bindEvent(name, Commands.runOnce(() -> current.record(name)));
    }
    // Commands v2 only runs commands while enabled
    HAL.initialize();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @Test
  void everyEventNameIsBoundAndEveryPathHasAPreview() {
    assertEquals(Set.of(), library.unboundEventNames());
    assertEquals(preview.paths().stream().map(Preview.PathPreview::name).collect(java.util.stream.Collectors.toSet()),
        library.all().keySet());
    assertTrue(library.eventNames().size() >= 5, "the suite uses several event names: " + library.eventNames());
  }

  /** Every path on its own, except those inside a rolling chain, which run as the chain. */
  static List<List<String>> scenarios() {
    Set<String> chained = new HashSet<>();
    List<List<String>> out = new ArrayList<>();
    for (List<String> chain : preview.chains()) {
      chained.addAll(chain);
    }
    for (Preview.PathPreview p : preview.paths()) {
      if (!chained.contains(p.name())) {
        out.add(List.of(p.name()));
      }
    }
    out.addAll(preview.chains());
    return out;
  }

  @TestFactory
  Stream<DynamicTest> everyPathStartToFinish() {
    List<DynamicTest> tests = new ArrayList<>();
    for (List<String> chain : scenarios()) {
      for (Framework framework : Framework.values()) {
        for (FieldFrame frame : FRAMES) {
          String name = String.join(" → ", chain) + ", " + framework + ", " + frame;
          tests.add(DynamicTest.dynamicTest(name, () -> {
            long seed = name.hashCode();
            EndToEndRun full = start(new EndToEndRun(library, preview, chain, framework, frame, Double.NaN)).run(seed);
            EndToEndRun cut = start(new EndToEndRun(library, preview, chain, framework, frame, 0.5)).run(seed + 1);
            rows.add(new Row(String.join(" → ", chain), framework, frame, full, cut));
            List<String> failures = new ArrayList<>(full.failures);
            cut.failures.forEach(f -> failures.add("interrupted: " + f));
            assertTrue(failures.isEmpty(), String.join("\n", failures));
          }));
        }
      }
    }
    return tests.stream();
  }

  private static EndToEndRun start(EndToEndRun run) {
    current = run;
    return run;
  }

  // ------------------------------------------------------------------------------ the report

  @AfterAll
  static void report() throws IOException {
    if (rows.isEmpty()) {
      return;
    }
    Files.writeString(REPORT, Report.markdown(rows), StandardCharsets.UTF_8);
    System.out.println("wrote " + REPORT.toAbsolutePath());
  }

  /** Worst absolute value of a field over a run's events. */
  static double worst(EndToEndRun run, java.util.function.ToDoubleFunction<EventResult> f) {
    return run.events.stream().mapToDouble(e -> Math.abs(f.applyAsDouble(e))).max().orElse(0.0);
  }
}
