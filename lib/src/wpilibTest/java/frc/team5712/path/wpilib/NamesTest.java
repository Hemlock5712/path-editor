package frc.team5712.path.wpilib;

import static frc.team5712.path.wpilib.names.PathEditorNames.Events;
import static frc.team5712.path.wpilib.names.PathEditorNames.Paths;
import static frc.team5712.path.wpilib.names.PathEditorNames.Points;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.PathEvent;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.tools.FixturePaths;
import frc.team5712.path.wpilib.names.PathEditorNames;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** The editor's generated names: overloads that take them, and the boot check against deploy. */
class NamesTest {
  private static final String PROJECT = "{ \"schema\": 1, \"paths\": [], \"namedPoints\": [{ \"id\": \"pt-1\", \"name\": \"Score L\", \"x\": 1.5, \"y\": -2 }], "
      + "\"settings\": { \"javaPackage\": \"%s\" }, \"view\": {} }";

  /** A deploy folder matching names.PathEditorNames: SCurve with intakeDown and shoot, and the point Score L. */
  private static void deploy(File dir, String javaPackage, String... events) throws IOException {
    List<PathEvent> es = java.util.Arrays.stream(events).map(e -> PathEvent.atMeters(e, 1.0)).toList();
    PathFile.write(FixturePaths.sCurve().withEvents(es), dir.toPath().resolve("SCurve.path.json"));
    Files.writeString(dir.toPath().resolve(PathLibrary.PROJECT_FILE), String.format(PROJECT, javaPackage));
  }

  @Test
  void everyNameTakingMethodTakesTheGeneratedNames(@TempDir File dir) throws IOException {
    deploy(dir, "frc.team5712.path.wpilib.names", "intakeDown", "shoot");
    PathLibrary lib = PathLibrary.loadAll(dir);
    assertSame(lib.get("SCurve"), lib.get(Paths.S_CURVE));
    assertArrayEquals(new double[] {1.5, -2}, lib.namedPoint(Points.SCORE_L), 0.0);
    assertThrows(IllegalStateException.class, lib::requireAllEventsBound);
    org.wpilib.command3.Command noop = org.wpilib.command3.Command.noRequirements(co -> {}).named("noop");
    lib.bindEvent(Events.INTAKE_DOWN, noop).bindEvent(Events.SHOOT, org.wpilib.command2.Commands.none());
    assertEquals(Set.of(), lib.unboundEventNames());
    assertSame(lib, lib.requireAllEventsBound());
    assertTrue(lib.eventCommand("intakeDown").isPresent());
  }

  @Test
  void aStaleGeneratedClassFailsAtLoadNamingWhatsMissing(@TempDir File dir) throws IOException {
    // the path no longer uses "shoot": names.PathEditorNames.Events.SHOOT is stale
    deploy(dir, "frc.team5712.path.wpilib.names", "intakeDown");
    IllegalStateException e = assertThrows(IllegalStateException.class, () -> PathLibrary.loadAll(dir));
    assertTrue(e.getMessage().contains("Events.SHOOT (no path uses the event \"shoot\")"), e.getMessage());
    assertTrue(e.getMessage().contains("Save the project in Myelin"), e.getMessage());
    // the same check by hand, and no check when the project names a package with no generated class
    deploy(dir, "frc.robot.nothing.here", "intakeDown");
    PathLibrary lib = PathLibrary.loadAll(dir);
    assertThrows(IllegalStateException.class, () -> lib.requireNames(PathEditorNames.class));
  }
}
