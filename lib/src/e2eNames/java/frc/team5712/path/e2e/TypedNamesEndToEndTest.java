package frc.team5712.path.e2e;

import static frc.robot.PathEditorNames.Events;
import static frc.robot.PathEditorNames.Paths;
import static frc.robot.PathEditorNames.Points;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.PathEditorNames;
import frc.team5712.path.wpilib.PathLibrary;
import java.io.File;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;

/**
 * The editor's generated names for the authored suite (e2e/out/robot-project/src/main/java/frc/robot/
 * PathEditorNames.java), compiled here as robot code compiles them: every constant names something deploy
 * has, the library takes them everywhere it takes a name, and its boot check passes on them.
 */
class TypedNamesEndToEndTest {
  private static final Path DIR = Path.of(System.getProperty("e2e.dir", "../e2e/out"));

  @Test
  void theGeneratedNamesMatchDeployAndWorkEverywhere() {
    File deploy = DIR.resolve("robot-project/src/main/deploy/paths").toFile();
    // loadAll finds frc.robot.PathEditorNames (the project's package) and checks it: a stale class throws here
    PathLibrary library = PathLibrary.loadAll(deploy);
    assertSame(library, library.requireNames(PathEditorNames.class));

    assertEquals(library.all().keySet(), Arrays.stream(Paths.values()).map(Paths::fileName).collect(Collectors.toSet()));
    for (Paths p : Paths.values()) {
      assertSame(library.get(p.fileName()), library.get(p));
    }
    assertEquals(library.eventNames(), Arrays.stream(Events.values()).map(Events::eventName).collect(Collectors.toSet()));
    for (Events e : Events.values()) {
      library.bindEvent(e, Command.noRequirements(co -> {}).named(e.eventName()));
    }
    assertSame(library, library.requireAllEventsBound());
    assertEquals(Set.of(), library.unboundEventNames());
    assertEquals(library.namedPoints().keySet(), Arrays.stream(Points.values()).map(Points::pointName).collect(Collectors.toSet()));
    for (Points p : Points.values()) {
      assertTrue(Double.isFinite(library.namedPoint(p)[0]), p.name());
    }
  }
}
