// Shaped like the editor's generated file (editor/app/src/io/javaEnum.ts), for the library's own tests.
package frc.team5712.path.wpilib.names;

import frc.team5712.path.wpilib.EventId;
import frc.team5712.path.wpilib.PathId;
import frc.team5712.path.wpilib.PointId;

public final class PathEditorNames {
  private PathEditorNames() {}

  public enum Paths implements PathId {
    S_CURVE("SCurve");

    private final String name;

    Paths(String name) {
      this.name = name;
    }

    @Override
    public String fileName() {
      return name;
    }
  }

  public enum Events implements EventId {
    INTAKE_DOWN("intakeDown"),
    SHOOT("shoot");

    private final String name;

    Events(String name) {
      this.name = name;
    }

    @Override
    public String eventName() {
      return name;
    }
  }

  public enum Points implements PointId {
    SCORE_L("Score L");

    private final String name;

    Points(String name) {
      this.name = name;
    }

    @Override
    public String pointName() {
      return name;
    }
  }
}
