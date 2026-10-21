package frc.team5712.path.core;

/**
 * What the robot's heading does where no heading is set. A swerve's heading is independent of where it
 * travels, so the default holds: before the first set heading it holds that one, between two it blends,
 * after the last it holds it, and with none at all it holds the direction it starts out in.
 */
public enum HeadingMode {
  /** Hold the set headings and blend between them (the default). */
  HOLD("hold"),
  /** With no set headings, face the direction of travel (the behaviour before schema 3). */
  FACE_TRAVEL("faceTravel");

  private final String fileName;

  HeadingMode(String fileName) {
    this.fileName = fileName;
  }

  /** The name in a path file. */
  public String fileName() {
    return fileName;
  }

  public static HeadingMode fromFileName(String name) {
    for (HeadingMode m : values()) {
      if (m.fileName.equals(name)) {
        return m;
      }
    }
    throw new IllegalArgumentException("unknown headingMode '" + name + "' (hold or faceTravel)");
  }
}
