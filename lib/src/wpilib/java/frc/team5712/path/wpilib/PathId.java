package frc.team5712.path.wpilib;

/**
 * A path by name, checked by the compiler: the editor generates an enum of every path file
 * ({@code PathEditorNames.Paths}) that implements this, so a typo or a renamed path is a compile error,
 * not a boot error. Anything that takes a path name takes one of these too.
 */
public interface PathId {
  /** The path file's name without {@code .path.json}. */
  String fileName();
}
