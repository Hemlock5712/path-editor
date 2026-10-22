package frc.team5712.path.wpilib;

/**
 * An event name, checked by the compiler: the editor generates {@code PathEditorNames.Events}, every event
 * name its paths use, implementing this. Bind them with {@link PathLibrary#bindEvent(EventId,
 * org.wpilib.command3.Command)} and check with {@link PathLibrary#requireAllEventsBound()}.
 */
public interface EventId {
  /** The event's name as the path files spell it. */
  String eventName();
}
