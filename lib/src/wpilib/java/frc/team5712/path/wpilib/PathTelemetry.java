package frc.team5712.path.wpilib;

import frc.team5712.path.follow.PathFollower;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

/**
 * Logs a follower's state each loop (watch it in AdvantageScope, or live over the path in the editor).
 * WPILib publishes Telemetry under /Telemetry in NetworkTables, so for a path named ToReef:
 *
 * <ul>
 *   <li>{@code /Telemetry/Path/ToReef/}: Phase, Progress (m), Remaining (m), CrossTrack (m), Reference and
 *       Goal (Pose2d structs, field frame), TailRebuilds, EventsFired (this run, in order);
 *   <li>{@code /Telemetry/PathFollower/}: Active (the running path's name, "" when none), LastEvent and
 *       EventsFired, shared by every follow command.
 * </ul>
 */
public final class PathTelemetry {
  private static final TelemetryTable SHARED = Telemetry.getTable("PathFollower");

  private final TelemetryTable table;
  private final String pathName;
  private final List<String> fired = new ArrayList<>();

  /** Telemetry for the path named {@code pathName}, under Path/&lt;pathName&gt;. */
  public PathTelemetry(String pathName) {
    this.pathName = pathName;
    this.table = Telemetry.getTable("Path/" + pathName);
  }

  /** The path started: it becomes the active one and its fired events start over. */
  public void begin() {
    fired.clear();
    String[] none = new String[0];
    table.log("EventsFired", none);
    SHARED.log("Active", pathName);
    SHARED.log("EventsFired", none);
  }

  /** An event fired. */
  public void event(String name) {
    fired.add(name);
    String[] all = fired.toArray(String[]::new);
    table.log("EventsFired", all);
    SHARED.log("EventsFired", all);
    SHARED.log("LastEvent", name);
  }

  /** The path ended (finished or cancelled): no path is active. */
  public void end() {
    SHARED.log("Active", "");
  }

  public void log(PathFollower f) {
    table.log("Phase", f.phase().name());
    table.log("Progress", f.progressMeters());
    table.log("Remaining", f.remainingMeters());
    table.log("CrossTrack", f.crossTrackMeters());
    table.log("Reference", Conversions.toPose2d(f.reference()), Pose2d.struct);
    table.log("Goal", Conversions.toPose2d(f.goal()), Pose2d.struct);
    table.log("TailRebuilds", f.tailRebuildCount());
  }
}
