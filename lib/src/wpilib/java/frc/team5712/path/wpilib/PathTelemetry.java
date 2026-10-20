package frc.team5712.path.wpilib;

import frc.team5712.path.follow.PathFollower;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

/** Logs a follower's state each loop under one telemetry table (watch it in AdvantageScope). */
public final class PathTelemetry {
  private final TelemetryTable table;

  public PathTelemetry(String tableName) {
    this.table = Telemetry.getTable(tableName);
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
