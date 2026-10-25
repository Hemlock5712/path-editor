package frc.team5712.path.wpilib;

import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.Pose;
import frc.team5712.path.follow.PathFollower;
import frc.team5712.path.plan.PlanResult;
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
 *   <li>{@code /Telemetry/Path/ToReef/}, each loop: Phase, Progress (m), Remaining (m), CrossTrack (m),
 *       AlongTrack (m, ahead of the plan's schedule), HeadingError (rad), Speed and PlannedSpeed (m/s),
 *       Omega and PlannedOmega (rad/s), Elapsed (s), Reference and Goal (Pose2d structs, field frame);
 *       EventsFired (this run, in order) as they fire;
 *   <li>{@code /Telemetry/Path/ToReef/}, once per start: Plan (the planned path every 5 cm and its end:
 *       s, x, y, heading, t, v per sample, flattened, in the field frame the robot runs it in),
 *       PlanEvents and PlanEventMeters (the path's events and where they are), and PlanSeconds;
 *   <li>{@code /Telemetry/Path/<name>/}, for an on-the-fly path, at each plan: PlanOk, PlanMicros, Replans,
 *       StartSnap, GoalSnap, MinClearance, PlanFailure ({@link #plan}); the plan itself goes out as Plan;
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

  /** Plan samples every this many metres (a 5 m path is about 600 doubles, once per start). */
  private static final double PLAN_STEP_METERS = 0.05;

  /** The path started (call after the follower's start): publishes its plan, then {@link #begin()}. */
  public void begin(PathFollower f) {
    var path = f.path();
    var frame = f.frame();
    int n = Math.max(1, (int) Math.ceil(path.lengthMeters() / PLAN_STEP_METERS));
    double[] plan = new double[(n + 1) * 6];
    for (int i = 0; i <= n; i++) {
      PathSample p = path.sample(Math.min(path.lengthMeters(), i * PLAN_STEP_METERS));
      Pose pose = frame.fromBlue(new Pose(p.xMeters(), p.yMeters(), p.headingRadians()));
      int k = i * 6;
      plan[k] = p.s();
      plan[k + 1] = pose.xMeters();
      plan[k + 2] = pose.yMeters();
      plan[k + 3] = pose.headingRadians();
      plan[k + 4] = p.seconds();
      plan[k + 5] = p.metersPerSecond();
    }
    table.log("Plan", plan);
    table.log("PlanEvents", f.eventNames().toArray(String[]::new));
    table.log("PlanEventMeters", f.eventMeters());
    table.log("PlanSeconds", path.totalTimeSeconds());
    begin();
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

  /**
   * An on-the-fly path was planned (or re-planned): PlanOk, PlanMicros, Replans, StartSnap and GoalSnap (m,
   * how far a start or goal inside an obstacle was moved out), MinClearance (m, bumpers to the nearest
   * obstacle or wall along the plan) and PlanFailure ("" when it planned).
   */
  public void plan(PlanResult r, int replans) {
    table.log("PlanOk", r.ok());
    table.log("PlanMicros", r.totalMicros());
    table.log("Replans", replans);
    table.log("StartSnap", r.startSnapMeters());
    table.log("GoalSnap", r.goalSnapMeters());
    table.log("MinClearance", r.ok() ? r.minClearanceMeters() : Double.NaN);
    table.log("PlanClear", r.ok() && r.clear());
    table.log("PlanFailure", r.failure() == null ? "" : r.failure());
  }

  public void log(PathFollower f) {
    table.log("Phase", f.phase().name());
    table.log("Progress", f.progressMeters());
    table.log("Remaining", f.remainingMeters());
    table.log("CrossTrack", f.crossTrackMeters());
    table.log("AlongTrack", f.alongTrackMeters());
    table.log("HeadingError", f.headingErrorRadians());
    table.log("Speed", f.measuredMetersPerSecond());
    table.log("PlannedSpeed", f.plannedMetersPerSecond());
    table.log("Omega", f.measuredOmegaRadPerSec());
    table.log("PlannedOmega", f.plannedOmegaRadPerSec());
    table.log("Elapsed", f.elapsedSeconds());
    table.log("Reference", Conversions.toPose2d(f.reference()), Pose2d.struct);
    table.log("Goal", Conversions.toPose2d(f.goal()), Pose2d.struct);
  }
}
