package frc.team5712.path.plan;

import frc.team5712.path.core.PathFile;
import java.util.Map;

/**
 * Pathfinder settings.
 *
 * @param bumperLengthMeters robot length with bumpers (along its heading)
 * @param bumperWidthMeters robot width with bumpers
 * @param marginMeters extra clearance on top of the bumpers' half-diagonal when obstacles are grown,
 *     and the clearance the swept-bumper check demands of the smoothed path
 * @param arcStepRadians largest step of the polygonal arcs that round the grown corners
 * @param maxRefits how many times the smoother may add waypoints and refit before giving up
 * @param checkStepMeters spacing of the swept-bumper check along the path
 * @param maxMetersPerSecond path speed cap
 * @param maxMetersPerSecondSquared path acceleration and braking cap
 */
public record PlannerConfig(
    double bumperLengthMeters,
    double bumperWidthMeters,
    double marginMeters,
    double arcStepRadians,
    int maxRefits,
    double checkStepMeters,
    double maxMetersPerSecond,
    double maxMetersPerSecondSquared) {

  /** 0.85 m square bumpers, 5 cm margin, 30° arcs, 8 refits, 2 cm checks, 4 m/s and 6 m/s². */
  public static final PlannerConfig DEFAULT =
      new PlannerConfig(0.85, 0.85, 0.05, Math.toRadians(30), 8, 0.02, 4.0, 6.0);

  public PlannerConfig {
    if (!(bumperLengthMeters > 0) || !(bumperWidthMeters > 0) || !(marginMeters >= 0)
        || !(arcStepRadians > 0) || arcStepRadians > Math.PI / 2 || maxRefits < 0
        || !(checkStepMeters > 0) || !(maxMetersPerSecond > 0) || !(maxMetersPerSecondSquared > 0)) {
      throw new IllegalArgumentException("bad planner settings");
    }
  }

  /**
   * {@link #DEFAULT} with the bumper size from a robot file's tree ({@code deploy/paths/robot.json},
   * keys {@code bumperLengthMeters} and {@code bumperWidthMeters}, both optional). The editor writes
   * them next to the {@code RobotConfig} keys, which {@code RobotFile} reads.
   */
  public static PlannerConfig fromRobotFile(Map<String, Object> robot) {
    return new PlannerConfig(
        PathFile.number(robot, "bumperLengthMeters", DEFAULT.bumperLengthMeters),
        PathFile.number(robot, "bumperWidthMeters", DEFAULT.bumperWidthMeters),
        DEFAULT.marginMeters, DEFAULT.arcStepRadians, DEFAULT.maxRefits, DEFAULT.checkStepMeters,
        DEFAULT.maxMetersPerSecond, DEFAULT.maxMetersPerSecondSquared);
  }

  /** The bumpers' half-diagonal: the robot's reach in any heading. */
  public double halfDiagonalMeters() {
    return Math.hypot(bumperLengthMeters, bumperWidthMeters) / 2;
  }

  /** How far obstacles are grown: half-diagonal plus margin. */
  public double growMeters() {
    return halfDiagonalMeters() + marginMeters;
  }

  public PlannerConfig withMargin(double margin) {
    return new PlannerConfig(bumperLengthMeters, bumperWidthMeters, margin, arcStepRadians, maxRefits,
        checkStepMeters, maxMetersPerSecond, maxMetersPerSecondSquared);
  }

  public PlannerConfig withLimits(double maxSpeed, double maxAccel) {
    return new PlannerConfig(bumperLengthMeters, bumperWidthMeters, marginMeters, arcStepRadians, maxRefits,
        checkStepMeters, maxSpeed, maxAccel);
  }
}
