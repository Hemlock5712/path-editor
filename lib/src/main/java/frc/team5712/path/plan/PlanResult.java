package frc.team5712.path.plan;

import frc.team5712.path.core.Path;
import java.util.List;

/**
 * What {@link Pathfinder#plan} produced.
 *
 * @param path the path to follow, or null when there is no route ({@link #failure} says why)
 * @param waypoints the spline's waypoints: the route's corners plus any the smoother added (and the
 *     robot's and goal's own positions when they were snapped)
 * @param routeLengthMeters length of the corner-to-corner route (before smoothing)
 * @param straightLineMeters start to goal in a straight line
 * @param startSnapMeters how far the start was moved out of a grown obstacle (0 = not moved)
 * @param goalSnapMeters the same for the goal
 * @param refits how many times the smoother added waypoints
 * @param clear true when the swept bumpers keep the margin everywhere (outside snapped stretches)
 * @param minClearanceMeters the smallest bumper-to-obstacle-or-wall distance along the path
 * @param searchMicros snapping, connecting and A*
 * @param smoothMicros spline fits and swept-bumper checks
 * @param profileMicros building the final path (geometry, heading and velocity profile)
 * @param totalMicros the whole query
 */
public record PlanResult(
    Path path,
    List<double[]> waypoints,
    double routeLengthMeters,
    double straightLineMeters,
    double startSnapMeters,
    double goalSnapMeters,
    int refits,
    boolean clear,
    double minClearanceMeters,
    double searchMicros,
    double smoothMicros,
    double profileMicros,
    double totalMicros,
    String failure) {

  static PlanResult failed(String why, double startSnap, double goalSnap, double straight, double micros) {
    return new PlanResult(null, List.of(), Double.NaN, straight, startSnap, goalSnap, 0, false, Double.NaN,
        micros, 0, 0, micros, why);
  }

  public boolean ok() {
    return path != null;
  }

  public boolean startSnapped() {
    return startSnapMeters > 1e-9;
  }

  public boolean goalSnapped() {
    return goalSnapMeters > 1e-9;
  }
}
