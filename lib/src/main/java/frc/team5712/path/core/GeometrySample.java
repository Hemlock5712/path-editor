package frc.team5712.path.core;

/**
 * The path at one distance along it.
 *
 * @param s metres along the path
 * @param xMeters field x
 * @param yMeters field y
 * @param tangentX unit direction of travel, x
 * @param tangentY unit direction of travel, y
 * @param curvature signed curvature in 1/m (positive = turning left)
 * @param dCurvatureDs rate of change of curvature along the path, 1/m²
 */
public record GeometrySample(
    double s,
    double xMeters,
    double yMeters,
    double tangentX,
    double tangentY,
    double curvature,
    double dCurvatureDs) {

  /** Direction of travel in radians. */
  public double tangentRadians() {
    return Angles.atan2(tangentY, tangentX);
  }
}
