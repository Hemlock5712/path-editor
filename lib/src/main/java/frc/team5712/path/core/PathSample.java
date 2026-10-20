package frc.team5712.path.core;

/**
 * Everything the plan says about one distance along the path.
 *
 * @param s metres along the path
 * @param xMeters field x
 * @param yMeters field y
 * @param tangentX unit direction of travel, x
 * @param tangentY unit direction of travel, y
 * @param curvature signed curvature, 1/m
 * @param dCurvatureDs dκ/ds, 1/m²
 * @param headingRadians planned robot heading, (-π, π]
 * @param headingRadiansPerMeter dθ/ds
 * @param metersPerSecond planned speed along the path
 * @param metersPerSecondSquared planned acceleration along the path
 * @param seconds planned time to get here
 */
public record PathSample(
    double s,
    double xMeters,
    double yMeters,
    double tangentX,
    double tangentY,
    double curvature,
    double dCurvatureDs,
    double headingRadians,
    double headingRadiansPerMeter,
    double metersPerSecond,
    double metersPerSecondSquared,
    double seconds) {

  /** Planned field velocity: speed along the tangent, and spin v·dθ/ds. */
  public FieldSpeeds fieldSpeeds() {
    return new FieldSpeeds(
        metersPerSecond * tangentX, metersPerSecond * tangentY, metersPerSecond * headingRadiansPerMeter);
  }

  /** Planned pose. */
  public Pose pose() {
    return new Pose(xMeters, yMeters, headingRadians);
  }
}
