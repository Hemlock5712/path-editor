package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A single cubic Hermite spline segment.
 *
 * <p>Represents a parametric cubic curve p(t) = a*t^3 + b*t^2 + c*t + d for t in [0,1]. Provides
 * position, derivative, curvature, and arc-length integrand evaluation.
 *
 * <p>Constructed from two endpoints and two tangent vectors using the Hermite basis.
 */
public final class CubicSegment {

  // Coefficients for x(t) = ax*t^3 + bx*t^2 + cx*t + dx
  private final double ax, bx, cx, dx;
  // Coefficients for y(t) = ay*t^3 + by*t^2 + cy*t + dy
  private final double ay, by, cy, dy;

  /**
   * Creates a cubic Hermite segment from endpoints and tangent vectors.
   *
   * <p>The Hermite basis matrix gives:
   *
   * <ul>
   *   <li>a = 2*p0 - 2*p1 + m0 + m1
   *   <li>b = -3*p0 + 3*p1 - 2*m0 - m1
   *   <li>c = m0
   *   <li>d = p0
   * </ul>
   *
   * @param p0 Start point
   * @param p1 End point
   * @param m0 Tangent vector at start
   * @param m1 Tangent vector at end
   */
  public CubicSegment(Translation2d p0, Translation2d p1, Translation2d m0, Translation2d m1) {
    double p0x = p0.getX(), p0y = p0.getY();
    double p1x = p1.getX(), p1y = p1.getY();
    double m0x = m0.getX(), m0y = m0.getY();
    double m1x = m1.getX(), m1y = m1.getY();

    ax = 2 * p0x - 2 * p1x + m0x + m1x;
    bx = -3 * p0x + 3 * p1x - 2 * m0x - m1x;
    cx = m0x;
    dx = p0x;

    ay = 2 * p0y - 2 * p1y + m0y + m1y;
    by = -3 * p0y + 3 * p1y - 2 * m0y - m1y;
    cy = m0y;
    dy = p0y;
  }

  /**
   * Evaluates position at parameter t.
   *
   * @param t Parameter in [0, 1]
   * @return Position on the curve
   */
  public Translation2d getPoint(double t) {
    double t2 = t * t;
    double t3 = t2 * t;
    return new Translation2d(ax * t3 + bx * t2 + cx * t + dx, ay * t3 + by * t2 + cy * t + dy);
  }

  /**
   * Evaluates the first derivative dp/dt at parameter t.
   *
   * @param t Parameter in [0, 1]
   * @return Tangent vector (not normalized)
   */
  public Translation2d getDerivative(double t) {
    double t2 = t * t;
    return new Translation2d(3 * ax * t2 + 2 * bx * t + cx, 3 * ay * t2 + 2 * by * t + cy);
  }

  /**
   * Evaluates the second derivative d2p/dt2 at parameter t.
   *
   * @param t Parameter in [0, 1]
   * @return Second derivative vector
   */
  public Translation2d getSecondDerivative(double t) {
    return new Translation2d(6 * ax * t + 2 * bx, 6 * ay * t + 2 * by);
  }

  /**
   * Computes signed curvature at parameter t.
   *
   * <p>Curvature kappa = (x'*y'' - y'*x'') / (x'^2 + y'^2)^(3/2). Positive curvature means turning
   * left (counterclockwise).
   *
   * @param t Parameter in [0, 1]
   * @return Signed curvature in 1/meters
   */
  public double getCurvature(double t) {
    Translation2d d1 = getDerivative(t);
    Translation2d d2 = getSecondDerivative(t);

    double cross = d1.getX() * d2.getY() - d1.getY() * d2.getX();
    double speedSq = d1.getX() * d1.getX() + d1.getY() * d1.getY();
    double speedCubed = speedSq * Math.sqrt(speedSq);

    if (speedCubed < 1e-12) {
      return 0.0;
    }
    return cross / speedCubed;
  }

  /**
   * Computes the parametric speed |dp/dt| at parameter t.
   *
   * <p>This is the integrand for arc-length computation: L = integral(|dp/dt| dt).
   *
   * @param t Parameter in [0, 1]
   * @return Parametric speed (always non-negative)
   */
  public double getSpeed(double t) {
    Translation2d d = getDerivative(t);
    return Math.hypot(d.getX(), d.getY());
  }
}
