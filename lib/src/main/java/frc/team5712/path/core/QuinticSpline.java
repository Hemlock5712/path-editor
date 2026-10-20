package frc.team5712.path.core;

import java.util.List;

/**
 * A chain of quintic Hermite pieces through the waypoints, with curvature continuous at every knot.
 *
 * <p>Piece {@code k} runs from waypoint {@code k} to {@code k + 1} with a local parameter
 * {@code u ∈ [0, 1]}. Each end of a piece is pinned by a position, a first derivative {@code V} and a
 * second derivative {@code A}:
 *
 * <ul>
 *   <li>{@code V = a·d}: {@code d} is the waypoint's unit tangent and {@code a} the piece's estimated
 *       arc length, {@code tangentScale × chord × h / sin h}, where {@code h} is half the turn the two
 *       end tangents make against the chord (capped at π/2). That is the exact arc length when the
 *       piece is a circular arc, which keeps the speed |p'| nearly constant along the piece. Each
 *       piece scales by its own length, so short and long neighbours both stay loop-free.
 *   <li>{@code A = a²·κ·n}: {@code n} is the unit normal and {@code κ} the curvature chosen for that
 *       knot. With no along-tangent part, the curvature of either piece at the knot is exactly
 *       {@code (V × A) / a³ = κ}, so the two pieces agree on κ whatever κ is: G2 by construction.
 * </ul>
 *
 * <p>The knot curvatures are chosen to make the curve as smooth as possible: they minimise
 * {@code Σ ∫|p'''(u)|² du / L⁵} (roughly the integral of the squared third derivative along the arc,
 * so long and short pieces count fairly). Each piece couples only its two knots, so this is one
 * tridiagonal solve, O(N). With the arc-length magnitudes above, points on a circle give that
 * circle's constant curvature.
 */
public final class QuinticSpline {
  /** Shortest allowed distance between consecutive waypoints. */
  public static final double MIN_CHORD_METERS = 1e-6;

  private final int segmentCount;
  private final double[] originX;
  private final double[] originY;
  // Power-basis coefficients c1..c5 per piece, relative to the piece's start point:
  // p(u) = origin + c1 u + c2 u² + c3 u³ + c4 u⁴ + c5 u⁵.
  private final double[] cx;
  private final double[] cy;
  private final double[] chordMeters;
  private final double[] knotTangentRadians;
  private final double[] knotCurvature;

  private QuinticSpline(int segmentCount) {
    this.segmentCount = segmentCount;
    originX = new double[segmentCount];
    originY = new double[segmentCount];
    cx = new double[5 * segmentCount];
    cy = new double[5 * segmentCount];
    chordMeters = new double[segmentCount];
    knotTangentRadians = new double[segmentCount + 1];
    knotCurvature = new double[segmentCount + 1];
  }

  /** Builds the spline through these waypoints. */
  public static QuinticSpline through(List<Waypoint> waypoints) {
    int n = waypoints.size() - 1;
    if (n < 1) {
      throw new IllegalArgumentException("a spline needs at least two waypoints");
    }
    QuinticSpline spline = new QuinticSpline(n);

    double[] px = new double[n + 1];
    double[] py = new double[n + 1];
    for (int i = 0; i <= n; i++) {
      px[i] = waypoints.get(i).xMeters();
      py[i] = waypoints.get(i).yMeters();
    }

    // Unit chords
    double[] ex = new double[n];
    double[] ey = new double[n];
    double[] chord = spline.chordMeters;
    for (int k = 0; k < n; k++) {
      double dx = px[k + 1] - px[k];
      double dy = py[k + 1] - py[k];
      chord[k] = Math.sqrt(dx * dx + dy * dy);
      if (!(chord[k] >= MIN_CHORD_METERS)) {
        throw new IllegalArgumentException("waypoints " + k + " and " + (k + 1) + " are at the same place");
      }
      ex[k] = dx / chord[k];
      ey[k] = dy / chord[k];
    }

    // Unit tangents: given ones first, then automatic interior ones, then automatic ends
    double[] dx = new double[n + 1];
    double[] dy = new double[n + 1];
    for (int i = 0; i <= n; i++) {
      Waypoint w = waypoints.get(i);
      if (w.hasTangent()) {
        dx[i] = Angles.cos(w.tangentRadians());
        dy[i] = Angles.sin(w.tangentRadians());
      } else if (i > 0 && i < n) {
        // Bessel tangent: the slope of the parabola through the three points, by chord length
        double l0 = chord[i - 1];
        double l1 = chord[i];
        double tx = (l1 * ex[i - 1] + l0 * ex[i]) / (l0 + l1);
        double ty = (l1 * ey[i - 1] + l0 * ey[i]) / (l0 + l1);
        double len = Math.sqrt(tx * tx + ty * ty);
        if (len < 1e-9) { // the path doubles straight back: no parabola, leave along the next chord
          dx[i] = ex[i];
          dy[i] = ey[i];
        } else {
          dx[i] = tx / len;
          dy[i] = ty / len;
        }
      }
    }
    boolean autoStart = !waypoints.get(0).hasTangent();
    boolean autoEnd = !waypoints.get(n).hasTangent();
    if (n == 1 && autoStart && autoEnd) {
      dx[0] = ex[0];
      dy[0] = ey[0];
      dx[1] = ex[0];
      dy[1] = ey[0];
    } else {
      // An automatic end mirrors its neighbour's tangent across the chord between them, which makes
      // the end piece symmetric (the parabolic end condition).
      if (autoStart) {
        double k = 2.0 * (dx[1] * ex[0] + dy[1] * ey[0]);
        dx[0] = k * ex[0] - dx[1];
        dy[0] = k * ey[0] - dy[1];
      }
      if (autoEnd) {
        double k = 2.0 * (dx[n - 1] * ex[n - 1] + dy[n - 1] * ey[n - 1]);
        dx[n] = k * ex[n - 1] - dx[n - 1];
        dy[n] = k * ey[n - 1] - dy[n - 1];
      }
    }
    for (int i = 0; i <= n; i++) {
      spline.knotTangentRadians[i] = Angles.atan2(dy[i], dx[i]);
    }

    // End speeds |V| per piece: the arc length of a circular arc with the same end turns, h / sin h
    // times the chord, then each waypoint's tangentScale
    double[] mag0 = new double[n];
    double[] mag1 = new double[n];
    for (int k = 0; k < n; k++) {
      double turn0 = Angles.atan2(ex[k] * dy[k] - ey[k] * dx[k], ex[k] * dx[k] + ey[k] * dy[k]);
      double turn1 =
          Angles.atan2(ex[k] * dy[k + 1] - ey[k] * dx[k + 1], ex[k] * dx[k + 1] + ey[k] * dy[k + 1]);
      double h = 0.5 * (Math.abs(turn0) + Math.abs(turn1));
      if (h > 0.5 * Math.PI) {
        h = 0.5 * Math.PI;
      }
      double arc = h > 1e-6 ? chord[k] * h / Angles.sin(h) : chord[k];
      mag0[k] = waypoints.get(k).tangentScale() * arc;
      mag1[k] = waypoints.get(k + 1).tangentScale() * arc;
    }

    // Knot curvatures: minimise Σ w ∫|p'''|² with w = 1/L⁵. With the quintic Hermite basis H0..H5
    // (for P0, V0, A0, A1, V1, P1), the exact integrals of products of third derivatives are
    // ∫H2'''² = ∫H3'''² = 9, ∫H2'''H3''' = -3, ∫H2'''H5''' = -60, ∫H2'''H4''' = 24,
    // ∫H3'''H5''' = 60, ∫H3'''H1''' = -24 (and H0''' = -H5''', so P0 and P1 enter as the chord).
    // V0 and V1 are perpendicular to their own normals, which drops the remaining terms.
    double[] diag = new double[n + 1];
    double[] upper = new double[n + 1];
    double[] rhs = new double[n + 1];
    for (int k = 0; k < n; k++) {
      double l = chord[k];
      double weight = 1.0 / (l * l * l * l * l);
      double a0 = mag0[k];
      double a1 = mag1[k];
      double a0sq = a0 * a0;
      double a1sq = a1 * a1;
      double chordX = px[k + 1] - px[k];
      double chordY = py[k + 1] - py[k];
      // normals n = d turned left
      double n0x = -dy[k];
      double n0y = dx[k];
      double n1x = -dy[k + 1];
      double n1y = dx[k + 1];
      double chordDotN0 = chordX * n0x + chordY * n0y;
      double chordDotN1 = chordX * n1x + chordY * n1y;
      double v1DotN0 = a1 * (dx[k + 1] * n0x + dy[k + 1] * n0y);
      double v0DotN1 = a0 * (dx[k] * n1x + dy[k] * n1y);

      diag[k] += weight * 9.0 * a0sq * a0sq;
      diag[k + 1] += weight * 9.0 * a1sq * a1sq;
      upper[k] += weight * -3.0 * a0sq * a1sq * (n0x * n1x + n0y * n1y);
      rhs[k] -= weight * a0sq * (-60.0 * chordDotN0 + 24.0 * v1DotN0);
      rhs[k + 1] -= weight * a1sq * (60.0 * chordDotN1 - 24.0 * v0DotN1);
    }
    // Thomas algorithm (the matrix is a sum of positive-definite 2×2 blocks, so no pivoting needed)
    for (int i = 1; i <= n; i++) {
      double m = upper[i - 1] / diag[i - 1];
      diag[i] -= m * upper[i - 1];
      rhs[i] -= m * rhs[i - 1];
    }
    double[] kappa = spline.knotCurvature;
    kappa[n] = rhs[n] / diag[n];
    for (int i = n - 1; i >= 0; i--) {
      kappa[i] = (rhs[i] - upper[i] * kappa[i + 1]) / diag[i];
    }

    // Power-basis coefficients per piece
    for (int k = 0; k < n; k++) {
      double a0 = mag0[k];
      double a1 = mag1[k];
      double v0x = a0 * dx[k];
      double v0y = a0 * dy[k];
      double v1x = a1 * dx[k + 1];
      double v1y = a1 * dy[k + 1];
      double acc0 = a0 * a0 * kappa[k];
      double acc1 = a1 * a1 * kappa[k + 1];
      double acc0x = -dy[k] * acc0;
      double acc0y = dx[k] * acc0;
      double acc1x = -dy[k + 1] * acc1;
      double acc1y = dx[k + 1] * acc1;
      double chordX = px[k + 1] - px[k];
      double chordY = py[k + 1] - py[k];
      spline.originX[k] = px[k];
      spline.originY[k] = py[k];
      setCoefficients(spline.cx, k, chordX, v0x, v1x, acc0x, acc1x);
      setCoefficients(spline.cy, k, chordY, v0y, v1y, acc0y, acc1y);
    }
    return spline;
  }

  private static void setCoefficients(
      double[] c, int k, double chord, double v0, double v1, double a0, double a1) {
    int o = 5 * k;
    c[o] = v0;
    c[o + 1] = 0.5 * a0;
    c[o + 2] = 10.0 * chord - 6.0 * v0 - 4.0 * v1 - 1.5 * a0 + 0.5 * a1;
    c[o + 3] = -15.0 * chord + 8.0 * v0 + 7.0 * v1 + 1.5 * a0 - a1;
    c[o + 4] = 6.0 * chord - 3.0 * v0 - 3.0 * v1 - 0.5 * a0 + 0.5 * a1;
  }

  /** Number of pieces (waypoints − 1). */
  public int segmentCount() {
    return segmentCount;
  }

  /** Straight-line distance between waypoints {@code k} and {@code k + 1}. */
  public double chordMeters(int k) {
    return chordMeters[k];
  }

  /** Direction of travel at waypoint {@code i}, given or chosen. */
  public double knotTangentRadians(int i) {
    return knotTangentRadians[i];
  }

  /** Curvature at waypoint {@code i}, in 1/m (positive = turning left). */
  public double knotCurvature(int i) {
    return knotCurvature[i];
  }

  /**
   * Evaluates piece {@code k} at {@code u}. Writes x, y, x', y', x'', y'', x''', y''' (derivatives
   * with respect to u) into {@code out[0..7]}.
   */
  public void evaluate(int k, double u, double[] out) {
    int o = 5 * k;
    double x1 = cx[o];
    double x2 = cx[o + 1];
    double x3 = cx[o + 2];
    double x4 = cx[o + 3];
    double x5 = cx[o + 4];
    double y1 = cy[o];
    double y2 = cy[o + 1];
    double y3 = cy[o + 2];
    double y4 = cy[o + 3];
    double y5 = cy[o + 4];
    out[0] = originX[k] + u * (x1 + u * (x2 + u * (x3 + u * (x4 + u * x5))));
    out[1] = originY[k] + u * (y1 + u * (y2 + u * (y3 + u * (y4 + u * y5))));
    out[2] = x1 + u * (2.0 * x2 + u * (3.0 * x3 + u * (4.0 * x4 + u * 5.0 * x5)));
    out[3] = y1 + u * (2.0 * y2 + u * (3.0 * y3 + u * (4.0 * y4 + u * 5.0 * y5)));
    out[4] = 2.0 * x2 + u * (6.0 * x3 + u * (12.0 * x4 + u * 20.0 * x5));
    out[5] = 2.0 * y2 + u * (6.0 * y3 + u * (12.0 * y4 + u * 20.0 * y5));
    out[6] = 6.0 * x3 + u * (24.0 * x4 + u * 60.0 * x5);
    out[7] = 6.0 * y3 + u * (24.0 * y4 + u * 60.0 * y5);
  }

  /** |p'(u)| on piece {@code k}: metres of arc per unit of u. */
  public double speed(int k, double u) {
    int o = 5 * k;
    double dx = cx[o] + u * (2.0 * cx[o + 1] + u * (3.0 * cx[o + 2] + u * (4.0 * cx[o + 3] + u * 5.0 * cx[o + 4])));
    double dy = cy[o] + u * (2.0 * cy[o + 1] + u * (3.0 * cy[o + 2] + u * (4.0 * cy[o + 3] + u * 5.0 * cy[o + 4])));
    return Math.sqrt(dx * dx + dy * dy);
  }
}
