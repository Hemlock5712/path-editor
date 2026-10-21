package frc.team5712.path.plan;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A convex polygon, counter-clockwise, stored as flat coordinate arrays. Immutable.
 *
 * <p>{@link #of} takes the convex hull of whatever it is given, so a concave obstacle is planned
 * around as its hull (conservative).
 */
public final class ConvexPolygon {
  /** Points closer than this to the boundary count as on it (outside the open interior). */
  static final double EPS = 1e-7;

  final double[] xs;
  final double[] ys;
  /** Outward unit normal of edge i (from vertex i to i + 1). */
  final double[] nx;
  final double[] ny;
  final double cx;
  final double cy;
  /** Distance from (cx, cy) to the farthest vertex. */
  final double radius;

  private ConvexPolygon(double[] xs, double[] ys) {
    int n = xs.length;
    if (n < 3) {
      throw new IllegalArgumentException("a polygon needs at least three distinct corners");
    }
    this.xs = xs;
    this.ys = ys;
    nx = new double[n];
    ny = new double[n];
    double sx = 0;
    double sy = 0;
    for (int i = 0; i < n; i++) {
      int j = (i + 1) % n;
      double ex = xs[j] - xs[i];
      double ey = ys[j] - ys[i];
      double len = Math.hypot(ex, ey);
      nx[i] = ey / len; // right of a CCW edge is outside
      ny[i] = -ex / len;
      sx += xs[i];
      sy += ys[i];
    }
    cx = sx / n;
    cy = sy / n;
    double r = 0;
    for (int i = 0; i < n; i++) {
      r = Math.max(r, Math.hypot(xs[i] - cx, ys[i] - cy));
    }
    radius = r;
  }

  /** The convex hull of the points (Andrew's monotone chain), counter-clockwise. */
  public static ConvexPolygon of(double[][] points) {
    double[][] p = points.clone();
    Arrays.sort(p, (a, b) -> a[0] != b[0] ? Double.compare(a[0], b[0]) : Double.compare(a[1], b[1]));
    int n = p.length;
    double[][] hull = new double[2 * n][];
    int k = 0;
    for (int i = 0; i < n; i++) {
      while (k >= 2 && cross(hull[k - 2], hull[k - 1], p[i]) <= 1e-12) {
        k--;
      }
      hull[k++] = p[i];
    }
    for (int i = n - 2, t = k + 1; i >= 0; i--) {
      while (k >= t && cross(hull[k - 2], hull[k - 1], p[i]) <= 1e-12) {
        k--;
      }
      hull[k++] = p[i];
    }
    int m = k - 1; // the last point repeats the first
    double[] xs = new double[m];
    double[] ys = new double[m];
    for (int i = 0; i < m; i++) {
      xs[i] = hull[i][0];
      ys[i] = hull[i][1];
    }
    return new ConvexPolygon(xs, ys);
  }

  private static double cross(double[] o, double[] a, double[] b) {
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  }

  public int size() {
    return xs.length;
  }

  public double x(int i) {
    return xs[i];
  }

  public double y(int i) {
    return ys[i];
  }

  /**
   * The Minkowski sum with a disc of radius {@code r}, approximated from outside: each corner's arc
   * is split into steps of at most {@code maxStepRadians} and its points sit at r / cos(step / 2), so
   * every chord stays at least r from the corner. The result contains the exact sum.
   */
  public ConvexPolygon grow(double r, double maxStepRadians) {
    List<double[]> pts = new ArrayList<>();
    int n = xs.length;
    for (int i = 0; i < n; i++) {
      int prev = (i + n - 1) % n;
      double a0 = StrictMath.atan2(ny[prev], nx[prev]);
      double a1 = StrictMath.atan2(ny[i], nx[i]);
      double turn = a1 - a0;
      while (turn < 0) {
        turn += 2 * Math.PI;
      }
      int steps = Math.max(1, (int) Math.ceil(turn / maxStepRadians - 1e-9));
      double step = turn / steps;
      double rr = r / StrictMath.cos(step / 2);
      for (int k = 0; k <= steps; k++) {
        double a = a0 + k * step;
        pts.add(new double[] {xs[i] + rr * StrictMath.cos(a), ys[i] + rr * StrictMath.sin(a)});
      }
    }
    return of(pts.toArray(new double[0][]));
  }

  /** True when (x, y) is in the open interior (more than EPS inside every edge). */
  public boolean containsStrictly(double x, double y) {
    for (int i = 0; i < xs.length; i++) {
      if (nx[i] * (x - xs[i]) + ny[i] * (y - ys[i]) >= -EPS) {
        return false;
      }
    }
    return true;
  }

  /**
   * True when the segment from (x0, y0) to (x1, y1) passes through the open interior (Cyrus-Beck
   * clipping). Touching a corner or running along an edge does not count.
   */
  public boolean blocksSegment(double x0, double y0, double x1, double y1) {
    double dx = x1 - x0;
    double dy = y1 - y0;
    if (segmentPointDistance(x0, y0, x1, y1, cx, cy) > radius) {
      return false; // misses the bounding circle
    }
    double tEnter = 0.0;
    double tLeave = 1.0;
    for (int i = 0; i < xs.length; i++) {
      double num = nx[i] * (x0 - xs[i]) + ny[i] * (y0 - ys[i]) + EPS;
      double den = nx[i] * dx + ny[i] * dy;
      if (Math.abs(den) < 1e-15) {
        if (num >= 0) {
          return false; // parallel and outside (or on) this edge
        }
        continue;
      }
      double t = -num / den;
      if (den < 0) {
        tEnter = Math.max(tEnter, t);
      } else {
        tLeave = Math.min(tLeave, t);
      }
      if (tEnter >= tLeave) {
        return false;
      }
    }
    return tLeave - tEnter > 1e-9;
  }

  /** Distance from (x, y) to the polygon: 0 inside. */
  public double distanceTo(double x, double y) {
    boolean inside = true;
    for (int i = 0; i < xs.length; i++) {
      if (nx[i] * (x - xs[i]) + ny[i] * (y - ys[i]) > 0) {
        inside = false;
        break;
      }
    }
    if (inside) {
      return 0.0;
    }
    double best = Double.POSITIVE_INFINITY;
    for (int i = 0; i < xs.length; i++) {
      int j = (i + 1) % xs.length;
      best = Math.min(best, segmentPointDistance(xs[i], ys[i], xs[j], ys[j], x, y));
    }
    return best;
  }

  /** The closest point on the boundary to (x, y), from inside or outside. */
  public double[] nearestBoundaryPoint(double x, double y) {
    double best = Double.POSITIVE_INFINITY;
    double bx = 0;
    double by = 0;
    for (int i = 0; i < xs.length; i++) {
      int j = (i + 1) % xs.length;
      double ex = xs[j] - xs[i];
      double ey = ys[j] - ys[i];
      double t = Math.max(0.0, Math.min(1.0, ((x - xs[i]) * ex + (y - ys[i]) * ey) / (ex * ex + ey * ey)));
      double px = xs[i] + t * ex;
      double py = ys[i] + t * ey;
      double d = Math.hypot(x - px, y - py);
      if (d < best) {
        best = d;
        bx = px;
        by = py;
      }
    }
    return new double[] {bx, by};
  }

  static double segmentPointDistance(double ax, double ay, double bx, double by, double px, double py) {
    double ex = bx - ax;
    double ey = by - ay;
    double l2 = ex * ex + ey * ey;
    double t = l2 == 0 ? 0 : Math.max(0.0, Math.min(1.0, ((px - ax) * ex + (py - ay) * ey) / l2));
    return Math.hypot(px - ax - t * ex, py - ay - t * ey);
  }

  /**
   * Distance between this polygon and another convex polygon given by corner arrays (counter-clockwise):
   * 0 when they touch or overlap.
   */
  public double distanceToPolygon(double[] qx, double[] qy) {
    int m = qx.length;
    // overlap: any corner of one inside the other, or crossing edges
    for (int k = 0; k < m; k++) {
      if (distanceTo(qx[k], qy[k]) == 0.0) {
        return 0.0;
      }
    }
    for (int i = 0; i < xs.length; i++) {
      if (insideCcw(qx, qy, xs[i], ys[i])) {
        return 0.0;
      }
    }
    double best = Double.POSITIVE_INFINITY;
    for (int i = 0; i < xs.length; i++) {
      int j = (i + 1) % xs.length;
      for (int k = 0; k < m; k++) {
        int l = (k + 1) % m;
        if (segmentsCross(xs[i], ys[i], xs[j], ys[j], qx[k], qy[k], qx[l], qy[l])) {
          return 0.0;
        }
        best = Math.min(best, segmentPointDistance(xs[i], ys[i], xs[j], ys[j], qx[k], qy[k]));
        best = Math.min(best, segmentPointDistance(qx[k], qy[k], qx[l], qy[l], xs[i], ys[i]));
      }
    }
    return best;
  }

  private static boolean insideCcw(double[] qx, double[] qy, double x, double y) {
    int m = qx.length;
    for (int k = 0; k < m; k++) {
      int l = (k + 1) % m;
      if ((qx[l] - qx[k]) * (y - qy[k]) - (qy[l] - qy[k]) * (x - qx[k]) < 0) {
        return false;
      }
    }
    return true;
  }

  private static boolean segmentsCross(
      double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy) {
    double d1 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    double d2 = (bx - ax) * (dy - ay) - (by - ay) * (dx - ax);
    double d3 = (dx - cx) * (ay - cy) - (dy - cy) * (ax - cx);
    double d4 = (dx - cx) * (by - cy) - (dy - cy) * (bx - cx);
    return d1 * d2 < 0 && d3 * d4 < 0;
  }
}
