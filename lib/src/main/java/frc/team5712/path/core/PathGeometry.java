package frc.team5712.path.core;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * The path's shape, indexed by distance along it ({@code s}, metres).
 *
 * <p>The spline is parameterised by {@code u}, not distance, so this builds an arc-length table:
 * each spline piece is cut into sub-pieces, the length of each comes from 5-point Gauss-Legendre
 * quadrature, and inside a sub-piece {@code u(s)} is a cubic Hermite interpolant through the
 * end values and slopes ({@code du/ds = 1/|p'|}). Sub-pieces are halved until that interpolant puts
 * the point within {@link #LOOKUP_TOLERANCE_METERS} of where the exact inverse would, and until
 * halving no longer changes the quadrature. A query is then a binary search plus one polynomial,
 * and the point, tangent and curvature are the exact spline values at that {@code u}.
 *
 * <p><b>Cusps.</b> A spline can stop and double back: where |p'| falls to zero the curve reverses, and the
 * curvature there is infinite but only at a point, which a table or a grid steps over. {@link #cusps} finds
 * them once, at build: every local minimum of |p'| (scanned at {@link #CUSP_SCAN} steps per piece, then
 * golden-section) where the tangent {@link #CUSP_ARC_METERS} before and after turns by more than 120°. That
 * takes in near-cusps too (hairpins tighter than about 2 cm). The velocity profile stops the robot at each,
 * and the follower stops, then reverses; the editor flags them, since they are almost never meant.
 */
public final class PathGeometry {
  /** Worst position error of a lookup by s, measured at each sub-piece's middle while building. */
  public static final double LOOKUP_TOLERANCE_METERS = 1e-5;

  /** Sub-pieces whose length estimates disagree by more than this when halved are halved. */
  private static final double QUADRATURE_TOLERANCE_METERS = 1e-10;

  /** Each spline piece starts cut into equal-u sub-pieces of about this much chord. */
  private static final double MAX_INITIAL_PIECE_METERS = 0.5;

  private static final int MAX_DEPTH = 24;

  /** Samples of |p'| per spline piece when looking for cusps. */
  public static final int CUSP_SCAN = 64;

  /** A cusp turns the tangent more than 120° between this far before and this far after it. */
  public static final double CUSP_ARC_METERS = 0.02;

  /** cos 120°. */
  private static final double CUSP_COS = -0.5;

  /** 1 − 1/φ, the golden section. */
  private static final double GOLDEN = 0.3819660112501051;

  // 5-point Gauss-Legendre on [0, 1]: nodes 0.5 ± 0.5ξ, weights w/2
  private static final double GL_XI1 = 0.5384693101056830910363144207002088;
  private static final double GL_XI2 = 0.9061798459386639927976268782993929;
  private static final double GL_W0 = 0.5688888888888888888888888888888889 / 2.0;
  private static final double GL_W1 = 0.4786286704993664680412915148356382 / 2.0;
  private static final double GL_W2 = 0.2369268850561890875142640407199173 / 2.0;

  private final QuinticSpline spline;
  private final double lengthMeters;
  private final double[] waypointS;

  // Sub-piece table. pieceS has pieceCount + 1 entries; the last is the total length.
  private final int pieceCount;
  private final double[] pieceS;
  private final int[] pieceSegment;
  private final double[] pieceU0;
  private final double[] pieceU1;
  // u(σ) = u0 + σ(m0 + σ(c2 + σ c3)) with σ = (s − s0) / length
  private final double[] pieceM0;
  private final double[] pieceC2;
  private final double[] pieceC3;
  private final double[] cusps;

  private static final java.util.concurrent.atomic.AtomicLong BUILDS =
      new java.util.concurrent.atomic.AtomicLong();

  /**
   * How many geometries this JVM has built. A diagnostic for the rule that starting a path does no
   * path work: read it before and after.
   */
  public static long buildCount() {
    return BUILDS.get();
  }

  private PathGeometry(QuinticSpline spline, TableBuilder table, double[] waypointS) {
    BUILDS.incrementAndGet();
    this.spline = spline;
    this.waypointS = waypointS;
    this.lengthMeters = waypointS[waypointS.length - 1];
    this.pieceCount = table.count;
    this.pieceS = Arrays.copyOf(table.s, table.count + 1);
    this.pieceSegment = Arrays.copyOf(table.segment, table.count);
    this.pieceU0 = Arrays.copyOf(table.u0, table.count);
    this.pieceU1 = Arrays.copyOf(table.u1, table.count);
    this.pieceM0 = Arrays.copyOf(table.m0, table.count);
    this.pieceC2 = Arrays.copyOf(table.c2, table.count);
    this.pieceC3 = Arrays.copyOf(table.c3, table.count);
    this.cusps = findCusps();
  }

  /** Builds the spline through these waypoints and its arc-length table. */
  public static PathGeometry build(List<Waypoint> waypoints) {
    return build(QuinticSpline.through(waypoints));
  }

  /** The same, with the curvature at the first waypoint fixed (see {@link QuinticSpline}). */
  public static PathGeometry build(List<Waypoint> waypoints, double startCurvature) {
    return build(QuinticSpline.through(waypoints, startCurvature));
  }

  /** Builds the arc-length table for a spline. */
  public static PathGeometry build(QuinticSpline spline) {
    int n = spline.segmentCount();
    double[] waypointS = new double[n + 1];
    TableBuilder table = new TableBuilder(spline);
    for (int k = 0; k < n; k++) {
      int splits = Math.max(1, (int) Math.ceil(spline.chordMeters(k) / MAX_INITIAL_PIECE_METERS));
      double prevU = 0.0;
      double prevSpeed = table.speed(k, 0.0);
      for (int j = 1; j <= splits; j++) {
        double u = j == splits ? 1.0 : (double) j / splits;
        double speed = table.speed(k, u);
        table.refine(k, prevU, u, prevSpeed, speed, table.gaussLength(k, prevU, u), 0);
        prevU = u;
        prevSpeed = speed;
      }
      waypointS[k + 1] = table.totalS;
    }
    table.s[table.count] = table.totalS;
    return new PathGeometry(spline, table, waypointS);
  }

  /** The spline under this table. */
  public QuinticSpline spline() {
    return spline;
  }

  /** Total length in metres. */
  public double lengthMeters() {
    return lengthMeters;
  }

  /** Number of waypoints. */
  public int waypointCount() {
    return waypointS.length;
  }

  /** Metres along the path at waypoint {@code i}. */
  public double waypointS(int i) {
    return waypointS[i];
  }

  /**
   * Metres along the path at a waypoint-relative position: {@code 1.5} is halfway along the arc
   * from waypoint 1 to waypoint 2. Clamped to the path.
   */
  public double sAtWaypoint(double waypoint) {
    int last = waypointS.length - 1;
    if (!(waypoint > 0.0)) {
      return 0.0;
    }
    if (waypoint >= last) {
      return lengthMeters;
    }
    int k = (int) Math.floor(waypoint);
    double f = waypoint - k;
    return waypointS[k] + f * (waypointS[k + 1] - waypointS[k]);
  }

  /** Where the path reverses (cusps and near-cusps), metres along it, ascending; empty for most paths. */
  public double[] cusps() {
    return cusps.clone();
  }

  /** True when the path has a cusp. */
  public boolean hasCusps() {
    return cusps.length > 0;
  }

  /** The first cusp after {@code s} (by more than a micrometre), or NaN. */
  public double nextCusp(double s) {
    for (double c : cusps) {
      if (c > s + 1e-6) {
        return c;
      }
    }
    return Double.NaN;
  }

  /** The last cusp at or before {@code s} (within a micrometre), or NaN. */
  public double previousCusp(double s) {
    double out = Double.NaN;
    for (double c : cusps) {
      if (c <= s + 1e-6) {
        out = c;
      }
    }
    return out;
  }

  private double[] findCusps() {
    List<Double> out = new ArrayList<>();
    double[] v = new double[CUSP_SCAN + 1];
    for (int k = 0; k < spline.segmentCount(); k++) {
      for (int j = 0; j <= CUSP_SCAN; j++) {
        v[j] = spline.speed(k, (double) j / CUSP_SCAN);
      }
      for (int j = 1; j < CUSP_SCAN; j++) {
        if (!(v[j] <= v[j - 1] && v[j] < v[j + 1])) {
          continue;
        }
        double a = (j - 1.0) / CUSP_SCAN;
        double b = (j + 1.0) / CUSP_SCAN;
        for (int it = 0; it < 60; it++) {
          double m1 = a + (b - a) * GOLDEN;
          double m2 = b - (b - a) * GOLDEN;
          if (spline.speed(k, m1) < spline.speed(k, m2)) {
            b = m2;
          } else {
            a = m1;
          }
        }
        double s = sAtParameter(k, 0.5 * (a + b));
        GeometrySample before = sample(Math.max(0.0, s - CUSP_ARC_METERS));
        GeometrySample after = sample(Math.min(lengthMeters, s + CUSP_ARC_METERS));
        if (before.tangentX() * after.tangentX() + before.tangentY() * after.tangentY() < CUSP_COS) {
          out.add(s);
        }
      }
    }
    double[] c = new double[out.size()];
    for (int i = 0; i < c.length; i++) {
      c[i] = out.get(i);
    }
    return c;
  }

  /** Metres along the path at spline piece {@code k}, parameter {@code u}: the table's own inverse, by bisection. */
  private double sAtParameter(int k, double u) {
    double lo = waypointS[k];
    double hi = waypointS[k + 1];
    for (int it = 0; it < 60; it++) {
      double mid = 0.5 * (lo + hi);
      int i = findPiece(mid);
      double len = pieceS[i + 1] - pieceS[i];
      double sigma = len > 0.0 ? (mid - pieceS[i]) / len : 0.0;
      double um = pieceU0[i] + sigma * (pieceM0[i] + sigma * (pieceC2[i] + sigma * pieceC3[i]));
      if (pieceSegment[i] < k || (pieceSegment[i] == k && um < u)) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    return 0.5 * (lo + hi);
  }

  /** Number of sub-pieces in the arc-length table (for tests and tuning). */
  public int tablePieceCount() {
    return pieceCount;
  }

  /** The path at {@code s} metres along it (clamped to the path). */
  public GeometrySample sample(double s) {
    double[] d = new double[8];
    double sc = evaluateAt(s, d);
    return toSample(sc, d);
  }

  /** The point at {@code s}. */
  public Vec2 pointAt(double s) {
    double[] d = new double[8];
    evaluateAt(s, d);
    return new Vec2(d[0], d[1]);
  }

  /** Signed curvature at {@code s}, 1/m. */
  public double curvatureAt(double s) {
    return sample(s).curvature();
  }

  /**
   * Finds the spline piece and parameter for {@code s} and evaluates the spline there into
   * {@code d} (see {@link QuinticSpline#evaluate}). Returns s clamped to the path.
   */
  double evaluateAt(double s, double[] d) {
    double sc = s < 0.0 ? 0.0 : (s > lengthMeters ? lengthMeters : s);
    int i = findPiece(sc);
    double len = pieceS[i + 1] - pieceS[i];
    double sigma = len > 0.0 ? (sc - pieceS[i]) / len : 0.0;
    double u = pieceU0[i] + sigma * (pieceM0[i] + sigma * (pieceC2[i] + sigma * pieceC3[i]));
    if (u < pieceU0[i]) {
      u = pieceU0[i];
    } else if (u > pieceU1[i]) {
      u = pieceU1[i];
    }
    spline.evaluate(pieceSegment[i], u, d);
    return sc;
  }

  /** The last sub-piece starting at or before s. */
  private int findPiece(double s) {
    int lo = 0;
    int hi = pieceCount - 1;
    while (lo < hi) {
      int mid = (lo + hi + 1) >>> 1;
      if (pieceS[mid] <= s) {
        lo = mid;
      } else {
        hi = mid - 1;
      }
    }
    return lo;
  }

  static GeometrySample toSample(double s, double[] d) {
    double dx = d[2];
    double dy = d[3];
    double q = dx * dx + dy * dy;
    double speed = Math.sqrt(q);
    double cross = dx * d[5] - dy * d[4];
    double dot = dx * d[4] + dy * d[5];
    double dCross = dx * d[7] - dy * d[6];
    double q15 = q * speed;
    double curvature = cross / q15;
    double dCurvatureDu = dCross / q15 - 3.0 * cross * dot / (q * q15);
    return new GeometrySample(
        s, d[0], d[1], dx / speed, dy / speed, curvature, dCurvatureDu / speed);
  }

  // ------------------------------------------------------------------------------ projection

  /** Coarse spacing for the windowed projection, metres. */
  private static final double WINDOW_STEP_METERS = 0.05;

  /** Coarse spacing for the global projection, metres. */
  private static final double GLOBAL_STEP_METERS = 0.10;

  /**
   * The closest point to (x, y) with s inside {@code [sGuess − halfWindow, sGuess + halfWindow]}:
   * the way a follower tracks progress, so a path that passes near itself can't make it jump ahead.
   * A coarse scan picks the best sample, then safeguarded Newton on d(distance²)/ds = 0 polishes it;
   * the answer is clamped to the window.
   */
  public Projection project(double xMeters, double yMeters, double sGuess, double halfWindowMeters) {
    double lo = Math.max(0.0, sGuess - halfWindowMeters);
    double hi = Math.min(lengthMeters, sGuess + halfWindowMeters);
    if (hi < lo) { // the guess is off the path entirely: use the nearer end
      lo = hi = sGuess < 0.0 ? 0.0 : lengthMeters;
    }
    return closestBetween(xMeters, yMeters, lo, hi, WINDOW_STEP_METERS);
  }

  /** The closest point to (x, y) with s in [lo, hi] (the window's spacing): a window that stops at a cusp. */
  public Projection projectWithin(double xMeters, double yMeters, double lo, double hi) {
    lo = Math.max(0.0, lo);
    hi = Math.min(lengthMeters, hi);
    if (hi < lo) {
      hi = lo;
    }
    return closestBetween(xMeters, yMeters, lo, hi, WINDOW_STEP_METERS);
  }

  /** The closest point to (x, y) with s in [lo, hi], at the global search's spacing. */
  public Projection projectGlobalWithin(double xMeters, double yMeters, double lo, double hi) {
    lo = Math.max(0.0, lo);
    hi = Math.min(lengthMeters, hi);
    if (hi < lo) {
      hi = lo;
    }
    return closestBetween(xMeters, yMeters, lo, hi, GLOBAL_STEP_METERS);
  }

  /** The closest point on the whole path to (x, y), for starting up or after losing track. */
  public Projection projectGlobal(double xMeters, double yMeters) {
    return closestBetween(xMeters, yMeters, 0.0, lengthMeters, GLOBAL_STEP_METERS);
  }

  private Projection closestBetween(double qx, double qy, double lo, double hi, double step) {
    double[] d = new double[8];
    int steps = Math.max(1, (int) Math.ceil((hi - lo) / step));
    double h = (hi - lo) / steps;
    int best = 0;
    double bestDistSq = Double.POSITIVE_INFINITY;
    for (int i = 0; i <= steps; i++) {
      evaluateAt(lo + i * h, d);
      double ex = d[0] - qx;
      double ey = d[1] - qy;
      double distSq = ex * ex + ey * ey;
      if (distSq < bestDistSq) {
        bestDistSq = distSq;
        best = i;
      }
    }
    double sBest = lo + best * h;
    double a = best == 0 ? lo : lo + (best - 1) * h;
    double b = best == steps ? hi : lo + (best + 1) * h;

    // f(s) = (p(s) − q)·T(s) is half the slope of distance²; f' = 1 + κ (p − q)·N
    double s = sBest;
    for (int iter = 0; iter < 30; iter++) {
      evaluateAt(s, d);
      GeometrySample g = toSample(s, d);
      double ex = g.xMeters() - qx;
      double ey = g.yMeters() - qy;
      double f = ex * g.tangentX() + ey * g.tangentY();
      double fp = 1.0 + g.curvature() * (-ex * g.tangentY() + ey * g.tangentX());
      if (f > 0.0) {
        b = s;
      } else {
        a = s;
      }
      double next = fp > 0.0 ? s - f / fp : Double.NaN;
      if (!(next > a && next < b)) {
        next = 0.5 * (a + b);
      }
      if (Math.abs(next - s) < 1e-10) {
        s = next;
        break;
      }
      s = next;
    }

    // Keep whichever is closer: the polished point or the best coarse sample
    double sFinal = distanceSq(s, qx, qy, d) <= bestDistSq ? s : sBest;
    evaluateAt(sFinal, d);
    GeometrySample g = toSample(sFinal, d);
    double ex = qx - g.xMeters();
    double ey = qy - g.yMeters();
    return new Projection(
        sFinal,
        g.xMeters(),
        g.yMeters(),
        Math.sqrt(ex * ex + ey * ey),
        -ex * g.tangentY() + ey * g.tangentX());
  }

  private double distanceSq(double s, double qx, double qy, double[] d) {
    evaluateAt(s, d);
    double ex = d[0] - qx;
    double ey = d[1] - qy;
    return ex * ex + ey * ey;
  }

  // ------------------------------------------------------------------------------ table building

  /** Growable arrays for the sub-piece table, plus the adaptive refinement. */
  private static final class TableBuilder {
    final QuinticSpline spline;
    int count;
    double totalS;
    double[] s = new double[64];
    int[] segment = new int[64];
    double[] u0 = new double[64];
    double[] u1 = new double[64];
    double[] m0 = new double[64];
    double[] c2 = new double[64];
    double[] c3 = new double[64];

    TableBuilder(QuinticSpline spline) {
      this.spline = spline;
    }

    double speed(int k, double u) {
      double v = spline.speed(k, u);
      return v > 1e-12 ? v : 1e-12; // a cusp: keep the slopes finite, the refinement copes
    }

    /** Arc length of piece k from ua to ub, 5-point Gauss-Legendre. */
    double gaussLength(int k, double ua, double ub) {
      double h = ub - ua;
      double mid = ua + 0.5 * h;
      double o1 = 0.5 * GL_XI1 * h;
      double o2 = 0.5 * GL_XI2 * h;
      double sum = GL_W0 * spline.speed(k, mid);
      sum += GL_W1 * (spline.speed(k, mid - o1) + spline.speed(k, mid + o1));
      sum += GL_W2 * (spline.speed(k, mid - o2) + spline.speed(k, mid + o2));
      return sum * h;
    }

    /** Halves [ua, ub] until the u(s) interpolant and the quadrature are both good enough. */
    void refine(int k, double ua, double ub, double speedA, double speedB, double length, int depth) {
      double um = 0.5 * (ua + ub);
      double lengthLeft = gaussLength(k, ua, um);
      double lengthRight = gaussLength(k, um, ub);
      double total = lengthLeft + lengthRight;
      double speedM = speed(k, um);

      // Where the interpolant puts the arc-length midpoint of the halves, against where it is
      double ma = total / speedA;
      double mb = total / speedB;
      double cc2 = 3.0 * (ub - ua) - 2.0 * ma - mb;
      double cc3 = 2.0 * (ua - ub) + ma + mb;
      double sigma = lengthLeft / total;
      double uHat = ua + sigma * (ma + sigma * (cc2 + sigma * cc3));
      double positionError = Math.abs(uHat - um) * speedM;
      double quadratureError = Math.abs(total - length);

      if (depth < MAX_DEPTH
          && (positionError > LOOKUP_TOLERANCE_METERS || quadratureError > QUADRATURE_TOLERANCE_METERS)) {
        refine(k, ua, um, speedA, speedM, lengthLeft, depth + 1);
        refine(k, um, ub, speedM, speedB, lengthRight, depth + 1);
        return;
      }
      add(k, ua, ub, ma, cc2, cc3, total);
    }

    private void add(int k, double ua, double ub, double ma, double cc2, double cc3, double length) {
      if (count + 1 >= s.length) {
        int cap = s.length * 2;
        s = Arrays.copyOf(s, cap);
        segment = Arrays.copyOf(segment, cap);
        u0 = Arrays.copyOf(u0, cap);
        u1 = Arrays.copyOf(u1, cap);
        m0 = Arrays.copyOf(m0, cap);
        c2 = Arrays.copyOf(c2, cap);
        c3 = Arrays.copyOf(c3, cap);
      }
      s[count] = totalS;
      segment[count] = k;
      u0[count] = ua;
      u1[count] = ub;
      m0[count] = ma;
      c2[count] = cc2;
      c3[count] = cc3;
      count++;
      totalS += length;
    }
  }
}
