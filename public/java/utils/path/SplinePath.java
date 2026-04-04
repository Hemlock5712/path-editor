package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.List;

/**
 * Arc-length parameterized Catmull-Rom spline path.
 *
 * <p>Takes a list of control points and builds a smooth cubic spline that passes through all
 * points. The path is parameterized by arc length (meters) rather than the spline parameter t, so
 * all queries use physical distance along the path.
 *
 * <p>Arc-length parameterization is computed via 5-point Gauss-Legendre quadrature with a dense
 * lookup table for O(log N) queries.
 */
public final class SplinePath {

  /** Samples per meter of path length for the arc-length lookup table. */
  private static final int SAMPLES_PER_METER = 1000;

  /** Minimum samples to ensure even short paths have adequate resolution. */
  private static final int MIN_SAMPLES = 100;

  /** Coarse search step size in meters for closest-point projection. */
  private static final double COARSE_SEARCH_STEP = 0.05;

  /** Maximum Newton-Raphson iterations for closest-point refinement. */
  private static final int MAX_NEWTON_ITERATIONS = 8;

  // 5-point Gauss-Legendre quadrature nodes and weights on [-1, 1]
  private static final double[] GL_NODES = {
    -0.9061798459386640, -0.5384693101056831, 0.0, 0.5384693101056831, 0.9061798459386640
  };
  private static final double[] GL_WEIGHTS = {
    0.2369268850561891,
    0.4786286704993665,
    0.5688888888888889,
    0.4786286704993665,
    0.2369268850561891
  };

  private final List<Translation2d> controlPoints;
  private final CubicSegment[] segments;

  // Arc-length lookup table: sTable[i] = cumulative arc length at the i-th sample
  // tTable[i] = the corresponding (segmentIndex, localT) encoded for fast lookup
  private final double[] sTable;
  private final int[] segIndexTable;
  private final double[] tTable;
  private final double totalLength;

  /**
   * Creates a spline path through the given control points using Catmull-Rom interpolation.
   *
   * @param controlPoints Ordered list of points the path passes through (minimum 2)
   * @throws IllegalArgumentException if fewer than 2 control points
   */
  public SplinePath(List<Translation2d> controlPoints) {
    if (controlPoints.size() < 2) {
      throw new IllegalArgumentException("SplinePath requires at least 2 control points");
    }

    this.controlPoints = List.copyOf(controlPoints);
    int n = controlPoints.size();

    // Build Catmull-Rom segments (N-1 segments for N points)
    segments = new CubicSegment[n - 1];
    for (int i = 0; i < n - 1; i++) {
      Translation2d p0 = controlPoints.get(i);
      Translation2d p1 = controlPoints.get(i + 1);

      // Catmull-Rom tangents: 0.5 * (P_{i+1} - P_{i-1})
      // Endpoints use chord to single neighbor
      Translation2d m0;
      if (i == 0) {
        m0 = p1.minus(p0);
      } else {
        m0 = controlPoints.get(i + 1).minus(controlPoints.get(i - 1)).times(0.5);
      }

      Translation2d m1;
      if (i == n - 2) {
        m1 = p1.minus(p0);
      } else {
        m1 = controlPoints.get(i + 2).minus(controlPoints.get(i)).times(0.5);
      }

      segments[i] = new CubicSegment(p0, p1, m0, m1);
    }

    // Build arc-length lookup table
    // First pass: compute total length to determine table size
    double roughLength = 0;
    for (CubicSegment segment : segments) {
      roughLength += integrateSegmentLength(segment, 0, 1);
    }

    int numSamples = Math.max(MIN_SAMPLES, (int) (roughLength * SAMPLES_PER_METER));
    sTable = new double[numSamples + 1];
    segIndexTable = new int[numSamples + 1];
    tTable = new double[numSamples + 1];

    buildArcLengthTable(numSamples);
    totalLength = sTable[numSamples];
  }

  /**
   * Creates a spline path through the given control points.
   *
   * @param controlPoints Control points (minimum 2)
   */
  public SplinePath(Translation2d... controlPoints) {
    this(Arrays.asList(controlPoints));
  }

  /**
   * Returns the position at arc length s along the path.
   *
   * @param s Arc length in meters, clamped to [0, totalLength]
   * @return Position on the path
   */
  public Translation2d getPoint(double s) {
    s = clampS(s);
    LookupResult lr = lookupS(s);
    return segments[lr.segIndex].getPoint(lr.t);
  }

  /**
   * Returns the unit tangent vector at arc length s.
   *
   * @param s Arc length in meters, clamped to [0, totalLength]
   * @return Unit tangent vector (direction of travel)
   */
  public Translation2d getTangent(double s) {
    s = clampS(s);
    LookupResult lr = lookupS(s);
    Translation2d d = segments[lr.segIndex].getDerivative(lr.t);
    double mag = d.getNorm();
    if (mag < 1e-12) {
      return new Translation2d(1, 0);
    }
    return d.div(mag);
  }

  /**
   * Returns the signed curvature at arc length s.
   *
   * @param s Arc length in meters, clamped to [0, totalLength]
   * @return Signed curvature in 1/meters (positive = turning left)
   */
  public double getCurvature(double s) {
    s = clampS(s);
    LookupResult lr = lookupS(s);
    return segments[lr.segIndex].getCurvature(lr.t);
  }

  /**
   * Returns the total path length in meters.
   *
   * @return Total arc length
   */
  public double getTotalLength() {
    return totalLength;
  }

  /**
   * Returns the arc length at the given waypoint index.
   *
   * <p>Waypoint index i corresponds to control point i. The globalT for waypoint i is just i
   * (segment i spans globalT i to i+1).
   *
   * @param waypointIndex The waypoint index (0-based)
   * @return Arc length in meters at that waypoint
   */
  public double getArcLengthAtWaypointIndex(int waypointIndex) {
    if (waypointIndex <= 0) return 0;
    if (waypointIndex >= segments.length) return totalLength;
    int numSamples = sTable.length - 1;
    int tableIndex = (int) Math.round((double) waypointIndex * numSamples / segments.length);
    return sTable[Math.min(tableIndex, numSamples)];
  }

  /**
   * Projects a point onto the path, searching the entire path.
   *
   * <p>Use this for initial projection when no prior position is known (e.g., at command start).
   * For per-cycle tracking, use {@link #getClosestPointInRange} instead to prevent the projection
   * from jumping across crossings or to distant segments.
   *
   * @param robotPosition The point to project onto the path
   * @return Projection result with arc length, closest point, cross-track error, and tangent
   */
  public ProjectionResult getClosestPoint(Translation2d robotPosition) {
    double bestS = searchRange(robotPosition, 0, totalLength);
    bestS = refineProjection(bestS, robotPosition, 0, totalLength);
    return buildProjectionResult(bestS, robotPosition);
  }

  /**
   * Projects a point onto the path within a bounded arc-length range.
   *
   * <p>Use this for per-cycle tracking. By bounding the search to a small window around the last
   * known position, the projection cannot jump to a distant segment when the robot is hit, and
   * cannot cross to the wrong segment on a self-intersecting path.
   *
   * @param robotPosition The point to project onto the path
   * @param sMin Minimum arc length to search (clamped to 0)
   * @param sMax Maximum arc length to search (clamped to totalLength)
   * @return Projection result with arc length, closest point, cross-track error, and tangent
   */
  public ProjectionResult getClosestPointInRange(
      Translation2d robotPosition, double sMin, double sMax) {
    sMin = Math.max(0, sMin);
    sMax = Math.min(totalLength, sMax);
    if (sMin >= sMax) {
      // Degenerate range: return the clamped endpoint
      return buildProjectionResult(sMin, robotPosition);
    }
    double bestS = searchRange(robotPosition, sMin, sMax);
    bestS = refineProjection(bestS, robotPosition, sMin, sMax);
    return buildProjectionResult(bestS, robotPosition);
  }

  /** Returns the number of cubic segments. */
  public int getNumSegments() {
    return segments.length;
  }

  /** Returns the control points (unmodifiable). */
  public List<Translation2d> getControlPoints() {
    return controlPoints;
  }

  // ---- Arc-length table construction ----

  /** Builds the arc-length lookup table by uniformly sampling the parameter space. */
  private void buildArcLengthTable(int numSamples) {
    // Total parameter range is [0, numSegments]. Each segment has t in [0, 1].
    double totalT = segments.length;
    double dtPerSample = totalT / numSamples;

    sTable[0] = 0;
    segIndexTable[0] = 0;
    tTable[0] = 0;

    double cumulativeS = 0;

    for (int i = 1; i <= numSamples; i++) {
      double globalT = i * dtPerSample;
      int segIdx = Math.min((int) globalT, segments.length - 1);
      double localT = globalT - segIdx;

      // Integrate from previous sample to this one
      double prevGlobalT = (i - 1) * dtPerSample;
      int prevSegIdx = Math.min((int) prevGlobalT, segments.length - 1);
      double prevLocalT = prevGlobalT - prevSegIdx;

      if (prevSegIdx == segIdx) {
        // Same segment: integrate directly
        cumulativeS += integrateSegmentLength(segments[segIdx], prevLocalT, localT);
      } else {
        // Crosses segment boundary: integrate to end of prev, then from start of new
        cumulativeS += integrateSegmentLength(segments[prevSegIdx], prevLocalT, 1.0);
        cumulativeS += integrateSegmentLength(segments[segIdx], 0.0, localT);
      }

      sTable[i] = cumulativeS;
      segIndexTable[i] = segIdx;
      tTable[i] = localT;
    }
  }

  /**
   * Integrates arc length over a portion of a segment using 5-point Gauss-Legendre quadrature.
   *
   * @param segment The cubic segment
   * @param t0 Start parameter
   * @param t1 End parameter
   * @return Arc length of the segment from t0 to t1
   */
  private static double integrateSegmentLength(CubicSegment segment, double t0, double t1) {
    if (Math.abs(t1 - t0) < 1e-12) {
      return 0;
    }

    // Transform from [-1, 1] to [t0, t1]
    double halfRange = (t1 - t0) / 2.0;
    double midpoint = (t0 + t1) / 2.0;

    double sum = 0;
    for (int i = 0; i < GL_NODES.length; i++) {
      double t = midpoint + halfRange * GL_NODES[i];
      sum += GL_WEIGHTS[i] * segment.getSpeed(t);
    }
    return sum * halfRange;
  }

  // ---- Arc-length lookup ----

  /** Result of looking up arc length in the table. */
  private record LookupResult(int segIndex, double t) {}

  /**
   * Converts arc length s to (segmentIndex, localT) via binary search and interpolation.
   *
   * @param s Arc length in meters (assumed clamped)
   * @return Segment index and local parameter
   */
  private LookupResult lookupS(double s) {
    if (s <= 0) {
      return new LookupResult(0, 0);
    }
    if (s >= totalLength) {
      return new LookupResult(segments.length - 1, 1.0);
    }

    // Binary search for the bracket
    int lo = 0;
    int hi = sTable.length - 1;
    while (lo + 1 < hi) {
      int mid = (lo + hi) >>> 1;
      if (sTable[mid] <= s) {
        lo = mid;
      } else {
        hi = mid;
      }
    }

    // Linear interpolation within the bracket
    double sRange = sTable[hi] - sTable[lo];
    double frac = (sRange > 1e-12) ? (s - sTable[lo]) / sRange : 0;

    // Interpolate segment index and local t
    int segLo = segIndexTable[lo];
    double tLo = tTable[lo];
    int segHi = segIndexTable[hi];
    double tHi = tTable[hi];

    if (segLo == segHi) {
      // Same segment: simple interpolation
      return new LookupResult(segLo, tLo + frac * (tHi - tLo));
    } else {
      // Crosses segment boundary: pick the side based on fraction
      if (frac < 0.5) {
        double t = tLo + frac * 2 * (1.0 - tLo);
        return new LookupResult(segLo, Math.min(t, 1.0));
      } else {
        double t = (frac - 0.5) * 2 * tHi;
        return new LookupResult(segHi, Math.max(t, 0.0));
      }
    }
  }

  private double clampS(double s) {
    return Math.max(0, Math.min(s, totalLength));
  }

  // ---- Closest-point projection ----

  /** Builds a ProjectionResult from an arc-length and robot position. */
  private ProjectionResult buildProjectionResult(double s, Translation2d robotPosition) {
    Translation2d point = getPoint(s);
    Translation2d tangent = getTangent(s);
    Translation2d toRobot = robotPosition.minus(point);

    // Cross-track error: signed distance, positive = left of path direction
    // Cross product of tangent x toRobot gives signed perpendicular distance
    double crossTrack = tangent.getX() * toRobot.getY() - tangent.getY() * toRobot.getX();

    return new ProjectionResult(s, point, crossTrack, tangent);
  }

  /** Searches a range of arc lengths for the closest point to the query. */
  private double searchRange(Translation2d query, double sStart, double sEnd) {
    double bestS = sStart;
    double bestDistSq = Double.MAX_VALUE;

    int numSteps = Math.max(1, (int) ((sEnd - sStart) / COARSE_SEARCH_STEP));
    double step = (sEnd - sStart) / numSteps;

    for (int i = 0; i <= numSteps; i++) {
      double s = sStart + i * step;
      Translation2d point = getPoint(s);
      double distSq = squaredDist(query, point);
      if (distSq < bestDistSq) {
        bestDistSq = distSq;
        bestS = s;
      }
    }
    return bestS;
  }

  /**
   * Refines a coarse projection using Newton-Raphson iteration, clamped to [sMin, sMax].
   *
   * <p>Minimizes f(s) = dot(path(s) - robot, tangent(s)). When f(s) = 0, the vector from the path
   * to the robot is perpendicular to the tangent, meaning we've found the closest point.
   */
  private double refineProjection(
      double s, Translation2d robotPosition, double sMin, double sMax) {
    for (int iter = 0; iter < MAX_NEWTON_ITERATIONS; iter++) {
      Translation2d point = getPoint(s);
      Translation2d tangent = getTangent(s);
      Translation2d diff = point.minus(robotPosition);

      // f(s) = dot(diff, tangent) — when zero, diff is perpendicular to tangent
      double f = diff.getX() * tangent.getX() + diff.getY() * tangent.getY();

      // f'(s) ~ dot(tangent, tangent) + dot(diff, curvature * normal)
      // Simplified: f'(s) ~ 1 (since tangent is unit and we're close)
      // More accurate: use actual derivative
      double curvature = getCurvature(s);
      Translation2d normal = new Translation2d(-tangent.getY(), tangent.getX());
      double fPrime =
          1.0 + curvature * (diff.getX() * normal.getX() + diff.getY() * normal.getY());

      if (Math.abs(fPrime) < 1e-12) {
        break;
      }

      double ds = -f / fPrime;
      s = Math.max(sMin, Math.min(sMax, s + ds));

      if (Math.abs(ds) < 1e-6) {
        break; // Converged
      }
    }
    return s;
  }

  private static double squaredDist(Translation2d a, Translation2d b) {
    double dx = a.getX() - b.getX();
    double dy = a.getY() - b.getY();
    return dx * dx + dy * dy;
  }
}
