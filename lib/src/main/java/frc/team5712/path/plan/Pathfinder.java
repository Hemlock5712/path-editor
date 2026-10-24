package frc.team5712.path.plan;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.GeometrySample;
import frc.team5712.path.core.HeadingProfile;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathGeometry;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.PathSpec;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.RotationTarget;
import frc.team5712.path.core.Waypoint;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Plans a collision-free path across the field during a match.
 *
 * <p>At boot ({@link #Pathfinder}): every obstacle is grown by the bumpers' half-diagonal plus a margin
 * (a Minkowski sum with a disc, rounded corners as short arcs, approximated from outside), the field
 * walls are moved in by the same amount, and the visibility graph between the grown corners is built.
 *
 * <p>Per query ({@link #plan}):
 * <ol>
 *   <li>a start or goal inside a grown obstacle (or too near a wall) is snapped out to the nearest free
 *       point, and the result says so;
 *   <li>the start and goal are connected to the graph and A* finds the shortest route (exact for the
 *       grown polygons);
 *   <li>the route's corners become waypoints of the core quintic, curvature-continuous spline. The
 *       oriented bumper rectangle is swept along it (with the planned heading) and must keep the margin
 *       from every obstacle and wall; where it doesn't, the nearest point of the route is added as a
 *       waypoint and the spline refit, at most {@code maxRefits} times;
 *   <li>the velocity profile is built from the robot's current speed (along its direction of travel
 *       when moving) to rest, with the heading blended from the current heading to the goal's.
 * </ol>
 * The result is an ordinary {@link Path} for the follower. Deterministic: same inputs, same path.
 */
public final class Pathfinder {
  private final FieldMap field;
  private final RobotConfig robot;
  private final PlannerConfig config;
  private final List<ConvexPolygon> grown;
  /** The robot centre must stay within these (walls moved in by the grow distance). */
  private final double xMin;
  private final double xMax;
  private final double yMin;
  private final double yMax;
  private final double[] nodeX;
  private final double[] nodeY;
  /** Straight-line length between visible node pairs, +∞ where blocked. */
  private final double[][] edge;
  private final double bootMicros;

  public Pathfinder(FieldMap field, RobotConfig robot, PlannerConfig config) {
    long t0 = System.nanoTime();
    this.field = field;
    this.robot = robot;
    this.config = config;
    double r = config.growMeters();
    List<ConvexPolygon> g = new ArrayList<>();
    for (ConvexPolygon p : field.obstacles()) {
      g.add(p.grow(r, config.arcStepRadians()));
    }
    grown = List.copyOf(g);
    xMin = -field.lengthMeters() / 2 + r;
    xMax = field.lengthMeters() / 2 - r;
    yMin = -field.widthMeters() / 2 + r;
    yMax = field.widthMeters() / 2 - r;
    List<double[]> nodes = new ArrayList<>();
    for (ConvexPolygon p : grown) {
      for (int i = 0; i < p.size(); i++) {
        if (isFree(p.x(i), p.y(i))) {
          nodes.add(new double[] {p.x(i), p.y(i)});
        }
      }
    }
    int n = nodes.size();
    nodeX = new double[n];
    nodeY = new double[n];
    for (int i = 0; i < n; i++) {
      nodeX[i] = nodes.get(i)[0];
      nodeY[i] = nodes.get(i)[1];
    }
    edge = new double[n][n];
    for (double[] row : edge) {
      Arrays.fill(row, Double.POSITIVE_INFINITY);
    }
    for (int i = 0; i < n; i++) {
      edge[i][i] = 0;
      for (int j = i + 1; j < n; j++) {
        if (visible(nodeX[i], nodeY[i], nodeX[j], nodeY[j])) {
          edge[i][j] = edge[j][i] = Math.hypot(nodeX[j] - nodeX[i], nodeY[j] - nodeY[i]);
        }
      }
    }
    bootMicros = (System.nanoTime() - t0) / 1e3;
  }

  public FieldMap field() {
    return field;
  }

  public PlannerConfig config() {
    return config;
  }

  /** The grown obstacles (what the robot centre must stay out of). */
  public List<ConvexPolygon> grownObstacles() {
    return grown;
  }

  public int nodeCount() {
    return nodeX.length;
  }

  /** Time the constructor took (grow + graph), microseconds. */
  public double bootMicros() {
    return bootMicros;
  }

  /**
   * Plans {@code queries} times between free points spread over the field and throws the results away, so
   * the JIT has compiled the planner before the first real query (a cold first plan takes tens of
   * milliseconds; a warm one a few). Call it once at boot, after the constructor. Returns the time it took,
   * milliseconds.
   */
  public double warmUp(int queries) {
    long t0 = System.nanoTime();
    double hx = xMax - 0.1;
    double hy = yMax - 0.1;
    FieldSpeeds still = new FieldSpeeds(0, 0, 0);
    for (int i = 0; i < queries; i++) {
      // a fixed spread of points (no randomness at boot): across the field and back along the diagonals
      double u = (i + 0.5) / queries;
      Pose a = new Pose(-hx + 2 * hx * u, -hy + 2 * hy * ((i * 7 % queries) + 0.5) / queries, 0.0);
      Pose b = new Pose(hx - 2 * hx * u, hy - 2 * hy * ((i * 3 % queries) + 0.5) / queries, Math.PI / 2);
      plan(a, still, b);
    }
    return (System.nanoTime() - t0) / 1e6;
  }

  /** True when a robot centre at (x, y) is outside every grown obstacle and inside the moved-in walls. */
  public boolean isFree(double x, double y) {
    if (x < xMin - ConvexPolygon.EPS || x > xMax + ConvexPolygon.EPS
        || y < yMin - ConvexPolygon.EPS || y > yMax + ConvexPolygon.EPS) {
      return false;
    }
    for (ConvexPolygon p : grown) {
      if (p.containsStrictly(x, y)) {
        return false;
      }
    }
    return true;
  }

  /** True when the straight segment crosses no grown obstacle (the walls are convex, so ends suffice). */
  public boolean visible(double x0, double y0, double x1, double y1) {
    for (ConvexPolygon p : grown) {
      if (p.blocksSegment(x0, y0, x1, y1)) {
        return false;
      }
    }
    return true;
  }

  // ------------------------------------------------------------------------------ query

  /** A straight approach this long (plus the braking distance from the end speed) ends a route with a set direction. */
  public static final double APPROACH_METERS = 0.5;

  /**
   * The length of the shortest corner-to-corner route from start to goal (m), +∞ when there is none: the search
   * alone, a tenth of a millisecond, without the smoothing and the profile {@link #plan} adds.
   */
  public double routeLengthMeters(Pose start, Pose goal) {
    double[] s = snap(start.xMeters(), start.yMeters());
    double[] g = snap(goal.xMeters(), goal.yMeters());
    List<double[]> route = shortestRoute(s, g);
    if (route == null) {
      return Double.POSITIVE_INFINITY;
    }
    double len = Math.hypot(s[0] - start.xMeters(), s[1] - start.yMeters())
        + Math.hypot(g[0] - goal.xMeters(), g[1] - goal.yMeters());
    for (int i = 1; i < route.size(); i++) {
      len += Math.hypot(route.get(i)[0] - route.get(i - 1)[0], route.get(i)[1] - route.get(i - 1)[1]);
    }
    return len;
  }

  /** Plans from the robot's pose and field velocity to the goal pose (arriving at rest). */
  public PlanResult plan(Pose start, FieldSpeeds startVelocity, Pose goal) {
    return plan(start, startVelocity, goal, 0.0, Double.NaN);
  }

  /**
   * Plans from the robot's pose and field velocity to the goal pose, arriving at {@code endSpeed} m/s while
   * travelling in direction {@code endTangent} (radians; NaN = whatever the route gives), e.g. the start of
   * a path to hand over to. With a direction, the route ends with a straight approach along it
   * ({@link #APPROACH_METERS} plus the braking distance from the end speed) when that stretch is free and the
   * goal is far enough away; otherwise the direction is dropped.
   */
  public PlanResult plan(Pose start, FieldSpeeds startVelocity, Pose goal, double endSpeed, double endTangent) {
    long t0 = System.nanoTime();
    // 1. Snap: a start or goal inside a grown obstacle (or too near a wall) moves out to the nearest free point
    double[] s = snap(start.xMeters(), start.yMeters());
    double[] g = snap(goal.xMeters(), goal.yMeters());
    double startSnap = Math.hypot(s[0] - start.xMeters(), s[1] - start.yMeters());
    double goalSnap = Math.hypot(g[0] - goal.xMeters(), g[1] - goal.yMeters());
    double arrive = Math.min(Math.max(0.0, endSpeed), config.maxMetersPerSecond());

    // 2. Route: the shortest way round the grown obstacles, corner to corner (A* on the visibility graph)
    Route route = route(s, g, arrive, endTangent, goalSnap);
    long t1 = System.nanoTime();
    double straight = Math.hypot(goal.xMeters() - start.xMeters(), goal.yMeters() - start.yMeters());
    if (route == null) {
      return PlanResult.failed("no route between the start and the goal", startSnap, goalSnap, straight,
          (t1 - t0) / 1e3);
    }

    // 3. Waypoints: the route's corners, from where the robot really is to where the goal really is
    List<double[]> pts = waypoints(start, goal, route.points(), startSnap, goalSnap);
    double routeLength = polylineLength(pts);

    // 4. Smooth: a spline through the waypoints, the bumpers swept along it, more waypoints where it comes too close
    double v = Math.hypot(startVelocity.vxMetersPerSecond(), startVelocity.vyMetersPerSecond());
    boolean moving = v > 0.1;
    double startTangent = moving ? StrictMath.atan2(startVelocity.vyMetersPerSecond(), startVelocity.vxMetersPerSecond())
        : Double.NaN;
    double startSpeed = moving ? Math.min(v, config.maxMetersPerSecond()) : 0.0;
    Smoothed smoothed = smooth(pts, startTangent, startSpeed, start.headingRadians(), goal.headingRadians(),
        route.endDirection(), arrive, startSnap, goalSnap);
    long t2 = System.nanoTime();

    // 5. Profile: the same speed plan as a drawn path
    Path path = Path.build(smoothed.spec(), robot);
    long t3 = System.nanoTime();
    return new PlanResult(path, List.copyOf(pts), routeLength, straight, startSnap, goalSnap, smoothed.refits(),
        smoothed.sweep().violations.isEmpty(), smoothed.sweep().minClearance, (t1 - t0) / 1e3, (t2 - t1) / 1e3,
        (t3 - t2) / 1e3, (t3 - t0) / 1e3, null);
  }

  /** A route's corners, and the direction it must arrive in (NaN = whatever the route gives). */
  private record Route(List<double[]> points, double endDirection) {}

  /**
   * Step 2: the shortest route from s to g, or null if there is none. With an end direction (a hand-over to a path),
   * the route ends with a straight approach along it when that stretch is free and the goal is far enough away.
   */
  private Route route(double[] s, double[] g, double arrive, double endTangent, double goalSnap) {
    if (Double.isFinite(endTangent) && goalSnap <= 1e-9) {
      double len = APPROACH_METERS + arrive * arrive / (2 * config.maxMetersPerSecondSquared());
      double[] a = {g[0] - len * StrictMath.cos(endTangent), g[1] - len * StrictMath.sin(endTangent)};
      if (Math.hypot(g[0] - s[0], g[1] - s[1]) > 2 * len && isFree(a[0], a[1]) && visible(a[0], a[1], g[0], g[1])) {
        List<double[]> head = shortestRoute(s, a);
        if (head != null) {
          List<double[]> points = new ArrayList<>(head);
          points.add(g);
          return new Route(points, endTangent);
        }
      }
    }
    List<double[]> points = shortestRoute(s, g);
    return points == null ? null : new Route(points, Double.NaN);
  }

  /** Step 3: the path starts where the robot is and ends where the goal is, through the snapped points if any. */
  private static List<double[]> waypoints(Pose start, Pose goal, List<double[]> route, double startSnap, double goalSnap) {
    List<double[]> pts = new ArrayList<>();
    if (startSnap > 1e-9) {
      pts.add(new double[] {start.xMeters(), start.yMeters()});
    }
    pts.addAll(route);
    if (goalSnap > 1e-9) {
      pts.add(new double[] {goal.xMeters(), goal.yMeters()});
    }
    return dedupe(pts);
  }

  private static double polylineLength(List<double[]> pts) {
    double length = 0;
    for (int i = 1; i < pts.size(); i++) {
      length += Math.hypot(pts.get(i)[0] - pts.get(i - 1)[0], pts.get(i)[1] - pts.get(i - 1)[1]);
    }
    return length;
  }

  /** The smoothed path's spec, its last sweep and how many refits it took. */
  private record Smoothed(PathSpec spec, Sweep sweep, int refits) {}

  /**
   * Step 4: fit the spline through {@code pts}, sweep the bumpers along it, and where it comes closer than the margin
   * add a waypoint and fit again, up to {@code maxRefits} times. Adds the waypoints to {@code pts}. The snapped ends
   * (already inside the margin) aren't checked.
   */
  private Smoothed smooth(List<double[]> pts, double startTangent, double startSpeed, double startHeading,
      double goalHeading, double endDirection, double arrive, double startSnap, double goalSnap) {
    int refits = 0;
    while (true) {
      PathSpec spec = Path.pureRotation(spec(pts, startTangent, startSpeed, startHeading, goalHeading, endDirection,
          arrive));
      PathGeometry geometry = PathGeometry.build(spec.waypoints());
      HeadingProfile heading = HeadingProfile.build(geometry, spec.rotationTargets(), spec.facePointZones());
      double skipStart = startSnap > 1e-9 ? geometry.waypointS(1) : 0.0;
      double skipEnd = goalSnap > 1e-9 ? geometry.waypointS(pts.size() - 2) : geometry.lengthMeters();
      Sweep sweep = sweep(geometry, heading, skipStart, skipEnd);
      if (sweep.violations.isEmpty() || refits >= config.maxRefits()) {
        return new Smoothed(spec, sweep, refits);
      }
      refits++;
      addWaypointsUnder(sweep.violations, geometry, pts);
    }
  }

  /**
   * One new waypoint per stretch that came too close, on the route segment under its worst point (at least 5 cm from
   * the segment's ends, one per segment per round).
   */
  private static void addWaypointsUnder(List<Double> violations, PathGeometry geometry, List<double[]> pts) {
    List<double[]> inserts = new ArrayList<>();
    for (double sv : violations) {
      int seg = segmentAt(geometry, sv, pts.size());
      double[] a = pts.get(seg);
      double[] b = pts.get(seg + 1);
      double[] p = xy(geometry, sv);
      double ex = b[0] - a[0];
      double ey = b[1] - a[1];
      double len = Math.hypot(ex, ey);
      double t = ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / (len * len);
      double minT = Math.min(0.5, 0.05 / len);
      t = Math.max(minT, Math.min(1 - minT, t));
      inserts.add(new double[] {seg, a[0] + t * ex, a[1] + t * ey});
    }
    inserts.sort((x, y) -> Double.compare(y[0], x[0])); // back to front keeps indices valid
    int lastSeg = -1;
    for (double[] ins : inserts) {
      if ((int) ins[0] == lastSeg) {
        continue;
      }
      lastSeg = (int) ins[0];
      pts.add(lastSeg + 1, new double[] {ins[1], ins[2]});
    }
  }

  private static double[] xy(PathGeometry geometry, double s) {
    GeometrySample p = geometry.sample(s);
    return new double[] {p.xMeters(), p.yMeters()};
  }

  /** Index of the waypoint segment containing arc length s. */
  private static int segmentAt(PathGeometry geometry, double s, int waypoints) {
    for (int i = 1; i < waypoints; i++) {
      if (s <= geometry.waypointS(i)) {
        return i - 1;
      }
    }
    return waypoints - 2;
  }

  private PathSpec spec(List<double[]> pts, double startTangent, double startSpeed, double startHeading,
      double goalHeading, double endTangent, double endSpeed) {
    List<Waypoint> wps = new ArrayList<>(pts.size());
    for (int i = 0; i < pts.size(); i++) {
      double tangent = i == 0 ? startTangent : i == pts.size() - 1 ? endTangent : Double.NaN;
      wps.add(new Waypoint(pts.get(i)[0], pts.get(i)[1], tangent, 1.0));
    }
    return PathSpec.of(wps)
        .withRotationTargets(List.of(RotationTarget.atWaypoint(0, startHeading),
            RotationTarget.atWaypoint(pts.size() - 1, goalHeading)))
        .withLimits(config.maxMetersPerSecond(), config.maxMetersPerSecondSquared())
        .withEndSpeeds(startSpeed, endSpeed);
  }

  private static List<double[]> dedupe(List<double[]> pts) {
    List<double[]> out = new ArrayList<>();
    for (double[] p : pts) {
      if (out.isEmpty() || Math.hypot(p[0] - out.getLast()[0], p[1] - out.getLast()[1]) > 0.01) {
        out.add(p);
      }
    }
    if (out.size() == 1) {
      out.add(pts.getLast()); // start and goal within 1 cm: keep both so there is a path
    }
    return out;
  }

  // ------------------------------------------------------------------------------ snapping and A*

  /** The point itself if free, else the nearest free point on a grown obstacle's boundary or the walls. */
  double[] snap(double x, double y) {
    if (isFree(x, y)) {
      return new double[] {x, y};
    }
    List<double[]> candidates = new ArrayList<>();
    candidates.add(new double[] {clamp(x, xMin, xMax), clamp(y, yMin, yMax)});
    for (ConvexPolygon p : grown) {
      double[] b = p.nearestBoundaryPoint(x, y);
      // push 1 mm outwards from the polygon's centre so the point is clearly free
      double dx = b[0] - p.cx;
      double dy = b[1] - p.cy;
      double d = Math.hypot(dx, dy);
      candidates.add(new double[] {b[0] + 1e-3 * dx / d, b[1] + 1e-3 * dy / d});
    }
    candidates.sort((a, b) -> Double.compare(Math.hypot(a[0] - x, a[1] - y), Math.hypot(b[0] - x, b[1] - y)));
    for (double[] c : candidates) {
      if (isFree(c[0], c[1])) {
        return c;
      }
    }
    // overlapping obstacles cover every nearest point: search outwards on rings
    for (double r = 0.05; r < 8.0; r += 0.05) {
      for (int k = 0; k < 72; k++) {
        double a = k * Math.PI / 36;
        double cx = x + r * StrictMath.cos(a);
        double cy = y + r * StrictMath.sin(a);
        if (isFree(cx, cy)) {
          return new double[] {cx, cy};
        }
      }
    }
    throw new IllegalStateException("no free point on the field");
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  /** A* over the static graph plus the start and goal; the route's points, or null if there is none. */
  private List<double[]> shortestRoute(double[] s, double[] g) {
    if (visible(s[0], s[1], g[0], g[1])) {
      return List.of(s, g);
    }
    int n = nodeX.length;
    int start = n;
    int goal = n + 1;
    double[] toStart = new double[n];
    double[] toGoal = new double[n];
    for (int i = 0; i < n; i++) {
      toStart[i] = visible(s[0], s[1], nodeX[i], nodeY[i]) ? Math.hypot(nodeX[i] - s[0], nodeY[i] - s[1])
          : Double.POSITIVE_INFINITY;
      toGoal[i] = visible(nodeX[i], nodeY[i], g[0], g[1]) ? Math.hypot(g[0] - nodeX[i], g[1] - nodeY[i])
          : Double.POSITIVE_INFINITY;
    }
    double[] cost = new double[n + 2];
    int[] parent = new int[n + 2];
    boolean[] closed = new boolean[n + 2];
    Arrays.fill(cost, Double.POSITIVE_INFINITY);
    Arrays.fill(parent, -1);
    cost[start] = 0;
    while (true) {
      int best = -1;
      double bestF = Double.POSITIVE_INFINITY;
      for (int i = 0; i < n + 2; i++) {
        if (!closed[i] && cost[i] < Double.POSITIVE_INFINITY) {
          double f = cost[i] + heuristic(i, n, s, g);
          if (f < bestF) { // ties keep the lowest index: deterministic
            bestF = f;
            best = i;
          }
        }
      }
      if (best < 0) {
        return null;
      }
      if (best == goal) {
        break;
      }
      closed[best] = true;
      for (int j = 0; j < n + 2; j++) {
        if (closed[j] || j == best) {
          continue;
        }
        double w = weight(best, j, n, toStart, toGoal);
        if (w < Double.POSITIVE_INFINITY && cost[best] + w < cost[j]) {
          cost[j] = cost[best] + w;
          parent[j] = best;
        }
      }
    }
    List<double[]> out = new ArrayList<>();
    for (int i = goal; i >= 0; i = parent[i]) {
      out.add(i == start ? s : i == goal ? g : new double[] {nodeX[i], nodeY[i]});
      if (i == start) {
        break;
      }
    }
    java.util.Collections.reverse(out);
    return out;
  }

  private double heuristic(int i, int n, double[] s, double[] g) {
    double x = i == n ? s[0] : i == n + 1 ? g[0] : nodeX[i];
    double y = i == n ? s[1] : i == n + 1 ? g[1] : nodeY[i];
    return Math.hypot(g[0] - x, g[1] - y);
  }

  private double weight(int a, int b, int n, double[] toStart, double[] toGoal) {
    if (a < n && b < n) {
      return edge[a][b];
    }
    if (a == n || b == n) {
      int other = a == n ? b : a;
      return other < n ? toStart[other] : Double.POSITIVE_INFINITY; // start-goal was handled up front
    }
    int other = a == n + 1 ? b : a;
    return other < n ? toGoal[other] : Double.POSITIVE_INFINITY;
  }

  // ------------------------------------------------------------------------------ swept bumpers

  /** Result of sweeping the bumper rectangle along a path. */
  private record Sweep(double minClearance, List<Double> violations) {}

  /**
   * Sweeps the oriented bumper rectangle along the geometry with the planned heading. Returns the
   * smallest clearance to an obstacle or a wall, and for each stretch closer than the margin the arc
   * length of its worst point. Stretches before {@code skipStart} or after {@code skipEnd} (a snapped
   * start or goal, already inside the margin) are not reported.
   */
  private Sweep sweep(PathGeometry geometry, HeadingProfile heading, double skipStart, double skipEnd) {
    double len = geometry.lengthMeters();
    int n = Math.max(2, (int) Math.ceil(len / config.checkStepMeters()) + 1);
    double need = config.marginMeters();
    double reach = config.halfDiagonalMeters();
    double min = Double.POSITIVE_INFINITY;
    List<Double> worst = new ArrayList<>();
    boolean inBad = false;
    double badMin = 0;
    double badS = 0;
    double[] qx = new double[4];
    double[] qy = new double[4];
    for (int k = 0; k < n; k++) {
      double s = len * k / (n - 1);
      GeometrySample gs = geometry.sample(s);
      double th = heading.sample(gs).radians();
      double x = gs.xMeters();
      double y = gs.yMeters();
      double c = clearance(x, y, th, reach, qx, qy);
      boolean counted = s >= skipStart && s <= skipEnd;
      if (counted) {
        min = Math.min(min, c);
      }
      boolean bad = counted && c < need;
      if (bad) {
        if (!inBad || c < badMin) {
          badMin = c;
          badS = s;
        }
        inBad = true;
      } else if (inBad) {
        worst.add(badS);
        inBad = false;
      }
    }
    if (inBad) {
      worst.add(badS);
    }
    return new Sweep(min, worst);
  }

  /** Clearance of the bumper rectangle at (x, y, th) to every obstacle and the field walls. */
  public double clearance(double x, double y, double th) {
    return clearance(x, y, th, config.halfDiagonalMeters(), new double[4], new double[4]);
  }

  private double clearance(double x, double y, double th, double reach, double[] qx, double[] qy) {
    double c = StrictMath.cos(th);
    double s = StrictMath.sin(th);
    double hl = config.bumperLengthMeters() / 2;
    double hw = config.bumperWidthMeters() / 2;
    double[] px = {hl, -hl, -hl, hl}; // front-left, back-left, back-right, front-right: counter-clockwise
    double[] py = {hw, hw, -hw, -hw};
    double halfL = field.lengthMeters() / 2;
    double halfW = field.widthMeters() / 2;
    double best = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 4; i++) {
      qx[i] = x + c * px[i] - s * py[i];
      qy[i] = y + s * px[i] + c * py[i];
      best = Math.min(best, Math.min(halfL - Math.abs(qx[i]), halfW - Math.abs(qy[i])));
    }
    for (ConvexPolygon p : field.obstacles()) {
      double centre = Math.hypot(x - p.cx, y - p.cy) - p.radius - reach;
      if (centre >= best) {
        continue; // cannot be closer than what we already have
      }
      best = Math.min(best, p.distanceToPolygon(qx, qy));
    }
    return best;
  }
}
