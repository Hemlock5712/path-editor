package frc.team5712.path.core;

/**
 * A ready-to-follow path: geometry, heading profile and velocity profile, built once (at boot) and
 * then only read. Immutable and safe to share between threads.
 */
public final class Path {
  private final PathSpec spec;
  private final PathGeometry geometry;
  private final HeadingProfile heading;
  private final VelocityProfile profile;

  private Path(PathSpec spec, PathGeometry geometry, HeadingProfile heading, VelocityProfile profile) {
    this.spec = spec;
    this.geometry = geometry;
    this.heading = heading;
    this.profile = profile;
  }

  /** Builds everything with the default profile grid. */
  public static Path build(PathSpec spec, RobotConfig robot) {
    return build(spec, robot, VelocityProfile.DEFAULT_GRID_METERS);
  }

  public static Path build(PathSpec spec, RobotConfig robot, double gridMeters) {
    return build(spec, robot, gridMeters, Double.NaN);
  }

  /**
   * Builds with the curvature at the first waypoint fixed (NaN = free), for tails that must join an
   * existing path smoothly.
   */
  public static Path build(PathSpec spec, RobotConfig robot, double gridMeters, double startCurvature) {
    return build(spec, robot, gridMeters, startCurvature, Double.NaN);
  }

  /** Builds with the curvature at the first and the last waypoint fixed (NaN = free). */
  public static Path build(PathSpec spec, RobotConfig robot, double gridMeters, double startCurvature,
      double endCurvature) {
    spec = pureRotation(withWaypointHeadings(spec));
    PathGeometry geometry = PathGeometry.build(spec.waypoints(), startCurvature, endCurvature);
    HeadingProfile heading = HeadingProfile.build(geometry, spec.rotationTargets(), spec.facePointZones(), spec.headingMode());
    VelocityProfile profile = VelocityProfile.build(geometry, heading, spec, robot, gridMeters);
    return new Path(spec, geometry, heading, profile);
  }

  /**
   * A path that follows a trajectory planned elsewhere (a Choreo {@code .traj}): its samples' positions,
   * headings and speeds, at rest at both ends. The geometry is our quintic spline through the sample
   * positions, each with the direction of travel from the sample's velocity; the heading and the speed
   * are tables along it ({@link HeadingProfile#tabulated}, {@link VelocityProfile#tabulated}), taken as
   * planned. Samples closer than a millimetre to the last one kept are dropped (a spin on the spot has
   * no length to put a heading table on). {@code maxMetersPerSecondSquared} is only for re-plans the
   * follower makes from this path (a moving start, a rejoin); the planned profile is the trajectory's.
   *
   * @param t sample times, s; x, y, heading in metres and radians; vx, vy field velocity, m/s
   */
  public static Path fromTrajectory(double[] t, double[] x, double[] y, double[] heading, double[] vx, double[] vy,
      double maxMetersPerSecondSquared) {
    int n = t.length;
    int[] keep = new int[n];
    int k = 0;
    for (int i = 0; i < n; i++) {
      if (k == 0 || Math.hypot(x[i] - x[keep[k - 1]], y[i] - y[keep[k - 1]]) > 1e-3) {
        keep[k++] = i;
      } else if (i == n - 1) {
        keep[k - 1] = i; // the last sample is the end: it replaces the one too close before it
      }
    }
    if (k < 2) {
      throw new IllegalArgumentException("the trajectory doesn't move");
    }
    // The end pieces are millimetres long (the robot starts and stops there), so the ends are pinned to
    // the circle through their three nearest samples: its curvature, and its tangent where the robot is
    // at rest (the chord to the neighbour is turned by half the arc across it, which on a piece that
    // short bends the curve)
    double startK = k < 3 ? 0.0 : circleCurvature(x, y, keep[0], keep[1], keep[2]);
    double endK = k < 3 ? 0.0 : circleCurvature(x, y, keep[k - 3], keep[k - 2], keep[k - 1]);
    java.util.List<Waypoint> wps = new java.util.ArrayList<>();
    double vmax = 0.0;
    for (int j = 0; j < k; j++) {
      int i = keep[j];
      double v = Math.hypot(vx[i], vy[i]);
      vmax = Math.max(vmax, v);
      double dir;
      if (v > 1e-3) {
        dir = StrictMath.atan2(vy[i], vx[i]);
      } else { // at rest: the chord to the neighbour, turned back to the circle's tangent
        int a = j == 0 ? keep[0] : keep[j - 1];
        int b = j == 0 ? keep[1] : keep[j];
        double chord = Math.hypot(x[b] - x[a], y[b] - y[a]);
        double halfTurn = 0.5 * chord * (j == 0 ? startK : endK);
        dir = StrictMath.atan2(y[b] - y[a], x[b] - x[a]) + (j == 0 ? -halfTurn : halfTurn);
      }
      wps.add(Waypoint.at(x[i], y[i], dir));
    }
    PathGeometry geometry = PathGeometry.build(wps, k < 3 ? Double.NaN : startK, k < 3 ? Double.NaN : endK);
    double[] s = new double[k];
    double[] h = new double[k];
    double[] v = new double[k];
    for (int j = 0; j < k; j++) {
      int i = keep[j];
      s[j] = j == 0 ? 0.0 : geometry.sAtWaypoint(j);
      h[j] = heading[i];
      v[j] = j == 0 || j == k - 1 ? 0.0 : Math.hypot(vx[i], vy[i]);
    }
    s[k - 1] = geometry.lengthMeters();
    PathSpec spec = new PathSpec(wps, java.util.List.of(), java.util.List.of(), java.util.List.of(), vmax,
        maxMetersPerSecondSquared, 0.0, 0.0, java.util.List.of(), HeadingMode.HOLD);
    return new Path(spec, geometry, HeadingProfile.tabulated(geometry, s, h), VelocityProfile.tabulated(s, v));
  }

  /** Signed curvature of the circle through three points (0 when they are in a line), 1/m. */
  private static double circleCurvature(double[] x, double[] y, int a, int b, int c) {
    double abx = x[b] - x[a];
    double aby = y[b] - y[a];
    double bcx = x[c] - x[b];
    double bcy = y[c] - y[b];
    double acx = x[c] - x[a];
    double acy = y[c] - y[a];
    double cross = abx * bcy - aby * bcx;
    double d = Math.hypot(abx, aby) * Math.hypot(bcx, bcy) * Math.hypot(acx, acy);
    return d > 0.0 ? 2.0 * cross / d : 0.0;
  }

  /**
   * The waypoints' headings as rotation targets at those waypoints, ahead of the authored targets, and
   * the waypoints themselves without them: what the heading profile plans with. A spec with no waypoint
   * headings is returned unchanged. (A rotation target at the same place as a waypoint heading makes a
   * jump there; the editor doesn't make one.)
   */
  public static PathSpec withWaypointHeadings(PathSpec spec) {
    java.util.List<RotationTarget> targets = new java.util.ArrayList<>();
    java.util.List<Waypoint> plain = new java.util.ArrayList<>();
    java.util.List<Waypoint> w = spec.waypoints();
    for (int i = 0; i < w.size(); i++) {
      if (w.get(i).hasHeading()) {
        targets.add(RotationTarget.atWaypoint(i, w.get(i).headingRadians()));
      }
      plain.add(w.get(i).withHeading(Double.NaN));
    }
    if (targets.isEmpty()) {
      return spec;
    }
    targets.addAll(spec.rotationTargets());
    return spec.withWaypoints(plain).withRotationTargets(targets);
  }

  /** Length of the segment a pure rotation is planned on, metres. */
  public static final double PURE_ROTATION_METERS = 0.001;

  /**
   * A spec whose two waypoints are at the same place is a turn on the spot. The heading is tied to
   * distance, so it is planned on a {@link #PURE_ROTATION_METERS} segment that ends exactly on the
   * point, pointing along the final heading: the spin limits then set the time, and the robot moves
   * at most 1 mm. Any other spec is returned unchanged.
   */
  public static PathSpec pureRotation(PathSpec spec) {
    java.util.List<Waypoint> w = spec.waypoints();
    if (w.size() != 2 || w.get(0).position().minus(w.get(1).position()).norm() > 1e-9) {
      return spec;
    }
    double end = Double.NaN;
    for (RotationTarget t : spec.rotationTargets()) {
      end = t.headingRadians();
    }
    double dir = Double.isNaN(end) ? 0.0 : end;
    Waypoint p = w.get(1);
    Waypoint a = new Waypoint(p.xMeters() - PURE_ROTATION_METERS * StrictMath.cos(dir),
        p.yMeters() - PURE_ROTATION_METERS * StrictMath.sin(dir), dir, 1.0);
    Waypoint b = new Waypoint(p.xMeters(), p.yMeters(), dir, 1.0);
    return new PathSpec(java.util.List.of(a, b), spec.rotationTargets(), spec.facePointZones(),
        spec.constraintZones(), spec.maxMetersPerSecond(), spec.maxMetersPerSecondSquared(),
        0.0, 0.0, spec.events(), spec.headingMode());
  }

  public PathSpec spec() {
    return spec;
  }

  public PathGeometry geometry() {
    return geometry;
  }

  public HeadingProfile heading() {
    return heading;
  }

  public VelocityProfile profile() {
    return profile;
  }

  public double lengthMeters() {
    return geometry.lengthMeters();
  }

  /** Where the path reverses, metres along it ({@link PathGeometry#cusps}); the profile stops the robot at each. */
  public double[] cusps() {
    return geometry.cusps();
  }

  public double totalTimeSeconds() {
    return profile.totalTimeSeconds();
  }

  /** The whole plan at {@code s} (clamped to the path). */
  public PathSample sample(double s) {
    GeometrySample g = geometry.sample(s);
    HeadingSample h = heading.sample(g);
    double sc = g.s();
    return new PathSample(
        sc,
        g.xMeters(),
        g.yMeters(),
        g.tangentX(),
        g.tangentY(),
        g.curvature(),
        g.dCurvatureDs(),
        h.radians(),
        h.radiansPerMeter(),
        profile.velocityAt(sc),
        profile.accelerationAt(sc),
        profile.timeAt(sc));
  }

  /** The plan at time {@code t} seconds after the start. */
  public PathSample sampleAtTime(double t) {
    return sample(profile.sAtTime(t));
  }
}
