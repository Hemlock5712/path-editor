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
    spec = pureRotation(spec);
    PathGeometry geometry = PathGeometry.build(spec.waypoints(), startCurvature);
    HeadingProfile heading = HeadingProfile.build(geometry, spec.rotationTargets(), spec.facePointZones());
    VelocityProfile profile = VelocityProfile.build(geometry, heading, spec, robot, gridMeters);
    return new Path(spec, geometry, heading, profile);
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
        0.0, 0.0, spec.events());
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
