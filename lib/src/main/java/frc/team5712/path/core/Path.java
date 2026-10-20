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
    PathGeometry geometry = PathGeometry.build(spec.waypoints(), startCurvature);
    HeadingProfile heading = HeadingProfile.build(geometry, spec.rotationTargets(), spec.facePointZones());
    VelocityProfile profile = VelocityProfile.build(geometry, heading, spec, robot, gridMeters);
    return new Path(spec, geometry, heading, profile);
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
