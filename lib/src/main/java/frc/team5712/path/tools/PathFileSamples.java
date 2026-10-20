package frc.team5712.path.tools;

import frc.team5712.path.core.ConstraintZone;
import frc.team5712.path.core.FacePointZone;
import frc.team5712.path.core.PathEvent;
import frc.team5712.path.core.PathSpec;
import frc.team5712.path.core.RotationTarget;
import frc.team5712.path.core.Vec2;
import frc.team5712.path.core.Waypoint;
import java.util.ArrayList;
import java.util.List;

/**
 * Pseudo-random path specs, every part of the file format exercised (events with awkward names,
 * optional keys on and off, numbers that need rounding). The generator is a 32-bit LCG with integer
 * arithmetic only, ported line for line to {@code editor/core/test/pathFileSamples.ts}, so both
 * languages make the same specs. {@code fixtures/files/path_files.json} holds the Java writer's text
 * for each; the TS writer must print the same characters, and each reader must read it back.
 */
public final class PathFileSamples {
  /** How many files the fixture holds. */
  public static final int COUNT = 40;

  public static final int SEED = 5712;

  private static final String[] NAMES = {
    "intakeDown", "shoot", "score \"L4\"", "align tag 18", "back\\slash", "tab\there", "ünïcödé", "x"
  };

  private int state;

  private PathFileSamples(int seed) {
    this.state = seed;
  }

  /** {@link #COUNT} specs from {@link #SEED}. */
  public static List<PathSpec> all() {
    PathFileSamples g = new PathFileSamples(SEED);
    List<PathSpec> specs = new ArrayList<>();
    for (int i = 0; i < COUNT; i++) {
      specs.add(g.spec());
    }
    return specs;
  }

  /** Uniform on [0, 1), 24 bits. */
  private double next() {
    state = state * 1664525 + 1013904223;
    return (state >>> 8) / 16777216.0;
  }

  private int below(int n) {
    return (int) Math.floor(next() * n);
  }

  private PathSpec spec() {
    int n = 2 + below(5);
    double last = n - 1;
    List<Waypoint> waypoints = new ArrayList<>();
    for (int i = 0; i < n; i++) {
      double x = -8.0 + 16.0 * next();
      double y = -4.0 + 8.0 * next();
      double tangent = next() < 0.4 ? (next() * 2.0 - 1.0) * Math.PI : Double.NaN;
      double scale = next() < 0.3 ? 0.5 + next() : 1.0;
      waypoints.add(new Waypoint(x, y, tangent, scale));
    }

    List<RotationTarget> targets = new ArrayList<>();
    for (int k = below(3); k > 0; k--) {
      boolean byWaypoint = next() < 0.5;
      double where = byWaypoint ? next() * last : next() * 10.0;
      double heading = (next() * 2.0 - 1.0) * Math.PI;
      targets.add(byWaypoint ? RotationTarget.atWaypoint(where, heading) : RotationTarget.atMeters(where, heading));
    }

    List<FacePointZone> faces = new ArrayList<>();
    for (int k = below(2); k > 0; k--) {
      double from = next() * last * 0.5;
      double to = from + 0.1 + next() * (last - from - 0.1);
      faces.add(new FacePointZone(from, to, new Vec2(-8.0 + 16.0 * next(), -4.0 + 8.0 * next()), next() * 2.0));
    }

    List<ConstraintZone> zones = new ArrayList<>();
    for (int k = below(2); k > 0; k--) {
      double from = next() * last * 0.5;
      double to = from + 0.1 + next() * (last - from - 0.1);
      double maxSpeed = next() < 0.3 ? Double.POSITIVE_INFINITY : 0.5 + next() * 3.0;
      double maxAccel = next() < 0.5 ? Double.POSITIVE_INFINITY : 1.0 + next() * 5.0;
      zones.add(new ConstraintZone(from, to, maxSpeed, maxAccel));
    }

    List<PathEvent> events = new ArrayList<>();
    for (int k = below(4); k > 0; k--) {
      String name = NAMES[below(NAMES.length)];
      events.add(next() < 0.5 ? PathEvent.atWaypoint(name, next() * last) : PathEvent.atMeters(name, next() * 12.0));
    }

    double maxSpeed = next() < 0.5 ? Double.POSITIVE_INFINITY : 1.0 + next() * 4.0;
    double maxAccel = next() < 0.5 ? Double.POSITIVE_INFINITY : 1.0 + next() * 4.0;
    double startSpeed = next() < 0.3 ? next() * 2.0 : 0.0;
    double endSpeed = next() < 0.3 ? next() * 2.0 : 0.0;
    return new PathSpec(waypoints, targets, faces, zones, maxSpeed, maxAccel, startSpeed, endSpeed, events);
  }
}
