package frc.team5712.path.core;

import java.util.List;

/**
 * Speed along the path as a function of distance: the fastest the robot can drive the path without
 * breaking any limit, from a forward pass (how fast it can get) and a backward pass (how fast it can
 * still stop or slow in time), both O(N) on a uniform grid of about {@link #DEFAULT_GRID_METERS}.
 *
 * <h2>The model</h2>
 *
 * The robot is a rigid body of mass m and yaw inertia I, with its wheels a distance r from the centre.
 * Driving the path at speed v with tangential acceleration a, it needs a sideways acceleration
 * {@code v²κ}, and its planned spin {@code ω = v·θ'} needs an angular acceleration
 * {@code α = a·θ' + v²·θ''} (θ' and θ'' are the heading profile's derivatives along s). Spinning up at α
 * makes each wheel push as if accelerating by {@code c·α} more, with {@code c = I/(m·r)}. Limits:
 *
 * <ul>
 *   <li><b>Grip</b> (the worst wheel, where the spin push lines up with the rest):
 *       {@code √(a² + (v²κ)²) + c·|α| ≤ μg}.
 *   <li><b>Motors</b>, speeding up: the treads' rolling push {@code a + c·|α|} is at most what the
 *       motors give at the fastest wheel's speed {@code v·(1 + r·|θ'|)}: full voltage minus back-EMF,
 *       capped by the stator limit. The sideways {@code v²κ} comes from tyre side-grip, not torque.
 *       Slowing down, back-EMF helps, so only the stator limit binds.
 *   <li><b>Wheel speed</b>: the fastest wheel rolls at {@code v + r·|ω|}, so
 *       {@code v ≤ v_wheel / (1 + r·|θ'|)}. Planned spin takes speed away from translation.
 *   <li><b>Caps</b>: the path's and each zone's speed and acceleration, the robot's spin cap
 *       {@code v·|θ'| ≤ ω_max}, and the start and end speeds. A zone caps every grid point of every
 *       cell it overlaps, so the speed is already down when the zone starts.
 * </ul>
 *
 * <p>{@code |α| ≤ |a|·|θ'| + v²·|θ''|} turns the grip limit into a closed form for the largest |a|:
 * with {@code n = v²|κ|}, {@code k = c|θ'|} and {@code e = μg − c·v²|θ''|},
 * {@code |a| ≤ (e² − n²) / (e·k + √(e² − n² + k²n²))}. The speed at which the robot can hold a steady
 * speed ({@code a = 0}) is the ceiling {@code v ≤ √(μg / (|κ| + c|θ''|))}.
 *
 * <h2>The passes</h2>
 *
 * The acceleration is constant across each grid cell, which gives exact v(s) and t(s) between grid
 * points. A step must keep the limits at both of its ends, each with its own geometry and speed, and
 * the end speed depends on the step's acceleration. The step solves that by a short fixed-point
 * iteration that approaches from above and keeps the last answer from below, so the limits hold
 * at every grid point. On the fixture paths a 3 cm grid's total time is within 0.1-0.9 % of a 1 mm
 * grid's (the larger figures come from zones capping every cell they touch). Curvature is
 * continuous, so no smoothing window is needed.
 */
public final class VelocityProfile {
  /** Default grid spacing, metres. */
  public static final double DEFAULT_GRID_METERS = 0.03;

  private final double lengthMeters;
  private final double ds;
  private final int cells;
  private final double[] velocity; // m/s at each grid point, cells + 1
  private final double[] accel; // m/s² in each cell, constant across it
  private final double[] time; // s at each grid point

  private VelocityProfile(double lengthMeters, double ds, double[] velocity, double[] accel, double[] time) {
    this.lengthMeters = lengthMeters;
    this.ds = ds;
    this.cells = accel.length;
    this.velocity = velocity;
    this.accel = accel;
    this.time = time;
  }

  public static VelocityProfile build(
      PathGeometry geometry, HeadingProfile heading, PathSpec spec, RobotConfig robot) {
    return build(geometry, heading, spec, robot, DEFAULT_GRID_METERS);
  }

  public static VelocityProfile build(
      PathGeometry geometry,
      HeadingProfile heading,
      PathSpec spec,
      RobotConfig robot,
      double gridMeters) {
    double length = geometry.lengthMeters();
    int cells = Math.max(1, (int) Math.ceil(length / gridMeters));
    double ds = length / cells;
    int nodes = cells + 1;

    // Zones, resolved to the grid points they cap: every point of every cell the zone overlaps
    List<ConstraintZone> zones = spec.constraintZones();
    int[] zoneFirst = new int[zones.size()];
    int[] zoneLast = new int[zones.size()];
    for (int j = 0; j < zoneFirst.length; j++) {
      zoneFirst[j] = (int) Math.floor(snapToGrid(geometry.sAtWaypoint(zones.get(j).fromWaypoint()) / ds));
      zoneLast[j] = (int) Math.ceil(snapToGrid(geometry.sAtWaypoint(zones.get(j).toWaypoint()) / ds));
    }

    double grip = robot.gripMetersPerSecondSquared();
    double lever = robot.spinLeverMeters();
    double moduleRadius = robot.moduleRadiusMeters();
    double wheelCap = robot.wheelSpeedCapMetersPerSecond();
    double spinCap = robot.maxAngularRadPerSec();

    // Per grid point: |κ|, |θ'|, |θ''|, the speed ceiling and the acceleration cap
    Budget budget = new Budget(robot, nodes);
    double[] speedCap = new double[nodes];
    double[] d = new double[8];
    for (int i = 0; i < nodes; i++) {
      double s = i == cells ? length : i * ds;
      geometry.evaluateAt(s, d);
      GeometrySample g = PathGeometry.toSample(s, d);
      HeadingSample h = heading.sample(g);
      double kAbs = Math.abs(g.curvature());
      double hp = Math.abs(h.radiansPerMeter());
      double hpp = Math.abs(h.radiansPerMeterSquared());

      double vCap = spec.maxMetersPerSecond();
      double aCap = spec.maxMetersPerSecondSquared();
      for (int j = 0; j < zoneFirst.length; j++) {
        if (i >= zoneFirst[j] && i <= zoneLast[j]) {
          vCap = Math.min(vCap, zones.get(j).maxMetersPerSecond());
          aCap = Math.min(aCap, zones.get(j).maxMetersPerSecondSquared());
        }
      }
      vCap = Math.min(vCap, wheelCap / (1.0 + moduleRadius * hp));
      if (hp > 0.0) {
        vCap = Math.min(vCap, spinCap / hp);
      }
      double bend = kAbs + lever * hpp;
      if (bend > 0.0) {
        vCap = Math.min(vCap, Math.sqrt(grip / bend));
      }
      speedCap[i] = vCap;
      budget.curvature[i] = kAbs;
      budget.spin[i] = hp;
      budget.spinBend[i] = hpp;
      budget.cap[i] = aCap;
    }

    // Forward pass: as fast as the robot can get there
    double[] forward = new double[nodes];
    forward[0] = Math.min(spec.startMetersPerSecond(), speedCap[0]);
    for (int i = 0; i < cells; i++) {
      forward[i + 1] = step(budget, forward[i], i, i + 1, speedCap[i + 1], ds, true);
    }

    // Backward pass: slow enough to brake for everything ahead. The final speed is the lower of the two.
    double[] velocity = new double[nodes];
    double next = Math.min(spec.endMetersPerSecond(), speedCap[cells]);
    velocity[cells] = Math.min(forward[cells], next);
    for (int i = cells - 1; i >= 0; i--) {
      next = step(budget, next, i + 1, i, speedCap[i], ds, false);
      velocity[i] = Math.min(forward[i], next);
    }

    // Constant acceleration per cell, and time
    double[] accel = new double[cells];
    double[] time = new double[nodes];
    for (int i = 0; i < cells; i++) {
      double v0 = velocity[i];
      double v1 = velocity[i + 1];
      accel[i] = (v1 * v1 - v0 * v0) / (2.0 * ds);
      if (!(v0 + v1 > 0.0)) {
        throw new IllegalStateException("the profile stalls at s = " + (i * ds) + " m");
      }
      time[i + 1] = time[i] + 2.0 * ds / (v0 + v1);
    }
    return new VelocityProfile(length, ds, velocity, accel, time);
  }

  /**
   * A position in grid cells, snapped to the nearest grid point when within a millionth of a cell,
   * so a zone that starts exactly on a grid point (common on symmetric paths) doesn't flip to the
   * cell before it on rounding.
   */
  private static double snapToGrid(double cells) {
    double nearest = Math.rint(cells);
    return Math.abs(cells - nearest) < 1e-6 ? nearest : cells;
  }

  /**
   * One step of a pass, from grid point {@code from} at speed {@code v0} to grid point {@code to}:
   * the fastest speed at {@code to} such that the step's constant |a| fits the budget at both ends.
   * {@code f(v) = √(v0² + 2·ds·min(budget(from, v0), budget(to, v)))} falls as v rises, so starting
   * above the answer and applying f alternates around it; two rounds settle it, and the answer
   * from below is kept.
   */
  private static double step(
      Budget budget, double v0, int from, int to, double capTo, double ds, boolean speedingUp) {
    double atStart = Math.min(budget.accel(from, v0, speedingUp), budget.cellCap(from, to));
    double above = Math.min(Math.sqrt(v0 * v0 + 2.0 * atStart * ds), capTo);
    double below = 0.0;
    for (int round = 0; round < 2; round++) {
      double a = Math.min(atStart, budget.accel(to, above, speedingUp));
      below = Math.min(Math.sqrt(v0 * v0 + 2.0 * a * ds), capTo);
      a = Math.min(atStart, budget.accel(to, below, speedingUp));
      above = Math.min(Math.sqrt(v0 * v0 + 2.0 * a * ds), capTo);
    }
    return below;
  }

  /** The acceleration budget at each grid point. */
  private static final class Budget {
    final double[] curvature;
    final double[] spin;
    final double[] spinBend;
    final double[] cap;
    private final RobotConfig robot;
    private final double grip;
    private final double lever;
    private final double moduleRadius;
    private final double brake;

    Budget(RobotConfig robot, int nodes) {
      this.robot = robot;
      this.grip = robot.gripMetersPerSecondSquared();
      this.lever = robot.spinLeverMeters();
      this.moduleRadius = robot.moduleRadiusMeters();
      this.brake = robot.brakeAccelMetersPerSecondSquared();
      curvature = new double[nodes];
      spin = new double[nodes];
      spinBend = new double[nodes];
      cap = new double[nodes];
    }

    /** The acceleration cap for the cell between two neighbouring grid points. */
    double cellCap(int i, int j) {
      return Math.min(cap[i], cap[j]);
    }

    /** Largest |a| at grid point i and speed v, speeding up or slowing down. Never negative. */
    double accel(int i, double v, boolean speedingUp) {
      double vSq = v * v;
      double n = vSq * curvature[i];
      double k = lever * spin[i];
      double spinPush = lever * vSq * spinBend[i];
      double e = grip - spinPush;
      double a = 0.0;
      if (e > n) {
        double room = e * e - n * n;
        a = room / (e * k + Math.sqrt(room + k * k * n * n));
      }
      double drive = speedingUp
          ? robot.driveAccelMetersPerSecondSquared(v * (1.0 + moduleRadius * spin[i]))
          : brake;
      a = Math.min(a, (drive - spinPush) / (1.0 + k));
      return a > 0.0 ? a : 0.0;
    }
  }

  // ------------------------------------------------------------------------------ queries

  public double lengthMeters() {
    return lengthMeters;
  }

  /** Time to drive the whole path, seconds. */
  public double totalTimeSeconds() {
    return time[cells];
  }

  /** Grid spacing actually used, metres (the path length split into equal cells). */
  public double gridMeters() {
    return ds;
  }

  /** Number of grid points (cells + 1). */
  public int gridPointCount() {
    return cells + 1;
  }

  /** Planned speed at grid point {@code i}. */
  public double gridVelocity(int i) {
    return velocity[i];
  }

  /** Planned speed along the path at {@code s}, m/s. */
  public double velocityAt(double s) {
    int i = cellAt(s);
    double d = clampedOffset(s, i);
    double vSq = velocity[i] * velocity[i] + 2.0 * accel[i] * d;
    return vSq > 0.0 ? Math.sqrt(vSq) : 0.0;
  }

  /** Planned acceleration along the path at {@code s}, m/s² (constant across each grid cell). */
  public double accelerationAt(double s) {
    return accel[cellAt(s)];
  }

  /** Planned time to reach {@code s}, seconds. */
  public double timeAt(double s) {
    int i = cellAt(s);
    double d = clampedOffset(s, i);
    if (d <= 0.0) {
      return time[i];
    }
    return time[i] + 2.0 * d / (velocity[i] + velocityAt(s));
  }

  /** Where the plan is at time {@code t}, metres along the path (clamped to the path). */
  public double sAtTime(double t) {
    if (!(t > 0.0)) {
      return 0.0;
    }
    if (t >= time[cells]) {
      return lengthMeters;
    }
    int lo = 0;
    int hi = cells - 1;
    while (lo < hi) {
      int mid = (lo + hi + 1) >>> 1;
      if (time[mid] <= t) {
        lo = mid;
      } else {
        hi = mid - 1;
      }
    }
    double tau = t - time[lo];
    double d = velocity[lo] * tau + 0.5 * accel[lo] * tau * tau;
    d = d < 0.0 ? 0.0 : (d > ds ? ds : d);
    double s = lo * ds + d;
    return s > lengthMeters ? lengthMeters : s;
  }

  private int cellAt(double s) {
    int i = (int) Math.floor(s / ds);
    return i < 0 ? 0 : (i >= cells ? cells - 1 : i);
  }

  private double clampedOffset(double s, int i) {
    double d = s - i * ds;
    return d < 0.0 ? 0.0 : (d > ds ? ds : d);
  }
}
