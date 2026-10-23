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
 *       {@code v·|θ'| ≤ ω_max} and spin acceleration cap {@code |a·θ' + v²·θ''| ≤ α_max}, and the start
 *       and end speeds. A turn that needs more time than the translation slows the translation down:
 *       the heading is tied to s, and the plan simply takes longer to cover s. A zone caps every grid point of every
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
 * at every grid point. Where the budget falls steeply with speed (a fast turn over a short hop), the
 * two rounds don't close, and the step bisects inside their bracket instead; that keeps O(N).
 * Every path gets at least {@link #MIN_CELLS} cells, so short hops are resolved too. On the fixture paths a 3 cm grid's total time is within 0.1-0.9 % of a 1 mm
 * grid's (the larger figures come from zones capping every cell they touch). Curvature is
 * continuous, so no smoothing window is needed.
 *
 * <h2>Cusps</h2>
 *
 * Where the path doubles back ({@link PathGeometry#cusps}) the robot must stop: the grid gains a point at
 * each cusp (splitting its cell in two) whose speed cap is zero, so the plan comes to rest exactly there
 * and sets off the other way. A cusp in the first or last cell, or within a cell of the previous one, gets
 * no point of its own (the robot is already near a stop there). A path without cusps keeps the uniform
 * grid, number for number.
 */
public final class VelocityProfile {
  /** Default grid spacing, metres. */
  public static final double DEFAULT_GRID_METERS = 0.03;

  /** Fewest grid cells on any path, so a short hop's turn is resolved. */
  public static final int MIN_CELLS = 16;

  private final double lengthMeters;
  private final double ds;
  private final int cells;
  /** s at each grid point, and each cell's length: uniform unless a cusp added a point. */
  private final double[] nodeS;
  private final double[] cellLength;
  private final boolean uniform;
  private final double[] velocity; // m/s at each grid point, cells + 1
  private final double[] accel; // m/s² in each cell, constant across it
  private final double[] time; // s at each grid point

  private static final java.util.concurrent.atomic.AtomicLong BUILDS =
      new java.util.concurrent.atomic.AtomicLong();

  /** How many profiles this JVM has built (a diagnostic, like {@link PathGeometry#buildCount}). */
  public static long buildCount() {
    return BUILDS.get();
  }

  private VelocityProfile(double lengthMeters, double ds, double[] nodeS, double[] cellLength, boolean uniform,
      double[] velocity, double[] accel, double[] time) {
    BUILDS.incrementAndGet();
    this.lengthMeters = lengthMeters;
    this.ds = ds;
    this.nodeS = nodeS;
    this.cellLength = cellLength;
    this.uniform = uniform;
    this.cells = accel.length;
    this.velocity = velocity;
    this.accel = accel;
    this.time = time;
  }

  /**
   * A profile from a speed table made elsewhere (a Choreo trajectory): speed {@code v[i]} at
   * {@code s[i]} metres, s strictly rising from 0, with constant acceleration between the points
   * like a built profile's cells. The speeds are taken as given: no limit is applied.
   */
  public static VelocityProfile tabulated(double[] s, double[] v) {
    int cells = s.length - 1;
    if (cells < 1 || v.length != s.length || s[0] != 0.0) {
      throw new IllegalArgumentException("a speed table needs two or more points, starting at s = 0");
    }
    double[] cellLength = new double[cells];
    double[] accel = new double[cells];
    double[] time = new double[cells + 1];
    for (int i = 0; i < cells; i++) {
      cellLength[i] = s[i + 1] - s[i];
      if (!(cellLength[i] > 0.0) || !(v[i] + v[i + 1] > 0.0)) {
        throw new IllegalArgumentException("the speed table must rise in s and keep moving, at point " + i);
      }
      accel[i] = (v[i + 1] * v[i + 1] - v[i] * v[i]) / (2.0 * cellLength[i]);
      time[i + 1] = time[i] + 2.0 * cellLength[i] / (v[i] + v[i + 1]);
    }
    return new VelocityProfile(s[cells], s[cells] / cells, s.clone(), cellLength, false, v.clone(), accel, time);
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
    int uniformCells = Math.max(MIN_CELLS, (int) Math.ceil(length / gridMeters));
    double ds = length / uniformCells;

    // The grid: uniform, plus a stopping point at each cusp (nodeS, cellLength; stop marks the points)
    double[] cusps = geometry.cusps();
    List<Double> extra = new java.util.ArrayList<>();
    List<Integer> stopAt = new java.util.ArrayList<>(); // uniform grid points that are cusps themselves
    double lastStop = Double.NEGATIVE_INFINITY;
    for (double c : cusps) {
      if (c < ds || c > length - ds || c - lastStop < ds) {
        continue;
      }
      lastStop = c;
      int i = (int) Math.floor(c / ds);
      if (Math.abs(c - i * ds) < 1e-6) {
        stopAt.add(i);
      } else if (Math.abs(c - (i + 1) * ds) < 1e-6) {
        stopAt.add(i + 1);
      } else {
        extra.add(c);
      }
    }
    int cells = uniformCells + extra.size();
    int nodes = cells + 1;
    double[] nodeS = new double[nodes];
    double[] cellLength = new double[cells];
    boolean[] stop = new boolean[nodes];
    int[] gridIndex = new int[nodes]; // the uniform grid point, or -1 for a cusp's own point
    for (int i = 0, e = 0, n = 0; i <= uniformCells; i++) {
      double si = i == uniformCells ? length : i * ds;
      while (e < extra.size() && extra.get(e) < si) {
        nodeS[n] = extra.get(e);
        gridIndex[n] = -1;
        stop[n] = true;
        n++;
        e++;
      }
      nodeS[n] = si;
      gridIndex[n] = i;
      stop[n] = stopAt.contains(i);
      n++;
    }
    for (int i = 0; i < cells; i++) {
      boolean whole = gridIndex[i] >= 0 && gridIndex[i + 1] == gridIndex[i] + 1;
      cellLength[i] = whole ? ds : nodeS[i + 1] - nodeS[i];
    }

    // Zones, resolved to the grid points they cap: every point of every cell the zone overlaps
    List<ConstraintZone> zones = spec.constraintZones();
    int[] zoneFirst = new int[zones.size()];
    int[] zoneLast = new int[zones.size()];
    for (int j = 0; j < zoneFirst.length; j++) {
      zoneFirst[j] = (int) Math.floor(snapToGrid(geometry.sAtWaypoint(zones.get(j).fromWaypoint()) / ds));
      zoneLast[j] = (int) Math.ceil(snapToGrid(geometry.sAtWaypoint(zones.get(j).toWaypoint()) / ds));
    }

    double grip = robot.tractionMetersPerSecondSquared();
    double lever = robot.spinLeverMeters();
    double moduleRadius = robot.moduleRadiusMeters();
    double wheelCap = robot.wheelSpeedCapMetersPerSecond();
    double spinCap = robot.maxAngularRadPerSec();
    double spinAccelCap = robot.plannedSpinAccelRadPerSecSquared();

    // Per grid point: |κ|, |θ'|, |θ''|, the speed ceiling and the acceleration cap
    Budget budget = new Budget(robot, nodes);
    double[] speedCap = new double[nodes];
    double[] d = new double[8];
    for (int i = 0; i < nodes; i++) {
      double s = nodeS[i];
      geometry.evaluateAt(s, d);
      GeometrySample g = PathGeometry.toSample(s, d);
      HeadingSample h = heading.sample(g);
      double kAbs = Math.abs(g.curvature());
      double hp = Math.abs(h.radiansPerMeter());
      double hpp = Math.abs(h.radiansPerMeterSquared());

      double vCap = spec.maxMetersPerSecond();
      double aCap = spec.maxMetersPerSecondSquared();
      for (int j = 0; j < zoneFirst.length; j++) {
        boolean in = gridIndex[i] >= 0 ? gridIndex[i] >= zoneFirst[j] && gridIndex[i] <= zoneLast[j]
            : s >= zoneFirst[j] * ds && s <= zoneLast[j] * ds;
        if (in) {
          vCap = Math.min(vCap, zones.get(j).maxMetersPerSecond());
          aCap = Math.min(aCap, zones.get(j).maxMetersPerSecondSquared());
        }
      }
      double wheels = wheelSpeedFactor(g, h, moduleRadius);
      vCap = Math.min(vCap, wheelCap / wheels);
      if (hp > 0.0) {
        vCap = Math.min(vCap, spinCap / hp);
      }
      if (hpp > 0.0) {
        vCap = Math.min(vCap, Math.sqrt(spinAccelCap / hpp)); // steady speed: α = v²θ''
      }
      double bend = kAbs + lever * hpp;
      if (bend > 0.0) {
        vCap = Math.min(vCap, Math.sqrt(grip / bend));
      }
      speedCap[i] = stop[i] ? 0.0 : vCap;
      budget.curvature[i] = kAbs;
      budget.spin[i] = hp;
      budget.wheels[i] = wheels;
      budget.spinBend[i] = hpp;
      budget.cap[i] = aCap;
    }

    // Forward pass: as fast as the robot can get there
    double[] forward = new double[nodes];
    forward[0] = Math.min(spec.startMetersPerSecond(), speedCap[0]);
    for (int i = 0; i < cells; i++) {
      forward[i + 1] = step(budget, forward[i], i, i + 1, speedCap[i + 1], cellLength[i], true);
    }

    // Backward pass: slow enough to brake for everything ahead. The final speed is the lower of the two.
    double[] velocity = new double[nodes];
    double next = Math.min(spec.endMetersPerSecond(), speedCap[cells]);
    velocity[cells] = Math.min(forward[cells], next);
    for (int i = cells - 1; i >= 0; i--) {
      next = step(budget, next, i + 1, i, speedCap[i], cellLength[i], false);
      velocity[i] = Math.min(forward[i], next);
    }

    // Constant acceleration per cell, and time
    double[] accel = new double[cells];
    double[] time = new double[nodes];
    for (int i = 0; i < cells; i++) {
      double v0 = velocity[i];
      double v1 = velocity[i + 1];
      accel[i] = (v1 * v1 - v0 * v0) / (2.0 * cellLength[i]);
      if (!(v0 + v1 > 0.0)) {
        throw new IllegalStateException("the profile stalls at s = " + nodeS[i] + " m");
      }
      time[i + 1] = time[i] + 2.0 * cellLength[i] / (v0 + v1);
    }
    return new VelocityProfile(length, ds, nodeS, cellLength, extra.isEmpty(), velocity, accel, time);
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
    // f(v) = min(√(v0² + 2·ds·min(atStart, budget(to, v))), capTo) falls as v rises, so the answer
    // v = f(v) lies in [below, above]. When the rounds haven't closed, bisect: f(m) ≥ m means m is
    // still at or below the answer.
    if (above - below > 1e-9) {
      double lo = below;
      double hi = above;
      for (int k = 0; k < 60 && hi - lo > 1e-12; k++) {
        double mid = 0.5 * (lo + hi);
        double a = Math.min(atStart, budget.accel(to, mid, speedingUp));
        if (Math.min(Math.sqrt(v0 * v0 + 2.0 * a * ds), capTo) >= mid) {
          lo = mid;
        } else {
          hi = mid;
        }
      }
      below = lo;
    }
    return below;
  }

  /** Half of √2: a square base's modules sit at 45° + k·90° from the robot's x axis. */
  private static final double HALF_ROOT2 = Math.sqrt(0.5);

  /**
   * The fastest wheel's speed over the robot's, driving along the path while turning at dθ/ds: each
   * module's velocity is the robot's plus the spin's ω × r, so its speed is
   * {@code v·√(1 + (R·θ')² + 2·R·θ'·sin(β − αᵢ))}, with β the direction of travel in the robot's frame and
   * αᵢ the module's bearing (a square base). Between 1 and 1 + R·|θ'|: the spin adds fully to a wheel only
   * when that wheel's spin direction lines up with the travel, and every quarter turn the wheels sit 45° off.
   */
  static double wheelSpeedFactor(GeometrySample g, HeadingSample h, double moduleRadius) {
    double rt = moduleRadius * h.radiansPerMeter();
    if (rt == 0.0) {
      return 1.0;
    }
    double c = Angles.cos(h.radians());
    double sn = Angles.sin(h.radians());
    double ux = g.tangentX() * c + g.tangentY() * sn; // travel in the robot's frame
    double uy = -g.tangentX() * sn + g.tangentY() * c;
    // sin(β − αᵢ) = u · (−sin αᵢ, cos αᵢ), the module's spin direction, for αᵢ = 45°, 135°, 225°, 315°
    double a = HALF_ROOT2 * (-ux + uy);
    double b = HALF_ROOT2 * (-ux - uy);
    double most = Math.max(Math.max(rt * a, -rt * a), Math.max(rt * b, -rt * b));
    return Math.sqrt(1.0 + rt * rt + 2.0 * most);
  }

  /** The acceleration budget at each grid point. */
  private static final class Budget {
    final double[] curvature;
    final double[] spin;
    final double[] wheels; // wheelSpeedFactor: the fastest wheel over the robot's speed
    final double[] spinBend;
    final double[] cap;
    private final RobotConfig robot;
    private final double grip;
    private final double lever;
    private final double brake;
    private final double spinAccelCap;

    Budget(RobotConfig robot, int nodes) {
      this.robot = robot;
      this.grip = robot.tractionMetersPerSecondSquared();
      this.lever = robot.spinLeverMeters();
      this.brake = robot.brakeAccelMetersPerSecondSquared();
      this.spinAccelCap = robot.plannedSpinAccelRadPerSecSquared();
      curvature = new double[nodes];
      spin = new double[nodes];
      wheels = new double[nodes];
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
          ? robot.driveAccelMetersPerSecondSquared(v * wheels[i])
          : brake;
      a = Math.min(a, (drive - spinPush) / (1.0 + k));
      if (spinAccelCap < Double.POSITIVE_INFINITY) {
        // |α| ≤ |a|·|θ'| + v²·|θ''| ≤ α_max
        double room = spinAccelCap - vSq * spinBend[i];
        a = room <= 0.0 ? 0.0 : (spin[i] > 0.0 ? Math.min(a, room / spin[i]) : a);
      }
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
    d = d < 0.0 ? 0.0 : (d > cellLength[lo] ? cellLength[lo] : d);
    double s = nodeS[lo] + d;
    return s > lengthMeters ? lengthMeters : s;
  }

  private int cellAt(double s) {
    if (uniform) {
      int i = (int) Math.floor(s / ds);
      return i < 0 ? 0 : (i >= cells ? cells - 1 : i);
    }
    int lo = 0;
    int hi = cells - 1;
    while (lo < hi) {
      int mid = (lo + hi + 1) >>> 1;
      if (nodeS[mid] <= s) {
        lo = mid;
      } else {
        hi = mid - 1;
      }
    }
    return lo;
  }

  private double clampedOffset(double s, int i) {
    double d = s - nodeS[i];
    return d < 0.0 ? 0.0 : (d > cellLength[i] ? cellLength[i] : d);
  }
}
