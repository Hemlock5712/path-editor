// Speed along the path over s: forward and backward passes on a uniform grid. Port of lib's
// VelocityProfile.java; the model (grip with the worst wheel's spin push, motor curve, wheel-speed
// cap, zone and spin caps) is documented there.

import { toSample, type PathGeometry } from "./geometry.ts";
import type { HeadingProfile } from "./heading.ts";
import {
  brakeAccelMetersPerSecondSquared,
  driveAccelMetersPerSecondSquared,
  gripMetersPerSecondSquared,
  spinLeverMeters,
  wheelSpeedCapMetersPerSecond,
  type RobotConfig,
} from "./robot.ts";
import type { PathSpec } from "./spec.ts";

/** Default grid spacing, metres. */
export const DEFAULT_GRID_METERS = 0.03;

/** Fewest grid cells on any path, so a short hop's turn is resolved (as in Java). */
export const MIN_CELLS = 16;

export class VelocityProfile {
  readonly lengthMeters: number;
  /** Grid spacing actually used, metres (the path length split into equal cells). */
  readonly gridMeters: number;
  private readonly cells: number;
  private readonly velocity: Float64Array; // m/s at each grid point, cells + 1
  private readonly accel: Float64Array; // m/s² in each cell, constant across it
  private readonly time: Float64Array; // s at each grid point

  private constructor(
    lengthMeters: number,
    gridMeters: number,
    velocity: Float64Array,
    accel: Float64Array,
    time: Float64Array,
  ) {
    this.lengthMeters = lengthMeters;
    this.gridMeters = gridMeters;
    this.cells = accel.length;
    this.velocity = velocity;
    this.accel = accel;
    this.time = time;
  }

  static build(
    geometry: PathGeometry,
    heading: HeadingProfile,
    spec: PathSpec,
    robot: RobotConfig,
    gridMeters = DEFAULT_GRID_METERS,
  ): VelocityProfile {
    const length = geometry.lengthMeters;
    const cells = Math.max(MIN_CELLS, Math.ceil(length / gridMeters));
    const ds = length / cells;
    const nodes = cells + 1;

    // Zones, resolved to the grid points they cap
    const zones = spec.constraintZones;
    const zoneFirst = new Int32Array(zones.length);
    const zoneLast = new Int32Array(zones.length);
    for (let j = 0; j < zones.length; j++) {
      zoneFirst[j] = Math.floor(snapToGrid(geometry.sAtWaypoint(zones[j]!.fromWaypoint) / ds));
      zoneLast[j] = Math.ceil(snapToGrid(geometry.sAtWaypoint(zones[j]!.toWaypoint) / ds));
    }

    const grip = gripMetersPerSecondSquared(robot);
    const lever = spinLeverMeters(robot);
    const moduleRadius = robot.moduleRadiusMeters;
    const wheelCap = wheelSpeedCapMetersPerSecond(robot);
    const spinCap = robot.maxAngularRadPerSec;
    const spinAccelCap = robot.maxAngularRadPerSecSquared;

    const budget = new Budget(robot, nodes);
    const speedCap = new Float64Array(nodes);
    const d = new Float64Array(8);
    for (let i = 0; i < nodes; i++) {
      const s = i === cells ? length : i * ds;
      geometry.evaluateAt(s, d);
      const g = toSample(s, d);
      const h = heading.sampleAt(g);
      const kAbs = Math.abs(g.curvature);
      const hp = Math.abs(h.radiansPerMeter);
      const hpp = Math.abs(h.radiansPerMeterSquared);

      let vCap = spec.maxMetersPerSecond;
      let aCap = spec.maxMetersPerSecondSquared;
      for (let j = 0; j < zones.length; j++) {
        if (i >= zoneFirst[j]! && i <= zoneLast[j]!) {
          vCap = Math.min(vCap, zones[j]!.maxMetersPerSecond);
          aCap = Math.min(aCap, zones[j]!.maxMetersPerSecondSquared);
        }
      }
      vCap = Math.min(vCap, wheelCap / (1.0 + moduleRadius * hp));
      if (hp > 0.0) vCap = Math.min(vCap, spinCap / hp);
      if (hpp > 0.0) vCap = Math.min(vCap, Math.sqrt(spinAccelCap / hpp)); // steady speed: α = v²θ''
      const bend = kAbs + lever * hpp;
      if (bend > 0.0) vCap = Math.min(vCap, Math.sqrt(grip / bend));
      speedCap[i] = vCap;
      budget.curvature[i] = kAbs;
      budget.spin[i] = hp;
      budget.spinBend[i] = hpp;
      budget.cap[i] = aCap;
    }

    // Forward pass
    const forward = new Float64Array(nodes);
    forward[0] = Math.min(spec.startMetersPerSecond, speedCap[0]!);
    for (let i = 0; i < cells; i++) {
      forward[i + 1] = step(budget, forward[i]!, i, i + 1, speedCap[i + 1]!, ds, true);
    }

    // Backward pass; the final speed is the lower of the two
    const velocity = new Float64Array(nodes);
    let next = Math.min(spec.endMetersPerSecond, speedCap[cells]!);
    velocity[cells] = Math.min(forward[cells]!, next);
    for (let i = cells - 1; i >= 0; i--) {
      next = step(budget, next, i + 1, i, speedCap[i]!, ds, false);
      velocity[i] = Math.min(forward[i]!, next);
    }

    // Constant acceleration per cell, and time
    const accel = new Float64Array(cells);
    const time = new Float64Array(nodes);
    for (let i = 0; i < cells; i++) {
      const v0 = velocity[i]!;
      const v1 = velocity[i + 1]!;
      accel[i] = (v1 * v1 - v0 * v0) / (2.0 * ds);
      if (!(v0 + v1 > 0.0)) throw new Error(`the profile stalls at s = ${i * ds} m`);
      time[i + 1] = time[i]! + (2.0 * ds) / (v0 + v1);
    }
    return new VelocityProfile(length, ds, velocity, accel, time);
  }

  /** Time to drive the whole path, seconds. */
  get totalTimeSeconds(): number {
    return this.time[this.cells]!;
  }

  /** Number of grid points (cells + 1). */
  get gridPointCount(): number {
    return this.cells + 1;
  }

  /** Planned speed at grid point i. */
  gridVelocity(i: number): number {
    return this.velocity[i]!;
  }

  /** Planned speed along the path at s, m/s. */
  velocityAt(s: number): number {
    const i = this.cellAt(s);
    const d = this.clampedOffset(s, i);
    const vSq = this.velocity[i]! * this.velocity[i]! + 2.0 * this.accel[i]! * d;
    return vSq > 0.0 ? Math.sqrt(vSq) : 0.0;
  }

  /** Planned acceleration at s, m/s² (constant across each grid cell). */
  accelerationAt(s: number): number {
    return this.accel[this.cellAt(s)]!;
  }

  /** Planned time to reach s, seconds. */
  timeAt(s: number): number {
    const i = this.cellAt(s);
    const d = this.clampedOffset(s, i);
    if (d <= 0.0) return this.time[i]!;
    return this.time[i]! + (2.0 * d) / (this.velocity[i]! + this.velocityAt(s));
  }

  /** Where the plan is at time t, metres along the path (clamped). */
  sAtTime(t: number): number {
    if (!(t > 0.0)) return 0.0;
    if (t >= this.time[this.cells]!) return this.lengthMeters;
    let lo = 0;
    let hi = this.cells - 1;
    while (lo < hi) {
      const mid = (lo + hi + 1) >>> 1;
      if (this.time[mid]! <= t) {
        lo = mid;
      } else {
        hi = mid - 1;
      }
    }
    const tau = t - this.time[lo]!;
    let d = this.velocity[lo]! * tau + 0.5 * this.accel[lo]! * tau * tau;
    d = d < 0.0 ? 0.0 : d > this.gridMeters ? this.gridMeters : d;
    const s = lo * this.gridMeters + d;
    return s > this.lengthMeters ? this.lengthMeters : s;
  }

  private cellAt(s: number): number {
    const i = Math.floor(s / this.gridMeters);
    return i < 0 ? 0 : i >= this.cells ? this.cells - 1 : i;
  }

  private clampedOffset(s: number, i: number): number {
    const d = s - i * this.gridMeters;
    return d < 0.0 ? 0.0 : d > this.gridMeters ? this.gridMeters : d;
  }
}

/** Snaps a grid position to the nearest grid point when within a millionth of a cell. */
function snapToGrid(cells: number): number {
  const nearest = Math.round(cells);
  return Math.abs(cells - nearest) < 1e-6 ? nearest : cells;
}

/**
 * One step of a pass: a two-round fixed point so the step's |a| fits the budget at both ends, then
 * bisection inside the rounds' bracket when they haven't closed (a fast turn over a short hop).
 */
function step(
  budget: Budget,
  v0: number,
  from: number,
  to: number,
  capTo: number,
  ds: number,
  speedingUp: boolean,
): number {
  const atStart = Math.min(budget.accel(from, v0, speedingUp), budget.cellCap(from, to));
  let above = Math.min(Math.sqrt(v0 * v0 + 2.0 * atStart * ds), capTo);
  let below = 0.0;
  for (let round = 0; round < 2; round++) {
    let a = Math.min(atStart, budget.accel(to, above, speedingUp));
    below = Math.min(Math.sqrt(v0 * v0 + 2.0 * a * ds), capTo);
    a = Math.min(atStart, budget.accel(to, below, speedingUp));
    above = Math.min(Math.sqrt(v0 * v0 + 2.0 * a * ds), capTo);
  }
  if (above - below > 1e-9) {
    let lo = below;
    let hi = above;
    for (let k = 0; k < 60 && hi - lo > 1e-12; k++) {
      const mid = 0.5 * (lo + hi);
      const a = Math.min(atStart, budget.accel(to, mid, speedingUp));
      if (Math.min(Math.sqrt(v0 * v0 + 2.0 * a * ds), capTo) >= mid) lo = mid;
      else hi = mid;
    }
    below = lo;
  }
  return below;
}

/** The acceleration budget at each grid point. */
class Budget {
  readonly curvature: Float64Array;
  readonly spin: Float64Array;
  readonly spinBend: Float64Array;
  readonly cap: Float64Array;
  private readonly robot: RobotConfig;
  private readonly grip: number;
  private readonly lever: number;
  private readonly moduleRadius: number;
  private readonly brake: number;
  private readonly spinAccelCap: number;

  constructor(robot: RobotConfig, nodes: number) {
    this.robot = robot;
    this.grip = gripMetersPerSecondSquared(robot);
    this.lever = spinLeverMeters(robot);
    this.moduleRadius = robot.moduleRadiusMeters;
    this.brake = brakeAccelMetersPerSecondSquared(robot);
    this.spinAccelCap = robot.maxAngularRadPerSecSquared;
    this.curvature = new Float64Array(nodes);
    this.spin = new Float64Array(nodes);
    this.spinBend = new Float64Array(nodes);
    this.cap = new Float64Array(nodes);
  }

  cellCap(i: number, j: number): number {
    return Math.min(this.cap[i]!, this.cap[j]!);
  }

  /** Largest |a| at grid point i and speed v, speeding up or slowing down. Never negative. */
  accel(i: number, v: number, speedingUp: boolean): number {
    const vSq = v * v;
    const n = vSq * this.curvature[i]!;
    const k = this.lever * this.spin[i]!;
    const spinPush = this.lever * vSq * this.spinBend[i]!;
    const e = this.grip - spinPush;
    let a = 0.0;
    if (e > n) {
      const room = e * e - n * n;
      a = room / (e * k + Math.sqrt(room + k * k * n * n));
    }
    const drive = speedingUp
      ? driveAccelMetersPerSecondSquared(this.robot, v * (1.0 + this.moduleRadius * this.spin[i]!))
      : this.brake;
    a = Math.min(a, (drive - spinPush) / (1.0 + k));
    if (this.spinAccelCap < Infinity) {
      // |α| ≤ |a|·|θ'| + v²·|θ''| ≤ α_max
      const room = this.spinAccelCap - vSq * this.spinBend[i]!;
      a = room <= 0.0 ? 0.0 : this.spin[i]! > 0.0 ? Math.min(a, room / this.spin[i]!) : a;
    }
    return a > 0.0 ? a : 0.0;
  }
}
