// The path's shape indexed by distance along it. Port of lib's PathGeometry.java: Gauss-Legendre
// lengths per sub-piece, a cubic u(s) interpolant per sub-piece refined to LOOKUP_TOLERANCE_METERS,
// queries by s, and windowed / global projection.

import { QuinticSpline } from "./spline.ts";
import type { Waypoint } from "./spec.ts";

/** Worst position error of a lookup by s, measured at each sub-piece's middle while building. */
export const LOOKUP_TOLERANCE_METERS = 1e-5;
const QUADRATURE_TOLERANCE_METERS = 1e-10;
const MAX_INITIAL_PIECE_METERS = 0.5;
const MAX_DEPTH = 24;

// 5-point Gauss-Legendre on [0, 1]: nodes 0.5 ± 0.5ξ, weights w/2
const GL_XI1 = 0.5384693101056830910363144207002088;
const GL_XI2 = 0.9061798459386639927976268782993929;
const GL_W0 = 0.5688888888888888888888888888888889 / 2.0;
const GL_W1 = 0.4786286704993664680412915148356382 / 2.0;
const GL_W2 = 0.2369268850561890875142640407199173 / 2.0;

const WINDOW_STEP_METERS = 0.05;

/** Samples of |p'| per spline piece when looking for cusps (as in Java). */
export const CUSP_SCAN = 64;
/** A cusp turns the tangent more than 120° between this far before and this far after it. */
export const CUSP_ARC_METERS = 0.02;
const CUSP_COS = -0.5;
const GOLDEN = 0.3819660112501051;
const GLOBAL_STEP_METERS = 0.1;

/** The path at one distance along it. Curvature is signed (positive = turning left). */
export interface GeometrySample {
  readonly s: number;
  readonly xMeters: number;
  readonly yMeters: number;
  readonly tangentX: number;
  readonly tangentY: number;
  readonly curvature: number;
  readonly dCurvatureDs: number;
}

/** The closest point on the path; crossTrackMeters is positive when the query is left of the path. */
export interface Projection {
  readonly s: number;
  readonly xMeters: number;
  readonly yMeters: number;
  readonly distanceMeters: number;
  readonly crossTrackMeters: number;
}

export function tangentRadians(g: GeometrySample): number {
  return Math.atan2(g.tangentY, g.tangentX);
}

export class PathGeometry {
  readonly spline: QuinticSpline;
  readonly lengthMeters: number;
  private readonly waypointDistances: Float64Array;
  private readonly pieceCount: number;
  private readonly pieceS: Float64Array;
  private readonly pieceSegment: Int32Array;
  private readonly pieceU0: Float64Array;
  private readonly pieceU1: Float64Array;
  private readonly pieceM0: Float64Array;
  private readonly pieceC2: Float64Array;
  private readonly pieceC3: Float64Array;
  /** Where the path reverses (cusps and near-cusps), metres along it, ascending (PathGeometry.cusps in Java). */
  readonly cusps: readonly number[];

  private constructor(spline: QuinticSpline, table: TableBuilder, waypointS: Float64Array) {
    this.spline = spline;
    this.waypointDistances = waypointS;
    this.lengthMeters = waypointS[waypointS.length - 1]!;
    this.pieceCount = table.count;
    this.pieceS = Float64Array.from(table.s);
    this.pieceSegment = Int32Array.from(table.segment);
    this.pieceU0 = Float64Array.from(table.u0);
    this.pieceU1 = Float64Array.from(table.u1);
    this.pieceM0 = Float64Array.from(table.m0);
    this.pieceC2 = Float64Array.from(table.c2);
    this.pieceC3 = Float64Array.from(table.c3);
    this.cusps = this.findCusps();
  }

  /** The first cusp after s (by more than a micrometre), or NaN. */
  nextCusp(s: number): number {
    for (const c of this.cusps) if (c > s + 1e-6) return c;
    return Number.NaN;
  }

  /** The last cusp at or before s (within a micrometre), or NaN. */
  previousCusp(s: number): number {
    let out = Number.NaN;
    for (const c of this.cusps) if (c <= s + 1e-6) out = c;
    return out;
  }

  private findCusps(): number[] {
    const out: number[] = [];
    const spline = this.spline;
    const v = new Float64Array(CUSP_SCAN + 1);
    for (let k = 0; k < spline.segmentCount; k++) {
      for (let j = 0; j <= CUSP_SCAN; j++) v[j] = spline.speed(k, j / CUSP_SCAN);
      for (let j = 1; j < CUSP_SCAN; j++) {
        if (!(v[j]! <= v[j - 1]! && v[j]! < v[j + 1]!)) continue;
        let a = (j - 1.0) / CUSP_SCAN;
        let b = (j + 1.0) / CUSP_SCAN;
        for (let it = 0; it < 60; it++) {
          const m1 = a + (b - a) * GOLDEN;
          const m2 = b - (b - a) * GOLDEN;
          if (spline.speed(k, m1) < spline.speed(k, m2)) b = m2;
          else a = m1;
        }
        const s = this.sAtParameter(k, 0.5 * (a + b));
        const before = this.sample(Math.max(0.0, s - CUSP_ARC_METERS));
        const after = this.sample(Math.min(this.lengthMeters, s + CUSP_ARC_METERS));
        if (before.tangentX * after.tangentX + before.tangentY * after.tangentY < CUSP_COS) out.push(s);
      }
    }
    return out;
  }

  /** Metres along the path at spline piece k, parameter u: the table's own inverse, by bisection. */
  private sAtParameter(k: number, u: number): number {
    let lo = this.waypointDistances[k]!;
    let hi = this.waypointDistances[k + 1]!;
    for (let it = 0; it < 60; it++) {
      const mid = 0.5 * (lo + hi);
      const i = this.findPiece(mid);
      const len = this.pieceS[i + 1]! - this.pieceS[i]!;
      const sigma = len > 0.0 ? (mid - this.pieceS[i]!) / len : 0.0;
      const um = this.pieceU0[i]! + sigma * (this.pieceM0[i]! + sigma * (this.pieceC2[i]! + sigma * this.pieceC3[i]!));
      if (this.pieceSegment[i]! < k || (this.pieceSegment[i] === k && um < u)) lo = mid;
      else hi = mid;
    }
    return 0.5 * (lo + hi);
  }

  /** Builds the spline through these waypoints and its arc-length table. */
  /** The geometry through these waypoints (the end curvatures fixed when given, as QuinticSpline.through). */
  static build(waypoints: readonly Waypoint[], startCurvature = Number.NaN, endCurvature = Number.NaN): PathGeometry {
    return PathGeometry.fromSpline(QuinticSpline.through(waypoints, startCurvature, endCurvature));
  }

  static fromSpline(spline: QuinticSpline): PathGeometry {
    const n = spline.segmentCount;
    const waypointS = new Float64Array(n + 1);
    const table = new TableBuilder(spline);
    for (let k = 0; k < n; k++) {
      const splits = Math.max(1, Math.ceil(spline.chordMeters(k) / MAX_INITIAL_PIECE_METERS));
      let prevU = 0.0;
      let prevSpeed = table.speed(k, 0.0);
      for (let j = 1; j <= splits; j++) {
        const u = j === splits ? 1.0 : j / splits;
        const speed = table.speed(k, u);
        table.refine(k, prevU, u, prevSpeed, speed, table.gaussLength(k, prevU, u), 0);
        prevU = u;
        prevSpeed = speed;
      }
      waypointS[k + 1] = table.totalS;
    }
    table.s.push(table.totalS);
    return new PathGeometry(spline, table, waypointS);
  }

  /** Number of waypoints. */
  get waypointCount(): number {
    return this.waypointDistances.length;
  }

  /** Metres along the path at waypoint i. */
  waypointS(i: number): number {
    return this.waypointDistances[i]!;
  }

  /** Metres along the path at a waypoint-relative position (1.5 = halfway from 1 to 2), clamped. */
  sAtWaypoint(waypoint: number): number {
    const ws = this.waypointDistances;
    const last = ws.length - 1;
    if (!(waypoint > 0.0)) return 0.0;
    if (waypoint >= last) return this.lengthMeters;
    const k = Math.floor(waypoint);
    const f = waypoint - k;
    return ws[k]! + f * (ws[k + 1]! - ws[k]!);
  }

  /** Number of sub-pieces in the arc-length table. */
  get tablePieceCount(): number {
    return this.pieceCount;
  }

  /** The path at s metres along it (clamped). */
  sample(s: number): GeometrySample {
    const d = new Float64Array(8);
    const sc = this.evaluateAt(s, d);
    return toSample(sc, d);
  }

  /** Finds the piece and parameter for s and evaluates the spline into d; returns s clamped. */
  evaluateAt(s: number, d: Float64Array): number {
    const sc = s < 0.0 ? 0.0 : s > this.lengthMeters ? this.lengthMeters : s;
    const i = this.findPiece(sc);
    const len = this.pieceS[i + 1]! - this.pieceS[i]!;
    const sigma = len > 0.0 ? (sc - this.pieceS[i]!) / len : 0.0;
    let u = this.pieceU0[i]! + sigma * (this.pieceM0[i]! + sigma * (this.pieceC2[i]! + sigma * this.pieceC3[i]!));
    if (u < this.pieceU0[i]!) {
      u = this.pieceU0[i]!;
    } else if (u > this.pieceU1[i]!) {
      u = this.pieceU1[i]!;
    }
    this.spline.evaluate(this.pieceSegment[i]!, u, d);
    return sc;
  }

  private findPiece(s: number): number {
    let lo = 0;
    let hi = this.pieceCount - 1;
    while (lo < hi) {
      const mid = (lo + hi + 1) >>> 1;
      if (this.pieceS[mid]! <= s) {
        lo = mid;
      } else {
        hi = mid - 1;
      }
    }
    return lo;
  }

  /** Closest point to (x, y) with s in [sGuess − halfWindow, sGuess + halfWindow]. */
  project(xMeters: number, yMeters: number, sGuess: number, halfWindowMeters: number): Projection {
    let lo = Math.max(0.0, sGuess - halfWindowMeters);
    let hi = Math.min(this.lengthMeters, sGuess + halfWindowMeters);
    if (hi < lo) {
      lo = hi = sGuess < 0.0 ? 0.0 : this.lengthMeters;
    }
    return this.closestBetween(xMeters, yMeters, lo, hi, WINDOW_STEP_METERS);
  }

  /** Closest point on the whole path to (x, y). */
  /** The closest point to (x, y) with s in [lo, hi] (the window's spacing): a window that stops at a cusp. */
  projectWithin(xMeters: number, yMeters: number, lo: number, hi: number): Projection {
    lo = Math.max(0.0, lo);
    hi = Math.min(this.lengthMeters, hi);
    if (hi < lo) hi = lo;
    return this.closestBetween(xMeters, yMeters, lo, hi, WINDOW_STEP_METERS);
  }

  /** The closest point to (x, y) with s in [lo, hi], at the global search's spacing. */
  projectGlobalWithin(xMeters: number, yMeters: number, lo: number, hi: number): Projection {
    lo = Math.max(0.0, lo);
    hi = Math.min(this.lengthMeters, hi);
    if (hi < lo) hi = lo;
    return this.closestBetween(xMeters, yMeters, lo, hi, GLOBAL_STEP_METERS);
  }

  projectGlobal(xMeters: number, yMeters: number): Projection {
    return this.closestBetween(xMeters, yMeters, 0.0, this.lengthMeters, GLOBAL_STEP_METERS);
  }

  private closestBetween(qx: number, qy: number, lo: number, hi: number, step: number): Projection {
    const d = new Float64Array(8);
    const steps = Math.max(1, Math.ceil((hi - lo) / step));
    const h = (hi - lo) / steps;
    let best = 0;
    let bestDistSq = Infinity;
    for (let i = 0; i <= steps; i++) {
      this.evaluateAt(lo + i * h, d);
      const ex = d[0]! - qx;
      const ey = d[1]! - qy;
      const distSq = ex * ex + ey * ey;
      if (distSq < bestDistSq) {
        bestDistSq = distSq;
        best = i;
      }
    }
    const sBest = lo + best * h;
    let a = best === 0 ? lo : lo + (best - 1) * h;
    let b = best === steps ? hi : lo + (best + 1) * h;

    let s = sBest;
    for (let iter = 0; iter < 30; iter++) {
      this.evaluateAt(s, d);
      const g = toSample(s, d);
      const ex = g.xMeters - qx;
      const ey = g.yMeters - qy;
      const f = ex * g.tangentX + ey * g.tangentY;
      const fp = 1.0 + g.curvature * (-ex * g.tangentY + ey * g.tangentX);
      if (f > 0.0) {
        b = s;
      } else {
        a = s;
      }
      let next = fp > 0.0 ? s - f / fp : Number.NaN;
      if (!(next > a && next < b)) next = 0.5 * (a + b);
      if (Math.abs(next - s) < 1e-10) {
        s = next;
        break;
      }
      s = next;
    }

    const sFinal = this.distanceSq(s, qx, qy, d) <= bestDistSq ? s : sBest;
    this.evaluateAt(sFinal, d);
    const g = toSample(sFinal, d);
    const ex = qx - g.xMeters;
    const ey = qy - g.yMeters;
    return {
      s: sFinal,
      xMeters: g.xMeters,
      yMeters: g.yMeters,
      distanceMeters: Math.sqrt(ex * ex + ey * ey),
      crossTrackMeters: -ex * g.tangentY + ey * g.tangentX,
    };
  }

  private distanceSq(s: number, qx: number, qy: number, d: Float64Array): number {
    this.evaluateAt(s, d);
    const ex = d[0]! - qx;
    const ey = d[1]! - qy;
    return ex * ex + ey * ey;
  }
}

/** Spline derivatives at a point → the sample (tangent, curvature, dκ/ds). */
export function toSample(s: number, d: Float64Array): GeometrySample {
  const dx = d[2]!;
  const dy = d[3]!;
  const q = dx * dx + dy * dy;
  const speed = Math.sqrt(q);
  const cross = dx * d[5]! - dy * d[4]!;
  const dot = dx * d[4]! + dy * d[5]!;
  const dCross = dx * d[7]! - dy * d[6]!;
  const q15 = q * speed;
  const curvature = cross / q15;
  const dCurvatureDu = dCross / q15 - (3.0 * cross * dot) / (q * q15);
  return {
    s,
    xMeters: d[0]!,
    yMeters: d[1]!,
    tangentX: dx / speed,
    tangentY: dy / speed,
    curvature,
    dCurvatureDs: dCurvatureDu / speed,
  };
}

/** The sub-piece table under construction, plus the adaptive refinement. */
class TableBuilder {
  count = 0;
  totalS = 0.0;
  readonly s: number[] = [];
  readonly segment: number[] = [];
  readonly u0: number[] = [];
  readonly u1: number[] = [];
  readonly m0: number[] = [];
  readonly c2: number[] = [];
  readonly c3: number[] = [];

  private readonly spline: QuinticSpline;

  constructor(spline: QuinticSpline) {
    this.spline = spline;
  }

  speed(k: number, u: number): number {
    const v = this.spline.speed(k, u);
    return v > 1e-12 ? v : 1e-12;
  }

  /** Arc length of piece k from ua to ub, 5-point Gauss-Legendre. */
  gaussLength(k: number, ua: number, ub: number): number {
    const h = ub - ua;
    const mid = ua + 0.5 * h;
    const o1 = 0.5 * GL_XI1 * h;
    const o2 = 0.5 * GL_XI2 * h;
    let sum = GL_W0 * this.spline.speed(k, mid);
    sum += GL_W1 * (this.spline.speed(k, mid - o1) + this.spline.speed(k, mid + o1));
    sum += GL_W2 * (this.spline.speed(k, mid - o2) + this.spline.speed(k, mid + o2));
    return sum * h;
  }

  refine(k: number, ua: number, ub: number, speedA: number, speedB: number, length: number, depth: number): void {
    const um = 0.5 * (ua + ub);
    const lengthLeft = this.gaussLength(k, ua, um);
    const lengthRight = this.gaussLength(k, um, ub);
    const total = lengthLeft + lengthRight;
    const speedM = this.speed(k, um);

    const ma = total / speedA;
    const mb = total / speedB;
    const cc2 = 3.0 * (ub - ua) - 2.0 * ma - mb;
    const cc3 = 2.0 * (ua - ub) + ma + mb;
    const sigma = lengthLeft / total;
    const uHat = ua + sigma * (ma + sigma * (cc2 + sigma * cc3));
    const positionError = Math.abs(uHat - um) * speedM;
    const quadratureError = Math.abs(total - length);

    if (
      depth < MAX_DEPTH &&
      (positionError > LOOKUP_TOLERANCE_METERS || quadratureError > QUADRATURE_TOLERANCE_METERS)
    ) {
      this.refine(k, ua, um, speedA, speedM, lengthLeft, depth + 1);
      this.refine(k, um, ub, speedM, speedB, lengthRight, depth + 1);
      return;
    }
    this.s.push(this.totalS);
    this.segment.push(k);
    this.u0.push(ua);
    this.u1.push(ub);
    this.m0.push(ma);
    this.c2.push(cc2);
    this.c3.push(cc3);
    this.count++;
    this.totalS += total;
  }
}
