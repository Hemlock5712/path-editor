import { Point } from '../types';
import { CubicSegment } from './CubicSegment';

// Exact Gauss-Legendre constants from SplinePath.java
const GL_NODES = [
  -0.906179845938664, -0.5384693101056831, 0.0, 0.5384693101056831, 0.906179845938664,
];
const GL_WEIGHTS = [
  0.2369268850561891, 0.4786286704993665, 0.5688888888888889, 0.4786286704993665,
  0.2369268850561891,
];

const SAMPLES_PER_METER = 1000;
const MIN_SAMPLES = 100;

/**
 * Arc-length parameterized Catmull-Rom spline path.
 * Direct port of SplinePath.java.
 */
export class SplinePath {
  private readonly segments: CubicSegment[];
  private readonly sTable: Float64Array;
  private readonly segIndexTable: Int32Array;
  private readonly tTable: Float64Array;
  readonly totalLength: number;
  readonly controlPoints: Point[];

  constructor(controlPoints: Point[]) {
    if (controlPoints.length < 2) {
      throw new Error('SplinePath requires at least 2 control points');
    }

    this.controlPoints = [...controlPoints];
    const n = controlPoints.length;

    // Build Catmull-Rom segments (N-1 segments for N points)
    this.segments = [];
    for (let i = 0; i < n - 1; i++) {
      const p0 = controlPoints[i];
      const p1 = controlPoints[i + 1];

      // Catmull-Rom tangents: 0.5 * (P[i+1] - P[i-1]), endpoints use chord to neighbor
      let m0: Point;
      if (i === 0) {
        m0 = { x: p1.x - p0.x, y: p1.y - p0.y };
      } else {
        m0 = {
          x: (controlPoints[i + 1].x - controlPoints[i - 1].x) * 0.5,
          y: (controlPoints[i + 1].y - controlPoints[i - 1].y) * 0.5,
        };
      }

      let m1: Point;
      if (i === n - 2) {
        m1 = { x: p1.x - p0.x, y: p1.y - p0.y };
      } else {
        m1 = {
          x: (controlPoints[i + 2].x - controlPoints[i].x) * 0.5,
          y: (controlPoints[i + 2].y - controlPoints[i].y) * 0.5,
        };
      }

      this.segments.push(new CubicSegment(p0, p1, m0, m1));
    }

    // Build arc-length lookup table
    let roughLength = 0;
    for (const seg of this.segments) {
      roughLength += integrateSegmentLength(seg, 0, 1);
    }

    const numSamples = Math.max(MIN_SAMPLES, Math.floor(roughLength * SAMPLES_PER_METER));
    this.sTable = new Float64Array(numSamples + 1);
    this.segIndexTable = new Int32Array(numSamples + 1);
    this.tTable = new Float64Array(numSamples + 1);

    this.buildArcLengthTable(numSamples);
    this.totalLength = this.sTable[numSamples];
  }

  getPoint(s: number): Point {
    s = this.clampS(s);
    const lr = this.lookupS(s);
    return this.segments[lr.segIndex].getPoint(lr.t);
  }

  getTangent(s: number): Point {
    s = this.clampS(s);
    const lr = this.lookupS(s);
    const d = this.segments[lr.segIndex].getDerivative(lr.t);
    const mag = Math.hypot(d.x, d.y);
    if (mag < 1e-12) return { x: 1, y: 0 };
    return { x: d.x / mag, y: d.y / mag };
  }

  getCurvature(s: number): number {
    s = this.clampS(s);
    const lr = this.lookupS(s);
    return this.segments[lr.segIndex].getCurvature(lr.t);
  }

  getTotalLength(): number {
    return this.totalLength;
  }

  getNumSegments(): number {
    return this.segments.length;
  }

  getArcLengthAtWaypointIndex(waypointIndex: number): number {
    if (waypointIndex <= 0) return 0;
    if (waypointIndex >= this.segments.length) return this.totalLength;
    const numSamples = this.sTable.length - 1;
    const tableIndex = Math.round((waypointIndex * numSamples) / this.segments.length);
    return this.sTable[Math.min(tableIndex, numSamples)];
  }

  private buildArcLengthTable(numSamples: number): void {
    const totalT = this.segments.length;
    const dtPerSample = totalT / numSamples;

    this.sTable[0] = 0;
    this.segIndexTable[0] = 0;
    this.tTable[0] = 0;

    let cumulativeS = 0;

    for (let i = 1; i <= numSamples; i++) {
      const globalT = i * dtPerSample;
      const segIdx = Math.min(Math.floor(globalT), this.segments.length - 1);
      const localT = globalT - segIdx;

      const prevGlobalT = (i - 1) * dtPerSample;
      const prevSegIdx = Math.min(Math.floor(prevGlobalT), this.segments.length - 1);
      const prevLocalT = prevGlobalT - prevSegIdx;

      if (prevSegIdx === segIdx) {
        cumulativeS += integrateSegmentLength(this.segments[segIdx], prevLocalT, localT);
      } else {
        cumulativeS += integrateSegmentLength(this.segments[prevSegIdx], prevLocalT, 1.0);
        cumulativeS += integrateSegmentLength(this.segments[segIdx], 0.0, localT);
      }

      this.sTable[i] = cumulativeS;
      this.segIndexTable[i] = segIdx;
      this.tTable[i] = localT;
    }
  }

  private lookupS(s: number): { segIndex: number; t: number } {
    if (s <= 0) return { segIndex: 0, t: 0 };
    if (s >= this.totalLength) return { segIndex: this.segments.length - 1, t: 1.0 };

    let lo = 0;
    let hi = this.sTable.length - 1;
    while (lo + 1 < hi) {
      const mid = (lo + hi) >>> 1;
      if (this.sTable[mid] <= s) lo = mid;
      else hi = mid;
    }

    const sRange = this.sTable[hi] - this.sTable[lo];
    const frac = sRange > 1e-12 ? (s - this.sTable[lo]) / sRange : 0;

    const segLo = this.segIndexTable[lo];
    const tLo = this.tTable[lo];
    const segHi = this.segIndexTable[hi];
    const tHi = this.tTable[hi];

    if (segLo === segHi) {
      return { segIndex: segLo, t: tLo + frac * (tHi - tLo) };
    } else {
      if (frac < 0.5) {
        const t = tLo + frac * 2 * (1.0 - tLo);
        return { segIndex: segLo, t: Math.min(t, 1.0) };
      } else {
        const t = (frac - 0.5) * 2 * tHi;
        return { segIndex: segHi, t: Math.max(t, 0.0) };
      }
    }
  }

  /** Find the arc-length of the closest point on the path to the given field point. */
  getClosestPointS(point: Point): number {
    // Coarse pass: sample at intervals of ~2cm (balances precision vs performance)
    const COARSE_SEARCH_STEP = 0.02;
    const step = Math.max(COARSE_SEARCH_STEP, this.totalLength / 500);
    let bestS = 0;
    let bestDist = Infinity;

    for (let s = 0; s <= this.totalLength; s += step) {
      const p = this.getPoint(s);
      const d = (p.x - point.x) ** 2 + (p.y - point.y) ** 2;
      if (d < bestDist) {
        bestDist = d;
        bestS = s;
      }
    }

    // Refine with bisection around the best sample
    let lo = Math.max(0, bestS - step);
    let hi = Math.min(this.totalLength, bestS + step);
    for (let i = 0; i < 16; i++) {
      const m1 = lo + (hi - lo) / 3;
      const m2 = hi - (hi - lo) / 3;
      const p1 = this.getPoint(m1);
      const p2 = this.getPoint(m2);
      const d1 = (p1.x - point.x) ** 2 + (p1.y - point.y) ** 2;
      const d2 = (p2.x - point.x) ** 2 + (p2.y - point.y) ** 2;
      if (d1 < d2) hi = m2;
      else lo = m1;
    }
    return (lo + hi) / 2;
  }

  private clampS(s: number): number {
    return Math.max(0, Math.min(s, this.totalLength));
  }
}

function integrateSegmentLength(segment: CubicSegment, t0: number, t1: number): number {
  if (Math.abs(t1 - t0) < 1e-12) return 0;

  const halfRange = (t1 - t0) / 2.0;
  const midpoint = (t0 + t1) / 2.0;

  let sum = 0;
  for (let i = 0; i < GL_NODES.length; i++) {
    const t = midpoint + halfRange * GL_NODES[i];
    sum += GL_WEIGHTS[i] * segment.getSpeed(t);
  }
  return sum * halfRange;
}
