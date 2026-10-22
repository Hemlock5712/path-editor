// A chain of quintic Hermite pieces with curvature continuous at every knot. Line-for-line port of
// lib's QuinticSpline.java (see there for the maths); keep the arithmetic in the same order so the
// fixtures match to the last bits.

import { hasTangent, type Waypoint } from "./spec.ts";

/** Shortest allowed distance between consecutive waypoints. */
export const MIN_CHORD_METERS = 1e-6;

export class QuinticSpline {
  readonly segmentCount: number;
  private readonly originX: Float64Array;
  private readonly originY: Float64Array;
  // Power-basis coefficients c1..c5 per piece, relative to the piece's start point
  private readonly cx: Float64Array;
  private readonly cy: Float64Array;
  private readonly chord: Float64Array;
  private readonly knotTangent: Float64Array;
  private readonly knotKappa: Float64Array;

  private constructor(segmentCount: number) {
    this.segmentCount = segmentCount;
    this.originX = new Float64Array(segmentCount);
    this.originY = new Float64Array(segmentCount);
    this.cx = new Float64Array(5 * segmentCount);
    this.cy = new Float64Array(5 * segmentCount);
    this.chord = new Float64Array(segmentCount);
    this.knotTangent = new Float64Array(segmentCount + 1);
    this.knotKappa = new Float64Array(segmentCount + 1);
  }

  /**
   * Builds the spline through these waypoints, with the curvature at the first and at the last waypoint
   * fixed when given (NaN = chosen like the rest): a piece that joins an existing path matches its curvature.
   */
  static through(waypoints: readonly Waypoint[], startCurvature = Number.NaN, endCurvature = Number.NaN): QuinticSpline {
    const n = waypoints.length - 1;
    if (n < 1) throw new RangeError("a spline needs at least two waypoints");
    const spline = new QuinticSpline(n);

    const px = new Float64Array(n + 1);
    const py = new Float64Array(n + 1);
    for (let i = 0; i <= n; i++) {
      px[i] = waypoints[i]!.xMeters;
      py[i] = waypoints[i]!.yMeters;
    }

    // Unit chords
    const ex = new Float64Array(n);
    const ey = new Float64Array(n);
    const chord = spline.chord;
    for (let k = 0; k < n; k++) {
      const dx = px[k + 1]! - px[k]!;
      const dy = py[k + 1]! - py[k]!;
      chord[k] = Math.sqrt(dx * dx + dy * dy);
      if (!(chord[k]! >= MIN_CHORD_METERS)) {
        throw new RangeError(`waypoints ${k} and ${k + 1} are at the same place`);
      }
      ex[k] = dx / chord[k]!;
      ey[k] = dy / chord[k]!;
    }

    // Unit tangents: given ones first, then automatic interior ones, then automatic ends
    const dx = new Float64Array(n + 1);
    const dy = new Float64Array(n + 1);
    for (let i = 0; i <= n; i++) {
      const w = waypoints[i]!;
      if (hasTangent(w)) {
        dx[i] = Math.cos(w.tangentRadians);
        dy[i] = Math.sin(w.tangentRadians);
      } else if (i > 0 && i < n) {
        // Bessel tangent: the slope of the parabola through the three points, by chord length
        const l0 = chord[i - 1]!;
        const l1 = chord[i]!;
        const tx = (l1 * ex[i - 1]! + l0 * ex[i]!) / (l0 + l1);
        const ty = (l1 * ey[i - 1]! + l0 * ey[i]!) / (l0 + l1);
        const len = Math.sqrt(tx * tx + ty * ty);
        if (len < 1e-9) {
          dx[i] = ex[i]!;
          dy[i] = ey[i]!;
        } else {
          dx[i] = tx / len;
          dy[i] = ty / len;
        }
      }
    }
    const autoStart = !hasTangent(waypoints[0]!);
    const autoEnd = !hasTangent(waypoints[n]!);
    if (n === 1 && autoStart && autoEnd) {
      dx[0] = ex[0]!;
      dy[0] = ey[0]!;
      dx[1] = ex[0]!;
      dy[1] = ey[0]!;
    } else {
      // An automatic end mirrors its neighbour's tangent across the chord between them
      if (autoStart) {
        const k = 2.0 * (dx[1]! * ex[0]! + dy[1]! * ey[0]!);
        dx[0] = k * ex[0]! - dx[1]!;
        dy[0] = k * ey[0]! - dy[1]!;
      }
      if (autoEnd) {
        const k = 2.0 * (dx[n - 1]! * ex[n - 1]! + dy[n - 1]! * ey[n - 1]!);
        dx[n] = k * ex[n - 1]! - dx[n - 1]!;
        dy[n] = k * ey[n - 1]! - dy[n - 1]!;
      }
    }
    for (let i = 0; i <= n; i++) {
      spline.knotTangent[i] = Math.atan2(dy[i]!, dx[i]!);
    }

    // End speeds |V| per piece: circular-arc length h / sin h times the chord, times tangentScale
    const mag0 = new Float64Array(n);
    const mag1 = new Float64Array(n);
    for (let k = 0; k < n; k++) {
      const turn0 = Math.atan2(ex[k]! * dy[k]! - ey[k]! * dx[k]!, ex[k]! * dx[k]! + ey[k]! * dy[k]!);
      const turn1 = Math.atan2(
        ex[k]! * dy[k + 1]! - ey[k]! * dx[k + 1]!,
        ex[k]! * dx[k + 1]! + ey[k]! * dy[k + 1]!,
      );
      let h = 0.5 * (Math.abs(turn0) + Math.abs(turn1));
      if (h > 0.5 * Math.PI) h = 0.5 * Math.PI;
      const arc = h > 1e-6 ? (chord[k]! * h) / Math.sin(h) : chord[k]!;
      mag0[k] = waypoints[k]!.tangentScale * arc;
      mag1[k] = waypoints[k + 1]!.tangentScale * arc;
    }

    // Knot curvatures: minimise Σ w ∫|p'''|² with w = 1/L⁵ (integrals as in the Java source)
    const diag = new Float64Array(n + 1);
    const upper = new Float64Array(n + 1);
    const rhs = new Float64Array(n + 1);
    for (let k = 0; k < n; k++) {
      const l = chord[k]!;
      const weight = 1.0 / (l * l * l * l * l);
      const a0 = mag0[k]!;
      const a1 = mag1[k]!;
      const a0sq = a0 * a0;
      const a1sq = a1 * a1;
      const chordX = px[k + 1]! - px[k]!;
      const chordY = py[k + 1]! - py[k]!;
      const n0x = -dy[k]!;
      const n0y = dx[k]!;
      const n1x = -dy[k + 1]!;
      const n1y = dx[k + 1]!;
      const chordDotN0 = chordX * n0x + chordY * n0y;
      const chordDotN1 = chordX * n1x + chordY * n1y;
      const v1DotN0 = a1 * (dx[k + 1]! * n0x + dy[k + 1]! * n0y);
      const v0DotN1 = a0 * (dx[k]! * n1x + dy[k]! * n1y);

      diag[k] = diag[k]! + weight * 9.0 * a0sq * a0sq;
      diag[k + 1] = diag[k + 1]! + weight * 9.0 * a1sq * a1sq;
      upper[k] = upper[k]! + weight * -3.0 * a0sq * a1sq * (n0x * n1x + n0y * n1y);
      rhs[k] = rhs[k]! - weight * a0sq * (-60.0 * chordDotN0 + 24.0 * v1DotN0);
      rhs[k + 1] = rhs[k + 1]! - weight * a1sq * (60.0 * chordDotN1 - 24.0 * v0DotN1);
    }
    if (!Number.isNaN(startCurvature)) {
      // A fixed κ0: move its column to the right-hand side and make row 0 say κ0 = startCurvature
      rhs[1] = rhs[1]! - upper[0]! * startCurvature;
      diag[0] = 1.0;
      upper[0] = 0.0;
      rhs[0] = startCurvature;
    }
    if (!Number.isNaN(endCurvature)) {
      // the same for κn
      rhs[n - 1] = rhs[n - 1]! - upper[n - 1]! * endCurvature;
      upper[n - 1] = 0.0;
      diag[n] = 1.0;
      rhs[n] = endCurvature;
    }
    // Thomas algorithm
    for (let i = 1; i <= n; i++) {
      const m = upper[i - 1]! / diag[i - 1]!;
      diag[i] = diag[i]! - m * upper[i - 1]!;
      rhs[i] = rhs[i]! - m * rhs[i - 1]!;
    }
    const kappa = spline.knotKappa;
    kappa[n] = rhs[n]! / diag[n]!;
    for (let i = n - 1; i >= 0; i--) {
      kappa[i] = (rhs[i]! - upper[i]! * kappa[i + 1]!) / diag[i]!;
    }

    // Power-basis coefficients per piece
    for (let k = 0; k < n; k++) {
      const a0 = mag0[k]!;
      const a1 = mag1[k]!;
      const v0x = a0 * dx[k]!;
      const v0y = a0 * dy[k]!;
      const v1x = a1 * dx[k + 1]!;
      const v1y = a1 * dy[k + 1]!;
      const acc0 = a0 * a0 * kappa[k]!;
      const acc1 = a1 * a1 * kappa[k + 1]!;
      const acc0x = -dy[k]! * acc0;
      const acc0y = dx[k]! * acc0;
      const acc1x = -dy[k + 1]! * acc1;
      const acc1y = dx[k + 1]! * acc1;
      const chordX = px[k + 1]! - px[k]!;
      const chordY = py[k + 1]! - py[k]!;
      spline.originX[k] = px[k]!;
      spline.originY[k] = py[k]!;
      setCoefficients(spline.cx, k, chordX, v0x, v1x, acc0x, acc1x);
      setCoefficients(spline.cy, k, chordY, v0y, v1y, acc0y, acc1y);
    }
    return spline;
  }

  /** Straight-line distance between waypoints k and k + 1. */
  chordMeters(k: number): number {
    return this.chord[k]!;
  }

  /** Direction of travel at waypoint i, given or chosen. */
  knotTangentRadians(i: number): number {
    return this.knotTangent[i]!;
  }

  /** Curvature at waypoint i, 1/m (positive = turning left). */
  knotCurvature(i: number): number {
    return this.knotKappa[i]!;
  }

  /** Evaluates piece k at u into out[0..7]: x, y, x', y', x'', y'', x''', y''' (d/du). */
  evaluate(k: number, u: number, out: Float64Array | number[]): void {
    const o = 5 * k;
    const cx = this.cx;
    const cy = this.cy;
    const x1 = cx[o]!;
    const x2 = cx[o + 1]!;
    const x3 = cx[o + 2]!;
    const x4 = cx[o + 3]!;
    const x5 = cx[o + 4]!;
    const y1 = cy[o]!;
    const y2 = cy[o + 1]!;
    const y3 = cy[o + 2]!;
    const y4 = cy[o + 3]!;
    const y5 = cy[o + 4]!;
    out[0] = this.originX[k]! + u * (x1 + u * (x2 + u * (x3 + u * (x4 + u * x5))));
    out[1] = this.originY[k]! + u * (y1 + u * (y2 + u * (y3 + u * (y4 + u * y5))));
    out[2] = x1 + u * (2.0 * x2 + u * (3.0 * x3 + u * (4.0 * x4 + u * 5.0 * x5)));
    out[3] = y1 + u * (2.0 * y2 + u * (3.0 * y3 + u * (4.0 * y4 + u * 5.0 * y5)));
    out[4] = 2.0 * x2 + u * (6.0 * x3 + u * (12.0 * x4 + u * 20.0 * x5));
    out[5] = 2.0 * y2 + u * (6.0 * y3 + u * (12.0 * y4 + u * 20.0 * y5));
    out[6] = 6.0 * x3 + u * (24.0 * x4 + u * 60.0 * x5);
    out[7] = 6.0 * y3 + u * (24.0 * y4 + u * 60.0 * y5);
  }

  /** |p'(u)| on piece k: metres of arc per unit of u. */
  speed(k: number, u: number): number {
    const o = 5 * k;
    const cx = this.cx;
    const cy = this.cy;
    const dx = cx[o]! + u * (2.0 * cx[o + 1]! + u * (3.0 * cx[o + 2]! + u * (4.0 * cx[o + 3]! + u * 5.0 * cx[o + 4]!)));
    const dy = cy[o]! + u * (2.0 * cy[o + 1]! + u * (3.0 * cy[o + 2]! + u * (4.0 * cy[o + 3]! + u * 5.0 * cy[o + 4]!)));
    return Math.sqrt(dx * dx + dy * dy);
  }
}

function setCoefficients(
  c: Float64Array,
  k: number,
  chord: number,
  v0: number,
  v1: number,
  a0: number,
  a1: number,
): void {
  const o = 5 * k;
  c[o] = v0;
  c[o + 1] = 0.5 * a0;
  c[o + 2] = 10.0 * chord - 6.0 * v0 - 4.0 * v1 - 1.5 * a0 + 0.5 * a1;
  c[o + 3] = -15.0 * chord + 8.0 * v0 + 7.0 * v1 + 1.5 * a0 - a1;
  c[o + 4] = 6.0 * chord - 3.0 * v0 - 3.0 * v1 - 0.5 * a0 + 0.5 * a1;
}
