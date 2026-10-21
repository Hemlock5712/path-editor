// A convex polygon for the pathfinder: a port of lib's plan/ConvexPolygon.java, line for line, so the
// pathfinder's fixtures (fixtures/plan) come out the same in both languages.

import { javaHypot } from "../jmath.ts";

/** Points closer than this to the boundary count as on it (outside the open interior). */
export const EPS = 1e-7;

type Pt = readonly [number, number];

function cross(o: Pt, a: Pt, b: Pt): number {
  return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

export function segmentPointDistance(ax: number, ay: number, bx: number, by: number, px: number, py: number): number {
  const ex = bx - ax;
  const ey = by - ay;
  const l2 = ex * ex + ey * ey;
  const t = l2 === 0 ? 0 : Math.max(0.0, Math.min(1.0, ((px - ax) * ex + (py - ay) * ey) / l2));
  return javaHypot(px - ax - t * ex, py - ay - t * ey);
}

function insideCcw(qx: ArrayLike<number>, qy: ArrayLike<number>, x: number, y: number): boolean {
  const m = qx.length;
  for (let k = 0; k < m; k++) {
    const l = (k + 1) % m;
    if ((qx[l]! - qx[k]!) * (y - qy[k]!) - (qy[l]! - qy[k]!) * (x - qx[k]!) < 0) return false;
  }
  return true;
}

function segmentsCross(ax: number, ay: number, bx: number, by: number, cx: number, cy: number, dx: number, dy: number): boolean {
  const d1 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  const d2 = (bx - ax) * (dy - ay) - (by - ay) * (dx - ax);
  const d3 = (dx - cx) * (ay - cy) - (dy - cy) * (ax - cx);
  const d4 = (dx - cx) * (by - cy) - (dy - cy) * (bx - cx);
  return d1 * d2 < 0 && d3 * d4 < 0;
}

/** A convex polygon, counter-clockwise. Immutable. */
export class ConvexPolygon {
  readonly xs: Float64Array;
  readonly ys: Float64Array;
  /** Outward unit normal of edge i (from vertex i to i + 1). */
  readonly nx: Float64Array;
  readonly ny: Float64Array;
  readonly cx: number;
  readonly cy: number;
  /** Distance from (cx, cy) to the farthest vertex. */
  readonly radius: number;

  private constructor(xs: Float64Array, ys: Float64Array) {
    const n = xs.length;
    if (n < 3) throw new RangeError("a polygon needs at least three distinct corners");
    this.xs = xs;
    this.ys = ys;
    this.nx = new Float64Array(n);
    this.ny = new Float64Array(n);
    let sx = 0;
    let sy = 0;
    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n;
      const ex = xs[j]! - xs[i]!;
      const ey = ys[j]! - ys[i]!;
      const len = javaHypot(ex, ey);
      this.nx[i] = ey / len; // right of a CCW edge is outside
      this.ny[i] = -ex / len;
      sx += xs[i]!;
      sy += ys[i]!;
    }
    this.cx = sx / n;
    this.cy = sy / n;
    let r = 0;
    for (let i = 0; i < n; i++) r = Math.max(r, javaHypot(xs[i]! - this.cx, ys[i]! - this.cy));
    this.radius = r;
  }

  /** The convex hull of the points (Andrew's monotone chain), counter-clockwise. */
  static of(points: readonly Pt[]): ConvexPolygon {
    const p = points.slice().sort((a, b) => (a[0] !== b[0] ? cmp(a[0], b[0]) : cmp(a[1], b[1])));
    const n = p.length;
    const hull: Pt[] = new Array(2 * n);
    let k = 0;
    for (let i = 0; i < n; i++) {
      while (k >= 2 && cross(hull[k - 2]!, hull[k - 1]!, p[i]!) <= 1e-12) k--;
      hull[k++] = p[i]!;
    }
    for (let i = n - 2, t = k + 1; i >= 0; i--) {
      while (k >= t && cross(hull[k - 2]!, hull[k - 1]!, p[i]!) <= 1e-12) k--;
      hull[k++] = p[i]!;
    }
    const m = k - 1; // the last point repeats the first
    const xs = new Float64Array(m);
    const ys = new Float64Array(m);
    for (let i = 0; i < m; i++) {
      xs[i] = hull[i]![0];
      ys[i] = hull[i]![1];
    }
    return new ConvexPolygon(xs, ys);
  }

  get size(): number {
    return this.xs.length;
  }

  /** The Minkowski sum with a disc of radius r, approximated from outside (see ConvexPolygon.grow). */
  grow(r: number, maxStepRadians: number): ConvexPolygon {
    const pts: Pt[] = [];
    const n = this.xs.length;
    for (let i = 0; i < n; i++) {
      const prev = (i + n - 1) % n;
      const a0 = Math.atan2(this.ny[prev]!, this.nx[prev]!);
      const a1 = Math.atan2(this.ny[i]!, this.nx[i]!);
      let turn = a1 - a0;
      while (turn < 0) turn += 2 * Math.PI;
      const steps = Math.max(1, Math.ceil(turn / maxStepRadians - 1e-9));
      const step = turn / steps;
      const rr = r / Math.cos(step / 2);
      for (let k = 0; k <= steps; k++) {
        const a = a0 + k * step;
        pts.push([this.xs[i]! + rr * Math.cos(a), this.ys[i]! + rr * Math.sin(a)]);
      }
    }
    return ConvexPolygon.of(pts);
  }

  /** True when (x, y) is in the open interior (more than EPS inside every edge). */
  containsStrictly(x: number, y: number): boolean {
    for (let i = 0; i < this.xs.length; i++) {
      if (this.nx[i]! * (x - this.xs[i]!) + this.ny[i]! * (y - this.ys[i]!) >= -EPS) return false;
    }
    return true;
  }

  /** True when the segment passes through the open interior (Cyrus-Beck clipping). */
  blocksSegment(x0: number, y0: number, x1: number, y1: number): boolean {
    const dx = x1 - x0;
    const dy = y1 - y0;
    if (segmentPointDistance(x0, y0, x1, y1, this.cx, this.cy) > this.radius) return false;
    let tEnter = 0.0;
    let tLeave = 1.0;
    for (let i = 0; i < this.xs.length; i++) {
      const num = this.nx[i]! * (x0 - this.xs[i]!) + this.ny[i]! * (y0 - this.ys[i]!) + EPS;
      const den = this.nx[i]! * dx + this.ny[i]! * dy;
      if (Math.abs(den) < 1e-15) {
        if (num >= 0) return false;
        continue;
      }
      const t = -num / den;
      if (den < 0) tEnter = Math.max(tEnter, t);
      else tLeave = Math.min(tLeave, t);
      if (tEnter >= tLeave) return false;
    }
    return tLeave - tEnter > 1e-9;
  }

  /** Distance from (x, y) to the polygon: 0 inside. */
  distanceTo(x: number, y: number): number {
    let inside = true;
    for (let i = 0; i < this.xs.length; i++) {
      if (this.nx[i]! * (x - this.xs[i]!) + this.ny[i]! * (y - this.ys[i]!) > 0) {
        inside = false;
        break;
      }
    }
    if (inside) return 0.0;
    let best = Infinity;
    for (let i = 0; i < this.xs.length; i++) {
      const j = (i + 1) % this.xs.length;
      best = Math.min(best, segmentPointDistance(this.xs[i]!, this.ys[i]!, this.xs[j]!, this.ys[j]!, x, y));
    }
    return best;
  }

  /** The closest point on the boundary to (x, y), from inside or outside. */
  nearestBoundaryPoint(x: number, y: number): [number, number] {
    let best = Infinity;
    let bx = 0;
    let by = 0;
    for (let i = 0; i < this.xs.length; i++) {
      const j = (i + 1) % this.xs.length;
      const ex = this.xs[j]! - this.xs[i]!;
      const ey = this.ys[j]! - this.ys[i]!;
      const t = Math.max(0.0, Math.min(1.0, ((x - this.xs[i]!) * ex + (y - this.ys[i]!) * ey) / (ex * ex + ey * ey)));
      const px = this.xs[i]! + t * ex;
      const py = this.ys[i]! + t * ey;
      const d = javaHypot(x - px, y - py);
      if (d < best) {
        best = d;
        bx = px;
        by = py;
      }
    }
    return [bx, by];
  }

  /** Distance to another convex polygon given by corner arrays (counter-clockwise): 0 when they touch or overlap. */
  distanceToPolygon(qx: ArrayLike<number>, qy: ArrayLike<number>): number {
    const m = qx.length;
    for (let k = 0; k < m; k++) if (this.distanceTo(qx[k]!, qy[k]!) === 0.0) return 0.0;
    for (let i = 0; i < this.xs.length; i++) if (insideCcw(qx, qy, this.xs[i]!, this.ys[i]!)) return 0.0;
    let best = Infinity;
    for (let i = 0; i < this.xs.length; i++) {
      const j = (i + 1) % this.xs.length;
      for (let k = 0; k < m; k++) {
        const l = (k + 1) % m;
        if (segmentsCross(this.xs[i]!, this.ys[i]!, this.xs[j]!, this.ys[j]!, qx[k]!, qy[k]!, qx[l]!, qy[l]!)) return 0.0;
        best = Math.min(best, segmentPointDistance(this.xs[i]!, this.ys[i]!, this.xs[j]!, this.ys[j]!, qx[k]!, qy[k]!));
        best = Math.min(best, segmentPointDistance(qx[k]!, qy[k]!, qx[l]!, qy[l]!, this.xs[i]!, this.ys[i]!));
      }
    }
    return best;
  }
}

/** Java's Double.compare for finite values (the sort comparator). */
function cmp(a: number, b: number): number {
  return a < b ? -1 : a > b ? 1 : 0;
}
