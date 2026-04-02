import { Point } from '../types';

/**
 * Single cubic Hermite spline segment: p(t) = a*t^3 + b*t^2 + c*t + d for t in [0,1].
 * Direct port of CubicSegment.java — same Hermite basis coefficients.
 */
export class CubicSegment {
  private readonly ax: number;
  private readonly bx: number;
  private readonly cx: number;
  private readonly dx: number;
  private readonly ay: number;
  private readonly by: number;
  private readonly cy: number;
  private readonly dy: number;

  constructor(p0: Point, p1: Point, m0: Point, m1: Point) {
    this.ax = 2 * p0.x - 2 * p1.x + m0.x + m1.x;
    this.bx = -3 * p0.x + 3 * p1.x - 2 * m0.x - m1.x;
    this.cx = m0.x;
    this.dx = p0.x;

    this.ay = 2 * p0.y - 2 * p1.y + m0.y + m1.y;
    this.by = -3 * p0.y + 3 * p1.y - 2 * m0.y - m1.y;
    this.cy = m0.y;
    this.dy = p0.y;
  }

  getPoint(t: number): Point {
    const t2 = t * t;
    const t3 = t2 * t;
    return {
      x: this.ax * t3 + this.bx * t2 + this.cx * t + this.dx,
      y: this.ay * t3 + this.by * t2 + this.cy * t + this.dy,
    };
  }

  getDerivative(t: number): Point {
    const t2 = t * t;
    return {
      x: 3 * this.ax * t2 + 2 * this.bx * t + this.cx,
      y: 3 * this.ay * t2 + 2 * this.by * t + this.cy,
    };
  }

  getSecondDerivative(t: number): Point {
    return {
      x: 6 * this.ax * t + 2 * this.bx,
      y: 6 * this.ay * t + 2 * this.by,
    };
  }

  getCurvature(t: number): number {
    const d1 = this.getDerivative(t);
    const d2 = this.getSecondDerivative(t);
    const cross = d1.x * d2.y - d1.y * d2.x;
    const speedSq = d1.x * d1.x + d1.y * d1.y;
    const speedCubed = speedSq * Math.sqrt(speedSq);
    if (speedCubed < 1e-12) return 0;
    return cross / speedCubed;
  }

  getSpeed(t: number): number {
    const d = this.getDerivative(t);
    return Math.hypot(d.x, d.y);
  }
}
