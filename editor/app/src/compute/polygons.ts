// Convex polygon helpers for the swept-bumper check: hulls (the Java FieldMap hulls every obstacle
// too), the bumper rectangle at a pose, and the exact distance between two convex polygons.

export type Pt = readonly [number, number];

/** Convex hull, counter-clockwise, no repeated or collinear points (Andrew's monotone chain). */
export function convexHull(points: readonly Pt[]): Pt[] {
  const p = [...points].sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  if (p.length < 3) return p;
  const cross = (o: Pt, a: Pt, b: Pt) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower: Pt[] = [];
  for (const q of p) {
    while (lower.length >= 2 && cross(lower[lower.length - 2]!, lower[lower.length - 1]!, q) <= 0) lower.pop();
    lower.push(q);
  }
  const upper: Pt[] = [];
  for (let i = p.length - 1; i >= 0; i--) {
    const q = p[i]!;
    while (upper.length >= 2 && cross(upper[upper.length - 2]!, upper[upper.length - 1]!, q) <= 0) upper.pop();
    upper.push(q);
  }
  return lower.slice(0, -1).concat(upper.slice(0, -1));
}

/** The bumper rectangle's corners at a pose: length along the heading, width across it. */
export function bumperCorners(x: number, y: number, heading: number, length: number, width: number): Pt[] {
  const c = Math.cos(heading);
  const s = Math.sin(heading);
  const hl = length / 2;
  const hw = width / 2;
  const corner = (u: number, v: number): Pt => [x + u * c - v * s, y + u * s + v * c];
  return [corner(hl, hw), corner(-hl, hw), corner(-hl, -hw), corner(hl, -hw)];
}

/** True when two convex polygons overlap or touch (separating-axis test). */
export function convexOverlap(a: readonly Pt[], b: readonly Pt[]): boolean {
  return !hasSeparatingAxis(a, b) && !hasSeparatingAxis(b, a);
}

function hasSeparatingAxis(a: readonly Pt[], b: readonly Pt[]): boolean {
  for (let i = 0; i < a.length; i++) {
    const p = a[i]!;
    const q = a[(i + 1) % a.length]!;
    const nx = q[1] - p[1];
    const ny = p[0] - q[0];
    let minA = Infinity;
    let maxA = -Infinity;
    for (const v of a) {
      const d = v[0] * nx + v[1] * ny;
      minA = Math.min(minA, d);
      maxA = Math.max(maxA, d);
    }
    let minB = Infinity;
    let maxB = -Infinity;
    for (const v of b) {
      const d = v[0] * nx + v[1] * ny;
      minB = Math.min(minB, d);
      maxB = Math.max(maxB, d);
    }
    if (maxA < minB || maxB < minA) return true;
  }
  return false;
}

function pointSegmentDistance(p: Pt, a: Pt, b: Pt): number {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const len2 = dx * dx + dy * dy;
  const t = len2 > 0 ? Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2)) : 0;
  return Math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy));
}

/** Distance between two convex polygons; 0 when they overlap. */
export function convexDistance(a: readonly Pt[], b: readonly Pt[]): number {
  if (convexOverlap(a, b)) return 0;
  let best = Infinity;
  for (const [from, to] of [
    [a, b],
    [b, a],
  ] as const) {
    for (const p of from) {
      for (let i = 0; i < to.length; i++) best = Math.min(best, pointSegmentDistance(p, to[i]!, to[(i + 1) % to.length]!));
    }
  }
  return best;
}

/** How far a polygon's points stay inside a centred rectangle (the field walls); negative = outside. */
export function insideRectMargin(poly: readonly Pt[], halfLength: number, halfWidth: number): number {
  let m = Infinity;
  for (const [x, y] of poly) m = Math.min(m, halfLength - Math.abs(x), halfWidth - Math.abs(y));
  return m;
}
