/**
 * Shortest-arc signed difference b - a, wrapped to [-PI, PI].
 */
export function shortestArcDiff(a: number, b: number): number {
  let diff = ((b - a + Math.PI) % (2 * Math.PI)) - Math.PI;
  if (diff < -Math.PI) diff += 2 * Math.PI;
  return diff;
}

/**
 * Shortest-arc angle interpolation.
 */
export function lerpAngle(a: number, b: number, t: number): number {
  return a + shortestArcDiff(a, b) * t;
}
