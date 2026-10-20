// Waypoint-relative positions (1.5 = halfway along the arc from waypoint 1 to 2) are how rotation
// targets, zones and events stay attached when the geometry changes. Inserting or deleting a waypoint
// renumbers them, so these maps keep each position on the same stretch of path.

import type { PathGeometry } from "../../../core/src/index.ts";

/**
 * Where position p goes when a waypoint is inserted between k and k + 1 at fraction f of that
 * stretch (0 < f < 1). The stretch [k, k + 1] splits into [k, k + 1] and [k + 1, k + 2].
 */
export function afterInsert(p: number, k: number, f: number): number {
  if (p <= k) return p;
  if (p >= k + 1) return p + 1;
  const local = p - k;
  return local < f ? k + local / f : k + 1 + (local - f) / (1 - f);
}

/**
 * Where position p goes when waypoint i of n is deleted. The two stretches either side of it merge
 * into one, shared in proportion; positions on a deleted end stretch clamp to the new end.
 */
export function afterDelete(p: number, i: number, n: number): number {
  const last = n - 1;
  if (i === 0) return Math.max(0, p - 1);
  if (i === last) return Math.min(p, last - 1);
  if (p <= i - 1) return p;
  if (p >= i + 1) return p - 1;
  return i - 1 + (p - (i - 1)) / 2;
}

/** The waypoint-relative position at s metres along the path: the inverse of sAtWaypoint. */
export function waypointAtS(geometry: PathGeometry, s: number): number {
  const n = geometry.waypointCount;
  if (s <= 0) return 0;
  if (s >= geometry.lengthMeters) return n - 1;
  for (let k = 0; k < n - 1; k++) {
    const a = geometry.waypointS(k);
    const b = geometry.waypointS(k + 1);
    if (s <= b) return b > a ? k + (s - a) / (b - a) : k;
  }
  return n - 1;
}

/** Rounds to the file's 5 decimals, so what the editor shows is what gets saved. */
export function round5(x: number): number {
  return Math.round(x * 100000) / 100000;
}

const DEGREES = Math.PI / 180.0;

/**
 * An angle the file holds exactly: whole 1e-5 degrees, converted the way the readers convert, so a
 * saved and reloaded path plans exactly as previewed.
 */
export function round5Radians(radians: number): number {
  return round5(radians / DEGREES) * DEGREES;
}
