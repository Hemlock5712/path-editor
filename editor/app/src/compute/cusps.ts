// Cusps: places where the path doubles back on itself, so the robot overshoots and comes back. The core
// finds them (PathGeometry.cusps: a speed minimum where the direction turns more than 120°); here are the
// editor's uses of them: whether a spec reverses at all (including a tangent set against its chord, which
// the optimiser rules out before it can make one), and which waypoint to blame for one.

import { PathGeometry, type PathSpec } from "../../../core/src/index.ts";

/** How far (radians) a tangent may point from the local chord before it counts as reversed. */
export const REVERSED_RADIANS = (100 * Math.PI) / 180;

/** The local chord direction at waypoint i: from the one before to the one after (the ends: their one chord). */
function chordAt(spec: PathSpec, i: number): number {
  const w = spec.waypoints;
  const a = w[Math.max(0, i - 1)]!;
  const b = w[Math.min(w.length - 1, i + 1)]!;
  return Math.atan2(b.yMeters - a.yMeters, b.xMeters - a.xMeters);
}

/** The waypoints whose given tangent points more than REVERSED_RADIANS from the local chord. */
export function reversedTangents(spec: PathSpec): number[] {
  if (spec.waypoints.length < 2) return [];
  const out: number[] = [];
  spec.waypoints.forEach((w, i) => {
    if (Number.isNaN(w.tangentRadians)) return;
    const d = Math.abs(Math.atan2(Math.sin(w.tangentRadians - chordAt(spec, i)), Math.cos(w.tangentRadians - chordAt(spec, i))));
    if (d > REVERSED_RADIANS) out.push(i);
  });
  return out;
}

/** True if the spec plans a path that doubles back anywhere (or has a tangent set against its chord). */
export function hasReversal(spec: PathSpec, g: PathGeometry | null = null): boolean {
  if (reversedTangents(spec).length > 0) return true;
  let geometry = g;
  if (!geometry) {
    try {
      geometry = PathGeometry.build(spec.waypoints);
    } catch {
      return false;
    }
  }
  return geometry.cusps.length > 0;
}

/** The waypoint to blame for a cusp at s: a reversed tangent nearby, or the closer end of its stretch. */
export function cuspWaypoint(spec: PathSpec, g: PathGeometry, s: number): number {
  const n = spec.waypoints.length;
  let k = 0;
  for (let i = 0; i < n - 1; i++) if (g.sAtWaypoint(i) <= s) k = i;
  const ends = [k, Math.min(n - 1, k + 1)];
  const reversed = reversedTangents(spec).filter((i) => ends.includes(i));
  if (reversed.length > 0) return reversed[0]!;
  const given = ends.filter((i) => !Number.isNaN(spec.waypoints[i]!.tangentRadians));
  const pool = given.length > 0 ? given : ends;
  return pool.reduce((best, i) => (Math.abs(g.sAtWaypoint(i) - s) < Math.abs(g.sAtWaypoint(best) - s) ? i : best));
}
