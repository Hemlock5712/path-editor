// Cusps: places where the path doubles back on itself, so the robot overshoots and comes back. The
// spline's direction flips there (two samples a centimetre apart point opposite ways). They come from a
// tangent set against the way the path goes (by hand, or as the optimiser once could), and from
// automatic tangents at very sharp zigzags. The core will report them itself (Path.cusps(), on its way
// from core-builder); until then this scan finds them, and the check, the optimiser's guard and the
// fix-its all use it.

import { PathGeometry, type PathSpec } from "../../../core/src/index.ts";

/** Sampling step for the scan, metres. */
const STEP = 0.01;

/** Where the path reverses, metres along it (one entry per reversal). */
export function cusps(g: PathGeometry): number[] {
  const out: number[] = [];
  const L = g.lengthMeters;
  let prev = g.sample(0);
  for (let s = STEP; s <= L + 1e-9; s += STEP) {
    const p = g.sample(Math.min(s, L));
    if (prev.tangentX * p.tangentX + prev.tangentY * p.tangentY < 0 && (out.length === 0 || s - out[out.length - 1]! > 0.05)) out.push(s);
    prev = p;
  }
  return out;
}

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
  return cusps(geometry).length > 0;
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
