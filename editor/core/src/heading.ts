// The planned robot heading over s, with dθ/ds and d²θ/ds². Port of lib's HeadingProfile.java:
// face-forward with no targets, otherwise quintic blends between targets the shortest way round
// (the spin stops at the first and last target and where the turn reverses; elsewhere it passes the
// target still turning, see passThroughSlopes), then face-point zones eased in and out on top.

import { smoothstep5, smoothstep5Bend, smoothstep5Slope, wrap } from "./angles.ts";
import type { GeometrySample, PathGeometry } from "./geometry.ts";
import type { FacePointZone, HeadingMode, RotationTarget } from "./spec.ts";

export interface HeadingSample {
  /** Heading, wrapped to (-π, π]. */
  readonly radians: number;
  /** dθ/ds: the planned spin is v · dθ/ds. */
  readonly radiansPerMeter: number;
  /** d²θ/ds². */
  readonly radiansPerMeterSquared: number;
}

export class HeadingProfile {
  private readonly geometry: PathGeometry;
  // Rotation targets sorted by s, headings unwrapped so each step is the shortest turn
  private readonly targetS: Float64Array;
  private readonly targetRadians: Float64Array;
  private readonly targetSlope: Float64Array; // dθ/ds at each target (passThroughSlopes)
  // Face-point zones in authored order
  private readonly zoneStartS: Float64Array;
  private readonly zoneEndS: Float64Array;
  private readonly zoneBlendMeters: Float64Array;
  private readonly zonePointX: Float64Array;
  private readonly zonePointY: Float64Array;

  private constructor(
    geometry: PathGeometry,
    targetS: Float64Array,
    targetRadians: Float64Array,
    zoneStartS: Float64Array,
    zoneEndS: Float64Array,
    zoneBlendMeters: Float64Array,
    zonePointX: Float64Array,
    zonePointY: Float64Array,
  ) {
    this.geometry = geometry;
    this.targetS = targetS;
    this.targetRadians = targetRadians;
    this.targetSlope = passThroughSlopes(targetS, targetRadians);
    this.zoneStartS = zoneStartS;
    this.zoneEndS = zoneEndS;
    this.zoneBlendMeters = zoneBlendMeters;
    this.zonePointX = zonePointX;
    this.zonePointY = zonePointY;
  }

  /** With no targets, "hold" holds the direction the path starts out in; "faceTravel" faces travel. */
  static build(
    geometry: PathGeometry,
    targets: readonly RotationTarget[],
    zones: readonly FacePointZone[],
    mode: HeadingMode = "faceTravel",
  ): HeadingProfile {
    if (mode === "hold" && targets.length === 0) {
      // nothing set: hold the direction the path starts out in
      const start = geometry.sample(0.0);
      targets = [{ waypoint: Number.NaN, meters: 0.0, headingRadians: Math.atan2(start.tangentY, start.tangentX) }];
    }
    // Resolve to metres and insertion-sort by s (stable: ties keep the authored order)
    const nt = targets.length;
    const ts = new Float64Array(nt);
    const heading = new Float64Array(nt);
    for (let j = 0; j < nt; j++) {
      const t = targets[j]!;
      const s = !Number.isNaN(t.waypoint) ? geometry.sAtWaypoint(t.waypoint) : clamp(t.meters, geometry);
      let at = j;
      while (at > 0 && ts[at - 1]! > s) {
        ts[at] = ts[at - 1]!;
        heading[at] = heading[at - 1]!;
        at--;
      }
      ts[at] = s;
      heading[at] = t.headingRadians;
    }
    const tr = new Float64Array(nt);
    for (let j = 0; j < nt; j++) {
      tr[j] = j === 0 ? heading[0]! : tr[j - 1]! + wrap(heading[j]! - tr[j - 1]!);
    }

    const nz = zones.length;
    const za = new Float64Array(nz);
    const zb = new Float64Array(nz);
    const zblend = new Float64Array(nz);
    const zx = new Float64Array(nz);
    const zy = new Float64Array(nz);
    for (let j = 0; j < nz; j++) {
      const z = zones[j]!;
      za[j] = geometry.sAtWaypoint(z.fromWaypoint);
      zb[j] = geometry.sAtWaypoint(z.toWaypoint);
      zblend[j] = Math.min(z.blendMeters, 0.5 * (zb[j]! - za[j]!));
      zx[j] = z.point.x;
      zy[j] = z.point.y;
    }
    return new HeadingProfile(geometry, ts, tr, za, zb, zblend, zx, zy);
  }

  /** The heading at s. */
  sample(s: number): HeadingSample {
    return this.sampleAt(this.geometry.sample(s));
  }

  /** The heading at a geometry sample already in hand. */
  sampleAt(g: GeometrySample): HeadingSample {
    const s = g.s;
    const targetS = this.targetS;
    const targetRadians = this.targetRadians;
    let theta: number;
    let dTheta: number;
    let ddTheta: number;

    if (targetS.length === 0) {
      theta = Math.atan2(g.tangentY, g.tangentX);
      dTheta = g.curvature;
      ddTheta = g.dCurvatureDs;
    } else if (s <= targetS[0]!) {
      theta = targetRadians[0]!;
      dTheta = 0.0;
      ddTheta = 0.0;
    } else if (s >= targetS[targetS.length - 1]!) {
      theta = targetRadians[targetS.length - 1]!;
      dTheta = 0.0;
      ddTheta = 0.0;
    } else {
      let j = 0;
      while (targetS[j + 1]! <= s) j++;
      const length = targetS[j + 1]! - targetS[j]!;
      const turn = targetRadians[j + 1]! - targetRadians[j]!;
      const tau = (s - targetS[j]!) / length;
      // quintic Hermite: the targets, the rates m0 and m1 there, zero d²θ/ds² at both
      const m0 = this.targetSlope[j]!;
      const m1 = this.targetSlope[j + 1]!;
      const t2 = tau * tau;
      const t3 = t2 * tau;
      const t4 = t3 * tau;
      const h1 = tau - 6.0 * t3 + 8.0 * t4 - 3.0 * t4 * tau;
      const h4 = -4.0 * t3 + 7.0 * t4 - 3.0 * t4 * tau;
      const h1d = 1.0 - 18.0 * t2 + 32.0 * t3 - 15.0 * t4;
      const h4d = -12.0 * t2 + 28.0 * t3 - 15.0 * t4;
      const h1dd = -36.0 * tau + 96.0 * t2 - 60.0 * t3;
      const h4dd = -24.0 * tau + 84.0 * t2 - 60.0 * t3;
      theta = targetRadians[j]! + turn * smoothstep5(tau) + length * (m0 * h1 + m1 * h4);
      dTheta = (turn * smoothstep5Slope(tau)) / length + m0 * h1d + m1 * h4d;
      ddTheta = (turn * smoothstep5Bend(tau)) / (length * length) + (m0 * h1dd + m1 * h4dd) / length;
    }

    for (let j = 0; j < this.zoneStartS.length; j++) {
      const a = this.zoneStartS[j]!;
      const b = this.zoneEndS[j]!;
      if (s < a || s > b) continue;
      const dx = this.zonePointX[j]! - g.xMeters;
      const dy = this.zonePointY[j]! - g.yMeters;
      const distSq = dx * dx + dy * dy;
      if (distSq < 1e-9) continue;
      const blend = this.zoneBlendMeters[j]!;
      let w = 1.0;
      let dw = 0.0;
      let ddw = 0.0;
      if (blend > 0.0) {
        if (s < a + blend) {
          const tau = (s - a) / blend;
          w = smoothstep5(tau);
          dw = smoothstep5Slope(tau) / blend;
          ddw = smoothstep5Bend(tau) / (blend * blend);
        } else if (s > b - blend) {
          const tau = (b - s) / blend;
          w = smoothstep5(tau);
          dw = -smoothstep5Slope(tau) / blend;
          ddw = smoothstep5Bend(tau) / (blend * blend);
        }
      }
      const face = Math.atan2(dy, dx);
      const cross = g.tangentX * dy - g.tangentY * dx;
      const along = g.tangentX * dx + g.tangentY * dy;
      const dFace = cross / distSq;
      const ddFace = (2.0 * cross * along - g.curvature * along * distSq) / (distSq * distSq);

      const gap = wrap(face - theta);
      const newTheta = theta + w * gap;
      const newDTheta = dTheta + dw * gap + w * (dFace - dTheta);
      const newDdTheta = ddTheta + ddw * gap + 2.0 * dw * (dFace - dTheta) + w * (ddFace - ddTheta);
      theta = newTheta;
      dTheta = newDTheta;
      ddTheta = newDdTheta;
    }
    return { radians: wrap(theta), radiansPerMeter: dTheta, radiansPerMeterSquared: ddTheta };
  }
}

function clamp(s: number, g: PathGeometry): number {
  return s < 0.0 ? 0.0 : s > g.lengthMeters ? g.lengthMeters : s;
}

/**
 * dθ/ds at each target (lib's HeadingProfile.passThroughSlopes): zero at the first and the last, and where
 * the turn reverses or pauses; otherwise the harmonic mean of the two stretches' average rates, which never
 * overshoots a target.
 */
export function passThroughSlopes(s: Float64Array, radians: Float64Array): Float64Array {
  const n = s.length;
  const m = new Float64Array(n);
  for (let i = 1; i < n - 1; i++) {
    const la = s[i]! - s[i - 1]!;
    const lb = s[i + 1]! - s[i]!;
    if (!(la > 0.0) || !(lb > 0.0)) continue; // a jump at a shared position: stop there
    const da = (radians[i]! - radians[i - 1]!) / la;
    const db = (radians[i + 1]! - radians[i]!) / lb;
    if (da * db > 0.0) m[i] = (2.0 * da * db) / (da + db);
  }
  return m;
}
