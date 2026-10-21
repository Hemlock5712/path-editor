// Everything the editor shows about one path, computed from the core: the plan sampled along s and
// in time, event and target positions, and the live checks. Pure, so it runs in the worker and in
// tests alike. No path maths is done here; every number comes from Path.build and its samples.

import {
  eventMeters,
  gripMetersPerSecondSquared,
  makeRotationTarget,
  Path,
  wheelSpeedCapMetersPerSecond,
  type PathSpec,
  type RobotConfig,
} from "../../../core/src/index.ts";
import { bumperCorners, convexDistance, convexHull, insideRectMargin, type Pt } from "./polygons.ts";

export interface AnalysisInput {
  readonly spec: PathSpec;
  readonly robot: RobotConfig;
  readonly bumperLengthMeters: number;
  readonly bumperWidthMeters: number;
  readonly fieldLengthMeters: number;
  readonly fieldWidthMeters: number;
  readonly obstacles: readonly { readonly name: string; readonly polygon: readonly Pt[] }[];
}

/** Columns sampled along the path. omega = v·dθ/ds. */
export interface Stations {
  readonly s: Float64Array;
  readonly x: Float64Array;
  readonly y: Float64Array;
  readonly heading: Float64Array;
  readonly v: Float64Array;
  readonly a: Float64Array;
  readonly kappa: Float64Array;
  readonly t: Float64Array;
  readonly omega: Float64Array;
}

/** Columns sampled in time every dt seconds (the playback reads these). */
export interface TimeSamples {
  readonly dt: number;
  readonly s: Float64Array;
  readonly x: Float64Array;
  readonly y: Float64Array;
  readonly heading: Float64Array;
  readonly v: Float64Array;
  readonly omega: Float64Array;
}

export interface PlacedEvent {
  /** Index in spec.events. */
  readonly index: number;
  readonly name: string;
  readonly s: number;
  readonly t: number;
  readonly x: number;
  readonly y: number;
}

export type CheckKind = "curvature" | "heading" | "bumper" | "event" | "position" | "speed";

export interface Check {
  readonly kind: CheckKind;
  readonly severity: "error" | "warning";
  readonly message: string;
  /** The stretch of path it applies to, metres. */
  readonly sFrom: number;
  readonly sTo: number;
}

export interface Analysis {
  readonly ok: true;
  readonly lengthMeters: number;
  readonly totalTimeSeconds: number;
  readonly stations: Stations;
  readonly byTime: TimeSamples;
  readonly waypointS: readonly number[];
  readonly events: readonly PlacedEvent[];
  readonly checks: readonly Check[];
  /** Closest the swept bumpers come to any obstacle or the field wall, metres (0 = touching). */
  readonly minClearanceMeters: number;
  readonly buildMillis: number;
}

export interface AnalysisFailure {
  readonly ok: false;
  readonly error: string;
}

/** Thresholds for the checks. */
export const CHECKS = {
  /** A bend is "tight" when its steady cornering cap is below this share of the path's top speed. */
  tightBendShare: 0.5,
  /** A turn is "too sharp" when it slows the plan to below this share of the speed without it... */
  headingSlowShare: 0.75,
  /** ...and by at least this much, m/s. */
  headingSlowMetersPerSecond: 0.3,
  /** Bumper clearance below this is a warning (the pathfinder's default margin). */
  bumperMarginMeters: 0.05,
};

const STATION_METERS = 0.02;
const TIME_STEP = 0.02;

export function analyse(input: AnalysisInput): Analysis | AnalysisFailure {
  const t0 = performance.now();
  let path: Path;
  try {
    path = Path.build(input.spec, input.robot);
  } catch (e) {
    return { ok: false, error: e instanceof Error ? e.message : String(e) };
  }
  const buildMillis = performance.now() - t0;
  const stations = sampleStations(path);
  const byTime = sampleTimes(path);
  const waypointS: number[] = [];
  for (let i = 0; i < path.geometry.waypointCount; i++) waypointS.push(path.geometry.waypointS(i));
  const events = input.spec.events.map((e, index) => {
    const s = Math.min(path.lengthMeters, Math.max(0, eventMeters(e, path.geometry)));
    const p = path.sample(s);
    return { index, name: e.name, s, t: p.seconds, x: p.xMeters, y: p.yMeters };
  });
  const bumpers = bumperChecks(input, stations);
  const checks = [
    ...speedChecks(input, path, stations),
    ...curvatureChecks(input, path, stations),
    ...headingChecks(input, stations),
    ...bumpers.checks,
    ...positionChecks(input, path),
  ];
  return {
    ok: true,
    lengthMeters: path.lengthMeters,
    totalTimeSeconds: path.totalTimeSeconds,
    stations,
    byTime,
    waypointS,
    events,
    checks,
    minClearanceMeters: bumpers.minClearance,
    buildMillis,
  };
}

function stationList(length: number): number[] {
  const n = Math.max(2, Math.ceil(length / STATION_METERS) + 1);
  const out: number[] = [];
  for (let i = 0; i < n; i++) out.push((length * i) / (n - 1));
  return out;
}

function sampleStations(path: Path): Stations {
  const list = stationList(path.lengthMeters);
  const cols = {
    s: new Float64Array(list.length),
    x: new Float64Array(list.length),
    y: new Float64Array(list.length),
    heading: new Float64Array(list.length),
    v: new Float64Array(list.length),
    a: new Float64Array(list.length),
    kappa: new Float64Array(list.length),
    t: new Float64Array(list.length),
    omega: new Float64Array(list.length),
  };
  list.forEach((s, i) => {
    const p = path.sample(s);
    cols.s[i] = p.s;
    cols.x[i] = p.xMeters;
    cols.y[i] = p.yMeters;
    cols.heading[i] = p.headingRadians;
    cols.v[i] = p.metersPerSecond;
    cols.a[i] = p.metersPerSecondSquared;
    cols.kappa[i] = p.curvature;
    cols.t[i] = p.seconds;
    cols.omega[i] = p.metersPerSecond * p.headingRadiansPerMeter;
  });
  return cols;
}

function sampleTimes(path: Path): TimeSamples {
  const total = path.totalTimeSeconds;
  const n = Math.max(2, Math.ceil(total / TIME_STEP) + 1);
  const cols = {
    dt: TIME_STEP,
    s: new Float64Array(n),
    x: new Float64Array(n),
    y: new Float64Array(n),
    heading: new Float64Array(n),
    v: new Float64Array(n),
    omega: new Float64Array(n),
  };
  for (let i = 0; i < n; i++) {
    const p = path.sampleAtTime(Math.min(total, i * TIME_STEP));
    cols.s[i] = p.s;
    cols.x[i] = p.xMeters;
    cols.y[i] = p.yMeters;
    cols.heading[i] = p.headingRadians;
    cols.v[i] = p.metersPerSecond;
    cols.omega[i] = p.metersPerSecond * p.headingRadiansPerMeter;
  }
  return cols;
}

/** Runs of consecutive stations where pred holds, as [first, last] index pairs. */
function runs(n: number, pred: (i: number) => boolean): [number, number][] {
  const out: [number, number][] = [];
  let start = -1;
  for (let i = 0; i < n; i++) {
    if (pred(i)) {
      if (start < 0) start = i;
    } else if (start >= 0) {
      out.push([start, i - 1]);
      start = -1;
    }
  }
  if (start >= 0) out.push([start, n - 1]);
  return out;
}

const f2 = (x: number) => x.toFixed(2);
const deg = (r: number) => Math.round((r * 180) / Math.PI);

/** The plan can't start or end at the authored speed (a bend or a turn caps it there). */
function speedChecks(input: AnalysisInput, path: Path, st: Stations): Check[] {
  const out: Check[] = [];
  const n = st.s.length;
  const { startMetersPerSecond: start, endMetersPerSecond: end } = input.spec;
  if (start - st.v[0]! > 1e-6) {
    out.push({
      kind: "speed",
      severity: "warning",
      message: `The plan can't start at ${f2(start)} m/s; it starts at ${f2(st.v[0]!)} m/s (a bend or turn caps it there).`,
      sFrom: 0,
      sTo: Math.min(path.lengthMeters, 0.3),
    });
  }
  if (end - st.v[n - 1]! > 1e-6) {
    out.push({
      kind: "speed",
      severity: "warning",
      message: `The plan can't end at ${f2(end)} m/s; it ends at ${f2(st.v[n - 1]!)} m/s (a bend or turn caps it there).`,
      sFrom: Math.max(0, path.lengthMeters - 0.3),
      sTo: path.lengthMeters,
    });
  }
  return out;
}

/** Bends whose steady cornering speed sqrt(μg/|κ|) is well under the path's top speed. */
function curvatureChecks(input: AnalysisInput, path: Path, st: Stations): Check[] {
  const grip = gripMetersPerSecondSquared(input.robot);
  const top = Math.min(input.spec.maxMetersPerSecond, wheelSpeedCapMetersPerSecond(input.robot));
  const limit = CHECKS.tightBendShare * top;
  const cap = (i: number) => Math.sqrt(grip / Math.max(1e-12, Math.abs(st.kappa[i]!)));
  return runs(st.s.length, (i) => cap(i) < limit).map(([a, b]) => {
    let worst = a;
    for (let i = a; i <= b; i++) if (Math.abs(st.kappa[i]!) > Math.abs(st.kappa[worst]!)) worst = i;
    const radius = 1 / Math.abs(st.kappa[worst]!);
    return {
      kind: "curvature",
      severity: "warning",
      message:
        `Tight bend at ${f2(st.s[worst]!)} m: radius ${f2(radius)} m caps the speed at ${f2(cap(worst))} m/s ` +
        `(path top speed ${f2(top)} m/s). Spread the waypoints or lengthen the tangents.`,
      sFrom: st.s[a]!,
      sTo: Math.min(path.lengthMeters, st.s[b]!),
    };
  });
}

/**
 * Turns too sharp for their distance: where the planned heading slows the plan well below the same
 * path planned without turning (one constant heading). This is the "face-point blend turns more
 * than 100° and the plan drops to 0.7 m/s" case.
 */
function headingChecks(input: AnalysisInput, st: Stations): Check[] {
  const spec = input.spec;
  if (spec.rotationTargets.length === 0 && spec.facePointZones.length === 0) return [];
  let still: Path;
  try {
    still = Path.build(
      { ...spec, rotationTargets: [makeRotationTarget(Number.NaN, 0, st.heading[0]!)], facePointZones: [] },
      input.robot,
    );
  } catch {
    return [];
  }
  const n = st.s.length;
  const free = new Float64Array(n);
  for (let i = 0; i < n; i++) free[i] = still.profile.velocityAt(Math.min(still.lengthMeters, st.s[i]!));
  const slowed = (i: number) =>
    st.v[i]! < CHECKS.headingSlowShare * free[i]! && free[i]! - st.v[i]! > CHECKS.headingSlowMetersPerSecond;
  return runs(n, slowed).map(([a, b]) => {
    let worst = a;
    for (let i = a; i <= b; i++) if (free[i]! - st.v[i]! > free[worst]! - st.v[worst]!) worst = i;
    // the turn: total heading change over the stretch, widened to where the heading rate is high
    let lo = a;
    let hi = b;
    while (lo > 0 && Math.abs(st.omega[lo - 1]!) > 0.5) lo--;
    while (hi < n - 1 && Math.abs(st.omega[hi + 1]!) > 0.5) hi++;
    let turn = 0;
    for (let i = lo; i < hi; i++) turn += Math.abs(wrapDiff(st.heading[i + 1]! - st.heading[i]!));
    const dist = st.s[hi]! - st.s[lo]!;
    return {
      kind: "heading",
      severity: "warning",
      message:
        `The heading turns ${deg(turn)}° over ${f2(dist)} m (${f2(st.s[lo]!)}–${f2(st.s[hi]!)} m), so the plan ` +
        `slows to ${f2(st.v[worst]!)} m/s here (${f2(free[worst]!)} m/s without the turn). ` +
        `Spread the turn over more distance, e.g. a longer face-point blend.`,
      sFrom: st.s[a]!,
      sTo: st.s[b]!,
    };
  });
}

function wrapDiff(d: number): number {
  return Math.atan2(Math.sin(d), Math.cos(d));
}

/** The bumper rectangle swept along the plan (with its heading) against obstacles and walls. */
function bumperChecks(input: AnalysisInput, st: Stations): { checks: Check[]; minClearance: number } {
  const n = st.s.length;
  const L = input.bumperLengthMeters;
  const W = input.bumperWidthMeters;
  const shapes = input.obstacles.map((o) => ({ name: o.name, hull: convexHull(o.polygon) })).filter((o) => o.hull.length >= 3);
  const bodies: Pt[][] = [];
  for (let i = 0; i < n; i++) bodies.push(bumperCorners(st.x[i]!, st.y[i]!, st.heading[i]!, L, W));
  const out: Check[] = [];
  const margin = CHECKS.bumperMarginMeters;
  let minClearance = Infinity;

  const report = (what: string, gap: (i: number) => number) => {
    const gaps = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      gaps[i] = Math.max(0, gap(i));
      minClearance = Math.min(minClearance, gaps[i]!);
    }
    for (const [a, b] of runs(n, (i) => gaps[i]! < margin)) {
      let worst = a;
      for (let i = a; i <= b; i++) if (gaps[i]! < gaps[worst]!) worst = i;
      const touching = gaps[worst]! <= 0;
      out.push({
        kind: "bumper",
        severity: touching ? "error" : "warning",
        message: touching
          ? `Bumpers hit ${what} from ${f2(st.s[a]!)} to ${f2(st.s[b]!)} m.`
          : `Bumpers pass ${f2(gaps[worst]! * 100)} cm from ${what} at ${f2(st.s[worst]!)} m ` +
            `(under the ${f2(margin * 100)} cm margin).`,
        sFrom: st.s[a]!,
        sTo: st.s[b]!,
      });
    }
  };

  for (const o of shapes) report(o.name, (i) => convexDistance(bodies[i]!, o.hull));
  report("the field wall", (i) => insideRectMargin(bodies[i]!, input.fieldLengthMeters / 2, input.fieldWidthMeters / 2));
  return { checks: out, minClearance };
}

/** Events, rotation targets and zones placed off the path. */
function positionChecks(input: AnalysisInput, path: Path): Check[] {
  const out: Check[] = [];
  const last = input.spec.waypoints.length - 1;
  const length = path.lengthMeters;
  const off = (waypoint: number, meters: number) =>
    Number.isNaN(waypoint) ? meters < 0 || meters > length + 1e-5 : waypoint < 0 || waypoint > last;
  const where = (waypoint: number, meters: number) =>
    Number.isNaN(waypoint) ? `${f2(meters)} m (the path is ${f2(length)} m)` : `waypoint ${waypoint} (the last is ${last})`;
  input.spec.events.forEach((e) => {
    if (off(e.waypoint, e.meters)) {
      const s = Number.isNaN(e.waypoint) ? e.meters : e.waypoint < 0 ? 0 : length;
      out.push({
        kind: "event",
        severity: "error",
        message: `Event "${e.name}" is off the path, at ${where(e.waypoint, e.meters)}. The robot would fire it at the nearest end.`,
        sFrom: Math.min(length, Math.max(0, s)),
        sTo: Math.min(length, Math.max(0, s)),
      });
    }
  });
  input.spec.rotationTargets.forEach((t, i) => {
    if (off(t.waypoint, t.meters)) {
      out.push({
        kind: "position",
        severity: "warning",
        message: `Rotation target ${i + 1} is off the path, at ${where(t.waypoint, t.meters)}.`,
        sFrom: Number.isNaN(t.waypoint) && t.meters < 0 ? 0 : length,
        sTo: Number.isNaN(t.waypoint) && t.meters < 0 ? 0 : length,
      });
    }
  });
  const zones = [
    ...input.spec.facePointZones.map((z, i) => ({ z, label: `Face-point zone ${i + 1}` })),
    ...input.spec.constraintZones.map((z, i) => ({ z, label: `Constraint zone ${i + 1}` })),
  ];
  for (const { z, label } of zones) {
    if (z.fromWaypoint < 0 || z.toWaypoint > last) {
      out.push({
        kind: "position",
        severity: "warning",
        message: `${label} runs off the path (waypoints ${z.fromWaypoint} to ${z.toWaypoint}; the last is ${last}).`,
        sFrom: path.geometry.sAtWaypoint(z.fromWaypoint),
        sTo: path.geometry.sAtWaypoint(z.toWaypoint),
      });
    }
  }
  return out;
}
