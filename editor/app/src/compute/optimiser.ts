// The waypoint optimiser: shortens a path's planned time by moving its free interior waypoints (each
// within a corridor around where it was) and their tangent scales. Every plan comes from the core.
//
// Method: Nelder-Mead (adaptive coefficients for the dimension) on normalised coordinates, restarted
// around the best point with a smaller simplex when it stalls. The start simplex's directions come
// from a seeded generator, so a run is deterministic for a seed and an evaluation budget; the wall-clock
// budget only cuts it short.
//
// What stays put:
//   - the first and last waypoints, any the user locked, any linked to a named point, and any that a
//     zone starts or ends on or between (so zone anchors don't move);
//   - tangent directions (only the scales change);
//   - events, rotation targets and zones by waypoint position keep that position (the same fraction of
//     their stretch); those by metres keep the same fraction of the path's length.
// A candidate only counts if it plans, its swept bumpers keep at least the clearance the path had (up to
// the 5 cm margin), and it has no more heading warnings or errors than the path had.

import {
  makeWaypoint,
  Path,
  PathGeometry,
  type PathEvent,
  type PathSpec,
  type RotationTarget,
} from "../../../core/src/index.ts";
import { round5 } from "../model/positions.ts";
import { analyse, CHECKS, type AnalysisInput } from "./analysis.ts";

export interface OptimiseOptions {
  /** How far each waypoint may move, metres. */
  readonly corridorMeters: number;
  /** Per waypoint: true = don't move it. */
  readonly locked: readonly boolean[];
  /** Also change the free waypoints' tangent scales (within half to double). */
  readonly optimiseScales: boolean;
  readonly seed: number;
  /** The deterministic budget. */
  readonly maxEvaluations: number;
  /** Wall-clock cut, milliseconds (Infinity in tests). */
  readonly timeBudgetMs: number;
}

export const DEFAULT_OPTIONS: OptimiseOptions = {
  corridorMeters: 0.5,
  locked: [],
  optimiseScales: true,
  seed: 1,
  maxEvaluations: 3000,
  timeBudgetMs: 5000,
};

export interface OptimiseResult {
  readonly spec: PathSpec;
  readonly beforeSeconds: number;
  readonly afterSeconds: number;
  readonly evaluations: number;
  readonly stoppedBy: "evaluations" | "time" | "converged" | "cancelled";
  /** Indices of the waypoints it was allowed to move. */
  readonly free: readonly number[];
}

/** The waypoints that must stay put: ends, locked, and zone anchors. */
export function fixedWaypoints(spec: PathSpec, locked: readonly boolean[]): boolean[] {
  const n = spec.waypoints.length;
  const fixed = spec.waypoints.map((_, i) => i === 0 || i === n - 1 || locked[i] === true);
  const anchor = (p: number) => {
    for (const i of [Math.floor(p), Math.ceil(p)]) if (i >= 0 && i < n) fixed[i] = true;
  };
  for (const z of [...spec.facePointZones, ...spec.constraintZones]) {
    anchor(z.fromWaypoint);
    anchor(z.toWaypoint);
  }
  return fixed;
}

/** A small seeded generator (mulberry32), so runs repeat. */
function random(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

interface Feasibility {
  readonly ok: boolean;
  readonly time: number;
}

export class Optimiser {
  readonly free: readonly number[];
  readonly beforeSeconds: number;
  private readonly dims: number;
  private readonly baseClearance: number;
  private readonly baseHeading: number;
  private readonly baseErrors: number;
  private readonly baseLength: number;
  private readonly rand: () => number;
  private evaluations = 0;
  private bestX: number[];
  private bestTime: number;
  private bestSpec: PathSpec;
  private simplex: number[][] = [];
  private values: number[] = [];
  private step = 0.5;
  private stalls = 0;
  private converged = false;

  constructor(
    private readonly input: AnalysisInput,
    private readonly options: OptimiseOptions,
  ) {
    const spec = input.spec;
    const fixed = fixedWaypoints(spec, options.locked);
    this.free = spec.waypoints.map((_, i) => i).filter((i) => !fixed[i]);
    this.dims = this.free.length * (options.optimiseScales ? 3 : 2);
    const base = analyse(input);
    if (!base.ok) throw new Error(`the path can't be planned: ${base.error}`);
    this.beforeSeconds = base.totalTimeSeconds;
    this.baseClearance = base.minClearanceMeters;
    this.baseHeading = base.checks.filter((c) => c.kind === "heading").length;
    this.baseErrors = base.checks.filter((c) => c.severity === "error").length;
    this.baseLength = base.lengthMeters;
    this.rand = random(options.seed);
    this.bestX = new Array(this.dims).fill(0);
    this.bestTime = this.beforeSeconds;
    this.bestSpec = spec;
    if (this.dims > 0) this.restart();
  }

  get evaluationCount(): number {
    return this.evaluations;
  }

  get bestSeconds(): number {
    return this.bestTime;
  }

  get best(): PathSpec {
    return this.bestSpec;
  }

  get isConverged(): boolean {
    return this.converged || this.dims === 0;
  }

  /** The spec for normalised coordinates x (rounded to what the file holds). */
  candidate(x: readonly number[]): PathSpec {
    const spec = this.input.spec;
    const r = this.options.corridorMeters;
    const k = this.options.optimiseScales ? 3 : 2;
    const waypoints = spec.waypoints.slice();
    this.free.forEach((wi, j) => {
      let u = x[j * k]!;
      let v = x[j * k + 1]!;
      const len = Math.hypot(u, v);
      if (len > 1) {
        u /= len;
        v /= len;
      }
      const w = spec.waypoints[wi]!;
      const scale = this.options.optimiseScales
        ? Math.round(w.tangentScale * Math.pow(2, Math.max(-1, Math.min(1, x[j * k + 2]!))) * 100) / 100
        : w.tangentScale;
      waypoints[wi] = makeWaypoint(round5(w.xMeters + r * u), round5(w.yMeters + r * v), w.tangentRadians, scale);
    });
    // things placed by metres keep their fraction of the length
    let ratio = 1;
    try {
      ratio = PathGeometry.build(waypoints).lengthMeters / this.baseLength;
    } catch {
      ratio = 1;
    }
    const rotationTargets = spec.rotationTargets.map(
      (t): RotationTarget => (Number.isNaN(t.meters) ? t : { ...t, meters: round5(t.meters * ratio) }),
    );
    const events = spec.events.map((e): PathEvent => (Number.isNaN(e.meters) ? e : { ...e, meters: round5(e.meters * ratio) }));
    return { ...spec, waypoints, rotationTargets, events };
  }

  /** Planned time of x, or Infinity; feasibility is checked whenever it could become the answer. */
  private evaluate(x: readonly number[]): number {
    this.evaluations++;
    const spec = this.candidate(x);
    let time: number;
    try {
      time = Path.build(spec, this.input.robot).totalTimeSeconds;
    } catch {
      return Infinity;
    }
    if (time < this.bestTime - 1e-9) {
      const f = this.feasible(spec);
      if (!f.ok) return Infinity;
      this.bestTime = f.time;
      this.bestX = x.slice();
      this.bestSpec = spec;
    }
    return time;
  }

  private feasible(spec: PathSpec): Feasibility {
    const a = analyse({ ...this.input, spec });
    if (!a.ok) return { ok: false, time: Infinity };
    const clearanceNeeded = Math.min(CHECKS.bumperMarginMeters, this.baseClearance) - 1e-9;
    const heading = a.checks.filter((c) => c.kind === "heading").length;
    const errors = a.checks.filter((c) => c.severity === "error").length;
    const ok = a.minClearanceMeters >= clearanceNeeded && heading <= this.baseHeading && errors <= this.baseErrors;
    return { ok, time: a.totalTimeSeconds };
  }

  /** A fresh simplex around the best point, along seeded random signs. */
  private restart(): void {
    const x0 = this.bestX;
    this.simplex = [x0.slice()];
    this.values = [this.evaluate(x0)];
    for (let i = 0; i < this.dims; i++) {
      const x = x0.slice();
      x[i] = x[i]! + (this.rand() < 0.5 ? -1 : 1) * this.step * (0.75 + 0.5 * this.rand());
      this.simplex.push(x);
      this.values.push(this.evaluate(x));
    }
  }

  /** One Nelder-Mead iteration (or a restart when the simplex has collapsed). */
  iterate(): void {
    if (this.isConverged) return;
    const n = this.dims;
    const order = this.values.map((_, i) => i).sort((a, b) => this.values[a]! - this.values[b]!);
    this.simplex = order.map((i) => this.simplex[i]!);
    this.values = order.map((i) => this.values[i]!);
    const size = Math.max(...this.simplex.slice(1).map((x) => Math.max(...x.map((v, i) => Math.abs(v - this.simplex[0]![i]!)))));
    if (size < 2e-3 || this.stalls > 20 * n) {
      this.step /= 2;
      this.stalls = 0;
      if (this.step < 0.01) {
        this.converged = true;
        return;
      }
      this.restart();
      return;
    }
    const alpha = 1;
    const gamma = 1 + 2 / n;
    const rho = 0.75 - 1 / (2 * n);
    const sigma = 1 - 1 / n;
    const centroid = new Array(n).fill(0);
    for (let i = 0; i < n; i++) for (let d = 0; d < n; d++) centroid[d] += this.simplex[i]![d]! / n;
    const worst = this.simplex[n]!;
    const along = (t: number) => centroid.map((c, d) => c + t * (c - worst[d]!));
    const bestBefore = this.values[0]!;
    const xr = along(alpha);
    const fr = this.evaluate(xr);
    if (fr < this.values[0]!) {
      const xe = along(gamma);
      const fe = this.evaluate(xe);
      [this.simplex[n], this.values[n]] = fe < fr ? [xe, fe] : [xr, fr];
    } else if (fr < this.values[n - 1]!) {
      this.simplex[n] = xr;
      this.values[n] = fr;
    } else {
      const outside = fr < this.values[n]!;
      const xc = along(outside ? rho : -rho);
      const fc = this.evaluate(xc);
      if (fc < Math.min(fr, this.values[n]!)) {
        this.simplex[n] = xc;
        this.values[n] = fc;
      } else {
        for (let i = 1; i <= n; i++) {
          this.simplex[i] = this.simplex[i]!.map((v, d) => this.simplex[0]![d]! + sigma * (v - this.simplex[0]![d]!));
          this.values[i] = this.evaluate(this.simplex[i]!);
        }
      }
    }
    this.stalls = Math.min(...this.values) < bestBefore - 1e-6 ? 0 : this.stalls + 1;
  }

  /** Iterates until a budget runs out or shouldStop() says so; returns why it stopped. */
  run(evaluationLimit: number, deadline: number, shouldStop: () => boolean = () => false): OptimiseResult["stoppedBy"] | null {
    while (!this.isConverged) {
      if (this.evaluations >= this.options.maxEvaluations) return "evaluations";
      if (performance.now() >= deadline) return "time";
      if (shouldStop()) return "cancelled";
      if (this.evaluations >= evaluationLimit) return null;
      this.iterate();
    }
    return "converged";
  }

  result(stoppedBy: OptimiseResult["stoppedBy"]): OptimiseResult {
    const improved = this.bestTime < this.beforeSeconds - 1e-9;
    return {
      spec: improved ? this.bestSpec : this.input.spec,
      beforeSeconds: this.beforeSeconds,
      afterSeconds: improved ? this.bestTime : this.beforeSeconds,
      evaluations: this.evaluations,
      stoppedBy,
      free: this.free,
    };
  }
}

/** Runs to completion in one go (tests, and the worker's chunks use the class directly). */
export function optimise(input: AnalysisInput, options: Partial<OptimiseOptions> = {}): OptimiseResult {
  const opts = { ...DEFAULT_OPTIONS, ...options };
  const o = new Optimiser(input, opts);
  const deadline = performance.now() + opts.timeBudgetMs;
  const stopped = o.run(Infinity, deadline) ?? "evaluations";
  return o.result(stopped);
}
