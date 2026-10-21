// The path optimiser: shortens a path's planned time by reshaping its curve. By default ("curves and
// speeds") only the tangents change, direction and scale, at every waypoint including the ends; the
// waypoints and their headings stay exactly where they are, and the speed plan follows the new curve
// (it slows where the curve is tighter). Opt in with moveWaypoints and the unlocked interior waypoints
// may also move, each within a corridor around where it was, keeping their headings. Every plan comes
// from the core. It has no editor state, so it runs in the browser's worker and headless under Node
// (tools/optimise.mjs).
//
// Method: Nelder-Mead (adaptive coefficients for the dimension) on normalised coordinates, restarted
// around the best point with a smaller simplex when it stalls. The start simplex's directions come
// from a seeded generator, so a run is deterministic for a seed and an evaluation budget. A budget in
// seconds is turned into evaluations (EVALUATIONS_PER_SECOND), so the same budget gives the same path on
// any machine; the wall clock is only a cap, for a slow machine.
//
// What stays put:
//   - every heading, and every waypoint unless moveWaypoints is on;
//   - with it on, still the first and last waypoints (the start and end poses), any locked, any linked
//     to a named point, any outside the segment being optimised, and any that a zone starts or ends on
//     or between (zone anchors). A waypoint with a set heading may move; its heading goes with it;
//   - tangents outside the segment (its two ends are in it);
//   - events, rotation targets and zones by waypoint position keep that position (the same fraction of
//     their stretch); those by metres keep the same fraction of the path's length.
// A candidate only counts if it plans, its swept bumpers clear every obstacle and the field wall by the
// margin (5 cm), and it has no more heading warnings or errors than the path had. That is a hard rule:
// a path that starts in collision is first pushed out (the search minimises time plus a large weight
// times the clearance it lacks along the swept bumpers) and then made faster among clear paths. If no
// clear path is found, the result says what blocks it and where, suggests a waypoint that would help,
// and changes nothing. It never adds waypoints, and only moves them when asked to.

import {
  makeWaypoint,
  Path,
  PathGeometry,
  tangentRadians,
  type PathEvent,
  type PathSpec,
  type RotationTarget,
} from "../../../core/src/index.ts";
import { round5 } from "../model/positions.ts";
import { analyse, CHECKS, type AnalysisInput } from "./analysis.ts";
import { bumperCorners, convexHull, insideRectMargin, signedClearance, type Pt } from "./polygons.ts";

export interface OptimiseOptions {
  /** Also move the unlocked interior waypoints (off: only the curve's tangents change). */
  readonly moveWaypoints: boolean;
  /** How far each waypoint may move, metres (with moveWaypoints). */
  readonly corridorMeters: number;
  /** Per waypoint: true = never move it (locked, or linked to a named point). */
  readonly locked: readonly boolean[];
  /** Only change the path between these two waypoints (null = the whole path). */
  readonly segment: readonly [number, number] | null;
  /** Also change the tangent scales (within half to double), not just their directions. */
  readonly optimiseScales: boolean;
  readonly seed: number;
  /** The deterministic budget. */
  readonly maxEvaluations: number;
  /** Wall-clock cut, milliseconds (Infinity in tests). */
  readonly timeBudgetMs: number;
}

/**
 * Evaluations per second of budget: under what one evaluation costs in a browser on a short path
 * (about 1-2 ms), so the wall-clock cap rarely cuts a run short.
 */
export const EVALUATIONS_PER_SECOND = 500;

export const DEFAULT_BUDGET_SECONDS = 3;

/** The deterministic evaluation budget for a budget in seconds. */
export function evaluationsFor(budgetSeconds: number): number {
  return Math.max(1, Math.round(budgetSeconds * EVALUATIONS_PER_SECOND));
}

export const DEFAULT_OPTIONS: OptimiseOptions = {
  moveWaypoints: false,
  corridorMeters: 0.5,
  locked: [],
  segment: null,
  optimiseScales: true,
  seed: 1,
  maxEvaluations: evaluationsFor(DEFAULT_BUDGET_SECONDS),
  timeBudgetMs: Infinity,
};

export interface OptimiseResult {
  readonly spec: PathSpec;
  readonly beforeSeconds: number;
  readonly afterSeconds: number;
  readonly evaluations: number;
  readonly stoppedBy: "evaluations" | "time" | "converged" | "cancelled";
  /** Indices of the waypoints it was allowed to move. */
  readonly free: readonly number[];
  /** Indices of the waypoints whose tangents it was allowed to change. */
  readonly turned: readonly number[];
  /** The spec is a new one (faster, or pushed clear of the obstacles it hit). */
  readonly changed: boolean;
  /** The result clears every obstacle and the wall by the margin (false: nothing found; spec unchanged). */
  readonly clear: boolean;
  /** What the path couldn't be pushed clear of, when not clear. */
  readonly blockedBy: string | null;
  /** Where, when not clear: the closest-to-clear try's worst point, and a waypoint that would help. */
  readonly blockedAt: BlockedAt | null;
  /** The result's least clearance from obstacles and the wall, m (negative: overlapping). */
  readonly clearanceMeters: number;
}

export interface BlockedAt {
  /** The worst point: metres along the path and where it is. */
  readonly s: number;
  readonly x: number;
  readonly y: number;
  /** It lies between this waypoint and the next. */
  readonly fromWaypoint: number;
  /** A waypoint there, pushed away from what it hits by the clearance it lacks. */
  readonly suggestX: number;
  readonly suggestY: number;
}

/** How far a tangent may turn from where it was, radians. */
const MAX_TURN = Math.PI / 3;
const DEG = Math.PI / 180;

/** Weight on the clearance a colliding path lacks (s per m·m of shortfall along the path). */
const SHORTFALL_WEIGHT = 100;
/** Where the clearance is sampled along the path, m. */
const CLEARANCE_STEP = 0.05;

export interface Hull {
  readonly name: string;
  readonly hull: Pt[];
  /** Bounding box: min x, min y, max x, max y. */
  readonly box: readonly [number, number, number, number];
}

function box(points: readonly Pt[]): [number, number, number, number] {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const [x, y] of points) {
    x0 = Math.min(x0, x);
    y0 = Math.min(y0, y);
    x1 = Math.max(x1, x);
    y1 = Math.max(y1, y);
  }
  return [x0, y0, x1, y1];
}

/** The obstacles as convex hulls with their boxes (degenerate ones dropped). */
export function hullsOf(obstacles: AnalysisInput["obstacles"]): Hull[] {
  return obstacles
    .map((o) => ({ name: o.name, hull: convexHull(o.polygon) }))
    .filter((o) => o.hull.length >= 3)
    .map((o) => ({ ...o, box: box(o.hull) }));
}

export interface Shortfall {
  /** The clearance short of the margin, integrated along the path, m·m (0: clear everywhere). */
  readonly total: number;
  /** The least clearance anywhere, m (negative: overlapping; exact below the margin, a lower bound above it). */
  readonly least: number;
  /** What the least clearance is from. */
  readonly worst: string;
  /** Where (metres along the path, and the robot's centre), and which way is away from it. */
  readonly s: number;
  readonly x: number;
  readonly y: number;
  readonly away: readonly [number, number];
}

/** The unit direction away from what a body at (x, y) is closest to (the wall: straight in from it). */
function awayFrom(x: number, y: number, what: Hull | null, halfLength: number, halfWidth: number): [number, number] {
  if (what) {
    let cx = 0;
    let cy = 0;
    for (const [px, py] of what.hull) {
      cx += px / what.hull.length;
      cy += py / what.hull.length;
    }
    const d = Math.hypot(x - cx, y - cy);
    return d > 1e-9 ? [(x - cx) / d, (y - cy) / d] : [0, 1];
  }
  const gaps: [number, [number, number]][] = [
    [halfLength - x, [-1, 0]],
    [x + halfLength, [1, 0]],
    [halfWidth - y, [0, -1]],
    [y + halfWidth, [0, 1]],
  ];
  return gaps.reduce((a, b) => (b[0] < a[0] ? b : a))[1];
}

/** How far the swept bumpers fall short of the margin from each obstacle and the field wall. */
export function shortfall(path: Path, input: AnalysisInput, hulls: readonly Hull[], margin: number): Shortfall {
  const L = path.lengthMeters;
  const n = Math.max(1, Math.ceil(L / CLEARANCE_STEP));
  let total = 0;
  let least = Infinity;
  let worst = "";
  let worstHull: Hull | null = null;
  let at = { s: 0, x: 0, y: 0 };
  for (let i = 0; i <= n; i++) {
    const p = path.sample((L * i) / n);
    const body = bumperCorners(p.xMeters, p.yMeters, p.headingRadians, input.bumperLengthMeters, input.bumperWidthMeters);
    let here = insideRectMargin(body, input.fieldLengthMeters / 2, input.fieldWidthMeters / 2);
    let what = "the field wall";
    let whatHull: Hull | null = null;
    const [x0, y0, x1, y1] = box(body);
    for (const o of hulls) {
      // boxes apart by the margin: the polygons are too (only clearance under the margin counts)
      const gap = Math.max(o.box[0] - x1, x0 - o.box[2], o.box[1] - y1, y0 - o.box[3]);
      const c = gap >= margin ? gap : signedClearance(body, o.hull);
      if (c < here) {
        here = c;
        what = o.name;
        whatHull = o;
      }
    }
    if (here < least) {
      least = here;
      worst = what;
      worstHull = whatHull;
      at = { s: (L * i) / n, x: p.xMeters, y: p.yMeters };
    }
    if (here < margin) total += (margin - here) * (L / n);
  }
  const away = awayFrom(at.x, at.y, worstHull, input.fieldLengthMeters / 2, input.fieldWidthMeters / 2);
  return { total, least, worst, ...at, away };
}

/** Why each waypoint stays put when moving waypoints is on (null = the optimiser may move it). */
export function whyFixed(
  spec: PathSpec,
  locked: readonly boolean[],
  segment: readonly [number, number] | null = null,
): (string | null)[] {
  const n = spec.waypoints.length;
  const why: (string | null)[] = spec.waypoints.map((_, i) =>
    i === 0 || i === n - 1
      ? "the start and end poses stay put"
      : locked[i] === true
        ? "locked"
        : segment && (i <= Math.min(...segment) || i >= Math.max(...segment))
          ? "outside the segment"
          : null,
  );
  const anchor = (p: number) => {
    for (const i of [Math.floor(p), Math.ceil(p)]) if (i >= 0 && i < n) why[i] ??= "a zone starts or ends here";
  };
  for (const z of [...spec.facePointZones, ...spec.constraintZones]) {
    anchor(z.fromWaypoint);
    anchor(z.toWaypoint);
  }
  return why;
}

/** The waypoints that must stay put when moving waypoints is on (see whyFixed). */
export function fixedWaypoints(spec: PathSpec, locked: readonly boolean[], segment: readonly [number, number] | null = null): boolean[] {
  return whyFixed(spec, locked, segment).map((w) => w !== null);
}

/** The waypoints whose tangents may change: all of them, or the segment's (its ends included). */
export function turnableWaypoints(spec: PathSpec, segment: readonly [number, number] | null = null): number[] {
  const n = spec.waypoints.length;
  if (n < 2) return [];
  const [a, b] = segment ? [Math.max(0, Math.min(...segment)), Math.min(n - 1, Math.max(...segment))] : [0, n - 1];
  return spec.waypoints.map((_, i) => i).filter((i) => i >= a && i <= b);
}

/** Says what the optimiser will change, for the button's tooltip. */
export function whatMoves(spec: PathSpec, options: Pick<OptimiseOptions, "moveWaypoints" | "locked" | "segment" | "corridorMeters">): string {
  const t = turnableWaypoints(spec, options.segment).length;
  const curves = `Reshape the curve at ${t} waypoint${t === 1 ? "" : "s"} (tangent directions and scales; the speeds follow)`;
  if (!options.moveWaypoints) return `${curves}. Waypoints and headings stay where they are.`;
  const m = fixedWaypoints(spec, options.locked, options.segment).filter((f) => !f).length;
  return `${curves}, and move ${m} unlocked waypoint${m === 1 ? "" : "s"} within ${options.corridorMeters} m, keeping their headings.`;
}

/** The plain-words reason a result isn't clear, and what to do about it. */
export function blockedMessage(
  r: Pick<OptimiseResult, "blockedBy" | "blockedAt">,
  options: Pick<OptimiseOptions, "moveWaypoints" | "corridorMeters">,
): string {
  const name = r.blockedBy ?? "the obstacles";
  const what = name === "the field wall" ? "the wall" : /^(the |obstacle )/.test(name) ? name : `the ${name}`;
  const where = r.blockedAt ? ` between waypoints ${r.blockedAt.fromWaypoint} and ${r.blockedAt.fromWaypoint + 1}` : "";
  return options.moveWaypoints
    ? `Can't clear ${what}${where} within ${options.corridorMeters} m: widen the corridor, add a waypoint, or use Route around obstacles.`
    : `Can't clear ${what} by reshaping the curve${where}: add a waypoint or move one.`;
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
  readonly clearance: number;
}

export class Optimiser {
  readonly free: readonly number[];
  readonly turned: readonly number[];
  /** Each waypoint's tangent direction as planned now (given, or the automatic one). */
  private readonly baseTangents: readonly number[];
  readonly beforeSeconds: number;
  private readonly dims: number;
  private readonly baseClearance: number;
  private readonly margin = CHECKS.bumperMarginMeters;
  private readonly hulls: Hull[];
  /** The path starts too close to (or in) an obstacle: first push it clear. */
  private readonly clearMode: boolean;
  private cleared = false;
  private leastShortfall = Infinity;
  private leastX: number[];
  private leastWhere: Shortfall | null = null;
  private leastSpec: PathSpec | null = null;
  private blockedBy = "";
  private bestClearance: number;
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

  private readonly input: AnalysisInput;
  private readonly options: OptimiseOptions;

  constructor(input: AnalysisInput, options: OptimiseOptions) {
    this.input = input;
    this.options = options;
    const spec = input.spec;
    const fixed = fixedWaypoints(spec, options.locked, options.segment);
    this.free = options.moveWaypoints ? spec.waypoints.map((_, i) => i).filter((i) => !fixed[i]) : [];
    let g: PathGeometry | null = null;
    try {
      g = PathGeometry.build(spec.waypoints);
    } catch {
      g = null; // a turn on the spot: no curve to shape
    }
    this.turned = g ? turnableWaypoints(spec, options.segment) : [];
    this.baseTangents = spec.waypoints.map((w, i) =>
      Number.isNaN(w.tangentRadians) && g ? tangentRadians(g.sample(g.sAtWaypoint(i))) : w.tangentRadians,
    );
    this.dims = this.free.length * 2 + this.turned.length * (options.optimiseScales ? 2 : 1);
    const base = analyse(input);
    if (!base.ok) throw new Error(`the path can't be planned: ${base.error}`);
    this.beforeSeconds = base.totalTimeSeconds;
    this.baseClearance = base.minClearanceMeters;
    this.hulls = hullsOf(input.obstacles);
    this.clearMode = base.minClearanceMeters < this.margin - 1e-9;
    this.bestClearance = base.minClearanceMeters;
    this.baseHeading = base.checks.filter((c) => c.kind === "heading").length;
    this.baseErrors = base.checks.filter((c) => c.severity === "error" && c.kind !== "bumper").length;
    this.baseLength = base.lengthMeters;
    this.rand = random(options.seed);
    this.bestX = new Array(this.dims).fill(0);
    this.leastX = this.bestX;
    // a colliding path has no time to beat yet: any clear one is better
    this.bestTime = this.clearMode ? Infinity : this.beforeSeconds;
    this.bestSpec = spec;
    if (this.clearMode) {
      const s0 = shortfall(Path.build(spec, input.robot), input, this.hulls, this.margin);
      this.blockedBy = s0.worst;
      this.leastWhere = s0;
    }
    if (this.dims > 0) this.restart();
  }

  get evaluationCount(): number {
    return this.evaluations;
  }

  get bestSeconds(): number {
    return Number.isFinite(this.bestTime) ? this.bestTime : this.beforeSeconds;
  }

  get best(): PathSpec {
    return this.bestSpec;
  }

  get isConverged(): boolean {
    return this.converged || this.dims === 0;
  }

  /**
   * The spec for normalised coordinates x (rounded to what the file holds): first each free waypoint's
   * offset in the corridor, then each turned waypoint's tangent turn (and scale).
   */
  candidate(x: readonly number[]): PathSpec {
    const spec = this.input.spec;
    const r = this.options.corridorMeters;
    const waypoints = spec.waypoints.slice();
    this.free.forEach((wi, j) => {
      let u = x[2 * j]!;
      let v = x[2 * j + 1]!;
      const len = Math.hypot(u, v);
      if (len > 1) {
        u /= len;
        v /= len;
      }
      const w = spec.waypoints[wi]!;
      waypoints[wi] = makeWaypoint(round5(w.xMeters + r * u), round5(w.yMeters + r * v), w.tangentRadians, w.tangentScale, w.headingRadians);
    });
    const k = this.options.optimiseScales ? 2 : 1;
    const at = 2 * this.free.length;
    this.turned.forEach((wi, j) => {
      const w = waypoints[wi]!;
      const turn = Math.max(-1, Math.min(1, x[at + k * j]!)) * MAX_TURN;
      // to 0.01 degrees; an automatic tangent that hasn't turned stays automatic
      const degrees = Math.round(((this.baseTangents[wi]! + turn) / DEG) * 100) / 100;
      const wrapped = degrees > 180 ? degrees - 360 : degrees <= -180 ? degrees + 360 : degrees;
      const tangent = Math.abs(turn) < 0.005 * DEG ? w.tangentRadians : wrapped * DEG;
      const scale = this.options.optimiseScales
        ? Math.round(w.tangentScale * Math.pow(2, Math.max(-1, Math.min(1, x[at + k * j + 1]!))) * 100) / 100
        : w.tangentScale;
      waypoints[wi] = makeWaypoint(w.xMeters, w.yMeters, tangent, scale, w.headingRadians);
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

  /**
   * The score of x: its planned time, or Infinity; feasibility is checked whenever it could become the
   * answer. While no clear path is known, the time plus the weighted clearance shortfall, so the search
   * heads out of the obstacles first.
   */
  private evaluate(x: readonly number[]): number {
    this.evaluations++;
    const spec = this.candidate(x);
    let path: Path;
    try {
      path = Path.build(spec, this.input.robot);
    } catch {
      return Infinity;
    }
    const time = path.totalTimeSeconds;
    if (this.clearMode && !this.cleared) {
      const sf = shortfall(path, this.input, this.hulls, this.margin);
      if (sf.total < this.leastShortfall) {
        this.leastShortfall = sf.total;
        this.leastX = x.slice();
        this.blockedBy = sf.worst;
        this.leastWhere = sf;
        this.leastSpec = spec;
      }
      if (sf.total > 0) return time + SHORTFALL_WEIGHT * sf.total;
    }
    if (time < this.bestTime - 1e-9) {
      const f = this.feasible(spec);
      if (!f.ok) return this.clearMode && !this.cleared ? time + SHORTFALL_WEIGHT * 1e-3 : Infinity;
      this.bestTime = f.time;
      this.bestX = x.slice();
      this.bestSpec = spec;
      this.bestClearance = f.clearance;
      this.cleared = true;
    }
    return time;
  }

  private feasible(spec: PathSpec): Feasibility {
    const a = analyse({ ...this.input, spec });
    if (!a.ok) return { ok: false, time: Infinity, clearance: Number.NaN };
    // clear by the margin is a hard rule, whatever the path started with
    const heading = a.checks.filter((c) => c.kind === "heading").length;
    const errors = a.checks.filter((c) => c.severity === "error" && c.kind !== "bumper").length;
    const ok = a.minClearanceMeters >= this.margin - 1e-9 && heading <= this.baseHeading && errors <= this.baseErrors;
    return { ok, time: a.totalTimeSeconds, clearance: a.minClearanceMeters };
  }

  /** A fresh simplex around the best point, along seeded random signs. */
  private restart(): void {
    // around the best clear point, or while none is known, the one closest to clear
    const x0 = this.clearMode && !this.cleared ? this.leastX : this.bestX;
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

  /** Where the closest-to-clear try still hits, and a waypoint there that would help. */
  private where(): BlockedAt | null {
    const w = this.leastWhere;
    if (!w) return null;
    const spec = this.leastSpec ?? this.input.spec;
    let fromWaypoint = 0;
    try {
      const g = PathGeometry.build(spec.waypoints);
      for (let i = 0; i < spec.waypoints.length - 1; i++) if (g.sAtWaypoint(i) <= w.s) fromWaypoint = i;
    } catch {
      fromWaypoint = 0;
    }
    const push = Math.max(0, this.margin - w.least) + 0.1;
    return {
      s: w.s,
      x: round5(w.x),
      y: round5(w.y),
      fromWaypoint,
      suggestX: round5(w.x + push * w.away[0]),
      suggestY: round5(w.y + push * w.away[1]),
    };
  }

  result(stoppedBy: OptimiseResult["stoppedBy"]): OptimiseResult {
    // a colliding path: any clear result is the answer, even a slower one; none found changes nothing
    const improved = this.clearMode ? this.cleared : this.bestTime < this.beforeSeconds - 1e-9;
    const clear = !this.clearMode || this.cleared;
    return {
      spec: improved ? this.bestSpec : this.input.spec,
      beforeSeconds: this.beforeSeconds,
      afterSeconds: improved ? this.bestTime : this.beforeSeconds,
      evaluations: this.evaluations,
      stoppedBy,
      free: this.free,
      turned: this.turned,
      changed: improved,
      clear,
      blockedBy: clear ? null : this.blockedBy || null,
      blockedAt: clear ? null : this.where(),
      clearanceMeters: improved ? this.bestClearance : this.baseClearance,
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
