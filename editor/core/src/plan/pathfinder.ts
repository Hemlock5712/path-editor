// The pathfinder: a port of lib's plan/Pathfinder.java, FieldMap and PlannerConfig, line for line, so a route
// planned in the editor ("Route around obstacles") is the route the robot plans. The Java writes the fixtures
// (fixtures/plan, PlanFixtureGen); this must reproduce them to 1e-9.
//
// At construction: every obstacle is grown by the bumpers' half-diagonal plus a margin, the walls are moved in
// by the same, and the visibility graph between the grown corners is built. Per plan: snap the start and goal
// out of grown obstacles, connect them, A*, fit the core's spline through the route's corners, sweep the
// oriented bumper rectangle along it with the planned heading, add route points where the margin is broken and
// refit, then build the Path (profile from the robot's speed to the end speed).

import type { FieldSpeeds, Pose } from "../frame.ts";
import { PathGeometry } from "../geometry.ts";
import { HeadingProfile } from "../heading.ts";
import { javaHypot } from "../jmath.ts";
import { Path, pureRotation } from "../path.ts";
import type { RobotConfig } from "../robot.ts";
import { makePathSpec, makeRotationTarget, makeWaypoint, type PathSpec } from "../spec.ts";
import { ConvexPolygon, EPS } from "./polygon.ts";

export interface PlannerConfig {
  readonly bumperLengthMeters: number;
  readonly bumperWidthMeters: number;
  readonly marginMeters: number;
  readonly arcStepRadians: number;
  readonly maxRefits: number;
  readonly checkStepMeters: number;
  readonly maxMetersPerSecond: number;
  readonly maxMetersPerSecondSquared: number;
}

/** 0.85 m square bumpers, 5 cm margin, 30° arcs, 8 refits, 2 cm checks, 4 m/s and 6 m/s². */
export const DEFAULT_PLANNER: PlannerConfig = {
  bumperLengthMeters: 0.85,
  bumperWidthMeters: 0.85,
  marginMeters: 0.05,
  arcStepRadians: (30 * Math.PI) / 180,
  maxRefits: 8,
  checkStepMeters: 0.02,
  maxMetersPerSecond: 4.0,
  maxMetersPerSecondSquared: 6.0,
};

export function halfDiagonalMeters(c: PlannerConfig): number {
  return javaHypot(c.bumperLengthMeters, c.bumperWidthMeters) / 2;
}

export function growMeters(c: PlannerConfig): number {
  return halfDiagonalMeters(c) + c.marginMeters;
}

/** The static field: size and obstacles (each replaced by its convex hull), centre origin. */
export interface FieldMap {
  readonly lengthMeters: number;
  readonly widthMeters: number;
  readonly names: readonly string[];
  readonly obstacles: readonly ConvexPolygon[];
}

/** field.json's tree (the editor's format: {field: {length_m, width_m}, obstacles: [{name, polygon}]}). */
export function fieldFromJson(root: {
  field: { length_m: number; width_m: number };
  obstacles?: readonly { name?: string; polygon: readonly (readonly number[])[] }[];
}): FieldMap {
  const names: string[] = [];
  const obstacles: ConvexPolygon[] = [];
  for (const o of root.obstacles ?? []) {
    obstacles.push(ConvexPolygon.of(o.polygon.map((p) => [p[0]!, p[1]!] as const)));
    names.push(o.name ?? `obstacle ${obstacles.length}`);
  }
  if (!(root.field.length_m > 0) || !(root.field.width_m > 0)) throw new RangeError("field size must be positive");
  return { lengthMeters: root.field.length_m, widthMeters: root.field.width_m, names, obstacles };
}

export interface PlanResult {
  readonly path: Path | null;
  readonly waypoints: readonly (readonly [number, number])[];
  readonly routeLengthMeters: number;
  readonly straightLineMeters: number;
  readonly startSnapMeters: number;
  readonly goalSnapMeters: number;
  readonly refits: number;
  readonly clear: boolean;
  readonly minClearanceMeters: number;
  readonly failure: string | null;
}

type Xy = [number, number];

/** A straight approach this long (plus the braking distance from the end speed) ends a route with a set direction. */
export const APPROACH_METERS = 0.5;

export class Pathfinder {
  readonly field: FieldMap;
  readonly robot: RobotConfig;
  readonly config: PlannerConfig;
  /** The grown obstacles (what the robot centre must stay out of). */
  readonly grown: readonly ConvexPolygon[];
  private readonly xMin: number;
  private readonly xMax: number;
  private readonly yMin: number;
  private readonly yMax: number;
  private readonly nodeX: Float64Array;
  private readonly nodeY: Float64Array;
  private readonly edge: Float64Array[];

  constructor(field: FieldMap, robot: RobotConfig, config: PlannerConfig = DEFAULT_PLANNER) {
    this.field = field;
    this.robot = robot;
    this.config = config;
    const r = growMeters(config);
    this.grown = field.obstacles.map((p) => p.grow(r, config.arcStepRadians));
    this.xMin = -field.lengthMeters / 2 + r;
    this.xMax = field.lengthMeters / 2 - r;
    this.yMin = -field.widthMeters / 2 + r;
    this.yMax = field.widthMeters / 2 - r;
    const nodes: Xy[] = [];
    for (const p of this.grown) {
      for (let i = 0; i < p.size; i++) if (this.isFree(p.xs[i]!, p.ys[i]!)) nodes.push([p.xs[i]!, p.ys[i]!]);
    }
    const n = nodes.length;
    this.nodeX = new Float64Array(n);
    this.nodeY = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      this.nodeX[i] = nodes[i]![0];
      this.nodeY[i] = nodes[i]![1];
    }
    this.edge = [];
    for (let i = 0; i < n; i++) this.edge.push(new Float64Array(n).fill(Infinity));
    for (let i = 0; i < n; i++) {
      this.edge[i]![i] = 0;
      for (let j = i + 1; j < n; j++) {
        if (this.visible(this.nodeX[i]!, this.nodeY[i]!, this.nodeX[j]!, this.nodeY[j]!)) {
          const d = javaHypot(this.nodeX[j]! - this.nodeX[i]!, this.nodeY[j]! - this.nodeY[i]!);
          this.edge[i]![j] = d;
          this.edge[j]![i] = d;
        }
      }
    }
  }

  get nodeCount(): number {
    return this.nodeX.length;
  }

  /** True when a robot centre at (x, y) is outside every grown obstacle and inside the moved-in walls. */
  isFree(x: number, y: number): boolean {
    if (x < this.xMin - EPS || x > this.xMax + EPS || y < this.yMin - EPS || y > this.yMax + EPS) return false;
    for (const p of this.grown) if (p.containsStrictly(x, y)) return false;
    return true;
  }

  /** True when the straight segment crosses no grown obstacle. */
  visible(x0: number, y0: number, x1: number, y1: number): boolean {
    for (const p of this.grown) if (p.blocksSegment(x0, y0, x1, y1)) return false;
    return true;
  }

  /**
   * Plans from the robot's pose and field velocity to the goal, arriving at endSpeed travelling in direction
   * endTangent (NaN = whatever the route gives; see Pathfinder.plan in Java).
   */
  plan(start: Pose, startVelocity: FieldSpeeds, goal: Pose, endSpeed = 0.0, endTangent = Number.NaN): PlanResult {
    const cfg = this.config;
    const s = this.snap(start.xMeters, start.yMeters);
    const g = this.snap(goal.xMeters, goal.yMeters);
    const startSnap = javaHypot(s[0] - start.xMeters, s[1] - start.yMeters);
    const goalSnap = javaHypot(g[0] - goal.xMeters, g[1] - goal.yMeters);
    const arrive = Math.min(Math.max(0.0, endSpeed), cfg.maxMetersPerSecond);
    let route: Xy[] | null = null;
    let approach = false;
    if (Number.isFinite(endTangent) && goalSnap <= 1e-9) {
      const len = APPROACH_METERS + (arrive * arrive) / (2 * cfg.maxMetersPerSecondSquared);
      const a: Xy = [g[0] - len * Math.cos(endTangent), g[1] - len * Math.sin(endTangent)];
      if (javaHypot(g[0] - s[0], g[1] - s[1]) > 2 * len && this.isFree(a[0], a[1]) && this.visible(a[0], a[1], g[0], g[1])) {
        const head = this.shortestRoute(s, a);
        if (head !== null) {
          route = [...head, g];
          approach = true;
        }
      }
    }
    if (route === null) route = this.shortestRoute(s, g);
    const endDirection = approach ? endTangent : Number.NaN;
    const straight = javaHypot(goal.xMeters - start.xMeters, goal.yMeters - start.yMeters);
    if (route === null) {
      return {
        path: null, waypoints: [], routeLengthMeters: Number.NaN, straightLineMeters: straight, startSnapMeters: startSnap,
        goalSnapMeters: goalSnap, refits: 0, clear: false, minClearanceMeters: Number.NaN,
        failure: "no route between the start and the goal",
      };
    }
    let pts: Xy[] = [];
    if (startSnap > 1e-9) pts.push([start.xMeters, start.yMeters]);
    pts.push(...route);
    if (goalSnap > 1e-9) pts.push([goal.xMeters, goal.yMeters]);
    pts = dedupe(pts);
    let routeLength = 0;
    for (let i = 1; i < pts.length; i++) routeLength += javaHypot(pts[i]![0] - pts[i - 1]![0], pts[i]![1] - pts[i - 1]![1]);

    const v = javaHypot(startVelocity.vxMetersPerSecond, startVelocity.vyMetersPerSecond);
    const moving = v > 0.1;
    const startTangent = moving ? Math.atan2(startVelocity.vyMetersPerSecond, startVelocity.vxMetersPerSecond) : Number.NaN;
    const startSpeed = moving ? Math.min(v, cfg.maxMetersPerSecond) : 0.0;

    let refits = 0;
    let sweep: Sweep;
    let spec: PathSpec;
    for (;;) {
      spec = pureRotation(this.spec(pts, startTangent, startSpeed, start.headingRadians, goal.headingRadians, endDirection, arrive));
      const geometry = PathGeometry.build(spec.waypoints);
      const heading = HeadingProfile.build(geometry, spec.rotationTargets, spec.facePointZones);
      const skipStart = startSnap > 1e-9 ? geometry.waypointS(1) : 0.0;
      const skipEnd = goalSnap > 1e-9 ? geometry.waypointS(pts.length - 2) : geometry.lengthMeters;
      sweep = this.sweep(geometry, heading, skipStart, skipEnd);
      if (sweep.violations.length === 0 || refits >= cfg.maxRefits) break;
      refits++;
      const inserts: [number, number, number][] = [];
      for (const sv of sweep.violations) {
        const seg = segmentAt(geometry, sv, pts.length);
        const a = pts[seg]!;
        const b = pts[seg + 1]!;
        const q = geometry.sample(sv);
        const ex = b[0] - a[0];
        const ey = b[1] - a[1];
        const len = javaHypot(ex, ey);
        let t = ((q.xMeters - a[0]) * ex + (q.yMeters - a[1]) * ey) / (len * len);
        const minT = Math.min(0.5, 0.05 / len);
        t = Math.max(minT, Math.min(1 - minT, t));
        inserts.push([seg, a[0] + t * ex, a[1] + t * ey]);
      }
      inserts.sort((x, y) => (y[0] < x[0] ? -1 : y[0] > x[0] ? 1 : 0)); // back to front
      let lastSeg = -1;
      for (const ins of inserts) {
        if (ins[0] === lastSeg) continue;
        lastSeg = ins[0];
        pts.splice(lastSeg + 1, 0, [ins[1], ins[2]]);
      }
    }
    const path = Path.build(spec, this.robot);
    return {
      path, waypoints: pts.map((p) => [p[0], p[1]] as const), routeLengthMeters: routeLength, straightLineMeters: straight,
      startSnapMeters: startSnap, goalSnapMeters: goalSnap, refits, clear: sweep.violations.length === 0,
      minClearanceMeters: sweep.minClearance, failure: null,
    };
  }

  private spec(
    pts: readonly Xy[], startTangent: number, startSpeed: number, startHeading: number, goalHeading: number,
    endTangent: number, endSpeed: number,
  ): PathSpec {
    const wps = pts.map((p, i) =>
      makeWaypoint(p[0], p[1], i === 0 ? startTangent : i === pts.length - 1 ? endTangent : Number.NaN, 1.0));
    return makePathSpec({
      waypoints: wps,
      rotationTargets: [makeRotationTarget(0, Number.NaN, startHeading), makeRotationTarget(pts.length - 1, Number.NaN, goalHeading)],
      maxMetersPerSecond: this.config.maxMetersPerSecond,
      maxMetersPerSecondSquared: this.config.maxMetersPerSecondSquared,
      startMetersPerSecond: startSpeed,
      endMetersPerSecond: endSpeed,
    });
  }

  /** The point itself if free, else the nearest free point on a grown obstacle's boundary or the walls. */
  snap(x: number, y: number): Xy {
    if (this.isFree(x, y)) return [x, y];
    const candidates: Xy[] = [[clamp(x, this.xMin, this.xMax), clamp(y, this.yMin, this.yMax)]];
    for (const p of this.grown) {
      const b = p.nearestBoundaryPoint(x, y);
      const dx = b[0] - p.cx;
      const dy = b[1] - p.cy;
      const d = javaHypot(dx, dy);
      candidates.push([b[0] + (1e-3 * dx) / d, b[1] + (1e-3 * dy) / d]);
    }
    const dist = (c: Xy) => javaHypot(c[0] - x, c[1] - y);
    candidates.sort((a, b) => {
      const da = dist(a);
      const db = dist(b);
      return da < db ? -1 : da > db ? 1 : 0;
    });
    for (const c of candidates) if (this.isFree(c[0], c[1])) return c;
    for (let r = 0.05; r < 8.0; r += 0.05) {
      for (let k = 0; k < 72; k++) {
        const a = (k * Math.PI) / 36;
        const cx = x + r * Math.cos(a);
        const cy = y + r * Math.sin(a);
        if (this.isFree(cx, cy)) return [cx, cy];
      }
    }
    throw new RangeError("no free point on the field");
  }

  /** A* over the static graph plus the start and goal; the route's points, or null. */
  private shortestRoute(s: Xy, g: Xy): Xy[] | null {
    if (this.visible(s[0], s[1], g[0], g[1])) return [s, g];
    const n = this.nodeX.length;
    const start = n;
    const goal = n + 1;
    const toStart = new Float64Array(n);
    const toGoal = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      toStart[i] = this.visible(s[0], s[1], this.nodeX[i]!, this.nodeY[i]!)
        ? javaHypot(this.nodeX[i]! - s[0], this.nodeY[i]! - s[1]) : Infinity;
      toGoal[i] = this.visible(this.nodeX[i]!, this.nodeY[i]!, g[0], g[1])
        ? javaHypot(g[0] - this.nodeX[i]!, g[1] - this.nodeY[i]!) : Infinity;
    }
    const cost = new Float64Array(n + 2).fill(Infinity);
    const parent = new Int32Array(n + 2).fill(-1);
    const closed = new Uint8Array(n + 2);
    cost[start] = 0;
    for (;;) {
      let best = -1;
      let bestF = Infinity;
      for (let i = 0; i < n + 2; i++) {
        if (!closed[i] && cost[i]! < Infinity) {
          const f = cost[i]! + this.heuristic(i, n, s, g);
          if (f < bestF) {
            bestF = f;
            best = i;
          }
        }
      }
      if (best < 0) return null;
      if (best === goal) break;
      closed[best] = 1;
      for (let j = 0; j < n + 2; j++) {
        if (closed[j] || j === best) continue;
        const w = this.weight(best, j, n, toStart, toGoal);
        if (w < Infinity && cost[best]! + w < cost[j]!) {
          cost[j] = cost[best]! + w;
          parent[j] = best;
        }
      }
    }
    const out: Xy[] = [];
    for (let i = goal; i >= 0; i = parent[i]!) {
      out.push(i === start ? s : i === goal ? g : [this.nodeX[i]!, this.nodeY[i]!]);
      if (i === start) break;
    }
    return out.reverse();
  }

  private heuristic(i: number, n: number, s: Xy, g: Xy): number {
    const x = i === n ? s[0] : i === n + 1 ? g[0] : this.nodeX[i]!;
    const y = i === n ? s[1] : i === n + 1 ? g[1] : this.nodeY[i]!;
    return javaHypot(g[0] - x, g[1] - y);
  }

  private weight(a: number, b: number, n: number, toStart: Float64Array, toGoal: Float64Array): number {
    if (a < n && b < n) return this.edge[a]![b]!;
    if (a === n || b === n) {
      const other = a === n ? b : a;
      return other < n ? toStart[other]! : Infinity;
    }
    const other = a === n + 1 ? b : a;
    return other < n ? toGoal[other]! : Infinity;
  }

  private sweep(geometry: PathGeometry, heading: HeadingProfile, skipStart: number, skipEnd: number): Sweep {
    const len = geometry.lengthMeters;
    const n = Math.max(2, Math.ceil(len / this.config.checkStepMeters) + 1);
    const need = this.config.marginMeters;
    const reach = halfDiagonalMeters(this.config);
    let min = Infinity;
    const worst: number[] = [];
    let inBad = false;
    let badMin = 0;
    let badS = 0;
    const qx = new Float64Array(4);
    const qy = new Float64Array(4);
    for (let k = 0; k < n; k++) {
      const s = (len * k) / (n - 1);
      const gs = geometry.sample(s);
      const th = heading.sampleAt(gs).radians;
      const c = this.clearanceWith(gs.xMeters, gs.yMeters, th, reach, qx, qy);
      const counted = s >= skipStart && s <= skipEnd;
      if (counted) min = Math.min(min, c);
      const bad = counted && c < need;
      if (bad) {
        if (!inBad || c < badMin) {
          badMin = c;
          badS = s;
        }
        inBad = true;
      } else if (inBad) {
        worst.push(badS);
        inBad = false;
      }
    }
    if (inBad) worst.push(badS);
    return { minClearance: min, violations: worst };
  }

  /** Clearance of the bumper rectangle at (x, y, th) to every obstacle and the field walls. */
  clearance(x: number, y: number, th: number): number {
    return this.clearanceWith(x, y, th, halfDiagonalMeters(this.config), new Float64Array(4), new Float64Array(4));
  }

  private clearanceWith(x: number, y: number, th: number, reach: number, qx: Float64Array, qy: Float64Array): number {
    const c = Math.cos(th);
    const s = Math.sin(th);
    const hl = this.config.bumperLengthMeters / 2;
    const hw = this.config.bumperWidthMeters / 2;
    const px = [hl, -hl, -hl, hl];
    const py = [hw, hw, -hw, -hw];
    const halfL = this.field.lengthMeters / 2;
    const halfW = this.field.widthMeters / 2;
    let best = Infinity;
    for (let i = 0; i < 4; i++) {
      qx[i] = x + c * px[i]! - s * py[i]!;
      qy[i] = y + s * px[i]! + c * py[i]!;
      best = Math.min(best, Math.min(halfL - Math.abs(qx[i]!), halfW - Math.abs(qy[i]!)));
    }
    for (const p of this.field.obstacles) {
      const centre = javaHypot(x - p.cx, y - p.cy) - p.radius - reach;
      if (centre >= best) continue;
      best = Math.min(best, p.distanceToPolygon(qx, qy));
    }
    return best;
  }
}

interface Sweep {
  readonly minClearance: number;
  readonly violations: readonly number[];
}

function segmentAt(geometry: PathGeometry, s: number, waypoints: number): number {
  for (let i = 1; i < waypoints; i++) if (s <= geometry.waypointS(i)) return i - 1;
  return waypoints - 2;
}

function dedupe(pts: readonly Xy[]): Xy[] {
  const out: Xy[] = [];
  for (const p of pts) {
    const last = out[out.length - 1];
    if (last === undefined || javaHypot(p[0] - last[0], p[1] - last[1]) > 0.01) out.push(p);
  }
  if (out.length === 1) out.push(pts[pts.length - 1]!);
  return out;
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}
