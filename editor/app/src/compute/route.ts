// "Route around obstacles": the robot's own pathfinder (the core's port of lib's Pathfinder, bit for bit)
// plans a way round the first stretch where the swept bumpers hit something, and its corners go into the
// path as unlocked pass-through waypoints. The waypoints either side of the stretch stay where they are,
// and so does any waypoint between them that is locked, linked to a named point or has a heading (the
// route goes through it, one leg at a time); the plain pass-through points in between are what the route
// replaces. It doesn't optimise: the caller runs the curves-only optimiser on the result.

import { DEFAULT_PLANNER, fieldFromJson, Pathfinder, type RobotConfig } from "../../../core/src/index.ts";
import { deleteWaypoint, insertWaypoint } from "../model/edits.ts";
import { round5 } from "../model/positions.ts";
import type { EditorPath, FieldDoc } from "../model/project.ts";
import { CHECKS, type Analysis } from "./analysis.ts";
import { bumperCorners, convexHull, convexOverlap } from "./polygons.ts";

export interface RouteSetup {
  readonly field: FieldDoc;
  readonly robot: RobotConfig;
  readonly bumperLengthMeters: number;
  readonly bumperWidthMeters: number;
}

export type RouteOutcome =
  | {
      readonly ok: true;
      /** The path with the route's corners in (and the pass-through points they replace gone). */
      readonly path: EditorPath;
      readonly added: number;
      readonly removed: number;
      /** The waypoints either side of the stretch it routed round (indices in the original path). */
      readonly from: number;
      readonly to: number;
    }
  | { readonly ok: false; readonly reason: string };

/** The first stretch where the bumpers hit something (or, failing that, come under the margin). */
export function firstCollision(a: Analysis): { sFrom: number; sTo: number } | null {
  const bumper = a.checks.filter((c) => c.kind === "bumper");
  const hit = bumper.find((c) => c.severity === "error") ?? bumper[0];
  return hit ? { sFrom: hit.sFrom, sTo: hit.sTo } : null;
}

/** Plans round the first collision on the path the analysis is for. */
export function routeAround(path: EditorPath, a: Analysis, setup: RouteSetup): RouteOutcome {
  const stretch = firstCollision(a);
  if (!stretch) return { ok: false, reason: "Nothing to route around: the path clears every obstacle." };
  const n = path.spec.waypoints.length;
  const s = a.waypointS;
  let from = 0;
  for (let i = 0; i < n; i++) if (s[i]! <= stretch.sFrom + 1e-9) from = i;
  let to = n - 1;
  for (let i = n - 1; i >= 0; i--) if (s[i]! >= stretch.sTo - 1e-9) to = i;
  if (to <= from) to = Math.min(n - 1, from + 1);
  const kept = (i: number) =>
    i === from || i === to || path.pinned[i] === true || path.links[i] !== null || !Number.isNaN(path.spec.waypoints[i]!.headingRadians);
  const legs: number[] = [];
  for (let i = from; i <= to; i++) if (kept(i)) legs.push(i);

  const spec = path.spec;
  const finder = new Pathfinder(
    fieldFromJson({
      field: { length_m: setup.field.lengthMeters, width_m: setup.field.widthMeters },
      obstacles: setup.field.obstacles.map((o) => ({ name: o.name, polygon: o.polygon.map(([x, y]) => [x, y]) })),
    }),
    setup.robot,
    {
      ...DEFAULT_PLANNER,
      bumperLengthMeters: setup.bumperLengthMeters,
      bumperWidthMeters: setup.bumperWidthMeters,
      marginMeters: CHECKS.bumperMarginMeters,
      maxMetersPerSecond: Number.isFinite(spec.maxMetersPerSecond) ? spec.maxMetersPerSecond : DEFAULT_PLANNER.maxMetersPerSecond,
      maxMetersPerSecondSquared: Number.isFinite(spec.maxMetersPerSecondSquared) ? spec.maxMetersPerSecondSquared : DEFAULT_PLANNER.maxMetersPerSecondSquared,
    },
  );
  const still = { vxMetersPerSecond: 0, vyMetersPerSecond: 0, omegaRadPerSec: 0 };
  const pose = (i: number) => {
    const w = spec.waypoints[i]!;
    return { xMeters: w.xMeters, yMeters: w.yMeters, headingRadians: Number.isNaN(w.headingRadians) ? 0 : w.headingRadians };
  };
  // a waypoint the robot can't stand on: say so (the route goes between waypoints and never moves them)
  for (const i of legs) {
    const w = spec.waypoints[i]!;
    const body = bumperCorners(w.xMeters, w.yMeters, Number.isNaN(w.headingRadians) ? 0 : w.headingRadians, setup.bumperLengthMeters, setup.bumperWidthMeters);
    const hit = setup.field.obstacles.find((o) => {
      const hull = convexHull(o.polygon);
      return hull.length >= 3 && convexOverlap(body, hull);
    });
    if (!hit) continue;
    return {
      ok: false,
      reason: `Waypoint ${i}'s bumpers are inside the ${hit.name}. Move it out first: Route around goes between your waypoints and doesn't move them.`,
    };
  }
  // each leg's corners, between two kept waypoints
  const corners: (readonly [number, number])[][] = [];
  for (let k = 0; k + 1 < legs.length; k++) {
    const r = finder.plan(pose(legs[k]!), still, pose(legs[k + 1]!));
    if (r.failure) return { ok: false, reason: `No route from waypoint ${legs[k]} to ${legs[k + 1]}: ${r.failure}.` };
    corners.push(r.waypoints.slice(1, -1).map(([x, y]) => [round5(x), round5(y)] as const));
  }

  // drop the plain pass-through points between, back to front, then put each leg's corners in after its start
  let out = path;
  let removed = 0;
  for (let i = to - 1; i > from; i--) {
    if (!kept(i)) {
      out = deleteWaypoint(out, i);
      removed++;
    }
  }
  // indices of the kept waypoints after the deletions
  const droppedBefore = (i: number) => {
    let d = 0;
    for (let j = from + 1; j < i; j++) if (!kept(j)) d++;
    return d;
  };
  const keptAfter = legs.map((i) => i - droppedBefore(i));
  // legs back to front (earlier indices stay put); within a leg front to back, each corner splitting what
  // is left of the stretch evenly, so things placed by waypoint position keep their place
  let added = 0;
  for (let k = legs.length - 2; k >= 0; k--) {
    const pts = corners[k]!;
    pts.forEach(([x, y], m) => {
      out = insertWaypoint(out, keptAfter[k]! + m, 1 / (pts.length - m + 1), x, y);
      added++;
    });
  }
  return { ok: true, path: out, added, removed, from, to };
}
