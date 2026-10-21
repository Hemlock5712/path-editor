// The optimiser's run: options, progress, the result, and applying it as one undo step. The field draws
// the best path so far, dashed, while it runs and until it is applied or discarded (or the path is
// edited, which drops the result). A result that couldn't clear an obstacle marks the spot on the field
// and offers a waypoint there. Route around obstacles is a run too: the pathfinder's corners go into a copy
// of the path, the curves-only optimiser shapes that, and Apply puts both in as one undo step.

import type { PathSpec } from "../../../core/src/index.ts";
import { analyse } from "../compute/analysis.ts";
import { routeAround } from "../compute/route.ts";
import type { EditorPath } from "../model/project.ts";
import { DEFAULT_BUDGET_SECONDS, DEFAULT_OPTIONS, evaluationsFor, type OptimiseOptions, type OptimiseResult } from "../compute/optimiser.ts";
import type { OptimiserRequest, OptimiserResponse } from "../compute/optimiserWorker.ts";
import { analysis, analysisInput, geometryOf } from "./analysis.ts";
import { addWaypointAfter } from "../model/edits.ts";
import { activePath, editor, editPath, notify } from "./editor.ts";
import { createStore } from "./store.ts";

export interface OptimiserState {
  readonly status: "idle" | "running" | "done" | "error";
  /** The path and spec the run started from: applying needs both unchanged. */
  readonly pathId: string | null;
  readonly basis: PathSpec | null;
  /** Also move the unlocked interior waypoints (off: curves and speeds only). */
  readonly moveWaypoints: boolean;
  readonly corridorMeters: number;
  /** Sets the evaluation budget (so the result repeats); the clock stops a slow machine at twice it. */
  readonly budgetSeconds: number;
  readonly seed: number;
  readonly optimiseScales: boolean;
  /** Only move the waypoints between these two (null = the whole path). */
  readonly segment: readonly [number, number] | null;
  readonly evaluations: number;
  readonly beforeSeconds: number;
  readonly bestSeconds: number;
  /** The best path so far (or the result), drawn on the field. */
  readonly preview: PathSpec | null;
  readonly result: OptimiseResult | null;
  readonly error: string | null;
  /** A Route around run: the path with the route's corners in (what the run shapes), and how it changed. */
  readonly routed: { readonly path: EditorPath; readonly added: number; readonly removed: number } | null;
}

export const optimiser = createStore<OptimiserState>({
  status: "idle",
  pathId: null,
  basis: null,
  moveWaypoints: DEFAULT_OPTIONS.moveWaypoints,
  corridorMeters: DEFAULT_OPTIONS.corridorMeters,
  budgetSeconds: DEFAULT_BUDGET_SECONDS,
  seed: DEFAULT_OPTIONS.seed,
  optimiseScales: DEFAULT_OPTIONS.optimiseScales,
  segment: null,
  evaluations: 0,
  beforeSeconds: Number.NaN,
  bestSeconds: Number.NaN,
  preview: null,
  result: null,
  error: null,
  routed: null,
});

let worker: Worker | null = null;
let runId = 0;

function stopWorker(): void {
  worker?.terminate();
  worker = null;
}

/**
 * Optimises the active path, or (Route around) a routed copy of it; either way the result is shown dashed
 * until it is applied or discarded.
 */
export function startOptimise(routed: OptimiserState["routed"] = null): void {
  const s = editor.get();
  const path = activePath(s);
  const o = optimiser.get();
  const base = routed?.path ?? path;
  // named points stay where they are: their waypoints are locked too
  const locked = base.spec.waypoints.map((_, i) => base.pinned[i] === true || base.links[i] !== null);
  const n = base.spec.waypoints.length;
  const segment = !routed && o.segment && o.segment.every((i) => i >= 0 && i < n) ? o.segment : null;
  const options: OptimiseOptions = {
    ...DEFAULT_OPTIONS,
    // a route is shaped by its curve only: its corners are where the pathfinder put them
    moveWaypoints: routed ? false : o.moveWaypoints,
    corridorMeters: o.corridorMeters,
    maxEvaluations: evaluationsFor(o.budgetSeconds),
    timeBudgetMs: 2000 * o.budgetSeconds,
    seed: o.seed,
    optimiseScales: o.optimiseScales,
    locked,
    segment,
  };
  stopWorker();
  const id = ++runId;
  worker = new Worker(new URL("../compute/optimiserWorker.ts", import.meta.url), { type: "module" });
  worker.onmessage = (event: MessageEvent<OptimiserResponse>) => {
    const msg = event.data;
    if (msg.id !== runId) return;
    if (msg.type === "progress") {
      optimiser.set({ evaluations: msg.evaluations, beforeSeconds: msg.beforeSeconds, bestSeconds: msg.bestSeconds, preview: msg.best });
    } else if (msg.type === "done") {
      optimiser.set({ status: "done", result: msg.result, preview: msg.result.spec, bestSeconds: msg.result.afterSeconds });
      stopWorker();
    } else {
      optimiser.set({ status: "error", error: msg.message, preview: null });
      stopWorker();
    }
  };
  optimiser.set({
    status: "running",
    pathId: path.id,
    basis: path.spec,
    evaluations: 0,
    beforeSeconds: Number.NaN,
    bestSeconds: Number.NaN,
    preview: routed?.path.spec ?? null,
    result: null,
    error: null,
    routed,
  });
  const request: OptimiserRequest = { type: "start", id, input: analysisInput(s.project, base.spec), options };
  worker.postMessage(request);
}

/** Stops the run; the best found so far comes back as the result. */
export function cancelOptimise(): void {
  if (worker && optimiser.get().status === "running") worker.postMessage({ type: "cancel", id: runId } satisfies OptimiserRequest);
}

export function discardOptimise(): void {
  runId++;
  stopWorker();
  optimiser.set({ status: "idle", preview: null, result: null, basis: null, pathId: null, error: null, routed: null });
}

/**
 * Route around obstacles: the pathfinder plans round the first stretch the bumpers hit, its corners go in
 * as pass-through points, and the curves-only optimiser shapes the result (shown dashed until applied).
 */
export function startRouteAround(): void {
  const s = editor.get();
  const path = activePath(s);
  const known = analysis.get();
  const a = known.spec === path.spec && known.result ? known.result : analyse(analysisInput(s.project, path.spec));
  if (!a.ok) {
    notify(`The path can't be planned: ${a.error}`, "error");
    return;
  }
  const r = routeAround(path, a, {
    field: s.project.field,
    robot: s.project.robot.config,
    bumperLengthMeters: s.project.robot.bumperLengthMeters,
    bumperWidthMeters: s.project.robot.bumperWidthMeters,
  });
  if (!r.ok) {
    notify(r.reason, "error");
    return;
  }
  startOptimise({ path: r.path, added: r.added, removed: r.removed });
}

/** Adds the waypoint a blocked result suggests, where the path still hits (one undo step). */
export function addSuggestedWaypoint(): void {
  const o = optimiser.get();
  const at = o.result?.blockedAt;
  const path = activePath();
  if (!at || o.pathId !== path.id) return;
  const g = geometryOf(path.spec);
  const from = at.fromWaypoint;
  // where the hit lies on its stretch, so things placed by waypoint keep their place
  const f = g ? (at.s - g.sAtWaypoint(from)) / Math.max(1e-9, g.sAtWaypoint(from + 1) - g.sAtWaypoint(from)) : 0.5;
  editPath("Add waypoint", (p) => addWaypointAfter(p, from, at.suggestX, at.suggestY, f), { kind: "waypoint", index: from + 1 });
}

// an edit to the path a result was made for makes the result stale: drop it (and its dashed preview)
editor.subscribe(() => {
  const o = optimiser.get();
  if (o.status !== "done" || !o.pathId) return;
  const path = editor.get().project.paths.find((p) => p.id === o.pathId);
  if (!path || path.spec !== o.basis) discardOptimise();
});

/** Applies the result to the path it was made for, as one undoable command. */
export function applyOptimise(): void {
  const o = optimiser.get();
  const path = activePath();
  if (!o.result || o.pathId !== path.id || o.basis !== path.spec) {
    notify("The path changed since the optimiser ran; run it again.", "error");
    return;
  }
  const spec = o.result.spec;
  const routed = o.routed;
  if (routed) editPath("Route around obstacles", () => ({ ...routed.path, spec }));
  else editPath("Optimise path", (p) => ({ ...p, spec }));
  discardOptimise();
}
