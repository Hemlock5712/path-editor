// The waypoint optimiser's run: options, progress, the result, and applying it as one undo step. The
// field draws the best path so far, dashed, while it runs and until it is applied or discarded.

import type { PathSpec } from "../../../core/src/index.ts";
import { DEFAULT_BUDGET_SECONDS, DEFAULT_OPTIONS, evaluationsFor, type OptimiseOptions, type OptimiseResult } from "../compute/optimiser.ts";
import type { OptimiserRequest, OptimiserResponse } from "../compute/optimiserWorker.ts";
import { analysisInput } from "./analysis.ts";
import { activePath, editor, editPath, notify } from "./editor.ts";
import { createStore } from "./store.ts";

export interface OptimiserState {
  readonly status: "idle" | "running" | "done" | "error";
  /** The path and spec the run started from: applying needs both unchanged. */
  readonly pathId: string | null;
  readonly basis: PathSpec | null;
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
}

export const optimiser = createStore<OptimiserState>({
  status: "idle",
  pathId: null,
  basis: null,
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
});

let worker: Worker | null = null;
let runId = 0;

function stopWorker(): void {
  worker?.terminate();
  worker = null;
}

export function startOptimise(): void {
  const s = editor.get();
  const path = activePath(s);
  const o = optimiser.get();
  // named points stay where they are: their waypoints are locked too
  const locked = path.spec.waypoints.map((_, i) => path.pinned[i] === true || path.links[i] !== null);
  const n = path.spec.waypoints.length;
  const segment = o.segment && o.segment.every((i) => i >= 0 && i < n) ? o.segment : null;
  const options: OptimiseOptions = {
    ...DEFAULT_OPTIONS,
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
    preview: null,
    result: null,
    error: null,
  });
  const request: OptimiserRequest = { type: "start", id, input: analysisInput(s.project, path.spec), options };
  worker.postMessage(request);
}

/** Stops the run; the best found so far comes back as the result. */
export function cancelOptimise(): void {
  if (worker && optimiser.get().status === "running") worker.postMessage({ type: "cancel", id: runId } satisfies OptimiserRequest);
}

export function discardOptimise(): void {
  runId++;
  stopWorker();
  optimiser.set({ status: "idle", preview: null, result: null, basis: null, pathId: null, error: null });
}

/** Applies the result to the path it was made for, as one undoable command. */
export function applyOptimise(): void {
  const o = optimiser.get();
  const path = activePath();
  if (!o.result || o.pathId !== path.id || o.basis !== path.spec) {
    notify("The path changed since the optimiser ran; run it again.", "error");
    return;
  }
  const spec = o.result.spec;
  editPath("Optimise path", (p) => ({ ...p, spec }));
  discardOptimise();
}
