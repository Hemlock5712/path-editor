// The optimiser's worker: runs in chunks of evaluations so a cancel message gets through, and reports
// the best so far after each chunk (the field previews it).

import type { PathSpec } from "../../../core/src/index.ts";
import type { AnalysisInput } from "./analysis.ts";
import { Optimiser, type OptimiseOptions, type OptimiseResult } from "./optimiser.ts";

export type OptimiserRequest =
  | { readonly type: "start"; readonly id: number; readonly input: AnalysisInput; readonly options: OptimiseOptions }
  | { readonly type: "cancel"; readonly id: number };

export type OptimiserResponse =
  | {
      readonly type: "progress";
      readonly id: number;
      readonly evaluations: number;
      readonly beforeSeconds: number;
      readonly bestSeconds: number;
      readonly best: PathSpec;
    }
  | { readonly type: "done"; readonly id: number; readonly result: OptimiseResult }
  | { readonly type: "error"; readonly id: number; readonly message: string };

const CHUNK = 25;

const scope = self as unknown as {
  onmessage: ((event: MessageEvent<OptimiserRequest>) => void) | null;
  postMessage(message: OptimiserResponse): void;
};

let cancelled = -1;

scope.onmessage = (event) => {
  const msg = event.data;
  if (msg.type === "cancel") {
    cancelled = msg.id;
    return;
  }
  let o: Optimiser;
  try {
    o = new Optimiser(msg.input, msg.options);
  } catch (e) {
    scope.postMessage({ type: "error", id: msg.id, message: (e as Error).message });
    return;
  }
  const deadline = performance.now() + msg.options.timeBudgetMs;
  const chunk = () => {
    const stopped = o.run(o.evaluationCount + CHUNK, deadline, () => cancelled === msg.id);
    scope.postMessage({
      type: "progress",
      id: msg.id,
      evaluations: o.evaluationCount,
      beforeSeconds: o.beforeSeconds,
      bestSeconds: o.bestSeconds,
      best: o.best,
    });
    if (stopped) scope.postMessage({ type: "done", id: msg.id, result: o.result(stopped) });
    else setTimeout(chunk, 0); // let a cancel in
  };
  chunk();
};
