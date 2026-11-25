// Runs sim previews in a Web Worker the way the analysis runner does: at most one run in flight and one
// pending, which a newer request replaces, so dragging a waypoint always ends on the newest path.

import { LatestJobRunner, type WorkerLike } from "../compute/runner.ts";
import type { SimInput, SimResult } from "./preview.ts";

export type SimWorkerLike = WorkerLike<SimInput, SimResult>;

/** A run that hasn't answered in this long is abandoned and the worker restarted. */
export const SIM_TIMEOUT_MS = 5000;

export class SimRunner extends LatestJobRunner<SimInput, SimResult> {
  constructor(
    createWorker: () => SimWorkerLike = () =>
      new Worker(new URL("./worker.ts", import.meta.url), { type: "module" }) as unknown as SimWorkerLike,
  ) {
    super(createWorker, SIM_TIMEOUT_MS, { ok: false, error: "the simulation didn't finish" });
  }
}
