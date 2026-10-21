// The sim preview's worker: runs the follower on the simulated drivetrain, one run per message, off the
// UI thread (a run is a few thousand follower steps and 1 ms drivetrain substeps).

import { simulate, type SimInput, type SimResult } from "./preview.ts";

export interface SimRequest {
  readonly id: number;
  readonly input: SimInput;
}

export interface SimResponse {
  readonly id: number;
  readonly result: SimResult;
}

/** Answers one request (exported so tests can drive it without a worker). */
export function handle(request: SimRequest): SimResponse {
  let result: SimResult;
  try {
    result = simulate(request.input);
  } catch (e) {
    result = { ok: false, error: e instanceof Error ? e.message : String(e) };
  }
  return { id: request.id, result };
}

// The app's types use the DOM lib; inside a worker, self is the worker scope
const scope = (typeof self === "undefined" ? undefined : self) as unknown as
  | { onmessage: ((event: MessageEvent<SimRequest>) => void) | null; postMessage(message: SimResponse): void }
  | undefined;

if (scope) scope.onmessage = (event) => scope.postMessage(handle(event.data));
