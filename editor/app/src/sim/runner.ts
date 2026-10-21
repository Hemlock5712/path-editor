// Runs sim previews in a Web Worker the way the analysis runner does: at most one run in flight and one
// pending, which a newer request replaces, so dragging a waypoint always ends on the newest path.

import type { SimInput, SimResult } from "./preview.ts";
import type { SimRequest, SimResponse } from "./worker.ts";

/** What the runner needs from a worker; tests pass a fake. */
export interface SimWorkerLike {
  postMessage(message: SimRequest): void;
  terminate(): void;
  onmessage: ((event: MessageEvent<SimResponse>) => void) | null;
  onerror: ((event: ErrorEvent) => void) | null;
}

interface Job {
  readonly id: number;
  readonly input: SimInput;
  readonly accept: (result: SimResult) => void;
}

/** A run that hasn't answered in this long is abandoned and the worker restarted. */
export const SIM_TIMEOUT_MS = 5000;

export class SimRunner {
  private worker: SimWorkerLike | null = null;
  private running: Job | null = null;
  private pending: Job | null = null;
  private deadline: ReturnType<typeof setTimeout> | null = null;
  private nextId = 0;
  private startedCount = 0;
  private disposed = false;

  constructor(
    private readonly createWorker: () => SimWorkerLike = () =>
      new Worker(new URL("./worker.ts", import.meta.url), { type: "module" }) as unknown as SimWorkerLike,
  ) {}

  /** Queues a run, replacing any that hasn't started yet. */
  request(input: SimInput, accept: (result: SimResult) => void): void {
    if (this.disposed) return;
    this.pending = { id: ++this.nextId, input, accept };
    if (!this.running) this.startNext();
  }

  /** Drops the pending run (the one in flight still answers, to nobody). */
  clear(): void {
    this.pending = null;
    if (this.running) this.running = { ...this.running, accept: () => {} };
  }

  /** Runs started so far (for tests). */
  get started(): number {
    return this.startedCount;
  }

  dispose(): void {
    this.disposed = true;
    this.pending = null;
    this.running = null;
    this.clearDeadline();
    this.worker?.terminate();
    this.worker = null;
  }

  private startNext(): void {
    const job = this.pending;
    if (!job || this.disposed) return;
    this.pending = null;
    this.running = job;
    this.startedCount++;
    const worker = this.ensureWorker();
    this.deadline = setTimeout(() => this.restart(), SIM_TIMEOUT_MS);
    worker.postMessage({ id: job.id, input: job.input });
  }

  private ensureWorker(): SimWorkerLike {
    if (this.worker) return this.worker;
    const worker = this.createWorker();
    worker.onmessage = (event) => {
      const job = this.running;
      if (!job || event.data.id !== job.id) return;
      this.running = null;
      this.clearDeadline();
      job.accept(event.data.result);
      this.startNext();
    };
    worker.onerror = () => this.restart();
    this.worker = worker;
    return worker;
  }

  /** The worker died or hung: drop the run in flight, start afresh with the pending one. */
  private restart(): void {
    const lost = this.running;
    this.clearDeadline();
    this.worker?.terminate();
    this.worker = null;
    this.running = null;
    if (lost && !this.pending) lost.accept({ ok: false, error: "the simulation didn't finish" });
    this.startNext();
  }

  private clearDeadline(): void {
    if (this.deadline !== null) clearTimeout(this.deadline);
    this.deadline = null;
  }
}
