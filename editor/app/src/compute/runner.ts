// Runs analyses in a Web Worker with at most one job in flight and one pending job, which a newer
// request replaces. While a waypoint is dragged, the worker always starts on the newest position as
// soon as it is free, and every finished result is shown, so the preview never starves.

import type { Analysis, AnalysisFailure, AnalysisInput } from "./analysis.ts";
import type { AnalysisRequest, AnalysisResponse } from "./worker.ts";

export type AnalysisResult = Analysis | AnalysisFailure;

/** What the runner needs from a worker; tests pass a fake. */
export interface WorkerLike {
  postMessage(message: AnalysisRequest): void;
  terminate(): void;
  onmessage: ((event: MessageEvent<AnalysisResponse>) => void) | null;
  onerror: ((event: ErrorEvent) => void) | null;
}

interface Job {
  readonly id: number;
  readonly input: AnalysisInput;
  readonly accept: (result: AnalysisResult) => void;
}

/** A job that hasn't answered in this long is abandoned and the worker restarted. */
export const TIMEOUT_MS = 2000;

export class AnalysisRunner {
  private worker: WorkerLike | null = null;
  private running: Job | null = null;
  private pending: Job | null = null;
  private deadline: ReturnType<typeof setTimeout> | null = null;
  private nextId = 0;
  private startedCount = 0;
  private disposed = false;

  constructor(
    private readonly createWorker: () => WorkerLike = () =>
      new Worker(new URL("./worker.ts", import.meta.url), { type: "module" }) as unknown as WorkerLike,
  ) {}

  /** Queues an analysis, replacing any that hasn't started yet. */
  request(input: AnalysisInput, accept: (result: AnalysisResult) => void): void {
    if (this.disposed) return;
    this.pending = { id: ++this.nextId, input, accept };
    if (!this.running) this.startNext();
  }

  /** Jobs started so far (for tests). */
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
    this.deadline = setTimeout(() => this.restart(), TIMEOUT_MS);
    worker.postMessage({ id: job.id, input: job.input });
  }

  private ensureWorker(): WorkerLike {
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

  /** The worker died or hung: drop the job in flight, start afresh with the pending one. */
  private restart(): void {
    this.clearDeadline();
    this.worker?.terminate();
    this.worker = null;
    this.running = null;
    this.startNext();
  }

  private clearDeadline(): void {
    if (this.deadline !== null) clearTimeout(this.deadline);
    this.deadline = null;
  }
}
