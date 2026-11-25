// Runs jobs in a Web Worker with at most one in flight and one pending, which a newer request replaces.
// While a waypoint is dragged, the worker always starts on the newest position as soon as it is free, and
// every finished result is shown, so the preview never starves. The analysis and the sim preview both use it.

import type { Analysis, AnalysisFailure, AnalysisInput } from "./analysis.ts";

export type AnalysisResult = Analysis | AnalysisFailure;

/** What the runner needs from a worker; tests pass a fake. */
export interface WorkerLike<In, Out> {
  postMessage(message: { readonly id: number; readonly input: In }): void;
  terminate(): void;
  onmessage: ((event: MessageEvent<{ readonly id: number; readonly result: Out }>) => void) | null;
  onerror: ((event: ErrorEvent) => void) | null;
}

interface Job<In, Out> {
  readonly id: number;
  readonly input: In;
  readonly accept: (result: Out) => void;
}

/** An analysis that hasn't answered in this long is abandoned and the worker restarted. */
export const TIMEOUT_MS = 2000;

export class LatestJobRunner<In, Out> {
  private worker: WorkerLike<In, Out> | null = null;
  private running: Job<In, Out> | null = null;
  private pending: Job<In, Out> | null = null;
  private deadline: ReturnType<typeof setTimeout> | null = null;
  private nextId = 0;
  private startedCount = 0;
  private disposed = false;

  /** `lost`, if given, is what a job gets when its worker dies or hangs and nothing newer is waiting. */
  constructor(
    private readonly createWorker: () => WorkerLike<In, Out>,
    private readonly timeoutMs: number,
    private readonly lost?: Out,
  ) {}

  /** Queues a job, replacing any that hasn't started yet. */
  request(input: In, accept: (result: Out) => void): void {
    if (this.disposed) return;
    this.pending = { id: ++this.nextId, input, accept };
    if (!this.running) this.startNext();
  }

  /** Drops the pending job (the one in flight still answers, to nobody). */
  clear(): void {
    this.pending = null;
    if (this.running) this.running = { ...this.running, accept: () => {} };
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
    this.deadline = setTimeout(() => this.restart(), this.timeoutMs);
    worker.postMessage({ id: job.id, input: job.input });
  }

  private ensureWorker(): WorkerLike<In, Out> {
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
    const lost = this.running;
    this.clearDeadline();
    this.worker?.terminate();
    this.worker = null;
    this.running = null;
    if (lost && !this.pending && this.lost !== undefined) lost.accept(this.lost);
    this.startNext();
  }

  private clearDeadline(): void {
    if (this.deadline !== null) clearTimeout(this.deadline);
    this.deadline = null;
  }
}

export class AnalysisRunner extends LatestJobRunner<AnalysisInput, AnalysisResult> {
  constructor(
    createWorker: () => WorkerLike<AnalysisInput, AnalysisResult> = () =>
      new Worker(new URL("./worker.ts", import.meta.url), { type: "module" }) as unknown as WorkerLike<
        AnalysisInput,
        AnalysisResult
      >,
  ) {
    super(createWorker, TIMEOUT_MS);
  }
}
