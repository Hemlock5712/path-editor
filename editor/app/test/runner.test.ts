import { describe, expect, it, vi } from "vitest";
import { makePathSpec, makeWaypoint, DEFAULT_ROBOT } from "../../core/src/index.ts";
import type { AnalysisInput } from "../src/compute/analysis.ts";
import { AnalysisRunner, TIMEOUT_MS, type AnalysisResult, type WorkerLike } from "../src/compute/runner.ts";
import type { AnalysisRequest, AnalysisResponse } from "../src/compute/worker.ts";

class FakeWorker implements WorkerLike {
  onmessage: ((event: MessageEvent<AnalysisResponse>) => void) | null = null;
  onerror: ((event: ErrorEvent) => void) | null = null;
  posted: AnalysisRequest[] = [];
  terminated = false;
  postMessage(message: AnalysisRequest): void {
    this.posted.push(message);
  }
  terminate(): void {
    this.terminated = true;
  }
  /** Answers the latest job. */
  finish(): void {
    const job = this.posted.at(-1)!;
    this.onmessage!({ data: { id: job.id, result: { ok: false, error: `done ${job.input.spec.waypoints[1]!.xMeters}` } } } as MessageEvent<AnalysisResponse>);
  }
}

const job = (x: number): AnalysisInput => ({
  spec: makePathSpec({ waypoints: [makeWaypoint(0, 0), makeWaypoint(x, 0)] }),
  robot: DEFAULT_ROBOT,
  bumperLengthMeters: 0.85,
  bumperWidthMeters: 0.85,
  fieldLengthMeters: 16.54,
  fieldWidthMeters: 8.07,
  obstacles: [],
});

describe("AnalysisRunner", () => {
  it("runs one job at a time and replaces the pending one", () => {
    const workers: FakeWorker[] = [];
    const runner = new AnalysisRunner(() => {
      const w = new FakeWorker();
      workers.push(w);
      return w;
    });
    const results: string[] = [];
    const accept = (r: AnalysisResult) => results.push(r.ok ? "ok" : r.error);
    for (let x = 1; x <= 5; x++) runner.request(job(x), accept);
    expect(runner.started).toBe(1); // 1 in flight; 2, 3 and 4 were replaced by 5
    workers[0]!.finish();
    expect(runner.started).toBe(2);
    expect(workers[0]!.posted.map((p) => p.input.spec.waypoints[1]!.xMeters)).toEqual([1, 5]);
    workers[0]!.finish();
    expect(results).toEqual(["done 1", "done 5"]);
    runner.dispose();
    expect(workers[0]!.terminated).toBe(true);
  });

  it("restarts a hung worker and carries on with the newest job", () => {
    vi.useFakeTimers();
    const workers: FakeWorker[] = [];
    const runner = new AnalysisRunner(() => {
      const w = new FakeWorker();
      workers.push(w);
      return w;
    });
    const results: string[] = [];
    runner.request(job(1), (r) => results.push(r.ok ? "ok" : r.error));
    runner.request(job(2), (r) => results.push(r.ok ? "ok" : r.error));
    vi.advanceTimersByTime(TIMEOUT_MS + 1);
    expect(workers[0]!.terminated).toBe(true);
    expect(workers[1]!.posted[0]!.input.spec.waypoints[1]!.xMeters).toBe(2);
    workers[1]!.finish();
    expect(results).toEqual(["done 2"]);
    runner.dispose();
    vi.useRealTimers();
  });
});
