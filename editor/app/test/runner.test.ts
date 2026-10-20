import { describe, expect, it, vi } from "vitest";
import { makePathSpec, makeWaypoint, DEFAULT_ROBOT } from "../../core/src/index.ts";
import type { AnalysisInput } from "../src/compute/analysis.ts";
import { AnalysisRunner, TIMEOUT_MS, type AnalysisResult, type WorkerLike } from "../src/compute/runner.ts";
import type { AnalysisRequest, AnalysisResponse } from "../src/compute/worker.ts";
import { decodeAll } from "../src/nt4/msgpack.ts";
import { decodePose2d, Nt4Client, poseOf } from "../src/nt4/client.ts";
import { live } from "../src/state/live.ts";

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

describe("NT4", () => {
  // [1, 1000, 19, bin(Pose2d x=1.5 y=-2 heading=0.5)] and [2, 1000, 1, 3.25]
  function frame(): Uint8Array {
    const pose = new DataView(new ArrayBuffer(24));
    pose.setFloat64(0, 1.5, true);
    pose.setFloat64(8, -2, true);
    pose.setFloat64(16, 0.5, true);
    const bytes = [0x94, 0x01, 0xcd, 0x03, 0xe8, 0x13, 0xc4, 24, ...new Uint8Array(pose.buffer)];
    const dbl = new DataView(new ArrayBuffer(8));
    dbl.setFloat64(0, 3.25);
    bytes.push(0x94, 0x02, 0xcd, 0x03, 0xe8, 0x01, 0xcb, ...new Uint8Array(dbl.buffer));
    return new Uint8Array(bytes);
  }

  it("decodes MessagePack frames", () => {
    const values = decodeAll(frame());
    expect(values).toHaveLength(2);
    expect((values[0] as unknown[])[0]).toBe(1);
    expect((values[1] as unknown[])[3]).toBe(3.25);
    expect(decodePose2d((values[0] as unknown[])[3] as Uint8Array)).toEqual({ x: 1.5, y: -2, heading: 0.5 });
    expect(poseOf("double[]", [1, 2, 90])!.heading).toBeCloseTo(Math.PI / 2, 12);
  });

  it("maps announced topics to the robot pose and the follower's telemetry", () => {
    const client = new Nt4Client({ host: "x", poseTopic: "/Robot/Pose", telemetryPrefix: "/Telemetry/Path/" });
    client.onText(
      JSON.stringify([
        { method: "announce", params: { id: 1, name: "/Robot/Pose", type: "struct:Pose2d" } },
        { method: "announce", params: { id: 2, name: "/Telemetry/Path/ToReef/Progress", type: "double" } },
      ]),
    );
    client.onBinary(frame());
    expect(live.get().robot?.pose).toEqual({ x: 1.5, y: -2, heading: 0.5 });
    expect(live.get().telemetry["ToReef/Progress"]).toBe(3.25);
  });
});
