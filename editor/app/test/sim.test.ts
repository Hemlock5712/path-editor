// The sim preview: the numbers it shows (against the golden closed-loop traces), its worker, its runner
// and its store.

import { readFileSync } from "node:fs";
import { join } from "node:path";
import { afterEach, describe, expect, it, vi } from "vitest";
import {
  DEFAULT_DRIVE,
  DEFAULT_ROBOT,
  GRAVITY,
  makePathSpec,
  makeWaypoint,
  pathFromJson,
  robotFromJson,
  asObject,
  type JsonValue,
  type PathSpec,
} from "../../core/src/index.ts";
import { simulate, simPoseAt, type SimInput, type SimPreview, type SimResult } from "../src/sim/preview.ts";
import { SimRunner, SIM_TIMEOUT_MS, type SimWorkerLike } from "../src/sim/runner.ts";
import { DEFAULT_SIM_SETTINGS, driveOf, shoveOf, sim, simSettings, startSim } from "../src/sim/state.ts";
import { handle, type SimRequest, type SimResponse } from "../src/sim/worker.ts";
import { errorColour } from "../src/sim/draw.ts";
import { editor, editPath } from "../src/state/editor.ts";

const FIXTURES = join(import.meta.dirname, "..", "..", "..", "fixtures", "follow");

function fixture(name: string): { spec: PathSpec; robot: typeof DEFAULT_ROBOT; finishLoop: number; events: { name: string; loop: number }[]; x: number[]; crossTrack: number[] } {
  const fx = JSON.parse(readFileSync(join(FIXTURES, `${name}.json`), "utf8")) as {
    inputs: { path: JsonValue; robot: JsonValue };
    finishLoop: number;
    events: { name: string; loop: number }[];
    loops: { x: number[]; crossTrack: number[] };
  };
  return { spec: pathFromJson(asObject(fx.inputs.path, "path")), robot: robotFromJson(asObject(fx.inputs.robot, "robot")), finishLoop: fx.finishLoop, events: fx.events, x: fx.loops.x, crossTrack: fx.loops.crossTrack };
}

function ok(r: SimResult): SimPreview {
  if (!r.ok) throw new Error(r.error);
  return r;
}

const line = (x: number, end = 0) => makePathSpec({ waypoints: [makeWaypoint(0, 0), makeWaypoint(x, 0)], endMetersPerSecond: end });

describe("simulate", () => {
  it("runs the golden traces' run: the same finish, the same event loops, the same poses", () => {
    const fx = fixture("events");
    const p = ok(simulate({ spec: fx.spec, robot: fx.robot, drive: DEFAULT_DRIVE, shove: null }));
    expect(p.finishSeconds).toBeCloseTo(fx.finishLoop * 0.02, 12);
    expect(p.x).toEqual(fx.x);
    p.phase.forEach((phase, i) => phase === "TRACKING" && expect(p.crossTrack[i]).toBe(fx.crossTrack[i]));
    // in file order here, in firing order in the trace (arm, at 3 m, comes before waypoint 1)
    expect(p.events.map((e) => e.name)).toEqual(["intake", "arm", "score"]);
    expect([...p.events].sort((a, b) => a.loop - b.loop).map((e) => ({ name: e.name, loop: e.loop }))).toEqual(fx.events);
  });

  it("reports the gap to the plan, the settle and where the events fired", () => {
    const fx = fixture("events");
    const p = ok(simulate({ spec: fx.spec, robot: fx.robot, drive: DEFAULT_DRIVE, shove: null }));
    expect(p.gapSeconds).toBeCloseTo(p.finishSeconds - p.planSeconds, 12);
    expect(Math.abs(p.gapSeconds)).toBeLessThan(1.0);
    expect(p.terminalSeconds).toBeLessThan(p.finishSeconds);
    expect(p.settleSeconds).toBeGreaterThan(0);
    expect(p.endErrorMeters).toBeLessThan(0.02);
    for (const e of p.events) {
      expect(e.loop).toBeGreaterThanOrEqual(0);
      expect(e.offsetMeters).toBeLessThan(0.1); // fired on the nearest loop, within a loop's travel of its flag
      expect(e.offsetMeters).toBeCloseTo(Math.hypot(p.x[e.loop]! - e.x, p.y[e.loop]! - e.y), 12);
      expect(Math.abs(e.t - e.planT)).toBeLessThan(0.5);
    }
    expect(p.maxCrossTrackMeters).toBeGreaterThanOrEqual(p.rmsCrossTrackMeters);
    const end = simPoseAt(p, 1e9);
    expect(end.x).toBe(p.x.at(-1));
  });

  it("pairs events that share a name in order along the path", () => {
    const spec = makePathSpec({
      waypoints: [makeWaypoint(0, 0), makeWaypoint(2, 0), makeWaypoint(4, 0)],
      events: [
        { name: "beep", waypoint: Number.NaN, meters: 3 },
        { name: "beep", waypoint: Number.NaN, meters: 1 },
      ],
    });
    const p = ok(simulate({ spec, robot: DEFAULT_ROBOT, drive: DEFAULT_DRIVE, shove: null }));
    expect(p.events[0]!.s).toBeCloseTo(3, 9);
    expect(p.events[0]!.loop).toBeGreaterThan(p.events[1]!.loop);
    expect(p.events[1]!.loop).toBeGreaterThanOrEqual(0);
  });

  it("shows a shove's error and the recovery", () => {
    const fx = fixture("s_curve");
    const calm = ok(simulate({ spec: fx.spec, robot: fx.robot, drive: DEFAULT_DRIVE, shove: null }));
    const hit = ok(simulate({ spec: fx.spec, robot: fx.robot, drive: DEFAULT_DRIVE, shove: { atSeconds: 1.0, dvx: 0, dvy: 3 } }));
    expect(hit.maxCrossTrackMeters).toBeGreaterThan(calm.maxCrossTrackMeters + 0.03);
    expect(Number.isFinite(hit.finishSeconds)).toBe(true);
    expect(hit.shove).toEqual({ atSeconds: 1.0, dvx: 0, dvy: 3 });
  });

  it("measures the error across the path after the hand-over too, where the follower reports none", () => {
    const fx = fixture("s_curve");
    const calm = ok(simulate({ spec: fx.spec, robot: fx.robot, drive: DEFAULT_DRIVE, shove: null }));
    const t = calm.terminalSeconds + 0.04; // hit just after the hand-over to the goal
    const hit = ok(simulate({ spec: fx.spec, robot: fx.robot, drive: DEFAULT_DRIVE, shove: { atSeconds: t, dvx: 0, dvy: 1.5 } }));
    expect(hit.maxCrossTrackMeters).toBeGreaterThan(calm.maxCrossTrackMeters + 0.01);
    const rotation = fixture("pure_rotation");
    const spin = ok(simulate({ spec: rotation.spec, robot: rotation.robot, drive: DEFAULT_DRIVE, shove: null }));
    expect(spin.crossTrack.every(Number.isFinite)).toBe(true);
  });

  it("slides on a slick floor", () => {
    const fx = fixture("s_curve");
    const firm = ok(simulate({ spec: fx.spec, robot: fx.robot, drive: DEFAULT_DRIVE, shove: null }));
    const slick = ok(simulate({ spec: fx.spec, robot: fx.robot, drive: { ...DEFAULT_DRIVE, gripMetersPerSecondSquared: 0.2 * GRAVITY }, shove: null }));
    expect(slick.maxCrossTrackMeters).toBeGreaterThan(firm.maxCrossTrackMeters);
  });

  it("has no settle for a rolling end", () => {
    const p = ok(simulate({ spec: line(3, 1.0), robot: DEFAULT_ROBOT, drive: DEFAULT_DRIVE, shove: null }));
    expect(Number.isFinite(p.finishSeconds)).toBe(true);
    expect(Number.isNaN(p.terminalSeconds)).toBe(true);
    expect(Number.isNaN(p.settleSeconds)).toBe(true);
  });

  it("says why a path can't be simulated", () => {
    const r = simulate({ spec: { ...line(1), waypoints: [makeWaypoint(0, 0)] }, robot: DEFAULT_ROBOT, drive: DEFAULT_DRIVE, shove: null });
    expect(r.ok).toBe(false);
  });
});

describe("the sim worker", () => {
  afterEach(() => vi.unstubAllGlobals());

  it("answers each request with its id, and the answer survives the trip between threads", () => {
    const r = handle({ id: 7, input: { spec: line(2), robot: DEFAULT_ROBOT, drive: DEFAULT_DRIVE, shove: null } });
    expect(r.id).toBe(7);
    const copy = structuredClone(r);
    expect(ok(copy.result).finishSeconds).toBe(ok(r.result).finishSeconds);
    expect(handle({ id: 8, input: { spec: { ...line(1), waypoints: [] }, robot: DEFAULT_ROBOT, drive: DEFAULT_DRIVE, shove: null } }).result.ok).toBe(false);
  });

  it("listens on the worker scope", async () => {
    const posted: SimResponse[] = [];
    const scope = { onmessage: null as ((e: MessageEvent<SimRequest>) => void) | null, postMessage: (m: SimResponse) => posted.push(m) };
    vi.stubGlobal("self", scope);
    vi.resetModules();
    await import("../src/sim/worker.ts");
    scope.onmessage!({ data: { id: 3, input: { spec: line(1), robot: DEFAULT_ROBOT, drive: DEFAULT_DRIVE, shove: null } } } as MessageEvent<SimRequest>);
    expect(posted).toHaveLength(1);
    expect(posted[0]!.id).toBe(3);
    expect(posted[0]!.result.ok).toBe(true);
  });
});

class FakeWorker implements SimWorkerLike {
  onmessage: ((event: MessageEvent<SimResponse>) => void) | null = null;
  onerror: ((event: ErrorEvent) => void) | null = null;
  posted: SimRequest[] = [];
  terminated = false;
  postMessage(message: SimRequest): void {
    this.posted.push(message);
  }
  terminate(): void {
    this.terminated = true;
  }
  /** Answers the latest job by really running it. */
  finish(): void {
    this.onmessage!({ data: handle(this.posted.at(-1)!) } as MessageEvent<SimResponse>);
  }
}

const input = (x: number): SimInput => ({ spec: line(x), robot: DEFAULT_ROBOT, drive: DEFAULT_DRIVE, shove: null });

describe("SimRunner", () => {
  it("runs one at a time and keeps only the newest pending run", () => {
    const workers: FakeWorker[] = [];
    const runner = new SimRunner(() => {
      const w = new FakeWorker();
      workers.push(w);
      return w;
    });
    const plans: number[] = [];
    const accept = (r: SimResult) => plans.push(ok(r).planSeconds);
    for (let x = 1; x <= 4; x++) runner.request(input(x), accept);
    expect(runner.started).toBe(1);
    workers[0]!.finish();
    expect(runner.started).toBe(2);
    workers[0]!.finish();
    expect(workers[0]!.posted.map((p) => p.input.spec.waypoints[1]!.xMeters)).toEqual([1, 4]);
    expect(plans).toHaveLength(2);
    expect(plans[1]!).toBeGreaterThan(plans[0]!);
    runner.dispose();
    expect(workers[0]!.terminated).toBe(true);
  });

  it("restarts a hung worker and says the run didn't finish", () => {
    vi.useFakeTimers();
    try {
      const workers: FakeWorker[] = [];
      const runner = new SimRunner(() => {
        const w = new FakeWorker();
        workers.push(w);
        return w;
      });
      const results: SimResult[] = [];
      runner.request(input(1), (r) => results.push(r));
      vi.advanceTimersByTime(SIM_TIMEOUT_MS + 1);
      expect(workers[0]!.terminated).toBe(true);
      expect(results).toEqual([{ ok: false, error: "the simulation didn't finish" }]);
      runner.request(input(2), (r) => results.push(r));
      expect(workers).toHaveLength(2);
      runner.dispose();
    } finally {
      vi.useRealTimers();
    }
  });

  it("drops a cleared run's answer", () => {
    const w = new FakeWorker();
    const runner = new SimRunner(() => w);
    const results: SimResult[] = [];
    runner.request(input(1), (r) => results.push(r));
    runner.clear();
    w.finish();
    expect(results).toHaveLength(0);
  });
});

describe("the sim store", () => {
  afterEach(() => simSettings.set(DEFAULT_SIM_SETTINGS));

  it("defaults to the end-to-end suite's drivetrain and turns settings into inputs", () => {
    expect(driveOf(DEFAULT_SIM_SETTINGS)).toEqual(DEFAULT_DRIVE);
    expect(driveOf({ ...DEFAULT_SIM_SETTINGS, gripMu: 0.5 }).gripMetersPerSecondSquared).toBeCloseTo(0.5 * GRAVITY, 12);
    expect(shoveOf(DEFAULT_SIM_SETTINGS)).toBeNull();
    expect(shoveOf({ ...DEFAULT_SIM_SETTINGS, shoveOn: true })).toEqual({ atSeconds: 1.0, dvx: 0, dvy: 1.5 });
  });

  it("runs only while on, and again whenever the path or a setting changes", () => {
    const w = new FakeWorker();
    const runner = new SimRunner(() => w);
    const stop = startSim(runner);
    expect(runner.started).toBe(0);
    simSettings.set({ enabled: true });
    expect(runner.started).toBe(1);
    w.finish();
    const first = sim.get();
    expect(first.result?.ok).toBe(true);
    expect(first.input?.spec).toBe(editor.get().project.paths[0]!.spec);

    simSettings.set({ shoveOn: true });
    expect(runner.started).toBe(2);
    expect(w.posted.at(-1)!.input.shove).toEqual({ atSeconds: 1.0, dvx: 0, dvy: 1.5 });
    w.finish();

    editPath("Move", (p) => ({ ...p, spec: { ...p.spec, waypoints: p.spec.waypoints.map((q, i) => (i === 0 ? { ...q, xMeters: q.xMeters + 0.1 } : q)) } }));
    expect(runner.started).toBe(3);
    w.finish();
    expect(sim.get().input?.spec).toBe(editor.get().project.paths[0]!.spec);

    simSettings.set({ enabled: false });
    expect(sim.get().result).toBeNull();
    stop();
    expect(w.terminated).toBe(true);
  });

  it("shades the trace from orange to red with the error", () => {
    expect(errorColour(0)).toBe("rgb(251,146,60)");
    expect(errorColour(-1)).toBe("rgb(239,38,38)");
  });
});
