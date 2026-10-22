import { describe, expect, it } from "vitest";
import type { Nt4Topic, Nt4Value } from "../src/nt4/client.ts";
import { exportRun, importRun, Recorder, sampleAt, summarise } from "../src/nt4/recorder.ts";

const topics = new Map<string, Nt4Topic>();
const v = (name: string, value: Nt4Value["value"], type = typeof value === "string" ? "string" : Array.isArray(value) ? "double[]" : "double"): Nt4Value => {
  const topic = topics.get(name) ?? { id: topics.size + 1, name, type, properties: {} };
  topics.set(name, topic);
  return { topic, serverTimeUs: 0, value };
};
const P = "/Telemetry/Path/Hop/";

/** One robot loop, delivered as one frame. */
function loop(r: Recorder, t: number, x: number, extra: Nt4Value[] = []): void {
  for (const value of [
    v(`${P}Phase`, t < 0.1 ? "TRACKING" : "TERMINAL"),
    v(`${P}Progress`, x),
    v(`${P}CrossTrack`, 0.01 * t),
    v(`${P}AlongTrack`, -0.02),
    v(`${P}HeadingError`, 0.001),
    v(`${P}Speed`, 1),
    v(`${P}PlannedSpeed`, 1.1),
    v(`${P}Omega`, 0),
    v(`${P}PlannedOmega`, 0),
    v(`${P}Elapsed`, t),
    ...extra,
    v("/Telemetry/Robot/Pose", [x, 0, 0]),
  ])
    r.onValue(value);
  r.onFrameEnd();
}

describe("recording runs from PathTelemetry", () => {
  it("records each loop of a run, with its plan, events and goal, from start to end", () => {
    const r = new Recorder("/Telemetry/Robot/Pose", 3);
    // the start: the plan, then Active, and the first loop, all in one frame
    for (const value of [
      v(`${P}Plan`, [0, 0, 0, 0, 0, 0, 0.1, 0.1, 0, 0, 0.2, 1]),
      v(`${P}PlanEvents`, ["grab"]),
      v(`${P}PlanEventMeters`, [0.05]),
      v(`${P}PlanSeconds`, 0.2),
      v("/Telemetry/PathFollower/Active", "Hop"),
    ])
      r.onValue(value);
    loop(r, 0, 0);
    loop(r, 0.02, 0.05, [v("/Telemetry/PathFollower/EventsFired", ["grab"])]);
    loop(r, 0.2, 0.1, [v(`${P}Goal`, [0.1, 0, 0]), v("/Telemetry/PathFollower/Active", "")]);
    expect(r.runs).toHaveLength(1);
    const run = r.runs[0]!;
    expect(run.finished).toBe(true);
    expect(run.samples.map((x) => x.t)).toEqual([0, 0.02, 0.2]); // the first and the last loop too
    expect(run.samples[1]!.pose).toEqual({ x: 0.05, y: 0, heading: 0 });
    expect(run.samples[1]!.plannedSpeed).toBe(1.1);
    expect(run.events).toEqual([{ name: "grab", t: 0.02 }]);
    expect(run.plan?.x).toEqual([0, 0.1]);
    const sum = summarise(run);
    expect(sum.loops).toBe(3);
    expect(sum.endError).toBeCloseTo(0, 12);
    expect(sum.eventsExpected).toEqual(["grab"]);
    expect(sum.eventsFired).toEqual(["grab"]);
    expect(sum.crossTrackMax).toBeCloseTo(0.002, 12);
    expect(sampleAt(run, 0.1)!.t).toBe(0.02);
    // a file and back
    const back = importRun(exportRun(run));
    expect(back.samples).toEqual(run.samples);
    expect(() => importRun("{}")).toThrow(/not a recorded run/);
  });

  it("keeps the last few runs", () => {
    const r = new Recorder("/Telemetry/Robot/Pose", 2);
    for (let i = 0; i < 3; i++) {
      r.onValue(v("/Telemetry/PathFollower/Active", "Hop"));
      loop(r, 0, 0);
      r.onValue(v("/Telemetry/PathFollower/Active", ""));
      r.onFrameEnd();
    }
    expect(r.runs).toHaveLength(2);
    expect(r.runs.every((x) => x.finished && x.samples.length === 1)).toBe(true);
  });
});
