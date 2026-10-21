// Records each run of the path follower from its NT4 telemetry (PathTelemetry): one sample per loop, the
// plan the robot published at the start, the events as they fired, and the goal. A run starts when
// /Telemetry/PathFollower/Active names a path and ends when it goes back to "". The Telemetry page
// draws and graphs the runs, and they export to JSON and import back.
//
// A loop's values arrive together (the robot flushes them), so a sample is taken at the end of each
// incoming frame in which the follower's loop values ("Elapsed", logged last) changed, with the newest of
// everything: the robot's pose included, whichever order it came in.

import { poseOf, type Nt4Value } from "./client.ts";

export interface Pose {
  readonly x: number;
  readonly y: number;
  readonly heading: number;
}

/** One loop of a run. Poses are in the field frame the robot reports. */
export interface TrackSample {
  /** Seconds since the run started (the follower's own clock). */
  readonly t: number;
  /** Progress along the path, m. */
  readonly s: number;
  readonly pose: Pose | null;
  readonly reference: Pose | null;
  readonly crossTrack: number;
  readonly alongTrack: number;
  readonly headingError: number;
  readonly speed: number;
  readonly plannedSpeed: number;
  readonly omega: number;
  readonly plannedOmega: number;
  readonly phase: string;
}

/** The plan the robot published at the start: samples every 5 cm, its events, its time. */
export interface RunPlan {
  readonly s: readonly number[];
  readonly x: readonly number[];
  readonly y: readonly number[];
  readonly heading: readonly number[];
  readonly t: readonly number[];
  readonly v: readonly number[];
  readonly events: readonly { readonly name: string; readonly s: number }[];
  readonly seconds: number;
}

export interface RecordedRun {
  readonly id: string;
  readonly path: string;
  /** Wall clock at the start, ms (for the list). */
  readonly startedAt: number;
  plan: RunPlan | null;
  readonly samples: TrackSample[];
  /** Events in the order they fired, with the run time they fired at. */
  readonly events: { readonly name: string; readonly t: number }[];
  goal: Pose | null;
  /** Ended with Active going back to "" (else the connection dropped, or it is still running). */
  finished: boolean;
}

export const RUN_FILE_SCHEMA = 1;

const FOLLOWER = "/Telemetry/PathFollower/";
const PATHS = "/Telemetry/Path/";

const NUMERIC = {
  Progress: "s",
  CrossTrack: "crossTrack",
  AlongTrack: "alongTrack",
  HeadingError: "headingError",
  Speed: "speed",
  PlannedSpeed: "plannedSpeed",
  Omega: "omega",
  PlannedOmega: "plannedOmega",
  Elapsed: "t",
} as const;

function planFrom(flat: readonly number[], names: readonly string[], meters: readonly number[], seconds: number): RunPlan {
  const col = (k: number) => flat.filter((_, i) => i % 6 === k);
  return {
    s: col(0),
    x: col(1),
    y: col(2),
    heading: col(3),
    t: col(4),
    v: col(5),
    events: names.map((name, i) => ({ name, s: meters[i] ?? Number.NaN })),
    seconds,
  };
}

export class Recorder {
  /** Runs, oldest first; at most `keep`. */
  readonly runs: RecordedRun[] = [];
  private current: RecordedRun | null = null;
  private readonly latest = new Map<string, number | string>();
  private reference: Pose | null = null;
  private goal: Pose | null = null;
  private pose: Pose | null = null;
  private loopChanged = false;
  private pendingActive: string | null = null;
  private readonly plans = new Map<string, { flat?: number[]; names?: string[]; meters?: number[]; seconds?: number }>();
  private nextId = 1;

  constructor(
    private readonly poseTopic: string,
    private readonly keep = 10,
    private readonly onChange: () => void = () => undefined,
  ) {}

  get active(): RecordedRun | null {
    return this.current;
  }

  /** Every NT4 value, in order. */
  onValue(v: Nt4Value): void {
    const name = v.topic.name;
    const value = v.value;
    if (name === this.poseTopic) {
      this.pose = poseOf(v.topic.type, value) ?? this.pose;
      return;
    }
    if (name === `${FOLLOWER}Active` && typeof value === "string") {
      this.pendingActive = value; // applied at the end of the frame, after this frame's sample
      return;
    }
    if (name === `${FOLLOWER}EventsFired` && Array.isArray(value)) {
      const run = this.current;
      if (run) for (const e of value.slice(run.events.length)) run.events.push({ name: String(e), t: Number(this.latest.get("Elapsed") ?? 0) });
      return;
    }
    if (!name.startsWith(PATHS)) return;
    const rest = name.slice(PATHS.length);
    const slash = rest.lastIndexOf("/");
    const path = rest.slice(0, slash);
    const key = rest.slice(slash + 1);
    // the plan, published once per start (before Active names the path)
    if (key === "Plan" || key === "PlanEvents" || key === "PlanEventMeters" || key === "PlanSeconds") {
      const plan = this.plans.get(path) ?? {};
      if (key === "Plan" && Array.isArray(value)) plan.flat = value.map(Number);
      else if (key === "PlanEvents" && Array.isArray(value)) plan.names = value.map(String);
      else if (key === "PlanEventMeters" && Array.isArray(value)) plan.meters = value.map(Number);
      else if (key === "PlanSeconds" && typeof value === "number") plan.seconds = value;
      this.plans.set(path, plan);
      if (this.current?.path === path) this.current.plan = this.planOf(path);
      return;
    }
    if (this.current && path !== this.current.path) return;
    if (key === "Reference") this.reference = poseOf(v.topic.type, value) ?? this.reference;
    else if (key === "Goal") {
      this.goal = poseOf(v.topic.type, value) ?? this.goal;
      if (this.current) this.current.goal = this.goal;
    } else if (key === "Phase" && typeof value === "string") this.latest.set("Phase", value);
    else if (key in NUMERIC && typeof value === "number") {
      this.latest.set(key, value);
      if (key === "Elapsed") this.loopChanged = true;
    }
  }

  /**
   * The end of an incoming frame. A path that starts in this frame starts before its first loop's
   * sample; one that ends gets its last loop's sample first.
   */
  onFrameEnd(): void {
    const active = this.pendingActive;
    this.pendingActive = null;
    if (active !== null && !this.current && active) this.begin(active);
    if (this.loopChanged && this.current) this.sample(this.current);
    this.loopChanged = false;
    if (active !== null && this.current && active !== this.current.path) {
      this.current.finished = true;
      this.current = null;
      if (active) this.begin(active);
    }
    this.onChange();
  }

  private sample(run: RecordedRun): void {
    const n = (k: keyof typeof NUMERIC) => {
      const v = this.latest.get(k);
      return typeof v === "number" ? v : Number.NaN;
    };
    run.samples.push({
      t: n("Elapsed"),
      s: n("Progress"),
      pose: this.pose,
      reference: this.reference,
      crossTrack: n("CrossTrack"),
      alongTrack: n("AlongTrack"),
      headingError: n("HeadingError"),
      speed: n("Speed"),
      plannedSpeed: n("PlannedSpeed"),
      omega: n("Omega"),
      plannedOmega: n("PlannedOmega"),
      phase: String(this.latest.get("Phase") ?? ""),
    });
  }

  private planOf(path: string): RunPlan | null {
    const p = this.plans.get(path);
    if (!p?.flat || p.flat.length < 12) return null;
    return planFrom(p.flat, p.names ?? [], p.meters ?? [], p.seconds ?? Number.NaN);
  }

  private begin(path: string): void {
    const run: RecordedRun = {
      id: `run-${Date.now().toString(36)}-${this.nextId++}`,
      path,
      startedAt: Date.now(),
      plan: this.planOf(path),
      samples: [],
      events: [],
      goal: this.goal,
      finished: false,
    };
    this.current = run;
    this.runs.push(run);
    while (this.runs.length > this.keep) this.runs.shift();
  }
}

// ------------------------------------------------------------------------------ reading a run

export interface RunSummary {
  readonly loops: number;
  readonly seconds: number;
  readonly planSeconds: number;
  readonly crossTrackRms: number;
  readonly crossTrackMax: number;
  /** The last pose against the goal. */
  readonly endError: number;
  readonly endHeadingError: number;
  readonly eventsExpected: readonly string[];
  readonly eventsFired: readonly string[];
}

const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

export function summarise(run: RecordedRun): RunSummary {
  const cts = run.samples.map((p) => p.crossTrack).filter(Number.isFinite);
  const last = [...run.samples].reverse().find((p) => p.pose !== null);
  const goal = run.goal;
  return {
    loops: run.samples.length,
    seconds: run.samples.at(-1)?.t ?? Number.NaN,
    planSeconds: run.plan?.seconds ?? Number.NaN,
    crossTrackRms: cts.length ? Math.sqrt(cts.reduce((a, c) => a + c * c, 0) / cts.length) : Number.NaN,
    crossTrackMax: cts.length ? Math.max(...cts.map(Math.abs)) : Number.NaN,
    endError: last?.pose && goal ? Math.hypot(last.pose.x - goal.x, last.pose.y - goal.y) : Number.NaN,
    endHeadingError: last?.pose && goal ? wrap(goal.heading - last.pose.heading) : Number.NaN,
    eventsExpected: run.plan?.events.map((e) => e.name) ?? [],
    eventsFired: run.events.map((e) => e.name),
  };
}

/** A run as a file. */
export function exportRun(run: RecordedRun): string {
  return JSON.stringify({ schema: RUN_FILE_SCHEMA, run }, null, 1);
}

/** A run from a file; throws with a reason. */
export function importRun(text: string): RecordedRun {
  const d = JSON.parse(text) as { schema?: number; run?: RecordedRun };
  if (d.schema !== RUN_FILE_SCHEMA || !d.run || !Array.isArray(d.run.samples)) throw new Error("not a recorded run (export one from the Telemetry page)");
  return { ...d.run, id: `${d.run.id}-imported-${Date.now().toString(36)}` };
}

/** The sample at or just before time t (runs are in time order). */
export function sampleAt(run: RecordedRun, t: number): TrackSample | null {
  const xs = run.samples;
  if (xs.length === 0) return null;
  let lo = 0;
  let hi = xs.length - 1;
  if (t <= xs[0]!.t) return xs[0]!;
  while (lo < hi) {
    const mid = (lo + hi + 1) >> 1;
    if (xs[mid]!.t <= t) lo = mid;
    else hi = mid - 1;
  }
  return xs[lo]!;
}
