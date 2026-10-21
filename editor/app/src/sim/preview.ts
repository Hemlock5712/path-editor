// The sim preview: the follower run closed-loop on the simulated drivetrain (core's runFollowerSim, the
// same code the golden traces check against Java), and what the editor shows of it. Pure, so it runs in
// the worker and in tests alike.
//
// The preview assumes the floor grips: slip sensing and adaptive grip are Java-only. The optional grip cap
// makes the simulated chassis slide when asked for more, but the follower doesn't adapt to it.

import {
  eventMeters,
  followerConfig,
  LOOP_SECONDS,
  Path,
  responseDelaySeconds,
  runFollowerSim,
  wrap,
  type DriveSettings,
  type PathSpec,
  type Phase,
  type RobotConfig,
  type Shove,
} from "../../../core/src/index.ts";

export interface SimInput {
  readonly spec: PathSpec;
  readonly robot: RobotConfig;
  readonly drive: DriveSettings;
  /** A hit during the run, field frame (blue), or null. */
  readonly shove: Shove | null;
}

/** Seconds simulated after the follower finishes (as the golden traces do). */
export const AFTER_SECONDS = 0.5;
/** Seconds allowed beyond the plan before the run is cut off. */
export const OVERRUN_SECONDS = 6.0;

/** One event: where it was authored and where the simulated robot was when it fired. */
export interface SimEvent {
  readonly name: string;
  /** Authored: metres along the path, the plan's time there, and the point (blue frame). */
  readonly s: number;
  readonly planT: number;
  readonly x: number;
  readonly y: number;
  /** The loop it fired in (-1 = it never fired), and the robot then. */
  readonly loop: number;
  readonly t: number;
  readonly firedX: number;
  readonly firedY: number;
  /** How far the robot was from the authored point when it fired, metres. */
  readonly offsetMeters: number;
}

export interface SimPreview {
  readonly ok: true;
  /** Loops every 20 ms: the robot's pose, the follower's progress and cross-track error. */
  readonly t: number[];
  readonly x: number[];
  readonly y: number[];
  readonly heading: number[];
  readonly s: number[];
  /**
   * Metres off the path, positive left: the follower's cross-track error while tracking, then (it reports
   * none once it hands over to the goal) the robot's offset across the path where its progress stopped.
   */
  readonly crossTrack: number[];
  readonly phase: Phase[];
  readonly planSeconds: number;
  /** When the follower finished (NaN = it didn't, within the plan plus OVERRUN_SECONDS). */
  readonly finishSeconds: number;
  /** Sim minus plan, seconds (NaN if it didn't finish). */
  readonly gapSeconds: number;
  /** When the terminal phase began and how long it took to settle on the goal (NaN for a rolling end). */
  readonly terminalSeconds: number;
  readonly settleSeconds: number;
  /** The largest and RMS crossTrack over the run. */
  readonly maxCrossTrackMeters: number;
  readonly rmsCrossTrackMeters: number;
  /** At the end of the run: distance from the goal, metres, and heading error, radians. */
  readonly endErrorMeters: number;
  readonly endHeadingErrorRadians: number;
  readonly events: readonly SimEvent[];
  readonly shove: Shove | null;
  readonly millis: number;
}

export interface SimFailure {
  readonly ok: false;
  readonly error: string;
}

export type SimResult = SimPreview | SimFailure;

export function simulate(input: SimInput): SimResult {
  const t0 = performance.now();
  let path: Path;
  try {
    path = Path.build(input.spec, input.robot);
  } catch (e) {
    return { ok: false, error: e instanceof Error ? e.message : String(e) };
  }
  const drive = input.drive;
  const config = followerConfig(input.robot, { latencySeconds: responseDelaySeconds(drive) });
  const planSeconds = path.totalTimeSeconds;
  const trace = runFollowerSim(path, config, drive, "BLUE", input.shove, AFTER_SECONDS, planSeconds + OVERRUN_SECONDS);

  const n = trace.t.length;
  const finishSeconds = trace.finishLoop >= 0 ? trace.finishLoop * LOOP_SECONDS : Number.NaN;
  const terminalLoop = trace.phase.indexOf("TERMINAL");
  const terminalSeconds = terminalLoop >= 0 ? terminalLoop * LOOP_SECONDS : Number.NaN;
  let max = 0;
  let sumSq = 0;
  const crossTrack = trace.crossTrack.map((e, i) => {
    if (trace.phase[i] === "TRACKING") return e;
    const p = path.sample(trace.s[i]!);
    return (trace.y[i]! - p.yMeters) * p.tangentX - (trace.x[i]! - p.xMeters) * p.tangentY;
  });
  for (const e of crossTrack) {
    max = Math.max(max, Math.abs(e));
    sumSq += e * e;
  }
  const goal = path.sample(path.lengthMeters);
  const last = n - 1;

  // Events fire in order along the path; two events may share a name, so pair them up name by name in that order
  const authoredS = input.spec.events.map((e) => Math.min(path.lengthMeters, Math.max(0, eventMeters(e, path.geometry))));
  const firedLoop = new Map<number, number>();
  for (const name of new Set(input.spec.events.map((e) => e.name))) {
    const mine = input.spec.events.flatMap((e, i) => (e.name === name ? [i] : [])).sort((a, b) => authoredS[a]! - authoredS[b]!);
    const fires = trace.events.filter((f) => f.name === name);
    mine.forEach((i, k) => firedLoop.set(i, fires[k]?.loop ?? -1));
  }
  const events = input.spec.events.map((e, i): SimEvent => {
    const s = authoredS[i]!;
    const p = path.sample(s);
    const loop = firedLoop.get(i) ?? -1;
    const firedX = loop >= 0 ? trace.x[loop]! : Number.NaN;
    const firedY = loop >= 0 ? trace.y[loop]! : Number.NaN;
    return {
      name: e.name,
      s,
      planT: p.seconds,
      x: p.xMeters,
      y: p.yMeters,
      loop,
      t: loop >= 0 ? loop * LOOP_SECONDS : Number.NaN,
      firedX,
      firedY,
      offsetMeters: loop >= 0 ? Math.hypot(firedX - p.xMeters, firedY - p.yMeters) : Number.NaN,
    };
  });

  return {
    ok: true,
    t: trace.t,
    x: trace.x,
    y: trace.y,
    heading: trace.heading,
    s: trace.s,
    crossTrack,
    phase: trace.phase,
    planSeconds,
    finishSeconds,
    gapSeconds: finishSeconds - planSeconds,
    terminalSeconds,
    settleSeconds: finishSeconds - terminalSeconds,
    maxCrossTrackMeters: max,
    rmsCrossTrackMeters: n > 0 ? Math.sqrt(sumSq / n) : 0,
    endErrorMeters: Math.hypot(trace.x[last]! - goal.xMeters, trace.y[last]! - goal.yMeters),
    endHeadingErrorRadians: wrap(trace.heading[last]! - goal.headingRadians),
    events,
    shove: input.shove,
    millis: performance.now() - t0,
  };
}

/** The pose at a time, from the nearest loop (clamped to the run). */
export function simPoseAt(p: SimPreview, t: number): { x: number; y: number; heading: number } {
  const i = Math.max(0, Math.min(p.t.length - 1, Math.round(t / LOOP_SECONDS)));
  return { x: p.x[i]!, y: p.y[i]!, heading: p.heading[i]! };
}
