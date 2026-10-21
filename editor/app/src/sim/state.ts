// The sim preview's settings and its latest result. When the preview is on, the worker gets a new run
// whenever the active path, the robot or a sim setting changes. The drivetrain settings are the editor's
// own (kept in this browser), not part of robot.json: they describe the sim, not the robot.

import { DEFAULT_DRIVE, GRAVITY, type DriveSettings, type PathSpec, type Shove } from "../../../core/src/index.ts";
import type { Project } from "../model/project.ts";
import { activePath, editor } from "../state/editor.ts";
import { createStore } from "../state/store.ts";
import type { SimInput, SimResult } from "./preview.ts";
import { SimRunner } from "./runner.ts";

export interface SimSettings {
  /** Whether the preview runs and is drawn. */
  readonly enabled: boolean;
  /** Command latency and velocity lag, seconds; the end-to-end suite's 0.02 and 0.04 by default. */
  readonly latencySeconds: number;
  readonly lagSeconds: number;
  /** The floor's grip as a friction coefficient (Infinity = no cap, as in the end-to-end suite). */
  readonly gripMu: number;
  /** A hit at a chosen time, m/s in the blue frame. */
  readonly shoveOn: boolean;
  readonly shoveAtSeconds: number;
  readonly shoveDvx: number;
  readonly shoveDvy: number;
}

export const DEFAULT_SIM_SETTINGS: SimSettings = {
  enabled: false,
  latencySeconds: DEFAULT_DRIVE.latencySeconds,
  lagSeconds: DEFAULT_DRIVE.lagSeconds,
  gripMu: Infinity,
  shoveOn: false,
  shoveAtSeconds: 1.0,
  shoveDvx: 0.0,
  shoveDvy: 1.5,
};

const STORAGE_KEY = "pathEditor.simPreview";

function loadSettings(): SimSettings {
  try {
    const saved = JSON.parse(globalThis.localStorage?.getItem(STORAGE_KEY) ?? "null") as Partial<Record<keyof SimSettings, unknown>> | null;
    if (!saved || typeof saved !== "object") return DEFAULT_SIM_SETTINGS;
    const s: Record<string, unknown> = { ...DEFAULT_SIM_SETTINGS };
    for (const [k, v] of Object.entries(DEFAULT_SIM_SETTINGS)) {
      const got = saved[k as keyof SimSettings];
      // JSON has no Infinity: a saved null grip means none
      if (k === "gripMu" && got === null) s[k] = Infinity;
      else if (typeof got === typeof v) s[k] = got;
    }
    return s as unknown as SimSettings;
  } catch {
    return DEFAULT_SIM_SETTINGS;
  }
}

export const simSettings = createStore<SimSettings>(loadSettings());

simSettings.subscribe(() => {
  try {
    globalThis.localStorage?.setItem(STORAGE_KEY, JSON.stringify(simSettings.get()));
  } catch {
    // private window or storage blocked: the settings last for this session
  }
});

export interface SimState {
  /** What the result is for; it may lag the document while a run is in flight. */
  readonly input: SimInput | null;
  readonly result: SimResult | null;
}

export const sim = createStore<SimState>({ input: null, result: null });

export function driveOf(s: SimSettings): DriveSettings {
  return {
    latencySeconds: s.latencySeconds,
    lagSeconds: s.lagSeconds,
    substepSeconds: DEFAULT_DRIVE.substepSeconds,
    ...(Number.isFinite(s.gripMu) ? { gripMetersPerSecondSquared: s.gripMu * GRAVITY } : {}),
  };
}

export function shoveOf(s: SimSettings): Shove | null {
  return s.shoveOn ? { atSeconds: s.shoveAtSeconds, dvx: s.shoveDvx, dvy: s.shoveDvy } : null;
}

export function simInput(project: Project, spec: PathSpec, s: SimSettings): SimInput {
  return { spec, robot: project.robot.config, drive: driveOf(s), shove: shoveOf(s) };
}

/** Starts feeding the sim worker; returns a stop function. */
export function startSim(runner = new SimRunner()): () => void {
  let last: { spec: PathSpec; robot: unknown; settings: SimSettings } | null = null;
  const update = () => {
    const settings = simSettings.get();
    if (!settings.enabled) {
      if (last) {
        last = null;
        runner.clear();
        sim.set({ input: null, result: null });
      }
      return;
    }
    const s = editor.get();
    const spec = activePath(s).spec;
    if (last && last.spec === spec && last.robot === s.project.robot && last.settings === settings) return;
    last = { spec, robot: s.project.robot, settings };
    const input = simInput(s.project, spec, settings);
    runner.request(input, (result) => sim.set({ input, result }));
  };
  update();
  const unsubscribers = [editor.subscribe(update), simSettings.subscribe(update)];
  return () => {
    for (const u of unsubscribers) u();
    runner.dispose();
  };
}
