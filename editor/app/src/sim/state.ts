// The sim preview's settings and its latest result. When the preview is on, the worker gets a new run
// whenever the active path, the robot or a sim setting changes. The drivetrain (latency, lag, grip) is the
// project's (Settings > Sim preview, saved in the editor's sidecar file, not robot.json: it describes the
// sim, not the robot); whether the preview is on and the shove are this browser's.

import { DEFAULT_DRIVE, GRAVITY, type DriveSettings, type PathSpec, type Shove } from "../../../core/src/index.ts";
import type { Project, SimPreviewSettings } from "../model/project.ts";
import { activePath, editor } from "../state/editor.ts";
import { createStore } from "../state/store.ts";
import type { SimInput, SimResult } from "./preview.ts";
import { SimRunner } from "./runner.ts";

export interface SimSettings {
  /** Whether the preview runs and is drawn. */
  readonly enabled: boolean;
  /** A hit at a chosen time, m/s in the blue frame. */
  readonly shoveOn: boolean;
  readonly shoveAtSeconds: number;
  readonly shoveDvx: number;
  readonly shoveDvy: number;
}

export const DEFAULT_SIM_SETTINGS: SimSettings = {
  enabled: false,
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
      if (typeof got === typeof v) s[k] = got;
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

/** The project's sim drivetrain as the core's DriveSettings. */
export function driveOf(d: SimPreviewSettings): DriveSettings {
  return {
    latencySeconds: d.latencyMs / 1000,
    lagSeconds: Math.max(0.001, d.lagMs / 1000),
    substepSeconds: DEFAULT_DRIVE.substepSeconds,
    ...(d.gripMu !== null ? { gripMetersPerSecondSquared: d.gripMu * GRAVITY } : {}),
  };
}

export function shoveOf(s: SimSettings): Shove | null {
  return s.shoveOn ? { atSeconds: s.shoveAtSeconds, dvx: s.shoveDvx, dvy: s.shoveDvy } : null;
}

export function simInput(project: Project, spec: PathSpec, s: SimSettings): SimInput {
  return { spec, robot: project.robot.config, drive: driveOf(project.settings.simPreview), shove: shoveOf(s) };
}

/** Starts feeding the sim worker; returns a stop function. */
export function startSim(runner = new SimRunner()): () => void {
  let last: { spec: PathSpec; robot: unknown; drive: unknown; settings: SimSettings } | null = null;
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
    const drive = s.project.settings.simPreview;
    if (last && last.spec === spec && last.robot === s.project.robot && last.drive === drive && last.settings === settings) return;
    last = { spec, robot: s.project.robot, drive, settings };
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
