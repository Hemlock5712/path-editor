// The active path's analysis (from the worker) and the playback clock. The worker gets a new job
// whenever the active path, the robot or the field changes; a drag sends one per frame and the
// runner keeps only the newest pending one.

import { PathGeometry, pureRotation, type PathSpec } from "../../../core/src/index.ts";
import type { AnalysisInput } from "../compute/analysis.ts";
import { AnalysisRunner, type AnalysisResult } from "../compute/runner.ts";
import type { Project } from "../model/project.ts";
import { activePath, editor } from "./editor.ts";
import { createStore } from "./store.ts";

export interface AnalysisState {
  /** The spec the result is for (it may lag the document by a frame while dragging). */
  readonly spec: PathSpec | null;
  /** The robot and field it was planned with. */
  readonly robot: Project["robot"] | null;
  readonly field: Project["field"] | null;
  readonly result: AnalysisResult | null;
}

export const analysis = createStore<AnalysisState>({ spec: null, robot: null, field: null, result: null });

export interface PlaybackState {
  readonly playing: boolean;
  /** Seconds into the plan. */
  readonly t: number;
  /** Where the pointer is on a chart or the path, metres (NaN = nowhere). */
  readonly hoverS: number;
}

export const playback = createStore<PlaybackState>({ playing: false, t: 0, hoverS: Number.NaN });

export function analysisInput(project: Project, spec: PathSpec): AnalysisInput {
  return {
    spec,
    robot: project.robot.config,
    bumperLengthMeters: project.robot.bumperLengthMeters,
    bumperWidthMeters: project.robot.bumperWidthMeters,
    fieldLengthMeters: project.field.lengthMeters,
    fieldWidthMeters: project.field.widthMeters,
    obstacles: project.field.obstacles,
  };
}

/** Starts feeding the worker; returns a stop function. */
export function startAnalysis(runner = new AnalysisRunner()): () => void {
  let last: { spec: PathSpec; robot: unknown; field: unknown } | null = null;
  const update = () => {
    const s = editor.get();
    const spec = activePath(s).spec;
    if (last && last.spec === spec && last.robot === s.project.robot && last.field === s.project.field) return;
    const { robot, field } = s.project;
    last = { spec, robot, field };
    runner.request(analysisInput(s.project, spec), (result) => analysis.set({ spec, robot, field, result }));
  };
  update();
  const unsubscribe = editor.subscribe(update);
  return () => {
    unsubscribe();
    runner.dispose();
  };
}

const geometries = new WeakMap<PathSpec, PathGeometry | null>();

/**
 * The spline of a spec, built on this thread: cheap (no profile), and needed at once for hit tests,
 * snapping clicks to the path and drawing the other tabs. Null if the waypoints can't make a path.
 */
export function geometryOf(spec: PathSpec): PathGeometry | null {
  let g = geometries.get(spec);
  if (g === undefined) {
    try {
      g = PathGeometry.build(pureRotation(spec).waypoints);
    } catch {
      g = null;
    }
    geometries.set(spec, g);
  }
  return g;
}

/** The preview's headline numbers for one path, as the end-to-end suite compares them with the robot. */
export interface PreviewNumbers {
  readonly name: string;
  readonly lengthMeters: number;
  readonly totalTimeSeconds: number;
  readonly peakMetersPerSecond: number;
  readonly startMetersPerSecond: number;
  readonly endMetersPerSecond: number;
  /** In file order: where each event is along the path and when the plan reaches it. */
  readonly events: readonly { readonly name: string; readonly s: number; readonly t: number }[];
}

/** The active path's preview, once the worker has planned exactly what the document holds now. */
export function currentPreview(): Promise<PreviewNumbers> {
  return new Promise((resolve, reject) => {
    const check = () => {
      const s = editor.get();
      const path = activePath(s);
      const a = analysis.get();
      if (a.spec !== path.spec || a.robot !== s.project.robot || a.field !== s.project.field || !a.result) return false;
      for (const u of unsubscribers) u();
      if (!a.result.ok) {
        reject(new Error(`${path.name} can't be planned: ${a.result.error}`));
        return true;
      }
      const r = a.result;
      resolve({
        name: path.name,
        lengthMeters: r.lengthMeters,
        totalTimeSeconds: r.totalTimeSeconds,
        peakMetersPerSecond: Math.max(...r.stations.v),
        startMetersPerSecond: path.spec.startMetersPerSecond,
        endMetersPerSecond: path.spec.endMetersPerSecond,
        events: r.events.map((e) => ({ name: e.name, s: e.s, t: e.t })),
      });
      return true;
    };
    const unsubscribers = [analysis.subscribe(() => void check()), editor.subscribe(() => void check())];
    check();
  });
}
