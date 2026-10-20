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
  readonly result: AnalysisResult | null;
}

export const analysis = createStore<AnalysisState>({ spec: null, result: null });

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
    last = { spec, robot: s.project.robot, field: s.project.field };
    runner.request(analysisInput(s.project, spec), (result) => analysis.set({ spec, result }));
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
