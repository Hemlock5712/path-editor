// The bench's export for the editor (python bench/run.py --export-editor → bench/out/editor_bench.json):
// per pose source, course and method the mean and worst of a few metrics, each method's authored
// geometry, and true traces for some seeds. Read-only here.

export const BENCH_SCHEMA = 1;

export interface Stat {
  readonly mean: number | null;
  readonly worst: number | null;
  /** Runs where the metric didn't exist (e.g. never settled). */
  readonly nan: number;
}

export interface BenchData {
  readonly schema: number;
  readonly source: string;
  readonly meta: {
    readonly seeds: readonly number[];
    readonly courses: readonly string[];
    readonly poses: readonly string[];
    readonly tracePose: string;
    readonly traceSeeds: readonly number[];
  };
  readonly metrics: readonly { readonly key: string; readonly label: string; readonly unit: string }[];
  readonly methods: readonly { readonly key: string; readonly label: string }[];
  readonly courses: readonly { readonly key: string; readonly description: string }[];
  /** pose source → course → method → metric → stat (plus n, the run count) */
  readonly runs: Readonly<Record<string, Readonly<Record<string, Readonly<Record<string, Readonly<Record<string, Stat | number>>>>>>>>;
  /** course → method (or "neutral") → [[x, y], ...] */
  readonly geometry: Readonly<Record<string, Readonly<Record<string, readonly (readonly number[])[]>>>>;
  /** course → seed → method → [[t, x, y, heading], ...] */
  readonly traces: Readonly<Record<string, Readonly<Record<string, Readonly<Record<string, readonly (readonly number[])[]>>>>>>;
}

/** Parses and checks an export; throws with a reason. */
export function parseBench(text: string): BenchData {
  const d = JSON.parse(text) as Partial<BenchData>;
  if (d.schema !== BENCH_SCHEMA) {
    throw new Error(
      d.schema === undefined
        ? "not a bench export (run python bench/run.py --export-editor)"
        : `bench export schema ${String(d.schema)} (this editor reads ${BENCH_SCHEMA})`,
    );
  }
  for (const key of ["meta", "metrics", "methods", "courses", "runs", "geometry", "traces"] as const) {
    if (d[key] === undefined) throw new Error(`bench export has no '${key}'`);
  }
  return d as BenchData;
}

/** The methods shown on the field until the user picks: ours and each competitor's main entry. */
export const DEFAULT_SHOWN = ["ours_v2", "choreo", "pathplanner_tuned", "bline_tuned", "old_followpath"];

/** A colour per method, by family, so ours and the others read apart at a glance. */
export function methodColour(key: string): string {
  if (key.startsWith("ours")) return key === "ours_v2" ? "#38bdf8" : "#7dd3fc";
  if (key.startsWith("choreo")) return key === "choreo" ? "#fb923c" : "#fdba74";
  if (key.startsWith("pathplanner")) return key === "pathplanner_tuned" ? "#4ade80" : "#86efac";
  if (key.startsWith("bline")) return key === "bline_tuned" ? "#f472b6" : "#f9a8d4";
  if (key === "old_followpath") return "#facc15";
  return "#cbd5e1";
}

/**
 * The bench's courses start at (0, 0) in their own frame; this shift puts a course's authored path in
 * the middle of the field so it's drawn on it.
 */
export function courseOffset(data: BenchData, course: string): [number, number] {
  const pts = data.geometry[course]?.neutral ?? [];
  if (pts.length === 0) return [0, 0];
  let [x0, y0, x1, y1] = [Infinity, Infinity, -Infinity, -Infinity];
  for (const [x, y] of pts) {
    x0 = Math.min(x0, x!);
    x1 = Math.max(x1, x!);
    y0 = Math.min(y0, y!);
    y1 = Math.max(y1, y!);
  }
  return [-(x0 + x1) / 2, -(y0 + y1) / 2];
}

export function stat(data: BenchData, pose: string, course: string, method: string, metric: string): Stat | null {
  const v = data.runs[pose]?.[course]?.[method]?.[metric];
  return typeof v === "object" && v !== null ? v : null;
}

/** For each metric, the method with the lowest mean on this course (lower is better for every metric shown). */
export function bestByMetric(data: BenchData, pose: string, course: string, methods: readonly string[]): Record<string, string> {
  const best: Record<string, string> = {};
  for (const { key } of data.metrics) {
    let low = Infinity;
    for (const m of methods) {
      const v = stat(data, pose, course, m, key)?.mean;
      if (v !== null && v !== undefined && v < low) {
        low = v;
        best[key] = m;
      }
    }
  }
  return best;
}
