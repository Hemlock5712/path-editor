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
  readonly courses: readonly { readonly key: string; readonly description: string; readonly look?: string }[];
  /** pose source → course → method → metric → stat (plus n, the run count) */
  readonly runs: Readonly<Record<string, Readonly<Record<string, Readonly<Record<string, Readonly<Record<string, Stat | number>>>>>>>>;
  /** course → method (or "neutral") → [[x, y], ...] */
  readonly geometry: Readonly<Record<string, Readonly<Record<string, readonly (readonly number[])[]>>>>;
  /** course → seed → method → [[t, x, y, heading], ...] */
  readonly traces: Readonly<Record<string, Readonly<Record<string, Readonly<Record<string, readonly (readonly number[])[]>>>>>>;
  /** The PathPlanner comparison (run.py --pp-compare), when it has been run: rows only, and the 2026 routes. */
  readonly ppcompare?: {
    readonly source: string;
    readonly meta: { readonly seeds: readonly number[]; readonly poses: readonly string[] };
    readonly metrics: BenchData["metrics"];
    readonly methods: BenchData["methods"];
    readonly targets: readonly {
      readonly key: string;
      readonly description: string;
      readonly look: string;
      readonly onField: boolean;
      readonly legs: readonly (readonly (readonly number[])[])[];
    }[];
    readonly runs: BenchData["runs"];
  };
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

/** A colour per method, the same everywhere (field, scoreboard, tables, legend); ours is the accent. */
const COLOURS: Readonly<Record<string, string>> = {
  ours_v2: "#4ca6ff",
  ours_tuned: "#8cc8ff",
  ours_noadapt: "#2f6fb0",
  ours_optimised: "#a5d8ff",
  pathplanner_tuned: "#4ade80",
  pathplanner_hold: "#22a35a",
  pathplanner: "#86efac",
  pathplanner_hold_cof09: "#3f7d55",
  pp2027_tuned: "#4ade80",
  pp2027_hold: "#22a35a",
  pp2027_optimised: "#bbf7d0",
  pp_path2_tuned: "#2dd4bf",
  pp_path2: "#5eead4",
  pp_path2_optimised: "#99f6e4",
  choreo: "#fb923c",
  choreo_hybrid: "#fdba74",
  bline_tuned: "#f472b6",
  bline_tuned_caps: "#f9a8d4",
  bline_readme: "#be185d",
  bline_readme_r45: "#db2777",
  bline_caps: "#fbcfe8",
  old_followpath: "#facc15",
  drive_to_pose_chain: "#a78bfa",
};

export function methodColour(key: string): string {
  return COLOURS[key] ?? "#cbd5e1";
}

// ------------------------------------------------------------------------------ reading it fairly

/** Pose sources, by what they mean for the result. */
export const POSE_LABELS: Readonly<Record<string, string>> = {
  truth: "Perfect pose: the follower alone",
  odom: "Wheel odometry + gyro (no vision)",
  slipodom: "Slip-aware odometry + gyro + IMU",
};

/** The methods ticked first: ours, then each rival's best-tuned entry. The rest are "more methods". */
export const PRIMARY_METHODS = ["ours_v2", "pathplanner_tuned", "pp_path2_tuned", "pp2027_tuned", "choreo", "bline_tuned"];

/** Final error allowed for a run to count, cm (the settle box is 2 cm; this allows for drift after). */
export const END_TOLERANCE_CM = 5;

export const VALID_RULE =
  "Valid means every seed settled (within 2 cm and 1°, held 0.25 s) and ended within 5 cm. Only valid rows are ranked or bold.";

/** One table: what the Bench page shows for the bench's courses, or for the PathPlanner comparison. */
export interface BenchTable {
  readonly methods: readonly { readonly key: string; readonly label: string }[];
  readonly metrics: readonly { readonly key: string; readonly label: string; readonly unit: string }[];
  readonly targets: readonly BenchTarget[];
  readonly poses: readonly string[];
  readonly runs: BenchData["runs"];
}

export interface BenchTarget {
  readonly key: string;
  readonly description: string;
  readonly look: string;
  /** A real route on the 2026 field (drawn where it is), not a bench course. */
  readonly onField: boolean;
  /** For field routes: each leg's waypoints. */
  readonly legs?: readonly (readonly (readonly number[])[])[];
}

/** Keys of PathPlanner-comparison targets carry this prefix in the course picker. */
export const PP_PREFIX = "pp:";

export function tableFor(data: BenchData, target: string): BenchTable {
  if (target.startsWith(PP_PREFIX) && data.ppcompare) {
    const pp = data.ppcompare;
    return {
      methods: pp.methods,
      metrics: pp.metrics,
      targets: pp.targets.map((t) => ({ ...t, key: PP_PREFIX + t.key })),
      poses: pp.meta.poses,
      runs: Object.fromEntries(
        Object.entries(pp.runs).map(([pose, byTarget]) => [pose, Object.fromEntries(Object.entries(byTarget).map(([t, v]) => [PP_PREFIX + t, v]))]),
      ),
    };
  }
  return {
    methods: data.methods,
    metrics: data.metrics,
    targets: data.courses.map((c) => ({ key: c.key, description: c.description, look: c.look ?? "", onField: false })),
    poses: data.meta.poses,
    runs: data.runs,
  };
}

export function tableStat(t: BenchTable, pose: string, target: string, method: string, metric: string): Stat | null {
  const v = t.runs[pose]?.[target]?.[method]?.[metric];
  return typeof v === "object" && v !== null ? v : null;
}

function runCount(t: BenchTable, pose: string, target: string, method: string): number {
  const n = t.runs[pose]?.[target]?.[method]?.n;
  return typeof n === "number" ? n : 0;
}

export interface Validity {
  readonly valid: boolean;
  /** Why not, in a few words ("never settled", "3/10 never settled", "38 cm off"). */
  readonly reason: string;
  readonly settled: number;
  readonly n: number;
}

/** Whether a method's runs on a target count: every seed settled, and the final error is in tolerance. */
export function validity(t: BenchTable, pose: string, target: string, method: string): Validity {
  const n = runCount(t, pose, target, method);
  const settle = tableStat(t, pose, target, method, "t_settle");
  const end = tableStat(t, pose, target, method, "err_end_cm");
  const unsettled = settle ? settle.nan : n;
  const settled = Math.max(0, n - unsettled);
  if (n === 0) return { valid: false, reason: "no runs", settled, n };
  if (settled === 0) return { valid: false, reason: "never settled", settled, n };
  if (unsettled > 0) return { valid: false, reason: `${unsettled}/${n} never settled`, settled, n };
  const worst = end?.worst;
  if (worst !== null && worst !== undefined && worst > END_TOLERANCE_CM) return { valid: false, reason: `${Math.round(worst)} cm off`, settled, n };
  return { valid: true, reason: "", settled, n };
}

export interface ScoreRow {
  readonly method: string;
  readonly label: string;
  readonly validity: Validity;
  readonly settleTime: number;
  readonly endError: number;
  /** 1, 2, ... among valid rows; null for the rest. */
  readonly rank: number | null;
}

const orInf = (v: number | null | undefined) => (v === null || v === undefined || Number.isNaN(v) ? Infinity : v);

/**
 * The scoreboard: valid rows first, by mean time settled, then reliability (seeds settled), then final
 * error; then the rest in the same order.
 */
export function scoreboard(t: BenchTable, pose: string, target: string, methods: readonly string[]): ScoreRow[] {
  const rows = methods
    .filter((m) => t.runs[pose]?.[target]?.[m] !== undefined)
    .map((m) => ({
      method: m,
      label: t.methods.find((x) => x.key === m)?.label ?? m,
      validity: validity(t, pose, target, m),
      settleTime: orInf(tableStat(t, pose, target, m, "t_settle")?.mean),
      endError: orInf(tableStat(t, pose, target, m, "err_end_cm")?.mean),
    }));
  rows.sort(
    (a, b) =>
      Number(b.validity.valid) - Number(a.validity.valid) ||
      a.settleTime - b.settleTime ||
      b.validity.settled / Math.max(1, b.validity.n) - a.validity.settled / Math.max(1, a.validity.n) ||
      a.endError - b.endError,
  );
  let rank = 0;
  return rows.map((r) => ({ ...r, rank: r.validity.valid ? ++rank : null }));
}

/** For each metric, the valid method with the lowest mean (lower is better for every metric shown). */
export function bestValidByMetric(t: BenchTable, pose: string, target: string, methods: readonly string[]): Record<string, string> {
  const best: Record<string, string> = {};
  const valid = methods.filter((m) => validity(t, pose, target, m).valid);
  for (const { key } of t.metrics) {
    let low = Infinity;
    for (const m of valid) {
      const v = tableStat(t, pose, target, m, key)?.mean;
      if (v !== null && v !== undefined && v < low) {
        low = v;
        best[key] = m;
      }
    }
  }
  return best;
}

/** Where a bench course is drawn: its own (0, 0) at this field point, the same for every course. */
export const COURSE_ORIGIN: readonly [number, number] = [-6, -1.5];
