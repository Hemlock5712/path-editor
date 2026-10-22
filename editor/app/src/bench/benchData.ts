// The bench's export for the editor (python bench/run.py --export-editor → bench/out/editor_bench.json):
// per pose source, course and method the mean and worst of a few metrics, each method's authored
// geometry, and true traces for some seeds. Read-only here.

export const BENCH_SCHEMA = 1;

/** One of a course's own measures. */
export interface CourseMetric {
  readonly key: string;
  readonly label: string;
  readonly unit: string;
  readonly higherIsBetter: boolean;
}

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
  /**
   * The harder courses' own measures (wall clearance, a drive-through's miss, ...), apart from the metrics
   * above because some are better higher: course → its measures, and pose → course → method → key → stat.
   */
  readonly courseMetrics?: {
    readonly metrics: Readonly<Record<string, readonly CourseMetric[]>>;
    readonly runs: Readonly<Record<string, Readonly<Record<string, Readonly<Record<string, Readonly<Record<string, Stat>>>>>>>>;
  };
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
// The five headline methods take the documented categorical palette's first five slots (dark steps) in
// this order, validated on this app's surface (adjacent CVD ΔE >= 8.4, normal-vision >= 19.3, all >= 3:1);
// the other methods are lighter or darker steps of their family's slot.
const COLOURS: Readonly<Record<string, string>> = {
  ours_v2: "#3987e5",
  ours_tuned: "#7fb2f0",
  ours_noadapt: "#2a64ab",
  ours_optimised: "#7fb2f0",
  pathplanner_tuned: "#d95926",
  pathplanner_hold: "#f08a5d",
  pathplanner: "#f6b08f",
  pathplanner_hold_cof09: "#a8441c",
  pp2027_tuned: "#d95926",
  pp2027_hold: "#f08a5d",
  pp2027_optimised: "#f6b08f",
  pp_path2_tuned: "#199e70",
  pp_path2: "#4cc79a",
  pp_path2_optimised: "#8fe0c0",
  choreo: "#c98500",
  choreo_hybrid: "#e8b04a",
  bline_tuned: "#d55181",
  bline_tuned_caps: "#e886aa",
  bline_readme: "#a33a62",
  bline_readme_r45: "#b8476f",
  bline_caps: "#f0b3cb",
  old_followpath: "#9085e9",
  drive_to_pose_chain: "#6b6bd6",
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

// ------------------------------------------------------------------------------ the headline comparison

/**
 * A column of the headline table: one method family, with its key in the main bench and in the
 * PathPlanner comparison (which runs the 2026 routes, with fewer methods).
 */
export interface Column {
  readonly id: string;
  readonly label: string;
  /** For the small charts. */
  readonly short: string;
  readonly main?: string;
  readonly pp?: string;
  /** Only with "+ more". */
  readonly more?: boolean;
}

export const COLUMNS: readonly Column[] = [
  { id: "ours", label: "Ours", short: "Ours", main: "ours_v2", pp: "ours_v2" },
  { id: "pp", label: "PathPlanner (Bezier, tuned + hold)", short: "PathPlanner", main: "pathplanner_tuned", pp: "pp2027_tuned" },
  { id: "path2", label: "PathPlanner path2 (tuned)", short: "PP path2", main: "pp_path2_tuned", pp: "pp_path2_tuned" },
  { id: "choreo", label: "Choreo", short: "Choreo", main: "choreo" },
  { id: "bline", label: "BLine (tuned)", short: "BLine", main: "bline_tuned" },
  { id: "pp-shipped", label: "PathPlanner (kP 5 + hold, as shipped)", short: "PP shipped", main: "pathplanner_hold", pp: "pp2027_hold", more: true },
  { id: "path2-shipped", label: "PathPlanner path2 (as shipped)", short: "path2 shipped", main: "pp_path2", pp: "pp_path2", more: true },
  { id: "bline-readme", label: "BLine (README gains)", short: "BLine README", main: "bline_readme", more: true },
  { id: "choreo-hybrid", label: "Choreo + DriveToPose finish", short: "Choreo + DTP", main: "choreo_hybrid", more: true },
  { id: "dtp", label: "DriveToPose chain", short: "DriveToPose", main: "drive_to_pose_chain", more: true },
  { id: "old", label: "Old FollowPath", short: "Old FollowPath", main: "old_followpath", more: true },
  { id: "ours-tuned", label: "Ours (tuned)", short: "Ours tuned", main: "ours_tuned", more: true },
];

/** The colour of a column: its main method's. */
export function columnColour(c: Column): string {
  return methodColour(c.main ?? c.pp ?? "");
}

export interface Row {
  /** The target key in the course picker (bench courses as they are, routes with PP_PREFIX). */
  readonly key: string;
  readonly label: string;
  readonly route: boolean;
}

/** The headline's rows: every bench course, then the 2026 routes from the PathPlanner comparison. */
export function rows(data: BenchData): Row[] {
  const courses = data.courses.map((c) => ({ key: c.key, label: c.key, route: false }));
  const routes = (data.ppcompare?.targets ?? []).filter((t) => t.onField).map((t) => ({ key: PP_PREFIX + t.key, label: t.key, route: true }));
  return [...courses, ...routes];
}

/** The method that fills a cell, if that column was run on that row's target. */
export function cellMethod(c: Column, row: Row): string | null {
  return (row.route ? c.pp : c.main) ?? null;
}

export interface Cell {
  readonly method: string | null;
  readonly value: number;
  readonly validity: Validity | null;
}

export function cell(data: BenchData, pose: string, c: Column, row: Row, metric: string): Cell {
  const method = cellMethod(c, row);
  const t = tableFor(data, row.key);
  if (!method || !t.runs[pose]?.[row.key]?.[method]) return { method: null, value: Number.NaN, validity: null };
  const v = tableStat(t, pose, row.key, method, metric)?.mean;
  return { method, value: v ?? Number.NaN, validity: validity(t, pose, row.key, method) };
}

/** The column with the best valid (lowest) value on a row, or null. */
export function rowWinner(data: BenchData, pose: string, cols: readonly Column[], row: Row, metric: string): string | null {
  let best: string | null = null;
  let low = Infinity;
  for (const c of cols) {
    const x = cell(data, pose, c, row, metric);
    if (x.validity?.valid && Number.isFinite(x.value) && x.value < low) {
      low = x.value;
      best = c.id;
    }
  }
  return best;
}

/** The plain-English verdict, from the data: who settles first, who gets within 5 cm first, who fails. */
export function verdict(data: BenchData, pose: string, cols: readonly Column[]): string[] {
  const rs = rows(data).filter((r) => cols.some((c) => cell(data, pose, c, r, "t_settle").method));
  if (rs.length === 0) return [];
  const count = (metric: string) => {
    const wins = new Map<string, number>();
    for (const r of rs) {
      const w = rowWinner(data, pose, cols, r, metric);
      if (w) wins.set(w, (wins.get(w) ?? 0) + 1);
    }
    return [...wins.entries()].sort((a, b) => b[1] - a[1]);
  };
  const label = (id: string) => cols.find((c) => c.id === id)!.label;
  const lines: string[] = [];
  const anyValid = rs.some((r) => cols.some((c) => cell(data, pose, c, r, "t_settle").validity?.valid));
  if (!anyValid) {
    const end = aggregate(data, pose, cols, rs, "err_end_cm");
    lines.push("No method settles within 2 cm of the true goal with this pose source: its drift is bigger than the settle box.");
    if (end[0]) lines.push(`${end[0].column.label} ends closest on average (${end[0].value.toFixed(1)} cm, over ${end[0].over} targets every method ran).`);
    return lines;
  }
  const settle = count("t_settle");
  if (settle[0]) {
    const [first, n] = settle[0];
    const next = settle[1];
    lines.push(`${label(first)} settles first on ${n} of ${rs.length} targets${next ? `; ${label(next[0])} on ${next[1]}` : ""}.`);
  }
  const close = count("t_5cm");
  if (close[0]) {
    const [first, n] = close[0];
    const ours = close.find(([id]) => id === "ours");
    const tail = first === "ours" ? "" : ours ? `; ours on ${ours[1]}` : "; ours on none";
    lines.push(`${label(first)} gets within 5 cm first on ${n}${tail}.`);
  }
  const failures = cols
    .map((c) => ({ c, bad: rs.filter((r) => cell(data, pose, c, r, "t_settle").validity?.valid === false).length }))
    .filter((x) => x.bad > 0)
    .sort((a, b) => b.bad - a.bad);
  if (failures.length > 0) lines.push(`Didn't settle, or ended off: ${failures.map((f) => `${f.c.label} on ${f.bad}`).join(", ")}.`);
  else lines.push("Every method settled on every target.");
  return lines;
}

// ------------------------------------------------------------------------------ aggregates for the charts

export type CompareMetric = "t_settle" | "t_5cm" | "err_end_cm" | "ct_rms_cm" | "wheel_slip_ms" | "reliability" | "plan_gap";

export const COMPARE_METRICS: readonly { id: CompareMetric; label: string; unit: string; higherIsBetter?: boolean; note: string }[] = [
  { id: "t_settle", label: "Time settled", unit: "s", note: "mean over the targets every method shown settled on" },
  { id: "t_5cm", label: "Time to 5 cm", unit: "s", note: "mean over the targets every method shown reached" },
  { id: "err_end_cm", label: "Final error", unit: "cm", note: "mean over the targets every method shown was run on" },
  { id: "ct_rms_cm", label: "Cross-track RMS", unit: "cm", note: "mean over the targets every method shown was run on" },
  { id: "wheel_slip_ms", label: "Wheel slip", unit: "ms", note: "mean time a wheel slipped per run, over the targets every method shown was run on" },
  { id: "reliability", label: "Reliability", unit: "%", higherIsBetter: true, note: "seeds that settled in tolerance, over the targets every method shown was run on" },
  { id: "plan_gap", label: "Plan vs actual", unit: "s", note: "time settled minus the planned time; PathPlanner comparison runs only" },
];

export interface Bar {
  readonly column: Column;
  readonly value: number;
  /** How many targets the value covers, for the tooltip. */
  readonly over: number;
}

/** The rows every column was run on (and, if `valid`, completed validly), with a value for the metric. */
export function commonRows(data: BenchData, pose: string, cols: readonly Column[], rs: readonly Row[], metric: string, valid: boolean): Row[] {
  return rs.filter((r) =>
    cols.every((c) => {
      const x = cell(data, pose, c, r, metric);
      return x.method !== null && x.validity !== null && (metric === "t_settle" && !valid ? true : Number.isFinite(x.value)) && (!valid || x.validity.valid);
    }),
  );
}

function meanOf(xs: readonly number[]): number {
  return xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : Number.NaN;
}

/** One bar per column for a metric over the given rows, best first; columns with no value are left out. */
export function aggregate(data: BenchData, pose: string, cols: readonly Column[], rs: readonly Row[], metric: CompareMetric): Bar[] {
  let bars: Bar[];
  if (metric === "reliability") {
    bars = cols.map((c) => {
      let ok = 0;
      let n = 0;
      let over = 0;
      for (const r of commonRows(data, pose, cols, rs, "t_settle", false)) {
        const v = cell(data, pose, c, r, "t_settle").validity;
        if (!v) continue;
        over++;
        n += v.n;
        // a run counts if it settled; a target out of tolerance counts none of its runs
        ok += v.valid ? v.n : v.reason.endsWith("off") ? 0 : v.settled;
      }
      return { column: c, value: n ? (100 * ok) / n : Number.NaN, over };
    });
  } else if (metric === "plan_gap") {
    bars = cols.map((c) => {
      const gaps: number[] = [];
      for (const r of rs) {
        if (!c.pp) continue;
        const key = r.route ? r.key : PP_PREFIX + r.key;
        const t = tableFor(data, key);
        const settle = tableStat(t, pose, key, c.pp, "t_settle")?.mean;
        const plan = tableStat(t, pose, key, c.pp, "plan_s")?.mean;
        if (typeof settle === "number" && typeof plan === "number" && t.runs[pose]?.[key]?.[c.pp] && validity(t, pose, key, c.pp).valid) gaps.push(settle - plan);
      }
      return { column: c, value: meanOf(gaps), over: gaps.length };
    });
  } else {
    // the same targets for every column (each was run on it, and for the times, each settled there), so no
    // method is flattered by skipping the hard ones or the ones it wasn't run on
    const times = metric === "t_settle" || metric === "t_5cm";
    const common = commonRows(data, pose, cols, rs, metric, times);
    bars = cols.map((c) => ({ column: c, value: meanOf(common.map((r) => cell(data, pose, c, r, metric).value)), over: common.length }));
  }
  const higher = COMPARE_METRICS.find((m) => m.id === metric)?.higherIsBetter === true;
  return bars.filter((b) => Number.isFinite(b.value)).sort((a, b) => (higher ? b.value - a.value : a.value - b.value));
}

/**
 * One target's bars for a metric: every column with a value there (for the times, only runs that count),
 * best first. The per-target charts use this, so a method missing from one target doesn't empty the chart.
 */
export function perTarget(data: BenchData, pose: string, cols: readonly Column[], row: Row, metric: CompareMetric): Bar[] {
  if (metric === "plan_gap" || metric === "reliability") return aggregate(data, pose, cols.filter((c) => cell(data, pose, c, row, "t_settle").method), [row], metric);
  const times = metric === "t_settle" || metric === "t_5cm";
  const bars = cols
    .map((c) => ({ c, x: cell(data, pose, c, row, metric) }))
    .filter(({ x }) => x.method !== null && Number.isFinite(x.value) && (!times || x.validity?.valid))
    .map(({ c, x }) => ({ column: c, value: x.value, over: 1 }));
  return bars.sort((a, b) => a.value - b.value);
}

/**
 * The optimised-paths table: as drawn and after each tool's own optimiser, curves only (ours reshapes the
 * tangents; PathPlanner's genetic algorithm moves the Bezier handles). The waypoint-moving runs are extras,
 * all by our search, shown with "more methods".
 */
export const OPTIMISED_COLUMNS: readonly { readonly method: string; readonly label: string; readonly ours: boolean; readonly more?: boolean }[] = [
  { method: "ours_v2", label: "Ours, as drawn", ours: true },
  { method: "ours_optimised", label: "Ours, its optimiser (curves only)", ours: true },
  { method: "pp2027_hold", label: "PathPlanner Bezier, as drawn", ours: false },
  { method: "pp2027_optimised", label: "PathPlanner Bezier, its own optimiser (handles only)", ours: false },
  { method: "pp_path2", label: "PathPlanner path2, as drawn", ours: false },
  { method: "ours_optimised_moved", label: "Ours, our search moving waypoints", ours: true, more: true },
  { method: "pp2027_optimised_moved", label: "PathPlanner Bezier, our search moving anchors and handles", ours: false, more: true },
  { method: "pp_path2_optimised", label: "path2, our optimiser's search (path2 has none)", ours: false, more: true },
];

// ------------------------------------------------------------------------------ the courses' own measures

/** A course's own measures (none for most). */
export function courseMetricsFor(data: BenchData, course: string): readonly CourseMetric[] {
  return data.courseMetrics?.metrics[course] ?? [];
}

export function courseMetricStat(data: BenchData, pose: string, course: string, method: string, key: string): Stat | null {
  return data.courseMetrics?.runs[pose]?.[course]?.[method]?.[key] ?? null;
}

/** Which of some values is best (null: none finite), higher or lower as the measure says. */
export function bestOf<T>(items: readonly { readonly id: T; readonly value: number }[], higherIsBetter: boolean): T | null {
  let best: T | null = null;
  let at = higherIsBetter ? -Infinity : Infinity;
  for (const x of items) {
    if (!Number.isFinite(x.value)) continue;
    if (higherIsBetter ? x.value > at : x.value < at) {
      at = x.value;
      best = x.id;
    }
  }
  return best;
}

/**
 * A row's headline in the main table: time settled, or, on a course where nothing settles by design (a
 * drive-through), its first own measure instead.
 */
export function rowHeadline(data: BenchData, pose: string, cols: readonly Column[], row: Row): CourseMetric | null {
  if (row.route) return null;
  const own = courseMetricsFor(data, row.key);
  if (own.length === 0) return null;
  const settles = cols.some((c) => {
    const m = cellMethod(c, row);
    const st = m ? tableStat(tableFor(data, row.key), pose, row.key, m, "t_settle") : null;
    return st !== null && typeof st.mean === "number";
  });
  return settles ? null : own[0]!;
}
