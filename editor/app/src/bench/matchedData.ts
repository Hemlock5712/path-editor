// The Bench page's data: every library planning to the same limits on the same simulated robot
// (bench/matched.py writes the snapshot, public/bench/matched.json). Pure functions over that file, in the
// page's words: a run "finishes" when it settles on the end pose. Nothing here touches the project.

export interface RunStats {
  readonly n: number;
  readonly t_5cm: number | null;
  readonly t_settle: number | null;
  readonly t_settle_nan: number;
  readonly err_end_cm: number | null;
  readonly ct_own_rms_cm: number | null;
  readonly wheel_slip_ms: number | null;
  readonly overshoot_cm: number | null;
}

export interface MatchedRule {
  readonly traction: number;
  /** course -> method -> planned time, s (only the planners that make a timed plan). */
  readonly plans: Record<string, Record<string, number>>;
  /** pose -> course -> method -> statistics over the seeds. */
  readonly runs: Record<string, Record<string, Record<string, RunStats>>>;
}

export interface MatchedData {
  readonly schema: number;
  readonly seeds: number;
  readonly methods: readonly { key: string; label: string }[];
  readonly courses: { readonly plan: readonly string[]; readonly disturbed: readonly string[] };
  readonly courseLabels: Record<string, string>;
  readonly settleRule: string;
  readonly rules: Record<string, MatchedRule>;
}

export type CourseGroup = "plan" | "disturbed";

/** The methods on the page, ours first. */
export const SHOWN: readonly string[] = ["ours_opt", "ours", "choreo_hold", "choreo", "pathplanner", "pp_path2", "bline"];
export const OURS = new Set<string>(["ours_opt", "ours"]);

export const METHOD_NAMES: Record<string, string> = {
  ours_opt: "Ours (optimised)",
  ours: "Ours (as drawn)",
  choreo: "Choreo",
  choreo_hold: "Choreo + final hold",
  pathplanner: "PathPlanner",
  pp_path2: "PathPlanner path2 (beta)",
  bline: "BLine",
  ours_on_choreo: "Ours following Choreo's plan",
};

export const METHOD_NOTES: Record<string, string> = {
  ours_opt: "Our planner with the editor's Optimise path, followed by our follower (velocities only, as our robot code sends)",
  ours: "Our planner on the path exactly as drawn, followed by our follower (velocities only, as our robot code sends)",
  choreo_hold: "Choreo's trajectory and ChoreoLib's documented controller with its wheel forces, then held on the end pose until it gets there",
  choreo: "Choreo's trajectory, followed by ChoreoLib with its documented controller and its wheel forces, as CTRE's Choreo example sends them. It stops when the trajectory's time runs out",
  pathplanner: "PathPlanner 2027's trajectory and follower, tuned, with its wheel forces as CTRE's template sends them, holding on the end pose",
  pp_path2: "PathPlanner's new path system (pre-release), tuned",
  bline: "BLine, tuned, with BLine-Web's corner speed caps",
};

export const COURSE_NAMES: Record<string, string> = {
  straight_3m: "3 m straight",
  s_curve: "S-curve",
  hairpin: "Hairpin",
  slalom: "Slalom",
  long_auto: "12 m auto route",
  figure_eight: "Figure eight",
  wall_hug: "Along a wall",
  spin_while_driving: "Full spin while driving",
  fast_long: "Long and fast (5 m/s)",
  heavy_robot: "Heavy robot (S-curve)",
  hit_mid_path: "Shoved halfway",
  low_grip_s_curve: "Slippery floor",
  grip_drop_s_curve: "Grip drops halfway",
};


/** A finished run that still ends further off than this doesn't count as a clean finish. */
export const END_TOLERANCE_CM = 5;

export function parseMatched(text: string): MatchedData {
  const d = JSON.parse(text) as MatchedData;
  if (d.schema !== 1 || !d.rules || !d.methods) throw new Error("not a matched-limits bench snapshot");
  return d;
}

export function name(method: string): string {
  return METHOD_NAMES[method] ?? method;
}

export function courseName(course: string): string {
  return COURSE_NAMES[course] ?? course;
}

/** A method's statistics on a course, or null if it wasn't run there. */
export function stats(d: MatchedData, rule: string, pose: string, course: string, method: string): RunStats | null {
  return d.rules[rule]?.runs[pose]?.[course]?.[method] ?? null;
}

export function finished(s: RunStats): number {
  return s.n - s.t_settle_nan;
}

/** Every run finished, and ended within the tolerance. */
export function allFinished(s: RunStats | null): boolean {
  return !!s && s.t_settle_nan === 0 && s.t_settle !== null && (s.err_end_cm ?? Infinity) <= END_TOLERANCE_CM;
}

/** Times this close count as a tie (the page shows hundredths). */
export const TIE_SECONDS = 0.005;

/**
 * The fastest to finish a course among methods whose every run finished, with any within
 * {@link TIE_SECONDS} of it (a tie at the precision shown); empty when none finished every run.
 */
export function courseWinners(d: MatchedData, rule: string, pose: string, course: string, methods: readonly string[] = SHOWN): string[] {
  const times = methods.map((m) => {
    const s = stats(d, rule, pose, course, m);
    return allFinished(s) ? s!.t_settle! : Infinity;
  });
  const best = Math.min(...times);
  return Number.isFinite(best) ? methods.filter((_, i) => times[i]! <= best + TIE_SECONDS) : [];
}

export interface Summary {
  readonly method: string;
  readonly courses: number;
  readonly runs: number;
  readonly finished: number;
  /** Courses it finished fastest, ties included (every run finished). */
  readonly wins: number;
  /** Sum over the courses of the mean finishing time; NaN when some course never finished at all. */
  readonly totalTime: number;
  /** Courses where no run finished. */
  readonly neverFinished: readonly string[];
  /** Mean over the courses of how far from the end pose the robot ended, cm. */
  readonly endErrorCm: number;
  /** Sum over the courses of the mean time wheels spent slipping (wheel-seconds: two wheels for 1 s is 2). */
  readonly slipSeconds: number;
}

function mean(xs: readonly number[]): number {
  return xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : Number.NaN;
}

export function summaries(d: MatchedData, rule: string, pose: string, group: CourseGroup, methods: readonly string[] = SHOWN): Summary[] {
  const courses = d.courses[group];
  const winners = courses.map((c) => courseWinners(d, rule, pose, c, methods));
  const out: Summary[] = [];
  for (const m of methods) {
    let runs = 0;
    let done = 0;
    let total = 0;
    const never: string[] = [];
    const err: number[] = [];
    let slip = 0;
    for (const c of courses) {
      const s = stats(d, rule, pose, c, m);
      if (!s) {
        total = Number.NaN;
        continue;
      }
      runs += s.n;
      done += finished(s);
      if (s.t_settle === null) never.push(c);
      total += s.t_settle ?? Number.NaN;
      if (s.err_end_cm !== null) err.push(s.err_end_cm);
      slip += (s.wheel_slip_ms ?? Number.NaN) / 1000;
    }
    if (runs === 0) continue;
    out.push({
      method: m,
      courses: courses.length,
      runs,
      finished: done,
      wins: winners.filter((w) => w.includes(m)).length,
      totalTime: total,
      neverFinished: never,
      endErrorCm: mean(err),
      slipSeconds: slip,
    });
  }
  return out;
}

/** Planned time summed over the clean courses, per method that makes a timed plan. */
export function planTotals(d: MatchedData, rule: string): Record<string, number> {
  const out: Record<string, number> = {};
  for (const c of d.courses.plan) {
    for (const [m, t] of Object.entries(d.rules[rule]?.plans[c] ?? {})) out[m] = (out[m] ?? 0) + t;
  }
  return out;
}

/** "a, b and c". */
export function andList(xs: readonly string[]): string {
  return xs.length <= 1 ? (xs[0] ?? "") : `${xs.slice(0, -1).join(", ")} and ${xs[xs.length - 1]}`;
}
