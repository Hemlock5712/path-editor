// The Bench page's head-to-head: every library planning to the same limits (bench/matched.py writes the
// snapshot, public/bench/matched.json). Pure functions over that file: the scoreboard, the per-course table
// and the verdict lines. Nothing here touches the project.

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

/** The methods that are ours (tinted); ours_on_choreo is ours following Choreo's plan, listed apart. */
export const OURS = new Set(["ours_opt", "ours", "ours_on_choreo"]);
/** Shown by default: ours and each rival; "ours following Choreo's plan" is an experiment. */
export const PRIMARY = ["ours_opt", "ours", "choreo", "pathplanner", "pp_path2", "bline"];
/** A run settled but further off than this counts against the method like a run that never settled. */
export const END_TOLERANCE_CM = 5;

export function parseMatched(text: string): MatchedData {
  const d = JSON.parse(text) as MatchedData;
  if (d.schema !== 1 || !d.rules || !d.methods) throw new Error("not a matched-limits bench snapshot");
  return d;
}

export const RULE_LABELS: Record<string, string> = { equalPush: "Equal push", tipOrGrip: "Tip or grip" };

/** A method's statistics on a course, or null if it wasn't run there. */
export function stats(d: MatchedData, rule: string, pose: string, course: string, method: string): RunStats | null {
  return d.rules[rule]?.runs[pose]?.[course]?.[method] ?? null;
}

/** Every run settled, and on average within the end tolerance. */
export function allSettled(s: RunStats | null): boolean {
  return !!s && s.t_settle_nan === 0 && s.t_settle !== null && (s.err_end_cm ?? Infinity) <= END_TOLERANCE_CM;
}

/** The fastest method to settle on a course among those whose every run settled (null: nobody did). */
export function courseWinner(d: MatchedData, rule: string, pose: string, course: string, methods: readonly string[]): string | null {
  let best: string | null = null;
  let bestT = Infinity;
  for (const m of methods) {
    const s = stats(d, rule, pose, course, m);
    if (allSettled(s) && s!.t_settle! < bestT) {
      bestT = s!.t_settle!;
      best = m;
    }
  }
  return best;
}

export interface Score {
  readonly method: string;
  /** Courses where it settled fastest with every run settled. */
  readonly wins: number;
  readonly courses: number;
  readonly runs: number;
  /** Runs that never settled. */
  readonly never: number;
  /** Sum over the courses of the mean time settled; NaN if some course never settled at all. */
  readonly settledSum: number;
  /** Sum over the courses of the mean time to get within 5 cm; NaN if some course never got there. */
  readonly within5Sum: number;
  /** Mean over the courses of the mean final error, cm. */
  readonly endErrorCm: number;
  /** Mean over the courses of the RMS distance off its own path, cm. */
  readonly offPathCm: number;
}

function mean(xs: readonly number[]): number {
  return xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : Number.NaN;
}

export function scoreboard(d: MatchedData, rule: string, pose: string, group: CourseGroup, methods: readonly string[]): Score[] {
  const courses = d.courses[group];
  const winners = courses.map((c) => courseWinner(d, rule, pose, c, methods));
  return methods
    .filter((m) => courses.some((c) => stats(d, rule, pose, c, m)))
    .map((m) => {
      let runs = 0;
      let never = 0;
      let settledSum = 0;
      let within5Sum = 0;
      const err: number[] = [];
      const off: number[] = [];
      for (const c of courses) {
        const s = stats(d, rule, pose, c, m);
        if (!s) {
          settledSum = within5Sum = Number.NaN;
          continue;
        }
        runs += s.n;
        never += s.t_settle_nan;
        settledSum += s.t_settle ?? Number.NaN;
        within5Sum += s.t_5cm ?? Number.NaN;
        if (s.err_end_cm !== null) err.push(s.err_end_cm);
        if (s.ct_own_rms_cm !== null) off.push(s.ct_own_rms_cm);
      }
      return {
        method: m,
        wins: winners.filter((w) => w === m).length,
        courses: courses.length,
        runs,
        never,
        settledSum,
        within5Sum,
        endErrorCm: mean(err),
        offPathCm: mean(off),
      };
    });
}

/** Planned time summed over the planning courses, per method that makes a timed plan. */
export function planSums(d: MatchedData, rule: string): Record<string, number> {
  const out: Record<string, number> = {};
  for (const c of d.courses.plan) {
    for (const [m, t] of Object.entries(d.rules[rule]?.plans[c] ?? {})) out[m] = (out[m] ?? 0) + t;
  }
  return out;
}

export function label(d: MatchedData, method: string): string {
  return d.methods.find((m) => m.key === method)?.label ?? method;
}

function pct(a: number, b: number): string {
  return `${(((a - b) / b) * 100).toFixed(1)} %`;
}

const SHOVE = "hit_mid_path";

function list(d: MatchedData, parts: readonly { method: string; v: number }[], digits = 1): string {
  return parts.map((p) => `${label(d, p.method)} ${p.v.toFixed(digits)}`).join(", ");
}

/** The headline: who wins, who fails, and the honest notes (Choreo's plans; what odometry can't see). */
export function verdict(d: MatchedData, rule: string, pose: string, methods: readonly string[]): string[] {
  const lines: string[] = [];
  const board = scoreboard(d, rule, pose, "plan", methods);
  const ours = board.find((s) => s.method === "ours_opt");
  const rivals = board.filter((s) => !OURS.has(s.method));
  const plans = planSums(d, rule);
  const choreo = board.find((s) => s.method === "choreo");
  if (pose === "truth") {
    if (ours) {
      const top = [...board].sort((a, b) => b.wins - a.wins)[0]!;
      lines.push(
        top.method === "ours_opt" || top.wins === ours.wins
          ? `Ours (optimised) settles first on ${ours.wins} of ${ours.courses} courses, and every one of its ${ours.runs} runs settles.`
          : `Ours (optimised) settles first on ${ours.wins} of ${ours.courses} courses (${label(d, top.method)}: ${top.wins}); ${ours.never ? `${ours.never} of its ${ours.runs} runs never settle` : `every one of its ${ours.runs} runs settles`}.`,
      );
      const failures = rivals.filter((s) => s.never > 0).map((s) => `${label(d, s.method)} ${s.never}`);
      if (failures.length) lines.push(`Runs that never settle, of ${ours.runs} each: ${failures.join(", ")}.`);
    }
    if (plans.choreo && plans.ours_opt) {
      lines.push(
        `Choreo's solver plans the fastest paths: ${plans.choreo.toFixed(2)} s over the ${d.courses.plan.length} courses against ours optimised ` +
          `${plans.ours_opt.toFixed(2)} s (${pct(plans.ours_opt, plans.choreo)} slower). ChoreoLib's clock-driven follower gets within 5 cm first` +
          (choreo && choreo.never ? `, but ${choreo.never} of its ${choreo.runs} runs never settle.` : "."),
      );
    }
    const dist = scoreboard(d, rule, pose, "disturbed", methods);
    const o = dist.find((s) => s.method === "ours_opt");
    if (o) {
      const others = dist.filter((s) => !OURS.has(s.method)).sort((a, b) => b.endErrorCm - a.endErrorCm);
      lines.push(
        `Shoved, on low grip or a grip drop, ours ends ${o.endErrorCm.toFixed(1)} cm off on average and settles ${o.runs - o.never} of ${o.runs} runs ` +
          `(${others.map((s) => `${label(d, s.method)} ${s.endErrorCm.toFixed(1)} cm, ${s.runs - s.never}/${s.runs}`).join("; ")}).`,
      );
    }
    return lines;
  }
  // wheel odometry: the pose drifts, so the end error is the measure; a shove is invisible to it
  lines.push(
    "With wheel odometry alone nobody settles most runs: a 1 % wheel-size error is already more than 2 cm over these distances, so the end error is the measure here.",
  );
  if (ours) {
    lines.push(
      `End error over the clean courses, cm: ${list(d, board.map((s) => ({ method: s.method, v: s.endErrorCm })).sort((a, b) => a.v - b.v))}.`,
    );
  }
  for (const c of d.courses.disturbed.filter((c) => c !== SHOVE)) {
    const parts = methods
      .map((m) => ({ method: m, v: stats(d, rule, pose, c, m)?.err_end_cm ?? Number.NaN }))
      .filter((p) => Number.isFinite(p.v))
      .sort((a, b) => a.v - b.v);
    if (parts.length) lines.push(`${c === "low_grip_s_curve" ? "Low grip" : c === "grip_drop_s_curve" ? "Grip drop" : c}, end error, cm: ${list(d, parts)}.`);
  }
  const shove = methods.map((m) => stats(d, rule, pose, SHOVE, m)?.err_end_cm).filter((v): v is number => typeof v === "number");
  if (shove.length) {
    lines.push(
      `A shove is invisible to wheel odometry: the robot slides and the wheels don't count it, so every follower ends ` +
        `${Math.min(...shove).toFixed(0)}–${Math.max(...shove).toFixed(0)} cm off. Only vision fixes that.`,
    );
  }
  return lines;
}
