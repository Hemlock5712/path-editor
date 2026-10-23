// The Bench page (read-only): how our planner and follower compare with Choreo, PathPlanner and BLine when
// every library plans to the same limits on the same simulated robot. It reads one snapshot
// (public/bench/matched.json, from bench/matched.py) and tells it in order: the short version, speed,
// when something goes wrong, without vision, course by course, and how it was tested.

import type { ReactNode } from "react";
import {
  allFinished,
  andList,
  courseName,
  courseWinners,
  END_TOLERANCE_CM,
  finished,
  METHOD_NOTES,
  name,
  OURS,
  planTotals,
  SHOWN,
  stats,
  summaries,
  type MatchedData,
  type Summary,
} from "../bench/matchedData.ts";
import { bench } from "../state/bench.ts";
import { useStore } from "../state/store.ts";

const SHOVE = "hit_mid_path";

function s2(v: number): string {
  return Number.isFinite(v) ? v.toFixed(2) : "–";
}

function cm(v: number): string {
  return Number.isFinite(v) ? (v < 10 ? v.toFixed(1) : v.toFixed(0)) : "–";
}

interface BarRow {
  readonly method: string;
  readonly value: number;
  /** The text at the bar's tip. */
  readonly text: string;
  /** A muted note after the value (a caveat, never the value itself). */
  readonly note?: string;
  readonly tip?: string;
}

/** Horizontal bars, one per method: ours in the accent, the rest one quiet grey, the value at the tip. */
function Bars({ rows, testId, label }: { rows: readonly BarRow[]; testId?: string; label: string }) {
  const max = Math.max(...rows.map((r) => (Number.isFinite(r.value) ? r.value : 0)), 1e-9);
  return (
    <ul className="bars" data-testid={testId} aria-label={label}>
      {rows.map((r) => (
        <li key={r.method} className={OURS.has(r.method) ? "ours" : ""} data-method={r.method} data-tip={r.tip} tabIndex={0}>
          <span className="bar-label">{name(r.method)}</span>
          <span className="bar-track">
            {Number.isFinite(r.value) && <span className="bar" style={{ left: 0, width: `${(r.value / max) * 100}%` }} />}
            <span className="bar-value" style={{ left: Number.isFinite(r.value) ? `calc(${(r.value / max) * 100}% + 6px)` : 0 }}>
              {r.text}
              {r.note && <span className="bar-note"> {r.note}</span>}
            </span>
          </span>
        </li>
      ))}
    </ul>
  );
}

function Tile({ title, value, detail, testId }: { title: string; value: ReactNode; detail: ReactNode; testId?: string }) {
  return (
    <div className="bench-tile" data-testid={testId}>
      <div className="bench-tile-title">{title}</div>
      <div className="bench-tile-value">{value}</div>
      <div className="bench-tile-detail">{detail}</div>
    </div>
  );
}

function Section({ id, title, lead, children }: { id: string; title: string; lead?: ReactNode; children: ReactNode }) {
  return (
    <section className="bench-section" aria-labelledby={`bench-${id}-title`} data-testid={`bench-${id}`}>
      <h2 id={`bench-${id}-title`}>{title}</h2>
      {lead && <p className="bench-lead">{lead}</p>}
      {children}
    </section>
  );
}

function rivals(list: readonly Summary[]): Summary[] {
  return list.filter((s) => !OURS.has(s.method));
}

function byMethod(list: readonly Summary[], m: string): Summary | undefined {
  return list.find((s) => s.method === m);
}

function ShortVersion({ d, rule }: { d: MatchedData; rule: string }) {
  const clean = summaries(d, rule, "truth", "plan");
  const dist = summaries(d, rule, "truth", "disturbed");
  const ours = byMethod(clean, "ours_opt")!;
  const oursDist = byMethod(dist, "ours_opt")!;
  const others = rivals(clean);
  const winsOthers = others.filter((s) => s.wins > 0).sort((a, b) => b.wins - a.wins);
  const distOthers = rivals(dist).sort((a, b) => a.endErrorCm - b.endErrorCm);
  return (
    <div className="bench-tiles">
      <Tile
        testId="bench-tile-finish"
        title="Runs that finish"
        value={
          <>
            {ours.finished} <span className="of">of {ours.runs}</span>
          </>
        }
        detail={others.map((s) => `${name(s.method)} ${s.finished}`).join(" · ")}
      />
      <Tile
        testId="bench-tile-wins"
        title="Courses where it finishes first"
        value={
          <>
            {ours.wins} <span className="of">of {ours.courses}</span>
          </>
        }
        detail={winsOthers.length ? winsOthers.map((s) => `${name(s.method)} ${s.wins}`).join(" · ") : "No other library wins a course"}
      />
      <Tile
        testId="bench-tile-recover"
        title="Shoved, or on a slippery floor"
        value={
          <>
            {cm(oursDist.endErrorCm)} <span className="of">cm from the target</span>
          </>
        }
        detail={distOthers.map((s) => `${name(s.method)} ${cm(s.endErrorCm)} cm`).join(" · ")}
      />
    </div>
  );
}

function Speed({ d, rule }: { d: MatchedData; rule: string }) {
  const clean = summaries(d, rule, "truth", "plan");
  const plans = planTotals(d, rule);
  const rows: BarRow[] = clean
    .map((s) => {
      const unfinished = s.runs - s.finished;
      const note = s.neverFinished.length
        ? `no total: never finished the ${andList(s.neverFinished.map((c) => courseName(c).toLowerCase()))}`
        : unfinished
          ? `${unfinished} of ${s.runs} runs never finished`
          : undefined;
      return {
        method: s.method,
        value: s.totalTime,
        text: Number.isFinite(s.totalTime) ? `${s2(s.totalTime)} s` : "",
        note,
        tip: METHOD_NOTES[s.method],
        unfinished,
      };
    })
    // the ones that finished every run first, fastest first; then the rest
    .sort((a, b) => Number(a.unfinished > 0) - Number(b.unfinished > 0) || (Number.isFinite(a.value) ? a.value : Infinity) - (Number.isFinite(b.value) ? b.value : Infinity));
  const planRows: BarRow[] = Object.entries(plans)
    .filter(([m]) => SHOWN.includes(m))
    .map(([m, t]) => ({ method: m, value: t, text: `${s2(t)} s` }))
    .sort((a, b) => a.value - b.value);
  const choreoPlan = plans.choreo;
  const oursPlan = plans.ours_opt;
  const choreo = byMethod(clean, "choreo");
  return (
    <Section
      id="speed"
      title="Speed"
      lead={`Time to drive all ${d.courses.plan.length} courses and finish on the target (the average of ${d.seeds} runs each, added up). Shorter is better. The ones that finished every run come first.`}
    >
      <Bars rows={rows} testId="bench-speed-bars" label="Total time to finish" />
      {choreoPlan !== undefined && oursPlan !== undefined && (
        <>
          <p className="bench-lead">
            <strong>Choreo makes the fastest plans.</strong> Before driving, its solver plans the courses in {s2(choreoPlan)} s against our
            optimised {s2(oursPlan)} s ({(((oursPlan - choreoPlan) / choreoPlan) * 100).toFixed(1)} % slower), because it picks its own line
            and heading between the waypoints. It doesn't keep that lead on the floor: ChoreoLib follows on a clock, so when the robot lags it
            catches up by overshooting, and when the clock runs out it stops wherever it is
            {choreo && choreo.runs > choreo.finished ? ` (${choreo.runs - choreo.finished} of ${choreo.runs} runs never finished)` : ""}. Ours
            follows by position and brakes onto the target.
          </p>
          <Bars rows={planRows} testId="bench-plan-bars" label="Planned time" />
          <p className="hint">Planned time, before driving. PathPlanner path2 and BLine steer toward waypoints as they go and have no plan to time.</p>
        </>
      )}
    </Section>
  );
}

function GoesWrong({ d, rule }: { d: MatchedData; rule: string }) {
  const courses = d.courses.disturbed;
  return (
    <Section
      id="disturbed"
      title="When something goes wrong"
      lead="The S-curve three more ways: shoved sideways for 0.3 s halfway along, on a floor with 61 % of the grip, and with the grip dropping to that halfway along. Each follower knows exactly where the robot is."
    >
      <DisturbTable d={d} rule={rule} pose="truth" courses={courses} testId="bench-disturbed-table" showFinished />
    </Section>
  );
}

/** Rows: the disturbances; columns: the methods; each cell how far off it ended and how many runs finished. */
function DisturbTable({
  d,
  rule,
  pose,
  courses,
  testId,
  showFinished = false,
}: {
  d: MatchedData;
  rule: string;
  pose: string;
  courses: readonly string[];
  testId: string;
  showFinished?: boolean;
}) {
  return (
    <div className="bench-table-scroll">
      <table className="benchmark" data-testid={testId}>
        <thead>
          <tr>
            <th className="target" />
            {SHOWN.map((m) => (
              <th key={m} className={OURS.has(m) ? "ours" : ""} data-tip={METHOD_NOTES[m]}>
                {name(m)}
              </th>
            ))}
          </tr>
        </thead>
        <tbody>
          {courses.map((c) => {
            // best by the value shown (a tie at a tenth of a centimetre is a tie)
            const errs = SHOWN.map((m) => Number(cm(stats(d, rule, pose, c, m)?.err_end_cm ?? Infinity)) || Infinity);
            const best = Math.min(...errs);
            return (
              <tr key={c}>
                <th className="target" data-tip={d.courseLabels[c]}>
                  {courseName(c)}
                </th>
                {SHOWN.map((m, i) => {
                  const s = stats(d, rule, pose, c, m);
                  if (!s) return <td key={m}>–</td>;
                  const done = finished(s);
                  return (
                    <td key={m} className={[OURS.has(m) ? "ours" : "", errs[i] === best ? "best" : ""].filter(Boolean).join(" ")}>
                      {cm(s.err_end_cm ?? Number.NaN)} cm
                      {showFinished && (
                        <span className={`cell-sub ${done < s.n ? "warn" : ""}`}>
                          {done === s.n ? "all finished" : done === 0 ? "none finished" : `${done} of ${s.n} finished`}
                        </span>
                      )}
                    </td>
                  );
                })}
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}

function WithoutVision({ d, rule }: { d: MatchedData; rule: string }) {
  const clean = summaries(d, rule, "odom", "plan");
  const rows: BarRow[] = clean
    .map((s) => ({ method: s.method, value: s.endErrorCm, text: `${cm(s.endErrorCm)} cm`, tip: METHOD_NOTES[s.method] }))
    .sort((a, b) => a.value - b.value);
  const shove = SHOWN.map((m) => stats(d, rule, "odom", SHOVE, m)?.err_end_cm).filter((v): v is number => typeof v === "number");
  const slipAware: BarRow[] = SHOWN.flatMap((m) => {
    const e = stats(d, rule, "slipodom", SHOVE, m)?.err_end_cm;
    return typeof e === "number" ? [{ method: m, value: e, text: `${cm(e)} cm`, tip: METHOD_NOTES[m] }] : [];
  }).sort((a, b) => a.value - b.value);
  return (
    <Section
      id="odometry"
      title="Without vision"
      lead="The same courses with only wheel odometry and the gyro to say where the robot is, as on a robot with no camera. Odometry drifts (a 1 % wheel-size error alone is more than 2 cm here), so almost no run finishes within 2 cm for anyone. What matters is how far from the target each one really ends."
    >
      <Bars rows={rows} testId="bench-odom-bars" label="Distance from the target at the end, wheel odometry" />
      <p className="hint">Average distance from the target at the end, over the {d.courses.plan.length} clean courses. Shorter is better.</p>
      <DisturbTable d={d} rule={rule} pose="odom" courses={d.courses.disturbed.filter((c) => c !== SHOVE)} testId="bench-odom-table" />
      {shove.length > 0 && (
        <p className="bench-lead" data-testid="bench-shove-note">
          A shove is the hard case. The robot slides sideways, the wheels don't count the slide, and with plain wheel odometry every
          follower ends {Math.min(...shove).toFixed(0)}–{Math.max(...shove).toFixed(0)} cm off while sure it is on target.{" "}
          {slipAware.length > 0 && "Our library's slip-aware odometry uses the IMU to notice the slide. Given to every follower, the shove ends:"}
        </p>
      )}
      {slipAware.length > 0 && (
        <>
          <Bars rows={slipAware} testId="bench-slipaware-bars" label="Shoved halfway, slip-aware odometry: distance from the target at the end" />
          <p className="hint">Shoved halfway, slip-aware odometry: average distance from the target at the end. Shorter is better.</p>
        </>
      )}
    </Section>
  );
}

function CourseByCourse({ d, rule }: { d: MatchedData; rule: string }) {
  return (
    <Section id="courses" title="Course by course" lead="Average time to finish, in seconds. Bold: the fastest among those that finished every run (both, when they tie to the hundredth).">
      <div className="bench-table-scroll">
        <table className="benchmark" data-testid="bench-course-table">
          <thead>
            <tr>
              <th className="target">Course</th>
              {SHOWN.map((m) => (
                <th key={m} className={OURS.has(m) ? "ours" : ""} data-tip={METHOD_NOTES[m]}>
                  {name(m)}
                </th>
              ))}
            </tr>
          </thead>
          <tbody>
            {d.courses.plan.map((c) => {
              const win = courseWinners(d, rule, "truth", c);
              return (
                <tr key={c}>
                  <th className="target" data-tip={d.courseLabels[c]}>
                    {courseName(c)}
                  </th>
                  {SHOWN.map((m) => {
                    const s = stats(d, rule, "truth", c, m);
                    if (!s) return <td key={m}>–</td>;
                    const done = finished(s);
                    const cls = [OURS.has(m) ? "ours" : "", win.includes(m) ? "best" : ""].filter(Boolean).join(" ");
                    const tip = `${done} of ${s.n} runs finished; ended ${cm(s.err_end_cm ?? Number.NaN)} cm from the target on average`;
                    return (
                      <td key={m} className={cls} data-tip={tip}>
                        {s.t_settle === null ? <span className="cell-warn">never finished</span> : `${s2(s.t_settle)}`}
                        {s.t_settle !== null && !allFinished(s) && (
                          <span className="cell-sub warn">{done < s.n ? `${done} of ${s.n} finished` : `ends ${cm(s.err_end_cm ?? Number.NaN)} cm off`}</span>
                        )}
                      </td>
                    );
                  })}
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
      <p className="hint">Hover a course for what it is, and a cell for how many runs finished and how close they ended.</p>
    </Section>
  );
}

function HowTested({ d, rule }: { d: MatchedData; rule: string }) {
  const r = d.rules[rule]!;
  return (
    <details className="bench-section bench-how" data-testid="bench-how">
      <summary>
        <h2>How we tested</h2>
      </summary>
      <ul>
        <li>
          <strong>The robot</strong> is a simulated swerve drive at 500 Hz with tyres that grip and slip: 60 kg, Kraken X60 motors, SDS
          Spike Grip tread (grip 1.5), a 240 A current limit. Every library is given these numbers.
        </li>
        <li>
          <strong>The same limits for everyone.</strong> Each planner may accelerate and corner up to {r.traction.toFixed(2)} m/s² (the
          traction limit, see below), at up to 4 m/s (5 m/s on the long, fast course). Ours, Choreo and PathPlanner
          take that from their robot settings; path2 and BLine from their acceleration caps.
        </li>
        <li>
          <strong>Tuned followers.</strong> Each library's follower uses the gains a sweep picked as its best. Choreo uses ChoreoLib's
          documented controller, as teams start from.
        </li>
        <li>
          <strong>Finishing</strong> means {d.settleRule}. Every course is driven {d.seeds} times with small random differences in the
          robot.
        </li>
        <li>
          <strong>Traction.</strong> The limit is 90 % of grip or tipping, whichever comes first, as AccelerationLimiter:{" "}
          {r.traction.toFixed(2)} m/s². The loaded wheels push harder than the light ones, so a follower has to cope with some slip.
        </li>
        <li>
          <strong>Optimised</strong> is the editor's Optimise path: it reshapes the curves between the waypoints you drew, which stay put.
        </li>
        <li>
          The numbers come from <code>bench/matched.py</code> in the repository (tables in <code>bench/MATCHED.md</code>).
        </li>
      </ul>
    </details>
  );
}

export function BenchView() {
  const s = useStore(bench, (b) => b);
  const d = s.data;
  // the snapshot was run under two traction rules; the robot uses tip or grip (AccelerationLimiter's)
  const rule = d ? (d.rules.tipOrGrip ? "tipOrGrip" : Object.keys(d.rules)[0]!) : "tipOrGrip";
  return (
    <main className="bench-page" aria-label="Bench comparison (read-only)">
      <div className="bench-doc">
        <header className="bench-hero">
          <h1>Path followers, head to head</h1>
          <p className="bench-lead">
            Ours against Choreo, PathPlanner and BLine, driving the same autos on the same simulated robot. Every library plans to the same
            limits, so the differences come from how each one plans and follows.
          </p>
          {s.loadingSnapshot && <p className="stats">Loading the results…</p>}
          {s.error && (
            <p className="check error" role="alert">
              {s.error}
            </p>
          )}
          {d && (
            <div className="bench-controls-row">
              <p className="hint">
                Up to {d.rules[rule]!.traction.toFixed(2)} m/s² of acceleration and cornering. {d.courses.plan.length} courses, {d.seeds} runs
                each. A run <em>finishes</em> when the robot stops within 2 cm and 1° of the target and stays there.
              </p>
            </div>
          )}
        </header>
        {d && (
          <>
            <ShortVersion d={d} rule={rule} />
            <Speed d={d} rule={rule} />
            <GoesWrong d={d} rule={rule} />
            <WithoutVision d={d} rule={rule} />
            <CourseByCourse d={d} rule={rule} />
            <HowTested d={d} rule={rule} />
          </>
        )}
        {!d && !s.loadingSnapshot && !s.error && <p className="stats">No results yet.</p>}
        <p className="hint bench-foot">Tolerances: a clean finish also ends within {END_TOLERANCE_CM} cm of the target.</p>
      </div>
    </main>
  );
}
