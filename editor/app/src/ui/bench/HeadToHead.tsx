// The Bench page's head-to-head: every library planning to the same limits (bench/matched.py). A verdict
// written from the data, a scoreboard per method, the planned times, and the per-course table. It reads its
// own snapshot (public/bench/matched.json), so it stands apart from the older bench below it.

import { useEffect, useMemo, useState } from "react";
import {
  allSettled,
  courseWinner,
  END_TOLERANCE_CM,
  label,
  OURS,
  parseMatched,
  planSums,
  PRIMARY,
  RULE_LABELS,
  scoreboard,
  stats,
  verdict,
  type CourseGroup,
  type MatchedData,
} from "../../bench/matchedData.ts";
import { Segmented, Switch } from "../controls.tsx";

export const MATCHED_URL = `${import.meta.env.BASE_URL}bench/matched.json`;

const POSES: Record<string, { label: string; explain: string }> = {
  truth: { label: "Perfect pose", explain: "The follower is told exactly where the robot is: the control law on its own." },
  odom: { label: "Wheel odometry", explain: "As on a robot without vision: wheel odometry and the gyro, which drift; ends are judged on the true pose." },
};

const GROUPS: Record<CourseGroup, { label: string; explain: string }> = {
  plan: { label: "Clean courses", explain: "Ten courses from a 3 m straight to a figure eight, a wall hug and a full spin while driving." },
  disturbed: { label: "Disturbed", explain: "The S-curve with a 0.3 s shove halfway, on low grip (61 %), or with the grip dropping halfway." },
};

function n(v: number, d = 2): string {
  return Number.isFinite(v) ? v.toFixed(d) : "–";
}

export function HeadToHead() {
  const [data, setData] = useState<MatchedData | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [rule, setRule] = useState("equalPush");
  const [pose, setPose] = useState("truth");
  const [group, setGroup] = useState<CourseGroup>("plan");
  const [all, setAll] = useState(false);

  useEffect(() => {
    let live = true;
    fetch(MATCHED_URL)
      .then((r) => (r.ok ? r.text() : Promise.reject(new Error(`HTTP ${r.status}`))))
      .then((t) => live && setData(parseMatched(t)))
      .catch((e: Error) => live && setError(`No matched-limits snapshot (${e.message}).`));
    return () => {
      live = false;
    };
  }, []);

  const methods = useMemo(() => (data ? data.methods.map((m) => m.key).filter((m) => all || PRIMARY.includes(m)) : []), [data, all]);

  if (error) return <p className="check error">{error}</p>;
  if (!data) return <p className="stats">Loading the head-to-head…</p>;

  const r = data.rules[rule]!;
  const board = scoreboard(data, rule, pose, group, methods);
  const plans = planSums(data, rule);
  const courses = data.courses[group];
  const lines = verdict(data, rule, pose, methods);

  return (
    <section className="bench-section" aria-labelledby="bench-h2h-title" data-testid="bench-h2h">
      <div className="bench-section-head">
        <h2 id="bench-h2h-title">Every library at the same limits</h2>
      </div>
      <p className="hint">
        Each planner plans to the same physics: the traction limit 0.9 × grip ({n(r.traction)} m/s² under {RULE_LABELS[rule]}), the
        same 4 m/s cap (5 m/s on fast_long) and the same robot. Ours, Choreo and PathPlanner read it from their robot models; path2 and
        BLine from their acceleration caps. Each follower keeps its tuned gains. {data.seeds} seeds per course; the bench plant is a
        500 Hz swerve with stick-slip tyres.
      </p>
      <ul className="verdict" data-testid="bench-h2h-verdict">
        {lines.map((l) => (
          <li key={l}>{l}</li>
        ))}
      </ul>
      <div className="bench-controls-row">
        <div className="field">
          <span className="field-label">Traction rule</span>
          <Segmented
            label="Traction rule"
            value={rule}
            options={Object.keys(data.rules).map((k) => ({ value: k, label: RULE_LABELS[k] ?? k }))}
            onChange={setRule}
          />
        </div>
        <div className="field">
          <span className="field-label">Pose source</span>
          <Segmented label="Pose source" value={pose} options={Object.keys(POSES).map((k) => ({ value: k, label: POSES[k]!.label }))} onChange={setPose} />
        </div>
        <div className="field">
          <span className="field-label">Courses</span>
          <Segmented
            label="Courses"
            value={group}
            options={(Object.keys(GROUPS) as CourseGroup[]).map((k) => ({ value: k, label: GROUPS[k].label }))}
            onChange={setGroup}
          />
        </div>
        <Switch label="+ ours following Choreo's plan" checked={all} onChange={setAll} testId="bench-h2h-all" />
      </div>
      <p className="hint">
        {POSES[pose]!.explain} {GROUPS[group].explain}
      </p>

      <div className="bench-table-scroll">
        <table className="benchmark" data-testid="bench-h2h-scoreboard">
          <thead>
            <tr>
              <th className="target">Method</th>
              <th data-tip="Courses where it settled first, counting only methods whose every run settled">Courses won</th>
              <th data-tip="Runs that never got within 2 cm and 1° and held it">Runs that never settle</th>
              <th data-tip="Summed over the courses: the mean time to settle where it did">Time settled, s</th>
              <th data-tip="Summed over the courses: the mean time to first get within 5 cm of the end">Time to 5 cm, s</th>
              <th data-tip="Mean distance from the true end pose when the run ends">End error, cm</th>
              <th data-tip="RMS distance from its own planned path">Off its own path, cm</th>
              {group === "plan" && <th data-tip="The planner's own planned time, summed over the courses">Planned, s</th>}
            </tr>
          </thead>
          <tbody>
            {board.map((s) => {
              const ours = OURS.has(s.method);
              const best = (key: "wins" | "never" | "settledSum" | "endErrorCm" | "offPathCm" | "within5Sum", low: boolean) => {
                const vals = board.map((b) => b[key]).filter(Number.isFinite);
                const target = low ? Math.min(...vals) : Math.max(...vals);
                return s[key] === target ? "best" : "";
              };
              const cls = (b: string) => [ours ? "ours" : "", b].filter(Boolean).join(" ");
              return (
                <tr key={s.method} data-method={s.method}>
                  <th className={`target ${ours ? "ours" : ""}`}>{label(data, s.method)}</th>
                  <td className={cls(best("wins", false))}>
                    {s.wins} / {s.courses}
                  </td>
                  <td className={cls(best("never", true))}>
                    {s.never} / {s.runs}
                  </td>
                  <td className={cls(s.never === 0 ? best("settledSum", true) : "")} data-tip={s.never ? "mean over the runs that settled" : undefined}>
                    {n(s.settledSum)}
                    {s.never > 0 && Number.isFinite(s.settledSum) ? "*" : ""}
                  </td>
                  <td className={cls(best("within5Sum", true))}>{n(s.within5Sum)}</td>
                  <td className={cls(best("endErrorCm", true))}>{n(s.endErrorCm, 1)}</td>
                  <td className={cls(best("offPathCm", true))}>{n(s.offPathCm, 1)}</td>
                  {group === "plan" && (
                    <td className={cls(plans[s.method] !== undefined && plans[s.method] === Math.min(...Object.values(plans)) ? "best" : "")}>
                      {plans[s.method] !== undefined ? n(plans[s.method]!) : "–"}
                    </td>
                  )}
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
      <p className="hint">
        Bold: the best in the column. * a time over only the runs that settled. Settled: {data.settleRule}. A course is won by the fastest
        to settle among methods whose every run settled and ended within {END_TOLERANCE_CM} cm. Planned: only planners that make a timed
        plan; path2 and BLine steer to waypoints as they go.
      </p>

      <div className="bench-table-scroll">
        <table className="benchmark" data-testid="bench-h2h-courses">
          <caption className="bench-caption">Time settled per course, s (✕ k/{data.seeds}: settled on k seeds)</caption>
          <thead>
            <tr>
              <th className="target">Course</th>
              {methods.map((m) => (
                <th key={m} className={OURS.has(m) ? "ours" : ""}>
                  {label(data, m)}
                </th>
              ))}
            </tr>
          </thead>
          <tbody>
            {courses.map((c) => {
              const win = courseWinner(data, rule, pose, c, methods);
              return (
                <tr key={c}>
                  <th className="target" data-tip={data.courseLabels[c]}>
                    {c}
                  </th>
                  {methods.map((m) => {
                    const s = stats(data, rule, pose, c, m);
                    const cls = [OURS.has(m) ? "ours" : "", win === m ? "best" : ""].filter(Boolean).join(" ");
                    if (!s) return <td key={m} className={cls}>–</td>;
                    const tip = `to 5 cm ${n(s.t_5cm ?? Number.NaN)} s; ends ${n(s.err_end_cm ?? Number.NaN, 1)} cm off; ${n(s.ct_own_rms_cm ?? Number.NaN, 1)} cm RMS off its path`;
                    return (
                      <td key={m} className={cls} data-tip={tip}>
                        {allSettled(s) ? (
                          n(s.t_settle!)
                        ) : (
                          <span className="void-cell">
                            {s.t_settle_nan === s.n ? "✕ never" : s.t_settle_nan > 0 ? `✕ ${s.n - s.t_settle_nan}/${s.n}` : `✕ ${n(s.err_end_cm ?? Number.NaN, 1)} cm`}
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
      <p className="hint">
        Why Choreo plans fastest but rarely wins: its solver picks the line and the heading freely between the waypoints, while ours plans
        a spline the optimiser shapes. ChoreoLib's follower runs on a clock: it catches up after a lag by overshooting, and when the clock
        runs out it stops wherever it is. Ours follows by position and brakes onto the end pose, so it settles every run, recovers from a
        shove and can re-plan on the fly.
      </p>
    </section>
  );
}
