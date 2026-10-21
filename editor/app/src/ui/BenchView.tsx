// The Bench page (read-only), laid out like a model launch page: a verdict written from the data, one
// benchmark table across every course and route (methods as columns, ours tinted, the best valid result
// per row in bold), bar charts comparing the methods on one metric at a time, the optimised-paths table
// from the PathPlanner comparison, and then the course detail (the field, the scoreboard, every metric).

import { ChevronDown } from "lucide-react";
import { useMemo } from "react";
import {
  aggregate,
  cell,
  commonRows,
  COLUMNS,
  COMPARE_METRICS,
  END_TOLERANCE_CM,
  OPTIMISED_COLUMNS,
  perTarget,
  POSE_LABELS,
  PP_PREFIX,
  rowWinner,
  rows,
  tableFor,
  tableStat,
  validity,
  verdict,
  type Bar,
  type BenchData,
  type Column,
  type CompareMetric,
  type Row,
} from "../bench/benchData.ts";
import { bench, loadBench } from "../state/bench.ts";
import { useStore } from "../state/store.ts";
import { formatNumber, Segmented, Switch } from "./controls.tsx";
import { CourseDetail } from "./bench/CourseDetail.tsx";

const POSE_SHORT: Record<string, string> = { truth: "Perfect pose", odom: "Wheel odometry", slipodom: "Slip-aware" };

const POSE_EXPLAIN: Record<string, string> = {
  truth: "The follower alone: it is told exactly where the robot is, so these are the control law's own results.",
  odom: "As on a robot without vision: wheel odometry and the gyro, which drift, so ends are judged against the true pose.",
  slipodom: "Wheel odometry with lib's slip-aware correction (gyro and IMU), no vision.",
};

function shownColumns(more: boolean): Column[] {
  return COLUMNS.filter((c) => more || !c.more);
}

function fmt(v: number, unit: string): string {
  if (!Number.isFinite(v)) return "-";
  return v.toFixed(unit === "cm" ? 1 : unit === "ms" || unit === "%" ? 0 : 2);
}

/** A cell's text: the value, or why it doesn't count. */
function CellText({ data, pose, c, r }: { data: BenchData; pose: string; c: Column; r: Row }) {
  const x = cell(data, pose, c, r, "t_settle");
  if (!x.method) return <span className="na" data-tip="Not run: the 2026 routes (run.py --pp-compare) ran only PathPlanner and ours">–</span>;
  if (!x.validity?.valid) {
    const v = x.validity!;
    const text = v.reason === "never settled" ? "✕ never" : v.reason.endsWith("never settled") ? `✕ ${v.settled}/${v.n}` : v.reason;
    return (
      <span className="void-cell" data-tip={`${v.reason}${Number.isFinite(x.value) ? `; mean settle ${formatNumber(x.value, 2)} s where it did` : ""}`}>
        {text}
      </span>
    );
  }
  return <span>{x.value.toFixed(2)}</span>;
}

function BenchmarkTable({ data, pose, cols }: { data: BenchData; pose: string; cols: readonly Column[] }) {
  const rs = rows(data);
  const wins = new Map<string, number>();
  for (const r of rs) {
    const w = rowWinner(data, pose, cols, r, "t_settle");
    if (w) wins.set(w, (wins.get(w) ?? 0) + 1);
  }
  const common = commonRows(data, pose, cols, rs, "t_settle", true);
  const mean = (c: Column) => {
    const xs = common.map((r) => cell(data, pose, c, r, "t_settle").value);
    return xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : Number.NaN;
  };
  const means = cols.map(mean);
  const bestMean = Math.min(...means.filter(Number.isFinite));
  return (
    <div className="bench-table-scroll">
      <table className="benchmark" data-testid="bench-benchmark">
        <caption className="sr-only">Time settled in seconds, by target and method</caption>
        <thead>
          <tr>
            <th scope="col" className="target">
              Time settled, s
            </th>
            {cols.map((c) => (
              <th scope="col" key={c.id} className={c.id === "ours" ? "ours" : undefined} data-column={c.id}>
                {c.label}
              </th>
            ))}
          </tr>
        </thead>
        <tbody>
          {rs.map((r) => {
            const winner = rowWinner(data, pose, cols, r, "t_settle");
            return (
              <tr key={r.key} data-row={r.key}>
                <th scope="row" className="target">
                  <button
                    type="button"
                    className="link-row"
                    onClick={() => {
                      bench.set({ course: r.key });
                      document.getElementById("bench-detail")?.scrollIntoView({ behavior: "smooth", block: "start" });
                    }}
                    data-tip="Open this target in the course detail"
                  >
                    {r.label}
                    {r.route && <span className="tag">2026 route</span>}
                  </button>
                </th>
                {cols.map((c) => (
                  <td key={c.id} className={[c.id === "ours" ? "ours" : "", winner === c.id ? "best" : ""].filter(Boolean).join(" ") || undefined} data-column={c.id}>
                    <CellText data={data} pose={pose} c={c} r={r} />
                  </td>
                ))}
              </tr>
            );
          })}
        </tbody>
        <tfoot>
          <tr data-row="wins">
            <th scope="row" className="target">
              Targets won
            </th>
            {cols.map((c) => (
              <td key={c.id} className={c.id === "ours" ? "ours" : undefined} data-column={c.id}>
                {wins.get(c.id) ?? 0}
              </td>
            ))}
          </tr>
          <tr data-row="mean">
            <th
              scope="row"
              className="target"
              data-tip={`Over the targets every method shown settled on, so none is flattered by skipping the hard ones: ${common.map((r) => r.label).join(", ") || "none"}`}
              data-testid="bench-mean-targets"
            >
              Mean time settled ({common.length} {common.length === 1 ? "target" : "targets"} all settled)
            </th>
            {cols.map((c, i) => (
              <td key={c.id} className={[c.id === "ours" ? "ours" : "", means[i] === bestMean ? "best" : ""].filter(Boolean).join(" ") || undefined} data-column={c.id}>
                {Number.isFinite(means[i]!) ? means[i]!.toFixed(2) : "-"}
              </td>
            ))}
          </tr>
        </tfoot>
      </table>
    </div>
  );
}

/** Horizontal bars, best first; ours in the accent, the others in one quiet grey; values at the tips. */
function BarChart({ bars, unit, compact, testId }: { bars: readonly Bar[]; unit: string; compact?: boolean; testId?: string }) {
  const lo = Math.min(0, ...bars.map((b) => b.value));
  const hi = Math.max(0, ...bars.map((b) => b.value));
  const span = hi - lo || 1;
  const zero = ((0 - lo) / span) * 100;
  return (
    <ol className={compact ? "bars compact" : "bars"} data-testid={testId}>
      {bars.map((b) => {
        const left = ((Math.min(0, b.value) - lo) / span) * 100;
        const width = (Math.abs(b.value) / span) * 100;
        const ours = b.column.id === "ours";
        return (
          <li
            key={b.column.id}
            className={ours ? "ours" : undefined}
            data-column={b.column.id}
            tabIndex={0}
            data-tip={`${b.column.label}: ${fmt(b.value, unit)} ${unit} (${b.over} ${b.over === 1 ? "target" : "targets"})`}
          >
            <span className="bar-label">{compact ? b.column.short : b.column.label}</span>
            <span className="bar-track">
              {lo < 0 && <span className="bar-zero" style={{ left: `${zero}%` }} />}
              <span className={b.value < 0 ? "bar neg" : "bar"} style={{ left: `${left}%`, width: `max(2px, ${width}%)` }} />
              <span className="bar-value" style={b.value < 0 ? { left: `calc(${left}% - 4px)`, transform: "translateX(-100%)" } : { left: `calc(${left + width}% + 6px)` }}>
                {fmt(b.value, unit)}
              </span>
            </span>
          </li>
        );
      })}
    </ol>
  );
}

function CompareCharts({ data, pose, cols }: { data: BenchData; pose: string; cols: readonly Column[] }) {
  const metric = useStore(bench, (s) => s.compareMetric as CompareMetric);
  const view = useStore(bench, (s) => s.compareView);
  const m = COMPARE_METRICS.find((x) => x.id === metric) ?? COMPARE_METRICS[0]!;
  const rs = rows(data);
  const bars = useMemo(() => aggregate(data, pose, cols, rs, m.id), [data, pose, cols, rs, m.id]);
  const better = m.higherIsBetter ? "higher is better" : "lower is better";
  return (
    <section className="bench-section" aria-labelledby="bench-compare-title" data-testid="bench-charts">
      <div className="bench-section-head">
        <h2 id="bench-compare-title">How we compare</h2>
        <Segmented
          label="Charts"
          value={view}
          options={[
            { value: "aggregate", label: "All targets" },
            { value: "per", label: "Per target" },
          ]}
          onChange={(compareView) => bench.set({ compareView })}
        />
      </div>
      <div className="metric-switch" role="radiogroup" aria-label="Metric">
        {COMPARE_METRICS.map((x) => (
          <button key={x.id} type="button" role="radio" aria-checked={x.id === m.id} className="metric-pill" onClick={() => bench.set({ compareMetric: x.id })}>
            {x.label}
          </button>
        ))}
      </div>
      <p className="hint">
        {m.label} ({m.unit}, {better}): {view === "aggregate" ? m.note : "on each target, every method with a value there (for the times, runs that count)"}.{" "}
        {POSE_SHORT[pose] ?? pose}.
      </p>
      {view === "aggregate" ? (
        bars.length ? (
          <BarChart bars={bars} unit={m.unit} testId="bench-bars" />
        ) : (
          <p className="hint">No target has a value from every method shown for this metric with this pose source.</p>
        )
      ) : (
        <div className="multiples" data-testid="bench-multiples">
          {rs.map((r) => {
            const b = perTarget(data, pose, cols, r, m.id);
            const failed = cols.filter((c) => {
              const x = cell(data, pose, c, r, "t_settle");
              return x.method && !x.validity?.valid;
            });
            return (
              <figure key={r.key} className="multiple">
                <figcaption>{r.label}</figcaption>
                {b.length ? <BarChart bars={b} unit={m.unit} compact /> : <p className="hint">No method shown has a value here.</p>}
                {(m.id === "t_settle" || m.id === "t_5cm") && failed.length > 0 && (
                  <p className="multiple-note">Left out (didn&apos;t settle or ended off): {failed.map((c) => c.label).join(", ")}</p>
                )}
                {r.route && <p className="multiple-note">2026 route: Choreo and BLine weren&apos;t run on the routes.</p>}
              </figure>
            );
          })}
        </div>
      )}
    </section>
  );
}

function OptimisedTable({ data, pose }: { data: BenchData; pose: string }) {
  const pp = data.ppcompare;
  if (!pp) return null;
  const cols = OPTIMISED_COLUMNS.filter((c) => pp.methods.some((m) => m.key === c.method));
  const t = (key: string) => tableFor(data, PP_PREFIX + key);
  return (
    <section className="bench-section" aria-labelledby="bench-opt-title">
      <div className="bench-section-head">
        <h2 id="bench-opt-title">Optimised paths</h2>
      </div>
      <p className="hint">
        The PathPlanner comparison: the same waypoints as drawn, and after each one&apos;s own optimiser (ours with <code>tools/optimise.mjs</code>,
        PathPlanner&apos;s control points with a port of the same search). Each cell: time settled, and the planned time under it.
      </p>
      {!pp.meta.poses.includes(pose) && <p className="check warning">The PathPlanner comparison wasn't run with {POSE_LABELS[pose] ?? pose}.</p>}
      <div className="bench-table-scroll">
        <table className="benchmark" data-testid="bench-optimised">
          <thead>
            <tr>
              <th scope="col" className="target">
                Time settled (plan), s
              </th>
              {cols.map((c) => (
                <th scope="col" key={c.method} className={c.ours ? "ours" : undefined}>
                  {c.label}
                </th>
              ))}
            </tr>
          </thead>
          <tbody>
            {pp.targets.map((target) => {
              const key = PP_PREFIX + target.key;
              const table = t(target.key);
              let best: string | null = null;
              let low = Infinity;
              for (const c of cols) {
                const v = tableStat(table, pose, key, c.method, "t_settle")?.mean;
                if (table.runs[pose]?.[key]?.[c.method] && validity(table, pose, key, c.method).valid && typeof v === "number" && v < low) {
                  low = v;
                  best = c.method;
                }
              }
              return (
                <tr key={target.key}>
                  <th scope="row" className="target">
                    {target.key}
                    {target.onField && <span className="tag">2026 route</span>}
                  </th>
                  {cols.map((c) => {
                    const run = table.runs[pose]?.[key]?.[c.method];
                    const settle = tableStat(table, pose, key, c.method, "t_settle")?.mean;
                    const plan = tableStat(table, pose, key, c.method, "plan_s")?.mean;
                    const v = run ? validity(table, pose, key, c.method) : null;
                    return (
                      <td key={c.method} className={[c.ours ? "ours" : "", best === c.method ? "best" : ""].filter(Boolean).join(" ") || undefined}>
                        {!run ? (
                          <span className="na">–</span>
                        ) : v?.valid ? (
                          (settle ?? Number.NaN).toFixed(2)
                        ) : (
                          <span className="void-cell" data-tip={v?.reason}>
                            {v?.reason === "never settled" ? "✕ never" : v?.reason}
                          </span>
                        )}
                        {run && typeof plan === "number" && <span className="plan">plan {plan.toFixed(2)}</span>}
                      </td>
                    );
                  })}
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </section>
  );
}

export function BenchView() {
  const s = useStore(bench, (b) => b);
  const data = s.data;
  const cols = useMemo(() => shownColumns(s.moreMethods), [s.moreMethods]);
  const poses = data ? data.meta.poses : [];
  return (
    <main className="bench-page" aria-label="Bench comparison (read-only)">
      <div className="bench-doc">
        <header className="bench-hero">
          <h1>How the followers compare</h1>
          {s.loadingSnapshot && <p className="stats">Loading the snapshot…</p>}
          {s.error && (
            <p className="check error" role="alert">
              {s.error}
            </p>
          )}
          {data && (
            <>
              <ul className="verdict" data-testid="bench-verdict">
                {verdict(data, s.pose, cols).map((line) => (
                  <li key={line}>{line}</li>
                ))}
              </ul>
              <div className="bench-controls-row">
                <div className="field">
                  <span className="field-label">Pose source</span>
                  <Segmented
                    label="Pose source"
                    value={s.pose}
                    options={poses.map((p) => ({ value: p, label: p === "truth" ? "Perfect" : p === "odom" ? "Odometry" : p === "slipodom" ? "Slip-aware" : p, tip: POSE_LABELS[p] }))}
                    onChange={(pose) => bench.set({ pose })}
                  />
                </div>
                <p className="hint pose-explain" data-testid="bench-pose-explain">
                  {POSE_EXPLAIN[s.pose] ?? POSE_LABELS[s.pose] ?? s.pose}
                </p>
                <Switch label="+ more methods (as shipped, README gains)" checked={s.moreMethods} onChange={(on) => bench.set({ moreMethods: on })} testId="bench-more-methods" />
                <a
                  className="link"
                  href="#bench-detail"
                  onClick={(e) => {
                    e.preventDefault();
                    document.getElementById("bench-detail")?.scrollIntoView({ behavior: "smooth", block: "start" });
                  }}
                >
                  Full tables <ChevronDown size={12} />
                </a>
              </div>
            </>
          )}
        </header>
        {data && (
          <>
            <section className="bench-section" aria-labelledby="bench-table-title">
              <div className="bench-section-head">
                <h2 id="bench-table-title">Every course and route</h2>
              </div>
              <BenchmarkTable data={data} pose={s.pose} cols={cols} />
              <p className="hint" data-testid="bench-valid-rule-top">
                Bold: the fastest to settle among runs that count. A run counts if every seed settled (within 2 cm and 1°, held 0.25 s) and it
                ended within {END_TOLERANCE_CM} cm; &quot;✕ never&quot; never settled, &quot;✕ 3/10&quot; settled on 3 of 10 seeds. Click a target for its detail.
              </p>
              {rows(data).some((r) => r.route) && (
                <p className="hint" data-testid="bench-routes-note">
                  – not run: the 2026 routes come from the PathPlanner comparison (<code>run.py --pp-compare</code>), which ran only PathPlanner and
                  ours.
                </p>
              )}
            </section>
            <CompareCharts data={data} pose={s.pose} cols={cols} />
            <OptimisedTable data={data} pose={s.pose} />
            <CourseDetail data={data} />
          </>
        )}
        <footer className="bench-section bench-data">
          {data && (
            <p className="stats">
              {s.fileName}: {data.courses.length} courses, {data.methods.length} methods, {data.meta.seeds.length} seeds
              {data.ppcompare ? `; PathPlanner comparison: ${data.ppcompare.targets.length} targets` : ""}.
            </p>
          )}
          <p className="hint">
            Read-only. It opens with the committed snapshot. For the full export, run <code>python bench/run.py --export-editor</code> and load{" "}
            <code>bench/out/editor_bench.json</code>; that also refreshes the snapshot.
          </p>
          <label className="field">
            <span className="field-label">Load a bench export</span>
            <input
              type="file"
              className="file-input"
              accept=".json,application/json"
              data-testid="bench-file"
              onChange={async (e) => {
                const f = e.target.files?.[0];
                if (f) loadBench(await f.text(), f.name);
                e.target.value = "";
              }}
            />
          </label>
        </footer>
      </div>
    </main>
  );
}
