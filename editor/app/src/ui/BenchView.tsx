// The Bench page (read-only): the bench's results for one course, read fairly. It leads with a
// scoreboard ranked by time settled, then reliability, then final error, and only runs that settled and
// ended in tolerance can be ranked or bold. Below it is every metric. The course is on the field at a
// fixed spot, with each method's authored path and true trace.

import { useEffect, useRef } from "react";
import {
  bestValidByMetric,
  COURSE_ORIGIN,
  methodColour,
  POSE_LABELS,
  PP_PREFIX,
  PRIMARY_METHODS,
  scoreboard,
  tableFor,
  tableStat,
  validity,
  VALID_RULE,
  type BenchData,
  type BenchTable,
} from "../bench/benchData.ts";
import { bench, loadBench, toggleShown, type BenchState } from "../state/bench.ts";
import { DEFAULT_VIEW, editor } from "../state/editor.ts";
import { useStore } from "../state/store.ts";
import { Eye, EyeOff } from "lucide-react";
import { formatNumber, IconButton, Segmented, SelectField, Switch } from "./controls.tsx";
import { drawFieldImage, drawGrid } from "./draw.ts";
import { FieldView } from "./fieldView.ts";

/** The methods listed: ours and each rival's best first, the rest when asked for. */
function listedMethods(t: BenchTable, more: boolean): string[] {
  const keys = t.methods.map((m) => m.key);
  const primary = PRIMARY_METHODS.filter((m) => keys.includes(m));
  return more ? [...primary, ...keys.filter((k) => !primary.includes(k))] : primary;
}

function drawBench(ctx: CanvasRenderingContext2D, width: number, height: number, image: HTMLImageElement | null, s: BenchState): void {
  const project = editor.get().project;
  const view = new FieldView(width, height, project.field.lengthMeters, project.field.widthMeters, DEFAULT_VIEW, "RED_HALF_TURN");
  ctx.fillStyle = "#0b0e13";
  ctx.fillRect(0, 0, width, height);
  drawFieldImage(ctx, { view, image, project });
  drawGrid(ctx, { view, image, project });
  const data = s.data;
  if (!data) return;
  const t = tableFor(data, s.course);
  const target = t.targets.find((x) => x.key === s.course);
  const label = (text: string, x: number, y: number) => {
    const [px, py] = view.toCanvasRaw(x, y);
    ctx.font = "600 12px system-ui, sans-serif";
    const w = ctx.measureText(text).width;
    ctx.fillStyle = "rgba(11,14,19,0.82)";
    ctx.fillRect(px - 4, py - 14, w + 8, 20);
    ctx.fillStyle = "#e6ebf2";
    ctx.fillText(text, px, py);
  };
  const polyline = (pts: readonly (readonly number[])[], xi: number, dx: number, dy: number, colour: string, widthPx: number, dash: number[] = []) => {
    if (pts.length < 2) return;
    ctx.strokeStyle = colour;
    ctx.lineWidth = widthPx;
    ctx.setLineDash(dash);
    ctx.lineJoin = "round";
    ctx.beginPath();
    pts.forEach((p, i) => {
      const [px, py] = view.toCanvasRaw(p[xi]! + dx, p[xi + 1]! + dy);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.stroke();
    ctx.setLineDash([]);
  };

  if (target?.onField) {
    // a 2026 route, where it is on the field: its waypoints, leg by leg
    for (const leg of target.legs ?? []) {
      polyline(leg, 0, 0, 0, "rgba(255,255,255,0.55)", 4);
      for (const [x, y] of leg) {
        const [px, py] = view.toCanvasRaw(x!, y!);
        ctx.fillStyle = "#fbbf24";
        ctx.beginPath();
        ctx.arc(px, py, 4, 0, 2 * Math.PI);
        ctx.fill();
      }
    }
    const first = target.legs?.[0]?.[0];
    if (first) label("2026 field route (waypoints; each method plans its own path through them)", first[0]!, first[1]! + 0.55);
    return;
  }

  // a bench course: always at the same spot, and labelled as what it is
  const [dx, dy] = COURSE_ORIGIN;
  const geometry = data.geometry[s.course] ?? {};
  polyline(geometry.neutral ?? [], 0, dx, dy, "rgba(255,255,255,0.3)", 7);
  for (const m of s.shown) {
    polyline(geometry[m] ?? [], 0, dx, dy, methodColour(m), 1.5, [5, 4]);
    if (s.traces) {
      const trace = data.traces[s.course]?.[s.seed]?.[m] ?? [];
      polyline(trace, 1, dx, dy, methodColour(m), 2.5);
      const end = trace.at(-1);
      if (end) {
        const [px, py] = view.toCanvasRaw(end[1]! + dx, end[2]! + dy);
        ctx.fillStyle = methodColour(m);
        ctx.beginPath();
        ctx.arc(px, py, 4.5, 0, 2 * Math.PI);
        ctx.fill();
      }
    }
  }
  label("Bench course, not a field route", dx, dy - 0.45);
}

function BenchCanvas() {
  const wrap = useRef<HTMLDivElement>(null);
  const canvas = useRef<HTMLCanvasElement>(null);
  const image = useRef<HTMLImageElement | null>(null);
  useEffect(() => {
    const draw = () => {
      const c = canvas.current;
      const w = wrap.current;
      if (!c || !w) return;
      const rect = w.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      c.width = Math.max(1, Math.round(rect.width * dpr));
      c.height = Math.max(1, Math.round(rect.height * dpr));
      const ctx = c.getContext("2d")!;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      drawBench(ctx, rect.width, rect.height, image.current, bench.get());
    };
    const img = new Image();
    img.onload = () => {
      image.current = img;
      draw();
    };
    img.src = `${import.meta.env.BASE_URL}field-2026.png`;
    const observer = new ResizeObserver(draw);
    observer.observe(wrap.current!);
    const unsubscribe = bench.subscribe(draw);
    return () => {
      observer.disconnect();
      unsubscribe();
    };
  }, []);
  return (
    <div ref={wrap} className="field-wrap">
      <canvas ref={canvas} className="field-canvas" role="img" aria-label="The chosen course with each shown method's path and true trace" />
    </div>
  );
}

const num = (v: number | null | undefined, d = 2) => (v === null || v === undefined || Number.isNaN(v) ? "-" : formatNumber(v, d));

function Swatch({ method }: { method: string }) {
  return <span className="swatch" style={{ background: methodColour(method) }} aria-hidden="true" />;
}

function Scoreboard({ t, s }: { t: BenchTable; s: BenchState }) {
  const rows = scoreboard(t, s.pose, s.course, listedMethods(t, s.moreMethods));
  const planned = t.metrics.some((m) => m.key === "plan_s");
  return (
    <section className="scoreboard" aria-label="Scoreboard">
      <table className="bench-table" data-testid="bench-scoreboard">
        <caption>
          Scoreboard: ranked by time settled, then seeds settled, then final error. {POSE_LABELS[s.pose] ?? s.pose}.
        </caption>
        <thead>
          <tr>
            <th scope="col">#</th>
            <th scope="col">Method</th>
            {planned && <th scope="col">Planned s</th>}
            <th scope="col">Time settled s</th>
            <th scope="col">Settled</th>
            <th scope="col">Final error cm</th>
            <th scope="col">Cross-track RMS cm</th>
          </tr>
        </thead>
        <tbody>
          {rows.map((r) => {
            const st = (key: string) => tableStat(t, s.pose, s.course, r.method, key);
            return (
              <tr key={r.method} className={r.validity.valid ? undefined : "invalid"} data-method={r.method} data-valid={r.validity.valid}>
                <td className="rank">{r.rank ?? "-"}</td>
                <th scope="row">
                  <Swatch method={r.method} /> {r.label}
                  {!r.validity.valid && <span className="tag danger">{r.validity.reason}</span>}
                </th>
                {planned && <td>{num(st("plan_s")?.mean)}</td>}
                <td className={r.rank === 1 ? "best" : undefined}>{r.validity.valid ? num(st("t_settle")?.mean) : <span className="void">{num(st("t_settle")?.mean)}</span>}</td>
                <td>
                  {r.validity.settled}/{r.validity.n}
                </td>
                <td>
                  {num(st("err_end_cm")?.mean, 1)} <span className="worst">({num(st("err_end_cm")?.worst, 1)})</span>
                </td>
                <td>{num(st("ct_rms_cm")?.mean, 2)}</td>
              </tr>
            );
          })}
        </tbody>
      </table>
      <p className="hint" data-testid="bench-valid-rule">
        {VALID_RULE}
      </p>
    </section>
  );
}

/** Time columns: a run that didn't settle or ended out of tolerance has no time worth comparing. */
const TIME_METRICS = new Set(["t_done", "t_settle", "plan_s"]);

function ResultsTable({ t, s, seeds }: { t: BenchTable; s: BenchState; seeds: number }) {
  const methods = listedMethods(t, s.moreMethods).filter((m) => t.runs[s.pose]?.[s.course]?.[m] !== undefined);
  const best = bestValidByMetric(t, s.pose, s.course, methods);
  return (
    <table className="bench-table" data-testid="bench-table">
      <caption>
        Every metric: mean (worst) over {seeds} seeds. Bold: the lowest mean among valid runs. Greyed: a time that doesn&apos;t count.
      </caption>
      <thead>
        <tr>
          <th scope="col">Show</th>
          <th scope="col">Method</th>
          {t.metrics.map((m) => (
            <th scope="col" key={m.key}>
              {m.label} <span className="unit">{m.unit}</span>
            </th>
          ))}
        </tr>
      </thead>
      <tbody>
        {methods.map((key) => {
          const label = t.methods.find((m) => m.key === key)?.label ?? key;
          const v = validity(t, s.pose, s.course, key);
          return (
            <tr key={key} className={v.valid ? undefined : "invalid"} data-method={key}>
              <td>
                <IconButton label={`Show ${label} on the field`} pressed={s.shown.includes(key)} onClick={() => toggleShown(key)} className="small">
                  {s.shown.includes(key) ? <Eye size={14} /> : <EyeOff size={14} />}
                </IconButton>
              </td>
              <th scope="row">
                <Swatch method={key} /> {label}
                {!v.valid && <span className="tag danger">{v.reason}</span>}
              </th>
              {t.metrics.map((metric) => {
                const st = tableStat(t, s.pose, s.course, key, metric.key);
                const voided = !v.valid && TIME_METRICS.has(metric.key);
                return (
                  <td key={metric.key} className={best[metric.key] === key ? "best" : voided ? "void" : undefined} title={voided ? v.reason : undefined}>
                    {num(st?.mean)}
                    {st?.worst !== null && st?.worst !== undefined && <span className="worst"> ({formatNumber(st.worst, 2)})</span>}
                    {st && st.nan > 0 && <span className="worst"> [{st.nan} n/a]</span>}
                  </td>
                );
              })}
            </tr>
          );
        })}
      </tbody>
    </table>
  );
}

function courseOptions(data: BenchData): { value: string; label: string }[] {
  const courses = data.courses.map((c) => ({ value: c.key, label: `${c.key} (bench course)` }));
  const routes = (data.ppcompare?.targets ?? []).map((t) => ({ value: PP_PREFIX + t.key, label: `${t.key} (${t.onField ? "2026 route" : "course"}, PathPlanner comparison)` }));
  return [...courses, ...routes];
}

export function BenchView() {
  const s = useStore(bench, (b) => b);
  const data = s.data;
  const t = data ? tableFor(data, s.course) : null;
  const target = t?.targets.find((x) => x.key === s.course);
  const isRoute = s.course.startsWith(PP_PREFIX);
  return (
    <main className="bench-page" aria-label="Bench comparison (read-only)">
      <section className="bench-column" aria-label="The course on the field">
        <BenchCanvas />
        {data && t && (
          <div className="bench-table-wrap">
            <Scoreboard t={t} s={s} />
            <h3 className="subhead bench-subhead">All metrics</h3>
            <ResultsTable t={t} s={s} seeds={isRoute ? (data.ppcompare?.meta.seeds.length ?? 0) : data.meta.seeds.length} />
          </div>
        )}
      </section>
      <aside className="bench-side" aria-label="Bench choices">
        <div className="bench-controls">
          <h2 className="subhead">Bench comparison</h2>
          {s.loadingSnapshot && <p className="stats">Loading the snapshot…</p>}
          {s.error && (
            <p className="check error" role="alert">
              {s.error}
            </p>
          )}
          {data && t && (
            <>
              <SelectField
                label="Course"
                value={s.course}
                options={courseOptions(data)}
                onChange={(course) => {
                  const poses = tableFor(data, course).poses;
                  bench.set({ course, pose: poses.includes(s.pose) ? s.pose : poses.includes("truth") ? "truth" : (poses[0] ?? "") });
                }}
                testId="bench-course"
              />
              {target && (
                <div className="bench-course-note">
                  <p className="hint">{target.description}</p>
                  {target.look && (
                    <p className="look" data-testid="bench-look">
                      <strong>What to look at:</strong> {target.look}
                    </p>
                  )}
                </div>
              )}
              <div className="field">
                <span className="field-label">Pose source</span>
                <Segmented
                  label="Pose source"
                  value={s.pose}
                  options={t.poses.map((p) => ({ value: p, label: p === "truth" ? "Perfect" : p === "odom" ? "Odometry" : p === "slipodom" ? "Slip-aware" : p, tip: POSE_LABELS[p] }))}
                  onChange={(pose) => bench.set({ pose })}
                />
                <span className="hint">{POSE_LABELS[s.pose] ?? s.pose}</span>
              </div>
              {!isRoute && (
                <>
                  <SelectField
                    label={`Trace seed (pose: ${POSE_LABELS[data.meta.tracePose] ?? data.meta.tracePose})`}
                    value={s.seed}
                    options={data.meta.traceSeeds.map((x) => ({ value: String(x), label: String(x) }))}
                    onChange={(seed) => bench.set({ seed })}
                  />
                  <Switch label="True traces (solid); authored paths dashed" checked={s.traces} onChange={(on) => bench.set({ traces: on })} />
                </>
              )}
              <div className="field">
                <span className="field-label">On the field</span>
                <ul className="legend" aria-label="Methods">
                  {listedMethods(t, s.moreMethods).map((m) => (
                    <li key={m}>
                      <button type="button" className="method-chip" aria-pressed={s.shown.includes(m)} onClick={() => toggleShown(m)} data-testid={`show-${m}`}>
                        <Swatch method={m} /> {t.methods.find((x) => x.key === m)?.label ?? m}
                      </button>
                    </li>
                  ))}
                </ul>
                <Switch
                  label="More methods (README gains, as-shipped variants)"
                  checked={s.moreMethods}
                  onChange={(on) => bench.set({ moreMethods: on })}
                  testId="bench-more-methods"
                />
              </div>
              <p className="stats">
                {s.fileName}: {data.courses.length} courses, {data.methods.length} methods, {data.meta.seeds.length} seeds
                {data.ppcompare ? `; PathPlanner comparison: ${data.ppcompare.targets.length} targets` : ""}
              </p>
            </>
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
        </div>
      </aside>
    </main>
  );
}
