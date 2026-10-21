// The Bench view (read-only): the bench's results for one course, as a table per pose source, and each
// method's authored path and true trace for a chosen seed, drawn on the field.

import { useEffect, useRef } from "react";
import { bestByMetric, courseOffset, methodColour, stat, type BenchData } from "../bench/benchData.ts";
import { bench, loadBench, toggleShown, type BenchState } from "../state/bench.ts";
import { DEFAULT_VIEW, editor } from "../state/editor.ts";
import { useStore } from "../state/store.ts";
import { formatNumber, SelectField, Switch } from "./controls.tsx";
import { drawFieldImage, drawGrid } from "./draw.ts";
import { FieldView } from "./fieldView.ts";

function drawBench(ctx: CanvasRenderingContext2D, width: number, height: number, image: HTMLImageElement | null, s: BenchState): void {
  const project = editor.get().project;
  const view = new FieldView(width, height, project.field.lengthMeters, project.field.widthMeters, DEFAULT_VIEW, "RED_HALF_TURN");
  ctx.fillStyle = "#0b0f14";
  ctx.fillRect(0, 0, width, height);
  drawFieldImage(ctx, { view, image, project });
  drawGrid(ctx, { view, image, project });
  const data = s.data;
  if (!data) return;
  const [dx, dy] = courseOffset(data, s.course);
  const polyline = (pts: readonly (readonly number[])[], xi: number, colour: string, widthPx: number, dash: number[] = []) => {
    if (pts.length < 2) return;
    ctx.strokeStyle = colour;
    ctx.lineWidth = widthPx;
    ctx.setLineDash(dash);
    ctx.beginPath();
    pts.forEach((p, i) => {
      const [px, py] = view.toCanvasRaw(p[xi]! + dx, p[xi + 1]! + dy);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.stroke();
    ctx.setLineDash([]);
  };
  const geometry = data.geometry[s.course] ?? {};
  polyline(geometry.neutral ?? [], 0, "rgba(255,255,255,0.35)", 6);
  for (const m of s.shown) {
    polyline(geometry[m] ?? [], 0, methodColour(m), 1.5, [5, 4]);
    if (s.traces) {
      const trace = data.traces[s.course]?.[s.seed]?.[m] ?? [];
      polyline(trace, 1, methodColour(m), 2.5);
      const end = trace.at(-1);
      if (end) {
        const [px, py] = view.toCanvasRaw(end[1]! + dx, end[2]! + dy);
        ctx.fillStyle = methodColour(m);
        ctx.beginPath();
        ctx.arc(px, py, 4, 0, 2 * Math.PI);
        ctx.fill();
      }
    }
  }
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

function ResultsTable({ data, s }: { data: BenchData; s: BenchState }) {
  const methods = data.methods.map((m) => m.key);
  const best = bestByMetric(data, s.pose, s.course, methods);
  return (
    <div className="bench-table-wrap">
      <table className="bench-table" data-testid="bench-table">
        <caption>
          {s.course}, pose from {s.pose}: mean (worst) over {data.meta.seeds.length} seeds. Lowest mean in bold.
        </caption>
        <thead>
          <tr>
            <th scope="col">Show</th>
            <th scope="col">Method</th>
            {data.metrics.map((m) => (
              <th scope="col" key={m.key}>
                {m.label} <span className="unit">{m.unit}</span>
              </th>
            ))}
          </tr>
        </thead>
        <tbody>
          {data.methods.map((m) => (
            <tr key={m.key}>
              <td>
                <input type="checkbox" aria-label={`Show ${m.label} on the field`} checked={s.shown.includes(m.key)} onChange={() => toggleShown(m.key)} />
              </td>
              <th scope="row">
                <span className="swatch" style={{ background: methodColour(m.key) }} aria-hidden="true" /> {m.label}
              </th>
              {data.metrics.map((metric) => {
                const v = stat(data, s.pose, s.course, m.key, metric.key);
                const text = v?.mean === null || v?.mean === undefined ? "-" : formatNumber(v.mean, 2);
                return (
                  <td key={metric.key} className={best[metric.key] === m.key ? "best" : undefined}>
                    {text}
                    {v?.worst !== null && v?.worst !== undefined && <span className="worst"> ({formatNumber(v.worst, 2)})</span>}
                    {v && v.nan > 0 && (
                      <span className="worst" title="runs without this value (e.g. never settled)">
                        {" "}
                        [{v.nan} n/a]
                      </span>
                    )}
                  </td>
                );
              })}
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

export function BenchView() {
  const s = useStore(bench, (b) => b);
  const data = s.data;
  return (
    <main className="bench-page" aria-label="Bench comparison (read-only)">
      <section className="bench-column" aria-label="The course on the field">
        <BenchCanvas />
        {data && <ResultsTable data={data} s={s} />}
      </section>
      <aside className="bench-side" aria-label="Bench choices">
        <div className="bench-controls">
          <h2 className="subhead">Bench comparison</h2>
          <p className="hint">
            Read-only. It opens with the committed snapshot (one trace seed, traces every 0.2 s). For the full export, run{" "}
            <code>python bench/run.py --export-editor</code> (add <code>--trace-seeds 0,1,2</code> for more seeds) and load{" "}
            <code>bench/out/editor_bench.json</code>; that also refreshes the snapshot.
          </p>
          {s.loadingSnapshot && <p className="stats">Loading the snapshot…</p>}
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
          {s.error && (
            <p className="check error" role="alert">
              {s.error}
            </p>
          )}
          {data && (
            <>
              <p className="stats">
                {s.fileName}: {data.courses.length} courses, {data.methods.length} methods, {data.meta.seeds.length} seeds
              </p>
              <SelectField label="Course" value={s.course} options={data.courses.map((c) => ({ value: c.key, label: c.key }))} onChange={(course) => bench.set({ course })} />
              <p className="hint">{data.courses.find((c) => c.key === s.course)?.description}</p>
              <SelectField label="Pose source (tables)" value={s.pose} options={data.meta.poses.map((p) => ({ value: p, label: p }))} onChange={(pose) => bench.set({ pose })} />
              <SelectField
                label={`Seed (traces, pose from ${data.meta.tracePose})`}
                value={s.seed}
                options={data.meta.traceSeeds.map((x) => ({ value: String(x), label: String(x) }))}
                onChange={(seed) => bench.set({ seed })}
              />
              <Switch label="True traces (solid); authored paths dashed" checked={s.traces} onChange={(on) => bench.set({ traces: on })} />
              <ul className="legend" aria-label="Shown on the field">
                {s.shown.map((m) => (
                  <li key={m}>
                    <span className="swatch" style={{ background: methodColour(m) }} aria-hidden="true" /> {data.methods.find((x) => x.key === m)?.label ?? m}
                  </li>
                ))}
              </ul>
              <p className="hint">The course is moved to the middle of the field; the bench runs it from (0, 0).</p>
            </>
          )}
        </div>
      </aside>
    </main>
  );
}
