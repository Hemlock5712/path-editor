// How well the robot tracked the whole run, over time (or over the path, with the toggle): cross-track,
// along-track and heading error, planned against actual speed and turn rate. The finish phase is
// shaded, events are marked where they fired, and every chart shares the cursor (click or drag to move it).

import { useId, useMemo, useRef } from "react";
import type { RecordedRun, TrackSample } from "../../nt4/recorder.ts";
import { setCursor, telemetry } from "../../state/telemetry.ts";
import { useStore } from "../../state/store.ts";
import { formatNumber } from "../controls.tsx";

const W = 300;
const H = 120;
const PAD = { left: 36, right: 6, top: 18, bottom: 16 };

interface Series {
  readonly label: string;
  readonly colour: string;
  readonly ys: readonly number[];
  readonly dashed?: boolean;
}

interface ChartProps {
  readonly title: string;
  readonly unit: string;
  readonly xs: readonly number[];
  readonly series: readonly Series[];
  readonly bands: readonly [number, number][];
  readonly markers: readonly { x: number; label: string }[];
  readonly cursor: number;
  readonly xUnit: string;
  readonly onSeek: (x: number) => void;
  readonly testId: string;
  /** Draw a line at zero. */
  readonly zero?: boolean;
}

function Chart({ title, unit, xs, series, bands, markers, cursor, xUnit, onSeek, testId, zero }: ChartProps) {
  const ref = useRef<SVGSVGElement>(null);
  const titleId = useId();
  const x0 = xs.length ? Math.min(...xs) : 0;
  const x1 = xs.length ? Math.max(...xs) : 1;
  const { lo, hi } = useMemo(() => {
    let lo = Infinity;
    let hi = -Infinity;
    for (const s of series) for (const y of s.ys) if (Number.isFinite(y)) [lo, hi] = [Math.min(lo, y), Math.max(hi, y)];
    if (!Number.isFinite(lo)) [lo, hi] = [-1, 1];
    if (zero) [lo, hi] = [Math.min(lo, 0), Math.max(hi, 0)];
    if (hi - lo < 1e-9) [lo, hi] = [lo - 1, hi + 1];
    const pad = (hi - lo) * 0.08;
    return { lo: lo - pad, hi: hi + pad };
  }, [series, zero]);
  const px = (x: number) => PAD.left + ((x - x0) / (x1 - x0 || 1)) * (W - PAD.left - PAD.right);
  const py = (y: number) => PAD.top + (1 - (y - lo) / (hi - lo)) * (H - PAD.top - PAD.bottom);
  const toX = (clientX: number) => {
    const r = ref.current!.getBoundingClientRect();
    const f = ((clientX - r.left) / r.width) * W;
    return Math.min(x1, Math.max(x0, x0 + ((f - PAD.left) / (W - PAD.left - PAD.right)) * (x1 - x0)));
  };
  const line = (ys: readonly number[]) => {
    const pts: string[] = [];
    for (let i = 0; i < xs.length; i++) if (Number.isFinite(ys[i]!)) pts.push(`${px(xs[i]!).toFixed(1)},${py(ys[i]!).toFixed(1)}`);
    return pts.join(" ");
  };
  return (
    <figure className="chart" data-testid={testId}>
      <svg
        ref={ref}
        viewBox={`0 0 ${W} ${H}`}
        role="img"
        aria-labelledby={titleId}
        onPointerDown={(e) => {
          e.currentTarget.setPointerCapture(e.pointerId);
          onSeek(toX(e.clientX));
        }}
        onPointerMove={(e) => e.currentTarget.hasPointerCapture(e.pointerId) && onSeek(toX(e.clientX))}
      >
        <title id={titleId}>{`${title} (${unit}) against ${xUnit}`}</title>
        {bands.map(([a, b], i) => (
          <rect key={i} x={px(a)} y={PAD.top} width={Math.max(0, px(b) - px(a))} height={H - PAD.top - PAD.bottom} className="chart-band" />
        ))}
        <text x={4} y={12} className="chart-title">
          {title} <tspan className="chart-unit">{unit}</tspan>
        </text>
        {series.length > 1 &&
          series.map((s, i) => (
            <text key={s.label} x={W - PAD.right - 70 * (series.length - 1 - i)} y={12} className="chart-unit" textAnchor="end" fill={s.colour}>
              {s.label}
            </text>
          ))}
        <text x={PAD.left - 3} y={PAD.top + 4} className="chart-axis" textAnchor="end">
          {formatNumber(hi, 2)}
        </text>
        <text x={PAD.left - 3} y={H - PAD.bottom} className="chart-axis" textAnchor="end">
          {formatNumber(lo, 2)}
        </text>
        <text x={W - PAD.right} y={H - 3} className="chart-axis" textAnchor="end">
          {formatNumber(x1, 2)} {xUnit}
        </text>
        <rect x={PAD.left} y={PAD.top} width={W - PAD.left - PAD.right} height={H - PAD.top - PAD.bottom} className="chart-frame" />
        {lo < 0 && hi > 0 && <line x1={PAD.left} x2={W - PAD.right} y1={py(0)} y2={py(0)} className="chart-zero" />}
        {markers.map((m, i) => (
          <line key={i} x1={px(m.x)} x2={px(m.x)} y1={PAD.top} y2={H - PAD.bottom} className="chart-marker lit">
            <title>{m.label}</title>
          </line>
        ))}
        {series.map((s) => (
          <polyline key={s.label} points={line(s.ys)} fill="none" stroke={s.colour} strokeWidth={1.5} strokeDasharray={s.dashed ? "4 3" : undefined} />
        ))}
        {Number.isFinite(cursor) && cursor >= x0 && cursor <= x1 && <line x1={px(cursor)} x2={px(cursor)} y1={PAD.top} y2={H - PAD.bottom} className="chart-cursor" />}
      </svg>
    </figure>
  );
}

const col = (xs: readonly TrackSample[], f: (s: TrackSample) => number) => xs.map(f);

/** The finish phase's spans, in the chart's x. */
function terminalBands(samples: readonly TrackSample[], x: (s: TrackSample) => number): [number, number][] {
  const out: [number, number][] = [];
  let start: number | null = null;
  for (const s of samples) {
    const terminal = s.phase === "TERMINAL" || s.phase === "FINISHED";
    if (terminal && start === null) start = x(s);
    if (!terminal && start !== null) {
      out.push([start, x(s)]);
      start = null;
    }
  }
  if (start !== null && samples.length) out.push([start, x(samples.at(-1)!)]);
  return out;
}

export function TrackingCharts({ run }: { run: RecordedRun }) {
  const axis = useStore(telemetry, (s) => s.axis);
  const cursor = useStore(telemetry, (s) => s.cursor);
  useStore(telemetry, (s) => s.version); // the run grows in place while recording
  const samples = run.samples;
  const byS = axis === "s";
  const x = (s: TrackSample) => (byS ? s.s : s.t);
  const xs = col(samples, x);
  const sampleAtT = (t: number) => samples.find((p) => p.t >= t) ?? samples.at(-1);
  const cursorX = byS ? (sampleAtT(cursor)?.s ?? Number.NaN) : cursor;
  // seek by x: on the path axis, to the first loop that got that far
  const seek = (v: number) => setCursor(byS ? (samples.find((p) => p.s >= v)?.t ?? samples.at(-1)?.t ?? 0) : v);
  const bands = terminalBands(samples, x);
  const markers = run.events.map((e) => ({ x: byS ? (sampleAtT(e.t)?.s ?? Number.NaN) : e.t, label: `${e.name} at ${formatNumber(e.t, 2)} s` }));
  const common = { xs, bands, markers, cursor: cursorX, xUnit: byS ? "m" : "s", onSeek: seek };
  const deg = (r: number) => (r * 180) / Math.PI;
  return (
    <div className="tracking-charts" data-testid="tracking-charts">
      <Chart title="Cross-track" unit="cm" testId="chart-cross-track" zero series={[{ label: "cross-track", colour: "#f472b6", ys: col(samples, (s) => s.crossTrack * 100) }]} {...common} />
      <Chart title="Along-track" unit="cm (ahead of plan)" testId="chart-along-track" zero series={[{ label: "along-track", colour: "#fb923c", ys: col(samples, (s) => s.alongTrack * 100) }]} {...common} />
      <Chart title="Heading error" unit="°" testId="chart-heading" zero series={[{ label: "heading", colour: "#22d3ee", ys: col(samples, (s) => deg(s.headingError)) }]} {...common} />
      <Chart
        title="Speed"
        unit="m/s"
        testId="chart-speed"
        series={[
          { label: "planned", colour: "#94a3b8", ys: col(samples, (s) => s.plannedSpeed), dashed: true },
          { label: "actual", colour: "#4ca6ff", ys: col(samples, (s) => s.speed) },
        ]}
        {...common}
      />
      <Chart
        title="Turn rate ω"
        unit="rad/s"
        testId="chart-omega"
        zero
        series={[
          { label: "planned", colour: "#94a3b8", ys: col(samples, (s) => s.plannedOmega), dashed: true },
          { label: "actual", colour: "#e879f9", ys: col(samples, (s) => s.omega) },
        ]}
        {...common}
      />
    </div>
  );
}
