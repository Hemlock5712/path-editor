// Plots of the plan: v(s), a(s), κ(s), θ(s) and ω(t), every value from the core via the worker.
// The playback cursor and event markers show on each; hovering a distance chart marks the field,
// and clicking any chart seeks the playback there.

import { useId, useMemo, useRef } from "react";
import type { Analysis } from "../compute/analysis.ts";
import { playback } from "../state/analysis.ts";
import { useStore } from "../state/store.ts";
import { formatNumber } from "./controls.tsx";
import { atS, poseAtTime } from "./draw.ts";
import { useAnalysis } from "./panels/common.tsx";

const W = 240;
const H = 120;
const PAD = { left: 34, right: 6, top: 16, bottom: 16 };

interface Marker {
  readonly x: number;
  readonly label: string;
  readonly lit: boolean;
}

interface ChartProps {
  readonly title: string;
  readonly unit: string;
  readonly xs: ArrayLike<number>;
  readonly ys: ArrayLike<number>;
  readonly xUnit: string;
  readonly cursor: number;
  readonly hover: number;
  readonly markers: readonly Marker[];
  readonly onHover: (x: number) => void;
  readonly onSeek: (x: number) => void;
  readonly colour: string;
}

function LineChart({ title, unit, xs, ys, xUnit, cursor, hover, markers, onHover, onSeek, colour }: ChartProps) {
  const ref = useRef<SVGSVGElement>(null);
  const titleId = useId();
  const n = xs.length;
  const x0 = n > 0 ? xs[0]! : 0;
  const x1 = n > 0 ? xs[n - 1]! : 1;
  const { lo, hi, points } = useMemo(() => {
    let lo = Infinity;
    let hi = -Infinity;
    for (let i = 0; i < n; i++) {
      lo = Math.min(lo, ys[i]!);
      hi = Math.max(hi, ys[i]!);
    }
    if (!Number.isFinite(lo)) [lo, hi] = [0, 1];
    if (hi - lo < 1e-9) [lo, hi] = [lo - 0.5, hi + 0.5];
    if (lo > 0 && lo < 0.25 * hi) lo = 0;
    const px = (x: number) => PAD.left + ((x - x0) / (x1 - x0 || 1)) * (W - PAD.left - PAD.right);
    const py = (y: number) => PAD.top + (1 - (y - lo) / (hi - lo)) * (H - PAD.top - PAD.bottom);
    const step = Math.max(1, Math.floor(n / 400));
    const pts: string[] = [];
    for (let i = 0; i < n; i += step) pts.push(`${px(xs[i]!).toFixed(1)},${py(ys[i]!).toFixed(1)}`);
    if (n > 0) pts.push(`${px(xs[n - 1]!).toFixed(1)},${py(ys[n - 1]!).toFixed(1)}`);
    return { lo, hi, points: pts.join(" ") };
  }, [xs, ys, n, x0, x1]);

  const px = (x: number) => PAD.left + ((x - x0) / (x1 - x0 || 1)) * (W - PAD.left - PAD.right);
  const toX = (clientX: number) => {
    const r = ref.current!.getBoundingClientRect();
    const f = ((clientX - r.left) / r.width) * W;
    return x0 + ((f - PAD.left) / (W - PAD.left - PAD.right)) * (x1 - x0);
  };
  const zeroY = lo < 0 && hi > 0 ? PAD.top + (hi / (hi - lo)) * (H - PAD.top - PAD.bottom) : null;
  const inRange = (x: number) => Number.isFinite(x) && x >= x0 && x <= x1;

  return (
    <figure className="chart">
      <svg
        ref={ref}
        viewBox={`0 0 ${W} ${H}`}
        role="img"
        aria-labelledby={titleId}
        onPointerMove={(e) => onHover(toX(e.clientX))}
        onPointerLeave={() => onHover(Number.NaN)}
        onPointerDown={(e) => onSeek(Math.min(x1, Math.max(x0, toX(e.clientX))))}
      >
        <title id={titleId}>{`${title} (${unit}) against ${xUnit}`}</title>
        <text x={4} y={11} className="chart-title">
          {title} <tspan className="chart-unit">{unit}</tspan>
        </text>
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
        {zeroY !== null && <line x1={PAD.left} x2={W - PAD.right} y1={zeroY} y2={zeroY} className="chart-zero" />}
        {markers.filter((m) => inRange(m.x)).map((m, i) => (
          <line key={i} x1={px(m.x)} x2={px(m.x)} y1={PAD.top} y2={H - PAD.bottom} className={m.lit ? "chart-marker lit" : "chart-marker"}>
            <title>{m.label}</title>
          </line>
        ))}
        <polyline points={points} fill="none" stroke={colour} strokeWidth={1.5} />
        {inRange(hover) && <line x1={px(hover)} x2={px(hover)} y1={PAD.top} y2={H - PAD.bottom} className="chart-hover" />}
        {inRange(cursor) && <line x1={px(cursor)} x2={px(cursor)} y1={PAD.top} y2={H - PAD.bottom} className="chart-cursor" />}
      </svg>
    </figure>
  );
}

export function Charts() {
  const a = useAnalysis();
  const t = useStore(playback, (p) => p.t);
  const hoverS = useStore(playback, (p) => p.hoverS);
  if (!a) return <div className="charts empty">The charts appear once the path can be planned.</div>;
  return <ChartRow a={a} t={t} hoverS={hoverS} />;
}

function ChartRow({ a, t, hoverS }: { a: Analysis; t: number; hoverS: number }) {
  const st = a.stations;
  const cursorS = poseAtTime(a, t).s;
  const sMarkers = a.events.map((e) => ({ x: e.s, label: `${e.name} at ${formatNumber(e.s, 2)} m`, lit: t >= e.t && t > 0 }));
  const tMarkers = a.events.map((e) => ({ x: e.t, label: `${e.name} at ${formatNumber(e.t, 2)} s`, lit: t >= e.t && t > 0 }));
  const headingDeg = useMemo(() => {
    // unwrap so the plot has no ±180° jumps
    const out = new Float64Array(st.heading.length);
    let prev = st.heading[0] ?? 0;
    let acc = prev;
    for (let i = 0; i < st.heading.length; i++) {
      const h = st.heading[i]!;
      acc += Math.atan2(Math.sin(h - prev), Math.cos(h - prev));
      prev = h;
      out[i] = (acc * 180) / Math.PI;
    }
    return out;
  }, [st.heading]);
  const times = useMemo(() => Float64Array.from(a.byTime.s, (_, i) => Math.min(a.totalTimeSeconds, i * a.byTime.dt)), [a]);
  const onHoverS = (s: number) => playback.set({ hoverS: s });
  const seekS = (s: number) => playback.set({ t: atS(st, st.t, s), playing: false });
  const seekT = (time: number) => playback.set({ t: time, playing: false });
  const common = { cursor: cursorS, hover: hoverS, markers: sMarkers, onHover: onHoverS, onSeek: seekS, xUnit: "m" };
  return (
    <div className="charts" data-testid="charts">
      <LineChart title="Speed" unit="m/s" xs={st.s} ys={st.v} colour="#38bdf8" {...common} />
      <LineChart title="Acceleration" unit="m/s²" xs={st.s} ys={st.a} colour="#a3e635" {...common} />
      <LineChart title="Curvature" unit="1/m" xs={st.s} ys={st.kappa} colour="#f97316" {...common} />
      <LineChart title="Heading" unit="°" xs={st.s} ys={headingDeg} colour="#22d3ee" {...common} />
      <LineChart
        title="Spin ω"
        unit="rad/s"
        xs={times}
        ys={a.byTime.omega}
        xUnit="s"
        colour="#e879f9"
        cursor={t}
        hover={Number.isFinite(hoverS) ? atS(st, st.t, hoverS) : Number.NaN}
        markers={tMarkers}
        onHover={() => undefined}
        onSeek={seekT}
      />
    </div>
  );
}
