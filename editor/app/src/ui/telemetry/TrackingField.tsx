// The Telemetry page's field: the path the robot is following (the plan it published, or the open
// project's path of that name), where it should be (the reference, ghosted), where it is (solid), a line
// between them, the trail coloured by cross-track error, and the path's events lighting up as they fire.

import { useEffect, useRef } from "react";
import { geometryOf } from "../../state/analysis.ts";
import { DEFAULT_VIEW, editor } from "../../state/editor.ts";
import { selectedRun, telemetry } from "../../state/telemetry.ts";
import { sampleAt, type Pose, type RecordedRun } from "../../nt4/recorder.ts";
import { drawFieldImage, drawGrid } from "../draw.ts";
import { FieldView } from "../fieldView.ts";

/** Cross-track error to a colour: green on the line, amber at 5 cm, red from 10 cm. */
export function errorColour(meters: number): string {
  const e = Math.min(1, Math.abs(meters) / 0.1);
  const hue = 140 - 140 * e;
  return `hsl(${hue.toFixed(0)}, 85%, ${(52 - 6 * e).toFixed(0)}%)`;
}

/** The planned path's points: the plan the robot sent, or the open project's path of the same name. */
function plannedPoints(run: RecordedRun): { x: number; y: number }[] {
  if (run.plan) return run.plan.x.map((x, i) => ({ x, y: run.plan!.y[i]! }));
  const path = editor.get().project.paths.find((p) => p.name === run.path);
  const g = path ? geometryOf(path.spec) : null;
  if (!g) return [];
  const n = 200;
  return Array.from({ length: n + 1 }, (_, i) => {
    const p = g.sample((g.lengthMeters * i) / n);
    return { x: p.xMeters, y: p.yMeters };
  });
}

/** Where along the plan an event sits. */
function planPoint(run: RecordedRun, s: number): { x: number; y: number } | null {
  const plan = run.plan;
  if (!plan || plan.s.length === 0) return null;
  let i = 0;
  while (i < plan.s.length - 1 && plan.s[i + 1]! < s) i++;
  return { x: plan.x[i]!, y: plan.y[i]! };
}

function draw(ctx: CanvasRenderingContext2D, width: number, height: number, image: HTMLImageElement | null): void {
  const project = editor.get().project;
  const view = new FieldView(width, height, project.field.lengthMeters, project.field.widthMeters, DEFAULT_VIEW, "RED_HALF_TURN");
  ctx.fillStyle = "#0b0e13";
  ctx.fillRect(0, 0, width, height);
  drawFieldImage(ctx, { view, image, project });
  drawGrid(ctx, { view, image, project });
  const s = telemetry.get();
  const run = selectedRun(s);
  if (!run) return;
  const c = (x: number, y: number) => view.toCanvasRaw(x, y);

  // the plan
  const plan = plannedPoints(run);
  if (plan.length > 1) {
    ctx.strokeStyle = "rgba(255,255,255,0.4)";
    ctx.lineWidth = 6;
    ctx.lineJoin = "round";
    ctx.beginPath();
    plan.forEach((p, i) => (i === 0 ? ctx.moveTo(...c(p.x, p.y)) : ctx.lineTo(...c(p.x, p.y))));
    ctx.stroke();
  }

  // the trail up to the cursor, coloured by cross-track error
  ctx.lineWidth = 3;
  ctx.lineCap = "round";
  let prev: Pose | null = null;
  for (const sample of run.samples) {
    if (sample.t > s.cursor) break;
    if (sample.pose && prev) {
      ctx.strokeStyle = errorColour(sample.crossTrack);
      ctx.beginPath();
      ctx.moveTo(...c(prev.x, prev.y));
      ctx.lineTo(...c(sample.pose.x, sample.pose.y));
      ctx.stroke();
    }
    if (sample.pose) prev = sample.pose;
  }

  // events: a flag where each sits on the plan, lit once fired
  const firedBy = new Map(run.events.filter((e) => e.t <= s.cursor).map((e) => [e.name, e.t]));
  for (const e of run.plan?.events ?? []) {
    const p = planPoint(run, e.s);
    if (!p) continue;
    const [px, py] = c(p.x, p.y);
    const lit = firedBy.has(e.name);
    ctx.strokeStyle = lit ? "#34d399" : "rgba(255,255,255,0.5)";
    ctx.fillStyle = lit ? "#34d399" : "rgba(17,22,29,0.85)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(px, py - 22);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(px, py - 22);
    ctx.lineTo(px + 12, py - 18);
    ctx.lineTo(px, py - 14);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.font = "11px system-ui, sans-serif";
    ctx.fillStyle = lit ? "#a7f3d0" : "#cbd5e1";
    ctx.fillText(e.name, px + 14, py - 14);
  }

  // where it should be (ghost), where it is (solid), and the gap between them
  const at = sampleAt(run, s.cursor);
  if (!at) return;
  const { bumperLengthMeters: L, bumperWidthMeters: W } = project.robot;
  const robot = (p: Pose, stroke: string, fill: string, dashed: boolean) => {
    const pts = robotOutlineRaw(view, p, L, W);
    ctx.setLineDash(dashed ? [5, 4] : []);
    ctx.lineWidth = 2;
    ctx.strokeStyle = stroke;
    ctx.fillStyle = fill;
    ctx.beginPath();
    pts.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)));
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.setLineDash([]);
    const [cx, cy] = c(p.x, p.y);
    const [fx, fy] = c(p.x + (L / 2) * Math.cos(p.heading), p.y + (L / 2) * Math.sin(p.heading));
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(fx, fy);
    ctx.stroke();
  };
  if (at.reference) robot(at.reference, "rgba(255,255,255,0.8)", "rgba(255,255,255,0.08)", true);
  if (at.pose) robot(at.pose, "#ec4899", "rgba(236,72,153,0.22)", false);
  if (at.pose && at.reference) {
    ctx.strokeStyle = errorColour(Math.hypot(at.pose.x - at.reference.x, at.pose.y - at.reference.y));
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(...c(at.reference.x, at.reference.y));
    ctx.lineTo(...c(at.pose.x, at.pose.y));
    ctx.stroke();
  }
}

/** A robot's bumper outline at a pose in the real field frame (no alliance flip). */
function robotOutlineRaw(view: FieldView, p: Pose, L: number, W: number): [number, number][] {
  const cos = Math.cos(p.heading);
  const sin = Math.sin(p.heading);
  return [
    [L / 2, W / 2],
    [-L / 2, W / 2],
    [-L / 2, -W / 2],
    [L / 2, -W / 2],
  ].map(([u, v]) => view.toCanvasRaw(p.x + u! * cos - v! * sin, p.y + u! * sin + v! * cos));
}

export function TrackingField() {
  const wrap = useRef<HTMLDivElement>(null);
  const canvas = useRef<HTMLCanvasElement>(null);
  const image = useRef<HTMLImageElement | null>(null);
  useEffect(() => {
    let frame = 0;
    const redraw = () => {
      cancelAnimationFrame(frame);
      frame = requestAnimationFrame(() => {
        const c = canvas.current;
        const w = wrap.current;
        if (!c || !w) return;
        const rect = w.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        c.width = Math.max(1, Math.round(rect.width * dpr));
        c.height = Math.max(1, Math.round(rect.height * dpr));
        const ctx = c.getContext("2d")!;
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        draw(ctx, rect.width, rect.height, image.current);
      });
    };
    const img = new Image();
    img.onload = () => {
      image.current = img;
      redraw();
    };
    img.src = `${import.meta.env.BASE_URL}field-2026.png`;
    const observer = new ResizeObserver(redraw);
    observer.observe(wrap.current!);
    const unsubscribe = telemetry.subscribe(redraw);
    return () => {
      cancelAnimationFrame(frame);
      observer.disconnect();
      unsubscribe();
    };
  }, []);
  return (
    <div ref={wrap} className="field-wrap">
      <canvas
        ref={canvas}
        className="field-canvas"
        role="img"
        aria-label="The path the robot follows, where it should be (ghost) and where it is (pink), and its trail coloured by error"
        data-testid="tracking-field"
      />
    </div>
  );
}
