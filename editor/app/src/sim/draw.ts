// Draws the sim preview over the field: the simulated robot's trace, shaded by its cross-track error,
// where each event fired against its authored flag, the shove, the finish, and the simulated robot at the
// playback time (beside the plan's).

import { LOOP_SECONDS } from "../../../core/src/index.ts";
import { robotOutline } from "../ui/draw.ts";
import type { FieldView } from "../ui/fieldView.ts";
import { simPoseAt, type SimPreview } from "./preview.ts";

export const SIM_COLOURS = {
  trace: "#fb923c",
  robot: "#fb923c",
  fired: "#fdba74",
  shove: "#e879f9",
};

/** Cross-track error at which the trace is drawn fully red, metres. */
export const ERROR_SCALE_METERS = 0.1;

/** Orange on the path, shading to red as the cross-track error grows to ERROR_SCALE_METERS. */
export function errorColour(meters: number): string {
  const f = Math.min(1, Math.abs(meters) / ERROR_SCALE_METERS);
  const g = Math.round(146 * (1 - f) + 38 * f);
  const b = Math.round(60 * (1 - f) + 38 * f);
  return `rgb(${Math.round(251 * (1 - f) + 239 * f)},${g},${b})`;
}

export interface SimDrawModel {
  readonly view: FieldView;
  readonly preview: SimPreview;
  readonly bumperLengthMeters: number;
  readonly bumperWidthMeters: number;
  readonly playbackT: number;
  readonly showRobot: boolean;
}

export function drawSim(ctx: CanvasRenderingContext2D, m: SimDrawModel): void {
  const { view, preview: p } = m;
  ctx.save();
  ctx.lineCap = "round";
  ctx.lineWidth = 2.5;
  for (let i = 1; i < p.t.length; i++) {
    const [ax, ay] = view.toCanvas(p.x[i - 1]!, p.y[i - 1]!);
    const [bx, by] = view.toCanvas(p.x[i]!, p.y[i]!);
    ctx.strokeStyle = errorColour(p.crossTrack[i]!);
    ctx.beginPath();
    ctx.moveTo(ax, ay);
    ctx.lineTo(bx, by);
    ctx.stroke();
  }

  // the shove: an arrow from where the robot was hit, along the hit
  if (p.shove) {
    const i = Math.min(p.t.length - 1, Math.max(0, Math.ceil(p.shove.atSeconds / LOOP_SECONDS - 1e-9)));
    const len = Math.hypot(p.shove.dvx, p.shove.dvy);
    if (len > 0) {
      const [ax, ay] = view.toCanvas(p.x[i]!, p.y[i]!);
      const reach = 0.5 * Math.min(2, len);
      const [bx, by] = view.toCanvas(p.x[i]! + (reach * p.shove.dvx) / len, p.y[i]! + (reach * p.shove.dvy) / len);
      ctx.strokeStyle = SIM_COLOURS.shove;
      ctx.fillStyle = SIM_COLOURS.shove;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(bx, by);
      ctx.stroke();
      const angle = Math.atan2(by - ay, bx - ax);
      ctx.beginPath();
      ctx.moveTo(bx, by);
      ctx.lineTo(bx - 8 * Math.cos(angle - 0.45), by - 8 * Math.sin(angle - 0.45));
      ctx.lineTo(bx - 8 * Math.cos(angle + 0.45), by - 8 * Math.sin(angle + 0.45));
      ctx.closePath();
      ctx.fill();
    }
  }

  // each event: a diamond where the robot was when it fired, tied to its authored point
  ctx.font = "10px system-ui, sans-serif";
  for (const e of p.events) {
    if (e.loop < 0) continue;
    const [ax, ay] = view.toCanvas(e.x, e.y);
    const [fx, fy] = view.toCanvas(e.firedX, e.firedY);
    ctx.strokeStyle = SIM_COLOURS.fired;
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(ax, ay);
    ctx.lineTo(fx, fy);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = SIM_COLOURS.fired;
    ctx.beginPath();
    ctx.moveTo(fx, fy - 6);
    ctx.lineTo(fx + 6, fy);
    ctx.lineTo(fx, fy + 6);
    ctx.lineTo(fx - 6, fy);
    ctx.closePath();
    ctx.fill();
    ctx.fillText(`${e.name} ${(e.offsetMeters * 100).toFixed(1)} cm`, fx + 8, fy + 12);
  }

  // where it finished
  if (Number.isFinite(p.finishSeconds)) {
    const f = simPoseAt(p, p.finishSeconds);
    const [x, y] = view.toCanvas(f.x, f.y);
    ctx.strokeStyle = SIM_COLOURS.robot;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(x, y, 5, 0, 2 * Math.PI);
    ctx.stroke();
  }

  if (m.showRobot) {
    const r = simPoseAt(p, m.playbackT);
    const pts = robotOutline(view, r.x, r.y, r.heading, m.bumperLengthMeters, m.bumperWidthMeters);
    ctx.strokeStyle = SIM_COLOURS.robot;
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    pts.forEach(([px, py], i) => (i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py)));
    ctx.closePath();
    ctx.stroke();
    ctx.setLineDash([]);
    const [cx, cy] = view.toCanvas(r.x, r.y);
    const [fx, fy] = view.toCanvas(r.x + (m.bumperLengthMeters / 2) * Math.cos(r.heading), r.y + (m.bumperLengthMeters / 2) * Math.sin(r.heading));
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(fx, fy);
    ctx.stroke();
  }
  ctx.restore();
}
