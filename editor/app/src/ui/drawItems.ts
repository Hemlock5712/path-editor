// The handles on the field's own items: each zone's chip (a label beside its middle, the way to select
// it), the small × that deletes the item under the pointer or the selected one, and the guided flows'
// previews (the places picked so far, the zone being drawn, the heading being turned).

import type { FlowState } from "../state/flow.ts";
import { COLOURS, headingAtS, rotationTargetS, type DrawModel } from "./draw.ts";
import { eventMeters } from "../../../core/src/index.ts";

/** An item on the field that can be deleted from it. */
export type ItemRef = { readonly kind: "event" | "rotation" | "face" | "constraint"; readonly index: number } | { readonly kind: "point"; readonly id: string };

export type ItemModel = Pick<DrawModel, "view" | "active" | "geometry" | "project">;

const CHIP_FONT = "600 10px system-ui, sans-serif";
/** The delete ×'s radius, canvas px. */
export const DELETE_RADIUS = 8;

/** A zone's chip: its box in canvas px (centre, size) and text. */
export function zoneChip(m: ItemModel, kind: "constraint" | "face", i: number): { x: number; y: number; w: number; h: number; text: string } | null {
  const g = m.geometry;
  const spec = m.active.spec;
  const z = kind === "constraint" ? spec.constraintZones[i] : spec.facePointZones[i];
  if (!g || !z) return null;
  const s = (g.sAtWaypoint(z.fromWaypoint) + g.sAtWaypoint(z.toWaypoint)) / 2;
  const p = g.sample(s);
  const [x, y] = m.view.toCanvas(p.xMeters, p.yMeters);
  // beside the path, on its left as drawn
  const [ax, ay] = m.view.toCanvas(p.xMeters + p.tangentX, p.yMeters + p.tangentY);
  const len = Math.hypot(ax - x, ay - y) || 1;
  const nx = (ay - y) / len;
  const ny = -(ax - x) / len;
  let text: string;
  if (kind === "constraint") {
    const c = spec.constraintZones[i]!;
    text = Number.isFinite(c.maxMetersPerSecond) ? `≤ ${+c.maxMetersPerSecond.toFixed(2)} m/s` : `≤ ${+c.maxMetersPerSecondSquared.toFixed(2)} m/s²`;
  } else text = `faces ${i + 1}`;
  const w = text.length * 6 + 12;
  const h = 16;
  return { x: x + nx * 22, y: y + ny * 22, w, h, text };
}

/** Where an item's delete × goes, canvas px. */
export function deleteAnchor(m: ItemModel, ref: ItemRef): [number, number] | null {
  const g = m.geometry;
  const spec = m.active.spec;
  switch (ref.kind) {
    case "event": {
      const e = spec.events[ref.index];
      if (!g || !e) return null;
      const p = g.sample(Math.max(0, Math.min(g.lengthMeters, eventMeters(e, g))));
      const [x, y] = m.view.toCanvas(p.xMeters, p.yMeters);
      return [x - 10, y - 26];
    }
    case "rotation": {
      const t = spec.rotationTargets[ref.index];
      if (!g || !t) return null;
      const p = g.sample(rotationTargetS(t, g));
      const [x, y] = m.view.toCanvas(p.xMeters, p.yMeters);
      const r = (m.project.robot.bumperWidthMeters / 2) * m.view.scale;
      return [x - r - 4, y - r - 4];
    }
    case "constraint":
    case "face": {
      const c = zoneChip(m, ref.kind, ref.index);
      return c ? [c.x + c.w / 2 + DELETE_RADIUS + 2, c.y] : null;
    }
    case "point": {
      const p = m.project.namedPoints.find((q) => q.id === ref.id);
      if (!p) return null;
      const [x, y] = m.view.toCanvas(p.x, p.y);
      return [x - 14, y - 14];
    }
  }
}

export function drawZoneChips(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const spec = m.active.spec;
  const chip = (kind: "constraint" | "face", i: number) => {
    const c = zoneChip(m, kind, i);
    if (!c) return;
    const sel = m.selection?.kind === kind && m.selection.index === i;
    ctx.fillStyle = sel ? COLOURS.selected : kind === "constraint" ? "rgba(250,204,21,0.92)" : "rgba(192,132,252,0.92)";
    ctx.beginPath();
    ctx.roundRect(c.x - c.w / 2, c.y - c.h / 2, c.w, c.h, 8);
    ctx.fill();
    ctx.fillStyle = "#0b0f14";
    ctx.font = CHIP_FONT;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(c.text, c.x, c.y + 0.5);
    ctx.textAlign = "start";
    ctx.textBaseline = "alphabetic";
  };
  spec.constraintZones.forEach((_, i) => chip("constraint", i));
  spec.facePointZones.forEach((_, i) => chip("face", i));
}

export function drawDeleteHandle(ctx: CanvasRenderingContext2D, m: DrawModel, ref: ItemRef | null, hot: boolean): void {
  if (!ref) return;
  const a = deleteAnchor(m, ref);
  if (!a) return;
  const [x, y] = a;
  ctx.beginPath();
  ctx.arc(x, y, DELETE_RADIUS, 0, 2 * Math.PI);
  ctx.fillStyle = hot ? "#ef4444" : "rgba(17,22,29,0.92)";
  ctx.fill();
  ctx.strokeStyle = hot ? "#fecaca" : "#f87171";
  ctx.lineWidth = 1.5;
  ctx.stroke();
  const d = 3.2;
  ctx.strokeStyle = hot ? "#fff" : "#fca5a5";
  ctx.beginPath();
  ctx.moveTo(x - d, y - d);
  ctx.lineTo(x + d, y + d);
  ctx.moveTo(x + d, y - d);
  ctx.lineTo(x - d, y + d);
  ctx.stroke();
}

/** A guided flow's picks so far, the zone being drawn and the heading being turned. */
export function drawFlow(ctx: CanvasRenderingContext2D, m: DrawModel, f: FlowState, hover: { x: number; y: number } | null): void {
  const g = m.geometry;
  if (!f.kind) return;
  const colour = f.kind === "constraint" ? "rgba(250,204,21,0.9)" : f.kind === "face" ? COLOURS.face : f.kind === "event" ? COLOURS.event : COLOURS.rotation;
  const sAt = (wp: number) => (g ? g.sAtWaypoint(wp) : 0);
  // the zone so far: from the start to the pointer, or between the two picks
  if (g && (f.kind === "constraint" || f.kind === "face") && f.picks.length > 0) {
    const from = sAt(f.picks[0]!);
    const to = f.picks.length > 1 ? sAt(f.picks[1]!) : Number.isFinite(m.hoverS) ? m.hoverS : from;
    const [a, b] = from <= to ? [from, to] : [to, from];
    if (b > a) {
      ctx.strokeStyle = f.kind === "constraint" ? "rgba(250,204,21,0.55)" : "rgba(192,132,252,0.5)";
      ctx.lineWidth = 16;
      ctx.lineCap = "round";
      ctx.beginPath();
      for (let s = a; s <= b + 1e-9; s += Math.max(0.02, (b - a) / 100)) {
        const p = g.sample(Math.min(s, b));
        const [x, y] = m.view.toCanvas(p.xMeters, p.yMeters);
        if (s === a) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.lineCap = "butt";
    }
  }
  // each pick: a ring on the path
  if (g) {
    for (const wp of f.picks) {
      const p = g.sample(sAt(wp));
      const [x, y] = m.view.toCanvas(p.xMeters, p.yMeters);
      ctx.beginPath();
      ctx.arc(x, y, 7, 0, 2 * Math.PI);
      ctx.fillStyle = colour;
      ctx.fill();
      ctx.strokeStyle = "#0b0f14";
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  }
  // the face zone's target follows the pointer, with sight lines from the zone's ends
  if (f.kind === "face" && f.step === 2 && hover && g) {
    const [px, py] = m.view.toCanvas(hover.x, hover.y);
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = COLOURS.face;
    ctx.lineWidth = 1;
    for (const wp of f.picks) {
      const p = g.sample(sAt(wp));
      const [x, y] = m.view.toCanvas(p.xMeters, p.yMeters);
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(px, py);
      ctx.stroke();
    }
    ctx.setLineDash([]);
  }
  // the heading: a robot at the spot, turned to the flow's angle, with its knob
  if (f.kind === "rotation" && f.step === 1 && g) {
    const p = g.sample(sAt(f.picks[0]!));
    const h = flowKnob(m, f)!;
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 4]);
    const [cx, cy] = m.view.toCanvas(p.xMeters, p.yMeters);
    const corners = [
      [1, 1],
      [-1, 1],
      [-1, -1],
      [1, -1],
    ].map(([u, v]) => {
      const L = m.project.robot.bumperLengthMeters / 2;
      const W = m.project.robot.bumperWidthMeters / 2;
      const c = Math.cos(f.heading);
      const s = Math.sin(f.heading);
      return m.view.toCanvas(p.xMeters + u! * L * c - v! * W * s, p.yMeters + u! * L * s + v! * W * c);
    });
    ctx.strokeStyle = COLOURS.rotation;
    ctx.fillStyle = "rgba(34,211,238,0.12)";
    ctx.beginPath();
    corners.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)));
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.setLineDash([]);
    const [kx, ky] = m.view.toCanvas(h[0], h[1]);
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(kx, ky);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(kx, ky, 7, 0, 2 * Math.PI);
    ctx.fillStyle = COLOURS.rotation;
    ctx.fill();
  }
  // a named point's spot
  if (f.kind === "point" && f.point) {
    const [x, y] = m.view.toCanvas(f.point.x, f.point.y);
    ctx.beginPath();
    ctx.moveTo(x, y - 9);
    ctx.lineTo(x + 9, y);
    ctx.lineTo(x, y + 9);
    ctx.lineTo(x - 9, y);
    ctx.closePath();
    ctx.fillStyle = "rgba(250,204,21,0.35)";
    ctx.fill();
    ctx.strokeStyle = COLOURS.linked;
    ctx.lineWidth = 2;
    ctx.stroke();
  }
}

/** The heading flow's knob, field metres (blue frame), or null when there is none. */
export function flowKnob(m: Pick<DrawModel, "geometry" | "project">, f: FlowState): [number, number] | null {
  const g = m.geometry;
  if (f.kind !== "rotation" || f.step !== 1 || !g) return null;
  const p = g.sample(g.sAtWaypoint(f.picks[0]!));
  const reach = Math.hypot(m.project.robot.bumperLengthMeters, m.project.robot.bumperWidthMeters) / 2 + 0.2;
  return [p.xMeters + reach * Math.cos(f.heading), p.yMeters + reach * Math.sin(f.heading)];
}

/** The plan's heading at the hover point (where a heading flow's robot starts turned). */
export function planHeadingAt(m: Pick<DrawModel, "analysis">, s: number): number {
  return m.analysis ? headingAtS(m.analysis.stations, s) : 0;
}
