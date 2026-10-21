// Draws the field view. Pure drawing from a model: no state, no events. Positions along the path
// come from the core (the main-thread geometry for the line itself, so it follows a drag at once,
// and the worker's analysis for speeds, headings and checks).

import { eventMeters, type PathGeometry, type RotationTarget } from "../../../core/src/index.ts";
import type { Analysis, Check, Stations } from "../compute/analysis.ts";
import type { BlockedAt } from "../compute/optimiser.ts";
import type { EditorPath, Project } from "../model/project.ts";
import type { Selection, Tool } from "../state/editor.ts";
import type { FieldView } from "./fieldView.ts";

/** The field image's field rectangle in image pixels (AdvantageScope's 2026 field, as on master). */
const IMAGE_FIELD = { x: 524, y: 95, w: 3378 - 524, h: 1489 - 95 };

/** A tangent handle is this long (metres) at tangentScale 1. */
export const TANGENT_HANDLE_METERS = 0.6;

export const COLOURS = {
  background: "#0b0f14",
  grid: "rgba(255,255,255,0.07)",
  axis: "rgba(255,255,255,0.22)",
  obstacle: "rgba(248,113,113,0.13)",
  obstacleEdge: "rgba(248,113,113,0.7)",
  collision: "#ef4444",
  waypoint: "#f8fafc",
  selected: "#fbbf24",
  linked: "#facc15",
  rotation: "#22d3ee",
  face: "#c084fc",
  constraint: "rgba(250,204,21,0.35)",
  event: "#34d399",
  eventLit: "#a7f3d0",
  error: "#ef4444",
  warning: "#f59e0b",
  robot: "rgba(56,189,248,0.25)",
  live: "#f472b6",
};

export interface LivePose {
  readonly x: number;
  readonly y: number;
  readonly heading: number;
}

/** What the robot reports, in real field coordinates (whichever alliance it is on). */
export interface LiveRobot {
  /** The measured pose, if a pose topic is set. */
  readonly pose?: LivePose;
  /** The follower's reference, from its telemetry. */
  readonly reference?: LivePose;
}

/** The follower's progress, drawn on the path when the editor shows the path the robot is running. */
export interface LiveProgress {
  readonly pathName: string;
  readonly progressMeters: number;
}

export interface DrawModel {
  readonly view: FieldView;
  readonly image: HTMLImageElement | null;
  readonly project: Project;
  readonly active: EditorPath;
  readonly geometry: PathGeometry | null;
  readonly analysis: Analysis | null;
  readonly selection: Selection | null;
  readonly tool: Tool;
  readonly playbackT: number;
  readonly showPlaybackRobot: boolean;
  readonly hoverS: number;
  readonly draftObstacle: readonly (readonly [number, number])[] | null;
  readonly live: LiveRobot | null;
  readonly liveProgress: LiveProgress | null;
  /** The main-thread geometry of any spec (the other tabs are drawn from it). */
  readonly geometryOf: (spec: EditorPath["spec"]) => PathGeometry | null;
  /** The optimiser's proposal for the active path, drawn dashed with its waypoints. */
  readonly ghost: EditorPath["spec"] | null;
  /** Where the optimiser couldn't clear an obstacle, and the waypoint it suggests there. */
  readonly blocked?: BlockedAt | null;
  /** The item under the pointer, highlighted. */
  readonly hover?: { readonly kind: "waypoint" | "turn" | "rotation"; readonly index: number } | null;
  /** A clock (ms) for the selection outline's slow march; 0 holds it still. */
  readonly time?: number;
}

/** Linear interpolation of a station column at s. */
export function atS(st: Stations, column: Float64Array, s: number): number {
  const n = st.s.length;
  if (n === 0) return Number.NaN;
  if (s <= st.s[0]!) return column[0]!;
  if (s >= st.s[n - 1]!) return column[n - 1]!;
  let lo = 0;
  let hi = n - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (st.s[mid]! <= s) lo = mid;
    else hi = mid;
  }
  const f = (s - st.s[lo]!) / (st.s[hi]! - st.s[lo]! || 1);
  return column[lo]! + f * (column[hi]! - column[lo]!);
}

/** Heading interpolated the short way round. */
export function headingAtS(st: Stations, s: number): number {
  const n = st.s.length;
  let lo = 0;
  while (lo < n - 2 && st.s[lo + 1]! <= s) lo++;
  const hi = Math.min(n - 1, lo + 1);
  const a = st.heading[lo]!;
  const b = st.heading[hi]!;
  const f = Math.max(0, Math.min(1, (s - st.s[lo]!) / (st.s[hi]! - st.s[lo]! || 1)));
  return a + f * Math.atan2(Math.sin(b - a), Math.cos(b - a));
}

/** Speed colour: cyan (slow) to orange (fast). */
export function speedColour(fraction: number): string {
  const k = Math.max(0, Math.min(1, fraction));
  return `hsl(${190 - 165 * k}, 90%, ${55 + 5 * k}%)`;
}

export function rotationTargetS(t: RotationTarget, geometry: PathGeometry): number {
  return Number.isNaN(t.waypoint) ? t.meters : geometry.sAtWaypoint(t.waypoint);
}

export function drawField(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const { view } = m;
  ctx.fillStyle = COLOURS.background;
  ctx.fillRect(0, 0, view.width, view.height);
  drawFieldImage(ctx, m);
  drawGrid(ctx, m);
  drawObstacles(ctx, m);
  for (const p of m.project.paths) if (p.id !== m.active.id) drawOtherPath(ctx, m, p);
  drawZones(ctx, m);
  drawChecks(ctx, m);
  drawPath(ctx, m);
  drawCollisions(ctx, m);
  drawGhost(ctx, m);
  drawBlocked(ctx, m);
  drawFootprints(ctx, m);
  drawRotationTargets(ctx, m);
  drawHeadingGhost(ctx, m);
  drawWaypoints(ctx, m);
  drawEvents(ctx, m);
  drawNamedPoints(ctx, m);
  drawDraftObstacle(ctx, m);
  drawHover(ctx, m);
  if (m.showPlaybackRobot) drawPlaybackRobot(ctx, m);
  drawLive(ctx, m);
  drawLegend(ctx, m);
}

/** What the field backdrop needs: the view, the image, and the field's size. */
export type FieldBackdrop = Pick<DrawModel, "view" | "image"> & { readonly project: Pick<Project, "field"> };

export function drawFieldImage(ctx: CanvasRenderingContext2D, m: FieldBackdrop): void {
  const { view } = m;
  const L = m.project.field.lengthMeters;
  const W = m.project.field.widthMeters;
  // The field itself never flips: corners in the displayed frame
  const [x0, y0] = view.toCanvasRaw(-L / 2, W / 2);
  const w = L * view.scale;
  const h = W * view.scale;
  if (m.image) {
    // The image has red on the left; turned half a turn, blue's wall is at -x as in WPILib
    ctx.save();
    ctx.translate(x0 + w / 2, y0 + h / 2);
    ctx.rotate(Math.PI);
    ctx.globalAlpha = 0.85;
    ctx.drawImage(m.image, IMAGE_FIELD.x, IMAGE_FIELD.y, IMAGE_FIELD.w, IMAGE_FIELD.h, -w / 2, -h / 2, w, h);
    ctx.restore();
  } else {
    ctx.fillStyle = "#1f2937";
    ctx.fillRect(x0, y0, w, h);
  }
  ctx.strokeStyle = "rgba(255,255,255,0.5)";
  ctx.lineWidth = 1;
  ctx.strokeRect(x0, y0, w, h);
  ctx.font = "600 11px system-ui, sans-serif";
  ctx.fillStyle = "#93c5fd";
  ctx.fillText("BLUE", x0 + 6, y0 - 6);
  ctx.fillStyle = "#fca5a5";
  ctx.textAlign = "right";
  ctx.fillText("RED", x0 + w - 6, y0 - 6);
  ctx.textAlign = "left";
}

export function drawGrid(ctx: CanvasRenderingContext2D, m: FieldBackdrop): void {
  const { view } = m;
  const L = m.project.field.lengthMeters / 2;
  const W = m.project.field.widthMeters / 2;
  const toPx = (x: number, y: number) => view.toCanvasRaw(x, y);
  ctx.lineWidth = 1;
  for (let x = Math.ceil(-L); x <= L; x++) {
    ctx.strokeStyle = x === 0 ? COLOURS.axis : COLOURS.grid;
    const [ax, ay] = toPx(x, -W);
    const [bx, by] = toPx(x, W);
    line(ctx, ax, ay, bx, by);
  }
  for (let y = Math.ceil(-W); y <= W; y++) {
    ctx.strokeStyle = y === 0 ? COLOURS.axis : COLOURS.grid;
    const [ax, ay] = toPx(-L, y);
    const [bx, by] = toPx(L, y);
    line(ctx, ax, ay, bx, by);
  }
  const [ox, oy] = toPx(0, 0);
  ctx.fillStyle = COLOURS.axis;
  ctx.font = "10px system-ui, sans-serif";
  ctx.fillText("(0, 0)", ox + 4, oy - 4);
}

function line(ctx: CanvasRenderingContext2D, ax: number, ay: number, bx: number, by: number): void {
  ctx.beginPath();
  ctx.moveTo(ax, ay);
  ctx.lineTo(bx, by);
  ctx.stroke();
}

function drawObstacles(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  // Obstacles are fixed field features: drawn in the displayed frame without flipping
  const { view } = m;
  m.project.field.obstacles.forEach((o, i) => {
    if (o.polygon.length === 0) return;
    const selected = m.selection?.kind === "obstacle" && m.selection.index === i;
    ctx.beginPath();
    o.polygon.forEach(([x, y], j) => {
      const [px, py] = view.toCanvasRaw(x, y);
      if (j === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.closePath();
    ctx.fillStyle = COLOURS.obstacle;
    ctx.fill();
    ctx.strokeStyle = selected ? COLOURS.selected : COLOURS.obstacleEdge;
    ctx.lineWidth = selected ? 2 : 1;
    ctx.stroke();
    if (selected || m.tool === "obstacle") {
      for (const [x, y] of o.polygon) {
        const [px, py] = view.toCanvasRaw(x, y);
        ctx.fillStyle = COLOURS.obstacleEdge;
        ctx.fillRect(px - 4, py - 4, 8, 8);
      }
    }
    const [cx, cy] = o.polygon.reduce(([sx, sy], [x, y]) => [sx + x / o.polygon.length, sy + y / o.polygon.length], [0, 0]);
    ctx.fillStyle = "rgba(254,202,202,0.75)";
    ctx.font = "10px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText(o.name, ...view.toCanvasRaw(cx, cy));
    ctx.textAlign = "left";
  });
}

/** Points along a geometry every step metres, as canvas pixels. */
function polyline(view: FieldView, g: PathGeometry, from: number, to: number, step: number): [number, number, number][] {
  const out: [number, number, number][] = [];
  const n = Math.max(2, Math.ceil((to - from) / step) + 1);
  for (let i = 0; i < n; i++) {
    const s = from + ((to - from) * i) / (n - 1);
    const p = g.sample(s);
    const [x, y] = view.toCanvas(p.xMeters, p.yMeters);
    out.push([x, y, s]);
  }
  return out;
}

function drawOtherPath(ctx: CanvasRenderingContext2D, m: DrawModel, p: EditorPath): void {
  const g = m.geometryOf(p.spec);
  if (!g) return;
  const pts = polyline(m.view, g, 0, g.lengthMeters, 0.05);
  ctx.strokeStyle = p.colour;
  ctx.globalAlpha = 0.35;
  ctx.lineWidth = 2;
  ctx.beginPath();
  pts.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)));
  ctx.stroke();
  ctx.globalAlpha = 1;
}

function drawGhost(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const g = m.ghost ? m.geometryOf(m.ghost) : null;
  if (!g || !m.ghost) return;
  const pts = polyline(m.view, g, 0, g.lengthMeters, 0.03);
  ctx.strokeStyle = "#e2e8f0";
  ctx.lineWidth = 2;
  ctx.setLineDash([7, 5]);
  ctx.beginPath();
  pts.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)));
  ctx.stroke();
  ctx.setLineDash([]);
  for (const w of m.ghost.waypoints) {
    const [x, y] = m.view.toCanvas(w.xMeters, w.yMeters);
    ctx.strokeStyle = "#e2e8f0";
    ctx.lineWidth = 1.5;
    ctx.strokeRect(x - 4, y - 4, 8, 8);
  }
}

/** The spot the optimiser couldn't clear (a ring) and the waypoint it suggests (a dashed + to add). */
function drawBlocked(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const b = m.blocked;
  if (!b) return;
  const [x, y] = m.view.toCanvas(b.x, b.y);
  const [sx, sy] = m.view.toCanvas(b.suggestX, b.suggestY);
  ctx.strokeStyle = COLOURS.collision;
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.arc(x, y, 16, 0, 2 * Math.PI);
  ctx.stroke();
  ctx.strokeStyle = "#e2e8f0";
  ctx.lineWidth = 1.5;
  ctx.setLineDash([4, 4]);
  line(ctx, x, y, sx, sy);
  ctx.beginPath();
  ctx.arc(sx, sy, 9, 0, 2 * Math.PI);
  ctx.stroke();
  ctx.setLineDash([]);
  line(ctx, sx - 5, sy, sx + 5, sy);
  line(ctx, sx, sy - 5, sx, sy + 5);
  ctx.fillStyle = "#e2e8f0";
  ctx.font = "11px system-ui, sans-serif";
  ctx.fillText("add a waypoint here?", sx + 13, sy + 4);
}

function drawPath(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const g = m.geometry;
  if (!g) return;
  const pts = polyline(m.view, g, 0, g.lengthMeters, 0.02);
  const a = m.analysis;
  const vmax = a ? Math.max(0.1, ...a.stations.v) : 1;
  ctx.lineWidth = 4;
  ctx.lineCap = "round";
  for (let i = 1; i < pts.length; i++) {
    const [x0, y0, s0] = pts[i - 1]!;
    const [x1, y1] = pts[i]!;
    ctx.strokeStyle = a ? speedColour(atS(a.stations, a.stations.v, s0) / vmax) : m.active.colour;
    line(ctx, x0, y0, x1, y1);
  }
  ctx.lineCap = "butt";
}

/** Where the swept bumpers hit an obstacle or the wall: the path itself turns red there, with the bumpers at the middle. */
function drawCollisions(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const a = m.analysis;
  const g = m.geometry;
  if (!a || !g) return;
  for (const c of collisions(a)) {
    band(ctx, m, c.sFrom, c.sTo, COLOURS.collision, 5);
    const s = (c.sFrom + c.sTo) / 2;
    const p = g.sample(s);
    const corners = robotOutline(m.view, p.xMeters, p.yMeters, headingAtS(a.stations, s), m.project.robot.bumperLengthMeters, m.project.robot.bumperWidthMeters);
    ctx.strokeStyle = COLOURS.collision;
    ctx.lineWidth = 2;
    ctx.beginPath();
    corners.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)));
    ctx.closePath();
    ctx.stroke();
  }
}

/** The stretches where the swept bumpers touch an obstacle or the field wall. */
export function collisions(a: Analysis): Check[] {
  return a.checks.filter((c) => c.kind === "bumper" && c.severity === "error");
}

/** Draws a stretch of the active path as a wide translucent band. */
function band(ctx: CanvasRenderingContext2D, m: DrawModel, from: number, to: number, colour: string, width: number): void {
  const g = m.geometry;
  if (!g || !(to > from)) return;
  const pts = polyline(m.view, g, Math.max(0, from), Math.min(g.lengthMeters, to), 0.03);
  ctx.strokeStyle = colour;
  ctx.lineWidth = width;
  ctx.lineCap = "round";
  ctx.beginPath();
  pts.forEach(([x, y], i) => (i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)));
  ctx.stroke();
  ctx.lineCap = "butt";
}

function drawZones(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const g = m.geometry;
  if (!g) return;
  const spec = m.active.spec;
  spec.constraintZones.forEach((z, i) => {
    const sel = m.selection?.kind === "constraint" && m.selection.index === i;
    band(ctx, m, g.sAtWaypoint(z.fromWaypoint), g.sAtWaypoint(z.toWaypoint), sel ? "rgba(250,204,21,0.6)" : COLOURS.constraint, 14);
  });
  spec.facePointZones.forEach((z, i) => {
    const sel = m.selection?.kind === "face" && m.selection.index === i;
    const from = g.sAtWaypoint(z.fromWaypoint);
    const to = g.sAtWaypoint(z.toWaypoint);
    band(ctx, m, from, to, sel ? "rgba(192,132,252,0.55)" : "rgba(192,132,252,0.28)", 10);
    const [px, py] = m.view.toCanvas(z.point.x, z.point.y);
    // sight lines from the zone's ends to the point
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = COLOURS.face;
    ctx.lineWidth = 1;
    for (const s of [from, to]) {
      const p = g.sample(s);
      const [x, y] = m.view.toCanvas(p.xMeters, p.yMeters);
      line(ctx, x, y, px, py);
    }
    ctx.setLineDash([]);
    ctx.lineWidth = sel ? 2.5 : 1.5;
    ctx.beginPath();
    ctx.arc(px, py, 7, 0, 2 * Math.PI);
    ctx.stroke();
    line(ctx, px - 10, py, px + 10, py);
    line(ctx, px, py - 10, px, py + 10);
    ctx.fillStyle = COLOURS.face;
    ctx.font = "10px system-ui, sans-serif";
    ctx.fillText(`face ${i + 1}`, px + 9, py - 9);
  });
}

function drawChecks(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const a = m.analysis;
  if (!a) return;
  for (const c of a.checks) {
    const colour = c.severity === "error" ? "rgba(239,68,68,0.55)" : "rgba(245,158,11,0.45)";
    if (c.sTo - c.sFrom < 0.02) {
      const g = m.geometry;
      if (!g) continue;
      const p = g.sample(c.sFrom);
      const [x, y] = m.view.toCanvas(p.xMeters, p.yMeters);
      ctx.strokeStyle = colour;
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.arc(x, y, 12, 0, 2 * Math.PI);
      ctx.stroke();
    } else {
      band(ctx, m, c.sFrom, c.sTo, colour, 20);
    }
  }
}

/** The bumper rectangle at a blue-frame pose, in canvas pixels. */
export function robotOutline(view: FieldView, x: number, y: number, heading: number, length: number, width: number): [number, number][] {
  const c = Math.cos(heading);
  const s = Math.sin(heading);
  return [
    [length / 2, width / 2],
    [-length / 2, width / 2],
    [-length / 2, -width / 2],
    [length / 2, -width / 2],
  ].map(([u, v]) => view.toCanvas(x + u! * c - v! * s, y + u! * s + v! * c));
}

function drawRobot(
  ctx: CanvasRenderingContext2D,
  m: DrawModel,
  x: number,
  y: number,
  heading: number,
  stroke: string,
  fill: string | null,
): void {
  const { bumperLengthMeters: L, bumperWidthMeters: W } = m.project.robot;
  const pts = robotOutline(m.view, x, y, heading, L, W);
  ctx.beginPath();
  pts.forEach(([px, py], i) => (i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py)));
  ctx.closePath();
  if (fill) {
    ctx.fillStyle = fill;
    ctx.fill();
  }
  ctx.strokeStyle = stroke;
  ctx.stroke();
  // front marker: a line from the centre to the middle of the front bumper
  const [cx, cy] = m.view.toCanvas(x, y);
  const [fx, fy] = m.view.toCanvas(x + (L / 2) * Math.cos(heading), y + (L / 2) * Math.sin(heading));
  line(ctx, cx, cy, fx, fy);
}

function drawFootprints(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const a = m.analysis;
  const every = m.view.view.footprintSpacing;
  if (!a || !(every > 0)) return;
  ctx.lineWidth = 1;
  for (let s = 0; s <= a.lengthMeters + 1e-9; s += every) {
    drawRobot(ctx, m, atS(a.stations, a.stations.x, s), atS(a.stations, a.stations.y, s), headingAtS(a.stations, s), "rgba(255,255,255,0.22)", null);
  }
  const end = a.lengthMeters;
  drawRobot(ctx, m, atS(a.stations, a.stations.x, end), atS(a.stations, a.stations.y, end), headingAtS(a.stations, end), "rgba(255,255,255,0.4)", null);
}

/** Where a rotation target's heading handle is (blue frame). */
/** How far out from the robot's centre a rotate handle sits, metres. */
export function headingReach(m: Pick<DrawModel, "project">): number {
  return Math.max(m.project.robot.bumperLengthMeters, m.project.robot.bumperWidthMeters) / 2 + 0.25;
}

/** A rotate handle for a pose: [centre x, centre y, handle x, handle y] (blue frame). */
function handleFor(m: Pick<DrawModel, "project">, x: number, y: number, heading: number): [number, number, number, number] {
  const reach = headingReach(m);
  return [x, y, x + reach * Math.cos(heading), y + reach * Math.sin(heading)];
}

export function rotationHandle(m: Pick<DrawModel, "project" | "geometry">, t: RotationTarget): [number, number, number, number] | null {
  const g = m.geometry;
  if (!g) return null;
  const p = g.sample(rotationTargetS(t, g));
  return handleFor(m, p.xMeters, p.yMeters, t.headingRadians);
}

/**
 * The robot's pose at waypoint i: its set heading, or for a pass-through point the plan's heading there
 * (from the analysis; 0 until there is one).
 */
export function waypointPose(m: Pick<DrawModel, "active" | "analysis">, i: number): { x: number; y: number; heading: number; set: boolean } {
  const w = m.active.spec.waypoints[i]!;
  if (!Number.isNaN(w.headingRadians)) return { x: w.xMeters, y: w.yMeters, heading: w.headingRadians, set: true };
  const a = m.analysis;
  const s = a?.waypointS[i];
  return { x: w.xMeters, y: w.yMeters, heading: a && s !== undefined ? headingAtS(a.stations, s) : 0, set: false };
}

/** Waypoint i's rotate handle: [centre x, centre y, handle x, handle y] (blue frame). */
export function waypointHandle(m: Pick<DrawModel, "active" | "analysis" | "project">, i: number): [number, number, number, number] {
  const p = waypointPose(m, i);
  return handleFor(m, p.x, p.y, p.heading);
}

/** True if a blue-frame point is inside waypoint i's bumper footprint. */
export function insideFootprint(m: Pick<DrawModel, "active" | "analysis" | "project">, i: number, x: number, y: number): boolean {
  const p = waypointPose(m, i);
  const dx = x - p.x;
  const dy = y - p.y;
  const u = dx * Math.cos(p.heading) + dy * Math.sin(p.heading);
  const v = -dx * Math.sin(p.heading) + dy * Math.cos(p.heading);
  return Math.abs(u) <= m.project.robot.bumperLengthMeters / 2 && Math.abs(v) <= m.project.robot.bumperWidthMeters / 2;
}

/** A rotate handle: the ring's arc around the robot and a knob on it, in the direction it faces. */
function drawRotateHandle(ctx: CanvasRenderingContext2D, m: DrawModel, h: [number, number, number, number], colour: string, filled: boolean, strong: boolean): void {
  const [cx, cy] = m.view.toCanvas(h[0], h[1]);
  const [hx, hy] = m.view.toCanvas(h[2], h[3]);
  const r = Math.hypot(hx - cx, hy - cy);
  const a = Math.atan2(hy - cy, hx - cx);
  ctx.strokeStyle = colour;
  ctx.globalAlpha = strong ? 0.9 : 0.45;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(cx, cy, r, a - 0.5, a + 0.5);
  ctx.stroke();
  ctx.globalAlpha = 1;
  line(ctx, cx, cy, hx, hy);
  ctx.beginPath();
  ctx.arc(hx, hy, strong ? 7 : 6, 0, 2 * Math.PI);
  ctx.fillStyle = filled ? colour : COLOURS.background;
  ctx.fill();
  ctx.lineWidth = 2;
  ctx.stroke();
}

function drawRotationTargets(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  m.active.spec.rotationTargets.forEach((t, i) => {
    const h = rotationHandle(m, t);
    if (!h) return;
    const sel = m.selection?.kind === "rotation" && m.selection.index === i;
    ctx.lineWidth = sel ? 3 : 2;
    drawRobot(ctx, m, h[0], h[1], t.headingRadians, sel ? COLOURS.selected : COLOURS.rotation, sel ? "rgba(251,191,36,0.18)" : "rgba(34,211,238,0.15)");
    drawRotateHandle(ctx, m, h, sel ? COLOURS.selected : COLOURS.rotation, true, sel);
  });
}

/** The Rotation tool's ghost: where a click would add a heading, at the plan's heading there. */
function drawHeadingGhost(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const a = m.analysis;
  const g = m.geometry;
  if (m.tool !== "rotation" || !a || !g || !Number.isFinite(m.hoverS)) return;
  const p = g.sample(m.hoverS);
  ctx.globalAlpha = 0.6;
  ctx.setLineDash([5, 4]);
  ctx.lineWidth = 2;
  drawRobot(ctx, m, p.xMeters, p.yMeters, headingAtS(a.stations, m.hoverS), COLOURS.rotation, "rgba(34,211,238,0.1)");
  ctx.setLineDash([]);
  ctx.globalAlpha = 1;
}

/** Where waypoint i's tangent handle is (blue frame), with its direction. */
export function tangentHandle(m: Pick<DrawModel, "active" | "geometry">, i: number): { x: number; y: number; angle: number; auto: boolean } | null {
  const w = m.active.spec.waypoints[i];
  if (!w) return null;
  const auto = Number.isNaN(w.tangentRadians);
  let angle = w.tangentRadians;
  if (auto) {
    if (!m.geometry || i >= m.geometry.spline.segmentCount + 1) return null;
    angle = m.geometry.spline.knotTangentRadians(i);
  }
  const reach = TANGENT_HANDLE_METERS * w.tangentScale;
  return { x: w.xMeters + reach * Math.cos(angle), y: w.yMeters + reach * Math.sin(angle), angle, auto };
}

/** A small padlock: this waypoint is pinned (the optimiser leaves it). */
function drawPin(ctx: CanvasRenderingContext2D, x: number, y: number): void {
  ctx.fillStyle = "#f8fafc";
  ctx.strokeStyle = "#f8fafc";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(x, y - 4, 3, Math.PI, 0);
  ctx.stroke();
  ctx.fillRect(x - 4, y - 4, 8, 6);
}

function drawWaypoints(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const spec = m.active.spec;
  const selected = m.selection?.kind === "waypoint" ? m.selection.index : -1;
  // every waypoint is a robot pose: its footprint, solid where its heading is set, dashed (at the
  // plan's heading) where the robot passes through
  spec.waypoints.forEach((_, i) => {
    const p = waypointPose(m, i);
    const sel = i === selected;
    const hovered = m.hover && (m.hover.kind === "waypoint" || m.hover.kind === "turn") && m.hover.index === i;
    if (hovered && !sel) {
      // a soft halo under the robot the pointer is on
      ctx.save();
      ctx.lineWidth = 8;
      ctx.globalAlpha = 0.25;
      drawRobot(ctx, m, p.x, p.y, p.heading, "#f8fafc", null);
      ctx.restore();
    }
    if (sel) {
      // the selection: an outline just outside the bumpers, marching slowly
      const { bumperLengthMeters: L, bumperWidthMeters: W } = m.project.robot;
      const pad = 6 / m.view.scale;
      const pts = robotOutline(m.view, p.x, p.y, p.heading, L + 2 * pad, W + 2 * pad);
      ctx.save();
      ctx.setLineDash([5, 4]);
      ctx.lineDashOffset = -((m.time ?? 0) / 60) % 9;
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = COLOURS.selected;
      ctx.beginPath();
      pts.forEach(([x, y], k) => (k === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y)));
      ctx.closePath();
      ctx.stroke();
      ctx.restore();
    }
    ctx.lineWidth = sel ? 3 : hovered ? 2.5 : 2;
    if (!p.set) ctx.setLineDash([6, 4]);
    const colour = sel ? COLOURS.selected : p.set ? COLOURS.waypoint : "rgba(248,250,252,0.55)";
    drawRobot(ctx, m, p.x, p.y, p.heading, colour, sel ? "rgba(251,191,36,0.15)" : p.set ? "rgba(248,250,252,0.08)" : null);
    ctx.setLineDash([]);
    drawRotateHandle(ctx, m, waypointHandle(m, i), sel ? COLOURS.selected : p.set ? COLOURS.waypoint : "rgba(248,250,252,0.6)", p.set, sel);
  });
  if (selected >= 0 && m.view.view.shapeCurve) {
    const h = tangentHandle(m, selected);
    const w = spec.waypoints[selected]!;
    if (h) {
      const [ax, ay] = m.view.toCanvas(w.xMeters, w.yMeters);
      const [bx, by] = m.view.toCanvas(h.x, h.y);
      ctx.strokeStyle = "#a78bfa";
      ctx.lineWidth = 1.5;
      if (h.auto) ctx.setLineDash([4, 3]);
      line(ctx, ax, ay, bx, by);
      ctx.setLineDash([]);
      ctx.fillStyle = h.auto ? COLOURS.background : "#a78bfa";
      ctx.fillRect(bx - 5, by - 5, 10, 10);
      ctx.strokeRect(bx - 5, by - 5, 10, 10);
    }
  }
  spec.waypoints.forEach((w, i) => {
    const [x, y] = m.view.toCanvas(w.xMeters, w.yMeters);
    const linked = m.active.links[i] !== null;
    ctx.beginPath();
    ctx.arc(x, y, i === selected ? 7 : 5, 0, 2 * Math.PI);
    ctx.fillStyle = i === selected ? COLOURS.selected : linked ? COLOURS.linked : COLOURS.waypoint;
    ctx.fill();
    ctx.strokeStyle = "#0f172a";
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = "#e2e8f0";
    ctx.font = i === selected ? "600 12px system-ui, sans-serif" : "11px system-ui, sans-serif";
    ctx.fillText(String(i), x + 9, y + 13);
    if (m.active.pinned[i]) drawPin(ctx, x - 12, y + 12);
  });
}

function drawEvents(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const g = m.geometry;
  if (!g) return;
  const placed = m.analysis?.events;
  m.active.spec.events.forEach((e, i) => {
    const s = Math.max(0, Math.min(g.lengthMeters, eventMeters(e, g)));
    const p = g.sample(s);
    const [x, y] = m.view.toCanvas(p.xMeters, p.yMeters);
    const t = placed?.[i]?.t ?? Infinity;
    const lit = m.showPlaybackRobot && m.playbackT >= t;
    const sel = m.selection?.kind === "event" && m.selection.index === i;
    // a flag: pole and pennant
    ctx.strokeStyle = sel ? COLOURS.selected : COLOURS.event;
    ctx.lineWidth = 2;
    line(ctx, x, y, x, y - 22);
    ctx.beginPath();
    ctx.moveTo(x, y - 22);
    ctx.lineTo(x + 13, y - 17);
    ctx.lineTo(x, y - 12);
    ctx.closePath();
    ctx.fillStyle = lit ? COLOURS.eventLit : sel ? COLOURS.selected : COLOURS.event;
    ctx.fill();
    if (lit) {
      ctx.beginPath();
      ctx.arc(x, y, 9, 0, 2 * Math.PI);
      ctx.strokeStyle = COLOURS.eventLit;
      ctx.stroke();
    }
    ctx.font = lit ? "600 11px system-ui, sans-serif" : "11px system-ui, sans-serif";
    ctx.fillStyle = lit ? COLOURS.eventLit : "#d1fae5";
    ctx.fillText(e.name, x + 15, y - 14);
  });
}

function drawNamedPoints(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  for (const p of m.project.namedPoints) {
    const [x, y] = m.view.toCanvas(p.x, p.y);
    const sel = m.selection?.kind === "point" && m.selection.id === p.id;
    ctx.beginPath();
    ctx.moveTo(x, y - 9);
    ctx.lineTo(x + 9, y);
    ctx.lineTo(x, y + 9);
    ctx.lineTo(x - 9, y);
    ctx.closePath();
    ctx.strokeStyle = sel ? COLOURS.selected : COLOURS.linked;
    ctx.lineWidth = sel ? 2.5 : 1.5;
    ctx.stroke();
    ctx.fillStyle = COLOURS.linked;
    ctx.font = "10px system-ui, sans-serif";
    ctx.fillText(p.name, x + 11, y - 8);
  }
}

function drawDraftObstacle(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const d = m.draftObstacle;
  if (!d || d.length === 0) return;
  const px = (x: number, y: number) => m.view.toCanvasRaw(x, y);
  ctx.strokeStyle = COLOURS.obstacleEdge;
  ctx.setLineDash([5, 4]);
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  d.forEach(([x, y], i) => {
    const [a, b] = px(x, y);
    if (i === 0) ctx.moveTo(a, b);
    else ctx.lineTo(a, b);
  });
  ctx.stroke();
  ctx.setLineDash([]);
  for (const [x, y] of d) {
    const [a, b] = px(x, y);
    ctx.fillStyle = COLOURS.obstacleEdge;
    ctx.fillRect(a - 4, b - 4, 8, 8);
  }
}

function drawHover(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const g = m.geometry;
  if (!g || !Number.isFinite(m.hoverS)) return;
  const p = g.sample(m.hoverS);
  const [x, y] = m.view.toCanvas(p.xMeters, p.yMeters);
  ctx.strokeStyle = "#f8fafc";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(x, y, 6, 0, 2 * Math.PI);
  ctx.stroke();
}

/** The playback sample at time t, interpolated between the worker's time samples. */
export function poseAtTime(a: Analysis, t: number): { x: number; y: number; heading: number; s: number; v: number } {
  const bt = a.byTime;
  const n = bt.s.length;
  const f = Math.max(0, Math.min(n - 1, t / bt.dt));
  const i = Math.min(n - 2, Math.floor(f));
  const k = f - i;
  const lerp = (c: Float64Array) => c[i]! + k * (c[i + 1]! - c[i]!);
  const h0 = bt.heading[i]!;
  const h1 = bt.heading[i + 1]!;
  return {
    x: lerp(bt.x),
    y: lerp(bt.y),
    heading: h0 + k * Math.atan2(Math.sin(h1 - h0), Math.cos(h1 - h0)),
    s: lerp(bt.s),
    v: lerp(bt.v),
  };
}

function drawPlaybackRobot(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const a = m.analysis;
  if (!a) return;
  const p = poseAtTime(a, m.playbackT);
  ctx.lineWidth = 2;
  drawRobot(ctx, m, p.x, p.y, p.heading, "#38bdf8", COLOURS.robot);
}

function drawLive(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const live = m.live;
  if (!live) return;
  // The robot reports real field coordinates, which is what the view shows on either alliance
  const draw = (pose: LivePose, colour: string, fill: string | null) => {
    const [bx, by, heading] = m.view.realToBlue(pose.x, pose.y, pose.heading);
    ctx.lineWidth = 2;
    drawRobot(ctx, m, bx, by, heading, colour, fill);
  };
  if (live.reference) draw(live.reference, "rgba(244,114,182,0.6)", null);
  if (live.pose) draw(live.pose, COLOURS.live, "rgba(244,114,182,0.2)");
  const progress = m.liveProgress;
  if (progress && progress.pathName === m.active.name && m.geometry && Number.isFinite(progress.progressMeters)) {
    const p = m.geometry.sample(progress.progressMeters);
    const [x, y] = m.view.toCanvas(p.xMeters, p.yMeters);
    ctx.strokeStyle = COLOURS.live;
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(x, y, 10, 0, 2 * Math.PI);
    ctx.stroke();
  }
}

function drawLegend(ctx: CanvasRenderingContext2D, m: DrawModel): void {
  const a = m.analysis;
  if (!a) return;
  const vmax = Math.max(...a.stations.v);
  // under the tool bar, clear of the playback bar along the bottom
  const x = 16;
  const y = 74;
  const w = 120;
  for (let i = 0; i < w; i++) {
    ctx.fillStyle = speedColour(i / (w - 1));
    ctx.fillRect(x + i, y, 1, 8);
  }
  ctx.fillStyle = "#cbd5e1";
  ctx.font = "10px system-ui, sans-serif";
  ctx.fillText("0", x, y - 3);
  ctx.textAlign = "right";
  ctx.fillText(`${vmax.toFixed(2)} m/s`, x + w, y - 3);
  ctx.textAlign = "left";
}
