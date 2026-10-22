// Small helpers that save clicks: turning the selection with Q and E, stepping through waypoints with
// Tab, facing the next waypoint or a target, copy and paste of waypoints and paths, and the pose that
// suggests itself when a waypoint is put down near a field element.

import { makeRotationTarget, readPath, writePath } from "../../../core/src/index.ts";
import { convexHull } from "../compute/polygons.ts";
import { addWaypointAfter, setHeading, updateRotationTarget } from "../model/edits.ts";
import { setPointHeading } from "../model/namedPoints.ts";
import { round5, round5Radians } from "../model/positions.ts";
import { makeEditorPath, uniquePathName, type EditorPath } from "../model/project.ts";
import { analysis } from "./analysis.ts";
import { insertPath } from "./commands.ts";
import { activePath, editor, editPath, editProject, execute, notify, select, setActivePath } from "./editor.ts";
import { headingAtS } from "../ui/draw.ts";

const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

/** The heading a waypoint has now: its own, or the plan's there. */
function plannedHeading(path: EditorPath, i: number): number {
  const w = path.spec.waypoints[i]!;
  if (!Number.isNaN(w.headingRadians)) return w.headingRadians;
  const a = analysis.get().result;
  return a && a.ok && a.waypointS[i] !== undefined ? headingAtS(a.stations, a.waypointS[i]!) : 0;
}

/** Q and E: turn the selected waypoint, heading along the path or named point by delta radians. */
export function rotateSelection(delta: number): boolean {
  const s = editor.get();
  const sel = s.selection;
  const path = activePath(s);
  if (sel?.kind === "waypoint") {
    const h = round5Radians(wrap(plannedHeading(path, sel.index) + delta));
    editPath("Turn waypoint", (p) => setHeading(p, sel.index, h));
    return true;
  }
  if (sel?.kind === "rotation") {
    const t = path.spec.rotationTargets[sel.index];
    if (!t) return false;
    editPath("Turn heading", (p) => updateRotationTarget(p, sel.index, makeRotationTarget(t.waypoint, t.meters, round5Radians(wrap(t.headingRadians + delta)))));
    return true;
  }
  if (sel?.kind === "point") {
    const p = s.project.namedPoints.find((q) => q.id === sel.id);
    if (!p) return false;
    const h = p.heading === undefined || Number.isNaN(p.heading) ? 0 : p.heading;
    editProject("Turn point", (pr) => setPointHeading(pr, sel.id, round5Radians(wrap(h + delta))));
    return true;
  }
  return false;
}

/**
 * Tab and Shift+Tab on the field: the next or previous waypoint. False past either end (the selection
 * clears, and the key moves the focus on as usual).
 */
export function cycleWaypoint(step: number): boolean {
  const s = editor.get();
  const n = activePath(s).spec.waypoints.length;
  const at = s.selection?.kind === "waypoint" ? s.selection.index : step > 0 ? -1 : n;
  const next = at + step;
  if (next < 0 || next >= n) {
    select(null);
    return false;
  }
  select({ kind: "waypoint", index: next });
  return true;
}

/** Turns waypoint i to face a field point. */
export function faceToward(i: number, x: number, y: number, label = "Face target"): void {
  const w = activePath().spec.waypoints[i];
  if (!w || Math.hypot(x - w.xMeters, y - w.yMeters) < 1e-6) return;
  editPath(label, (p) => setHeading(p, i, round5Radians(Math.atan2(y - w.yMeters, x - w.xMeters))));
}

/** Turns waypoint i to face the next waypoint (the last one: the way it arrives). */
export function faceNext(i: number): void {
  const ws = activePath().spec.waypoints;
  const w = ws[i];
  if (!w || ws.length < 2) return;
  if (i < ws.length - 1) faceToward(i, ws[i + 1]!.xMeters, ws[i + 1]!.yMeters, "Face the next waypoint");
  else {
    const prev = ws[i - 1]!;
    faceToward(i, 2 * w.xMeters - prev.xMeters, 2 * w.yMeters - prev.yMeters, "Face the way it arrives");
  }
}

/** Things worth facing: named points and the field elements' centres. */
export function faceTargets(): { name: string; x: number; y: number }[] {
  const s = editor.get();
  const out = s.project.namedPoints.map((p) => ({ name: p.name, x: p.x, y: p.y }));
  for (const o of s.project.field.obstacles) {
    if (o.polygon.length === 0) continue;
    out.push({
      name: o.name,
      x: round5(o.polygon.reduce((t, q) => t + q[0], 0) / o.polygon.length),
      y: round5(o.polygon.reduce((t, q) => t + q[1], 0) / o.polygon.length),
    });
  }
  return out;
}

// ------------------------------------------------------------------------------ copy and paste

type Clip = { readonly kind: "waypoint"; readonly x: number; readonly y: number; readonly heading: number } | { readonly kind: "path"; readonly path: EditorPath };
let clip: Clip | null = null;

/** Ctrl+C: the selected waypoint, or with nothing selected the active path (also as a path file, as text). */
export function copySelection(): void {
  const s = editor.get();
  const path = activePath(s);
  const sel = s.selection;
  if (sel?.kind === "waypoint") {
    const w = path.spec.waypoints[sel.index]!;
    clip = { kind: "waypoint", x: w.xMeters, y: w.yMeters, heading: w.headingRadians };
    notify(`Copied waypoint ${sel.index}. Select a waypoint and press Ctrl+V to put a copy after it.`);
    return;
  }
  if (path.spec.waypoints.length < 2) return;
  clip = { kind: "path", path };
  void navigator.clipboard?.writeText(writePath(path.spec)).catch(() => undefined);
  notify(`Copied ${path.name}. Ctrl+V pastes it as a new path (its file is on the clipboard too).`);
}

/** Ctrl+V: a copied waypoint goes in after the selected one (30 cm on); a path becomes a new path. */
export async function paste(): Promise<void> {
  const s = editor.get();
  let c = clip;
  if (!c) {
    // a path file copied from elsewhere
    const text = await navigator.clipboard?.readText().catch(() => "");
    try {
      if (text) c = { kind: "path", path: makeEditorPath("Pasted", readPath(text), s.project.paths.length) };
    } catch {
      c = null;
    }
  }
  if (!c) return;
  if (c.kind === "waypoint") {
    const sel = s.selection;
    if (sel?.kind !== "waypoint") {
      notify("Select a waypoint to paste the copy after it.");
      return;
    }
    const at = sel.index + 1;
    const { x, y, heading } = c;
    const n = activePath(s).spec.waypoints.length;
    editPath(
      "Paste waypoint",
      (p) => {
        const added = addWaypointAfter(p, sel.index, round5(x + 0.3), round5(y));
        // it brings its heading (a new end keeps the one it took over if the copy had none)
        return Number.isNaN(heading) ? added : setHeading(added, at, heading);
      },
      { kind: "waypoint", index: Math.min(at, n) },
    );
    return;
  }
  const source = c.path;
  const copy = makeEditorPath(uniquePathName(s.project, `${source.name}Copy`), source.spec, s.project.paths.length, source.links, source.pinned);
  execute(insertPath("Paste path", copy, s.project.paths.length));
  setActivePath(copy.id);
}

// ------------------------------------------------------------------------------ poses by field elements

/** How far in front of an element a suggested pose's bumpers stand, metres. */
export const STANDOFF_METERS = 0.45;

/**
 * A pose a waypoint dropped near a field element probably wants: square to the element's nearest face,
 * the bumpers STANDOFF_METERS from it, facing the element's centre. Null when nothing is close.
 */
export function poseNearElement(x: number, y: number, within = 0.75): { name: string; x: number; y: number; heading: number } | null {
  const s = editor.get();
  const half = s.project.robot.bumperLengthMeters / 2;
  let best: { name: string; x: number; y: number; heading: number; d: number } | null = null;
  for (const o of s.project.field.obstacles) {
    const hull = convexHull(o.polygon);
    if (hull.length < 3) continue;
    const cx = hull.reduce((t, q) => t + q[0], 0) / hull.length;
    const cy = hull.reduce((t, q) => t + q[1], 0) / hull.length;
    for (let i = 0; i < hull.length; i++) {
      const [ax, ay] = hull[i]!;
      const [bx, by] = hull[(i + 1) % hull.length]!;
      const ex = bx - ax;
      const ey = by - ay;
      const len = Math.hypot(ex, ey);
      if (len < 1e-9) continue;
      const t = Math.max(0, Math.min(1, ((x - ax) * ex + (y - ay) * ey) / (len * len)));
      const px = ax + t * ex;
      const py = ay + t * ey;
      const d = Math.hypot(x - px, y - py);
      // the face's outward normal
      let nx = ey / len;
      let ny = -ex / len;
      if ((px - cx) * nx + (py - cy) * ny < 0) {
        nx = -nx;
        ny = -ny;
      }
      if (d <= within && (!best || d < best.d)) {
        const rx = px + nx * (half + STANDOFF_METERS);
        const ry = py + ny * (half + STANDOFF_METERS);
        best = { name: o.name, x: round5(rx), y: round5(ry), heading: round5Radians(Math.atan2(-ny, -nx)), d };
      }
    }
  }
  return best ? { name: best.name, x: best.x, y: best.y, heading: best.heading } : null;
}
