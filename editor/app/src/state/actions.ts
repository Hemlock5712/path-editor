// Actions bound to keys and toolbar buttons that aren't a single panel's.

import { deleteWaypoint, moveWaypoint, removeConstraintZone, removeEvent, removeFacePointZone, removeRotationTarget } from "../model/edits.ts";
import { moveNamedPoint, removeNamedPoint } from "../model/namedPoints.ts";
import { makeEditorPath, starterSpec, uniquePathName } from "../model/project.ts";
import { round5 } from "../model/positions.ts";
import { analysis, playback } from "./analysis.ts";
import { deletePath, insertPath } from "./commands.ts";
import { activePath, editor, editPath, editProject, execute, notify, setActivePath, setView } from "./editor.ts";

export function deleteSelection(): void {
  const s = editor.get();
  const sel = s.selection;
  if (!sel) return;
  switch (sel.kind) {
    case "waypoint":
      if (activePath(s).spec.waypoints.length <= 2) notify("A path needs at least two waypoints.", "error");
      else editPath("Delete waypoint", (p) => deleteWaypoint(p, sel.index), null);
      return;
    case "rotation":
      editPath("Delete rotation target", (p) => removeRotationTarget(p, sel.index), null);
      return;
    case "event":
      editPath("Delete event", (p) => removeEvent(p, sel.index), null);
      return;
    case "face":
      editPath("Delete zone", (p) => removeFacePointZone(p, sel.index), null);
      return;
    case "constraint":
      editPath("Delete zone", (p) => removeConstraintZone(p, sel.index), null);
      return;
    case "obstacle":
      editProject("Delete obstacle", (p) => ({ ...p, field: { ...p.field, obstacles: p.field.obstacles.filter((_, i) => i !== sel.index) } }), null);
      return;
    case "point":
      editProject("Delete point", (p) => removeNamedPoint(p, sel.id), null);
      return;
  }
}

/** Nudges the selected waypoint or point by (dx, dy) metres in the displayed frame. */
export function nudge(dx: number, dy: number): void {
  const s = editor.get();
  const sel = s.selection;
  // on the red view the screen's axes are the flipped field's
  const flipX = s.view.alliance === "red" ? -1 : 1;
  const flipY = s.view.alliance === "red" && s.project.settings.redFrame === "RED_HALF_TURN" ? -1 : 1;
  const [ddx, ddy] = [dx * flipX, dy * flipY];
  if (sel?.kind === "waypoint") {
    const path = activePath(s);
    const w = path.spec.waypoints[sel.index]!;
    const link = path.links[sel.index];
    const x = round5(w.xMeters + ddx);
    const y = round5(w.yMeters + ddy);
    if (link) editProject("Move point", (p) => moveNamedPoint(p, link, x, y));
    else editPath("Move waypoint", (p) => moveWaypoint(p, sel.index, x, y));
  } else if (sel?.kind === "point") {
    const point = s.project.namedPoints.find((p) => p.id === sel.id)!;
    editProject("Move point", (p) => moveNamedPoint(p, sel.id, round5(point.x + ddx), round5(point.y + ddy)));
  }
}

/**
 * A new path starting where the active one ends, facing the way the robot ends up (a swerve auto is a
 * chain of paths), or at a given pose.
 */
export function newPath(start?: { x: number; y: number; heading: number; pointId?: string }): void {
  const s = editor.get();
  const from = activePath(s).spec.waypoints.at(-1)!;
  const pose = start ?? { x: from.xMeters, y: from.yMeters, heading: Number.isNaN(from.headingRadians) ? 0 : from.headingRadians };
  let path = makeEditorPath(uniquePathName(s.project), starterSpec(pose.x, pose.y, pose.heading), s.project.paths.length);
  if (start?.pointId) path = { ...path, links: path.links.map((l, i) => (i === 0 ? start.pointId! : l)) };
  execute(insertPath("New path", path, s.project.paths.length));
  setActivePath(path.id);
}

export function duplicatePath(): void {
  const s = editor.get();
  const source = activePath(s);
  const index = s.project.paths.indexOf(source);
  const copy = { ...makeEditorPath(uniquePathName(s.project, `${source.name}Copy`), source.spec, s.project.paths.length, source.links) };
  execute(insertPath("Duplicate path", copy, index + 1));
  setActivePath(copy.id);
}

export function removeActivePath(): void {
  const s = editor.get();
  if (s.project.paths.length <= 1) {
    notify("A project keeps at least one path.", "error");
    return;
  }
  const path = activePath(s);
  if (!window.confirm(`Delete the path ${path.name}? (Undo brings it back; saving removes its file.)`)) return;
  const index = s.project.paths.indexOf(path);
  execute(deletePath("Delete path", path, index), null);
}

export function cyclePath(step: number): void {
  const s = editor.get();
  const i = s.project.paths.findIndex((p) => p.id === s.activePathId);
  const n = s.project.paths.length;
  setActivePath(s.project.paths[(i + step + n) % n]!.id);
}

export function togglePlay(): void {
  const result = analysis.get().result;
  if (!result || !result.ok) return;
  const p = playback.get();
  if (p.playing) playback.set({ playing: false });
  else playback.set({ playing: true, t: p.t >= result.totalTimeSeconds - 1e-6 ? 0 : p.t });
}

export function stepPlayback(dt: number): void {
  const result = analysis.get().result;
  if (!result || !result.ok) return;
  const t = Math.min(result.totalTimeSeconds, Math.max(0, playback.get().t + dt));
  playback.set({ t, playing: false });
}

export function toggleAlliance(): void {
  setView({ alliance: editor.get().view.alliance === "blue" ? "red" : "blue" });
}

export function fitView(): void {
  setView({ zoom: 1, centerX: 0, centerY: 0 });
}
