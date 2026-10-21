// The field: draws the project and turns pointer input into edits. Every edit goes through the
// editor store (one undo step per click or drag); this component only works out what was hit.
//
// Swerve-first: a waypoint is a robot pose. Its footprint moves it, the knob on its ring turns it
// (15° steps; Alt turns freely), and a pass-through point shows the plan's heading, dashed. Travel
// direction is automatic; the tangent handle only shows with "shape the curve" on.

import { QuickStartOverlay } from "./QuickStart.tsx";
import { Compass, CornerDownRight, Lock, LockOpen, Maximize2, Navigation, Plus, Repeat, Trash2 } from "lucide-react";
import { useEffect, useRef } from "react";
import { eventMeters, makePathEvent, makeRotationTarget, type PathGeometry } from "../../../core/src/index.ts";
import {
  addEvent,
  addRotationTarget,
  appendWaypoint,
  deleteWaypoint,
  insertWaypoint,
  moveWaypoint,
  setHeading,
  setPinned,
  setTangent,
  updateEvent,
  updateFacePointZone,
  updateRotationTarget,
} from "../model/edits.ts";
import { linkToPoint, moveNamedPoint } from "../model/namedPoints.ts";
import { eventNames, type EditorPath, type Project } from "../model/project.ts";
import { round5, round5Radians, waypointAtS } from "../model/positions.ts";
import { analysis, geometryOf, playback } from "../state/analysis.ts";
import {
  activePath,
  beginGesture,
  editor,
  editPath,
  editProject,
  endGesture,
  notify,
  previewProject,
  select,
  setDraftObstacle,
  setTool,
  setView,
  type EditorState,
  type Selection,
} from "../state/editor.ts";
import { live } from "../state/live.ts";
import { optimiser } from "../state/optimiser.ts";
import { useStore } from "../state/store.ts";
import { openContextMenu, ui, type MenuEntry } from "../state/ui.ts";
import { fitView, newPath, toggleAlliance } from "../state/actions.ts";
import {
  drawField,
  headingAtS,
  insideFootprint,
  rotationHandle,
  rotationTargetS,
  tangentHandle,
  TANGENT_HANDLE_METERS,
  waypointHandle,
  waypointPose,
} from "./draw.ts";
import { FieldView } from "./fieldView.ts";

type Drag =
  | { kind: "pan"; startX: number; startY: number; cx0: number; cy0: number; moved: boolean; button: number; vx: number; vy: number; lastX: number; lastY: number; lastT: number }
  | { kind: "waypoint"; index: number; label: string }
  | { kind: "turn"; index: number }
  | { kind: "tangent"; index: number }
  | { kind: "rotation"; index: number }
  | { kind: "slide"; index: number }
  | { kind: "insert"; s: number; startX: number; startY: number; started: boolean }
  | { kind: "face"; index: number }
  | { kind: "point"; id: string }
  | { kind: "event"; index: number }
  | { kind: "vertex"; obstacle: number; vertex: number }
  | { kind: "obstacle"; index: number; startX: number; startY: number };

type Hit =
  | { kind: "waypoint"; index: number }
  | { kind: "turn"; index: number }
  | { kind: "tangent"; index: number }
  | { kind: "rotation"; index: number }
  | { kind: "slide"; index: number }
  | { kind: "face"; index: number }
  | { kind: "point"; id: string }
  | { kind: "event"; index: number }
  | { kind: "vertex"; obstacle: number; vertex: number }
  | { kind: "obstacle"; index: number };

const HIT_PX = 10;
/** A click this close to the path lands on it (inserting a waypoint, placing an event). */
const PATH_SNAP_PX = 14;
const EVENT_SNAP_PX = 40;
/** A dragged or new waypoint this close to a named point snaps onto it and follows it, metres. */
const MAGNET_METERS = 0.3;
/** Grid snap spacing, metres. */
const GRID_METERS = 0.25;
/** Headings snap to this unless Alt is held. */
const HEADING_STEP = (15 * Math.PI) / 180;
/** Dragging from the path inserts a point once the pointer has moved this far, pixels. */
const DRAG_START_PX = 4;

/** What each tool does, shown on the field. */
export const TOOL_HINTS: Record<EditorState["tool"], string> = {
  select: "Drag a robot to move it, the knob on its ring to turn it (Alt: freely). Drag the path to insert a point. Right-click a point for more.",
  waypoint: "Click to add a point at the end; click the path to insert one. Enter or Esc when done.",
  rotation: "Click the path to add a heading there. Drag a heading's robot along the path, or its knob to turn it.",
  event: "Click the path to place an event.",
  obstacle: "Click the corners of an obstacle; double-click or Enter to finish.",
};

function makeView(s: EditorState, width: number, height: number): FieldView {
  return new FieldView(width, height, s.project.field.lengthMeters, s.project.field.widthMeters, s.view, s.project.settings.redFrame);
}

/** Point in convex-or-not polygon (even-odd). */
function insidePolygon(x: number, y: number, poly: readonly (readonly [number, number])[]): boolean {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [xi, yi] = poly[i]!;
    const [xj, yj] = poly[j]!;
    if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

/** Metres: Shift snaps to 5 cm, the grid (when on) to 25 cm, otherwise what the file holds. */
function snapMeters(v: number, shift: boolean, grid: boolean): number {
  if (shift) return Math.round(v * 20) / 20;
  if (grid) return round5(Math.round(v / GRID_METERS) * GRID_METERS);
  return round5(v);
}

/** A heading in 15° steps, or free (to what the file holds) with Alt. */
function snapHeading(radians: number, alt: boolean): number {
  return alt ? round5Radians(radians) : round5Radians(Math.round(radians / HEADING_STEP) * HEADING_STEP);
}

/** The named point within the magnet radius of (x, y), if any. */
function magnet(project: Project, x: number, y: number): Project["namedPoints"][number] | undefined {
  let best: Project["namedPoints"][number] | undefined;
  let bestD = MAGNET_METERS;
  for (const p of project.namedPoints) {
    const d = Math.hypot(p.x - x, p.y - y);
    if (d <= bestD) {
      best = p;
      bestD = d;
    }
  }
  return best;
}

/** The rotate cursor: a curved arrow, drawn for the canvas (the browser has none). */
const ROTATE_CURSOR = `url("data:image/svg+xml,${encodeURIComponent(
  '<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24"><g fill="none" stroke-linecap="round" stroke-linejoin="round"><path d="M18 8a7 7 0 1 0 1.5 6" stroke="#000" stroke-width="4"/><path d="M19 3v5h-5" stroke="#000" stroke-width="4"/><path d="M18 8a7 7 0 1 0 1.5 6" stroke="#fff" stroke-width="2"/><path d="M19 3v5h-5" stroke="#fff" stroke-width="2"/></g></svg>',
)}") 12 12, alias`;

const reducedMotion = () => typeof matchMedia === "function" && matchMedia("(prefers-reduced-motion: reduce)").matches;

/** The item under the pointer, for the hover highlight (null: nothing). */
export type HoverTarget = { readonly kind: "waypoint" | "turn" | "rotation"; readonly index: number } | null;

export function FieldCanvas() {
  const wrapRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imageRef = useRef<HTMLImageElement | null>(null);
  const sizeRef = useRef({ width: 800, height: 400 });
  const dragRef = useRef<Drag | null>(null);
  const frameRef = useRef(0);
  const hoverRef = useRef<HoverTarget>(null);
  const motionRef = useRef(0);
  const zoomTarget = useRef<{ zoom: number; px: number; py: number } | null>(null);
  const tool = useStore(editor, (s) => s.tool);
  const liveOverlay = useStore(ui, (u) => u.liveOverlay);

  // ---------------------------------------------------------------------------- drawing

  const draw = () => {
    frameRef.current = 0;
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    const { width, height } = sizeRef.current;
    const dpr = window.devicePixelRatio || 1;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const s = editor.get();
    const active = activePath(s);
    const a = analysis.get().result;
    const pb = playback.get();
    drawField(ctx, {
      view: makeView(s, width, height),
      image: imageRef.current,
      project: s.project,
      active,
      geometry: geometryOf(active.spec),
      analysis: a && a.ok ? a : null,
      selection: s.selection,
      tool: s.tool,
      playbackT: pb.t,
      showPlaybackRobot: pb.playing || pb.t > 0,
      hoverS: pb.hoverS,
      draftObstacle: s.draftObstacle,
      live: ui.get().liveOverlay ? live.get().robot : null,
      liveProgress: ui.get().liveOverlay && live.get().active ? { pathName: live.get().active, progressMeters: live.get().progressMeters } : null,
      hover: hoverRef.current,
      time: reducedMotion() ? 0 : performance.now(),
      geometryOf,
      ghost: optimiser.get().pathId === active.id ? optimiser.get().preview : null,
    });
  };

  const schedule = () => {
    if (frameRef.current === 0) frameRef.current = requestAnimationFrame(draw);
  };

  useEffect(() => {
    if (reducedMotion()) return;
    let frame = 0;
    const tick = () => {
      if (editor.get().selection?.kind === "waypoint") schedule();
      frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frame);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useEffect(() => schedule(), [liveOverlay]); // eslint-disable-line react-hooks/exhaustive-deps

  /** Glides the view (pan inertia, smooth zoom) until it settles; any press stops it. */
  const stopMotion = () => {
    if (motionRef.current) cancelAnimationFrame(motionRef.current);
    motionRef.current = 0;
    zoomTarget.current = null;
  };

  const glide = (vx0: number, vy0: number) => {
    stopMotion();
    if (reducedMotion() || Math.hypot(vx0, vy0) < 0.15) return;
    let vx = vx0;
    let vy = vy0;
    let last = performance.now();
    const step = (now: number) => {
      const dt = Math.min(48, now - last);
      last = now;
      const v = view();
      setView({ centerX: v.view.centerX - (vx * dt) / v.scale, centerY: v.view.centerY + (vy * dt) / v.scale });
      const k = Math.pow(0.9, dt / 16);
      vx *= k;
      vy *= k;
      motionRef.current = Math.hypot(vx, vy) > 0.02 ? requestAnimationFrame(step) : 0;
    };
    motionRef.current = requestAnimationFrame(step);
  };

  /** Right-click on the empty field. */
  const fieldMenu = (clientX: number, clientY: number, [px, py]: [number, number]) => {
    const [fx, fy] = view().toField(px, py);
    const x = round5(fx);
    const y = round5(fy);
    const n = activePath().spec.waypoints.length;
    openContextMenu(clientX, clientY, "Field", [
      { label: "Add a waypoint here", icon: <Plus size={14} />, run: () => editPath("Add waypoint", (p) => appendWaypoint(p, x, y), { kind: "waypoint", index: n }) },
      { label: "Start a new path here", icon: <CornerDownRight size={14} />, run: () => newPath({ x, y, heading: 0 }) },
      "separator",
      { label: "Blue or red view", icon: <Repeat size={14} />, shortcut: "A", run: toggleAlliance },
      { label: "Fit the field", icon: <Maximize2 size={14} />, shortcut: "F", run: fitView },
    ]);
  };

  useEffect(() => {
    const img = new Image();
    img.onload = () => {
      imageRef.current = img;
      schedule();
    };
    img.src = `${import.meta.env.BASE_URL}field-2026.png`;
    const unsubs = [editor.subscribe(schedule), analysis.subscribe(schedule), playback.subscribe(schedule), live.subscribe(schedule), optimiser.subscribe(schedule)];
    const wrap = wrapRef.current!;
    const observer = new ResizeObserver(() => {
      const rect = wrap.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      sizeRef.current = { width: rect.width, height: rect.height };
      const canvas = canvasRef.current!;
      canvas.width = Math.max(1, Math.round(rect.width * dpr));
      canvas.height = Math.max(1, Math.round(rect.height * dpr));
      draw();
    });
    observer.observe(wrap);
    return () => {
      for (const u of unsubs) u();
      observer.disconnect();
      if (frameRef.current) cancelAnimationFrame(frameRef.current);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // ---------------------------------------------------------------------------- hit testing

  const view = () => makeView(editor.get(), sizeRef.current.width, sizeRef.current.height);

  const pointer = (e: { clientX: number; clientY: number }): [number, number] => {
    const rect = canvasRef.current!.getBoundingClientRect();
    return [e.clientX - rect.left, e.clientY - rect.top];
  };

  const near = (v: FieldView, px: number, py: number, x: number, y: number, r = HIT_PX) => {
    const [cx, cy] = v.toCanvas(x, y);
    return Math.hypot(cx - px, cy - py) <= r;
  };

  const nearRaw = (v: FieldView, px: number, py: number, x: number, y: number, r = HIT_PX) => {
    const [cx, cy] = v.toCanvasRaw(x, y);
    return Math.hypot(cx - px, cy - py) <= r;
  };

  /** What's under a canvas point: knobs first (they're small), then points, then bodies. */
  const hitTest = (px: number, py: number): Hit | null => {
    const s = editor.get();
    const v = view();
    const path = activePath(s);
    const g = geometryOf(path.spec);
    const spec = path.spec;
    const a = analysis.get().result;
    const model = { active: path, analysis: a && a.ok ? a : null, project: s.project, geometry: g };
    for (let i = spec.waypoints.length - 1; i >= 0; i--) {
      const h = waypointHandle(model, i);
      if (near(v, px, py, h[2], h[3])) return { kind: "turn", index: i };
    }
    for (let i = spec.rotationTargets.length - 1; i >= 0; i--) {
      const h = rotationHandle(model, spec.rotationTargets[i]!);
      if (h && near(v, px, py, h[2], h[3])) return { kind: "rotation", index: i };
    }
    if (s.view.shapeCurve && s.selection?.kind === "waypoint") {
      const h = tangentHandle(model, s.selection.index);
      if (h && near(v, px, py, h.x, h.y)) return { kind: "tangent", index: s.selection.index };
    }
    for (let i = spec.waypoints.length - 1; i >= 0; i--) {
      const w = spec.waypoints[i]!;
      if (near(v, px, py, w.xMeters, w.yMeters, HIT_PX + 2)) return { kind: "waypoint", index: i };
    }
    if (g) {
      for (let i = spec.events.length - 1; i >= 0; i--) {
        const p = g.sample(Math.max(0, Math.min(g.lengthMeters, eventMeters(spec.events[i]!, g))));
        const [cx, cy] = v.toCanvas(p.xMeters, p.yMeters);
        // the flag stands above the point: accept the pole and the pennant
        if (Math.abs(px - cx - 4) <= 9 && py <= cy - 4 && py >= cy - 24) return { kind: "event", index: i };
      }
    }
    for (const p of s.project.namedPoints) if (near(v, px, py, p.x, p.y)) return { kind: "point", id: p.id };
    for (let i = spec.facePointZones.length - 1; i >= 0; i--) {
      const z = spec.facePointZones[i]!;
      if (near(v, px, py, z.point.x, z.point.y)) return { kind: "face", index: i };
    }
    const [fx, fy] = v.toField(px, py);
    // robot bodies grab in the tools that move them; the placing tools click straight through them
    if (g && (s.tool === "select" || s.tool === "rotation")) {
      // a heading's robot slides along the path
      for (let i = spec.rotationTargets.length - 1; i >= 0; i--) {
        const p = g.sample(rotationTargetS(spec.rotationTargets[i]!, g));
        if (Math.hypot(p.xMeters - fx, p.yMeters - fy) <= s.project.robot.bumperWidthMeters / 2) return { kind: "slide", index: i };
      }
    }
    if (s.tool === "select" || s.tool === "waypoint") {
      for (let i = spec.waypoints.length - 1; i >= 0; i--) {
        if (insideFootprint(model, i, fx, fy)) return { kind: "waypoint", index: i };
      }
    }
    const obstacles = s.project.field.obstacles;
    const editingObstacles = s.tool === "obstacle" || s.selection?.kind === "obstacle";
    if (editingObstacles) {
      for (let i = obstacles.length - 1; i >= 0; i--) {
        const poly = obstacles[i]!.polygon;
        for (let j = 0; j < poly.length; j++) {
          if (nearRaw(v, px, py, poly[j]![0], poly[j]![1])) return { kind: "vertex", obstacle: i, vertex: j };
        }
      }
    }
    if (s.tool === "select" || s.tool === "obstacle") {
      const [rx, ry] = v.toRaw(px, py);
      for (let i = obstacles.length - 1; i >= 0; i--) {
        if (insidePolygon(rx, ry, obstacles[i]!.polygon)) return { kind: "obstacle", index: i };
      }
    }
    return null;
  };

  /** The nearest point on the active path to a canvas point, and how far it is in pixels. */
  const projectToPath = (px: number, py: number): { s: number; px: number; g: PathGeometry } | null => {
    const s = editor.get();
    const g = geometryOf(activePath(s).spec);
    if (!g) return null;
    const v = view();
    const [x, y] = v.toField(px, py);
    const p = g.projectGlobal(x, y);
    return { s: p.s, px: p.distanceMeters * v.scale, g };
  };

  /** Inserts a pass-through point at s on the active path (a preview, inside a gesture). */
  const insertAt = (base: EditorPath, s: number, x: number, y: number): EditorPath => {
    const g = geometryOf(base.spec)!;
    const w = waypointAtS(g, s);
    const k = Math.min(base.spec.waypoints.length - 2, Math.floor(w));
    return insertWaypoint(base, k, w - k, x, y);
  };

  // ---------------------------------------------------------------------------- clicks on empty field

  const clickTool = (px: number, py: number, shift: boolean): boolean => {
    const s = editor.get();
    const v = view();
    const [fx, fy] = v.toField(px, py);
    const x = snapMeters(fx, shift, s.view.gridSnap);
    const y = snapMeters(fy, shift, s.view.gridSnap);
    const path = activePath(s);
    switch (s.tool) {
      case "waypoint": {
        const hit = projectToPath(px, py);
        if (hit && hit.px <= PATH_SNAP_PX && hit.s > 1e-3 && hit.s < hit.g.lengthMeters - 1e-3) {
          const p = hit.g.sample(hit.s);
          const k = Math.min(path.spec.waypoints.length - 2, Math.floor(waypointAtS(hit.g, hit.s)));
          editPath("Insert point", (pp) => insertAt(pp, hit.s, round5(p.xMeters), round5(p.yMeters)), { kind: "waypoint", index: k + 1 });
        } else {
          const point = magnet(s.project, x, y);
          const n = path.spec.waypoints.length;
          editProject(
            "Add point",
            (pr) => {
              const added = { ...pr, paths: pr.paths.map((p) => (p.id === path.id ? appendWaypoint(p, point?.x ?? x, point?.y ?? y) : p)) };
              return point ? linkToPoint(added, path.id, n, point.id) : added;
            },
            { kind: "waypoint", index: n },
          );
        }
        return true;
      }
      case "event": {
        const hit = projectToPath(px, py);
        if (!hit || hit.px > EVENT_SNAP_PX) {
          notify("Click on the path to place an event.");
          return true;
        }
        const names = eventNames(s.project);
        const last = s.selection?.kind === "event" ? path.spec.events[s.selection.index]?.name : undefined;
        const name = last ?? names[0] ?? "event";
        editPath("Add event", (pp) => addEvent(pp, makePathEvent(name, round5(waypointAtS(hit.g, hit.s)), Number.NaN)), {
          kind: "event",
          index: path.spec.events.length,
        });
        return true;
      }
      case "rotation": {
        const hit = projectToPath(px, py);
        if (!hit || hit.px > EVENT_SNAP_PX) {
          notify("Click on the path to add a heading there.");
          return true;
        }
        const a = analysis.get().result;
        const heading = a && a.ok ? headingAtS(a.stations, hit.s) : 0;
        const target = makeRotationTarget(round5(waypointAtS(hit.g, hit.s)), Number.NaN, snapHeading(heading, false));
        editPath("Add heading", (pp) => addRotationTarget(pp, target), {
          kind: "rotation",
          index: path.spec.rotationTargets.length,
        });
        return true;
      }
      case "obstacle": {
        const [rx, ry] = v.toRaw(px, py);
        const draft = s.draftObstacle ?? [];
        setDraftObstacle([...draft, [snapMeters(rx, shift, s.view.gridSnap), snapMeters(ry, shift, s.view.gridSnap)]]);
        return true;
      }
      case "select":
        return false;
    }
  };

  // ---------------------------------------------------------------------------- dragging

  const selectionOf = (hit: Hit): Selection => {
    switch (hit.kind) {
      case "waypoint":
      case "turn":
      case "tangent":
        return { kind: "waypoint", index: hit.index };
      case "rotation":
      case "slide":
        return { kind: "rotation", index: hit.index };
      case "event":
        return { kind: "event", index: hit.index };
      case "face":
        return { kind: "face", index: hit.index };
      case "point":
        return { kind: "point", id: hit.id };
      case "vertex":
        return { kind: "obstacle", index: hit.obstacle };
      case "obstacle":
        return { kind: "obstacle", index: hit.index };
    }
  };

  const onPointerDown = (e: React.PointerEvent<HTMLCanvasElement>) => {
    canvasRef.current!.focus();
    stopMotion();
    const [px, py] = pointer(e);
    const s = editor.get();
    const pan = (): Drag => ({
      kind: "pan",
      startX: px,
      startY: py,
      cx0: s.view.centerX,
      cy0: s.view.centerY,
      moved: false,
      button: e.button,
      vx: 0,
      vy: 0,
      lastX: px,
      lastY: py,
      lastT: performance.now(),
    });
    if (e.button === 2) {
      // right-click on a waypoint: its menu; anywhere else: pan, or (without moving) the field's menu
      const hit = hitTest(px, py);
      if (hit && (hit.kind === "waypoint" || hit.kind === "turn")) {
        select({ kind: "waypoint", index: hit.index });
        waypointMenu(hit.index, e.clientX, e.clientY);
        return;
      }
    }
    if (e.button === 1 || e.button === 2) {
      dragRef.current = pan();
      canvasRef.current!.setPointerCapture(e.pointerId);
      canvasRef.current!.style.cursor = "grabbing";
      return;
    }
    if (e.button !== 0) return;
    const hit = hitTest(px, py);
    if (hit) {
      select(selectionOf(hit));
      beginGesture();
      if (hit.kind === "obstacle") {
        const [rx, ry] = view().toRaw(px, py);
        dragRef.current = { kind: "obstacle", index: hit.index, startX: rx, startY: ry };
      } else if (hit.kind === "waypoint") {
        dragRef.current = { kind: "waypoint", index: hit.index, label: activePath(s).links[hit.index] ? "Move point" : "Move waypoint" };
      } else {
        dragRef.current = hit;
      }
      canvasRef.current!.setPointerCapture(e.pointerId);
      return;
    }
    // pressing on the path in Select or Waypoint: dragging away inserts a point there
    if (s.tool === "select" || s.tool === "waypoint") {
      const onPath = projectToPath(px, py);
      if (onPath && onPath.px <= PATH_SNAP_PX && onPath.s > 1e-3 && onPath.s < onPath.g.lengthMeters - 1e-3) {
        dragRef.current = { kind: "insert", s: onPath.s, startX: px, startY: py, started: false };
        canvasRef.current!.setPointerCapture(e.pointerId);
        return;
      }
    }
    if (clickTool(px, py, e.shiftKey)) return;
    // select tool on empty field: pan, and deselect if it was just a click
    dragRef.current = pan();
    canvasRef.current!.setPointerCapture(e.pointerId);
  };

  const onPointerMove = (e: React.PointerEvent<HTMLCanvasElement>) => {
    const [px, py] = pointer(e);
    const drag = dragRef.current;
    const v = view();
    const s = editor.get();
    if (!drag) {
      const hit = projectToPath(px, py);
      const reach = s.tool === "rotation" ? EVENT_SNAP_PX : PATH_SNAP_PX;
      const hoverS = hit && hit.px <= reach ? hit.s : Number.NaN;
      if (!(Number.isNaN(hoverS) && Number.isNaN(playback.get().hoverS))) playback.set({ hoverS });
      const under = hitTest(px, py);
      const hover: HoverTarget = under && (under.kind === "waypoint" || under.kind === "turn" || under.kind === "rotation") ? { kind: under.kind, index: under.index } : null;
      if (hover?.kind !== hoverRef.current?.kind || hover?.index !== hoverRef.current?.index) {
        hoverRef.current = hover;
        schedule();
      }
      canvasRef.current!.style.cursor = under
        ? under.kind === "turn" || under.kind === "rotation"
          ? ROTATE_CURSOR
          : under.kind === "tangent" || under.kind === "vertex"
            ? "crosshair"
            : "move"
        : s.tool === "select"
          ? Number.isFinite(hoverS)
            ? "copy"
            : "default"
          : "crosshair";
      return;
    }
    const [fx, fy] = v.toField(px, py);
    const x = snapMeters(fx, e.shiftKey, s.view.gridSnap);
    const y = snapMeters(fy, e.shiftKey, s.view.gridSnap);
    const id = activePath(s).id;
    const onPath = (fn: (p: EditorPath) => EditorPath) =>
      previewProject((base) => ({ ...base, paths: base.paths.map((p) => (p.id === id ? fn(p) : p)) }));
    switch (drag.kind) {
      case "pan": {
        if (Math.hypot(px - drag.startX, py - drag.startY) > 3) drag.moved = true;
        const now = performance.now();
        const dt = Math.max(1, now - drag.lastT);
        // a smoothed velocity, px/ms, for the glide after release
        drag.vx = 0.7 * ((px - drag.lastX) / dt) + 0.3 * drag.vx;
        drag.vy = 0.7 * ((py - drag.lastY) / dt) + 0.3 * drag.vy;
        drag.lastX = px;
        drag.lastY = py;
        drag.lastT = now;
        setView({ centerX: drag.cx0 - (px - drag.startX) / v.scale, centerY: drag.cy0 + (py - drag.startY) / v.scale });
        return;
      }
      case "insert": {
        if (!drag.started && Math.hypot(px - drag.startX, py - drag.startY) < DRAG_START_PX) return;
        if (!drag.started) {
          drag.started = true;
          beginGesture();
        }
        onPath((p) => {
          const inserted = insertAt(p, drag.s, x, y);
          const k = inserted.spec.waypoints.findIndex((w, i) => w !== p.spec.waypoints[i]);
          return moveWaypoint(inserted, k, x, y);
        });
        return;
      }
      case "waypoint": {
        const path = activePath(s);
        const link = path.links[drag.index];
        if (link) {
          previewProject((base) => moveNamedPoint(base, link, x, y));
          return;
        }
        const point = magnet(s.project, fx, fy);
        previewProject((base) => {
          const moved = { ...base, paths: base.paths.map((p) => (p.id === id ? moveWaypoint(p, drag.index, point?.x ?? x, point?.y ?? y) : p)) };
          return point ? linkToPoint(moved, id, drag.index, point.id) : moved;
        });
        return;
      }
      case "turn": {
        onPath((p) => {
          const w = p.spec.waypoints[drag.index]!;
          return setHeading(p, drag.index, snapHeading(Math.atan2(fy - w.yMeters, fx - w.xMeters), e.altKey));
        });
        return;
      }
      case "tangent": {
        onPath((p) => {
          const w = p.spec.waypoints[drag.index]!;
          const dx = fx - w.xMeters;
          const dy = fy - w.yMeters;
          const angle = snapHeading(Math.atan2(dy, dx), !e.shiftKey);
          const scale = Math.round(Math.min(3, Math.max(0.2, Math.hypot(dx, dy) / TANGENT_HANDLE_METERS)) * 100) / 100;
          return setTangent(p, drag.index, angle, scale);
        });
        return;
      }
      case "rotation": {
        onPath((p) => {
          const t = p.spec.rotationTargets[drag.index]!;
          const g = geometryOf(p.spec);
          if (!g) return p;
          const c = g.sample(rotationTargetS(t, g));
          const heading = snapHeading(Math.atan2(fy - c.yMeters, fx - c.xMeters), e.altKey);
          return updateRotationTarget(p, drag.index, makeRotationTarget(t.waypoint, t.meters, heading));
        });
        return;
      }
      case "slide": {
        onPath((p) => {
          const t = p.spec.rotationTargets[drag.index]!;
          const g = geometryOf(p.spec);
          if (!g) return p;
          const along = g.projectGlobal(fx, fy).s;
          const moved = Number.isNaN(t.waypoint)
            ? makeRotationTarget(Number.NaN, round5(along), t.headingRadians)
            : makeRotationTarget(round5(waypointAtS(g, along)), Number.NaN, t.headingRadians);
          return updateRotationTarget(p, drag.index, moved);
        });
        return;
      }
      case "face": {
        onPath((p) => {
          const z = p.spec.facePointZones[drag.index]!;
          return updateFacePointZone(p, drag.index, { ...z, point: { x, y } });
        });
        return;
      }
      case "point": {
        previewProject((base) => moveNamedPoint(base, drag.id, x, y));
        return;
      }
      case "event": {
        onPath((p) => {
          const g = geometryOf(p.spec);
          if (!g) return p;
          const along = g.projectGlobal(fx, fy).s;
          const ev = p.spec.events[drag.index]!;
          const moved = Number.isNaN(ev.waypoint)
            ? makePathEvent(ev.name, Number.NaN, round5(along))
            : makePathEvent(ev.name, round5(waypointAtS(g, along)), Number.NaN);
          return updateEvent(p, drag.index, moved);
        });
        return;
      }
      case "vertex": {
        const [rx, ry] = v.toRaw(px, py);
        previewProject((base) =>
          withObstacle(base, drag.obstacle, (poly) =>
            poly.map((q, j) => (j === drag.vertex ? [snapMeters(rx, e.shiftKey, s.view.gridSnap), snapMeters(ry, e.shiftKey, s.view.gridSnap)] : q)),
          ),
        );
        return;
      }
      case "obstacle": {
        const [rx, ry] = v.toRaw(px, py);
        const dx = snapMeters(rx - drag.startX, e.shiftKey, s.view.gridSnap);
        const dy = snapMeters(ry - drag.startY, e.shiftKey, s.view.gridSnap);
        previewProject((base) => withObstacle(base, drag.index, (poly) => poly.map(([qx, qy]) => [round5(qx + dx), round5(qy + dy)])));
        return;
      }
    }
  };

  const onPointerUp = (e: React.PointerEvent<HTMLCanvasElement>) => {
    const drag = dragRef.current;
    dragRef.current = null;
    if (canvasRef.current!.hasPointerCapture(e.pointerId)) canvasRef.current!.releasePointerCapture(e.pointerId);
    if (!drag) return;
    if (drag.kind === "pan") {
      canvasRef.current!.style.cursor = "";
      if (!drag.moved && drag.button === 0) select(null);
      else if (!drag.moved && drag.button === 2) fieldMenu(e.clientX, e.clientY, pointer(e));
      else if (drag.moved && performance.now() - drag.lastT < 60) glide(drag.vx, drag.vy);
      return;
    }
    if (drag.kind === "insert") {
      if (drag.started) {
        endGesture("Insert point");
        const path = activePath();
        const g = geometryOf(path.spec);
        if (g) select({ kind: "waypoint", index: Math.min(path.spec.waypoints.length - 2, Math.floor(waypointAtS(g, drag.s))) + 1 });
      } else {
        // a click on the path: the Waypoint tool inserts there, Select just selects nothing
        const [px, py] = pointer(e);
        if (!clickTool(px, py, e.shiftKey)) select(null);
      }
      return;
    }
    endGesture(dragLabel(drag));
  };

  const onDoubleClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const [px, py] = pointer(e);
    const s = editor.get();
    if (s.tool === "obstacle" && s.draftObstacle) {
      finishObstacle();
      return;
    }
    const hit = hitTest(px, py);
    if (hit?.kind === "tangent") {
      editPath("Automatic tangent", (p) => setTangent(p, hit.index, Number.NaN, 1.0));
    }
  };

  const onWheel = (e: WheelEvent) => {
    e.preventDefault();
    const [px, py] = pointer(e);
    // a pinch arrives as Ctrl+wheel with small steps: it zooms faster per step
    const factor = Math.exp(-e.deltaY * (e.ctrlKey ? 0.01 : 0.0015));
    if (reducedMotion()) {
      setView(view().zoomAbout(px, py, factor));
      return;
    }
    const from = zoomTarget.current?.zoom ?? editor.get().view.zoom;
    if (motionRef.current && !zoomTarget.current) stopMotion();
    zoomTarget.current = { zoom: Math.min(40, Math.max(0.3, from * factor)), px, py };
    if (motionRef.current) return;
    const step = () => {
      const t = zoomTarget.current;
      if (!t) {
        motionRef.current = 0;
        return;
      }
      const z = editor.get().view.zoom;
      const next = Math.abs(t.zoom - z) < 1e-3 * z ? t.zoom : z + (t.zoom - z) * 0.35;
      setView(view().zoomAbout(t.px, t.py, next / z));
      if (next === t.zoom) {
        zoomTarget.current = null;
        motionRef.current = 0;
      } else motionRef.current = requestAnimationFrame(step);
    };
    motionRef.current = requestAnimationFrame(step);
  };

  useEffect(() => {
    const canvas = canvasRef.current!;
    canvas.addEventListener("wheel", onWheel, { passive: false });
    return () => canvas.removeEventListener("wheel", onWheel);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  return (
    <div ref={wrapRef} className="field-wrap">
      <canvas
        ref={canvasRef}
        className="field-canvas"
        tabIndex={0}
        role="application"
        aria-label="Field. Drag robots to move them and their knobs to turn them; every item can also be edited in the side panel."
        data-testid="field"
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerUp}
        onPointerLeave={() => !dragRef.current && playback.set({ hoverS: Number.NaN })}
        onDoubleClick={onDoubleClick}
        onContextMenu={(e) => e.preventDefault()}
      />
      <div className="canvas-hint" data-testid="canvas-hint" aria-live="polite">
        {TOOL_HINTS[tool]}
      </div>
      <QuickStartOverlay />
    </div>
  );
}

/** Right-click on a waypoint: its heading, pin and delete (the app's context menu). */
function waypointMenu(index: number, x: number, y: number): void {
  const path = activePath();
  const w = path.spec.waypoints[index];
  if (!w) return;
  const n = path.spec.waypoints.length;
  const isEnd = index === 0 || index === n - 1;
  const passThrough = Number.isNaN(w.headingRadians);
  const planned = () => {
    const a = analysis.get().result;
    return snapHeading(waypointPose({ active: path, analysis: a && a.ok ? a : null }, index).heading, false);
  };
  const items: MenuEntry[] = [
    {
      label: "Set heading…",
      icon: <Compass size={14} />,
      run: () => {
        if (passThrough) editPath("Set heading", (p) => setHeading(p, index, planned()));
        // the inspector's heading field takes the typed angle
        setTimeout(() => document.querySelector<HTMLInputElement>('[data-testid="waypoint-heading"]')?.focus(), 0);
      },
    },
  ];
  if (!isEnd) {
    items.push({
      label: passThrough ? "Hold a heading here" : "Pass through (no heading)",
      icon: <Navigation size={14} />,
      run: () => editPath(passThrough ? "Set heading" : "Pass through", (p) => setHeading(p, index, passThrough ? planned() : Number.NaN)),
    });
  }
  items.push(
    {
      label: path.pinned[index] ? "Unpin (the optimiser may move it)" : "Pin (the optimiser leaves it)",
      icon: path.pinned[index] ? <LockOpen size={14} /> : <Lock size={14} />,
      run: () => editPath(path.pinned[index] ? "Unpin" : "Pin", (p) => setPinned(p, index, !p.pinned[index])),
    },
    "separator",
    {
      label: "Delete waypoint",
      icon: <Trash2 size={14} />,
      shortcut: "Del",
      danger: true,
      disabled: n <= 2,
      run: () => editPath("Delete waypoint", (p) => deleteWaypoint(p, index), null),
    },
  );
  openContextMenu(x, y, `Waypoint ${index}`, items);
}

/** The undo label for a finished drag. */
function dragLabel(drag: Exclude<Drag, { kind: "pan" } | { kind: "insert" }>): string {
  switch (drag.kind) {
    case "waypoint":
      return drag.label;
    case "turn":
      return "Turn waypoint";
    case "tangent":
      return "Change tangent";
    case "rotation":
      return "Turn heading";
    case "slide":
      return "Move heading";
    case "face":
      return "Move face point";
    case "point":
      return "Move point";
    case "event":
      return "Move event";
    case "vertex":
      return "Move obstacle corner";
    case "obstacle":
      return "Move obstacle";
  }
}

function withObstacle(
  project: Project,
  index: number,
  fn: (poly: readonly (readonly [number, number])[]) => (readonly [number, number])[],
): Project {
  const obstacles = project.field.obstacles.map((o, i) => (i === index ? { ...o, polygon: fn(o.polygon) } : o));
  return { ...project, field: { ...project.field, obstacles } };
}

/** Turns the obstacle being drawn into an obstacle (three corners or more). */
export function finishObstacle(): void {
  const s = editor.get();
  // a double-click to finish also lands two clicks on the last corner: drop repeats
  const draft = s.draftObstacle?.filter((p, i, all) => i === 0 || Math.hypot(p[0] - all[i - 1]![0], p[1] - all[i - 1]![1]) > 0.01);
  setDraftObstacle(null);
  if (!draft || draft.length < 3) {
    if (draft && draft.length > 0) notify("An obstacle needs at least three corners.", "error");
    return;
  }
  const taken = new Set(s.project.field.obstacles.map((o) => o.name));
  let n = s.project.field.obstacles.length + 1;
  while (taken.has(`obstacle ${n}`)) n++;
  editProject(
    "Add obstacle",
    (p) => ({ ...p, field: { ...p.field, obstacles: [...p.field.obstacles, { name: `obstacle ${n}`, polygon: draft.map(([x, y]) => [x, y] as const) }] } }),
    { kind: "obstacle", index: s.project.field.obstacles.length },
  );
}

/** Leaves the Waypoint tool (Enter or Esc while drawing). */
export function finishDrawing(): void {
  setTool("select");
}
