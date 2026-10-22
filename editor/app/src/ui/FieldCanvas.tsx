// The field: draws the project and turns pointer input into edits. Every edit goes through the
// editor store (one undo step per click or drag); this component only works out what was hit.
//
// Swerve-first: a waypoint is a robot pose. Its footprint moves it, the knob on its ring turns it
// (15° steps; Alt turns freely), and a pass-through point shows the plan's heading, dashed. Travel
// direction is automatic; the tangent handle only shows with "shape the curve" on.

import { QuickStartOverlay } from "./QuickStart.tsx";
import { FlowBar, FlowForm } from "./editor/FlowBar.tsx";
import { deleteAnchor, DELETE_RADIUS, flowKnob, zoneChip, type ItemRef } from "./drawItems.ts";
import { selectionTitle } from "./inspector/Items.tsx";
import { finishRotation, flow, pickOnPath, pickPoint, setFlowHeading, stepTakes } from "../state/flow.ts";
import { faceNext, faceTargets, faceToward, poseNearElement, STANDOFF_METERS } from "../state/helpers.ts";
import { deleteItem, selectionOfItem } from "../state/actions.ts";
import { Compass, CornerDownRight, Crosshair, Lock, LockOpen, Maximize2, Navigation, Plus, Repeat, Trash2 } from "lucide-react";
import { useEffect, useRef } from "react";
import { eventMeters, makePathEvent, makeRotationTarget, type PathGeometry } from "../../../core/src/index.ts";
import {
  addWaypointAfter,
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
import { type EditorPath, type Project } from "../model/project.ts";
import { round5, round5Radians, waypointAtS } from "../model/positions.ts";
import { drawSim } from "../sim/draw.ts";
import { sim, simSettings } from "../sim/state.ts";
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
  refuseLinkedCopy,
  select,
  setDraftObstacle,
  setView,
  type EditorState,
  type Selection,
} from "../state/editor.ts";
import { live } from "../state/live.ts";
import { optimiser } from "../state/optimiser.ts";
import { useStore } from "../state/store.ts";
import { openContextMenu, toast, ui, type MenuEntry } from "../state/ui.ts";
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
  | { kind: "obstacle"; index: number; startX: number; startY: number }
  | { kind: "flowKnob" }
  | { kind: "measure" };

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
  | { kind: "obstacle"; index: number }
  | { kind: "zone"; zone: "constraint" | "face"; index: number }
  | { kind: "delete"; ref: ItemRef };

const HIT_PX = 10;
/** A field click this close to a named point or a field element's centre snaps to it, metres. */
const ELEMENT_SNAP_METERS = 0.4;
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
  select:
    "Select a waypoint and double-click the field to add one after it. Drag a robot to move it, the knob on its ring to turn it (Alt: freely). Drag the path to insert a point. Right-click a point for more.",
  obstacle: "Click the corners of an obstacle; double-click or Enter to finish.",
  measure: "Drag from one point to another to measure (ends snap to waypoints and named points; Shift: 5 cm grid).",
};

/** The Select tool's hint on a path still being drawn. */
function drawingHint(n: number): string | null {
  return n === 0 ? "Double-click the field to place the start." : n === 1 ? "Double-click the field to place the end." : null;
}

/** What a double-click on the field says when it can't add a waypoint. */
export const ADD_HINT = "Select a waypoint, then double-click to add one after it";
/** Presses closer together than this are one click sequence (a double-click), ms. */
const DOUBLE_CLICK_MS = 600;

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

/** The deletable item a hit is on (the × shows for it). */
function itemOf(hit: Hit | null): ItemRef | null {
  if (!hit) return null;
  switch (hit.kind) {
    case "event":
      return { kind: "event", index: hit.index };
    case "rotation":
    case "slide":
      return { kind: "rotation", index: hit.index };
    case "face":
      return { kind: "face", index: hit.index };
    case "zone":
      return { kind: hit.zone, index: hit.index };
    case "point":
      return { kind: "point", id: hit.id };
    case "delete":
      return hit.ref;
    default:
      return null;
  }
}

function itemOfSelection(sel: Selection | null): ItemRef | null {
  if (!sel) return null;
  if (sel.kind === "point") return { kind: "point", id: sel.id };
  if (sel.kind === "event" || sel.kind === "rotation" || sel.kind === "face" || sel.kind === "constraint") return { kind: sel.kind, index: sel.index };
  return null;
}

const sameItem = (a: ItemRef | null, b: ItemRef | null) =>
  a === b || (a !== null && b !== null && a.kind === b.kind && ("id" in a ? a.id === (b as { id: string }).id : a.index === (b as { index: number }).index));

export function FieldCanvas() {
  const wrapRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imageRef = useRef<HTMLImageElement | null>(null);
  const sizeRef = useRef({ width: 800, height: 400 });
  const dragRef = useRef<Drag | null>(null);
  const frameRef = useRef(0);
  const hoverRef = useRef<HoverTarget>(null);
  /** The item under the pointer (its × shows), whether the pointer is on the ×, and the field point under it. */
  const hoverItemRef = useRef<ItemRef | null>(null);
  const deleteHotRef = useRef(false);
  const fieldHoverRef = useRef<{ x: number; y: number } | null>(null);
  /** The Measure tool's line, blue frame. */
  const measureRef = useRef<{ a: [number, number]; b: [number, number] } | null>(null);
  const flowState = useStore(flow, (f) => f);
  const motionRef = useRef(0);
  const zoomTarget = useRef<{ zoom: number; px: number; py: number } | null>(null);
  const tool = useStore(editor, (s) => s.tool);
  const drawing = useStore(editor, (s) => drawingHint(activePath(s).spec.waypoints.length));
  /** The waypoint selected at the last press: a double-click adds after it, though its first click deselected it. */
  const deselectedRef = useRef<{ index: number; pathId: string } | null>(null);
  /** When the last press was: a press long after it starts a new click sequence. */
  const lastPressRef = useRef(-Infinity);
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
    const view = makeView(s, width, height);
    drawField(ctx, {
      view,
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
      deletable: hoverItemRef.current ?? itemOfSelection(s.selection),
      deleteHot: deleteHotRef.current,
      flow: flow.get(),
      flowHover: fieldHoverRef.current,
      measure: s.tool === "measure" ? measureRef.current : null,
      blocked: optimiser.get().pathId === active.id && optimiser.get().status === "done" ? (optimiser.get().result?.blockedAt ?? null) : null,
    });
    const simResult = sim.get().result;
    if (simSettings.get().enabled && simResult?.ok && sim.get().input?.spec === active.spec) {
      const { bumperLengthMeters, bumperWidthMeters } = s.project.robot;
      drawSim(ctx, { view, preview: simResult, bumperLengthMeters, bumperWidthMeters, playbackT: pb.t, showRobot: pb.playing || pb.t > 0 });
    }
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
    const s = editor.get();
    const n = activePath(s).spec.waypoints.length;
    const after = s.selection?.kind === "waypoint" ? s.selection.index : n < 2 ? n - 1 : null;
    const add: MenuEntry[] =
      after === null
        ? []
        : [
            {
              label: n === 0 ? "Place the start here" : n === 1 ? "Place the end here" : `Add a waypoint after ${after} here`,
              icon: <Plus size={14} />,
              run: () => addAfter(after, x, y),
            },
          ];
    openContextMenu(clientX, clientY, "Field", [
      ...add,
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
    const unsubs = [
      editor.subscribe(schedule),
      analysis.subscribe(schedule),
      playback.subscribe(schedule),
      live.subscribe(schedule),
      optimiser.subscribe(schedule),
      flow.subscribe(schedule),
      sim.subscribe(schedule),
      simSettings.subscribe(schedule),
    ];
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
    // the × on the item under the pointer, or the selected one
    const items = { view: v, active: path, geometry: g, project: s.project };
    for (const ref of [hoverItemRef.current, itemOfSelection(s.selection)]) {
      const at = ref ? deleteAnchor(items, ref) : null;
      if (ref && at && Math.hypot(at[0] - px, at[1] - py) <= DELETE_RADIUS + 3) return { kind: "delete", ref };
    }
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
    for (const zone of ["constraint", "face"] as const) {
      const n = zone === "constraint" ? spec.constraintZones.length : spec.facePointZones.length;
      for (let i = n - 1; i >= 0; i--) {
        const c = zoneChip(items, zone, i);
        if (c && Math.abs(px - c.x) <= c.w / 2 && Math.abs(py - c.y) <= c.h / 2 + 2) return { kind: "zone", zone, index: i };
      }
    }
    for (let i = spec.facePointZones.length - 1; i >= 0; i--) {
      const z = spec.facePointZones[i]!;
      if (near(v, px, py, z.point.x, z.point.y)) return { kind: "face", index: i };
    }
    const [fx, fy] = v.toField(px, py);
    // robot bodies grab in the tools that move them; the placing tools click straight through them
    if (g && s.tool === "select") {
      // a heading's robot slides along the path
      for (let i = spec.rotationTargets.length - 1; i >= 0; i--) {
        const p = g.sample(rotationTargetS(spec.rotationTargets[i]!, g));
        if (Math.hypot(p.xMeters - fx, p.yMeters - fy) <= s.project.robot.bumperWidthMeters / 2) return { kind: "slide", index: i };
      }
    }
    if (s.tool === "select") {
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
    switch (s.tool) {
      case "obstacle": {
        const [rx, ry] = v.toRaw(px, py);
        const draft = s.draftObstacle ?? [];
        setDraftObstacle([...draft, [snapMeters(rx, shift, s.view.gridSnap), snapMeters(ry, shift, s.view.gridSnap)]]);
        return true;
      }
      case "select":
      case "measure":
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
      case "zone":
        return { kind: hit.zone, index: hit.index };
      case "delete":
        return editor.get().selection!;
    }
  };

  // ---------------------------------------------------------------------------- guided flows

  /** A field point, snapped to a named point or a field element's centre when one is close. */
  const snapToElement = (x: number, y: number): { x: number; y: number; snappedTo: string | null } => {
    const s = editor.get();
    let best: { x: number; y: number; snappedTo: string | null; d: number } = { x, y, snappedTo: null, d: ELEMENT_SNAP_METERS };
    for (const p of s.project.namedPoints) {
      const d = Math.hypot(p.x - x, p.y - y);
      if (d < best.d) best = { x: p.x, y: p.y, snappedTo: p.name, d };
    }
    for (const o of s.project.field.obstacles) {
      if (o.polygon.length === 0) continue;
      const cx = o.polygon.reduce((t, q) => t + q[0], 0) / o.polygon.length;
      const cy = o.polygon.reduce((t, q) => t + q[1], 0) / o.polygon.length;
      const d = Math.hypot(cx - x, cy - y);
      if (d < best.d) best = { x: round5(cx), y: round5(cy), snappedTo: o.name, d };
    }
    return { x: best.x, y: best.y, snappedTo: best.snappedTo };
  };

  /** A Measure end: on a waypoint or named point within reach, on the 5 cm grid with Shift, else where pressed. */
  const measureEnd = (px: number, py: number, shift: boolean): [number, number] => {
    const s = editor.get();
    const v = view();
    for (const w of activePath(s).spec.waypoints) if (near(v, px, py, w.xMeters, w.yMeters, 12)) return [w.xMeters, w.yMeters];
    for (const q of s.project.namedPoints) if (near(v, px, py, q.x, q.y, 12)) return [q.x, q.y];
    const [fx, fy] = v.toField(px, py);
    return [snapMeters(fx, shift, false), snapMeters(fy, shift, false)];
  };

  /** A press while a flow runs: its knob, a place on the path, or a point on the field. True if it took it. */
  const flowPress = (px: number, py: number, shift: boolean): Drag | boolean => {
    const f = flow.get();
    if (!f.kind) return false;
    const s = editor.get();
    const v = view();
    const knob = flowKnob({ geometry: geometryOf(activePath(s).spec), project: s.project }, f);
    if (knob && near(v, px, py, knob[0], knob[1], HIT_PX + 2)) return { kind: "flowKnob" };
    const takes = stepTakes(f.kind, f.step);
    if (takes === "path") {
      const hit = projectToPath(px, py);
      if (!hit || hit.px > EVENT_SNAP_PX) {
        notify("Click on the path.");
        return true;
      }
      const a = analysis.get().result;
      pickOnPath(waypointAtS(hit.g, hit.s), a && a.ok ? headingAtS(a.stations, hit.s) : 0);
      return true;
    }
    if (takes === "field") {
      const [fx, fy] = v.toField(px, py);
      const p = shift ? { x: snapMeters(fx, true, false), y: snapMeters(fy, true, false), snappedTo: null } : snapToElement(fx, fy);
      pickPoint(p.x, p.y, p.snappedTo);
      return true;
    }
    return false;
  };

  const onPointerDown = (e: React.PointerEvent<HTMLCanvasElement>) => {
    canvasRef.current!.focus();
    // remember the selected waypoint for a double-click (its first click deselects it); a press long
    // after the last one starts a new click sequence with nothing remembered
    const now = performance.now();
    const s0 = editor.get();
    if (s0.selection?.kind === "waypoint") deselectedRef.current = { index: s0.selection.index, pathId: s0.activePathId };
    else if (now - lastPressRef.current > DOUBLE_CLICK_MS) deselectedRef.current = null;
    lastPressRef.current = now;
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
      // right-click on a waypoint or an item: its menu; anywhere else: pan, or (without moving) the field's menu
      const hit = hitTest(px, py);
      if (hit && (hit.kind === "waypoint" || hit.kind === "turn")) {
        select({ kind: "waypoint", index: hit.index });
        waypointMenu(hit.index, e.clientX, e.clientY);
        return;
      }
      const item = itemOf(hit);
      if (item) {
        const sel = selectionOfItem(item);
        select(sel);
        openContextMenu(e.clientX, e.clientY, selectionTitle(sel, { events: activePath(s).spec.events, obstacles: s.project.field.obstacles, points: s.project.namedPoints }), [
          { label: "Delete", icon: <Trash2 size={14} />, shortcut: "Del", danger: true, run: () => deleteItem(sel) },
        ]);
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
    if (s.tool === "measure") {
      const a = measureEnd(px, py, e.shiftKey);
      measureRef.current = { a, b: a };
      dragRef.current = { kind: "measure" };
      canvasRef.current!.setPointerCapture(e.pointerId);
      schedule();
      return;
    }
    const taken = flowPress(px, py, e.shiftKey);
    if (taken) {
      if (typeof taken === "object") {
        dragRef.current = taken;
        canvasRef.current!.setPointerCapture(e.pointerId);
      }
      return;
    }
    const hit = hitTest(px, py);
    if (hit?.kind === "delete") {
      deleteItem(selectionOfItem(hit.ref));
      hoverItemRef.current = null;
      deleteHotRef.current = false;
      return;
    }
    if (hit?.kind === "zone") {
      select(selectionOf(hit));
      return;
    }
    // a linked copy follows its source: its own robots and markers don't move
    const pathItem = hit && hit.kind !== "point" && hit.kind !== "vertex" && hit.kind !== "obstacle";
    if (hit && pathItem && activePath(s).mirrorOf) {
      // (a waypoint on a named point moves the point, which is shared: that's allowed)
      const link = hit.kind === "waypoint" ? activePath(s).links[hit.index] : null;
      if (link) {
        select({ kind: "point", id: link });
        beginGesture();
        dragRef.current = { kind: "point", id: link };
        canvasRef.current!.setPointerCapture(e.pointerId);
        return;
      }
      select(selectionOf(hit));
      refuseLinkedCopy();
      return;
    }
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
    // pressing on the path in Select: dragging away inserts a point there
    if (s.tool === "select") {
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
      const f = flow.get();
      if (f.kind) {
        const [fx, fy] = v.toField(px, py);
        fieldHoverRef.current = stepTakes(f.kind, f.step) === "field" ? snapToElement(fx, fy) : null;
        schedule();
      }
      const hit = projectToPath(px, py);
      const reach = f.kind && stepTakes(f.kind, f.step) === "path" ? EVENT_SNAP_PX : PATH_SNAP_PX;
      const hoverS = hit && hit.px <= reach ? hit.s : Number.NaN;
      if (!(Number.isNaN(hoverS) && Number.isNaN(playback.get().hoverS))) playback.set({ hoverS });
      const under = hitTest(px, py);
      const hover: HoverTarget = under && (under.kind === "waypoint" || under.kind === "turn" || under.kind === "rotation") ? { kind: under.kind, index: under.index } : null;
      if (hover?.kind !== hoverRef.current?.kind || hover?.index !== hoverRef.current?.index) {
        hoverRef.current = hover;
        schedule();
      }
      // the × stays while the pointer goes from the item to it
      const item = itemOf(under);
      const hot = under?.kind === "delete";
      if (!sameItem(item, hoverItemRef.current) || hot !== deleteHotRef.current) {
        hoverItemRef.current = item;
        deleteHotRef.current = hot;
        schedule();
      }
      if (f.kind) {
        canvasRef.current!.style.cursor = "crosshair";
        return;
      }
      canvasRef.current!.style.cursor = under
        ? under.kind === "delete" || under.kind === "zone"
          ? "pointer"
          : under.kind === "turn" || under.kind === "rotation"
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
      case "measure": {
        if (measureRef.current) measureRef.current = { a: measureRef.current.a, b: measureEnd(px, py, e.shiftKey) };
        schedule();
        return;
      }
      case "flowKnob": {
        const g = geometryOf(activePath(s).spec);
        const f = flow.get();
        if (!g || f.kind !== "rotation") return;
        const c = g.sample(g.sAtWaypoint(f.picks[0]!));
        setFlowHeading(snapHeading(Math.atan2(fy - c.yMeters, fx - c.xMeters), e.altKey));
        return;
      }
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
    if (drag.kind === "measure") return;
    // letting go of the ghost robot's knob sets the heading
    if (drag.kind === "flowKnob") {
      finishRotation();
      return;
    }
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
        // a click on the path just selects nothing
        const [px, py] = pointer(e);
        if (!clickTool(px, py, e.shiftKey)) select(null);
      }
      return;
    }
    endGesture(dragLabel(drag));
  };


  /** Adds a waypoint at (x, y) after waypoint i of the active path, and selects it (a named point nearby links it). */
  const addAfter = (i: number, x: number, y: number) => {
    const s = editor.get();
    const path = activePath(s);
    const n = path.spec.waypoints.length;
    const point = magnet(s.project, x, y);
    const [ax, ay] = point ? [point.x, point.y] : [x, y];
    // between two waypoints: where it lands on that stretch, so things placed by waypoint keep their place
    let f = 0.5;
    const g = i < n - 1 ? geometryOf(path.spec) : null;
    if (g) {
      const w = waypointAtS(g, g.projectGlobal(ax, ay).s);
      if (Math.floor(w) === i) f = w - i;
    }
    const at = Math.min(i + 1, n);
    const ok = editProject(
      n === 0 ? "Place start" : n === 1 ? "Place end" : "Add waypoint",
      (pr) => {
        const added = { ...pr, paths: pr.paths.map((p) => (p.id === path.id ? addWaypointAfter(p, i, ax, ay, f) : p)) };
        return point ? linkToPoint(added, path.id, at, point.id) : added;
      },
      { kind: "waypoint", index: at },
    );
    const pose = ok && !point ? poseNearElement(ax, ay) : null;
    if (pose) {
      const id = path.id;
      toast(`Snap to ${STANDOFF_METERS} m in front of the ${pose.name}, facing it?`, "info", 9000, {
        label: "Snap",
        run: () => {
          if (activePath().id !== id || !activePath().spec.waypoints[at]) return;
          editPath("Snap to the element", (p) => {
            const moved = moveWaypoint(p, at, pose.x, pose.y);
            return setHeading(moved, at, pose.heading);
          });
        },
      });
    }
  };

  const onDoubleClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const [px, py] = pointer(e);
    const s = editor.get();
    if (s.tool === "obstacle" && s.draftObstacle) {
      finishObstacle();
      return;
    }
    if (s.tool !== "select") return;
    const hit = hitTest(px, py);
    if (hit?.kind === "tangent") {
      editPath("Automatic tangent", (p) => setTangent(p, hit.index, Number.NaN, 1.0));
      return;
    }
    // on a robot, a knob or a marker the clicks selected it; an obstacle is just more field here, and a
    // named point is where the new waypoint goes (linked to it)
    if (hit && hit.kind !== "obstacle" && hit.kind !== "point") return;
    // the first click of the double-click deselected the waypoint it builds on
    const path = activePath(s);
    const n = path.spec.waypoints.length;
    const recent = deselectedRef.current;
    const after =
      s.selection?.kind === "waypoint"
        ? s.selection.index
        : recent && recent.pathId === path.id && recent.index < n
          ? recent.index
          : n < 2
            ? n - 1 // an empty path: the start, then the end
            : null;
    deselectedRef.current = null;
    lastPressRef.current = -Infinity; // the next press starts a new sequence
    if (after === null) {
      notify(ADD_HINT);
      return;
    }
    const [fx, fy] = view().toField(px, py);
    addAfter(after, snapMeters(fx, e.shiftKey, s.view.gridSnap), snapMeters(fy, e.shiftKey, s.view.gridSnap));
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

  const headingReachPx = () => {
    const r = editor.get().project.robot;
    return (Math.hypot(r.bumperLengthMeters, r.bumperWidthMeters) / 2 + 0.2) * view().scale;
  };

  /** The flow's last-step form, beside the spot it was placed at (kept inside the field). */
  const flowFormAt = () => {
    const f = flowState;
    const g = geometryOf(activePath().spec);
    let at: [number, number] | null = null;
    if (f.point) at = [f.point.x, f.point.y];
    else if (g && f.picks.length > 0) {
      const p = g.sample(g.sAtWaypoint(f.picks[f.picks.length - 1]!));
      at = [p.xMeters, p.yMeters];
    }
    if (!at) return null;
    const [cx, cy] = view().toCanvas(at[0], at[1]);
    const { width, height } = sizeRef.current;
    // clear of the spot and of the heading flow's ghost robot and its knob
    const off = f.kind === "rotation" ? 40 + headingReachPx() : 28;
    return <FlowForm left={Math.min(Math.max(8, cx + off), width - 260)} top={Math.min(Math.max(56, cy - 20), height - 240)} />;
  };

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
        onPointerLeave={() => {
          if (dragRef.current) return;
          playback.set({ hoverS: Number.NaN });
          hoverItemRef.current = null;
          deleteHotRef.current = false;
          schedule();
        }}
        onDoubleClick={onDoubleClick}
        onContextMenu={(e) => e.preventDefault()}
      />
      {!flowState.kind && (
        <div className="canvas-hint" data-testid="canvas-hint" aria-live="polite">
          {(tool === "select" && drawing) || TOOL_HINTS[tool]}
        </div>
      )}
      <FlowBar />
      {flowState.kind && flowFormAt()}
      <QuickStartOverlay />
    </div>
  );
}

/** Right-click on a waypoint: its heading, lock and delete (the app's context menu). */
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
  items.push({ label: "Face the next waypoint", icon: <Navigation size={14} />, run: () => faceNext(index) });
  for (const t of faceTargets().slice(0, 6)) items.push({ label: `Face ${t.name}`, icon: <Crosshair size={14} />, run: () => faceToward(index, t.x, t.y, `Face ${t.name}`) });
  if (!isEnd) {
    items.push({
      label: passThrough ? "Hold a heading here" : "Pass through (no heading)",
      icon: <Navigation size={14} />,
      run: () => editPath(passThrough ? "Set heading" : "Pass through", (p) => setHeading(p, index, passThrough ? planned() : Number.NaN)),
    });
  }
  items.push(
    {
      label: path.pinned[index] ? "Unlock" : "Lock in place (the optimiser never moves it)",
      icon: path.pinned[index] ? <LockOpen size={14} /> : <Lock size={14} />,
      run: () => editPath(path.pinned[index] ? "Unlock" : "Lock", (p) => setPinned(p, index, !p.pinned[index])),
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
function dragLabel(drag: Exclude<Drag, { kind: "pan" } | { kind: "insert" } | { kind: "flowKnob" } | { kind: "measure" }>): string {
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

