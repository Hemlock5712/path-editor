// Guided creation: a speed zone, a heading, an event, a point-towards zone or a named point, made in a
// few clicks and one small form, step by step, with a bar over the field saying what comes next. Nothing
// changes until the last step, so Esc cancels cleanly and the whole flow is one undo step. The toolbar,
// the keyboard and the inspector's + buttons all start the same flows.

import { makeConstraintZone, makeFacePointZone, makePathEvent, makeRotationTarget } from "../../../core/src/index.ts";
import { addConstraintZone, addEvent, addFacePointZone, addRotationTarget } from "../model/edits.ts";
import { addNamedPoint } from "../model/namedPoints.ts";
import { round5, round5Radians } from "../model/positions.ts";
import { activePath, editor, editPath, editProject, notify, select, setTool } from "./editor.ts";
import { createStore } from "./store.ts";

export type FlowKind = "constraint" | "rotation" | "event" | "face" | "point";

export interface FlowState {
  readonly kind: FlowKind | null;
  /** 0-based. */
  readonly step: number;
  /** Places picked on the path so far, as waypoint positions (zones: from, to; heading, event: at). */
  readonly picks: readonly number[];
  /** The field point picked (a face target, a named point), and what it snapped to. */
  readonly point: { readonly x: number; readonly y: number; readonly snappedTo: string | null } | null;
  /** The heading flow's angle, radians (the ghost robot's knob turns it). */
  readonly heading: number;
}

const IDLE: FlowState = { kind: null, step: 0, picks: [], point: null, heading: 0 };

export const flow = createStore<FlowState>(IDLE);

export const FLOW_TITLES: Record<FlowKind, string> = {
  constraint: "Constraint zone",
  rotation: "Heading",
  event: "Event",
  face: "Point-towards zone",
  point: "Named point",
};

/** Each step's instruction. */
export const FLOW_STEPS: Record<FlowKind, readonly string[]> = {
  constraint: ["Click where the zone starts on the path", "Click where the zone ends", "Set its speed limit"],
  rotation: ["Click the spot on the path", "Turn the robot: drag its knob, or type an angle"],
  event: ["Click the spot on the path", "Name the event"],
  face: ["Click where the zone starts on the path", "Click where the zone ends", "Click the point to face"],
  point: ["Click where the point goes", "Name it (and give it a heading, if it has one)"],
};

/** Which steps take a click on the path, on the field, or a form. */
export function stepTakes(kind: FlowKind, step: number): "path" | "field" | "form" {
  if (kind === "point") return step === 0 ? "field" : "form";
  if (kind === "face") return step < 2 ? "path" : "field";
  if (kind === "constraint") return step < 2 ? "path" : "form";
  return step === 0 ? "path" : "form";
}

export function startFlow(kind: FlowKind): void {
  if (kind !== "point" && activePath().spec.waypoints.length < 2) {
    notify("Draw the path first: double-click the field for its start and end.");
    return;
  }
  setTool("select");
  select(null);
  flow.set({ ...IDLE, kind });
}

export function cancelFlow(): void {
  if (flow.get().kind) flow.set(IDLE);
}

/** A click on the path at waypoint position wp; heading is the plan's heading there (the heading flow starts from it). */
export function pickOnPath(wp: number, heading: number): void {
  const f = flow.get();
  if (!f.kind) return;
  const at = round5(wp);
  if (f.kind === "rotation" || f.kind === "event") {
    flow.set({ step: 1, picks: [at], heading: round5Radians(Math.round(heading / (Math.PI / 12)) * (Math.PI / 12)) });
    return;
  }
  // zones: the start, then the end (either way round)
  if (f.step === 0) flow.set({ step: 1, picks: [at] });
  else if (Math.abs(at - f.picks[0]!) < 1e-3) notify("Click a different spot for the end of the zone.");
  else flow.set({ step: 2, picks: [Math.min(f.picks[0]!, at), Math.max(f.picks[0]!, at)] });
}

/** A click on the field (a face zone's target, or where a named point goes). */
export function pickPoint(x: number, y: number, snappedTo: string | null): void {
  const f = flow.get();
  if (f.kind === "face" && f.step === 2) {
    const [from, to] = f.picks as [number, number];
    const point = { x: round5(x), y: round5(y) };
    const index = activePath().spec.facePointZones.length;
    if (editPath("Add point-towards zone", (p) => addFacePointZone(p, makeFacePointZone(from, to, point, 1.0)), { kind: "face", index })) {
      flow.set(IDLE);
    }
  } else if (f.kind === "point" && f.step === 0) {
    flow.set({ step: 1, point: { x: round5(x), y: round5(y), snappedTo } });
  }
}

export function setFlowHeading(radians: number): void {
  flow.set({ heading: radians });
}

export function finishConstraint(maxMetersPerSecond: number, maxMetersPerSecondSquared: number): boolean {
  const f = flow.get();
  if (f.kind !== "constraint" || f.step !== 2) return false;
  const [from, to] = f.picks as [number, number];
  const index = activePath().spec.constraintZones.length;
  const ok = editPath(
    "Add constraint zone",
    (p) => addConstraintZone(p, makeConstraintZone(from, to, maxMetersPerSecond, maxMetersPerSecondSquared)),
    { kind: "constraint", index },
  );
  if (ok) flow.set(IDLE);
  return ok;
}

export function finishRotation(): boolean {
  const f = flow.get();
  if (f.kind !== "rotation" || f.step !== 1) return false;
  const index = activePath().spec.rotationTargets.length;
  const ok = editPath("Add heading", (p) => addRotationTarget(p, makeRotationTarget(f.picks[0]!, Number.NaN, round5Radians(f.heading))), {
    kind: "rotation",
    index,
  });
  if (ok) flow.set(IDLE);
  return ok;
}

export function finishEvent(name: string): boolean {
  const f = flow.get();
  if (f.kind !== "event" || f.step !== 1) return false;
  if (name.trim() === "") {
    notify("An event needs a name.", "error");
    return false;
  }
  const index = activePath().spec.events.length;
  const ok = editPath("Add event", (p) => addEvent(p, makePathEvent(name.trim(), f.picks[0]!, Number.NaN)), { kind: "event", index });
  if (ok) flow.set(IDLE);
  return ok;
}

export function finishPoint(name: string, heading: number): boolean {
  const f = flow.get();
  if (f.kind !== "point" || f.step !== 1 || !f.point) return false;
  const trimmed = name.trim();
  if (trimmed === "") {
    notify("A named point needs a name.", "error");
    return false;
  }
  if (editor.get().project.namedPoints.some((p) => p.name === trimmed)) {
    notify("Another point has that name.", "error");
    return false;
  }
  let id = "";
  const ok = editProject("Add named point", (pr) => {
    const [next, point] = addNamedPoint(pr, trimmed, f.point!.x, f.point!.y, heading);
    id = point.id;
    return next;
  });
  if (ok) {
    flow.set(IDLE);
    select({ kind: "point", id });
  }
  return ok;
}
