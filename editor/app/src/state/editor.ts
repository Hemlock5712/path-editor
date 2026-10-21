// The editor's state and every action that changes it. Document changes go through the history as
// commands; selection, tool and view don't (they aren't edits). A drag previews changes live and
// records one command when it ends.

import { emptyProject, type EditorPath, type Project } from "../model/project.ts";
import { diff, type ProjectCommand } from "./commands.ts";
import { History } from "./history.ts";
import { createStore } from "./store.ts";

export type Tool = "select" | "waypoint" | "event" | "rotation" | "obstacle";

export type Selection =
  | { readonly kind: "waypoint"; readonly index: number }
  | { readonly kind: "rotation"; readonly index: number }
  | { readonly kind: "event"; readonly index: number }
  | { readonly kind: "face"; readonly index: number }
  | { readonly kind: "constraint"; readonly index: number }
  | { readonly kind: "obstacle"; readonly index: number }
  | { readonly kind: "point"; readonly id: string };

export interface ViewState {
  /** Which alliance the field is shown for. Paths are always stored for blue. */
  readonly alliance: "blue" | "red";
  /** 1 = the whole field fits the canvas. */
  readonly zoom: number;
  /** The field point at the canvas centre, metres (displayed frame). */
  readonly centerX: number;
  readonly centerY: number;
  /** Ghost robot footprints along the path every this many metres; 0 = off. */
  readonly footprintSpacing: number;
  /** Show the selected waypoint's tangent handle, to shape the curve by hand (else travel is automatic). */
  readonly shapeCurve: boolean;
  /** Snap new and dragged waypoints to a 0.25 m grid. */
  readonly gridSnap: boolean;
}

export interface Status {
  readonly text: string;
  readonly kind: "info" | "error";
  readonly at: number;
}

export interface EditorState {
  readonly project: Project;
  readonly activePathId: string;
  readonly selection: Selection | null;
  readonly tool: Tool;
  readonly view: ViewState;
  readonly draftObstacle: readonly (readonly [number, number])[] | null;
  readonly status: Status | null;
  /** The project as last loaded or saved, for the unsaved-changes marker. */
  readonly savedProject: Project;
  readonly canUndo: boolean;
  readonly canRedo: boolean;
  readonly undoLabel: string | null;
  readonly redoLabel: string | null;
}

export const DEFAULT_VIEW: ViewState = {
  alliance: "blue",
  zoom: 1,
  centerX: 0,
  centerY: 0,
  footprintSpacing: 0,
  shapeCurve: false,
  gridSnap: false,
};

const history = new History<Project>();
let gestureBase: Project | null = null;

function initialState(project: Project): EditorState {
  return {
    project,
    activePathId: project.paths[0]!.id,
    selection: null,
    tool: "select",
    view: DEFAULT_VIEW,
    draftObstacle: null,
    status: null,
    savedProject: project,
    canUndo: false,
    canRedo: false,
    undoLabel: null,
    redoLabel: null,
  };
}

export const editor = createStore<EditorState>(initialState(emptyProject()));

function historyFlags() {
  return {
    canUndo: history.canUndo,
    canRedo: history.canRedo,
    undoLabel: history.undoLabel,
    redoLabel: history.redoLabel,
  };
}

/** Keeps the active path and the selection pointing at things that exist. */
function settle(project: Project, activePathId: string, selection: Selection | null) {
  const active = project.paths.find((p) => p.id === activePathId) ?? project.paths[0]!;
  return { project, activePathId: active.id, selection: validSelection(project, active, selection) };
}

function validSelection(project: Project, path: EditorPath, sel: Selection | null): Selection | null {
  if (!sel) return null;
  const spec = path.spec;
  switch (sel.kind) {
    case "waypoint":
      return sel.index < spec.waypoints.length ? sel : null;
    case "rotation":
      return sel.index < spec.rotationTargets.length ? sel : null;
    case "event":
      return sel.index < spec.events.length ? sel : null;
    case "face":
      return sel.index < spec.facePointZones.length ? sel : null;
    case "constraint":
      return sel.index < spec.constraintZones.length ? sel : null;
    case "obstacle":
      return sel.index < project.field.obstacles.length ? sel : null;
    case "point":
      return project.namedPoints.some((p) => p.id === sel.id) ? sel : null;
  }
}

// ------------------------------------------------------------------------------ reading

export function activePath(s: EditorState = editor.get()): EditorPath {
  return s.project.paths.find((p) => p.id === s.activePathId) ?? s.project.paths[0]!;
}

export function isDirty(s: EditorState): boolean {
  return s.project !== s.savedProject;
}

// ------------------------------------------------------------------------------ edits

export function notify(text: string, kind: Status["kind"] = "info"): void {
  editor.set({ status: { text, kind, at: Date.now() } });
}

/** Runs a command and records it. */
export function execute(command: ProjectCommand, selection?: Selection | null): void {
  const s = editor.get();
  const project = history.execute(s.project, command);
  editor.set({ ...settle(project, s.activePathId, selection === undefined ? s.selection : selection), ...historyFlags() });
}

/**
 * Applies fn to the project as one undo step. A bad result (the core's checks throw RangeError) is
 * reported in the status bar and nothing changes. Returns whether it applied.
 */
export function editProject(label: string, fn: (p: Project) => Project, selection?: Selection | null): boolean {
  const before = editor.get().project;
  let after: Project;
  try {
    after = fn(before);
  } catch (e) {
    if (e instanceof RangeError) {
      notify(e.message, "error");
      return false;
    }
    throw e;
  }
  const command = diff(label, before, after);
  if (command) execute(command, selection);
  else if (selection !== undefined) select(selection);
  return true;
}

/** The same for the active path. */
export function editPath(label: string, fn: (p: EditorPath) => EditorPath, selection?: Selection | null): boolean {
  const id = editor.get().activePathId;
  return editProject(label, (project) => ({ ...project, paths: project.paths.map((p) => (p.id === id ? fn(p) : p)) }), selection);
}

export function undo(): void {
  if (gestureBase) cancelGesture();
  const s = editor.get();
  editor.set({ ...settle(history.undo(s.project), s.activePathId, s.selection), ...historyFlags() });
}

export function redo(): void {
  const s = editor.get();
  editor.set({ ...settle(history.redo(s.project), s.activePathId, s.selection), ...historyFlags() });
}

// ------------------------------------------------------------------------------ gestures (drags)

export function beginGesture(): void {
  gestureBase = editor.get().project;
}

/** Shows a change while dragging, without recording it. Bad intermediate states are skipped. */
export function previewProject(fn: (base: Project, current: Project) => Project): void {
  if (!gestureBase) return;
  try {
    const project = fn(gestureBase, editor.get().project);
    const s = editor.get();
    editor.set(settle(project, s.activePathId, s.selection));
  } catch (e) {
    if (!(e instanceof RangeError)) throw e;
  }
}

export function previewPath(fn: (base: EditorPath) => EditorPath): void {
  const id = editor.get().activePathId;
  previewProject((base) => ({ ...base, paths: base.paths.map((p) => (p.id === id ? fn(p) : p)) }));
}

/** Ends a drag: whatever changed since beginGesture becomes one undo step. */
export function endGesture(label: string): void {
  if (!gestureBase) return;
  const command = diff(label, gestureBase, editor.get().project);
  gestureBase = null;
  if (command) {
    history.record(command);
    editor.set(historyFlags());
  }
}

export function cancelGesture(): void {
  if (!gestureBase) return;
  const s = editor.get();
  editor.set(settle(gestureBase, s.activePathId, s.selection));
  gestureBase = null;
}

export function inGesture(): boolean {
  return gestureBase !== null;
}

// ------------------------------------------------------------------------------ not edits

export function select(selection: Selection | null): void {
  const s = editor.get();
  editor.set({ selection: validSelection(s.project, activePath(s), selection) });
}

export function setTool(tool: Tool): void {
  editor.set({ tool, draftObstacle: tool === "obstacle" ? editor.get().draftObstacle : null });
}

export function setActivePath(id: string): void {
  const s = editor.get();
  if (s.project.paths.some((p) => p.id === id)) editor.set({ activePathId: id, selection: null });
}

export function setView(view: Partial<ViewState>): void {
  editor.set((s) => ({ view: { ...s.view, ...view } }));
}

export function setDraftObstacle(draft: EditorState["draftObstacle"]): void {
  editor.set({ draftObstacle: draft });
}

/** A freshly loaded project: new history, nothing unsaved. */
export function loadProject(project: Project, view?: Partial<ViewState>): void {
  history.clear();
  gestureBase = null;
  editor.set({ ...initialState(project), view: { ...DEFAULT_VIEW, ...view }, tool: editor.get().tool });
}

export function markSaved(): void {
  editor.set((s) => ({ savedProject: s.project }));
}
