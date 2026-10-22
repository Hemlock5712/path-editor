// Every app command in one list: the command palette (Ctrl+K) searches it, and menus and tooltips
// take their labels and shortcuts from it.

import { canOpenFolders } from "../io/files.ts";
import { cyclePath, duplicatePath, fitView, newPath, removeActivePath, toggleAlliance, togglePlay, zoomBy } from "../state/actions.ts";
import { editor, redo, setTool, setView, undo, type Tool } from "../state/editor.ts";
import { startOptimise, startRouteAround } from "../state/optimiser.ts";
import { startFlow, type FlowKind } from "../state/flow.ts";
import { downloadZip, newProject, openFolder, save } from "../state/session.ts";
import { openSettings, setPage, ui } from "../state/ui.ts";

export interface Command {
  readonly id: string;
  readonly label: string;
  readonly group: string;
  readonly shortcut?: string;
  readonly enabled?: () => boolean;
  readonly run: () => void;
}

export const TOOLS: readonly { readonly tool: Tool; readonly label: string; readonly key: string }[] = [
  { tool: "select", label: "Select", key: "V" },
  { tool: "obstacle", label: "Obstacle", key: "O" },
  { tool: "measure", label: "Measure lengths", key: "L" },
];

/** The guided flows, on the toolbar between Select and Obstacle. */
export const FLOWS: readonly { readonly flow: FlowKind; readonly label: string; readonly key: string; readonly hint: string }[] = [
  { flow: "rotation", label: "Heading along the path", key: "R", hint: "Click the path, then turn the robot there" },
  { flow: "event", label: "Event marker", key: "M", hint: "Click the path, then name the event" },
  { flow: "constraint", label: "Constraint zone (speed limit)", key: "Z", hint: "Click the zone's start and end on the path, then set its speed limit" },
  { flow: "face", label: "Point-towards zone", key: "T", hint: "Click the zone's start and end on the path, then the point to face" },
  { flow: "point", label: "Named point", key: "N", hint: "Click the spot, then name it" },
];

export const COMMANDS: readonly Command[] = [
  { id: "save", label: "Save", group: "File", shortcut: "Ctrl+S", run: () => void save() },
  { id: "open", label: "Open a robot project folder…", group: "File", shortcut: "Ctrl+O", enabled: canOpenFolders, run: () => void openFolder() },
  { id: "zip", label: "Download the project as a zip", group: "File", run: downloadZip },
  { id: "new-project", label: "New project", group: "File", run: newProject },
  { id: "undo", label: "Undo", group: "Edit", shortcut: "Ctrl+Z", enabled: () => editor.get().canUndo, run: undo },
  { id: "redo", label: "Redo", group: "Edit", shortcut: "Ctrl+Y", enabled: () => editor.get().canRedo, run: redo },
  ...TOOLS.map((t) => ({ id: `tool-${t.tool}`, label: `Tool: ${t.label}`, group: "Tools", shortcut: t.key, run: () => setTool(t.tool) })),
  { id: "new-path", label: "New path (from this one's end)", group: "Path", run: () => newPath() },
  { id: "duplicate-path", label: "Duplicate this path", group: "Path", run: duplicatePath },
  { id: "delete-path", label: "Delete this path", group: "Path", run: removeActivePath },
  { id: "prev-path", label: "Previous path", group: "Path", shortcut: "[", run: () => cyclePath(-1) },
  { id: "next-path", label: "Next path", group: "Path", shortcut: "]", run: () => cyclePath(1) },
  { id: "optimise", label: "Optimise path", group: "Path", run: () => startOptimise() },
  { id: "route-around", label: "Route around obstacles", group: "Path", run: startRouteAround },
  { id: "transform", label: "Flip or mirror path…", group: "Path", run: () => ui.set({ transformOpen: true }) },
  { id: "flow-constraint", label: "Add a constraint zone (speed limit)…", group: "Add", shortcut: "Z", run: () => startFlow("constraint") },
  { id: "flow-rotation", label: "Add a heading along the path…", group: "Add", shortcut: "R", run: () => startFlow("rotation") },
  { id: "flow-event", label: "Add an event marker…", group: "Add", shortcut: "M", run: () => startFlow("event") },
  { id: "flow-face", label: "Add a point-towards zone…", group: "Add", shortcut: "T", run: () => startFlow("face") },
  { id: "flow-point", label: "Add a named point…", group: "Add", shortcut: "N", run: () => startFlow("point") },
  { id: "play", label: "Play or pause", group: "View", shortcut: "Space", run: togglePlay },
  { id: "alliance", label: "Blue or red view", group: "View", shortcut: "A", run: toggleAlliance },
  { id: "fit", label: "Fit the field", group: "View", shortcut: "F", run: fitView },
  { id: "zoom-in", label: "Zoom in", group: "View", shortcut: "Ctrl+=", run: () => zoomBy(1.25) },
  { id: "zoom-out", label: "Zoom out", group: "View", shortcut: "Ctrl+-", run: () => zoomBy(0.8) },
  { id: "grid", label: "Snap to the grid on or off", group: "View", run: () => setView({ gridSnap: !editor.get().view.gridSnap }) },
  { id: "charts", label: "Show or hide the charts", group: "View", run: () => ui.set({ chartsOpen: !ui.get().chartsOpen }) },
  { id: "page-editor", label: "Go to the editor", group: "Go", run: () => setPage("editor") },
  { id: "page-telemetry", label: "Go to telemetry (live robot)", group: "Go", run: () => setPage("telemetry") },
  { id: "page-bench", label: "Go to the bench comparison", group: "Go", run: () => setPage("bench") },
  { id: "settings", label: "Settings", group: "Go", shortcut: "Ctrl+,", run: () => openSettings() },
  { id: "settings-robot", label: "Settings: robot", group: "Go", run: () => openSettings("robot") },
  { id: "settings-field", label: "Settings: field and obstacles", group: "Go", run: () => openSettings("field") },
  { id: "help", label: "Quick start and keyboard shortcuts", group: "Help", shortcut: "?", run: () => ui.set({ helpOpen: true }) },
];

export function commandById(id: string): Command | undefined {
  return COMMANDS.find((c) => c.id === id);
}
