// Keyboard shortcuts. Ignored while typing in a field, so Delete and the letters still edit text.

import { cyclePath, deleteSelection, fitView, nudge, stepPlayback, toggleAlliance, togglePlay } from "../state/actions.ts";
import { playback } from "../state/analysis.ts";
import { cancelGesture, editor, inGesture, redo, select, setDraftObstacle, setTool, undo } from "../state/editor.ts";
import { openFolder, save } from "../state/session.ts";
import { openSettings, ui } from "../state/ui.ts";
import { finishObstacle } from "./FieldCanvas.tsx";
import { cancelFlow, flow, startFlow } from "../state/flow.ts";
import { copySelection, cycleWaypoint, paste, rotateSelection } from "../state/helpers.ts";

const DEG = Math.PI / 180;

export const SHORTCUTS: [string, string][] = [
  ["Ctrl+K", "Command palette: every command, by name"],
  ["Ctrl+Z / Ctrl+Y (Ctrl+Shift+Z)", "Undo / redo"],
  ["Ctrl+S / Ctrl+O", "Save / open a folder"],
  ["Double-click the field", "Add a waypoint after the selected one (an empty path: the start, then the end)"],
  ["V O", "Select and Obstacle tools"],
  ["R M Z T N", "Add a heading, an event marker, a constraint zone, a point-towards zone or a named point, step by step (Esc cancels)"],
  ["Alt while turning", "Turn freely (headings snap to 15° otherwise)"],
  ["Right-click a waypoint", "Set heading, pass-through, lock, delete"],
  ["Delete", "Delete the selected item"],
  ["Arrows (Shift)", "Nudge the selected waypoint or point 1 cm (5 cm)"],
  ["Q / E", "Turn the selected waypoint, heading or named point 5° left / right"],
  ["Tab / Shift+Tab", "Select the next / previous waypoint"],
  ["L", "Measure lengths: drag between two points (they snap to waypoints and named points)"],
  ["Ctrl+C / Ctrl+V", "Copy the selected waypoint (paste puts it after the selected one), or with nothing selected the path (paste makes a new path)"],
  ["Space", "Play or pause"],
  [", and .", "Step the playback 0.1 s"],
  ["Home", "Playback to the start"],
  ["[ and ]", "Previous and next path"],
  ["A", "Blue or red view"],
  ["F", "Fit the field"],
  ["Enter / Esc", "Finish / cancel an obstacle; Esc also deselects"],
  ["Shift while dragging", "Snap to 5 cm"],
  ["Double-click a tangent handle", "Back to the automatic tangent"],
  ["Wheel, right or middle drag", "Zoom, pan"],
  ["Ctrl + / Ctrl − / Ctrl+0", "Zoom the field in, out, fit"],
  ["Drag a field's label", "Scrub the number (Shift: finer); arrow keys step it"],
  ["Ctrl+,", "Settings"],
];

function typing(target: EventTarget | null): boolean {
  if (!(target instanceof HTMLElement)) return false;
  const tag = target.tagName;
  return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT" || target.isContentEditable;
}

/** A focused control that Space and Enter press (a button, switch, slider, option). */
function pressable(target: EventTarget | null): boolean {
  return target instanceof HTMLElement && (target.tagName === "BUTTON" || target.getAttribute("role") === "slider" || target.getAttribute("role") === "option");
}

export function onKeyDown(e: KeyboardEvent): void {
  const ctrl = e.ctrlKey || e.metaKey;
  const key = e.key.toLowerCase();
  if (ctrl && key === "k") {
    e.preventDefault();
    ui.set({ paletteOpen: !ui.get().paletteOpen });
    return;
  }
  if (ctrl && e.key === ",") {
    e.preventDefault();
    openSettings();
    return;
  }
  if (ui.get().paletteOpen || ui.get().helpOpen) return;
  if (ctrl && key === "s") {
    e.preventDefault();
    void save();
    return;
  }
  if (ctrl && key === "o") {
    e.preventDefault();
    void openFolder();
    return;
  }
  if (typing(e.target)) return;
  if (ctrl && (key === "y" || (key === "z" && e.shiftKey))) {
    e.preventDefault();
    redo();
    return;
  }
  if (ctrl && key === "z") {
    e.preventDefault();
    undo();
    return;
  }
  if (ctrl && key === "c" && ui.get().page === "editor" && !window.getSelection()?.toString()) {
    e.preventDefault();
    copySelection();
    return;
  }
  if (ctrl && key === "v" && ui.get().page === "editor") {
    e.preventDefault();
    void paste();
    return;
  }
  if (ctrl || e.altKey) return;
  if ((e.key === " " || e.key === "Enter") && pressable(e.target)) return;
  // the editing keys belong to the editor page
  if (ui.get().page !== "editor") return;
  const step = e.shiftKey ? 0.05 : 0.01;
  switch (e.key) {
    case "Delete":
    case "Backspace":
      e.preventDefault();
      deleteSelection();
      return;
    case "Escape":
      if (inGesture()) cancelGesture();
      else if (flow.get().kind) cancelFlow();
      else if (editor.get().draftObstacle) setDraftObstacle(null);
      else if (editor.get().selection) select(null);
      else setTool("select");
      return;
    case "Enter":
      if (editor.get().draftObstacle) finishObstacle();
      return;
    case "ArrowLeft":
    case "ArrowRight":
    case "ArrowUp":
    case "ArrowDown": {
      const s = editor.get().selection;
      if (s?.kind !== "waypoint" && s?.kind !== "point") return;
      e.preventDefault();
      const dx = e.key === "ArrowLeft" ? -step : e.key === "ArrowRight" ? step : 0;
      const dy = e.key === "ArrowDown" ? -step : e.key === "ArrowUp" ? step : 0;
      nudge(dx, dy);
      return;
    }
    case " ":
      e.preventDefault();
      togglePlay();
      return;
    case ",":
      stepPlayback(-0.1);
      return;
    case ".":
      stepPlayback(0.1);
      return;
    case "Home":
      playback.set({ t: 0, playing: false });
      return;
    case "[":
      cyclePath(-1);
      return;
    case "]":
      cyclePath(1);
      return;
    case "?":
      ui.set({ helpOpen: true });
      return;
    case "Tab":
      if (flow.get().kind) return;
      e.preventDefault();
      cycleWaypoint(e.shiftKey ? -1 : 1);
      return;
  }
  switch (key) {
    case "v":
      setTool("select");
      return;
    case "q":
      rotateSelection(5 * DEG);
      return;
    case "e":
      rotateSelection(-5 * DEG);
      return;
    case "m":
      startFlow("event");
      return;
    case "l":
      setTool("measure");
      return;
    case "r":
      startFlow("rotation");
      return;
    case "z":
      startFlow("constraint");
      return;
    case "t":
      startFlow("face");
      return;
    case "n":
      startFlow("point");
      return;
    case "o":
      setTool("obstacle");
      return;
    case "a":
      toggleAlliance();
      return;
    case "f":
      fitView();
      return;
  }
}
