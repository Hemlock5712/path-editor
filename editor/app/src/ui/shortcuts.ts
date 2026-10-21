// Keyboard shortcuts. Ignored while typing in a field, so Delete and the letters still edit text.

import { cyclePath, deleteSelection, fitView, nudge, stepPlayback, toggleAlliance, togglePlay } from "../state/actions.ts";
import { playback } from "../state/analysis.ts";
import { cancelGesture, editor, inGesture, redo, select, setDraftObstacle, setTool, undo } from "../state/editor.ts";
import { openFolder, save } from "../state/session.ts";
import { finishObstacle } from "./FieldCanvas.tsx";

export const SHORTCUTS: [string, string][] = [
  ["Ctrl+Z / Ctrl+Y (Ctrl+Shift+Z)", "Undo / redo"],
  ["Ctrl+S / Ctrl+O", "Save / open a folder"],
  ["V W R E O", "Select, Draw (waypoints), Heading, Event and Obstacle tools"],
  ["Alt while turning", "Turn freely (headings snap to 15° otherwise)"],
  ["Right-click a waypoint", "Set heading, pass-through, pin, delete"],
  ["Delete", "Delete the selected item"],
  ["Arrows (Shift)", "Nudge the selected waypoint or point 1 cm (10 cm)"],
  ["Space", "Play or pause"],
  [", and .", "Step the playback 0.1 s"],
  ["Home", "Playback to the start"],
  ["[ and ]", "Previous and next path"],
  ["A", "Blue or red view"],
  ["F", "Fit the field"],
  ["Enter / Esc", "Done drawing points; finish / cancel an obstacle; Esc also deselects"],
  ["Shift while dragging", "Snap to 5 cm"],
  ["Double-click a tangent handle", "Back to the automatic tangent"],
  ["Wheel, right or middle drag", "Zoom, pan"],
];

function typing(target: EventTarget | null): boolean {
  if (!(target instanceof HTMLElement)) return false;
  const tag = target.tagName;
  return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT" || target.isContentEditable;
}

export function onKeyDown(e: KeyboardEvent, openHelp: () => void): void {
  const ctrl = e.ctrlKey || e.metaKey;
  const key = e.key.toLowerCase();
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
  if (ctrl || e.altKey) return;
  const step = e.shiftKey ? 0.1 : 0.01;
  switch (e.key) {
    case "Delete":
    case "Backspace":
      e.preventDefault();
      deleteSelection();
      return;
    case "Escape":
      if (inGesture()) cancelGesture();
      else if (editor.get().draftObstacle) setDraftObstacle(null);
      else if (editor.get().tool === "waypoint") setTool("select"); // done drawing
      else if (editor.get().selection) select(null);
      else setTool("select");
      return;
    case "Enter":
      if (editor.get().draftObstacle) finishObstacle();
      else if (editor.get().tool === "waypoint") setTool("select"); // done drawing
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
      openHelp();
      return;
  }
  switch (key) {
    case "v":
      setTool("select");
      return;
    case "w":
      setTool("waypoint");
      return;
    case "e":
      setTool("event");
      return;
    case "r":
      setTool("rotation");
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
