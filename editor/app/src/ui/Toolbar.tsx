// The top bar (file, undo, tools, view, playback), the path tabs, and the status line.

import { useRef } from "react";
import { canOpenFolders } from "../io/files.ts";
import { duplicatePath, fitView, newPath, removeActivePath, stepPlayback, toggleAlliance, togglePlay } from "../state/actions.ts";
import { analysis, playback } from "../state/analysis.ts";
import { isDirty, redo, setActivePath, setTool, setView, undo, type Tool } from "../state/editor.ts";
import { bench, toggleBench } from "../state/bench.ts";
import { downloadZip, newProject, openFolder, openUpload, save, session } from "../state/session.ts";
import { useStore } from "../state/store.ts";
import { formatNumber } from "./controls.tsx";
import { useEditor } from "./panels/common.tsx";

const TOOLS: { tool: Tool; label: string; key: string; hint: string }[] = [
  { tool: "select", label: "Select", key: "V", hint: "Select and drag; drag the empty field to pan" },
  { tool: "waypoint", label: "Waypoint", key: "W", hint: "Click the path to insert a waypoint, elsewhere to add one at the end" },
  { tool: "event", label: "Event", key: "E", hint: "Click the path to place a named event" },
  { tool: "rotation", label: "Rotation", key: "R", hint: "Click the path to add a rotation target" },
  { tool: "obstacle", label: "Obstacle", key: "O", hint: "Click corners; double-click or Enter to finish" },
];

export function Toolbar() {
  const tool = useEditor((s) => s.tool);
  const canUndo = useEditor((s) => s.canUndo);
  const canRedo = useEditor((s) => s.canRedo);
  const undoLabel = useEditor((s) => s.undoLabel);
  const redoLabel = useEditor((s) => s.redoLabel);
  const dirty = useEditor(isDirty);
  const files = useStore(session, (s) => s.files);
  const writable = useStore(session, (s) => s.writable);
  const upload = useRef<HTMLInputElement>(null);
  const benchOpen = useStore(bench, (b) => b.open);

  return (
    <header className="toolbar" role="toolbar" aria-label="Editor">
      <div className="group">
        <span className="brand">Path Editor 5712</span>
        <span className="project-name" data-testid="project-name">
          {files ? files.label : "unsaved project"}
          {dirty && <span aria-label="unsaved changes"> •</span>}
        </span>
      </div>
      <div className="group">
        <button type="button" onClick={newProject} title="New project">
          New
        </button>
        <button type="button" onClick={() => void openFolder()} disabled={!canOpenFolders()} title="Open a robot project folder (Ctrl+O)" data-testid="open-folder">
          Open folder
        </button>
        <button type="button" onClick={() => upload.current?.click()} title="Load a project folder by upload (any browser)">
          Upload
        </button>
        <input
          ref={upload}
          type="file"
          className="sr-only"
          aria-label="Upload a robot project folder"
          // @ts-expect-error webkitdirectory isn't in React's types, but every current browser supports it
          webkitdirectory=""
          multiple
          onChange={(e) => {
            if (e.target.files && e.target.files.length > 0) void openUpload(e.target.files);
            e.target.value = "";
          }}
        />
        <button type="button" onClick={() => void save()} title={writable ? "Save to the folder (Ctrl+S)" : "Save (Ctrl+S)"} data-testid="save" className={dirty ? "primary" : ""}>
          Save
        </button>
        <button type="button" onClick={downloadZip} title="Download every file as a zip">
          Zip
        </button>
      </div>
      <div className="group">
        <button type="button" onClick={undo} disabled={!canUndo} title={undoLabel ? `Undo ${undoLabel} (Ctrl+Z)` : "Undo (Ctrl+Z)"} data-testid="undo">
          Undo
        </button>
        <button type="button" onClick={redo} disabled={!canRedo} title={redoLabel ? `Redo ${redoLabel} (Ctrl+Y)` : "Redo (Ctrl+Y)"} data-testid="redo">
          Redo
        </button>
      </div>
      <div className="group">
        <button
          type="button"
          onClick={toggleBench}
          aria-pressed={benchOpen}
          className={benchOpen ? "tool active" : "tool"}
          title="Compare followers from the bench's results (read-only)"
          data-testid="bench-toggle"
        >
          Bench
        </button>
      </div>
      <div className="group" role="radiogroup" aria-label="Tool">
        {TOOLS.map((t) => (
          <button
            key={t.tool}
            type="button"
            role="radio"
            aria-checked={tool === t.tool}
            className={tool === t.tool ? "tool active" : "tool"}
            title={`${t.hint} (${t.key})`}
            onClick={() => setTool(t.tool)}
            data-testid={`tool-${t.tool}`}
          >
            {t.label} <kbd>{t.key}</kbd>
          </button>
        ))}
      </div>
    </header>
  );
}

export function PathTabs({ onHelp }: { onHelp: () => void }) {
  const paths = useEditor((s) => s.project.paths);
  const activeId = useEditor((s) => s.activePathId);
  const view = useEditor((s) => s.view);
  return (
    <nav className="tabs" aria-label="Paths">
      <div role="tablist" aria-label="Paths">
        {paths.map((p) => (
          <button
            key={p.id}
            type="button"
            role="tab"
            aria-selected={p.id === activeId}
            className={p.id === activeId ? "tab active" : "tab"}
            onClick={() => setActivePath(p.id)}
            style={{ borderTopColor: p.colour }}
            data-testid={`tab-${p.name}`}
          >
            {p.name}
          </button>
        ))}
      </div>
      <button type="button" onClick={newPath} title="New path" aria-label="New path" data-testid="new-path">
        +
      </button>
      <button type="button" onClick={duplicatePath} title="Duplicate this path">
        Duplicate
      </button>
      <button type="button" onClick={removeActivePath} title="Delete this path">
        Delete
      </button>
      <div className="group view-controls">
        <button
          type="button"
          onClick={toggleAlliance}
          className={view.alliance === "red" ? "alliance red" : "alliance blue"}
          aria-pressed={view.alliance === "red"}
          title="Show the paths as they run on the other alliance (A)"
          data-testid="alliance"
        >
          {view.alliance === "red" ? "Red view" : "Blue view"}
        </button>
        <label className="compact" title="Ghost robots along the path">
          Footprints
          <select value={String(view.footprintSpacing)} onChange={(e) => setView({ footprintSpacing: Number(e.target.value) })}>
            <option value="0">off</option>
            <option value="0.5">0.5 m</option>
            <option value="1">1 m</option>
            <option value="2">2 m</option>
          </select>
        </label>
        <button type="button" onClick={fitView} title="Fit the field (F)">
          Fit
        </button>
        <button type="button" onClick={onHelp} title="Keyboard shortcuts (?)" aria-label="Keyboard shortcuts">
          ?
        </button>
      </div>
    </nav>
  );
}

export function PlaybackBar() {
  const playing = useStore(playback, (p) => p.playing);
  const t = useStore(playback, (p) => p.t);
  const result = useStore(analysis, (a) => a.result);
  const total = result && result.ok ? result.totalTimeSeconds : 0;
  return (
    <div className="playback" role="group" aria-label="Playback">
      <button type="button" onClick={togglePlay} disabled={total === 0} aria-label={playing ? "Pause" : "Play"} title="Play or pause (Space)" data-testid="play">
        {playing ? "Pause" : "Play"}
      </button>
      <button type="button" onClick={() => stepPlayback(-0.1)} disabled={total === 0} title="Back 0.1 s (,)" aria-label="Back a tenth of a second">
        −0.1 s
      </button>
      <button type="button" onClick={() => stepPlayback(0.1)} disabled={total === 0} title="Forward 0.1 s (.)" aria-label="Forward a tenth of a second">
        +0.1 s
      </button>
      <label className="scrub">
        <span className="sr-only">Playback time</span>
        <input
          type="range"
          min={0}
          max={total || 1}
          step={0.01}
          value={Math.min(t, total)}
          disabled={total === 0}
          onChange={(e) => playback.set({ t: Number(e.target.value), playing: false })}
        />
      </label>
      <span className="time" data-testid="time">
        {formatNumber(Math.min(t, total), 2)} / <strong>{formatNumber(total, 2)} s</strong>
      </span>
    </div>
  );
}

export function StatusLine() {
  const status = useEditor((s) => s.status);
  const tool = useEditor((s) => s.tool);
  const draft = useEditor((s) => s.draftObstacle);
  const hint = TOOLS.find((t) => t.tool === tool)?.hint ?? "";
  return (
    <footer className="statusline">
      <span role="status" aria-live="polite" className={status?.kind === "error" ? "status error" : "status"} data-testid="status">
        {status?.text ?? ""}
      </span>
      <span className="hint">
        {draft ? `${draft.length} corners: double-click or Enter to finish, Esc to cancel` : hint}
      </span>
    </footer>
  );
}
