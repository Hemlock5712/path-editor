// The centre of the editor: the field, with the tool bar and the view bar floating on it, the playback
// bar along its bottom, and the charts in a drawer below (closed until asked for).

import {
  ChartSpline,
  Check,
  ChevronRight,
  CircleAlert,
  CircleCheck,
  Flag,
  Footprints,
  Grid3x3,
  Hexagon,
  Maximize2,
  MousePointer2,
  Pause,
  Play,
  Radio,
  Redo2,
  RotateCw,
  Spline,
  StepBack,
  StepForward,
  TriangleAlert,
  Undo2,
} from "lucide-react";
import { useRef, useState } from "react";
import { isDrawn } from "../../model/project.ts";
import { fitView, stepPlayback, toggleAlliance, togglePlay } from "../../state/actions.ts";
import { analysis, playback } from "../../state/analysis.ts";
import { redo, setTool, setView, undo, type Tool } from "../../state/editor.ts";
import { live } from "../../state/live.ts";
import { useStore } from "../../state/store.ts";
import { openContextMenu, ui } from "../../state/ui.ts";
import { SimResultRow, SimSwitch } from "../../sim/SimPanel.tsx";
import { Charts } from "../Charts.tsx";
import { TOOLS } from "../commands.ts";
import { formatNumber, IconButton, Portal, Slider, useDismiss, usePopoverPosition } from "../controls.tsx";
import { atS } from "../draw.ts";
import { FieldCanvas, TOOL_HINTS } from "../FieldCanvas.tsx";
import { useActivePath, useAnalysis, useEditor } from "../panels/common.tsx";

const TOOL_ICONS: Record<Tool, typeof Spline> = {
  select: MousePointer2,
  rotation: RotateCw,
  event: Flag,
  obstacle: Hexagon,
};

const KIND_LABELS = {
  curvature: "Bend",
  heading: "Turn",
  bumper: "Bumpers",
  event: "Event",
  position: "Placement",
  speed: "Speed",
};

function ToolBar() {
  const tool = useEditor((s) => s.tool);
  const canUndo = useEditor((s) => s.canUndo);
  const canRedo = useEditor((s) => s.canRedo);
  const undoLabel = useEditor((s) => s.undoLabel);
  const redoLabel = useEditor((s) => s.redoLabel);
  const path = useActivePath();
  return (
    <div className="float-bar canvas-tools" role="toolbar" aria-label="Tools">
      <div role="radiogroup" aria-label="Tool" className="row" style={{ gap: 2 }}>
        {TOOLS.map((t) => {
          const Icon = TOOL_ICONS[t.tool];
          return (
            <button
              key={t.tool}
              type="button"
              role="radio"
              aria-checked={tool === t.tool}
              aria-label={t.label}
              aria-keyshortcuts={t.key}
              className="icon-btn"
              data-tip={`${t.label}: ${TOOL_HINTS[t.tool]}|${t.key}`}
              onClick={() => setTool(t.tool)}
              data-testid={`tool-${t.tool}`}
            >
              <Icon size={16} />
            </button>
          );
        })}
      </div>
      <span className="sep" />
      <IconButton label={undoLabel ? `Undo ${undoLabel}` : "Undo"} shortcut="Ctrl+Z" onClick={undo} disabled={!canUndo} testId="undo">
        <Undo2 size={16} />
      </IconButton>
      <IconButton label={redoLabel ? `Redo ${redoLabel}` : "Redo"} shortcut="Ctrl+Y" onClick={redo} disabled={!canRedo} testId="redo">
        <Redo2 size={16} />
      </IconButton>
      <span className="sep" />
      <span className="path-title" data-testid="path-title" data-tip="The path being edited (rename it in the Paths rail)">
        {path.name}
      </span>
      <PathStats />
    </div>
  );
}

/** The plan's headline numbers: length, time, top speed. */
function PathStats() {
  const a = useAnalysis();
  const drawn = isDrawn(useActivePath().spec);
  return (
    <span className="path-stats" data-testid="path-stats">
      {!drawn ? (
        "no path yet"
      ) : a ? (
        <>
          {formatNumber(a.lengthMeters, 2)} m, <strong>{formatNumber(a.totalTimeSeconds, 2)} s</strong>, top {formatNumber(Math.max(...a.stations.v), 2)} m/s
        </>
      ) : (
        "planning…"
      )}
    </span>
  );
}

function ChecksBadge() {
  const a = useAnalysis();
  const failure = useStore(analysis, (s) => (s.result && !s.result.ok ? s.result.error : null));
  const [open, setOpen] = useState(false);
  const button = useRef<HTMLButtonElement>(null);
  const pop = useRef<HTMLDivElement>(null);
  const style = usePopoverPosition(button, open, 360, true);
  useDismiss(open, () => setOpen(false), button, pop);
  const drawn = isDrawn(useActivePath().spec);
  const checks = a?.checks ?? [];
  const errors = checks.filter((c) => c.severity === "error").length;
  const level = drawn && (failure || errors > 0) ? "error" : drawn && checks.length > 0 ? "warning" : "ok";
  const Icon = level === "error" ? CircleAlert : level === "warning" ? TriangleAlert : CircleCheck;
  const text = !drawn ? "No path yet" : failure ? "Can't plan" : checks.length === 0 ? "No issues" : `${checks.length} ${checks.length === 1 ? "issue" : "issues"}`;
  return (
    <>
      <button
        ref={button}
        type="button"
        className={`check-badge ${level}`}
        aria-expanded={open}
        aria-haspopup="dialog"
        onClick={() => setOpen(!open)}
        data-testid="checks-badge"
        data-tip="Checks on this path"
      >
        <Icon size={15} />
        {text}
      </button>
      {open && (
        <Portal>
          <div ref={pop} className="popover checks-popover" style={style} role="dialog" aria-label="Checks">
            {!drawn && <p className="hint">Double-click the field to place the start and the end.</p>}
            {drawn && failure && (
              <p className="check error" role="alert">
                The path can't be planned: {failure}
              </p>
            )}
            {a && checks.length === 0 && <p className="hint">No problems found: the plan keeps to the robot's limits and clear of obstacles.</p>}
            <ul className="checks" aria-label="Checks" data-testid="checks">
              {checks.map((c, i) => (
                <li key={i}>
                  <button
                    type="button"
                    className={`check ${c.severity}`}
                    onClick={() => a && playback.set({ t: atS(a.stations, a.stations.t, c.sFrom), playing: false })}
                  >
                    <span className="check-kind">{KIND_LABELS[c.kind]}</span> {c.message}
                  </button>
                </li>
              ))}
            </ul>
            {errors > 0 && (
              <p className="hint">{errors === 1 ? "1 error" : `${errors} errors`}: the robot would run this path, but it won't do what the plan shows.</p>
            )}
          </div>
        </Portal>
      )}
    </>
  );
}

const SPACINGS = [
  { value: 0, label: "Waypoints only" },
  { value: 0.5, label: "Every 0.5 m" },
  { value: 1, label: "Every 1 m" },
  { value: 2, label: "Every 2 m" },
];

function ViewBar() {
  const view = useEditor((s) => s.view);
  const overlay = useStore(ui, (s) => s.liveOverlay);
  const connected = useStore(live, (s) => s.status === "connected");
  const footprintButton = useRef<HTMLButtonElement>(null);
  return (
    <div className="float-bar canvas-view" role="toolbar" aria-label="View">
      <ChecksBadge />
      <span className="sep" />
      <button
        type="button"
        className={view.alliance === "red" ? "alliance-btn red" : "alliance-btn"}
        aria-pressed={view.alliance === "red"}
        onClick={toggleAlliance}
        data-tip="Show the paths as they run for the other alliance|A"
        data-testid="alliance"
      >
        <span className="dot" />
        {view.alliance === "red" ? "Red view" : "Blue view"}
      </button>
      <IconButton
        label="Snap to a 25 cm grid"
        pressed={view.gridSnap}
        onClick={() => setView({ gridSnap: !view.gridSnap })}
        testId="grid-snap"
      >
        <Grid3x3 size={16} />
      </IconButton>
      <button
        ref={footprintButton}
        type="button"
        className="icon-btn"
        aria-label="Robot footprints along the path"
        aria-pressed={view.footprintSpacing > 0}
        aria-haspopup="menu"
        data-tip="Robot footprints along the path"
        data-testid="footprints"
        onClick={() => {
          const r = footprintButton.current!.getBoundingClientRect();
          openContextMenu(
            r.left,
            r.bottom + 4,
            "Footprints",
            SPACINGS.map((o) => ({
              label: o.label,
              icon: o.value === view.footprintSpacing ? <Check size={14} /> : <span style={{ width: 14 }} />,
              run: () => setView({ footprintSpacing: o.value }),
            })),
          );
        }}
      >
        <Footprints size={16} />
      </button>
      {connected && (
        <IconButton label="Show the live robot on the field" pressed={overlay} onClick={() => ui.set({ liveOverlay: !overlay })} testId="live-overlay">
          <Radio size={16} />
        </IconButton>
      )}
      <IconButton label="Fit the field" shortcut="F" onClick={fitView} testId="fit">
        <Maximize2 size={16} />
      </IconButton>
    </div>
  );
}

function PlaybackBar() {
  const playing = useStore(playback, (p) => p.playing);
  const t = useStore(playback, (p) => p.t);
  const result = useStore(analysis, (a) => a.result);
  const total = result && result.ok ? result.totalTimeSeconds : 0;
  return (
    <div className="playback-overlay">
      <SimResultRow />
      <div className="playback-bar" role="group" aria-label="Playback">
      <IconButton label={playing ? "Pause" : "Play"} shortcut="Space" onClick={togglePlay} disabled={total === 0} testId="play">
        {playing ? <Pause size={16} /> : <Play size={16} />}
      </IconButton>
      <IconButton label="Back a tenth of a second" shortcut="," onClick={() => stepPlayback(-0.1)} disabled={total === 0}>
        <StepBack size={15} />
      </IconButton>
      <IconButton label="Forward a tenth of a second" shortcut="." onClick={() => stepPlayback(0.1)} disabled={total === 0}>
        <StepForward size={15} />
      </IconButton>
      <Slider
        label="Playback time"
        value={Math.min(t, total)}
        min={0}
        max={total || 1}
        step={0.01}
        disabled={total === 0}
        valueText={`${formatNumber(Math.min(t, total), 2)} s`}
        onChange={(v) => playback.set({ t: v, playing: false })}
        testId="scrub"
      />
      <span className="time" data-testid="time">
        {formatNumber(Math.min(t, total), 2)} / <strong>{formatNumber(total, 2)} s</strong>
      </span>
      <span className="sep" />
      <SimSwitch />
      </div>
    </div>
  );
}

function ChartsDrawer() {
  const open = useStore(ui, (s) => s.chartsOpen);
  return (
    <div className="drawer">
      <button type="button" className="drawer-head" aria-expanded={open} onClick={() => ui.set({ chartsOpen: !open })} data-testid="charts-toggle">
        <ChevronRight className="chev" size={14} />
        <ChartSpline size={14} />
        Charts
      </button>
      <div className={open ? "collapse open" : "collapse"}>
        <div className="collapse-inner">{open && <Charts />}</div>
      </div>
    </div>
  );
}

export function Stage() {
  return (
    <section className="stage" aria-label="Field">
      <div className="canvas-area">
        <FieldCanvas />
        <ToolBar />
        <ViewBar />
        <PlaybackBar />
      </div>
      <ChartsDrawer />
    </section>
  );
}
