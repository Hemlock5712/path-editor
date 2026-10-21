// The editor's layout: toolbar and tabs on top, the field with the side panels, charts and playback
// below. Starts the analysis worker and the playback clock.

import { useEffect, useRef, useState } from "react";
import { analysis, playback, startAnalysis } from "./state/analysis.ts";
import { bench } from "./state/bench.ts";
import { useStore } from "./state/store.ts";
import { BenchView } from "./ui/BenchView.tsx";
import { editor, isDirty } from "./state/editor.ts";
import { Charts } from "./ui/Charts.tsx";
import { FieldCanvas } from "./ui/FieldCanvas.tsx";
import { ChecksPanel } from "./ui/panels/ChecksPanel.tsx";
import { EventsPanel } from "./ui/panels/EventsPanel.tsx";
import { ObstaclesPanel, PointsPanel, ProjectPanel, RobotPanel } from "./ui/panels/FieldPanels.tsx";
import { RotationPanel, ZonesPanel } from "./ui/panels/HeadingPanel.tsx";
import { LivePanel } from "./ui/panels/LivePanel.tsx";
import { OptimisePanel } from "./ui/panels/OptimisePanel.tsx";
import { PathPanel, WaypointsPanel } from "./ui/panels/PathPanel.tsx";
import { onKeyDown, SHORTCUTS } from "./ui/shortcuts.ts";
import { PathTabs, PlaybackBar, StatusLine, Toolbar } from "./ui/Toolbar.tsx";

/** Advances the playback clock while playing. */
function usePlaybackClock(): void {
  useEffect(() => {
    let frame = 0;
    let last = performance.now();
    const tick = (now: number) => {
      const dt = (now - last) / 1000;
      last = now;
      const p = playback.get();
      const result = analysis.get().result;
      if (p.playing && result && result.ok) {
        const t = p.t + dt;
        if (t >= result.totalTimeSeconds) playback.set({ t: result.totalTimeSeconds, playing: false });
        else playback.set({ t });
      }
      frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frame);
  }, []);
}

function HelpDialog({ open, onClose }: { open: boolean; onClose: () => void }) {
  const ref = useRef<HTMLDialogElement>(null);
  useEffect(() => {
    const d = ref.current;
    if (!d) return;
    if (open && !d.open) d.showModal();
    if (!open && d.open) d.close();
  }, [open]);
  return (
    <dialog ref={ref} className="help" aria-labelledby="help-title" onClose={onClose}>
      <h2 id="help-title">Keyboard shortcuts</h2>
      <table>
        <tbody>
          {SHORTCUTS.map(([keys, what]) => (
            <tr key={keys}>
              <th scope="row">
                <kbd>{keys}</kbd>
              </th>
              <td>{what}</td>
            </tr>
          ))}
        </tbody>
      </table>
      <p>
        The field origin is its centre (WPILib 2027). Paths are stored for blue; the red view shows them through the
        project's red flip, and edits made there are stored for blue.
      </p>
      <form method="dialog">
        <button type="submit" autoFocus>
          Close
        </button>
      </form>
    </dialog>
  );
}

export function App() {
  const [help, setHelp] = useState(false);
  const benchOpen = useStore(bench, (b) => b.open);
  usePlaybackClock();

  useEffect(() => startAnalysis(), []);

  useEffect(() => {
    // the Bench view is read-only: no editing shortcuts while it is open
    const keys = (e: KeyboardEvent) => !bench.get().open && onKeyDown(e, () => setHelp(true));
    const leave = (e: BeforeUnloadEvent) => {
      if (isDirty(editor.get())) e.preventDefault();
    };
    window.addEventListener("keydown", keys);
    window.addEventListener("beforeunload", leave);
    return () => {
      window.removeEventListener("keydown", keys);
      window.removeEventListener("beforeunload", leave);
    };
  }, []);

  // Switching paths starts the playback over
  useEffect(() => {
    let shown = editor.get().activePathId;
    return editor.subscribe(() => {
      const id = editor.get().activePathId;
      if (id !== shown) {
        shown = id;
        playback.set({ t: 0, playing: false });
      }
    });
  }, []);

  return (
    <div className="app">
      <Toolbar />
      <PathTabs onHelp={() => setHelp(true)} />
      {benchOpen ? (
        <BenchView />
      ) : (
        <main className="workspace">
          <section className="field-column" aria-label="Field">
            <FieldCanvas />
            <PlaybackBar />
            <Charts />
          </section>
          <aside className="sidebar" aria-label="Path details">
            <ChecksPanel />
            <PathPanel />
            <WaypointsPanel />
            <EventsPanel />
            <RotationPanel />
            <ZonesPanel />
            <OptimisePanel />
            <PointsPanel />
            <ObstaclesPanel />
            <RobotPanel />
            <ProjectPanel />
            <LivePanel />
          </aside>
        </main>
      )}
      <StatusLine />
      <HelpDialog open={help} onClose={() => setHelp(false)} />
    </div>
  );
}
