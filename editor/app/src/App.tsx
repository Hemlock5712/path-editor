// The app's window: the top bar, the page (editor, telemetry, bench or settings), the status bar, and
// the app-wide layers (tooltips, toasts, menus, the palette, dialogs). Starts the analysis worker and the
// playback clock.

import { useEffect } from "react";
import { analysis, playback, startAnalysis } from "./state/analysis.ts";
import { editor, isDirty } from "./state/editor.ts";
import { useStore } from "./state/store.ts";
import { INSPECTOR, RAIL, ui } from "./state/ui.ts";
import { BenchView } from "./ui/BenchView.tsx";
import { ProjectRail } from "./ui/editor/ProjectRail.tsx";
import { Stage } from "./ui/editor/Stage.tsx";
import { Inspector } from "./ui/inspector/Inspector.tsx";
import { SettingsPage } from "./ui/pages/SettingsPage.tsx";
import { TelemetryPage } from "./ui/pages/TelemetryPage.tsx";
import { isTauri } from "./io/native.ts";
import { reopenLastProject } from "./state/session.ts";
import { listenToNativeMenu } from "./ui/nativeMenu.ts";
import { onKeyDown } from "./ui/shortcuts.ts";
import { CommandPalette, ConfirmHost, ContextMenuHost, HelpDialog, installAppBehaviour, Splitter, Toasts, TooltipHost } from "./ui/shell.tsx";
import { StatusBar, TopBar } from "./ui/TopBar.tsx";

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

function EditorPage() {
  const railWidth = useStore(ui, (s) => s.railWidth);
  const inspectorWidth = useStore(ui, (s) => s.inspectorWidth);
  return (
    <div className="split">
      <aside className="pane side" style={{ width: railWidth }} aria-label="Project">
        <ProjectRail />
      </aside>
      <Splitter label="Resize the paths rail" side="left" width={railWidth} min={RAIL.min} max={RAIL.max} onResize={(w) => ui.set({ railWidth: w })} />
      <div className="pane main">
        <Stage />
      </div>
      <Splitter
        label="Resize the inspector"
        side="right"
        width={inspectorWidth}
        min={INSPECTOR.min}
        max={INSPECTOR.max}
        onResize={(w) => ui.set({ inspectorWidth: w })}
      />
      <aside className="pane side" style={{ width: inspectorWidth }} aria-label="Inspector" data-testid="inspector">
        <Inspector />
      </aside>
    </div>
  );
}

export function App() {
  const page = useStore(ui, (s) => s.page);
  usePlaybackClock();

  useEffect(() => startAnalysis(), []);

  useEffect(() => {
    const leave = (e: BeforeUnloadEvent) => {
      if (isDirty(editor.get())) e.preventDefault();
    };
    const uninstall = installAppBehaviour();
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("beforeunload", leave);
    // the splash in index.html goes once the app has drawn
    document.getElementById("splash")?.remove();
    // the desktop app: native menus, and the robot project from last time
    if (isTauri()) {
      void reopenLastProject();
      void listenToNativeMenu();
    }
    return () => {
      uninstall();
      window.removeEventListener("keydown", onKeyDown);
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
      <TopBar />
      <div className="page" data-page={page}>
        {page === "editor" && <EditorPage />}
        {page === "telemetry" && <TelemetryPage />}
        {page === "bench" && <BenchView />}
        {page === "settings" && <SettingsPage />}
      </div>
      <StatusBar />
      <TooltipHost />
      <Toasts />
      <ContextMenuHost />
      <CommandPalette />
      <ConfirmHost />
      <HelpDialog />
    </div>
  );
}
