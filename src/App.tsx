import { useMemo, useCallback } from 'react';
import { usePathStore } from './stores/pathStore';
import { useEditorStore } from './stores/editorStore';
import { usePathComputation } from './hooks/usePathComputation';
import { usePlayback } from './hooks/usePlayback';
import { useKeyboardShortcuts } from './hooks/useKeyboardShortcuts';
import { AppShell } from './components/layout/AppShell';
import { Titlebar } from './components/layout/Titlebar';
import { FieldCanvas } from './components/field/FieldCanvas';
import { ChartPanel } from './components/charts/ChartPanel';
import { Toolbar } from './components/toolbar/Toolbar';
import { PathSettings } from './components/sidebar/PathSettings';
import { HeadingEditor } from './components/sidebar/HeadingEditor';
import { PointInfo } from './components/sidebar/PointInfo';
import { PathStats } from './components/sidebar/PathStats';
import { serialize, deserialize } from './utils/pathJson';

export default function App() {
  const { splinePath, velocityProfile, timeEstimator, analytics, stats } = usePathComputation();

  const playback = usePlayback(splinePath, velocityProfile);

  // Compute scrubber heading for ghost display
  const headingWaypoints = usePathStore((s) => s.headingWaypoints);
  const numControlPoints = usePathStore((s) => s.controlPoints.length);
  const scrubberDistance = useEditorStore((s) => s.scrubberDistance);

  const scrubberHeading = useMemo(() => {
    if (!splinePath || splinePath.totalLength === 0) return null;
    const progress = scrubberDistance / splinePath.totalLength;
    return interpolateHeading(headingWaypoints, numControlPoints, progress);
  }, [splinePath, headingWaypoints, numControlPoints, scrubberDistance]);

  // Save/Load callbacks for keyboard shortcuts
  const handleSave = useCallback(() => {
    const state = usePathStore.getState();
    const json = serialize(
      state.controlPoints,
      state.headingWaypoints,
      state.constraints,
    );

    if ('showSaveFilePicker' in window) {
      (window as any)
        .showSaveFilePicker({
          suggestedName: 'path.json',
          types: [{ description: 'Path JSON', accept: { 'application/json': ['.json'] } }],
        })
        .then(async (handle: any) => {
          const writable = await handle.createWritable();
          await writable.write(json);
          await writable.close();
        })
        .catch(() => {});
    } else {
      const blob = new Blob([json], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = 'path.json';
      a.click();
      URL.revokeObjectURL(url);
    }
  }, []);

  const handleLoad = useCallback(() => {
    if ('showOpenFilePicker' in window) {
      (window as any)
        .showOpenFilePicker({
          types: [{ description: 'Path JSON', accept: { 'application/json': ['.json'] } }],
        })
        .then(async ([handle]: any[]) => {
          const file = await handle.getFile();
          const text = await file.text();
          const data = deserialize(text);
          const { controlPoints: cp, headingWaypoints: hw, constraints: c } = data;
          usePathStore.getState().loadPath(cp, hw, c);
        })
        .catch(() => {});
    } else {
      const input = document.createElement('input');
      input.type = 'file';
      input.accept = '.json';
      input.onchange = async () => {
        const file = input.files?.[0];
        if (!file) return;
        const text = await file.text();
        const data = deserialize(text);
        const { controlPoints: cp, headingWaypoints: hw, constraints: c } = data;
        usePathStore.getState().loadPath(cp, hw, c);
      };
      input.click();
    }
  }, []);

  // Register keyboard shortcuts
  useKeyboardShortcuts({
    onPlay: playback.playbackState === 'paused' ? playback.resume : playback.play,
    onStop: playback.stop,
    onStepForward: playback.stepForward,
    onStepBackward: playback.stepBackward,
    onSave: handleSave,
    onLoad: handleLoad,
  });

  return (
    <AppShell
      titlebar={<Titlebar stats={stats} showSidebar />}
      field={
        <div className="flex flex-col h-full">
          <Toolbar
            onPlay={playback.play}
            onPause={playback.pause}
            onResume={playback.resume}
            onStop={playback.stop}
            onStepForward={playback.stepForward}
            onStepBackward={playback.stepBackward}
          />
          <div className="flex-1 min-h-0">
            <FieldCanvas splinePath={splinePath} scrubberHeading={scrubberHeading} />
          </div>
        </div>
      }
      sidebar={
        <>
          <SidebarSection title="Constraints" delay={0}>
            <PathSettings />
          </SidebarSection>
          <SidebarSection title="Heading" delay={1}>
            <HeadingEditor />
          </SidebarSection>
          <SidebarSection title="Point Info" delay={2}>
            <PointInfo
              splinePath={splinePath}
              velocityProfile={velocityProfile}
              timeEstimator={timeEstimator}
            />
          </SidebarSection>
          <SidebarSection title="Statistics" delay={3}>
            <PathStats stats={stats} />
          </SidebarSection>
        </>
      }
      bottomPanel={
        <ChartPanel
          analytics={analytics}
          velocityProfile={velocityProfile}
          timeEstimator={timeEstimator}
          splinePath={splinePath}
        />
      }
    />
  );
}

function SidebarSection({ title, children, delay = 0 }: { title: string; children: React.ReactNode; delay?: number }) {
  return (
    <div
      className="neon-panel p-3.5 animate-fadeIn"
      style={{ animationDelay: `${delay * 50}ms` }}
    >
      <h3 className="text-[11px] font-light tracking-wide text-accent-green/40 mb-2.5">{title}</h3>
      {children}
    </div>
  );
}

/** Interpolate robot heading from heading waypoints at a given path progress (0-1). */
function interpolateHeading(
  waypoints: { waypointIndex: number; degrees: number }[],
  numControlPoints: number,
  progress: number
): number | null {
  if (waypoints.length === 0 || numControlPoints < 2) return null;

  const sorted = waypoints
    .map((hw) => ({
      frac: hw.waypointIndex / (numControlPoints - 1),
      rad: (hw.degrees * Math.PI) / 180,
    }))
    .sort((a, b) => a.frac - b.frac);

  if (progress <= sorted[0].frac) return sorted[0].rad;
  if (progress >= sorted[sorted.length - 1].frac) return sorted[sorted.length - 1].rad;

  for (let i = 0; i < sorted.length - 1; i++) {
    if (progress >= sorted[i].frac && progress <= sorted[i + 1].frac) {
      const t = (progress - sorted[i].frac) / (sorted[i + 1].frac - sorted[i].frac);
      let diff = ((sorted[i + 1].rad - sorted[i].rad + Math.PI) % (2 * Math.PI)) - Math.PI;
      if (diff < -Math.PI) diff += 2 * Math.PI;
      return sorted[i].rad + diff * t;
    }
  }
  return sorted[sorted.length - 1].rad;
}
