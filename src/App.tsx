import { useMemo, useCallback } from 'react';
import { usePathStore } from './stores/pathStore';
import { useEditorStore } from './stores/editorStore';
import { usePathComputation } from './hooks/usePathComputation';
import { usePlayback } from './hooks/usePlayback';
import { useKeyboardShortcuts } from './hooks/useKeyboardShortcuts';
import { ErrorBoundary } from './components/ErrorBoundary';
import { AppShell } from './components/layout/AppShell';
import { Titlebar } from './components/layout/Titlebar';
import { FieldCanvas } from './components/field/FieldCanvas';
import { ChartPanel } from './components/charts/ChartPanel';
import { Toolbar } from './components/toolbar/Toolbar';
import { PathTabs } from './components/toolbar/PathTabs';
import { PathSettings } from './components/sidebar/PathSettings';
import { ConstraintZonesEditor } from './components/sidebar/ConstraintZonesEditor';
import { RotationZoneEditor } from './components/sidebar/RotationZoneEditor';
import { PointInfo } from './components/sidebar/PointInfo';
import { PathStats } from './components/sidebar/PathStats';
import { SidebarSection } from './components/sidebar/SidebarSection';
import { parsePathsJava } from './utils/javaParser';
import { generatePathsJava } from './utils/javaExport';
import { buildSortedHeadings, interpolateHeadingSorted } from './math/ProfileAnalytics';

export default function App() {
  const { splinePath, velocityProfile, timeEstimator, analytics, stats } = usePathComputation();

  const playback = usePlayback(splinePath, velocityProfile);

  // Compute scrubber heading for ghost display
  const headingWaypoints = usePathStore((s) => s.headingWaypoints);
  const controlPoints = usePathStore((s) => s.controlPoints);
  const rotationZones = usePathStore((s) => s.rotationZones);
  const numControlPoints = controlPoints.length;
  const scrubberDistance = useEditorStore((s) => s.scrubberDistance);

  const scrubberHeading = useMemo(() => {
    if (!splinePath || splinePath.totalLength === 0) return null;
    const progress = scrubberDistance / splinePath.totalLength;

    // Check if scrubber is inside a rotation zone
    const waypointIndex = progress * (numControlPoints - 1);
    for (const zone of rotationZones) {
      if (waypointIndex >= zone.startWaypointIndex && waypointIndex <= zone.endWaypointIndex) {
        // Face the target point
        const pathPoint = splinePath.getPoint(scrubberDistance);
        const dx = zone.targetPoint.x - pathPoint.x;
        const dy = zone.targetPoint.y - pathPoint.y;
        return Math.atan2(dy, dx);
      }
    }

    const sorted = buildSortedHeadings(headingWaypoints, numControlPoints);
    const heading = interpolateHeadingSorted(sorted, progress);
    return isNaN(heading) ? null : heading;
  }, [splinePath, headingWaypoints, numControlPoints, rotationZones, scrubberDistance]);

  // Save/Load callbacks for keyboard shortcuts
  const handleSave = useCallback(async () => {
    const allPaths = usePathStore.getState().getAllPaths();
    if (allPaths.length === 0) return;
    const java = generatePathsJava(allPaths, usePathStore.getState().namedPoints);

    if ('showSaveFilePicker' in window) {
      try {
        const handle = await (window as any).showSaveFilePicker({
          suggestedName: 'Paths.java',
          types: [
            {
              description: 'Java Source',
              accept: { 'text/x-java-source': ['.java'] },
            },
          ],
        });
        const writable = await handle.createWritable();
        await writable.write(java);
        await writable.close();
      } catch {
        // User cancelled
      }
    }
  }, []);

  const handleLoad = useCallback(() => {
    if ('showOpenFilePicker' in window) {
      (window as any)
        .showOpenFilePicker({
          types: [{ description: 'Java Source', accept: { 'text/x-java-source': ['.java'] } }],
        })
        .then(async ([handle]: any[]) => {
          const file = await handle.getFile();
          const text = await file.text();
          const parsed = parsePathsJava(text);
          usePathStore.getState().loadAllPaths(parsed);
        })
        .catch(() => {});
    } else {
      const input = document.createElement('input');
      input.type = 'file';
      input.accept = '.java';
      input.onchange = async () => {
        const file = input.files?.[0];
        if (!file) return;
        try {
          const text = await file.text();
          const parsed = parsePathsJava(text);
          usePathStore.getState().loadAllPaths(parsed);
        } catch (e) {
          alert(e instanceof Error ? e.message : 'Failed to parse Java file');
        }
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
          <PathTabs />
          <Toolbar
            onPlay={playback.play}
            onPause={playback.pause}
            onResume={playback.resume}
            onStop={playback.stop}
            onStepForward={playback.stepForward}
            onStepBackward={playback.stepBackward}
          />
          <ErrorBoundary>
            <div className="flex-1 min-h-0">
              <FieldCanvas splinePath={splinePath} scrubberHeading={scrubberHeading} />
            </div>
          </ErrorBoundary>
        </div>
      }
      sidebar={
        <>
          <SidebarSection title="Point Info" delay={0}>
            <PointInfo
              splinePath={splinePath}
              velocityProfile={velocityProfile}
              timeEstimator={timeEstimator}
            />
          </SidebarSection>
          <SidebarSection title="Statistics" delay={1}>
            <PathStats stats={stats} />
          </SidebarSection>
          <SidebarSection title="Rotation Zones" delay={2}>
            <RotationZoneEditor />
          </SidebarSection>
          <SidebarSection title="Constraint Zones" delay={3}>
            <ConstraintZonesEditor />
          </SidebarSection>
          <SidebarSection title="Constraints" delay={4}>
            <PathSettings />
          </SidebarSection>
        </>
      }
      bottomPanel={
        <ErrorBoundary>
          <ChartPanel
            analytics={analytics}
            velocityProfile={velocityProfile}
            timeEstimator={timeEstimator}
            splinePath={splinePath}
          />
        </ErrorBoundary>
      }
    />
  );
}

