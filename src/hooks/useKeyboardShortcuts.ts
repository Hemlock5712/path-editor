import { useHotkeys } from 'react-hotkeys-hook';
import { usePathStore } from '../stores/pathStore';
import { useEditorStore } from '../stores/editorStore';

export function useKeyboardShortcuts(callbacks: {
  onPlay: () => void;
  onStop: () => void;
  onStepForward: () => void;
  onStepBackward: () => void;
  onSave: () => void;
  onLoad: () => void;
}) {
  // Use stable store action references. Read transient values via getState()
  // at invocation time so dependency arrays stay stable across renders and
  // hotkeys don't re-register on every parent render.
  const undo = usePathStore((s) => s.undo);
  const redo = usePathStore((s) => s.redo);
  const deletePoint = usePathStore((s) => s.deletePoint);
  const selectPoint = usePathStore((s) => s.selectPoint);
  const movePoint = usePathStore((s) => s.movePoint);
  const toggleSnapToGrid = useEditorStore((s) => s.toggleSnapToGrid);
  const toggleMinimap = useEditorStore((s) => s.toggleMinimap);
  const resetView = useEditorStore((s) => s.resetView);

  // Undo / Redo
  useHotkeys('ctrl+z, meta+z', (e) => { e.preventDefault(); undo(); }, [undo]);
  useHotkeys('ctrl+shift+z, meta+shift+z, ctrl+y, meta+y', (e) => { e.preventDefault(); redo(); }, [redo]);

  // Delete selected point
  useHotkeys('delete, backspace', () => {
    const { selectedPointIndex, controlPoints } = usePathStore.getState();
    if (selectedPointIndex !== null && controlPoints.length > 2) {
      deletePoint(selectedPointIndex);
    }
  }, [deletePoint]);

  // Deselect
  useHotkeys('escape', () => selectPoint(null), [selectPoint]);

  // Play/Pause/Stop — read playbackState at invocation time
  useHotkeys('space', (e) => {
    e.preventDefault();
    const { playbackState } = useEditorStore.getState();
    if (playbackState === 'playing') callbacks.onStop();
    else callbacks.onPlay();
  }, [callbacks.onPlay, callbacks.onStop]);

  // Step forward/backward
  useHotkeys('period', () => callbacks.onStepForward(), [callbacks.onStepForward]);
  useHotkeys('comma', () => callbacks.onStepBackward(), [callbacks.onStepBackward]);

  // Nudge selected point with arrow keys
  useHotkeys('up,down,left,right', (e) => {
    e.preventDefault();
    const { selectedPointIndex, controlPoints } = usePathStore.getState();
    if (selectedPointIndex === null) return;
    const { snapToGrid, gridSize } = useEditorStore.getState();
    const NUDGE_STEP_METERS = 0.05;
    const nudge = snapToGrid ? gridSize : NUDGE_STEP_METERS;
    const pt = controlPoints[selectedPointIndex];
    const delta = { x: 0, y: 0 };
    if (e.key === 'ArrowUp') delta.y = nudge;
    if (e.key === 'ArrowDown') delta.y = -nudge;
    if (e.key === 'ArrowLeft') delta.x = -nudge;
    if (e.key === 'ArrowRight') delta.x = nudge;
    movePoint(selectedPointIndex, { x: pt.x + delta.x, y: pt.y + delta.y });
  }, [movePoint]);

  // Fine nudge with shift
  useHotkeys('shift+up,shift+down,shift+left,shift+right', (e) => {
    e.preventDefault();
    const { selectedPointIndex, controlPoints } = usePathStore.getState();
    if (selectedPointIndex === null) return;
    const FINE_NUDGE_METERS = 0.01;
    const pt = controlPoints[selectedPointIndex];
    const delta = { x: 0, y: 0 };
    if (e.key === 'ArrowUp') delta.y = FINE_NUDGE_METERS;
    if (e.key === 'ArrowDown') delta.y = -FINE_NUDGE_METERS;
    if (e.key === 'ArrowLeft') delta.x = -FINE_NUDGE_METERS;
    if (e.key === 'ArrowRight') delta.x = FINE_NUDGE_METERS;
    movePoint(selectedPointIndex, { x: pt.x + delta.x, y: pt.y + delta.y });
  }, [movePoint]);

  // Save/Load
  useHotkeys('ctrl+s, meta+s', (e) => { e.preventDefault(); callbacks.onSave(); }, [callbacks.onSave]);
  useHotkeys('ctrl+o, meta+o', (e) => { e.preventDefault(); callbacks.onLoad(); }, [callbacks.onLoad]);

  // Zoom
  useHotkeys('equal, plus', () =>
    useEditorStore.getState().setZoom(Math.min(8, useEditorStore.getState().zoom * 1.2)),
  );
  useHotkeys('minus', () =>
    useEditorStore.getState().setZoom(Math.max(0.5, useEditorStore.getState().zoom / 1.2)),
  );
  useHotkeys('0', resetView, [resetView]);

  // Toggles
  useHotkeys('g', () => toggleSnapToGrid(), [toggleSnapToGrid]);
  useHotkeys('m', () => toggleMinimap(), [toggleMinimap]);
}
