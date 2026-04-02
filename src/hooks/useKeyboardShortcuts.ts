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
  const {
    undo,
    redo,
    deletePoint,
    selectPoint,
    controlPoints,
    selectedPointIndex,
    movePoint,
  } = usePathStore();
  const {
    toggleGrid,
    toggleMinimap,
    toggleSnapToGrid,
    resetView,
    snapToGrid,
    gridSize,
    playbackState,
  } = useEditorStore();

  // Undo / Redo
  useHotkeys(
    'ctrl+z, meta+z',
    (e) => {
      e.preventDefault();
      undo();
    },
    [undo],
  );
  useHotkeys(
    'ctrl+shift+z, meta+shift+z, ctrl+y, meta+y',
    (e) => {
      e.preventDefault();
      redo();
    },
    [redo],
  );

  // Delete selected point
  useHotkeys(
    'delete, backspace',
    () => {
      if (selectedPointIndex !== null && controlPoints.length > 2) {
        deletePoint(selectedPointIndex);
      }
    },
    [selectedPointIndex, controlPoints.length, deletePoint],
  );

  // Deselect
  useHotkeys('escape', () => selectPoint(null), [selectPoint]);

  // Play/Pause/Stop
  useHotkeys(
    'space',
    (e) => {
      e.preventDefault();
      if (playbackState === 'playing') callbacks.onStop();
      else callbacks.onPlay();
    },
    [playbackState, callbacks],
  );

  // Step forward/backward
  useHotkeys('period', callbacks.onStepForward, [callbacks]);
  useHotkeys('comma', callbacks.onStepBackward, [callbacks]);

  // Nudge selected point with arrow keys
  useHotkeys(
    'up,down,left,right',
    (e) => {
      e.preventDefault();
      if (selectedPointIndex === null) return;
      const nudge = snapToGrid ? gridSize : 0.05;
      const pt = controlPoints[selectedPointIndex];
      const delta = { x: 0, y: 0 };
      if (e.key === 'ArrowUp') delta.y = nudge;
      if (e.key === 'ArrowDown') delta.y = -nudge;
      if (e.key === 'ArrowLeft') delta.x = -nudge;
      if (e.key === 'ArrowRight') delta.x = nudge;
      movePoint(selectedPointIndex, { x: pt.x + delta.x, y: pt.y + delta.y });
    },
    [selectedPointIndex, controlPoints, snapToGrid, gridSize, movePoint],
  );

  // Fine nudge with shift
  useHotkeys(
    'shift+up,shift+down,shift+left,shift+right',
    (e) => {
      e.preventDefault();
      if (selectedPointIndex === null) return;
      const nudge = 0.01;
      const pt = controlPoints[selectedPointIndex];
      const delta = { x: 0, y: 0 };
      if (e.key === 'ArrowUp') delta.y = nudge;
      if (e.key === 'ArrowDown') delta.y = -nudge;
      if (e.key === 'ArrowLeft') delta.x = -nudge;
      if (e.key === 'ArrowRight') delta.x = nudge;
      movePoint(selectedPointIndex, { x: pt.x + delta.x, y: pt.y + delta.y });
    },
    [selectedPointIndex, controlPoints, movePoint],
  );

  // Save/Load
  useHotkeys(
    'ctrl+s, meta+s',
    (e) => {
      e.preventDefault();
      callbacks.onSave();
    },
    [callbacks],
  );
  useHotkeys(
    'ctrl+o, meta+o',
    (e) => {
      e.preventDefault();
      callbacks.onLoad();
    },
    [callbacks],
  );

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
