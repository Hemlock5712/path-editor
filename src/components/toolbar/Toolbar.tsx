import { Save, FolderOpen, Undo2, Redo2, Trash2, Box } from 'lucide-react';
import { usePathStore } from '../../stores/pathStore';
import { useEditorStore } from '../../stores/editorStore';
import { serialize, deserialize } from '../../utils/pathJson';
import { PlaybackControls } from './PlaybackControls';

interface ToolbarProps {
  onPlay: () => void;
  onPause: () => void;
  onResume: () => void;
  onStop: () => void;
  onStepForward: () => void;
  onStepBackward: () => void;
  fieldCanvasRef?: React.RefObject<HTMLCanvasElement | null>;
}

export function Toolbar({
  onPlay,
  onPause,
  onResume,
  onStop,
  onStepForward,
  onStepBackward,
  fieldCanvasRef,
}: ToolbarProps) {
  const controlPoints = usePathStore((s) => s.controlPoints);
  const headingWaypoints = usePathStore((s) => s.headingWaypoints);
  const constraints = usePathStore((s) => s.constraints);
  const loadPath = usePathStore((s) => s.loadPath);
  const clear = usePathStore((s) => s.clear);
  const undo = usePathStore((s) => s.undo);
  const redo = usePathStore((s) => s.redo);
  const undoStack = usePathStore((s) => s.undoStack);
  const redoStack = usePathStore((s) => s.redoStack);

  const playbackState = useEditorStore((s) => s.playbackState);
  const showWaypointGhosts = useEditorStore((s) => s.showWaypointGhosts);
  const toggleWaypointGhosts = useEditorStore((s) => s.toggleWaypointGhosts);

  // Save handler with File System Access API + fallback
  const handleSave = async () => {
    const json = serialize(controlPoints, headingWaypoints, constraints);

    if ('showSaveFilePicker' in window) {
      try {
        const handle = await (window as any).showSaveFilePicker({
          suggestedName: 'path.json',
          types: [
            {
              description: 'Path JSON',
              accept: { 'application/json': ['.json'] },
            },
          ],
        });
        const writable = await handle.createWritable();
        await writable.write(json);
        await writable.close();
        return;
      } catch {
        // User cancelled or API unavailable
      }
      return;
    }

    // Fallback: download
    const blob = new Blob([json], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'path.json';
    a.click();
    URL.revokeObjectURL(url);
  };

  // Load handler with File System Access API + fallback
  const handleLoad = async () => {
    if ('showOpenFilePicker' in window) {
      try {
        const [handle] = await (window as any).showOpenFilePicker({
          types: [
            {
              description: 'Path JSON',
              accept: { 'application/json': ['.json'] },
            },
          ],
        });
        const file = await handle.getFile();
        const text = await file.text();
        const data = deserialize(text);
        loadPath(
          data.controlPoints,
          data.headingWaypoints,
          data.constraints,
        );
        return;
      } catch {
        // User cancelled
      }
      return;
    }

    // Fallback: file input
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';
    input.onchange = async () => {
      const file = input.files?.[0];
      if (!file) return;
      const text = await file.text();
      const data = deserialize(text);
      loadPath(
        data.controlPoints,
        data.headingWaypoints,
        data.constraints,
      );
    };
    input.click();
  };

  // Clear with confirmation
  const handleClear = () => {
    if (controlPoints.length === 0) return;
    clear();
  };

  return (
    <div className="flex items-center gap-1.5 px-4 py-2 bg-transparent">
      {/* File group */}
      <button onClick={handleSave} className="btn-default flex items-center gap-1.5 text-xs" title="Save (Ctrl+S)">
        <Save size={13} />
        <span>Save</span>
      </button>

      <button onClick={handleLoad} className="btn-default flex items-center gap-1.5 text-xs" title="Load (Ctrl+O)">
        <FolderOpen size={13} />
        <span>Load</span>
      </button>

      {/* Separator */}
      <div className="w-px h-4 bg-accent-green/[0.08] mx-1" />

      {/* Undo/Redo group — icon only */}
      <button
        onClick={undo}
        disabled={undoStack.length === 0}
        className="btn-ghost p-1.5 disabled:opacity-20 disabled:cursor-not-allowed"
        title="Undo (Ctrl+Z)"
      >
        <Undo2 size={14} />
      </button>

      <button
        onClick={redo}
        disabled={redoStack.length === 0}
        className="btn-ghost p-1.5 disabled:opacity-20 disabled:cursor-not-allowed"
        title="Redo (Ctrl+Y)"
      >
        <Redo2 size={14} />
      </button>

      {/* Separator */}
      <div className="w-px h-4 bg-accent-green/[0.08] mx-1" />

      {/* Clear */}
      <button
        onClick={handleClear}
        className="btn-danger flex items-center gap-1 text-xs"
        title="Clear all points"
      >
        <Trash2 size={13} />
        <span>Clear</span>
      </button>

      {/* Separator */}
      <div className="w-px h-4 bg-accent-green/[0.08] mx-1" />

      {/* Waypoint ghosts toggle */}
      <button
        onClick={toggleWaypointGhosts}
        className={`btn-ghost p-1.5 ${showWaypointGhosts ? 'text-accent-green' : 'text-zinc-600'}`}
        title="Toggle robot outlines at waypoints"
      >
        <Box size={14} />
      </button>

      {/* Separator */}
      <div className="w-px h-4 bg-accent-green/[0.08] mx-1" />

      {/* Playback controls inline */}
      <PlaybackControls
        onPlay={onPlay}
        onPause={onPause}
        onResume={onResume}
        onStop={onStop}
        onStepForward={onStepForward}
        onStepBackward={onStepBackward}
        playbackState={playbackState}
      />
    </div>
  );
}
