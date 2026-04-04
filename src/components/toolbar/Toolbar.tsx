import {
  FolderOpen,
  Undo2,
  Redo2,
  Trash2,
  Box,
  Save,
  FlipVertical2,
  Copy,
} from 'lucide-react';
import { usePathStore } from '../../stores/pathStore';
import { useEditorStore } from '../../stores/editorStore';
import { parsePathsJava } from '../../utils/javaParser';
import { generatePathsJava } from '../../utils/javaExport';
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
  const activePathName = usePathStore((s) => s.activePathName);
  const loadAllPaths = usePathStore((s) => s.loadAllPaths);
  const deletePath = usePathStore((s) => s.deletePath);
  const undo = usePathStore((s) => s.undo);
  const redo = usePathStore((s) => s.redo);
  const undoStack = usePathStore((s) => s.undoStack);
  const redoStack = usePathStore((s) => s.redoStack);
  const flipPathY = usePathStore((s) => s.flipPathY);
  const duplicatePath = usePathStore((s) => s.duplicatePath);
  const controlPoints = usePathStore((s) => s.controlPoints);
  const pathCount = Object.keys(usePathStore((s) => s.paths)).length;

  const playbackState = useEditorStore((s) => s.playbackState);
  const showWaypointGhosts = useEditorStore((s) => s.showWaypointGhosts);
  const toggleWaypointGhosts = useEditorStore((s) => s.toggleWaypointGhosts);

  // Load Java file
  const handleLoadJava = async () => {
    if ('showOpenFilePicker' in window) {
      try {
        const [handle] = await window.showOpenFilePicker({
          types: [
            {
              description: 'Java Source',
              accept: { 'text/x-java-source': ['.java'] },
            },
          ],
        });
        const file = await handle.getFile();
        const text = await file.text();
        const parsed = parsePathsJava(text);
        loadAllPaths(parsed);
        return;
      } catch {
        // User cancelled
      }
      return;
    }

    // Fallback: file input
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.java';
    input.onchange = async () => {
      const file = input.files?.[0];
      if (!file) return;
      try {
        const text = await file.text();
        const parsed = parsePathsJava(text);
        loadAllPaths(parsed);
      } catch (e) {
        alert(e instanceof Error ? e.message : 'Failed to parse Java file');
      }
    };
    input.click();
  };

  // Save as Java file
  const handleSaveJava = async () => {
    const allPaths = usePathStore.getState().getAllPaths();
    if (allPaths.length === 0) return;
    const java = generatePathsJava(
      allPaths,
      usePathStore.getState().namedPoints
    );

    if ('showSaveFilePicker' in window) {
      try {
        const handle = await window.showSaveFilePicker({
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
  };

  // Delete active path
  const handleDeletePath = () => {
    if (!activePathName) return;
    deletePath(activePathName);
  };

  return (
    <div className="flex items-center gap-1.5 bg-transparent px-4 py-2">
      {/* File group */}
      <button
        onClick={handleLoadJava}
        className="btn-default flex items-center gap-1.5 text-xs"
        title="Load Paths.java (Ctrl+O)"
      >
        <FolderOpen size={13} />
        <span>Load Java</span>
      </button>

      <button
        onClick={handleSaveJava}
        className="btn-default flex items-center gap-1.5 text-xs"
        title="Save Paths.java (Ctrl+S)"
      >
        <Save size={13} />
        <span>Save Java</span>
      </button>

      {/* Separator */}
      <div className="bg-accent-green/[0.08] mx-1 h-4 w-px" />

      {/* Undo/Redo group */}
      <button
        onClick={undo}
        disabled={undoStack.length === 0}
        className="btn-ghost p-1.5 disabled:cursor-not-allowed disabled:opacity-20"
        title="Undo (Ctrl+Z)"
      >
        <Undo2 size={14} />
      </button>

      <button
        onClick={redo}
        disabled={redoStack.length === 0}
        className="btn-ghost p-1.5 disabled:cursor-not-allowed disabled:opacity-20"
        title="Redo (Ctrl+Y)"
      >
        <Redo2 size={14} />
      </button>

      {/* Separator */}
      <div className="bg-accent-green/[0.08] mx-1 h-4 w-px" />

      {/* Delete active path */}
      <button
        onClick={handleDeletePath}
        disabled={!activePathName || pathCount <= 1}
        className="btn-danger flex items-center gap-1 text-xs disabled:cursor-not-allowed disabled:opacity-20"
        title="Delete active path"
      >
        <Trash2 size={13} />
        <span>Delete Path</span>
      </button>

      {/* Flip & Duplicate */}
      <button
        onClick={flipPathY}
        disabled={controlPoints.length < 2}
        className="btn-ghost p-1.5 disabled:cursor-not-allowed disabled:opacity-20"
        title="Flip path left/right (mirror Y)"
      >
        <FlipVertical2 size={14} />
      </button>

      <button
        onClick={duplicatePath}
        disabled={controlPoints.length === 0}
        className="btn-ghost p-1.5 disabled:cursor-not-allowed disabled:opacity-20"
        title="Duplicate active path"
      >
        <Copy size={14} />
      </button>

      {/* Separator */}
      <div className="bg-accent-green/[0.08] mx-1 h-4 w-px" />

      {/* Waypoint ghosts toggle */}
      <button
        onClick={toggleWaypointGhosts}
        className="rounded p-1.5 transition-colors"
        style={{
          color: showWaypointGhosts ? '#00FFaa' : '#6b6b7a',
          background: showWaypointGhosts
            ? 'rgba(0, 255, 170, 0.08)'
            : 'transparent',
        }}
        title="Toggle robot outlines at waypoints"
      >
        <Box size={14} />
      </button>

      {/* Separator */}
      <div className="bg-accent-green/[0.08] mx-1 h-4 w-px" />

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
