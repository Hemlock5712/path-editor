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
    <div className="flex items-center gap-3 border-b border-white/[0.04] px-4 py-2.5">
      <div className="flex items-center gap-1 rounded-full border border-white/[0.05] bg-white/[0.015] p-1">
        <button
          onClick={handleLoadJava}
          className="btn-default flex items-center gap-1.5 rounded-full border-transparent text-xs"
          title="Load Paths.java (Ctrl+O)"
        >
          <FolderOpen size={13} />
          <span>Load Java</span>
        </button>

        <button
          onClick={handleSaveJava}
          className="btn-primary flex items-center gap-1.5 rounded-full text-xs"
          title="Save Paths.java (Ctrl+S)"
        >
          <Save size={13} />
          <span>Save Java</span>
        </button>
      </div>

      <div className="flex items-center gap-1 rounded-full border border-white/[0.05] bg-white/[0.015] p-1">
        <button
          onClick={undo}
          disabled={undoStack.length === 0}
          className="btn-ghost rounded-full p-1.5 disabled:cursor-not-allowed disabled:opacity-20"
          title="Undo (Ctrl+Z)"
        >
          <Undo2 size={14} />
        </button>

        <button
          onClick={redo}
          disabled={redoStack.length === 0}
          className="btn-ghost rounded-full p-1.5 disabled:cursor-not-allowed disabled:opacity-20"
          title="Redo (Ctrl+Y)"
        >
          <Redo2 size={14} />
        </button>
      </div>

      <div className="flex items-center gap-1 rounded-full border border-white/[0.05] bg-white/[0.015] p-1">
        <button
          onClick={handleDeletePath}
          disabled={!activePathName || pathCount <= 1}
          className="btn-danger flex items-center gap-1 rounded-full text-xs disabled:cursor-not-allowed disabled:opacity-20"
          title="Delete active path"
        >
          <Trash2 size={13} />
          <span>Delete Path</span>
        </button>

        <button
          onClick={flipPathY}
          disabled={controlPoints.length < 2}
          className="btn-ghost rounded-full p-1.5 disabled:cursor-not-allowed disabled:opacity-20"
          title="Flip path left/right (mirror Y)"
        >
          <FlipVertical2 size={14} />
        </button>

        <button
          onClick={() => duplicatePath()}
          disabled={controlPoints.length === 0}
          className="btn-ghost rounded-full p-1.5 disabled:cursor-not-allowed disabled:opacity-20"
          title="Duplicate active path"
        >
          <Copy size={14} />
        </button>
      </div>

      <div className="mr-1 flex items-center gap-1 rounded-full border border-white/[0.05] bg-white/[0.015] p-1">
        <button
          onClick={toggleWaypointGhosts}
          className={`rounded-full p-1.5 transition-colors ${
            showWaypointGhosts
              ? 'bg-accent-green/[0.08] text-accent-green'
              : 'text-zinc-300 hover:bg-white/[0.05] hover:text-zinc-100'
          }`}
          title="Toggle robot outlines at waypoints"
        >
          <Box size={14} />
        </button>
      </div>

      <div className="min-w-0 flex-1" />

      <div className="flex items-center gap-2 rounded-full border border-white/[0.05] bg-white/[0.015] px-2 py-1">
        <span className="font-mono text-[11px] uppercase tracking-[0.08em] text-zinc-300">
          Playback
        </span>
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
    </div>
  );
}
