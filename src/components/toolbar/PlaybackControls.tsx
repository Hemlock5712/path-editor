import { Play, Pause, Square, SkipForward, SkipBack, Gauge } from 'lucide-react';
import { useEditorStore } from '../../stores/editorStore';

interface PlaybackControlsProps {
  onPlay: () => void;
  onPause: () => void;
  onResume: () => void;
  onStop: () => void;
  onStepForward: () => void;
  onStepBackward: () => void;
  playbackState: 'stopped' | 'playing' | 'paused';
}

const SPEED_OPTIONS = [0.25, 0.5, 1, 2, 4];

export function PlaybackControls({
  onPlay,
  onPause,
  onResume,
  onStop,
  onStepForward,
  onStepBackward,
  playbackState,
}: PlaybackControlsProps) {
  const playbackSpeed = useEditorStore((s) => s.playbackSpeed);
  const setPlaybackSpeed = useEditorStore((s) => s.setPlaybackSpeed);

  const handlePlayPause = () => {
    if (playbackState === 'stopped') {
      onPlay();
    } else if (playbackState === 'playing') {
      onPause();
    } else {
      onResume();
    }
  };

  const isPlaying = playbackState === 'playing';

  return (
    <div className="flex items-center gap-1">
      {/* Step backward */}
      <button
        onClick={onStepBackward}
        disabled={playbackState === 'playing'}
        className="btn-ghost p-1.5 disabled:opacity-20 disabled:cursor-not-allowed"
        title="Step backward (0.05m)"
      >
        <SkipBack size={14} />
      </button>

      {/* Play/Pause toggle — neon pulsing */}
      <button
        onClick={handlePlayPause}
        className={`btn p-1.5 rounded-full transition-all duration-300 ${
          isPlaying
            ? 'text-accent-amber border-accent-amber/30 animate-neonPulseAmber'
            : 'text-accent-green border-accent-green/30 hover:shadow-[0_0_12px_rgba(0,255,170,0.3)]'
        }`}
        title={isPlaying ? 'Pause' : 'Play'}
      >
        {isPlaying ? <Pause size={14} /> : <Play size={14} />}
      </button>

      {/* Stop */}
      <button
        onClick={onStop}
        disabled={playbackState === 'stopped'}
        className="btn-ghost p-1.5 disabled:opacity-20 disabled:cursor-not-allowed"
        title="Stop"
      >
        <Square size={14} />
      </button>

      {/* Step forward */}
      <button
        onClick={onStepForward}
        disabled={playbackState === 'playing'}
        className="btn-ghost p-1.5 disabled:opacity-20 disabled:cursor-not-allowed"
        title="Step forward (0.05m)"
      >
        <SkipForward size={14} />
      </button>

      {/* Speed selector */}
      <div className="flex items-center gap-1 ml-1">
        <Gauge size={11} className="text-zinc-600" />
        <select
          value={playbackSpeed}
          onChange={(e) => setPlaybackSpeed(Number(e.target.value))}
          className="bg-transparent border-none text-xs text-zinc-400 px-1 py-0.5 cursor-pointer focus:outline-none focus:text-accent-green"
          title="Playback speed"
        >
          {SPEED_OPTIONS.map((speed) => (
            <option key={speed} value={speed}>
              {speed}x
            </option>
          ))}
        </select>
      </div>
    </div>
  );
}
