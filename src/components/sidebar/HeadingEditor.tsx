import { memo } from 'react';
import { usePathStore } from '../../stores/pathStore';
import { useSelectionStore } from '../../stores/selectionStore';
import { Compass, X } from 'lucide-react';

export const HeadingEditor = memo(function HeadingEditor() {
  const selectedPointIndex = useSelectionStore((s) => s.selectedPointIndex);
  const headingWaypoints = usePathStore((s) => s.headingWaypoints);
  const setHeading = usePathStore((s) => s.setHeading);

  if (selectedPointIndex === null) return null;

  const existing = headingWaypoints.find(
    (hw) => Math.round(hw.waypointIndex) === selectedPointIndex
  );
  const headingDeg = existing?.degrees ?? null;
  const headingRad = headingDeg !== null ? (headingDeg * Math.PI) / 180 : null;

  return (
    <div className="flex items-start gap-3">
      {/* Compass preview — neon styled */}
      <div className="border-accent-green/10 relative flex h-12 w-12 shrink-0 items-center justify-center rounded-full border bg-transparent">
        {/* Tick marks — neon dots */}
        {[0, 90, 180, 270].map((deg) => {
          const rad = (deg * Math.PI) / 180;
          const x = Math.cos(rad) * 20;
          const y = -Math.sin(rad) * 20;
          return (
            <div
              key={deg}
              className="bg-accent-green/20 absolute h-1 w-1 rounded-full"
              style={{
                left: `calc(50% + ${x}px - 2px)`,
                top: `calc(50% + ${y}px - 2px)`,
              }}
            />
          );
        })}
        {/* Heading needle — neon green with glow */}
        {headingRad !== null && (
          <div
            className="bg-accent-green absolute h-5 w-0.5 origin-bottom rounded-full"
            style={{
              bottom: '50%',
              left: 'calc(50% - 1px)',
              transform: `rotate(${90 - headingDeg!}deg)`,
              boxShadow: '0 0 6px rgba(0, 255, 170, 0.5)',
            }}
          />
        )}
        {headingRad === null && <Compass size={14} className="text-zinc-600" />}
      </div>

      {/* Controls */}
      <div className="flex-1 space-y-2">
        <div className="text-xs text-zinc-400">
          Point {selectedPointIndex} heading
        </div>
        <div className="flex items-center gap-2">
          <input
            type="number"
            step={5}
            value={headingDeg ?? ''}
            placeholder="none"
            onChange={(e) => {
              const val = e.target.value;
              if (val === '') {
                setHeading(selectedPointIndex, null);
              } else {
                const deg = parseFloat(val);
                if (!isNaN(deg)) {
                  setHeading(selectedPointIndex, deg);
                }
              }
            }}
            className="w-20"
          />
          <span className="text-[10px] text-zinc-600">deg</span>
        </div>
        {existing && (
          <button
            onClick={() => setHeading(selectedPointIndex, null)}
            className="btn-ghost flex items-center gap-1 px-2 py-0.5 text-xs"
          >
            <X size={12} />
            Clear
          </button>
        )}
      </div>
    </div>
  );
});
