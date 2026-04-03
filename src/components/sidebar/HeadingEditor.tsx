import { usePathStore } from '../../stores/pathStore';
import { Compass, X } from 'lucide-react';

export function HeadingEditor() {
  const selectedPointIndex = usePathStore((s) => s.selectedPointIndex);
  const headingWaypoints = usePathStore((s) => s.headingWaypoints);
  const setHeading = usePathStore((s) => s.setHeading);

  if (selectedPointIndex === null) return null;

  const existing = headingWaypoints.find(
    (hw) => Math.round(hw.waypointIndex) === selectedPointIndex,
  );
  const headingDeg = existing?.degrees ?? null;
  const headingRad = headingDeg !== null ? (headingDeg * Math.PI) / 180 : null;

  return (
    <div className="flex items-start gap-3">
      {/* Compass preview — neon styled */}
      <div className="shrink-0 w-12 h-12 rounded-full border border-accent-green/10 bg-transparent relative flex items-center justify-center">
        {/* Tick marks — neon dots */}
        {[0, 90, 180, 270].map((deg) => {
          const rad = (deg * Math.PI) / 180;
          const x = Math.cos(rad) * 20;
          const y = -Math.sin(rad) * 20;
          return (
            <div
              key={deg}
              className="absolute w-1 h-1 rounded-full bg-accent-green/20"
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
            className="absolute w-0.5 h-5 bg-accent-green rounded-full origin-bottom"
            style={{
              bottom: '50%',
              left: 'calc(50% - 1px)',
              transform: `rotate(${90 - headingDeg!}deg)`,
              boxShadow: '0 0 6px rgba(0, 255, 170, 0.5)',
            }}
          />
        )}
        {headingRad === null && (
          <Compass size={14} className="text-zinc-600" />
        )}
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
            className="btn-ghost flex items-center gap-1 text-xs py-0.5 px-2"
          >
            <X size={12} />
            Clear
          </button>
        )}
      </div>
    </div>
  );
}
