import { memo } from 'react';
import { usePathStore } from '../../stores/pathStore';
import { useSelectionStore } from '../../stores/selectionStore';
import { Flag } from 'lucide-react';

export const PointList = memo(function PointList() {
  const controlPoints = usePathStore((s) => s.controlPoints);
  const controlPointRefs = usePathStore((s) => s.controlPointRefs);
  const waypointFlags = usePathStore((s) => s.waypointFlags);
  const selectedPointIndex = useSelectionStore((s) => s.selectedPointIndex);
  const selectPoint = useSelectionStore((s) => s.selectPoint);

  if (controlPoints.length === 0) {
    return (
      <p className="text-[12px] text-zinc-500 italic">No control points yet</p>
    );
  }

  return (
    <div className="max-h-48 space-y-0.5 overflow-y-auto">
      {controlPoints.map((pt, i) => {
        const isSelected = selectedPointIndex === i;
        const ref = controlPointRefs[i];
        const flags = waypointFlags.filter((flag) => flag.waypointIndex === i);

        return (
          <button
            key={i}
            onClick={() => selectPoint(i)}
            className={`flex w-full items-center rounded-lg border px-2.5 py-2 text-left transition-colors ${
              isSelected
                ? 'border-accent-green/30 bg-accent-green/[0.08]'
                : 'border-white/[0.06] bg-white/[0.015] hover:bg-white/[0.04]'
            }`}
          >
            <span className="w-6 shrink-0 font-mono text-[12px] text-accent-green">
              {i + 1}
            </span>
            <span className="shrink-0 font-mono text-[12px] text-zinc-100">
              {pt.x.toFixed(2)}, {pt.y.toFixed(2)}
            </span>
            {flags.length > 0 && (
              <span className="ml-auto flex items-center gap-1 pl-2 text-[12px] text-sky-300">
                <Flag size={11} />
                {flags.length}
              </span>
            )}
            {ref && (
              <span className="truncate pl-2 font-mono text-[12px] text-amber-300">
                {ref}
              </span>
            )}
          </button>
        );
      })}
    </div>
  );
});
