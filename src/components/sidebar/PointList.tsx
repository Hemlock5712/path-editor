import { memo } from 'react';
import { usePathStore } from '../../stores/pathStore';
import { useSelectionStore } from '../../stores/selectionStore';

export const PointList = memo(function PointList() {
  const controlPoints = usePathStore((s) => s.controlPoints);
  const controlPointRefs = usePathStore((s) => s.controlPointRefs);
  const selectedPointIndex = useSelectionStore((s) => s.selectedPointIndex);
  const selectPoint = useSelectionStore((s) => s.selectPoint);

  if (controlPoints.length === 0) {
    return <p className="text-xs text-zinc-600 italic">No control points yet</p>;
  }

  return (
    <div className="max-h-48 space-y-0.5 overflow-y-auto">
      {controlPoints.map((pt, i) => {
        const isSelected = selectedPointIndex === i;
        const ref = controlPointRefs[i];

        return (
          <button
            key={i}
            onClick={() => selectPoint(i)}
            className={`flex w-full items-center rounded border px-2 py-1 text-left transition-colors ${
              isSelected
                ? 'border-accent-green/30 bg-accent-green/[0.06]'
                : 'border-white/[0.03] bg-transparent hover:bg-white/[0.03]'
            }`}
          >
            <span className="w-5 shrink-0 font-mono text-[10px] text-accent-green/60">
              {i + 1}
            </span>
            <span className="shrink-0 font-mono text-[10px] text-zinc-400">
              {pt.x.toFixed(2)}, {pt.y.toFixed(2)}
            </span>
            {ref && (
              <span className="ml-auto truncate pl-2 font-mono text-[10px] text-amber-400/60">
                {ref}
              </span>
            )}
          </button>
        );
      })}
    </div>
  );
});
