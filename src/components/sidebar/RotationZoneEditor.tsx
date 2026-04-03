import { memo } from 'react';
import { usePathStore } from '../../stores/pathStore';
import { Plus, X, Crosshair } from 'lucide-react';

export const RotationZoneEditor = memo(function RotationZoneEditor() {
  const rotationZones = usePathStore((s) => s.rotationZones);
  const numPoints = usePathStore((s) => s.controlPoints.length);
  const selectedZoneId = usePathStore((s) => s.selectedZoneId);
  const addRotationZone = usePathStore((s) => s.addRotationZone);
  const updateRotationZone = usePathStore((s) => s.updateRotationZone);
  const deleteRotationZone = usePathStore((s) => s.deleteRotationZone);
  const selectZone = usePathStore((s) => s.selectZone);

  const maxIndex = Math.max(0, numPoints - 1);

  const handleAdd = () => {
    const midIdx = maxIndex / 2;
    const halfSpan = Math.max(0.5, maxIndex * 0.15);
    addRotationZone({
      id: crypto.randomUUID(),
      startWaypointIndex: Math.round(Math.max(0, midIdx - halfSpan) * 10) / 10,
      endWaypointIndex: Math.round(Math.min(maxIndex, midIdx + halfSpan) * 10) / 10,
      targetPoint: { x: 8, y: 4 },
    });
  };

  const handleUpdateField = (id: string, field: string, value: string) => {
    const num = parseFloat(value);
    if (isNaN(num)) return;

    if (field === 'startWaypointIndex' || field === 'endWaypointIndex') {
      const clamped = Math.max(0, Math.min(num, maxIndex));
      updateRotationZone(id, { [field]: Math.round(clamped * 10) / 10 });
    } else if (field === 'targetX' || field === 'targetY') {
      const zone = rotationZones.find((z) => z.id === id);
      if (!zone) return;
      const clampedX = field === 'targetX' ? Math.max(0, Math.min(num, 16.54)) : zone.targetPoint.x;
      const clampedY = field === 'targetY' ? Math.max(0, Math.min(num, 8.21)) : zone.targetPoint.y;
      updateRotationZone(id, { targetPoint: { x: clampedX, y: clampedY } });
    }
  };

  if (numPoints < 2) {
    return (
      <p className="text-xs text-zinc-600 italic">Load a Paths.java file to begin</p>
    );
  }

  return (
    <div className="space-y-3">
      {rotationZones.map((zone) => {
        const isSelected = zone.id === selectedZoneId;
        return (
          <div
            key={zone.id}
            className={`rounded border p-2.5 space-y-2 cursor-pointer ${
              isSelected
                ? 'border-orange-400/30 bg-orange-500/[0.06]'
                : 'border-orange-500/10 bg-orange-500/[0.03]'
            }`}
            onClick={() => selectZone(zone.id)}
          >
            <div className="flex items-center justify-between">
              <span className="text-[10px] font-mono text-orange-400/60 flex items-center gap-1">
                <Crosshair size={10} />
                Face Point
              </span>
              <button
                onClick={(e) => {
                  e.stopPropagation();
                  deleteRotationZone(zone.id);
                }}
                className="btn-ghost p-0.5 text-zinc-500 hover:text-red-400"
              >
                <X size={12} />
              </button>
            </div>

            {/* Waypoint range */}
            <div className="flex items-center gap-2">
              <span className="text-[10px] text-zinc-500 w-12 shrink-0">Range</span>
              <input
                type="number"
                min={0}
                max={maxIndex}
                step={0.1}
                value={zone.startWaypointIndex}
                onChange={(e) => handleUpdateField(zone.id, 'startWaypointIndex', e.target.value)}
                className="w-14 text-center"
              />
              <span className="text-[10px] text-zinc-600">to</span>
              <input
                type="number"
                min={0}
                max={maxIndex}
                step={0.1}
                value={zone.endWaypointIndex}
                onChange={(e) => handleUpdateField(zone.id, 'endWaypointIndex', e.target.value)}
                className="w-14 text-center"
              />
            </div>

            {/* Target X */}
            <div className="flex items-center gap-2">
              <span className="text-[10px] text-zinc-500 w-12 shrink-0">Target X</span>
              <input
                type="number"
                step={0.1}
                value={zone.targetPoint.x}
                onChange={(e) => handleUpdateField(zone.id, 'targetX', e.target.value)}
                className="w-full text-right"
              />
              <span className="text-[10px] text-zinc-600 w-4 shrink-0">m</span>
            </div>

            {/* Target Y */}
            <div className="flex items-center gap-2">
              <span className="text-[10px] text-zinc-500 w-12 shrink-0">Target Y</span>
              <input
                type="number"
                step={0.1}
                value={zone.targetPoint.y}
                onChange={(e) => handleUpdateField(zone.id, 'targetY', e.target.value)}
                className="w-full text-right"
              />
              <span className="text-[10px] text-zinc-600 w-4 shrink-0">m</span>
            </div>
          </div>
        );
      })}

      <button
        onClick={handleAdd}
        className="btn-ghost flex items-center gap-1.5 text-xs w-full justify-center py-1.5"
      >
        <Plus size={13} />
        Add Face-Point Zone
      </button>
    </div>
  );
});
