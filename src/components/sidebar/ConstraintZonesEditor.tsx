import { memo } from 'react';
import { usePathStore } from '../../stores/pathStore';
import { Plus, X, Gauge, Zap } from 'lucide-react';

export const ConstraintZonesEditor = memo(function ConstraintZonesEditor() {
  const constraintZones = usePathStore((s) => s.constraintZones);
  const constraints = usePathStore((s) => s.constraints);
  const numPoints = usePathStore((s) => s.controlPoints.length);
  const addConstraintZone = usePathStore((s) => s.addConstraintZone);
  const updateConstraintZone = usePathStore((s) => s.updateConstraintZone);
  const deleteConstraintZone = usePathStore((s) => s.deleteConstraintZone);

  const maxIndex = Math.max(0, numPoints - 1);

  const handleAdd = () => {
    addConstraintZone({
      id: crypto.randomUUID(),
      startWaypointIndex: 0,
      endWaypointIndex: Math.min(1, maxIndex),
      maxVelocity: constraints.maxVelocity,
      maxAcceleration: constraints.maxAcceleration,
    });
  };

  const handleUpdate = (zoneIndex: number, field: string, value: string) => {
    const num = parseFloat(value);
    if (isNaN(num) || num < 0) return;
    const clamped =
      field === 'maxVelocity' || field === 'maxAcceleration'
        ? Math.max(0.01, num)
        : num;
    const zone = constraintZones[zoneIndex];
    const updated = { ...zone, [field]: clamped };
    updateConstraintZone(zoneIndex, updated);
  };

  if (numPoints < 2) {
    return (
      <p className="text-xs text-zinc-600 italic">
        Load a Paths.java file to begin
      </p>
    );
  }

  return (
    <div className="space-y-3">
      {constraintZones.map((zone, i) => (
        <div
          key={zone.id}
          className="space-y-2 rounded border border-amber-500/10 bg-amber-500/[0.03] p-2.5"
        >
          <div className="flex items-center justify-between">
            <span className="font-mono text-[10px] text-amber-400/60">
              Zone {i + 1}
            </span>
            <button
              onClick={() => deleteConstraintZone(i)}
              className="btn-ghost p-0.5 text-zinc-500 hover:text-red-400"
            >
              <X size={12} />
            </button>
          </div>

          {/* Waypoint range */}
          <div className="flex items-center gap-2">
            <span className="w-12 shrink-0 text-[10px] text-zinc-500">
              Points
            </span>
            <select
              value={zone.startWaypointIndex}
              onChange={(e) =>
                handleUpdate(i, 'startWaypointIndex', e.target.value)
              }
              className="min-w-0 flex-1 rounded px-1.5 py-0.5 text-xs"
            >
              {Array.from({ length: maxIndex + 1 }, (_, idx) => idx)
                .filter((idx) => idx < zone.endWaypointIndex)
                .map((idx) => (
                  <option key={idx} value={idx}>
                    Pt {idx}
                  </option>
                ))}
            </select>
            <span className="text-[10px] text-zinc-600">to</span>
            <select
              value={zone.endWaypointIndex}
              onChange={(e) =>
                handleUpdate(i, 'endWaypointIndex', e.target.value)
              }
              className="min-w-0 flex-1 rounded px-1.5 py-0.5 text-xs"
            >
              {Array.from({ length: maxIndex + 1 }, (_, idx) => idx)
                .filter((idx) => idx > zone.startWaypointIndex)
                .map((idx) => (
                  <option key={idx} value={idx}>
                    Pt {idx}
                  </option>
                ))}
            </select>
          </div>

          {/* Max velocity */}
          <div className="flex items-center gap-2">
            <Gauge size={11} className="shrink-0 text-amber-400/50" />
            <span className="w-12 shrink-0 text-[10px] text-zinc-500">
              Max Vel
            </span>
            <input
              type="number"
              step={0.1}
              min={0}
              value={zone.maxVelocity}
              onChange={(e) => handleUpdate(i, 'maxVelocity', e.target.value)}
              className="w-full text-right"
            />
            <span className="w-8 shrink-0 text-[10px] text-zinc-600">m/s</span>
          </div>

          {/* Max acceleration */}
          <div className="flex items-center gap-2">
            <Zap size={11} className="shrink-0 text-amber-400/50" />
            <span className="w-12 shrink-0 text-[10px] text-zinc-500">
              Max Acc
            </span>
            <input
              type="number"
              step={0.1}
              min={0}
              value={zone.maxAcceleration}
              onChange={(e) =>
                handleUpdate(i, 'maxAcceleration', e.target.value)
              }
              className="w-full text-right"
            />
            <span className="w-8 shrink-0 text-[10px] text-zinc-600">m/s²</span>
          </div>
        </div>
      ))}

      <button
        onClick={handleAdd}
        className="btn-ghost flex w-full items-center justify-center gap-1.5 py-1.5 text-xs"
      >
        <Plus size={13} />
        Add Zone
      </button>
    </div>
  );
});
