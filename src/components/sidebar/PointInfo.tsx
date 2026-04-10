import { useCallback, useMemo, useState } from 'react';
import { usePathStore } from '../../stores/pathStore';
import { useSelectionStore } from '../../stores/selectionStore';
import { Flag, MapPin, X } from 'lucide-react';
import type { SplinePath } from '../../math/SplinePath';
import type { VelocityProfile } from '../../math/VelocityProfile';
import type { TimeEstimator } from '../../math/TimeEstimator';
import { NamedPointsPanel } from './NamedPointsPanel';
import { HeadingEditor } from './HeadingEditor';
import { Combobox } from '../ui/Combobox';

interface PointInfoProps {
  splinePath: SplinePath | null;
  velocityProfile: VelocityProfile | null;
  timeEstimator: TimeEstimator | null;
}

/**
 * Find the approximate arc-length distance of control point at the given index.
 * Uses the spline's control points to compute a fractional parameter,
 * then maps to arc-length.
 */
function getDistanceAtControlPoint(
  splinePath: SplinePath,
  pointIndex: number,
  totalPoints: number
): number {
  if (totalPoints < 2) return 0;
  // Each control point corresponds roughly to a segment boundary.
  // Point i maps to parameter fraction i / (totalPoints - 1) along the path.
  const frac = pointIndex / (totalPoints - 1);
  return frac * splinePath.totalLength;
}

export function PointInfo({
  splinePath,
  velocityProfile,
  timeEstimator,
}: PointInfoProps) {
  const controlPoints = usePathStore((s) => s.controlPoints);
  const selectedPointIndex = useSelectionStore((s) => s.selectedPointIndex);
  const selectPoint = useSelectionStore((s) => s.selectPoint);
  const movePoint = usePathStore((s) => s.movePoint);
  const insertPointAfter = usePathStore((s) => s.insertPointAfter);
  const waypointFlags = usePathStore((s) => s.waypointFlags);
  const addWaypointFlag = usePathStore((s) => s.addWaypointFlag);
  const updateWaypointFlag = usePathStore((s) => s.updateWaypointFlag);
  const deleteWaypointFlag = usePathStore((s) => s.deleteWaypointFlag);
  const paths = usePathStore((s) => s.paths);
  const [newFlagLabel, setNewFlagLabel] = useState('');

  const hasSelection =
    selectedPointIndex !== null && selectedPointIndex < controlPoints.length;

  const point = hasSelection ? controlPoints[selectedPointIndex] : null;
  const totalPoints = controlPoints.length;
  const selectedFlags = hasSelection
    ? waypointFlags.filter((flag) => flag.waypointIndex === selectedPointIndex)
    : [];

  // Gather flag labels from ALL paths
  const allFlagOptions = useMemo(() => {
    const seen = new Set<string>();
    for (const flag of waypointFlags) {
      const t = flag.label.trim();
      if (t) seen.add(t);
    }
    for (const path of Object.values(paths)) {
      for (const flag of path.waypointFlags || []) {
        const t = flag.label.trim();
        if (t) seen.add(t);
      }
    }
    return Array.from(seen)
      .sort((a, b) => a.localeCompare(b))
      .map((label) => ({ value: label, label }));
  }, [paths, waypointFlags]);

  // Compute derived values if we have a built spline
  let distance: number | null = null;
  let time: number | null = null;
  let velocity: number | null = null;
  let curvature: number | null = null;

  if (hasSelection && splinePath && totalPoints >= 2) {
    distance = getDistanceAtControlPoint(
      splinePath,
      selectedPointIndex,
      totalPoints
    );
    curvature = splinePath.getCurvature(distance);

    if (velocityProfile) {
      velocity = velocityProfile.getVelocity(distance);
    }
    if (timeEstimator) {
      time = timeEstimator.getTime(distance);
    }
  }

  const handleXChange = (value: string) => {
    const num = parseFloat(value);
    if (!isNaN(num) && hasSelection && point) {
      movePoint(selectedPointIndex, { x: num, y: point.y });
    }
  };

  const handleYChange = (value: string) => {
    const num = parseFloat(value);
    if (!isNaN(num) && hasSelection && point) {
      movePoint(selectedPointIndex, { x: point.x, y: num });
    }
  };

  const handleInsertBefore = useCallback(() => {
    if (!hasSelection) return;
    if (selectedPointIndex > 0) {
      const prev = controlPoints[selectedPointIndex - 1];
      const curr = controlPoints[selectedPointIndex];
      const mid = { x: (prev.x + curr.x) / 2, y: (prev.y + curr.y) / 2 };
      insertPointAfter(selectedPointIndex - 1, mid);
      selectPoint(selectedPointIndex);
    } else {
      const curr = controlPoints[0];
      insertPointAfter(-1, { x: curr.x - 0.5, y: curr.y });
      selectPoint(0);
    }
  }, [hasSelection, selectedPointIndex, controlPoints, insertPointAfter, selectPoint]);

  const handleInsertAfter = useCallback(() => {
    if (!hasSelection) return;
    if (selectedPointIndex < controlPoints.length - 1) {
      const curr = controlPoints[selectedPointIndex];
      const next = controlPoints[selectedPointIndex + 1];
      const mid = { x: (curr.x + next.x) / 2, y: (curr.y + next.y) / 2 };
      insertPointAfter(selectedPointIndex, mid);
      selectPoint(selectedPointIndex + 1);
    } else {
      const curr = controlPoints[selectedPointIndex];
      insertPointAfter(selectedPointIndex, { x: curr.x + 0.5, y: curr.y });
      selectPoint(selectedPointIndex + 1);
    }
  }, [hasSelection, selectedPointIndex, controlPoints, insertPointAfter, selectPoint]);

  return (
    <div>
      {hasSelection && point ? (
        <>
          <div className="mb-3 flex items-center gap-2 text-[12px] text-zinc-200">
            <MapPin size={13} className="text-accent-green" />
            <span>
              Point {selectedPointIndex} of {totalPoints}
            </span>
            <div className="ml-auto flex items-center gap-1">
              <button
                onClick={handleInsertBefore}
                className="btn-ghost px-2 py-1 font-mono text-[12px]"
                title="Insert point before"
              >
                + Before
              </button>
              <button
                onClick={handleInsertAfter}
                className="btn-ghost px-2 py-1 font-mono text-[12px]"
                title="Insert point after"
              >
                + After
              </button>
            </div>
          </div>

          {/* Editable coordinates */}
          <div className="mb-3 space-y-2">
            <div className="flex items-center gap-2">
              <label
                htmlFor="point-x"
                className="w-4 font-mono text-[12px] text-accent-green"
              >
                X
              </label>
              <input
                id="point-x"
                type="number"
                step={0.01}
                value={parseFloat(point.x.toFixed(3))}
                onChange={(e) => handleXChange(e.target.value)}
                className="flex-1"
              />
              <span className="text-[12px] text-zinc-500">m</span>
            </div>
            <div className="flex items-center gap-2">
              <label
                htmlFor="point-y"
                className="w-4 font-mono text-[12px] text-accent-green"
              >
                Y
              </label>
              <input
                id="point-y"
                type="number"
                step={0.01}
                value={parseFloat(point.y.toFixed(3))}
                onChange={(e) => handleYChange(e.target.value)}
                className="flex-1"
              />
              <span className="text-[12px] text-zinc-500">m</span>
            </div>
          </div>

          {/* Derived info */}
          {splinePath && (
            <div className="space-y-1 border-t border-white/[0.04] pt-2">
              <InfoRow
                label="Distance"
                value={distance}
                unit="m"
                decimals={3}
              />
              <InfoRow label="Time" value={time} unit="s" decimals={3} />
              <InfoRow
                label="Velocity"
                value={velocity}
                unit="m/s"
                decimals={2}
              />
              <InfoRow
                label="Curvature"
                value={curvature}
                unit="1/m"
                decimals={3}
              />
            </div>
          )}

          {/* Heading */}
          <div className="mt-3 border-t border-white/[0.04] pt-3">
            <h4 className="mb-2 text-xs font-semibold tracking-[0.08em] text-zinc-100 uppercase">
              HEADING
            </h4>
            <HeadingEditor />
          </div>

          <div className="mt-3 border-t border-white/[0.04] pt-3">
            <div className="mb-2 flex items-center gap-2">
              <Flag size={12} className="text-sky-300" />
              <h4 className="text-xs font-semibold tracking-[0.08em] text-zinc-100 uppercase">
                WAYPOINT FLAGS
              </h4>
            </div>

            <div className="space-y-2">
              {selectedFlags.length > 0 ? (
                selectedFlags.map((flag) => (
                  <div key={flag.id} className="flex items-center gap-2">
                    <Combobox
                      options={allFlagOptions}
                      value={flag.label}
                      onChange={(label) =>
                        updateWaypointFlag(flag.id, { label })
                      }
                      allowCustom
                      placeholder="Flag label"
                      className="flex-1"
                    />
                    <button
                      onClick={() => deleteWaypointFlag(flag.id)}
                      className="btn-ghost px-1.5 py-1 text-zinc-300 hover:text-red-300"
                      title="Delete flag"
                    >
                      <X size={12} />
                    </button>
                  </div>
                ))
              ) : (
                <p className="text-[12px] italic text-zinc-500">
                  No flags on this waypoint
                </p>
              )}

              <Combobox
                options={allFlagOptions}
                value={newFlagLabel}
                onChange={setNewFlagLabel}
                onCommit={(label) => {
                  addWaypointFlag(selectedPointIndex, label);
                  setNewFlagLabel('');
                }}
                allowCustom
                placeholder="Add flag label"
              />
            </div>
          </div>
        </>
      ) : (
        <p className="text-[12px] text-zinc-500 italic">
          Select a point to view info
        </p>
      )}

      {/* Named points */}
      <div className="mt-3 border-t border-white/[0.04] pt-3">
        <h4 className="mb-2 text-xs font-semibold tracking-[0.08em] text-zinc-100 uppercase">
          NAMED POINTS
        </h4>
        <NamedPointsPanel />
      </div>
    </div>
  );
}

function InfoRow({
  label,
  value,
  unit,
  decimals,
}: {
  label: string;
  value: number | null;
  unit: string;
  decimals: number;
}) {
  return (
    <div className="flex items-center justify-between text-[12px]">
      <span className="text-zinc-200">{label}</span>
      <span className="font-mono text-zinc-100">
        {value !== null ? value.toFixed(decimals) : '--'}{' '}
        <span className="text-zinc-300">{unit}</span>
      </span>
    </div>
  );
}
