import { usePathStore } from '../../stores/pathStore';
import { MapPin } from 'lucide-react';
import type { SplinePath } from '../../math/SplinePath';
import type { VelocityProfile } from '../../math/VelocityProfile';
import type { TimeEstimator } from '../../math/TimeEstimator';

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
  totalPoints: number,
): number {
  if (totalPoints < 2) return 0;
  // Each control point corresponds roughly to a segment boundary.
  // Point i maps to parameter fraction i / (totalPoints - 1) along the path.
  const frac = pointIndex / (totalPoints - 1);
  return frac * splinePath.totalLength;
}

export function PointInfo({ splinePath, velocityProfile, timeEstimator }: PointInfoProps) {
  const controlPoints = usePathStore((s) => s.controlPoints);
  const selectedPointIndex = usePathStore((s) => s.selectedPointIndex);
  const movePoint = usePathStore((s) => s.movePoint);

  if (selectedPointIndex === null || selectedPointIndex >= controlPoints.length) {
    return (
      <p className="text-xs text-zinc-600 italic">Select a point to view info</p>
    );
  }

  const point = controlPoints[selectedPointIndex];
  const totalPoints = controlPoints.length;

  // Compute derived values if we have a built spline
  let distance: number | null = null;
  let time: number | null = null;
  let velocity: number | null = null;
  let curvature: number | null = null;

  if (splinePath && totalPoints >= 2) {
    distance = getDistanceAtControlPoint(splinePath, selectedPointIndex, totalPoints);
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
    if (!isNaN(num)) {
      movePoint(selectedPointIndex, { x: num, y: point.y });
    }
  };

  const handleYChange = (value: string) => {
    const num = parseFloat(value);
    if (!isNaN(num)) {
      movePoint(selectedPointIndex, { x: point.x, y: num });
    }
  };

  return (
    <div>
      <div className="text-xs text-zinc-400 mb-3 flex items-center gap-1.5">
        <MapPin size={12} className="text-accent-green/60" />
        Point {selectedPointIndex + 1} of {totalPoints}
      </div>

      {/* Editable coordinates */}
      <div className="space-y-2 mb-3">
        <div className="flex items-center gap-2">
          <span className="text-xs text-accent-green/40 w-4 font-mono">X</span>
          <input
            type="number"
            step={0.01}
            value={parseFloat(point.x.toFixed(3))}
            onChange={(e) => handleXChange(e.target.value)}
            className="flex-1"
          />
          <span className="text-[10px] text-zinc-600">m</span>
        </div>
        <div className="flex items-center gap-2">
          <span className="text-xs text-accent-green/40 w-4 font-mono">Y</span>
          <input
            type="number"
            step={0.01}
            value={parseFloat(point.y.toFixed(3))}
            onChange={(e) => handleYChange(e.target.value)}
            className="flex-1"
          />
          <span className="text-[10px] text-zinc-600">m</span>
        </div>
      </div>

      {/* Derived info */}
      {splinePath && (
        <div className="border-t border-white/[0.04] pt-2 space-y-1">
          <InfoRow label="Distance" value={distance} unit="m" decimals={3} />
          <InfoRow label="Time" value={time} unit="s" decimals={3} />
          <InfoRow label="Velocity" value={velocity} unit="m/s" decimals={2} />
          <InfoRow label="Curvature" value={curvature} unit="1/m" decimals={3} />
        </div>
      )}
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
    <div className="flex items-center justify-between text-xs">
      <span className="text-zinc-500">{label}</span>
      <span className="text-zinc-300 font-mono">
        {value !== null ? value.toFixed(decimals) : '--'}{' '}
        <span className="text-zinc-600">{unit}</span>
      </span>
    </div>
  );
}
