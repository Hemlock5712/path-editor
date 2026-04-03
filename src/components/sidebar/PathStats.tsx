import { memo } from 'react';
import type { PathStats as PathStatsType } from '../../math/ProfileAnalytics';

interface PathStatsProps {
  stats: PathStatsType | null;
}

export const PathStats = memo(function PathStats({ stats }: PathStatsProps) {
  if (!stats) {
    return (
      <p className="text-xs text-zinc-600 italic">Load a Paths.java file to view stats</p>
    );
  }

  return (
    <div className="space-y-1">
      {/* Primary stats */}
      <StatRow label="Total length" value={`${stats.totalLength.toFixed(3)} m`} />
      <StatRow label="Estimated time" value={`${stats.estimatedTime.toFixed(3)} s`} />

      <div className="border-t border-white/[0.04] my-1.5" />

      {/* Counts */}
      <StatRow label="Control points" value={String(stats.numControlPoints)} />
      <StatRow label="Heading waypoints" value={String(stats.numHeadingWaypoints)} />

      <div className="border-t border-white/[0.04] my-1.5" />

      {/* Extrema */}
      <StatRow
        label="Max curvature"
        value={`${stats.maxCurvature.toFixed(3)} 1/m`}
        sub={`@ ${stats.maxCurvatureDistance.toFixed(2)} m`}
      />
      <StatRow
        label="Avg velocity"
        value={`${stats.averageVelocity.toFixed(2)} m/s`}
      />
      <StatRow
        label="Peak angular vel"
        value={`${stats.peakAngularVelocity.toFixed(2)} rad/s`}
        sub={stats.peakAngularVelocity > 0 ? `@ ${stats.peakAngularVelocityDistance.toFixed(2)} m` : undefined}
      />
    </div>
  );
});

function StatRow({
  label,
  value,
  sub,
}: {
  label: string;
  value: string;
  sub?: string;
}) {
  return (
    <div className="flex items-baseline justify-between text-xs gap-2">
      <span className="text-zinc-500 shrink-0">{label}</span>
      <div className="text-right">
        <span className="text-zinc-300 font-mono">{value}</span>
        {sub && <span className="text-zinc-600 ml-1.5 text-[10px]">{sub}</span>}
      </div>
    </div>
  );
}
