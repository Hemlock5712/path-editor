import { useEditorStore } from '../../stores/editorStore';
import { VelocityDistanceChart } from './VelocityDistanceChart';
import { VelocityTimeChart } from './VelocityTimeChart';
import { CurvatureChart } from './CurvatureChart';
import { AccelerationChart } from './AccelerationChart';
import { HeadingChart } from './HeadingChart';
import type { AnalyticsArrays } from '../../math/ProfileAnalytics';
import { VelocityProfile } from '../../math/VelocityProfile';
import { TimeEstimator } from '../../math/TimeEstimator';
import { SplinePath } from '../../math/SplinePath';

const TABS = [
  { key: 'velocity-distance', label: 'Vel / Dist' },
  { key: 'velocity-time', label: 'Vel / Time' },
  { key: 'curvature', label: 'Curvature' },
  { key: 'acceleration', label: 'Accel' },
  { key: 'heading', label: 'Heading' },
] as const;

interface ChartPanelProps {
  analytics: AnalyticsArrays | null;
  velocityProfile: VelocityProfile | null;
  timeEstimator: TimeEstimator | null;
  splinePath: SplinePath | null;
}

export function ChartPanel({
  analytics,
  velocityProfile,
  timeEstimator,
  splinePath,
}: ChartPanelProps) {
  const activeChart = useEditorStore((s) => s.activeChart);
  const setActiveChart = useEditorStore((s) => s.setActiveChart);
  const maxVelocity = 5.0;

  return (
    <div className="flex h-full flex-col bg-[#050505]">
      {/* Tab bar — neon underline tabs */}
      <div className="flex flex-shrink-0 items-center gap-1 px-3 pt-1.5 pb-0">
        {TABS.map((tab) => (
          <button
            key={tab.key}
            onClick={() => setActiveChart(tab.key)}
            className={`border-b px-2.5 py-1 text-xs font-medium transition-all duration-200 ${
              activeChart === tab.key
                ? 'text-accent-green border-accent-green shadow-[0_1px_6px_rgba(0,255,170,0.3)]'
                : 'border-transparent text-zinc-600 hover:text-zinc-400'
            }`}
          >
            {tab.label}
          </button>
        ))}
      </div>

      {/* Chart container */}
      <div className="min-h-0 flex-1">
        {activeChart === 'velocity-distance' && (
          <VelocityDistanceChart
            analytics={analytics}
            profile={velocityProfile}
            timeEstimator={timeEstimator}
            maxVelocity={maxVelocity}
          />
        )}
        {activeChart === 'velocity-time' && (
          <VelocityTimeChart
            analytics={analytics}
            profile={velocityProfile}
            timeEstimator={timeEstimator}
            maxVelocity={maxVelocity}
          />
        )}
        {activeChart === 'curvature' && (
          <CurvatureChart analytics={analytics} splinePath={splinePath} />
        )}
        {activeChart === 'acceleration' && (
          <AccelerationChart analytics={analytics} />
        )}
        {activeChart === 'heading' && (
          <HeadingChart analytics={analytics} splinePath={splinePath} />
        )}
      </div>
    </div>
  );
}
