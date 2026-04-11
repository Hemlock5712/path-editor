import { memo, useMemo } from 'react';
import { usePathStore } from '../../stores/pathStore';
import { useSettingsStore, computeDerived } from '../../stores/settingsStore';
import { Gauge, Zap, Play, Square, RotateCcw } from 'lucide-react';

const fields = [
  {
    key: 'maxVelocity' as const,
    label: 'Max Velocity',
    unit: 'm/s',
    icon: Gauge,
  },
  {
    key: 'maxAcceleration' as const,
    label: 'Max Acceleration',
    unit: 'm/s\u00B2',
    icon: Zap,
  },
  {
    key: 'maxAngularVelocity' as const,
    label: 'Max Angular Vel',
    unit: 'rad/s',
    icon: RotateCcw,
  },
  {
    key: 'maxAngularAcceleration' as const,
    label: 'Max Angular Accel',
    unit: 'rad/s\u00B2',
    icon: RotateCcw,
  },
  {
    key: 'startVelocity' as const,
    label: 'Start Velocity',
    unit: 'm/s',
    icon: Play,
  },
  {
    key: 'endVelocity' as const,
    label: 'End Velocity',
    unit: 'm/s',
    icon: Square,
  },
];

export const PathSettings = memo(function PathSettings() {
  const constraints = usePathStore((s) => s.constraints);
  const setConstraints = usePathStore((s) => s.setConstraints);
  const settings = useSettingsStore();
  const derived = useMemo(() => computeDerived(settings), [settings]);
  const theoreticalMax = derived.maxTheoreticalVelocity;
  const frictionAccel = derived.maxFrictionAccel;

  const update = (key: keyof typeof constraints, value: string) => {
    const num = parseFloat(value);
    if (!isNaN(num) && num >= 0) {
      setConstraints({ ...constraints, [key]: num });
    }
  };

  const isAutoVelocity = constraints.maxVelocity === 0;
  const isAutoAccel = constraints.maxAcceleration === 0;

  const getPlaceholder = (key: string) => {
    if (key === 'maxVelocity') return `Auto (${theoreticalMax.toFixed(2)})`;
    if (key === 'maxAcceleration') return `Auto (${frictionAccel.toFixed(2)})`;
    return undefined;
  };

  return (
    <div className="space-y-3">
      {fields.map(({ key, label, unit, icon: Icon }) => (
        <div key={key}>
          <div className="flex items-center gap-2.5">
            <Icon size={13} className="text-accent-green/60 shrink-0" />
            <label
              htmlFor={`constraint-${key}`}
              className="w-28 shrink-0 text-xs text-zinc-400"
            >
              {label}
            </label>
            <div className="flex flex-1 items-center gap-1.5">
              <input
                id={`constraint-${key}`}
                type="number"
                step={0.1}
                min={0}
                value={constraints[key]}
                placeholder={getPlaceholder(key)}
                onChange={(e) => update(key, e.target.value)}
                className="w-full text-right"
              />
              <span className="w-8 shrink-0 text-[10px] text-zinc-600">
                {unit}
              </span>
            </div>
          </div>
          {key === 'maxVelocity' && (
            <div className="ml-[calc(13px+0.625rem+7rem)] mt-0.5 text-[10px] text-zinc-600">
              {isAutoVelocity
                ? `Auto: ${theoreticalMax.toFixed(2)} m/s from robot config`
                : `Physics limit: ${theoreticalMax.toFixed(2)} m/s`}
            </div>
          )}
          {key === 'maxAcceleration' && (
            <div className="ml-[calc(13px+0.625rem+7rem)] mt-0.5 text-[10px] text-zinc-600">
              {isAutoAccel
                ? `Auto: ${frictionAccel.toFixed(2)} m/s\u00B2 from friction`
                : `Friction limit: ${frictionAccel.toFixed(2)} m/s\u00B2`}
            </div>
          )}
        </div>
      ))}
    </div>
  );
});
