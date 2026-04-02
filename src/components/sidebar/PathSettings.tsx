import { usePathStore } from '../../stores/pathStore';
import { Gauge, Zap, Play, Square } from 'lucide-react';

const fields = [
  { key: 'maxVelocity' as const, label: 'Max Velocity', unit: 'm/s', icon: Gauge },
  { key: 'maxAcceleration' as const, label: 'Max Acceleration', unit: 'm/s\u00B2', icon: Zap },
  { key: 'startVelocity' as const, label: 'Start Velocity', unit: 'm/s', icon: Play },
  { key: 'endVelocity' as const, label: 'End Velocity', unit: 'm/s', icon: Square },
];

export function PathSettings() {
  const constraints = usePathStore((s) => s.constraints);
  const setConstraints = usePathStore((s) => s.setConstraints);

  const update = (key: keyof typeof constraints, value: string) => {
    const num = parseFloat(value);
    if (!isNaN(num) && num >= 0) {
      setConstraints({ ...constraints, [key]: num });
    }
  };

  return (
    <div className="space-y-3">
      {fields.map(({ key, label, unit, icon: Icon }) => (
        <div key={key} className="flex items-center gap-2.5">
          <Icon size={13} className="text-accent-green/60 shrink-0" />
          <label className="text-xs text-zinc-400 w-28 shrink-0">{label}</label>
          <div className="flex items-center gap-1.5 flex-1">
            <input
              type="number"
              step={0.1}
              min={0}
              value={constraints[key]}
              onChange={(e) => update(key, e.target.value)}
              className="w-full text-right"
            />
            <span className="text-[10px] text-zinc-600 w-8 shrink-0">{unit}</span>
          </div>
        </div>
      ))}
    </div>
  );
}
