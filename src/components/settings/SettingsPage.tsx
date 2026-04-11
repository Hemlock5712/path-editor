import { RotateCcw, Zap, Cog, Scale, Ruler, Activity } from 'lucide-react';
import { Titlebar } from '../layout/Titlebar';
import { useSettingsStore, computeDerived } from '../../stores/settingsStore';
import { MOTOR_PRESETS } from '../../utils/motorPresets';

export function SettingsPage() {
  const settings = useSettingsStore();
  const derived = computeDerived(settings);
  const isCustomMotor = settings.motorPreset === 'custom';

  return (
    <div className="bg-surface-950 flex h-screen flex-col">
      <Titlebar />
      <div className="flex-1 overflow-y-auto">
        <div className="animate-fadeIn mx-auto max-w-2xl space-y-5 px-6 py-8">
          <div className="mb-6">
            <h2 className="text-accent-green/60 text-sm font-light tracking-[0.15em] uppercase">
              Robot Configuration
            </h2>
            <p className="mt-1 text-xs text-zinc-500">
              These settings affect velocity profiling and robot visualization.
              Changes are saved automatically.
            </p>
          </div>

          {/* Motor */}
          <Section icon={<Zap size={13} />} title="Motor">
            <div className="space-y-3">
              <div className="flex items-center gap-3">
                <label className="w-32 shrink-0 text-xs text-zinc-400">
                  Preset
                </label>
                <select
                  value={settings.motorPreset}
                  onChange={(e) => settings.applyMotorPreset(e.target.value)}
                  className="focus:border-accent-green/40 flex-1 border-b border-white/10 bg-transparent px-1 py-1 text-sm text-zinc-200 transition-colors focus:outline-none"
                >
                  {MOTOR_PRESETS.map((p) => (
                    <option key={p.id} value={p.id} className="bg-surface-800">
                      {p.name}
                    </option>
                  ))}
                </select>
              </div>
              <NumberField
                label="Stall Torque"
                unit="N*m"
                value={settings.stallTorque}
                onChange={(v) => settings.setField('stallTorque', v)}
                step={0.01}
                disabled={!isCustomMotor}
              />
              <NumberField
                label="Free Speed"
                unit="RPM"
                value={settings.freeSpeedRpm}
                onChange={(v) => settings.setField('freeSpeedRpm', v)}
                step={1}
                disabled={!isCustomMotor}
              />
              <NumberField
                label="Stall Current"
                unit="A"
                value={settings.stallCurrent}
                onChange={(v) => settings.setField('stallCurrent', v)}
                step={0.1}
                disabled={!isCustomMotor}
              />
              <NumberField
                label="Current Limit"
                unit="A"
                value={settings.statorCurrentLimit}
                onChange={(v) => settings.setField('statorCurrentLimit', v)}
                step={1}
              />
            </div>
          </Section>

          {/* Drivetrain */}
          <Section icon={<Cog size={13} />} title="Drivetrain">
            <div className="space-y-3">
              <NumberField
                label="Gear Ratio"
                unit=":1"
                value={settings.gearRatio}
                onChange={(v) => settings.setField('gearRatio', v)}
                step={0.01}
              />
              <NumberField
                label="Wheel Radius"
                unit="m"
                value={settings.wheelRadius}
                onChange={(v) => settings.setField('wheelRadius', v)}
                step={0.001}
              />
              <NumberField
                label="Robot Mass"
                unit="kg"
                value={settings.robotMass}
                onChange={(v) => settings.setField('robotMass', v)}
                step={0.5}
              />
              <NumberField
                label="Drive Motors"
                unit=""
                value={settings.numDriveMotors}
                onChange={(v) => settings.setField('numDriveMotors', v)}
                step={1}
              />
            </div>
          </Section>

          {/* Physics */}
          <Section icon={<Activity size={13} />} title="Physics">
            <div className="space-y-3">
              <NumberField
                label="Friction Coeff."
                unit="x g"
                value={settings.frictionCoefficient}
                onChange={(v) => settings.setField('frictionCoefficient', v)}
                step={0.01}
              />
              <NumberField
                label="Braking Reaction"
                unit="s"
                value={settings.brakingReactionTime}
                onChange={(v) => settings.setField('brakingReactionTime', v)}
                step={0.01}
              />
              <NumberField
                label="Min Velocity"
                unit="m/s"
                value={settings.minVelocity}
                onChange={(v) => settings.setField('minVelocity', v)}
                step={0.01}
              />
            </div>
          </Section>

          {/* Robot Dimensions */}
          <Section icon={<Ruler size={13} />} title="Robot Dimensions">
            <div className="space-y-3">
              <NumberField
                label="Length (bumpers)"
                unit="m"
                value={settings.robotLength}
                onChange={(v) => settings.setField('robotLength', v)}
                step={0.01}
              />
              <NumberField
                label="Width (bumpers)"
                unit="m"
                value={settings.robotWidth}
                onChange={(v) => settings.setField('robotWidth', v)}
                step={0.01}
              />
            </div>
          </Section>

          {/* Computed Preview */}
          <Section icon={<Scale size={13} />} title="Computed Values">
            <div className="space-y-2 font-mono text-xs">
              <ComputedRow
                label="Kt (torque constant)"
                value={`${derived.kt.toFixed(5)} N*m/A`}
              />
              <ComputedRow
                label="Max theoretical velocity"
                value={`${derived.maxTheoreticalVelocity.toFixed(2)} m/s`}
              />
              <ComputedRow
                label="Max friction accel"
                value={`${derived.maxFrictionAccel.toFixed(2)} m/s\u00B2`}
              />
              <ComputedRow
                label="Motor accel @ 0 m/s"
                value={`${derived.motorAccelAtZero.toFixed(2)} m/s\u00B2`}
              />
              <ComputedRow
                label="Motor accel @ 95% max v"
                value={`${derived.motorAccelAtMax.toFixed(2)} m/s\u00B2`}
              />
            </div>
          </Section>

          {/* Reset */}
          <div className="pt-2 pb-8">
            <button
              onClick={settings.resetToDefaults}
              className="btn-danger flex items-center gap-2 text-xs"
            >
              <RotateCcw size={12} />
              Reset to Defaults
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}

function Section({
  icon,
  title,
  children,
}: {
  icon: React.ReactNode;
  title: string;
  children: React.ReactNode;
}) {
  return (
    <div className="neon-panel p-4">
      <h3 className="text-accent-green/40 mb-3 flex items-center gap-2 text-[11px] font-light tracking-wide">
        {icon}
        {title}
      </h3>
      {children}
    </div>
  );
}

function NumberField({
  label,
  unit,
  value,
  onChange,
  step = 1,
  disabled = false,
}: {
  label: string;
  unit: string;
  value: number;
  onChange: (v: number) => void;
  step?: number;
  disabled?: boolean;
}) {
  return (
    <div className="flex items-center gap-3">
      <label className="w-32 shrink-0 text-xs text-zinc-400">{label}</label>
      <input
        type="number"
        value={value}
        onChange={(e) => {
          const v = parseFloat(e.target.value);
          if (!isNaN(v)) onChange(v);
        }}
        step={step}
        disabled={disabled}
        className={`flex-1 border-b bg-transparent px-1 py-1 text-right font-mono text-sm transition-colors focus:outline-none ${
          disabled
            ? 'cursor-not-allowed border-white/5 text-zinc-600'
            : 'focus:border-accent-green/40 border-white/10 text-zinc-200'
        }`}
      />
      {unit && (
        <span className="w-10 shrink-0 text-[10px] text-zinc-500">{unit}</span>
      )}
    </div>
  );
}

function ComputedRow({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex items-center justify-between">
      <span className="text-zinc-500">{label}</span>
      <span className="text-accent-green">{value}</span>
    </div>
  );
}
