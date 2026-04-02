import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import { getPreset } from '../utils/motorPresets';

export interface RobotSettings {
  // Motor
  motorPreset: string;
  stallTorque: number;
  freeSpeedRpm: number;
  stallCurrent: number;
  statorCurrentLimit: number;

  // Drivetrain
  gearRatio: number;
  wheelRadius: number;
  robotMass: number;
  numDriveMotors: number;

  // Physics
  frictionCoefficient: number;
  brakingReactionTime: number;
  minVelocity: number;

  // Robot dimensions
  robotLength: number;
  robotWidth: number;
}

export interface RobotComputed {
  kt: number;
  maxFrictionAccel: number;
  maxTheoreticalVelocity: number;
  motorAccelAtZero: number;
  motorAccelAtMax: number;
}

const GRAVITY = 9.81;

export const DEFAULT_SETTINGS: RobotSettings = {
  motorPreset: 'kraken-x60-foc',
  stallTorque: 9.3615,
  freeSpeedRpm: 5784.65,
  stallCurrent: 476.1,
  statorCurrentLimit: 150.0,

  gearRatio: 6.03,
  wheelRadius: 0.0508,
  robotMass: 60,
  numDriveMotors: 4,

  frictionCoefficient: 1.1,
  brakingReactionTime: 0.1,
  minVelocity: 0.1,

  robotLength: 0.84,
  robotWidth: 0.84,
};

export function computeDerived(s: RobotSettings): RobotComputed {
  const kt = s.stallTorque / s.stallCurrent;
  const maxFrictionAccel = s.frictionCoefficient * GRAVITY;
  const freeSpeedRadPerSec = (s.freeSpeedRpm * 2 * Math.PI) / 60;
  const maxTheoreticalVelocity = (freeSpeedRadPerSec / s.gearRatio) * s.wheelRadius;

  const getMotorAccel = (speed: number) => {
    const wheelAngVel = speed / s.wheelRadius;
    const motorAngVel = wheelAngVel * s.gearRatio;
    const motorRpm = (motorAngVel * 60) / (2 * Math.PI);
    let torque = s.stallTorque * (1.0 - motorRpm / s.freeSpeedRpm);
    torque = Math.min(torque, kt * s.statorCurrentLimit);
    torque = Math.max(torque, 0);
    const wheelTorque = torque * s.gearRatio;
    const force = (wheelTorque / s.wheelRadius) * s.numDriveMotors;
    return force / s.robotMass;
  };

  return {
    kt,
    maxFrictionAccel,
    maxTheoreticalVelocity,
    motorAccelAtZero: getMotorAccel(0),
    motorAccelAtMax: getMotorAccel(maxTheoreticalVelocity * 0.95),
  };
}

interface SettingsStore extends RobotSettings {
  setField: <K extends keyof RobotSettings>(key: K, value: RobotSettings[K]) => void;
  applyMotorPreset: (presetId: string) => void;
  resetToDefaults: () => void;
}

export const useSettingsStore = create<SettingsStore>()(
  persist(
    (set) => ({
      ...DEFAULT_SETTINGS,

      setField: (key, value) => set({ [key]: value }),

      applyMotorPreset: (presetId) => {
        const preset = getPreset(presetId);
        if (preset) {
          set({
            motorPreset: presetId,
            stallTorque: preset.stallTorque,
            freeSpeedRpm: preset.freeSpeedRpm,
            stallCurrent: preset.stallCurrent,
          });
        }
      },

      resetToDefaults: () => set(DEFAULT_SETTINGS),
    }),
    { name: 'frc-path-editor-settings' },
  ),
);

/** Read current settings without subscribing (for use in non-React code). */
export function getRobotConfig(): RobotSettings {
  return useSettingsStore.getState();
}
