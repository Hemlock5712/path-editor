export interface MotorPreset {
  id: string;
  name: string;
  stallTorque: number; // N*m
  freeSpeedRpm: number; // RPM
  stallCurrent: number; // A
}

export const MOTOR_PRESETS: MotorPreset[] = [
  {
    id: 'kraken-x60-foc',
    name: 'Kraken X60 FOC',
    stallTorque: 9.3615,
    freeSpeedRpm: 5784.65,
    stallCurrent: 476.1,
  },
  {
    id: 'kraken-x60',
    name: 'Kraken X60',
    stallTorque: 7.1573,
    freeSpeedRpm: 6065.33,
    stallCurrent: 374.4,
  },
  {
    id: 'falcon-500',
    name: 'Falcon 500',
    stallTorque: 4.69,
    freeSpeedRpm: 6380,
    stallCurrent: 257,
  },
  {
    id: 'neo',
    name: 'NEO',
    stallTorque: 3.36,
    freeSpeedRpm: 5820,
    stallCurrent: 181,
  },
  {
    id: 'neo-550',
    name: 'NEO 550',
    stallTorque: 1.08,
    freeSpeedRpm: 11710,
    stallCurrent: 111,
  },
  {
    id: 'custom',
    name: 'Custom',
    stallTorque: 9.3615,
    freeSpeedRpm: 5784.65,
    stallCurrent: 476.1,
  },
];

export function getPreset(id: string): MotorPreset | undefined {
  return MOTOR_PRESETS.find((p) => p.id === id);
}
