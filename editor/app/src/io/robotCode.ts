// Reads the robot's numbers out of its own code, each from the file that owns it:
//   - TunerConstants.java (CTRE Tuner X): the drivetrain's shape, the gear ratio, the wheel radius, where
//     the modules are and the speed at 12 V. kSlipCurrent is read only to check it: it is a hardware
//     ceiling, not what paths plan with.
//   - AccelerationLimiter.java (Precision-Drive): what paths plan with, MU, ROBOT_MASS, COM_HEIGHT,
//     MODEL_CURRENT, ANGULAR_ACCEL, the share in TRACTION_ACCEL and the DRIVE_MOTOR model.
// Everything else in the files (CAN ids, gains, offsets) is ignored. Values may be plain numbers or WPILib
// units (Inches.of(2.167), MetersPerSecond.of(4.54)), with simple arithmetic inside.

import type { Motor } from "../../../core/src/index.ts";

export interface TunerImport {
  readonly kind: "tuner";
  readonly driveGearRatio?: number;
  readonly wheelRadiusMeters?: number;
  /** Robot centre to the furthest module, metres. */
  readonly moduleRadiusMeters?: number;
  /** Centre to the nearest tipping edge, the smallest |x| or |y| of any module, metres (AccelerationLimiter's TIP_DISTANCE). */
  readonly tipDistanceMeters?: number;
  readonly speedAt12VoltsMetersPerSecond?: number;
  /** kSlipCurrent, amps: checked against the planning current, never applied. */
  readonly slipCurrentAmps?: number;
  /** The names of the constants found, for saying what was filled in. */
  readonly found: readonly string[];
}

export interface LimiterImport {
  readonly kind: "limiter";
  readonly frictionCoefficient?: number;
  readonly massKg?: number;
  readonly centerOfMassHeightMeters?: number;
  readonly statorCurrentLimitAmps?: number;
  readonly maxAngularRadPerSecSquared?: number;
  /** The share in TRACTION_ACCEL = 0.9 * Math.min(...). */
  readonly tractionShare?: number;
  readonly driveMotor?: Motor;
  readonly found: readonly string[];
}

export type RobotCodeImport = TunerImport | LimiterImport;

const LENGTH: Readonly<Record<string, number>> = { Meters: 1, Centimeters: 0.01, Millimeters: 0.001, Inches: 0.0254, Feet: 0.3048 };
const SPEED: Readonly<Record<string, number>> = { MetersPerSecond: 1, FeetPerSecond: 0.3048, InchesPerSecond: 0.0254 };
const CURRENT: Readonly<Record<string, number>> = { Amps: 1, Milliamps: 0.001 };
const MODULES = ["FrontLeft", "FrontRight", "BackLeft", "BackRight"] as const;

/** Whichever of the two files this is, read; throws when it is neither. */
export function parseRobotCode(text: string): RobotCodeImport {
  if (/\bTRACTION_ACCEL\b|\bclass\s+AccelerationLimiter\b/.test(text)) return parseAccelerationLimiter(text);
  return parseTunerConstants(text);
}

/** The constants in a TunerConstants.java; throws when it has none of them. */
export function parseTunerConstants(text: string): TunerImport {
  const { value, found } = reader(text);
  const driveGearRatio = value("kDriveGearRatio", null);
  const wheelRadiusMeters = value("kWheelRadius", LENGTH);
  const speedAt12VoltsMetersPerSecond = value("kSpeedAt12Volts", SPEED);
  const slipCurrentAmps = value("kSlipCurrent", CURRENT);
  const modules = MODULES.map((m) => ({ x: value(`k${m}XPos`, LENGTH), y: value(`k${m}YPos`, LENGTH) })).filter(
    (p): p is { x: number; y: number } => p.x !== undefined && p.y !== undefined,
  );
  const moduleRadiusMeters = modules.length > 0 ? Math.max(...modules.map((p) => Math.hypot(p.x, p.y))) : undefined;
  const tipDistanceMeters = modules.length > 0 ? Math.min(...modules.map((p) => Math.min(Math.abs(p.x), Math.abs(p.y)))) : undefined;
  if (found.length === 0) throw new RangeError("no TunerConstants found (kDriveGearRatio, kWheelRadius, the module positions, kSpeedAt12Volts)");
  return {
    kind: "tuner",
    ...(positive(driveGearRatio) && { driveGearRatio }),
    ...(positive(wheelRadiusMeters) && { wheelRadiusMeters }),
    ...(positive(moduleRadiusMeters) && { moduleRadiusMeters }),
    ...(positive(tipDistanceMeters) && { tipDistanceMeters }),
    ...(positive(speedAt12VoltsMetersPerSecond) && { speedAt12VoltsMetersPerSecond }),
    ...(positive(slipCurrentAmps) && { slipCurrentAmps }),
    found,
  };
}

/** The planning constants in Precision-Drive's AccelerationLimiter.java; throws when it has none of them. */
export function parseAccelerationLimiter(text: string): LimiterImport {
  const { code, value, found } = reader(text);
  const frictionCoefficient = value("MU", null);
  const massKg = value("ROBOT_MASS", null);
  const centerOfMassHeightMeters = value("COM_HEIGHT", null);
  const statorCurrentLimitAmps = value("MODEL_CURRENT", null);
  const maxAngularRadPerSecSquared = value("ANGULAR_ACCEL", null);
  // TRACTION_ACCEL = 0.9 * Math.min(MU * G, ...): the share is the number before Math.min
  const share = /\bTRACTION_ACCEL\s*=\s*([\d.]+)\s*\*\s*Math\.min\b/.exec(code);
  const tractionShare = share ? Number(share[1]) : undefined;
  if (tractionShare !== undefined) found.push("TRACTION_ACCEL");
  const driveMotor = motorOf(code);
  if (driveMotor) found.push("DRIVE_MOTOR");
  if (found.length === 0) throw new RangeError("no AccelerationLimiter constants found (MU, ROBOT_MASS, COM_HEIGHT, MODEL_CURRENT, ANGULAR_ACCEL)");
  return {
    kind: "limiter",
    ...(positive(frictionCoefficient) && { frictionCoefficient }),
    ...(positive(massKg) && { massKg }),
    ...(centerOfMassHeightMeters !== undefined && centerOfMassHeightMeters >= 0 && { centerOfMassHeightMeters }),
    ...(positive(statorCurrentLimitAmps) && { statorCurrentLimitAmps }),
    ...(positive(maxAngularRadPerSecSquared) && { maxAngularRadPerSecSquared }),
    ...(tractionShare !== undefined && tractionShare > 0 && tractionShare <= 1 && { tractionShare }),
    ...(driveMotor && { driveMotor }),
    found,
  };
}

/** The file without comments, and a reader of `NAME = value;` that notes each name it finds. */
function reader(text: string) {
  const code = text.replace(/\/\*[\s\S]*?\*\//g, "").replace(/\/\/[^\n]*/g, "");
  const found: string[] = [];
  const value = (name: string, units: Readonly<Record<string, number>> | null): number | undefined => {
    const m = new RegExp(`\\b${name}\\s*=\\s*([^;]+);`).exec(code);
    if (!m) return undefined;
    const v = units ? withUnit(m[1]!.trim(), units) : arithmetic(m[1]!.trim());
    if (v === undefined || !Number.isFinite(v)) return undefined;
    found.push(name);
    return v;
  };
  return { code, value, found };
}

/** DRIVE_MOTOR = new DCMotor(volts, stall torque, stall current, free current, free speed rad/s, motors). */
function motorOf(code: string): Motor | undefined {
  const m = /\bDRIVE_MOTOR\s*=\s*new\s+DCMotor\s*\(([\s\S]*?)\)\s*;/.exec(code);
  if (!m) return undefined;
  const args = splitArgs(m[1]!);
  if (args.length < 5) return undefined;
  const [volts, torque, stall] = args.slice(0, 3).map((a) => arithmetic(a));
  const rpm = /^Units\.rotationsPerMinuteToRadiansPerSecond\(([\s\S]*)\)$/.exec(args[4]!);
  const free = rpm ? (n(arithmetic(rpm[1]!.trim())) * 2 * Math.PI) / 60 : n(arithmetic(args[4]!));
  if (!positive(volts) || !positive(torque) || !positive(stall) || !positive(free)) return undefined;
  return { nominalVolts: volts, stallTorqueNewtonMeters: torque, stallCurrentAmps: stall, freeSpeedRadPerSec: free };
}

/** Top-level comma-separated arguments (brackets nest). */
function splitArgs(s: string): string[] {
  const out: string[] = [];
  let depth = 0;
  let start = 0;
  for (let i = 0; i < s.length; i++) {
    const c = s[i];
    if (c === "(") depth++;
    else if (c === ")") depth--;
    else if (c === "," && depth === 0) {
      out.push(s.slice(start, i).trim());
      start = i + 1;
    }
  }
  out.push(s.slice(start).trim());
  return out;
}

const n = (v: number | undefined) => (v === undefined ? Number.NaN : v);
const positive = (v: number | undefined): v is number => v !== undefined && v > 0;

/** "Inches.of(2.167)" in metres (or metres per second, or amps); a bare number is taken as SI. */
function withUnit(expr: string, units: Readonly<Record<string, number>>): number | undefined {
  const m = /^(\w+)\.of\(([\s\S]*)\)$/.exec(expr);
  if (!m) return arithmetic(expr);
  const scale = units[m[1]!];
  const v = arithmetic(m[2]!.trim());
  return scale === undefined || v === undefined ? undefined : v * scale;
}

/** + - * / and brackets over plain numbers (Java's 7.36 or 50.0 / 7.0); anything else is undefined. */
function arithmetic(expr: string): number | undefined {
  const tokens = expr.match(/\d+\.?\d*(?:[eE][+-]?\d+)?[dDfF]?|\.\d+(?:[eE][+-]?\d+)?[dDfF]?|[-+*/()]|\S/g) ?? [];
  let i = 0;
  const peek = () => tokens[i];
  const sum = (): number => {
    let v = product();
    while (peek() === "+" || peek() === "-") v = tokens[i++] === "+" ? v + product() : v - product();
    return v;
  };
  const product = (): number => {
    let v = unary();
    while (peek() === "*" || peek() === "/") v = tokens[i++] === "*" ? v * unary() : v / unary();
    return v;
  };
  const unary = (): number => {
    const t = tokens[i++];
    if (t === "-") return -unary();
    if (t === "+") return unary();
    if (t === "(") {
      const v = sum();
      if (tokens[i++] !== ")") return Number.NaN;
      return v;
    }
    return t !== undefined && /^[\d.]/.test(t) ? Number(t.replace(/[dDfF]$/, "")) : Number.NaN;
  };
  const v = sum();
  return i === tokens.length && Number.isFinite(v) ? v : undefined;
}
