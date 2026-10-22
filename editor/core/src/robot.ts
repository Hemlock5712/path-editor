// The robot numbers the velocity profile plans with. Port of lib's Motor.java and RobotConfig.java.

/** A brushless DC motor's straight-line torque-speed curve. */
export interface Motor {
  readonly nominalVolts: number;
  readonly stallTorqueNewtonMeters: number;
  readonly stallCurrentAmps: number;
  readonly freeSpeedRadPerSec: number;
}

/** Kraken X60 with FOC, from CTRE's dyno data. */
export const KRAKEN_X60_FOC: Motor = {
  nominalVolts: 12.0,
  stallTorqueNewtonMeters: 9.362,
  stallCurrentAmps: 476.1,
  freeSpeedRadPerSec: (5785.0 * 2.0 * Math.PI) / 60.0,
};

export const GRAVITY = 9.81; // m/s²

/**
 * maxAngularRadPerSecSquared: Infinity = only grip and motors cap the planned spin acceleration.
 * maxWheelMetersPerSecond: NaN = the motor's free speed. maxAngularRadPerSec: Infinity = only the
 * wheel-speed cap. frictionCoefficient is the tread's μ (1.5, Spike Grip); centerOfMassHeightMeters 0 =
 * no tipping limit; tipDistanceMeters NaN = a square base (moduleRadiusMeters / √2). Acceleration and
 * cornering are limited to tractionMetersPerSecondSquared (RobotConfig.java).
 */
export interface RobotConfig {
  readonly massKg: number;
  readonly momentOfInertiaKgM2: number;
  readonly moduleRadiusMeters: number;
  readonly wheelRadiusMeters: number;
  readonly driveGearRatio: number;
  readonly driveMotor: Motor;
  readonly driveMotorCount: number;
  readonly statorCurrentLimitAmps: number;
  readonly frictionCoefficient: number;
  readonly maxWheelMetersPerSecond: number;
  readonly maxAngularRadPerSec: number;
  readonly maxAngularRadPerSecSquared: number;
  readonly centerOfMassHeightMeters: number;
  readonly tipDistanceMeters: number;
  readonly tractionRule: TractionRule;
}

/**
 * How the traction limit is worked out (RobotConfig.TractionRule): "tipOrGrip" = 0.9 · min(μ·g, g·x/h);
 * "equalPush" = 0.9 · μ·g / (1 + μ·h/x), every wheel pushing the same so the unloaded ones slip first.
 */
export type TractionRule = "tipOrGrip" | "equalPush";

/** What sets the traction limit (RobotConfig.TractionLimit). */
export type TractionLimit = "grip" | "tipping" | "equalPush";

/** The share of the grip-or-tip limit the robot uses (the rest is margin). */
export const TRACTION_MARGIN = 0.9;

export const DEFAULT_ROBOT: RobotConfig = {
  massKg: 60.0,
  momentOfInertiaKgM2: 6.0,
  moduleRadiusMeters: Math.sqrt(0.29 * 0.29 + 0.29 * 0.29),
  wheelRadiusMeters: 0.0508,
  driveGearRatio: 6.75,
  driveMotor: KRAKEN_X60_FOC,
  driveMotorCount: 4,
  statorCurrentLimitAmps: 120.0,
  frictionCoefficient: 1.5,
  maxWheelMetersPerSecond: Number.NaN,
  maxAngularRadPerSec: Infinity,
  maxAngularRadPerSecSquared: Infinity,
  centerOfMassHeightMeters: 0.2,
  tipDistanceMeters: Number.NaN,
  tractionRule: "equalPush",
};

/** Checks a config the way RobotConfig's constructor does. */
export function checkRobot(r: RobotConfig): RobotConfig {
  if (
    !(r.massKg > 0.0) || !(r.momentOfInertiaKgM2 > 0.0) || !(r.moduleRadiusMeters > 0.0) ||
    !(r.wheelRadiusMeters > 0.0) || !(r.driveGearRatio > 0.0) || r.driveMotorCount < 1 ||
    !(r.statorCurrentLimitAmps > 0.0) || !(r.frictionCoefficient > 0.0) || !(r.maxAngularRadPerSec > 0.0) ||
    !(r.maxAngularRadPerSecSquared > 0.0)
  ) {
    throw new RangeError("robot parameters must be positive");
  }
  if (!Number.isNaN(r.maxWheelMetersPerSecond) && !(r.maxWheelMetersPerSecond > 0.0)) {
    throw new RangeError("maxWheelMetersPerSecond must be positive or NaN");
  }
  if (!(r.centerOfMassHeightMeters >= 0.0) || r.centerOfMassHeightMeters === Infinity) {
    throw new RangeError("centerOfMassHeightMeters must be 0 or more");
  }
  if (!Number.isNaN(r.tipDistanceMeters) && !(r.tipDistanceMeters > 0.0)) {
    throw new RangeError("tipDistanceMeters must be positive or NaN");
  }
  if (r.tractionRule !== "tipOrGrip" && r.tractionRule !== "equalPush") {
    throw new RangeError("unknown tractionRule '" + String(r.tractionRule) + "' (tipOrGrip or equalPush)");
  }
  return r;
}

/** The tread's grip, μ·g, m/s². */
export function gripMetersPerSecondSquared(r: RobotConfig): number {
  return r.frictionCoefficient * GRAVITY;
}

/** The tip distance in use: the given one, or a square base's moduleRadiusMeters / √2. */
export function tipDistanceInUseMeters(r: RobotConfig): number {
  return Number.isNaN(r.tipDistanceMeters) ? r.moduleRadiusMeters / Math.sqrt(2.0) : r.tipDistanceMeters;
}

/** The acceleration that would tip the robot over, g · tip / h, m/s² (Infinity with the centre of mass at 0). */
export function tipMetersPerSecondSquared(r: RobotConfig): number {
  return r.centerOfMassHeightMeters > 0.0 ? (GRAVITY * tipDistanceInUseMeters(r)) / r.centerOfMassHeightMeters : Infinity;
}

/** The "tipOrGrip" limit, 0.9 · min(μ·g, g·x/h), m/s². */
export function tipOrGripMetersPerSecondSquared(r: RobotConfig): number {
  return TRACTION_MARGIN * Math.min(gripMetersPerSecondSquared(r), tipMetersPerSecondSquared(r));
}

/** The "equalPush" limit, 0.9 · μ·g / (1 + μ·h/x), m/s². */
export function equalPushMetersPerSecondSquared(r: RobotConfig): number {
  return (TRACTION_MARGIN * gripMetersPerSecondSquared(r))
    / (1.0 + (r.frictionCoefficient * r.centerOfMassHeightMeters) / tipDistanceInUseMeters(r));
}

/** The most the robot accelerates or corners, by its traction rule, m/s² (planner and limiter). */
export function tractionMetersPerSecondSquared(r: RobotConfig): number {
  return r.tractionRule === "equalPush" ? equalPushMetersPerSecondSquared(r) : tipOrGripMetersPerSecondSquared(r);
}

/** What sets the traction limit (with the centre of mass at 0, grip either way). */
export function tractionLimit(r: RobotConfig): TractionLimit {
  if (!(r.centerOfMassHeightMeters > 0.0)) return "grip";
  if (r.tractionRule === "equalPush") return "equalPush";
  return tipMetersPerSecondSquared(r) < gripMetersPerSecondSquared(r) ? "tipping" : "grip";
}

/** Wheel speed at the motor's free speed, m/s. */
export function freeSpeedMetersPerSecond(r: RobotConfig): number {
  return (r.driveMotor.freeSpeedRadPerSec / r.driveGearRatio) * r.wheelRadiusMeters;
}

export function wheelSpeedCapMetersPerSecond(r: RobotConfig): number {
  return Number.isNaN(r.maxWheelMetersPerSecond) ? freeSpeedMetersPerSecond(r) : r.maxWheelMetersPerSecond;
}

/** The I / (m·r) lever, metres. */
export function spinLeverMeters(r: RobotConfig): number {
  return r.momentOfInertiaKgM2 / (r.massKg * r.moduleRadiusMeters);
}

function forcePerAmp(r: RobotConfig): number {
  const m = r.driveMotor;
  return (r.driveMotorCount * (m.stallTorqueNewtonMeters / m.stallCurrentAmps) * r.driveGearRatio) / r.wheelRadiusMeters;
}

/** Most acceleration the drive motors can add with the wheels rolling at wheelMetersPerSecond. */
export function driveAccelMetersPerSecondSquared(r: RobotConfig, wheelMetersPerSecond: number): number {
  const m = r.driveMotor;
  const motorRadPerSec = (wheelMetersPerSecond / r.wheelRadiusMeters) * r.driveGearRatio;
  const radPerSecPerVolt = m.freeSpeedRadPerSec / m.nominalVolts;
  const resistance = m.nominalVolts / m.stallCurrentAmps;
  let amps = (m.nominalVolts - motorRadPerSec / radPerSecPerVolt) / resistance;
  amps = amps < 0.0 ? 0.0 : amps > r.statorCurrentLimitAmps ? r.statorCurrentLimitAmps : amps;
  return (forcePerAmp(r) * amps) / r.massKg;
}

/** Most braking the drive motors can give (the stator limit). */
export function brakeAccelMetersPerSecondSquared(r: RobotConfig): number {
  return (forcePerAmp(r) * r.statorCurrentLimitAmps) / r.massKg;
}
