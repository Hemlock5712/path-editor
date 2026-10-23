// The robot's traction in words: the limit the plan and the follower use (the core's
// tractionMetersPerSecondSquared, AccelerationLimiter's TRACTION_ACCEL) and what sets it, and the Sim
// preview floor's cap.

import {
  GRAVITY,
  gripMetersPerSecondSquared,
  tipDistanceInUseMeters,
  tipMetersPerSecondSquared,
  tractionLimit,
  tractionMetersPerSecondSquared,
  wheelSlipOnsetMetersPerSecondSquared,
  type RobotConfig,
} from "../../../core/src/index.ts";

/** SDS MK5 Spike Grip tread on carpet. */
export const SPIKE_GRIP_MU = 1.5;
/** The planning μ robot files had before the tread μ (carpet grip with a margin built in). */
export const OLD_DEFAULT_MU = 0.7;

/** Whether this robot still has the old 0.7 planning default for μ. */
export function hasOldDefaultMu(r: RobotConfig): boolean {
  return Math.abs(r.frictionCoefficient - OLD_DEFAULT_MU) < 1e-9;
}

const ms2 = (v: number) => `${v.toFixed(1)} m/s²`;

const SETS_IT = { grip: "grip limits first", tipping: "tipping limits first" } as const;

/** "Traction limit 12.8 m/s², tipping limits first". */
export function tractionHeadline(r: RobotConfig): string {
  return `Traction limit ${ms2(tractionMetersPerSecondSquared(r))}, ${SETS_IT[tractionLimit(r)]}`;
}

/** The working: the share of the lower of grip and tipping. */
export function tractionWorking(r: RobotConfig): string {
  const share = `${Math.round(r.tractionShare * 100)} %`;
  const grip = `grip μ·g = ${ms2(gripMetersPerSecondSquared(r))}`;
  if (!(r.centerOfMassHeightMeters > 0)) return `${ms2(tractionMetersPerSecondSquared(r))} = ${share} of ${grip} (no tipping limit with the centre of mass at 0).`;
  const tip = `tipping g × tip ${tipDistanceInUseMeters(r).toFixed(2)} m ÷ height ${r.centerOfMassHeightMeters.toFixed(2)} m = ${ms2(tipMetersPerSecondSquared(r))}`;
  return `${ms2(tractionMetersPerSecondSquared(r))} = ${share} of the lower of ${grip} and ${tip}.`;
}

/**
 * The Sim preview floor's cap on the chassis's acceleration, m/s²: the most a floor of tread μ gives the
 * whole robot before it slides or tips, min(μ·g, g·tip/h), with the robot's centre of mass and tip distance,
 * no planning margin. The sim moves the chassis as one body, so
 * the bench plant's μ (each wheel's tread on carpet) and centre of mass height (weight transfer) carry over
 * only as this ceiling.
 */
export function simFloorCapMetersPerSecondSquared(mu: number, r: RobotConfig): number {
  return Math.min(mu * GRAVITY, tipMetersPerSecondSquared(r));
}

/**
 * Where the lightly loaded wheels start slipping on that floor when all four push equally, m/s²:
 * μ·g / (1 + μ·h/tip). The sim doesn't model wheel slip, so it doesn't show this.
 */
export function wheelSlipOnsetOnFloor(mu: number, r: RobotConfig): number {
  return wheelSlipOnsetMetersPerSecondSquared({ ...r, frictionCoefficient: mu });
}
