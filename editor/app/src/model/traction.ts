// The robot's traction in words: the limit the plan and the follower use (the core's
// tractionMetersPerSecondSquared, by the robot's traction rule) and what sets it, both rules' values, and
// the Sim preview floor's cap.

import {
  equalPushMetersPerSecondSquared,
  GRAVITY,
  gripMetersPerSecondSquared,
  tipDistanceInUseMeters,
  tipMetersPerSecondSquared,
  tipOrGripMetersPerSecondSquared,
  TRACTION_MARGIN,
  tractionLimit,
  tractionMetersPerSecondSquared,
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

const SETS_IT = { grip: "grip limits first", tipping: "tipping limits first", equalPush: "the front wheels slip first" } as const;

/** "Traction limit 12.8 m/s², tipping limits first". */
export function tractionHeadline(r: RobotConfig): string {
  return `Traction limit ${ms2(tractionMetersPerSecondSquared(r))}, ${SETS_IT[tractionLimit(r)]}`;
}

/** When to switch from equal push (the default) to tip or grip. */
export const TIP_OR_GRIP_ADVICE = "Faster in corners; try it once the centre-of-mass height is measured and slip is checked on the robot with vision";

/** The tip-or-grip rule's working: 0.9 × the lower of grip and tipping. */
export function tipOrGripWorking(r: RobotConfig): string {
  const grip = `grip μ·g = ${ms2(gripMetersPerSecondSquared(r))}`;
  if (!(r.centerOfMassHeightMeters > 0)) return `Tip or grip ${ms2(tipOrGripMetersPerSecondSquared(r))}: ${TRACTION_MARGIN} × ${grip} (no tipping limit with the centre of mass at 0).`;
  const tip = `tipping g × tip ${tipDistanceInUseMeters(r).toFixed(2)} m ÷ height ${r.centerOfMassHeightMeters.toFixed(2)} m = ${ms2(tipMetersPerSecondSquared(r))}`;
  return `Tip or grip ${ms2(tipOrGripMetersPerSecondSquared(r))}: ${TRACTION_MARGIN} × the lower of ${grip} and ${tip}.`;
}

/** The equal-push rule's value and what it means. */
export function equalPushWorking(r: RobotConfig): string {
  return `Equal push ${ms2(equalPushMetersPerSecondSquared(r))}: ${TRACTION_MARGIN} × μ·g ÷ (1 + μ·h/tip); every wheel pushes the same, so under load transfer the front wheels slip first.`;
}

/**
 * The Sim preview floor's cap on the chassis's acceleration, m/s²: the most a floor of tread μ gives the
 * whole robot before it slides or tips, min(μ·g, g·tip/h), with the robot's centre of mass and tip distance,
 * no planning margin, and whichever traction rule the plan uses. The sim moves the chassis as one body, so
 * the bench plant's μ (each wheel's tread on carpet) and centre of mass height (weight transfer) carry over
 * only as this ceiling.
 */
export function simFloorCapMetersPerSecondSquared(mu: number, r: RobotConfig): number {
  return Math.min(mu * GRAVITY, tipMetersPerSecondSquared(r));
}

/**
 * Where the lightly loaded wheels start slipping on that floor when all four push equally, m/s²:
 * μ·g / (1 + μ·h/tip), the equal-push rule without its margin. The sim doesn't model wheel slip, so it
 * doesn't show this.
 */
export function equalPushSlipMetersPerSecondSquared(mu: number, r: RobotConfig): number {
  return equalPushMetersPerSecondSquared({ ...r, frictionCoefficient: mu }) / TRACTION_MARGIN;
}
