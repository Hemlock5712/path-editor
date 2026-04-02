import { VelocityProfile } from './VelocityProfile';

/**
 * Estimates time from the velocity profile.
 * DISPLAY ONLY - not used for robot control (all control is distance-based).
 *
 * Uses trapezoidal integration of dt = ds / v(s) over each profile segment.
 */
export class TimeEstimator {
  readonly cumulativeTime: number[];
  readonly totalTime: number;
  private readonly samples: number[];
  private readonly totalLength: number;
  private readonly numSamples: number;

  constructor(profile: VelocityProfile) {
    this.totalLength = profile.totalLength;
    this.numSamples = profile.velocities.length;
    this.samples = profile.samples;
    this.cumulativeTime = new Array(this.numSamples);
    this.cumulativeTime[0] = 0;

    for (let i = 1; i < this.numSamples; i++) {
      const ds = profile.samples[i] - profile.samples[i - 1];
      const vAvg = (profile.velocities[i] + profile.velocities[i - 1]) / 2;
      const dt = vAvg > 1e-6 ? ds / vAvg : 0;
      this.cumulativeTime[i] = this.cumulativeTime[i - 1] + dt;
    }

    this.totalTime = this.cumulativeTime[this.numSamples - 1];
  }

  /**
   * Get estimated time at distance s (binary search + linear interpolation).
   */
  getTime(s: number): number {
    if (s <= 0) return 0;
    if (s >= this.totalLength) return this.totalTime;

    // Binary search for the interval containing s
    let lo = 0;
    let hi = this.numSamples - 1;
    while (lo + 1 < hi) {
      const mid = (lo + hi) >>> 1;
      if (this.samples[mid] <= s) {
        lo = mid;
      } else {
        hi = mid;
      }
    }

    const sLo = this.samples[lo];
    const sHi = this.samples[hi];
    const sRange = sHi - sLo;
    const frac = sRange > 1e-12 ? (s - sLo) / sRange : 0;

    return this.cumulativeTime[lo] + frac * (this.cumulativeTime[hi] - this.cumulativeTime[lo]);
  }

  /**
   * Get distance at estimated time t (binary search on cumulativeTime + interpolation).
   */
  getDistance(t: number): number {
    if (t <= 0) return 0;
    if (t >= this.totalTime) return this.totalLength;

    // Binary search for the interval containing t
    let lo = 0;
    let hi = this.numSamples - 1;
    while (lo + 1 < hi) {
      const mid = (lo + hi) >>> 1;
      if (this.cumulativeTime[mid] <= t) {
        lo = mid;
      } else {
        hi = mid;
      }
    }

    const tLo = this.cumulativeTime[lo];
    const tHi = this.cumulativeTime[hi];
    const tRange = tHi - tLo;
    const frac = tRange > 1e-12 ? (t - tLo) / tRange : 0;

    const sLo = this.samples[lo];
    const sHi = this.samples[hi];

    return sLo + frac * (sHi - sLo);
  }
}
