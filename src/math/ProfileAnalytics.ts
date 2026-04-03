import { SplinePath } from './SplinePath';
import { VelocityProfile } from './VelocityProfile';
import { TimeEstimator } from './TimeEstimator';
import { HeadingWaypoint } from '../types';
import { shortestArcDiff, lerpAngle } from './angleUtils';

export interface PathStats {
  totalLength: number;
  estimatedTime: number;
  numControlPoints: number;
  numHeadingWaypoints: number;
  maxCurvature: number;
  maxCurvatureDistance: number;
  averageVelocity: number;
  peakAngularVelocity: number;
  peakAngularVelocityDistance: number;
}

export interface AnalyticsArrays {
  distances: number[];
  times: number[];
  velocities: number[];
  curvatures: number[];
  accelerations: number[];
  headings: number[];
  angularVelocities: number[];
}

/**
 * Compute analytics arrays sampled at the same points as the velocity profile.
 */
export function computeAnalytics(
  path: SplinePath,
  profile: VelocityProfile,
  timeEst: TimeEstimator,
  headingWaypoints: HeadingWaypoint[],
  numControlPoints: number,
): AnalyticsArrays {
  const n = profile.velocities.length;
  const distances: number[] = new Array(n);
  const times: number[] = new Array(n);
  const velocities: number[] = new Array(n);
  const curvatures: number[] = new Array(n);
  const accelerations: number[] = new Array(n);
  const headings: number[] = new Array(n);

  const totalLength = profile.totalLength;

  for (let i = 0; i < n; i++) {
    const s = profile.samples[i];
    distances[i] = s;
    times[i] = timeEst.cumulativeTime[i];
    velocities[i] = profile.velocities[i];
    curvatures[i] = path.getCurvature(s);
  }

  // Tangential acceleration: a = (v[i+1]^2 - v[i]^2) / (2 * ds)
  for (let i = 0; i < n; i++) {
    if (i < n - 1) {
      const ds = distances[i + 1] - distances[i];
      if (ds > 1e-12) {
        const vCur = velocities[i];
        const vNext = velocities[i + 1];
        accelerations[i] = (vNext * vNext - vCur * vCur) / (2 * ds);
      } else {
        accelerations[i] = 0;
      }
    } else {
      // Last point: use backward difference
      if (n >= 2) {
        const ds = distances[n - 1] - distances[n - 2];
        if (ds > 1e-12) {
          const vPrev = velocities[n - 2];
          const vCur = velocities[n - 1];
          accelerations[i] = (vCur * vCur - vPrev * vPrev) / (2 * ds);
        } else {
          accelerations[i] = 0;
        }
      } else {
        accelerations[i] = 0;
      }
    }
  }

  // Headings: interpolate from heading waypoints
  if (headingWaypoints.length === 0 || numControlPoints < 2) {
    headings.fill(NaN);
  } else {
    const sorted = buildSortedHeadings(headingWaypoints, numControlPoints);

    for (let i = 0; i < n; i++) {
      const progress = totalLength > 1e-12 ? distances[i] / totalLength : 0;
      headings[i] = interpolateHeadingSorted(sorted, progress);
    }
  }

  // Angular velocities: omega = v * dtheta/ds
  const angularVelocities: number[] = new Array(n);
  const dthetaDs = computeHeadingRate(headingWaypoints, numControlPoints, distances, totalLength);
  for (let i = 0; i < n; i++) {
    angularVelocities[i] = velocities[i] * dthetaDs[i];
  }

  return { distances, times, velocities, curvatures, accelerations, headings, angularVelocities };
}

/**
 * Compute summary statistics from path, profile, and time estimator.
 */
export function computeStats(
  path: SplinePath,
  profile: VelocityProfile,
  timeEst: TimeEstimator,
  headingWaypoints: HeadingWaypoint[],
  numControlPoints: number,
  numHeadingWaypoints: number,
): PathStats {
  const n = profile.velocities.length;

  let maxCurvature = 0;
  let maxCurvatureDistance = 0;
  for (let i = 0; i < n; i++) {
    const s = profile.samples[i];
    const kappa = Math.abs(path.getCurvature(s));

    if (kappa > maxCurvature) {
      maxCurvature = kappa;
      maxCurvatureDistance = s;
    }
  }

  const averageVelocity = timeEst.totalTime > 1e-12
    ? profile.totalLength / timeEst.totalTime
    : 0;

  // Peak angular velocity
  const dthetaDs = computeHeadingRate(headingWaypoints, numControlPoints, profile.samples, profile.totalLength);
  let peakAngularVelocity = 0;
  let peakAngularVelocityDistance = 0;
  for (let i = 0; i < n; i++) {
    const omega = Math.abs(profile.velocities[i] * dthetaDs[i]);
    if (omega > peakAngularVelocity) {
      peakAngularVelocity = omega;
      peakAngularVelocityDistance = profile.samples[i];
    }
  }

  return {
    totalLength: profile.totalLength,
    estimatedTime: timeEst.totalTime,
    numControlPoints,
    numHeadingWaypoints,
    maxCurvature,
    maxCurvatureDistance,
    averageVelocity,
    peakAngularVelocity,
    peakAngularVelocityDistance,
  };
}

// ---- Heading interpolation & rate helpers ----

export interface SortedHeadingEntry {
  frac: number;
  rad: number;
}

/**
 * Sort and convert heading waypoints to fraction/radian entries.
 */
export function buildSortedHeadings(
  headingWaypoints: HeadingWaypoint[],
  numControlPoints: number,
): SortedHeadingEntry[] {
  return headingWaypoints
    .map((hw) => ({
      frac: hw.waypointIndex / (numControlPoints - 1),
      rad: (hw.degrees * Math.PI) / 180,
    }))
    .sort((a, b) => a.frac - b.frac);
}

/**
 * Compute dtheta/ds (radians per meter) at each sample distance.
 * Used by VelocityProfile to enforce angular velocity/acceleration limits.
 */
export function computeHeadingRate(
  headingWaypoints: HeadingWaypoint[],
  numControlPoints: number,
  sampleDistances: number[],
  totalLength: number,
): number[] {
  const n = sampleDistances.length;
  const dthetaDs = new Array<number>(n).fill(0);

  if (headingWaypoints.length < 2 || numControlPoints < 2 || totalLength < 1e-12) {
    return dthetaDs;
  }

  const sorted = buildSortedHeadings(headingWaypoints, numControlPoints);
  if (sorted.length < 2) return dthetaDs;

  // Compute heading at each sample, then finite-difference for dtheta/ds
  const headings = new Array<number>(n);
  for (let i = 0; i < n; i++) {
    const progress = sampleDistances[i] / totalLength;
    headings[i] = interpolateHeadingSorted(sorted, progress);
  }

  for (let i = 0; i < n; i++) {
    let dTheta: number;
    let ds: number;
    if (i < n - 1) {
      ds = sampleDistances[i + 1] - sampleDistances[i];
      dTheta = shortestArcDiff(headings[i], headings[i + 1]);
    } else {
      ds = sampleDistances[i] - sampleDistances[i - 1];
      dTheta = shortestArcDiff(headings[i - 1], headings[i]);
    }
    dthetaDs[i] = ds > 1e-12 ? dTheta / ds : 0;
  }

  return dthetaDs;
}

/**
 * Interpolate heading from pre-sorted heading entries at a given path progress (0-1).
 * Before first waypoint: hold first heading.
 * After last waypoint: hold last heading.
 * Between: linear interpolation with shortest-arc angle lerp.
 */
export function interpolateHeadingSorted(sorted: SortedHeadingEntry[], progress: number): number {
  if (sorted.length === 0) return NaN;

  // Before first or at first waypoint: hold constant
  if (progress <= sorted[0].frac) return sorted[0].rad;

  // After last or at last waypoint: hold constant
  if (progress >= sorted[sorted.length - 1].frac) return sorted[sorted.length - 1].rad;

  // Find surrounding pair and lerp
  for (let i = 0; i < sorted.length - 1; i++) {
    if (progress >= sorted[i].frac && progress <= sorted[i + 1].frac) {
      const rangeFrac = sorted[i + 1].frac - sorted[i].frac;
      const t = rangeFrac > 1e-12 ? (progress - sorted[i].frac) / rangeFrac : 0;
      return lerpAngle(sorted[i].rad, sorted[i + 1].rad, t);
    }
  }

  return sorted[sorted.length - 1].rad;
}
