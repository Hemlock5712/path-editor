import { describe, it, expect, beforeAll } from 'vitest';
import { SplinePath } from '../SplinePath';
import { VelocityProfile } from '../VelocityProfile';
import { TimeEstimator } from '../TimeEstimator';
import {
  computeAnalytics,
  computeStats,
  buildSortedHeadings,
  interpolateHeadingSorted,
} from '../ProfileAnalytics';
import { DEFAULT_CONSTRAINTS } from '../../types';
import { DEFAULT_SETTINGS } from '../../stores/settingsStore';

const controlPoints = [
  { x: 0, y: 0 },
  { x: 2, y: 3 },
  { x: 5, y: 3 },
  { x: 7, y: 0 },
];

const headingWaypoints = [
  { waypointIndex: 0, degrees: 0 },
  { waypointIndex: 3, degrees: 90 },
];

let path: SplinePath;
let profile: VelocityProfile;
let timeEst: TimeEstimator;

beforeAll(() => {
  path = new SplinePath(controlPoints);
  profile = new VelocityProfile(
    path,
    DEFAULT_CONSTRAINTS,
    headingWaypoints,
    controlPoints.length,
    DEFAULT_SETTINGS
  );
  timeEst = new TimeEstimator(profile);
});

describe('computeAnalytics', () => {
  let analytics: ReturnType<typeof computeAnalytics>;

  beforeAll(() => {
    analytics = computeAnalytics(
      path,
      profile,
      timeEst,
      headingWaypoints,
      controlPoints.length
    );
  });

  it('all arrays have same length as velocity profile', () => {
    const n = profile.velocities.length;
    expect(analytics.distances).toHaveLength(n);
    expect(analytics.times).toHaveLength(n);
    expect(analytics.velocities).toHaveLength(n);
    expect(analytics.curvatures).toHaveLength(n);
    expect(analytics.accelerations).toHaveLength(n);
    expect(analytics.headings).toHaveLength(n);
    expect(analytics.angularVelocities).toHaveLength(n);
  });

  it('distances are monotonically increasing', () => {
    for (let i = 1; i < analytics.distances.length; i++) {
      expect(analytics.distances[i]).toBeGreaterThanOrEqual(
        analytics.distances[i - 1]
      );
    }
  });

  it('times are monotonically increasing', () => {
    for (let i = 1; i < analytics.times.length; i++) {
      expect(analytics.times[i]).toBeGreaterThanOrEqual(analytics.times[i - 1]);
    }
  });

  it('headings are finite when waypoints exist', () => {
    for (const h of analytics.headings) {
      expect(Number.isFinite(h)).toBe(true);
    }
  });

  it('headings without waypoints are NaN', () => {
    const noHeadingAnalytics = computeAnalytics(
      path,
      profile,
      timeEst,
      [],
      controlPoints.length
    );
    for (const h of noHeadingAnalytics.headings) {
      expect(Number.isNaN(h)).toBe(true);
    }
  });
});

describe('computeStats', () => {
  let stats: ReturnType<typeof computeStats>;

  beforeAll(() => {
    stats = computeStats(
      path,
      profile,
      timeEst,
      headingWaypoints,
      controlPoints.length,
      headingWaypoints.length
    );
  });

  it('totalLength matches profile', () => {
    expect(stats.totalLength).toBeCloseTo(profile.totalLength, 3);
  });

  it('estimatedTime matches timeEstimator', () => {
    expect(stats.estimatedTime).toBeCloseTo(timeEst.totalTime, 6);
  });

  it('numControlPoints matches', () => {
    expect(stats.numControlPoints).toBe(controlPoints.length);
  });

  it('maxCurvature is non-negative', () => {
    expect(stats.maxCurvature).toBeGreaterThanOrEqual(0);
  });

  it('averageVelocity is positive', () => {
    expect(stats.averageVelocity).toBeGreaterThan(0);
  });
});

describe('buildSortedHeadings', () => {
  it('sorts by fraction', () => {
    const unsorted = [
      { waypointIndex: 3, degrees: 90 },
      { waypointIndex: 0, degrees: 0 },
    ];
    const sorted = buildSortedHeadings(unsorted, 4);
    expect(sorted[0].frac).toBeLessThan(sorted[1].frac);
  });

  it('converts degrees to radians', () => {
    const sorted = buildSortedHeadings([{ waypointIndex: 0, degrees: 180 }], 4);
    expect(sorted[0].rad).toBeCloseTo(Math.PI);
  });
});

describe('interpolateHeadingSorted', () => {
  const sorted = buildSortedHeadings(headingWaypoints, controlPoints.length);

  it('returns first heading before first waypoint', () => {
    const h = interpolateHeadingSorted(sorted, -0.1);
    expect(h).toBeCloseTo(sorted[0].rad);
  });

  it('returns last heading after last waypoint', () => {
    const h = interpolateHeadingSorted(sorted, 1.1);
    expect(h).toBeCloseTo(sorted[sorted.length - 1].rad);
  });

  it('interpolates between waypoints', () => {
    const h = interpolateHeadingSorted(sorted, 0.5);
    expect(h).toBeGreaterThan(sorted[0].rad);
    expect(h).toBeLessThan(sorted[sorted.length - 1].rad);
  });

  it('returns NaN for empty sorted array', () => {
    expect(Number.isNaN(interpolateHeadingSorted([], 0.5))).toBe(true);
  });
});
