import { describe, it, expect } from 'vitest';
import { SplinePath } from '../SplinePath';
import { VelocityProfile } from '../VelocityProfile';
import { TimeEstimator } from '../TimeEstimator';
import { DEFAULT_CONSTRAINTS } from '../../types';
import { DEFAULT_SETTINGS } from '../../stores/settingsStore';

const controlPoints = [
  { x: 0, y: 0 },
  { x: 2, y: 3 },
  { x: 5, y: 3 },
  { x: 7, y: 0 },
];

const path = new SplinePath(controlPoints);
const profile = new VelocityProfile(
  path,
  DEFAULT_CONSTRAINTS,
  [],
  controlPoints.length,
  DEFAULT_SETTINGS
);
const timeEst = new TimeEstimator(profile);

describe('TimeEstimator', () => {
  it('getTime(0) is 0', () => {
    expect(timeEst.getTime(0)).toBe(0);
  });

  it('totalTime is positive', () => {
    expect(timeEst.totalTime).toBeGreaterThan(0);
  });

  it('getTime(totalLength) equals totalTime', () => {
    expect(timeEst.getTime(profile.totalLength)).toBeCloseTo(
      timeEst.totalTime,
      6
    );
  });

  it('cumulativeTime is monotonically non-decreasing', () => {
    for (let i = 1; i < timeEst.cumulativeTime.length; i++) {
      expect(timeEst.cumulativeTime[i]).toBeGreaterThanOrEqual(
        timeEst.cumulativeTime[i - 1]
      );
    }
  });

  it('getTime is monotonically non-decreasing', () => {
    const steps = 20;
    let prevTime = 0;
    for (let i = 1; i <= steps; i++) {
      const s = (i / steps) * profile.totalLength;
      const t = timeEst.getTime(s);
      expect(t).toBeGreaterThanOrEqual(prevTime);
      prevTime = t;
    }
  });

  it('getDistance(0) is 0', () => {
    expect(timeEst.getDistance(0)).toBe(0);
  });

  it('getDistance(totalTime) is totalLength', () => {
    expect(timeEst.getDistance(timeEst.totalTime)).toBeCloseTo(
      profile.totalLength,
      3
    );
  });

  it('getDistance is inverse of getTime (roundtrip)', () => {
    const s = profile.totalLength * 0.4;
    const t = timeEst.getTime(s);
    const sBack = timeEst.getDistance(t);
    expect(sBack).toBeCloseTo(s, 2);
  });
});
