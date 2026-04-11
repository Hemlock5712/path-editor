import { describe, it, expect } from 'vitest';
import { SplinePath } from '../SplinePath';
import { VelocityProfile } from '../VelocityProfile';
import { DEFAULT_CONSTRAINTS } from '../../types';
import { DEFAULT_SETTINGS, computeDerived } from '../../stores/settingsStore';

const controlPoints = [
  { x: 0, y: 0 },
  { x: 2, y: 3 },
  { x: 5, y: 3 },
  { x: 7, y: 0 },
];

const path = new SplinePath(controlPoints);

describe('VelocityProfile', () => {
  const profile = new VelocityProfile(
    path,
    DEFAULT_CONSTRAINTS,
    [],
    controlPoints.length,
    DEFAULT_SETTINGS
  );

  it('has matching totalLength', () => {
    expect(profile.totalLength).toBeCloseTo(path.totalLength, 3);
  });

  it('start velocity equals constraint startVelocity', () => {
    expect(profile.getVelocity(0)).toBeCloseTo(
      DEFAULT_CONSTRAINTS.startVelocity || DEFAULT_SETTINGS.minVelocity,
      1
    );
  });

  it('end velocity is close to constraint endVelocity', () => {
    const endV = profile.getVelocity(profile.totalLength);
    expect(endV).toBeCloseTo(
      DEFAULT_CONSTRAINTS.endVelocity || DEFAULT_SETTINGS.minVelocity,
      1
    );
  });

  it('velocity is non-negative throughout', () => {
    for (let i = 0; i < profile.velocities.length; i++) {
      expect(profile.velocities[i]).toBeGreaterThanOrEqual(0);
    }
  });

  it('velocity is bounded by theoretical max velocity', () => {
    const { maxTheoreticalVelocity } = computeDerived(DEFAULT_SETTINGS);
    for (let i = 0; i < profile.velocities.length; i++) {
      expect(profile.velocities[i]).toBeLessThanOrEqual(
        maxTheoreticalVelocity + 0.01
      );
    }
  });

  it('samples array is monotonically increasing', () => {
    for (let i = 1; i < profile.samples.length; i++) {
      expect(profile.samples[i]).toBeGreaterThan(profile.samples[i - 1]);
    }
  });

  it('getVelocity interpolates within range', () => {
    const midS = profile.totalLength / 2;
    const v = profile.getVelocity(midS);
    expect(v).toBeGreaterThan(0);
    const { maxTheoreticalVelocity } = computeDerived(DEFAULT_SETTINGS);
    expect(v).toBeLessThanOrEqual(maxTheoreticalVelocity + 0.01);
  });

  describe('with non-zero start/end velocity', () => {
    const movingConstraints = {
      ...DEFAULT_CONSTRAINTS,
      startVelocity: 1.0,
      endVelocity: 0.5,
    };

    const movingProfile = new VelocityProfile(
      path,
      movingConstraints,
      [],
      controlPoints.length,
      DEFAULT_SETTINGS
    );

    it('start velocity matches constraint', () => {
      expect(movingProfile.getVelocity(0)).toBeCloseTo(1.0, 1);
    });

    it('end velocity is close to constraint', () => {
      expect(movingProfile.getVelocity(movingProfile.totalLength)).toBeCloseTo(
        0.5,
        1
      );
    });
  });

  describe('with constraint zones', () => {
    const zones = [
      {
        id: 'test-zone',
        startWaypointIndex: 1,
        endWaypointIndex: 2,
        maxVelocity: 1.5,
        maxAcceleration: 3.0,
      },
    ];

    const zonedProfile = new VelocityProfile(
      path,
      DEFAULT_CONSTRAINTS,
      [],
      controlPoints.length,
      DEFAULT_SETTINGS,
      zones
    );

    it('velocity in zone is bounded by zone maxVelocity', () => {
      const zoneStartS = path.getArcLengthAtWaypointIndex(1);
      const zoneEndS = path.getArcLengthAtWaypointIndex(2);
      const midS = (zoneStartS + zoneEndS) / 2;
      const v = zonedProfile.getVelocity(midS);
      expect(v).toBeLessThanOrEqual(1.5 + 0.01);
    });
  });
});
