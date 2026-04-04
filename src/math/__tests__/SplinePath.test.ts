import { describe, it, expect } from 'vitest';
import { SplinePath } from '../SplinePath';

describe('SplinePath', () => {
  const twoPoints = [
    { x: 0, y: 0 },
    { x: 3, y: 4 },
  ];

  const fourPoints = [
    { x: 0, y: 0 },
    { x: 2, y: 3 },
    { x: 5, y: 3 },
    { x: 7, y: 0 },
  ];

  it('throws for fewer than 2 points', () => {
    expect(() => new SplinePath([{ x: 0, y: 0 }])).toThrow();
  });

  describe('two-point path', () => {
    const path = new SplinePath(twoPoints);

    it('has positive total length', () => {
      expect(path.totalLength).toBeGreaterThan(0);
    });

    it('total length is close to Euclidean distance for a straight line', () => {
      expect(path.totalLength).toBeCloseTo(5, 1);
    });

    it('getPoint(0) is close to first control point', () => {
      const p = path.getPoint(0);
      expect(p.x).toBeCloseTo(0, 1);
      expect(p.y).toBeCloseTo(0, 1);
    });

    it('getPoint(totalLength) is close to last control point', () => {
      const p = path.getPoint(path.totalLength);
      expect(p.x).toBeCloseTo(3, 1);
      expect(p.y).toBeCloseTo(4, 1);
    });

    it('getTangent returns a unit-ish vector', () => {
      const t = path.getTangent(path.totalLength / 2);
      const mag = Math.hypot(t.x, t.y);
      expect(mag).toBeCloseTo(1, 2);
    });

    it('getCurvature returns finite values', () => {
      const k = path.getCurvature(path.totalLength / 2);
      expect(Number.isFinite(k)).toBe(true);
    });
  });

  describe('four-point path', () => {
    const path = new SplinePath(fourPoints);

    it('has positive total length', () => {
      expect(path.totalLength).toBeGreaterThan(0);
    });

    it('total length is longer than straight-line distance end-to-end', () => {
      const straightDist = Math.hypot(7 - 0, 0 - 0);
      expect(path.totalLength).toBeGreaterThan(straightDist);
    });

    it('getPoint(0) is close to first control point', () => {
      const p = path.getPoint(0);
      expect(p.x).toBeCloseTo(0, 1);
      expect(p.y).toBeCloseTo(0, 1);
    });

    it('getPoint(totalLength) is close to last control point', () => {
      const p = path.getPoint(path.totalLength);
      expect(p.x).toBeCloseTo(7, 1);
      expect(p.y).toBeCloseTo(0, 1);
    });

    it('getClosestPointS finds the start for the start point', () => {
      const s = path.getClosestPointS({ x: 0, y: 0 });
      expect(s).toBeLessThan(0.1);
    });

    it('getClosestPointS finds the end for the end point', () => {
      const s = path.getClosestPointS({ x: 7, y: 0 });
      expect(s).toBeGreaterThan(path.totalLength - 0.1);
    });

    it('getArcLengthAtWaypointIndex(0) is 0', () => {
      expect(path.getArcLengthAtWaypointIndex(0)).toBe(0);
    });

    it('getArcLengthAtWaypointIndex(last) is totalLength', () => {
      expect(path.getArcLengthAtWaypointIndex(path.getNumSegments())).toBe(
        path.totalLength
      );
    });

    it('has non-zero curvature somewhere along the curved path', () => {
      let maxCurvature = 0;
      for (let s = 0; s <= path.totalLength; s += path.totalLength / 20) {
        maxCurvature = Math.max(maxCurvature, Math.abs(path.getCurvature(s)));
      }
      expect(maxCurvature).toBeGreaterThan(0.01);
    });
  });
});
