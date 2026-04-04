import { describe, it, expect } from 'vitest';
import { shortestArcDiff, lerpAngle } from '../angleUtils';

const PI = Math.PI;

describe('shortestArcDiff', () => {
  it('returns 0 for identical angles', () => {
    expect(shortestArcDiff(1, 1)).toBeCloseTo(0);
  });

  it('returns positive for counter-clockwise', () => {
    expect(shortestArcDiff(0, 1)).toBeCloseTo(1);
  });

  it('returns negative for clockwise', () => {
    expect(shortestArcDiff(1, 0)).toBeCloseTo(-1);
  });

  it('wraps across 0/2PI boundary (short way)', () => {
    // From just below 2PI to just above 0 should be a small positive step
    const diff = shortestArcDiff(2 * PI - 0.1, 0.1);
    expect(diff).toBeCloseTo(0.2);
  });

  it('wraps the other direction across 0/2PI', () => {
    const diff = shortestArcDiff(0.1, 2 * PI - 0.1);
    expect(diff).toBeCloseTo(-0.2);
  });

  it('handles exact PI boundary', () => {
    const diff = shortestArcDiff(0, PI);
    expect(Math.abs(diff)).toBeCloseTo(PI);
  });

  it('handles negative inputs', () => {
    const diff = shortestArcDiff(-PI / 2, PI / 2);
    // At exactly PI distance, sign is implementation-defined
    expect(Math.abs(diff)).toBeCloseTo(PI);
  });
});

describe('lerpAngle', () => {
  it('returns a at t=0', () => {
    expect(lerpAngle(1, 2, 0)).toBeCloseTo(1);
  });

  it('returns b at t=1', () => {
    expect(lerpAngle(1, 2, 1)).toBeCloseTo(2);
  });

  it('returns midpoint at t=0.5', () => {
    expect(lerpAngle(0, PI / 2, 0.5)).toBeCloseTo(PI / 4);
  });

  it('interpolates across 0/2PI boundary via shortest arc', () => {
    const result = lerpAngle(2 * PI - 0.2, 0.2, 0.5);
    // Midpoint should be near 0 (or 2PI)
    expect(Math.abs(shortestArcDiff(0, result))).toBeLessThan(0.01);
  });
});
