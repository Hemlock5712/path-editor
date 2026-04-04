import { describe, it, expect } from 'vitest';
import { hitTestControlPoint, hitTestRotationZone } from '../hitTesting';
import { fieldToCanvas, type CanvasTransform } from '../canvasTransform';
import { SplinePath } from '../../math/SplinePath';

const transform: CanvasTransform = { zoom: 1, panX: 0, panY: 0 };
const cw = 1000;
const ch = 500;

describe('hitTestControlPoint', () => {
  const points = [
    { x: 1, y: 1 },
    { x: 5, y: 5 },
    { x: 10, y: 3 },
  ];

  it('returns index when clicking on a point', () => {
    const canvas = fieldToCanvas(points[1], cw, ch, transform);
    const hit = hitTestControlPoint(
      canvas.cx,
      canvas.cy,
      points,
      cw,
      ch,
      transform
    );
    expect(hit).toBe(1);
  });

  it('returns null when clicking empty space', () => {
    const hit = hitTestControlPoint(0, 0, points, cw, ch, transform);
    expect(hit).toBeNull();
  });

  it('returns last point when overlapping (reverse iteration)', () => {
    // Two points at same location
    const overlapping = [
      { x: 3, y: 3 },
      { x: 3, y: 3 },
    ];
    const canvas = fieldToCanvas(overlapping[0], cw, ch, transform);
    const hit = hitTestControlPoint(
      canvas.cx,
      canvas.cy,
      overlapping,
      cw,
      ch,
      transform
    );
    expect(hit).toBe(1);
  });
});

describe('hitTestRotationZone', () => {
  const points = [
    { x: 0, y: 0 },
    { x: 5, y: 5 },
    { x: 10, y: 0 },
  ];
  const spline = new SplinePath(points);

  it('returns null when no zones', () => {
    const hit = hitTestRotationZone(500, 250, [], spline, 3, cw, ch, transform);
    expect(hit).toBeNull();
  });

  it('returns null with null splinePath', () => {
    const hit = hitTestRotationZone(500, 250, [], null, 3, cw, ch, transform);
    expect(hit).toBeNull();
  });

  it('hits target point', () => {
    const target = { x: 5, y: 2 };
    const zones = [
      {
        id: 'z1',
        startWaypointIndex: 0,
        endWaypointIndex: 2,
        targetPoint: target,
      },
    ];
    const canvas = fieldToCanvas(target, cw, ch, transform);
    const hit = hitTestRotationZone(
      canvas.cx,
      canvas.cy,
      zones,
      spline,
      3,
      cw,
      ch,
      transform
    );
    expect(hit).toEqual({ zoneId: 'z1', handle: 'target' });
  });
});
