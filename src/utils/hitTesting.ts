import { Point, RotationZone } from '../types';
import { fieldToCanvas, type CanvasTransform } from './canvasTransform';
import { SplinePath } from '../math/SplinePath';

/** Find which control point (if any) is at the given canvas position. */
export function hitTestControlPoint(
  canvasX: number,
  canvasY: number,
  controlPoints: Point[],
  cw: number,
  ch: number,
  transform: CanvasTransform
): number | null {
  const hitRadius = Math.max(10, Math.min(18, 14 / Math.sqrt(transform.zoom)));

  for (let i = controlPoints.length - 1; i >= 0; i--) {
    const c = fieldToCanvas(controlPoints[i], cw, ch, transform);
    const dx = canvasX - c.cx;
    const dy = canvasY - c.cy;
    if (dx * dx + dy * dy <= hitRadius * hitRadius) {
      return i;
    }
  }
  return null;
}

/** Find which rotation zone handle or target (if any) is at the given canvas position. */
export function hitTestRotationZone(
  canvasX: number,
  canvasY: number,
  rotationZones: RotationZone[],
  splinePath: SplinePath | null,
  numControlPoints: number,
  cw: number,
  ch: number,
  transform: CanvasTransform
): { zoneId: string; handle: 'start' | 'end' | 'target' } | null {
  if (!splinePath || splinePath.totalLength <= 0) return null;
  const hitRadius = Math.max(12, Math.min(20, 16 / Math.sqrt(transform.zoom)));

  for (const zone of rotationZones) {
    // Check target point first (highest priority)
    const tc = fieldToCanvas(zone.targetPoint, cw, ch, transform);
    if (
      (canvasX - tc.cx) ** 2 + (canvasY - tc.cy) ** 2 <=
      hitRadius * hitRadius
    ) {
      return { zoneId: zone.id, handle: 'target' };
    }

    // Check boundary handles
    if (numControlPoints <= 1) continue;
    for (const handle of ['start', 'end'] as const) {
      const idx =
        handle === 'start' ? zone.startWaypointIndex : zone.endWaypointIndex;
      const frac = idx / (numControlPoints - 1);
      const s = frac * splinePath.totalLength;
      const pt = splinePath.getPoint(s);
      const pc = fieldToCanvas(pt, cw, ch, transform);
      if (
        (canvasX - pc.cx) ** 2 + (canvasY - pc.cy) ** 2 <=
        hitRadius * hitRadius
      ) {
        return { zoneId: zone.id, handle };
      }
    }
  }
  return null;
}
