import { Point, FIELD_WIDTH, FIELD_HEIGHT } from '../types';

export interface CanvasTransform {
  zoom: number;
  panX: number; // in field-space meters
  panY: number;
}

export const DEFAULT_TRANSFORM: CanvasTransform = { zoom: 1, panX: 0, panY: 0 };

// Padding around the field in pixels (at zoom=1)
export const FIELD_PADDING = 24;

// Compute effective padding that shrinks gracefully on very small viewports
function effectivePadding(canvasWidth: number): number {
  return Math.min(FIELD_PADDING, canvasWidth * 0.05);
}

// Convert field coordinates to canvas pixel coordinates given canvas dimensions and transform
export function fieldToCanvas(
  p: Point,
  canvasWidth: number,
  canvasHeight: number,
  transform: CanvasTransform
): { cx: number; cy: number } {
  const pad = effectivePadding(canvasWidth);
  const baseScale = (canvasWidth - 2 * pad) / FIELD_WIDTH;
  const scale = baseScale * transform.zoom;
  const cx = pad + (p.x - transform.panX) * scale;
  const cy = pad + (FIELD_HEIGHT - p.y - transform.panY) * scale;
  return { cx, cy };
}

// Convert canvas pixel coordinates back to field coordinates
export function canvasToField(
  cx: number,
  cy: number,
  canvasWidth: number,
  canvasHeight: number,
  transform: CanvasTransform
): Point {
  const pad = effectivePadding(canvasWidth);
  const baseScale = (canvasWidth - 2 * pad) / FIELD_WIDTH;
  const scale = baseScale * transform.zoom;
  return {
    x: (cx - pad) / scale + transform.panX,
    y: FIELD_HEIGHT - (cy - pad) / scale - transform.panY,
  };
}

// Get the scale factor (pixels per meter)
export function getScale(
  canvasWidth: number,
  transform: CanvasTransform
): number {
  const pad = effectivePadding(canvasWidth);
  return ((canvasWidth - 2 * pad) / FIELD_WIDTH) * transform.zoom;
}

// Clamp a field point to field boundaries
export function clampToField(p: Point): Point {
  return {
    x: Math.max(0, Math.min(FIELD_WIDTH, p.x)),
    y: Math.max(0, Math.min(FIELD_HEIGHT, p.y)),
  };
}

// Compute zoom centered on a canvas pixel position
export function zoomAtPoint(
  transform: CanvasTransform,
  cx: number,
  cy: number,
  canvasWidth: number,
  canvasHeight: number,
  zoomDelta: number
): CanvasTransform {
  const oldZoom = transform.zoom;
  const newZoom = Math.max(0.5, Math.min(8, oldZoom * (1 + zoomDelta)));

  // Convert cursor position to field coords with OLD transform
  const fieldPt = canvasToField(cx, cy, canvasWidth, canvasHeight, transform);

  // Compute new pan so fieldPt stays at same canvas position
  const pad = effectivePadding(canvasWidth);
  const baseScale = (canvasWidth - 2 * pad) / FIELD_WIDTH;
  const newScale = baseScale * newZoom;

  return {
    zoom: newZoom,
    panX: fieldPt.x - (cx - pad) / newScale,
    panY: FIELD_HEIGHT - fieldPt.y - (cy - pad) / newScale,
  };
}
