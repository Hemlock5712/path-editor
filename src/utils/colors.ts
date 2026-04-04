/**
 * Shared red→green quality gradient.
 * t=0 → red (bad), t=1 → green (good).
 * Used by all charts for a consistent color scheme.
 */
export function qualityToColor(t: number): string {
  t = Math.min(Math.max(t, 0), 1.0);
  if (t < 0.5) {
    const s = t * 2;
    const r = Math.round(255 - s * 200);
    const g = Math.round(60 + s * 195);
    const b = Math.round(60 + s * 50);
    return `rgb(${r}, ${g}, ${b})`;
  } else {
    const s = (t - 0.5) * 2;
    const r = Math.round(55 - s * 55);
    const g = 255;
    const b = Math.round(110 + s * 60);
    return `rgb(${r}, ${g}, ${b})`;
  }
}

/** Curvature value at which the color gradient is fully red (1/meters). */
const CURVATURE_NORMALIZATION = 1.5;

/** Curvature color: green (low/good) → red (high/bad) */
export function curvatureToColor(curvature: number): string {
  const absK = Math.abs(curvature);
  const t = Math.min(absK / CURVATURE_NORMALIZATION, 1.0);
  return qualityToColor(1 - t); // Invert: low curvature = green, high = red
}

/** Velocity color: red (slow/bad) → green (fast/good) */
export function velocityToColor(velocity: number, maxVelocity: number): string {
  const t = Math.min(Math.max(velocity / maxVelocity, 0), 1.0);
  return qualityToColor(t);
}

// Acceleration uses the gradient endpoints directly
// Green for positive (speeding up = good), red for negative (braking = caution)
export const ACCEL_GREEN = 'rgb(0, 255, 170)';
export const ACCEL_GREEN_FILL = 'rgba(0, 255, 170, 0.3)';
export const ACCEL_RED = 'rgb(255, 60, 60)';
export const ACCEL_RED_FILL = 'rgba(255, 60, 60, 0.3)';

// Heading uses the "good" end of the gradient (green)
export const HEADING_COLOR = 'rgb(0, 255, 170)';

// ─── Outline helpers for field visibility ────────────────────────────────────

export const OUTLINE_COLOR = 'rgba(0, 0, 0, 0.55)';

/** Draw text with a dark halo outline for readability over any background. */
export function drawOutlinedText(
  ctx: CanvasRenderingContext2D,
  text: string,
  x: number,
  y: number,
  fillColor: string,
  outlineWidth = 2.5
): void {
  ctx.save();
  ctx.lineJoin = 'round';
  ctx.strokeStyle = OUTLINE_COLOR;
  ctx.lineWidth = outlineWidth;
  ctx.strokeText(text, x, y);
  ctx.fillStyle = fillColor;
  ctx.fillText(text, x, y);
  ctx.restore();
}
