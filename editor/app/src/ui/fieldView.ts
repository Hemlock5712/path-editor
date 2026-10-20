// Field coordinates ↔ canvas pixels. Paths are stored for blue; on the red view every point is shown
// through the project's red frame (a half-turn about the centre, or a mirror) and every click comes
// back through it. Both maps are their own inverse, so one function does both directions.

import { flipHeading, flipPoint, type FieldFrame } from "../../../core/src/index.ts";
import type { ViewState } from "../state/editor.ts";

export const FIELD_PADDING_PX = 24;

export class FieldView {
  readonly frame: FieldFrame;
  readonly scale: number; // pixels per metre

  constructor(
    readonly width: number,
    readonly height: number,
    readonly fieldLength: number,
    readonly fieldWidth: number,
    readonly view: ViewState,
    redFrame: Exclude<FieldFrame, "BLUE">,
  ) {
    this.frame = view.alliance === "red" ? redFrame : "BLUE";
    const fit = Math.min((width - 2 * FIELD_PADDING_PX) / fieldLength, (height - 2 * FIELD_PADDING_PX) / fieldWidth);
    this.scale = Math.max(1e-3, fit) * view.zoom;
  }

  /** A blue-frame field point to canvas pixels. */
  toCanvas(x: number, y: number): [number, number] {
    const p = flipPoint(this.frame, { x, y });
    return [this.width / 2 + (p.x - this.view.centerX) * this.scale, this.height / 2 - (p.y - this.view.centerY) * this.scale];
  }

  /** A point in the displayed frame (the real field, as the robot reports it) to canvas pixels. */
  toCanvasRaw(x: number, y: number): [number, number] {
    return [this.width / 2 + (x - this.view.centerX) * this.scale, this.height / 2 - (y - this.view.centerY) * this.scale];
  }

  /** Canvas pixels to the displayed frame (no flip): for obstacles, which are real field features. */
  toRaw(px: number, py: number): [number, number] {
    return [(px - this.width / 2) / this.scale + this.view.centerX, -(py - this.height / 2) / this.scale + this.view.centerY];
  }

  /** A real-field pose (as on the robot, either alliance) in the blue frame the paths use. */
  realToBlue(x: number, y: number, heading: number): [number, number, number] {
    const p = flipPoint(this.frame, { x, y });
    return [p.x, p.y, flipHeading(this.frame, heading)];
  }

  /** Canvas pixels to a blue-frame field point. */
  toField(px: number, py: number): [number, number] {
    const x = (px - this.width / 2) / this.scale + this.view.centerX;
    const y = -(py - this.height / 2) / this.scale + this.view.centerY;
    const p = flipPoint(this.frame, { x, y });
    return [p.x, p.y];
  }

  /** A blue-frame heading as a canvas angle (canvas y points down). */
  toCanvasAngle(heading: number): number {
    return -flipHeading(this.frame, heading);
  }

  /** A canvas direction (dx, dy in pixels) as a blue-frame heading. */
  toFieldHeading(dx: number, dy: number): number {
    return flipHeading(this.frame, Math.atan2(-dy, dx));
  }

  /** The displayed-frame centre after zooming by factor about a canvas point, keeping it fixed. */
  zoomAbout(px: number, py: number, factor: number): { zoom: number; centerX: number; centerY: number } {
    const zoom = Math.min(40, Math.max(0.3, this.view.zoom * factor));
    const k = zoom / this.view.zoom;
    const dx = (px - this.width / 2) / this.scale;
    const dy = -(py - this.height / 2) / this.scale;
    return { zoom, centerX: this.view.centerX + dx - dx / k, centerY: this.view.centerY + dy - dy / k };
  }
}
