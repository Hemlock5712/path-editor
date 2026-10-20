// Blue/red frame maps about the centre origin. Port of lib's FieldFrame.java: paths are authored for
// blue, and on red each pose goes into the blue frame and each output speed comes back. Each map is
// its own inverse, and neither changes arc length, |κ| or the profile.

import { wrap } from "./angles.ts";
import type { Vec2 } from "./spec.ts";

export type FieldFrame = "BLUE" | "RED_HALF_TURN" | "RED_MIRROR";

export interface Pose {
  readonly xMeters: number;
  readonly yMeters: number;
  readonly headingRadians: number;
}

export interface FieldSpeeds {
  readonly vxMetersPerSecond: number;
  readonly vyMetersPerSecond: number;
  readonly omegaRadPerSec: number;
}

/** A point between this frame and blue (either way). */
export function flipPoint(frame: FieldFrame, p: Vec2): Vec2 {
  switch (frame) {
    case "BLUE":
      return p;
    case "RED_HALF_TURN":
      return { x: -p.x, y: -p.y };
    case "RED_MIRROR":
      return { x: -p.x, y: p.y };
  }
}

/** A heading between this frame and blue, wrapped to (-π, π]. */
export function flipHeading(frame: FieldFrame, radians: number): number {
  switch (frame) {
    case "BLUE":
      return radians;
    case "RED_HALF_TURN":
      return wrap(radians + Math.PI);
    case "RED_MIRROR":
      return wrap(Math.PI - radians);
  }
}

export function flipPose(frame: FieldFrame, pose: Pose): Pose {
  const p = flipPoint(frame, { x: pose.xMeters, y: pose.yMeters });
  return { xMeters: p.x, yMeters: p.y, headingRadians: flipHeading(frame, pose.headingRadians) };
}

export function flipSpeeds(frame: FieldFrame, v: FieldSpeeds): FieldSpeeds {
  switch (frame) {
    case "BLUE":
      return v;
    case "RED_HALF_TURN":
      return { vxMetersPerSecond: -v.vxMetersPerSecond, vyMetersPerSecond: -v.vyMetersPerSecond, omegaRadPerSec: v.omegaRadPerSec };
    case "RED_MIRROR":
      return { vxMetersPerSecond: -v.vxMetersPerSecond, vyMetersPerSecond: v.vyMetersPerSecond, omegaRadPerSec: -v.omegaRadPerSec };
  }
}
