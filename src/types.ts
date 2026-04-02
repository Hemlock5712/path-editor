export interface Point {
  x: number;
  y: number;
}

export interface HeadingWaypoint {
  waypointIndex: number;
  degrees: number;
}

export interface VelocityConstraints {
  maxVelocity: number;
  maxAcceleration: number;
  startVelocity: number;
  endVelocity: number;
}

export interface PathJson {
  version: string;
  controlPoints: Point[];
  headingWaypoints?: HeadingWaypoint[];
  constraints: VelocityConstraints;
}

export const DEFAULT_CONSTRAINTS: VelocityConstraints = {
  maxVelocity: 5.0,
  maxAcceleration: 10.791,
  startVelocity: 0,
  endVelocity: 0,
};

// Field dimensions (2026 Reefscape) — defaults, overridden by settingsStore
export const FIELD_WIDTH = 16.54; // meters
export const FIELD_HEIGHT = 8.21; // meters
