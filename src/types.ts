export interface Point {
  x: number;
  y: number;
}

export interface NamedPoint {
  name: string;
  x: number;
  y: number;
  headingDegrees: number | null;
  mirrorName: string | null;
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
  maxAngularVelocity: number; // rad/s
  maxAngularAcceleration: number; // rad/s²
}

export interface ConstraintZone {
  id: string;
  startWaypointIndex: number;
  endWaypointIndex: number;
  maxVelocity: number;
  maxAcceleration: number;
}

export interface RotationZone {
  id: string;
  startWaypointIndex: number;
  endWaypointIndex: number;
  targetPoint: Point;
}

export interface WaypointFlag {
  id: string;
  waypointIndex: number;
  label: string;
}

export interface NamedPath {
  name: string;
  controlPoints: Point[];
  controlPointRefs: (string | null)[];
  headingWaypoints: HeadingWaypoint[];
  constraints: VelocityConstraints;
  constraintZones: ConstraintZone[];
  rotationZones: RotationZone[];
  waypointFlags: WaypointFlag[];
}

export const DEFAULT_CONSTRAINTS: VelocityConstraints = {
  maxVelocity: 5.0,
  maxAcceleration: 10.791,
  startVelocity: 0,
  endVelocity: 0,
  maxAngularVelocity: 10.0, // ~573 deg/s, typical for swerve base radius ~0.42m
  maxAngularAcceleration: 20.0, // rad/s²
};

// Field dimensions (2026 Reefscape) — defaults, overridden by settingsStore
export const FIELD_WIDTH = 16.54; // meters
export const FIELD_HEIGHT = 8.21; // meters
