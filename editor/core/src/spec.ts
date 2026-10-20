// The authored inputs of a path, before any maths. Port of lib's Waypoint, RotationTarget,
// FacePointZone, ConstraintZone, PathEvent and PathSpec records; the constructors' checks are in the
// make* functions.

/** A point or vector on the field, metres. */
export interface Vec2 {
  readonly x: number;
  readonly y: number;
}

/**
 * A point the path passes through. tangentRadians is the direction of travel, or NaN to choose it
 * automatically. tangentScale scales the end speed |p'| of the neighbouring spline pieces (1 =
 * default; above 1 holds the direction longer).
 */
export interface Waypoint {
  readonly xMeters: number;
  readonly yMeters: number;
  readonly tangentRadians: number;
  readonly tangentScale: number;
}

/**
 * A robot heading to reach at a place on the path: waypoint-relative (1.5 = halfway along the arc
 * between waypoints 1 and 2) or metres along the path. Exactly one is set; the other is NaN.
 */
export interface RotationTarget {
  readonly waypoint: number;
  readonly meters: number;
  readonly headingRadians: number;
}

/** A stretch where the robot faces a point, easing in and out over blendMeters. */
export interface FacePointZone {
  readonly fromWaypoint: number;
  readonly toWaypoint: number;
  readonly point: Vec2;
  readonly blendMeters: number;
}

/** A stretch with lower limits (either may be Infinity). */
export interface ConstraintZone {
  readonly fromWaypoint: number;
  readonly toWaypoint: number;
  readonly maxMetersPerSecond: number;
  readonly maxMetersPerSecondSquared: number;
}

/**
 * A named event at a place on the path: waypoint-relative (1.5 = halfway along the arc between
 * waypoints 1 and 2) or metres along the path. Exactly one is set; the other is NaN.
 */
export interface PathEvent {
  readonly name: string;
  readonly waypoint: number;
  readonly meters: number;
}

/** Everything a path file says. */
export interface PathSpec {
  readonly waypoints: readonly Waypoint[];
  readonly rotationTargets: readonly RotationTarget[];
  readonly facePointZones: readonly FacePointZone[];
  readonly constraintZones: readonly ConstraintZone[];
  readonly maxMetersPerSecond: number;
  readonly maxMetersPerSecondSquared: number;
  readonly startMetersPerSecond: number;
  readonly endMetersPerSecond: number;
  /** Named events, in file order (the follower fires them in path order). */
  readonly events: readonly PathEvent[];
}

export const DEFAULT_BLEND_METERS = 0.5;

function fail(message: string): never {
  throw new RangeError(message);
}

export function makeWaypoint(
  xMeters: number,
  yMeters: number,
  tangentRadians = Number.NaN,
  tangentScale = 1.0,
): Waypoint {
  if (!Number.isFinite(xMeters) || !Number.isFinite(yMeters)) fail("waypoint position must be finite");
  if (!(tangentScale > 0.0) || !Number.isFinite(tangentScale)) {
    fail(`tangentScale must be positive, got ${tangentScale}`);
  }
  if (tangentRadians === Infinity || tangentRadians === -Infinity) fail("tangent must be finite or NaN");
  return { xMeters, yMeters, tangentRadians, tangentScale };
}

export function hasTangent(w: Waypoint): boolean {
  return !Number.isNaN(w.tangentRadians);
}

export function makeRotationTarget(waypoint: number, meters: number, headingRadians: number): RotationTarget {
  if (Number.isNaN(waypoint) === Number.isNaN(meters)) fail("set exactly one of waypoint or meters");
  if (!Number.isFinite(headingRadians)) fail("heading must be finite");
  return { waypoint, meters, headingRadians };
}

export function makeFacePointZone(
  fromWaypoint: number,
  toWaypoint: number,
  point: Vec2,
  blendMeters = DEFAULT_BLEND_METERS,
): FacePointZone {
  if (!(toWaypoint > fromWaypoint)) fail("face-point zone must end after it starts");
  if (!(blendMeters >= 0.0) || !Number.isFinite(blendMeters)) fail("blendMeters must be >= 0");
  return { fromWaypoint, toWaypoint, point, blendMeters };
}

export function makeConstraintZone(
  fromWaypoint: number,
  toWaypoint: number,
  maxMetersPerSecond = Infinity,
  maxMetersPerSecondSquared = Infinity,
): ConstraintZone {
  if (!(toWaypoint > fromWaypoint)) fail("constraint zone must end after it starts");
  if (!(maxMetersPerSecond > 0.0) || !(maxMetersPerSecondSquared > 0.0)) fail("zone limits must be positive");
  return { fromWaypoint, toWaypoint, maxMetersPerSecond, maxMetersPerSecondSquared };
}

export function makePathEvent(name: string, waypoint: number, meters: number): PathEvent {
  if (name.trim() === "") fail("an event needs a name");
  if (Number.isNaN(waypoint) === Number.isNaN(meters)) fail("set exactly one of atWaypoint or atMeters");
  if (Math.abs(waypoint) === Infinity || Math.abs(meters) === Infinity) fail("event position must be finite");
  return { name, waypoint, meters };
}

export function eventAtWaypoint(name: string, waypoint: number): PathEvent {
  return makePathEvent(name, waypoint, Number.NaN);
}

export function eventAtMeters(name: string, meters: number): PathEvent {
  return makePathEvent(name, Number.NaN, meters);
}

export function eventByWaypoint(e: PathEvent): boolean {
  return !Number.isNaN(e.waypoint);
}

/** A checked spec; unspecified parts default to none, no caps, from rest to rest. */
export function makePathSpec(parts: Partial<PathSpec> & { waypoints: readonly Waypoint[] }): PathSpec {
  const spec: PathSpec = {
    waypoints: parts.waypoints,
    rotationTargets: parts.rotationTargets ?? [],
    facePointZones: parts.facePointZones ?? [],
    constraintZones: parts.constraintZones ?? [],
    maxMetersPerSecond: parts.maxMetersPerSecond ?? Infinity,
    maxMetersPerSecondSquared: parts.maxMetersPerSecondSquared ?? Infinity,
    startMetersPerSecond: parts.startMetersPerSecond ?? 0.0,
    endMetersPerSecond: parts.endMetersPerSecond ?? 0.0,
    events: parts.events ?? [],
  };
  if (spec.waypoints.length < 2) fail("a path needs at least two waypoints");
  if (!(spec.maxMetersPerSecond > 0.0) || !(spec.maxMetersPerSecondSquared > 0.0)) {
    fail("path limits must be positive");
  }
  if (!(spec.startMetersPerSecond >= 0.0) || !(spec.endMetersPerSecond >= 0.0)) {
    fail("start and end speeds must be >= 0");
  }
  return spec;
}
