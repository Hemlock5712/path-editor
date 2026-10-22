// Pure edits of one path. Each returns a new EditorPath; the core's make* functions check the
// results the same way the Java records do, so an edit that would make a bad file throws here.

import {
  makeConstraintZone,
  makeFacePointZone,
  makePathEvent,
  makeRotationTarget,
  makeWaypoint,
  type ConstraintZone,
  type FacePointZone,
  type PathEvent,
  type PathSpec,
  type RotationTarget,
  type Waypoint,
} from "../../../core/src/index.ts";
import { draftSpec, type EditorPath } from "./project.ts";
import { afterDelete, afterInsert, normalAngle } from "./positions.ts";

export function withSpec(path: EditorPath, parts: Partial<PathSpec>): EditorPath {
  return { ...path, spec: draftSpec({ ...path.spec, ...parts }) };
}

function replaceAt<T>(list: readonly T[], i: number, item: T): T[] {
  const out = list.slice();
  out[i] = item;
  return out;
}

function removeAt<T>(list: readonly T[], i: number): T[] {
  return list.filter((_, j) => j !== i);
}

// ------------------------------------------------------------------------------ waypoints

/** A copy of w with some of its fields changed (checked by the core). */
export function changed(w: Waypoint, c: Partial<Waypoint>): Waypoint {
  const n = { ...w, ...c };
  return makeWaypoint(n.xMeters, n.yMeters, n.tangentRadians, n.tangentScale, n.headingRadians);
}

export function moveWaypoint(path: EditorPath, i: number, x: number, y: number): EditorPath {
  const w = path.spec.waypoints[i]!;
  return withSpec(path, { waypoints: replaceAt(path.spec.waypoints, i, changed(w, { xMeters: x, yMeters: y })) });
}

/** Sets the tangent direction (NaN = automatic) and scale of waypoint i. */
export function setTangent(path: EditorPath, i: number, radians: number, scale: number): EditorPath {
  const w = path.spec.waypoints[i]!;
  return withSpec(path, { waypoints: replaceAt(path.spec.waypoints, i, changed(w, { tangentRadians: normalAngle(radians), tangentScale: scale })) });
}

/**
 * Sets the robot's heading at waypoint i, or NaN for a pass-through point. The first and last
 * waypoints always keep one.
 */
export function setHeading(path: EditorPath, i: number, radians: number): EditorPath {
  const n = path.spec.waypoints.length;
  if (Number.isNaN(radians) && (i === 0 || i === n - 1)) throw new RangeError("the first and last waypoints always have a heading");
  const w = path.spec.waypoints[i]!;
  return withSpec(path, { waypoints: replaceAt(path.spec.waypoints, i, changed(w, { headingRadians: normalAngle(radians) })) });
}

export function setPinned(path: EditorPath, i: number, pinned: boolean): EditorPath {
  return { ...path, pinned: replaceAt(path.pinned, i, pinned) };
}

/** Maps every waypoint-relative position in the spec; zones that collapse are dropped. */
export function remapPositions(spec: PathSpec, map: (p: number) => number): Partial<PathSpec> {
  const rotationTargets = spec.rotationTargets.map((t) =>
    Number.isNaN(t.waypoint) ? t : makeRotationTarget(map(t.waypoint), Number.NaN, t.headingRadians),
  );
  const facePointZones: FacePointZone[] = [];
  for (const z of spec.facePointZones) {
    const from = map(z.fromWaypoint);
    const to = map(z.toWaypoint);
    if (to > from) facePointZones.push(makeFacePointZone(from, to, z.point, z.blendMeters));
  }
  const constraintZones: ConstraintZone[] = [];
  for (const z of spec.constraintZones) {
    const from = map(z.fromWaypoint);
    const to = map(z.toWaypoint);
    if (to > from) constraintZones.push(makeConstraintZone(from, to, z.maxMetersPerSecond, z.maxMetersPerSecondSquared));
  }
  const events = spec.events.map((e) => (Number.isNaN(e.waypoint) ? e : makePathEvent(e.name, map(e.waypoint), Number.NaN)));
  return { rotationTargets, facePointZones, constraintZones, events };
}

/** Inserts a pass-through waypoint between k and k + 1, at fraction f (0..1) of that stretch's arc. */
export function insertWaypoint(path: EditorPath, k: number, f: number, x: number, y: number): EditorPath {
  const waypoints = path.spec.waypoints.slice();
  waypoints.splice(k + 1, 0, makeWaypoint(x, y));
  const links = path.links.slice();
  links.splice(k + 1, 0, null);
  const pinned = path.pinned.slice();
  pinned.splice(k + 1, 0, false);
  const fraction = Math.min(0.999, Math.max(0.001, f));
  return { ...withSpec(path, { ...remapPositions(path.spec, (p) => afterInsert(p, k, fraction)), waypoints }), links, pinned };
}

/**
 * Adds a waypoint after the last one: it becomes the end and takes over the end's heading, and the old
 * end becomes a pass-through point (unless it is also the start, which keeps its heading).
 */
export function appendWaypoint(path: EditorPath, x: number, y: number): EditorPath {
  const old = path.spec.waypoints;
  const last = old.length - 1;
  const end = old[last]!;
  const waypoints = old.slice();
  if (last > 0) waypoints[last] = changed(end, { headingRadians: Number.NaN });
  waypoints.push(makeWaypoint(x, y, Number.NaN, 1, end.headingRadians));
  return { ...withSpec(path, { waypoints }), links: [...path.links, null], pinned: [...path.pinned, false] };
}

/**
 * Adds a waypoint at (x, y) right after waypoint i (a double-click with waypoint i selected). After the
 * last one it becomes the new end; on an empty path it is the start, facing 0. Between two, f is where
 * it lands on the old stretch (0 to 1), so things placed by waypoint position keep their place.
 */
export function addWaypointAfter(path: EditorPath, i: number, x: number, y: number, f = 0.5): EditorPath {
  const n = path.spec.waypoints.length;
  if (n === 0) return { ...withSpec(path, { waypoints: [makeWaypoint(x, y, Number.NaN, 1, 0)] }), links: [null], pinned: [false] };
  if (i >= n - 1) return appendWaypoint(path, x, y);
  return insertWaypoint(path, i, f, x, y);
}

/** Deletes waypoint i; if it was an end, the new end inherits its heading when it has none. */
export function deleteWaypoint(path: EditorPath, i: number): EditorPath {
  const n = path.spec.waypoints.length;
  if (n <= 2) throw new RangeError("a path needs at least two waypoints");
  const remapped = remapPositions(path.spec, (p) => afterDelete(p, i, n));
  const gone = path.spec.waypoints[i]!;
  const waypoints = removeAt(path.spec.waypoints, i);
  const heir = i === 0 ? 0 : i === n - 1 ? waypoints.length - 1 : -1;
  if (heir >= 0 && Number.isNaN(waypoints[heir]!.headingRadians)) {
    waypoints[heir] = changed(waypoints[heir]!, { headingRadians: gone.headingRadians });
  }
  return { ...withSpec(path, { ...remapped, waypoints }), links: removeAt(path.links, i), pinned: removeAt(path.pinned, i) };
}

export function linkWaypoint(path: EditorPath, i: number, pointId: string | null): EditorPath {
  return { ...path, links: replaceAt(path.links, i, pointId) };
}

// ------------------------------------------------------------------------------ rotation targets

export function addRotationTarget(path: EditorPath, t: RotationTarget): EditorPath {
  return withSpec(path, { rotationTargets: [...path.spec.rotationTargets, { ...t, headingRadians: normalAngle(t.headingRadians) }] });
}

export function updateRotationTarget(path: EditorPath, i: number, t: RotationTarget): EditorPath {
  return withSpec(path, { rotationTargets: replaceAt(path.spec.rotationTargets, i, { ...t, headingRadians: normalAngle(t.headingRadians) }) });
}

export function removeRotationTarget(path: EditorPath, i: number): EditorPath {
  return withSpec(path, { rotationTargets: removeAt(path.spec.rotationTargets, i) });
}

// ------------------------------------------------------------------------------ zones

export function addFacePointZone(path: EditorPath, z: FacePointZone): EditorPath {
  return withSpec(path, { facePointZones: [...path.spec.facePointZones, z] });
}

export function updateFacePointZone(path: EditorPath, i: number, z: FacePointZone): EditorPath {
  return withSpec(path, { facePointZones: replaceAt(path.spec.facePointZones, i, z) });
}

export function removeFacePointZone(path: EditorPath, i: number): EditorPath {
  return withSpec(path, { facePointZones: removeAt(path.spec.facePointZones, i) });
}

export function addConstraintZone(path: EditorPath, z: ConstraintZone): EditorPath {
  return withSpec(path, { constraintZones: [...path.spec.constraintZones, z] });
}

export function updateConstraintZone(path: EditorPath, i: number, z: ConstraintZone): EditorPath {
  return withSpec(path, { constraintZones: replaceAt(path.spec.constraintZones, i, z) });
}

export function removeConstraintZone(path: EditorPath, i: number): EditorPath {
  return withSpec(path, { constraintZones: removeAt(path.spec.constraintZones, i) });
}

// ------------------------------------------------------------------------------ events

export function addEvent(path: EditorPath, e: PathEvent): EditorPath {
  return withSpec(path, { events: [...path.spec.events, e] });
}

export function updateEvent(path: EditorPath, i: number, e: PathEvent): EditorPath {
  return withSpec(path, { events: replaceAt(path.spec.events, i, e) });
}

export function removeEvent(path: EditorPath, i: number): EditorPath {
  return withSpec(path, { events: removeAt(path.spec.events, i) });
}
