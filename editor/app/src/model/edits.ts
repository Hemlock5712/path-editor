// Pure edits of one path. Each returns a new EditorPath; the core's make* functions check the
// results the same way the Java records do, so an edit that would make a bad file throws here.

import {
  makeConstraintZone,
  makeFacePointZone,
  makePathEvent,
  makePathSpec,
  makeRotationTarget,
  makeWaypoint,
  type ConstraintZone,
  type FacePointZone,
  type PathEvent,
  type PathSpec,
  type RotationTarget,
} from "../../../core/src/index.ts";
import type { EditorPath } from "./project.ts";
import { afterDelete, afterInsert } from "./positions.ts";

export function withSpec(path: EditorPath, parts: Partial<PathSpec>): EditorPath {
  return { ...path, spec: makePathSpec({ ...path.spec, ...parts }) };
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

export function moveWaypoint(path: EditorPath, i: number, x: number, y: number): EditorPath {
  const w = path.spec.waypoints[i]!;
  return withSpec(path, { waypoints: replaceAt(path.spec.waypoints, i, makeWaypoint(x, y, w.tangentRadians, w.tangentScale)) });
}

/** Sets the tangent direction (NaN = automatic) and scale of waypoint i. */
export function setTangent(path: EditorPath, i: number, radians: number, scale: number): EditorPath {
  const w = path.spec.waypoints[i]!;
  return withSpec(path, { waypoints: replaceAt(path.spec.waypoints, i, makeWaypoint(w.xMeters, w.yMeters, radians, scale)) });
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

/** Inserts a waypoint between k and k + 1, at fraction f (0..1) of that stretch's arc. */
export function insertWaypoint(path: EditorPath, k: number, f: number, x: number, y: number): EditorPath {
  const waypoints = path.spec.waypoints.slice();
  waypoints.splice(k + 1, 0, makeWaypoint(x, y));
  const links = path.links.slice();
  links.splice(k + 1, 0, null);
  const fraction = Math.min(0.999, Math.max(0.001, f));
  return { ...withSpec(path, { ...remapPositions(path.spec, (p) => afterInsert(p, k, fraction)), waypoints }), links };
}

/** Adds a waypoint after the last one (the path grows; nothing is renumbered). */
export function appendWaypoint(path: EditorPath, x: number, y: number): EditorPath {
  return { ...withSpec(path, { waypoints: [...path.spec.waypoints, makeWaypoint(x, y)] }), links: [...path.links, null] };
}

export function deleteWaypoint(path: EditorPath, i: number): EditorPath {
  const n = path.spec.waypoints.length;
  if (n <= 2) throw new RangeError("a path needs at least two waypoints");
  const remapped = remapPositions(path.spec, (p) => afterDelete(p, i, n));
  return { ...withSpec(path, { ...remapped, waypoints: removeAt(path.spec.waypoints, i) }), links: removeAt(path.links, i) };
}

export function linkWaypoint(path: EditorPath, i: number, pointId: string | null): EditorPath {
  return { ...path, links: replaceAt(path.links, i, pointId) };
}

// ------------------------------------------------------------------------------ rotation targets

export function addRotationTarget(path: EditorPath, t: RotationTarget): EditorPath {
  return withSpec(path, { rotationTargets: [...path.spec.rotationTargets, t] });
}

export function updateRotationTarget(path: EditorPath, i: number, t: RotationTarget): EditorPath {
  return withSpec(path, { rotationTargets: replaceAt(path.spec.rotationTargets, i, t) });
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
