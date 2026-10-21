// Named points (linked targets): field points shared across paths. A waypoint linked to one always
// sits on it, so moving the point moves every linked waypoint in every path.

import { makeWaypoint } from "../../../core/src/index.ts";
import { withSpec } from "./edits.ts";
import { transformPoint } from "./mirror.ts";
import { newId, type NamedPoint, type Project } from "./project.ts";

/** Moves a named point and every waypoint linked to it, and its twin (if it has one) to match. */
export function moveNamedPoint(project: Project, id: string, x: number, y: number): Project {
  const twin = project.namedPoints.find((p) => p.id === id)?.twin;
  const moved = moveOne(project, id, x, y);
  if (!twin || !moved.namedPoints.some((p) => p.id === twin.id)) return moved;
  const [tx, ty] = transformPoint(twin.transform, x, y);
  return moveOne(moved, twin.id, tx, ty);
}

function moveOne(project: Project, id: string, x: number, y: number): Project {
  const namedPoints = project.namedPoints.map((p) => (p.id === id ? { ...p, x, y } : p));
  const paths = project.paths.map((path) => {
    if (!path.links.includes(id)) return path;
    const waypoints = path.spec.waypoints.map((w, i) =>
      path.links[i] === id ? makeWaypoint(x, y, w.tangentRadians, w.tangentScale, w.headingRadians) : w,
    );
    return withSpec(path, { waypoints });
  });
  return { ...project, namedPoints, paths };
}

/** Links waypoint i of a path to a point, moving the waypoint onto it. Null unlinks. */
export function linkToPoint(project: Project, pathId: string, i: number, pointId: string | null): Project {
  const point = project.namedPoints.find((p) => p.id === pointId);
  const paths = project.paths.map((path) => {
    if (path.id !== pathId) return path;
    const links = path.links.map((l, j) => (j === i ? pointId : l));
    if (!point) return { ...path, links };
    const w = path.spec.waypoints[i]!;
    const waypoints = path.spec.waypoints.map((v, j) => (j === i ? makeWaypoint(point.x, point.y, w.tangentRadians, w.tangentScale, w.headingRadians) : v));
    return { ...withSpec(path, { waypoints }), links };
  });
  return { ...project, paths };
}

export function addNamedPoint(project: Project, name: string, x: number, y: number, heading = Number.NaN): [Project, NamedPoint] {
  const point: NamedPoint = { id: newId("pt"), name, x, y, heading };
  return [{ ...project, namedPoints: [...project.namedPoints, point] }, point];
}

export function setPointHeading(project: Project, id: string, heading: number): Project {
  return { ...project, namedPoints: project.namedPoints.map((p) => (p.id === id ? { ...p, heading } : p)) };
}

export function renameNamedPoint(project: Project, id: string, name: string): Project {
  return { ...project, namedPoints: project.namedPoints.map((p) => (p.id === id ? { ...p, name } : p)) };
}

/** Removes a point; waypoints that followed it stay where they are, unlinked (and its twin, if any, is unlinked). */
export function removeNamedPoint(project: Project, id: string): Project {
  return {
    ...project,
    namedPoints: project.namedPoints.filter((p) => p.id !== id).map((p) => (p.twin?.id === id ? { ...p, twin: null } : p)),
    paths: project.paths.map((p) => (p.links.includes(id) ? { ...p, links: p.links.map((l) => (l === id ? null : l)) } : p)),
  };
}

/** How many waypoints, across all paths, follow a point. */
export function linkCount(project: Project, id: string): number {
  let n = 0;
  for (const p of project.paths) for (const l of p.links) if (l === id) n++;
  return n;
}

export function uniquePointName(project: Project, base = "Point"): string {
  const taken = new Set(project.namedPoints.map((p) => p.name));
  for (let i = 1; ; i++) if (!taken.has(`${base} ${i}`)) return `${base} ${i}`;
}
