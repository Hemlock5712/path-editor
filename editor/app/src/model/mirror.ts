// Flip and mirror: a path turned into the other alliance's or the other side's, with everything on it
// transformed the same way (waypoints, headings, tangents, rotation targets, face points). Things placed
// along the path (events, zones) keep their places, since the path's length doesn't change.
//
// Named points get twins: mirroring a path that uses "Score L" links the copy to "Score L (mirror)",
// made the first time and reused after, and the two stay in step (moving one moves the other). A path
// can be copied as a linked mirror too: the copy follows its source and can't be edited on its own.
// Both links live in the sidecar; Unlink breaks either.

import { makeWaypoint, type PathSpec } from "../../../core/src/index.ts";
import { round5, round5Radians } from "./positions.ts";
import { makeEditorPath, newId, uniquePathName, type EditorPath, type NamedPoint, type Project } from "./project.ts";

export type Transform = "flip" | "mirrorTopBottom" | "mirrorLeftRight";

export const TRANSFORMS: readonly Transform[] = ["flip", "mirrorTopBottom", "mirrorLeftRight"];

export const TRANSFORM_LABELS: Record<Transform, string> = {
  flip: "Flip (half-turn about the centre, blue ↔ red)",
  mirrorTopBottom: "Mirror top ↔ bottom",
  mirrorLeftRight: "Mirror left ↔ right",
};

/** What a copy's name and a twin's name get. */
const SUFFIX: Record<Transform, { path: string; point: string }> = {
  flip: { path: "Flip", point: "flip" },
  mirrorTopBottom: { path: "Mirror", point: "mirror" },
  mirrorLeftRight: { path: "MirrorLR", point: "mirror LR" },
};

/** A link between two things that are each other's transform. */
export interface Twin {
  readonly id: string;
  readonly transform: Transform;
}

/** Where a field point goes (each transform is its own inverse). */
export function transformPoint(t: Transform, x: number, y: number): [number, number] {
  // (+ 0 turns -0 into 0, so a mirrored 0 is written as 0)
  const r = (v: number) => round5(v) + 0;
  switch (t) {
    case "flip":
      return [r(-x), r(-y)];
    case "mirrorTopBottom":
      return [r(x), r(-y)];
    case "mirrorLeftRight":
      return [r(-x), r(y)];
  }
}

/** Where a direction goes (NaN, "none", stays NaN), in (-π, π]. */
export function transformAngle(t: Transform, a: number): number {
  if (Number.isNaN(a)) return a;
  const b = t === "flip" ? a + Math.PI : t === "mirrorTopBottom" ? -a : Math.PI - a;
  const wrapped = Math.atan2(Math.sin(b), Math.cos(b));
  return round5Radians(wrapped === -Math.PI ? Math.PI : wrapped);
}

/** The spec turned by t: positions, tangents, headings, rotation targets and face points. */
export function transformSpec(spec: PathSpec, t: Transform): PathSpec {
  return {
    ...spec,
    waypoints: spec.waypoints.map((w) => {
      const [x, y] = transformPoint(t, w.xMeters, w.yMeters);
      return makeWaypoint(x, y, transformAngle(t, w.tangentRadians), w.tangentScale, transformAngle(t, w.headingRadians));
    }),
    rotationTargets: spec.rotationTargets.map((r) => ({ ...r, headingRadians: transformAngle(t, r.headingRadians) })),
    facePointZones: spec.facePointZones.map((z) => {
      const [x, y] = transformPoint(t, z.point.x, z.point.y);
      return { ...z, point: { x, y } };
    }),
  };
}

/** The twin of a named point under t: the one it already has, or a new one. */
export function twinOf(project: Project, id: string, t: Transform): [Project, string] {
  const p = project.namedPoints.find((q) => q.id === id);
  if (!p) return [project, id];
  if (p.twin?.transform === t && project.namedPoints.some((q) => q.id === p.twin!.id)) return [project, p.twin.id];
  const [x, y] = transformPoint(t, p.x, p.y);
  const taken = new Set(project.namedPoints.map((q) => q.name));
  let name = `${p.name} (${SUFFIX[t].point})`;
  for (let i = 2; taken.has(name); i++) name = `${p.name} (${SUFFIX[t].point} ${i})`;
  const twin: NamedPoint = { id: newId("pt"), name, x, y, heading: transformAngle(t, p.heading ?? Number.NaN), twin: { id: p.id, transform: t } };
  const namedPoints = [...project.namedPoints.map((q) => (q.id === p.id ? { ...q, twin: { id: twin.id, transform: t } } : q)), twin];
  return [{ ...project, namedPoints }, twin.id];
}

/** The path turned by t, its waypoints linked to the twins of its named points (made as needed). */
function turned(project: Project, path: EditorPath, t: Transform): [Project, Pick<EditorPath, "spec" | "links">] {
  let out = project;
  const links = path.links.map((l) => {
    if (l === null) return null;
    const [next, id] = twinOf(out, l, t);
    out = next;
    return id;
  });
  return [out, { spec: transformSpec(path.spec, t), links }];
}

export type TransformMode = "inPlace" | "duplicate" | "linked";

/**
 * Flips or mirrors a path: in place, as a new path after it, or as a linked copy that follows it.
 * Returns the project and the id of the path that now holds the result.
 */
export function transformPath(project: Project, pathId: string, t: Transform, mode: TransformMode): [Project, string] {
  const index = project.paths.findIndex((p) => p.id === pathId);
  const path = project.paths[index];
  if (!path) return [project, pathId];
  const [withTwins, parts] = turned(project, path, t);
  if (mode === "inPlace") {
    return [{ ...withTwins, paths: withTwins.paths.map((p) => (p.id === pathId ? { ...p, ...parts } : p)) }, pathId];
  }
  const copy: EditorPath = {
    ...makeEditorPath(uniquePathName(withTwins, `${path.name}${SUFFIX[t].path}`), parts.spec, withTwins.paths.length, parts.links, path.pinned),
    mirrorOf: mode === "linked" ? { id: path.id, transform: t } : null,
  };
  const paths = withTwins.paths.slice();
  paths.splice(index + 1, 0, copy);
  return [{ ...withTwins, paths }, copy.id];
}

/**
 * Keeps linked copies in step with their sources after an edit (from `before` to `after`): a copy whose
 * source changed is made again from it, and an edit made to a copy itself is undone the same way. A copy
 * whose source is gone becomes an ordinary path.
 */
export function syncLinked(before: Project, after: Project): Project {
  if (!after.paths.some((p) => p.mirrorOf)) return after;
  let out = after;
  for (const copy of after.paths) {
    const link = copy.mirrorOf;
    if (!link) continue;
    const source = out.paths.find((p) => p.id === link.id);
    if (!source) {
      out = { ...out, paths: out.paths.map((p) => (p.id === copy.id ? { ...p, mirrorOf: null } : p)) };
      continue;
    }
    const was = before.paths.find((p) => p.id === copy.id);
    const sourceWas = before.paths.find((p) => p.id === link.id);
    if (was === copy && sourceWas === source) continue;
    const [next, parts] = turned(out, source, link.transform);
    out = { ...next, paths: next.paths.map((p) => (p.id === copy.id ? { ...p, ...parts, pinned: source.pinned } : p)) };
  }
  return out;
}

/** Breaks a path's link to its source (it becomes an ordinary path). */
export function unlinkPath(project: Project, pathId: string): Project {
  return { ...project, paths: project.paths.map((p) => (p.id === pathId ? { ...p, mirrorOf: null } : p)) };
}

/** Breaks a named point's link to its twin, on both ends. */
export function unlinkPoint(project: Project, id: string): Project {
  const p = project.namedPoints.find((q) => q.id === id);
  const other = p?.twin?.id;
  return { ...project, namedPoints: project.namedPoints.map((q) => (q.id === id || q.id === other ? { ...q, twin: null } : q)) };
}

/** The paths that follow this one as linked copies. */
export function linkedCopies(project: Project, pathId: string): EditorPath[] {
  return project.paths.filter((p) => p.mirrorOf?.id === pathId);
}
