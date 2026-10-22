import { describe, expect, it } from "vitest";
import {
  eventAtMeters,
  eventAtWaypoint,
  makeConstraintZone,
  makeFacePointZone,
  makePathSpec,
  makeRotationTarget,
  makeWaypoint,
} from "../../core/src/index.ts";
import { syncLinked, transformPath, transformSpec, twinOf, unlinkPath, unlinkPoint } from "../src/model/mirror.ts";
import { addNamedPoint, linkToPoint, moveNamedPoint, removeNamedPoint } from "../src/model/namedPoints.ts";
import { emptyProject, makeEditorPath, type Project } from "../src/model/project.ts";
import { moveWaypoint } from "../src/model/edits.ts";

const DEG = Math.PI / 180;

const spec = makePathSpec({
  waypoints: [makeWaypoint(-6, -2, 30 * DEG, 1.2, 0), makeWaypoint(-3, -1), makeWaypoint(-1, -2.5, Number.NaN, 1, 90 * DEG)],
  rotationTargets: [makeRotationTarget(1.5, Number.NaN, 45 * DEG)],
  facePointZones: [makeFacePointZone(0.5, 1.5, { x: -3.6, y: 0 }, 1)],
  constraintZones: [makeConstraintZone(0.2, 0.8, 1.5, Infinity)],
  events: [eventAtWaypoint("intake", 1), eventAtMeters("shoot", 2)],
  maxMetersPerSecond: 4,
});

/** A project with one path on a named point "Score L" at its end. */
function project(): Project {
  const base = { ...emptyProject(), paths: [makeEditorPath("Left", spec, 0)] };
  const [withPoint, point] = addNamedPoint(base, "Score L", -1, -2.5, 90 * DEG);
  return linkToPoint(withPoint, base.paths[0]!.id, 2, point.id);
}

describe("flip and mirror", () => {
  it("turn every position and direction on the path, and leave what's placed along it alone", () => {
    const m = transformSpec(spec, "mirrorTopBottom");
    expect(m.waypoints.map((w) => [w.xMeters, w.yMeters])).toEqual([
      [-6, 2],
      [-3, 1],
      [-1, 2.5],
    ]);
    expect(m.waypoints[0]!.tangentRadians).toBeCloseTo(-30 * DEG, 5);
    expect(m.waypoints[0]!.tangentScale).toBe(1.2);
    expect(m.waypoints[2]!.headingRadians).toBeCloseTo(-90 * DEG, 5);
    expect(Number.isNaN(m.waypoints[1]!.headingRadians)).toBe(true); // pass-through stays pass-through
    expect(m.rotationTargets[0]!.headingRadians).toBeCloseTo(-45 * DEG, 5);
    expect(m.facePointZones[0]!.point).toEqual({ x: -3.6, y: 0 });
    expect(Object.is(m.facePointZones[0]!.point.y, -0)).toBe(false);
    expect(m.constraintZones).toEqual(spec.constraintZones);
    expect(m.events).toEqual(spec.events);

    const f = transformSpec(spec, "flip");
    expect([f.waypoints[0]!.xMeters, f.waypoints[0]!.yMeters]).toEqual([6, 2]);
    expect(f.waypoints[0]!.headingRadians).toBeCloseTo(Math.PI, 5);
    expect(f.waypoints[2]!.headingRadians).toBeCloseTo(-90 * DEG, 5);
    const lr = transformSpec(spec, "mirrorLeftRight");
    expect([lr.waypoints[0]!.xMeters, lr.waypoints[0]!.yMeters]).toEqual([6, -2]);
    expect(lr.waypoints[0]!.tangentRadians).toBeCloseTo(150 * DEG, 5);
    // each is its own inverse
    for (const t of ["flip", "mirrorTopBottom", "mirrorLeftRight"] as const) {
      const back = transformSpec(transformSpec(spec, t), t);
      back.waypoints.forEach((w, i) => {
        expect(w.xMeters).toBeCloseTo(spec.waypoints[i]!.xMeters, 9);
        expect(w.yMeters).toBeCloseTo(spec.waypoints[i]!.yMeters, 9);
      });
    }
  });

  it("gives named points twins, made once and reused, that stay in step", () => {
    let pr = project();
    const id = pr.namedPoints[0]!.id;
    const [withTwin, twinId] = twinOf(pr, id, "mirrorTopBottom");
    expect(withTwin.namedPoints.find((p) => p.id === twinId)).toMatchObject({ name: "Score L (mirror)", x: -1, y: 2.5 });
    expect(withTwin.namedPoints.find((p) => p.id === twinId)!.heading).toBeCloseTo(-90 * DEG, 5);
    expect(twinOf(withTwin, id, "mirrorTopBottom")[1]).toBe(twinId);
    pr = moveNamedPoint(withTwin, id, -1.5, -3);
    expect(pr.namedPoints.find((p) => p.id === twinId)).toMatchObject({ x: -1.5, y: 3 });
    pr = moveNamedPoint(pr, twinId, -2, 3.2);
    expect(pr.namedPoints.find((p) => p.id === id)).toMatchObject({ x: -2, y: -3.2 });
    // unlinked, they move alone; deleting one unlinks the other
    const apart = moveNamedPoint(unlinkPoint(pr, id), id, 0, 0);
    expect(apart.namedPoints.find((p) => p.id === twinId)).toMatchObject({ x: -2, y: 3.2 });
    expect(removeNamedPoint(pr, id).namedPoints.find((p) => p.id === twinId)!.twin).toBeNull();
  });

  it("mirrors a path onto its named points' twins, in place or as a copy", () => {
    const pr = project();
    const source = pr.paths[0]!;
    const [copied, copyId] = transformPath(pr, source.id, "mirrorTopBottom", "duplicate");
    const copy = copied.paths.find((p) => p.id === copyId)!;
    expect(copy.name).toBe("LeftMirror");
    const twin = copied.namedPoints.find((p) => p.name === "Score L (mirror)")!;
    expect(copy.links).toEqual([null, null, twin.id]);
    expect(copy.mirrorOf).toBeNull();
    // moving the twin moves the copy's end, and the source's end the other way
    const moved = moveNamedPoint(copied, twin.id, -1, 2);
    expect(moved.paths.find((p) => p.id === copyId)!.spec.waypoints[2]).toMatchObject({ xMeters: -1, yMeters: 2 });
    expect(moved.paths.find((p) => p.id === source.id)!.spec.waypoints[2]).toMatchObject({ xMeters: -1, yMeters: -2 });
    const [inPlace] = transformPath(pr, source.id, "flip", "inPlace");
    expect(inPlace.paths).toHaveLength(1);
    expect(inPlace.paths[0]!.spec.waypoints[0]).toMatchObject({ xMeters: 6, yMeters: 2 });
  });

  it("keeps a linked copy in step with its source, and won't let it be edited on its own", () => {
    const pr = project();
    const source = pr.paths[0]!;
    const [linked, copyId] = transformPath(pr, source.id, "mirrorTopBottom", "linked");
    expect(linked.paths.find((p) => p.id === copyId)!.mirrorOf).toEqual({ id: source.id, transform: "mirrorTopBottom" });
    // edit the source: the copy follows
    const edited = { ...linked, paths: linked.paths.map((p) => (p.id === source.id ? moveWaypoint(p, 1, -3, -0.5) : p)) };
    const synced = syncLinked(linked, edited);
    expect(synced.paths.find((p) => p.id === copyId)!.spec.waypoints[1]).toMatchObject({ xMeters: -3, yMeters: 0.5 });
    // edit the copy: it snaps back to its source
    const tampered = { ...synced, paths: synced.paths.map((p) => (p.id === copyId ? moveWaypoint(p, 1, 0, 0) : p)) };
    expect(syncLinked(synced, tampered).paths.find((p) => p.id === copyId)!.spec.waypoints[1]).toMatchObject({ xMeters: -3, yMeters: 0.5 });
    // unlinked, it's an ordinary path
    const free = unlinkPath(synced, copyId);
    const after = { ...free, paths: free.paths.map((p) => (p.id === copyId ? moveWaypoint(p, 1, 0, 0) : p)) };
    expect(syncLinked(free, after).paths.find((p) => p.id === copyId)!.spec.waypoints[1]).toMatchObject({ xMeters: 0, yMeters: 0 });
    // and a copy whose source is deleted becomes one too
    const orphan = { ...synced, paths: synced.paths.filter((p) => p.id !== source.id) };
    expect(syncLinked(synced, orphan).paths[0]!.mirrorOf).toBeNull();
  });
});
