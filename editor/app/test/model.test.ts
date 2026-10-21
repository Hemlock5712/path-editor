import { describe, expect, it } from "vitest";
import {
  eventAtMeters,
  eventAtWaypoint,
  makeConstraintZone,
  makeFacePointZone,
  makePathSpec,
  makeRotationTarget,
  makeWaypoint,
  PathGeometry,
} from "../../core/src/index.ts";
import { appendWaypoint, deleteWaypoint, insertWaypoint, moveWaypoint, setHeading, setPinned } from "../src/model/edits.ts";
import { addNamedPoint, linkToPoint, moveNamedPoint, removeNamedPoint } from "../src/model/namedPoints.ts";
import { afterDelete, afterInsert, round5, round5Radians, waypointAtS } from "../src/model/positions.ts";
import { emptyProject, eventNames, makeEditorPath, starterSpec, type Project } from "../src/model/project.ts";

const spec = makePathSpec({
  waypoints: [makeWaypoint(-4, -1), makeWaypoint(-2, 0.5), makeWaypoint(0, -0.5), makeWaypoint(2, 1)],
  rotationTargets: [makeRotationTarget(1.5, Number.NaN, 1), makeRotationTarget(Number.NaN, 2.5, 0)],
  facePointZones: [makeFacePointZone(1, 2, { x: 0, y: 3 }, 0.5)],
  constraintZones: [makeConstraintZone(0.5, 1.5, 1.5, Infinity)],
  events: [eventAtWaypoint("intake", 1.25), eventAtMeters("shoot", 3)],
});

describe("waypoint-relative positions", () => {
  it("keep their stretch when a waypoint is inserted", () => {
    // insert between 1 and 2 at 40%
    expect(afterInsert(0.5, 1, 0.4)).toBe(0.5);
    expect(afterInsert(1.2, 1, 0.4)).toBeCloseTo(1.5, 12); // 0.2 of 0.4 = halfway into the first half
    expect(afterInsert(1.7, 1, 0.4)).toBeCloseTo(2.5, 12); // 0.3 of the remaining 0.6
    expect(afterInsert(2, 1, 0.4)).toBe(3);
    expect(afterInsert(2.5, 1, 0.4)).toBe(3.5);
  });

  it("merge the two stretches around a deleted waypoint", () => {
    expect(afterDelete(0.5, 2, 4)).toBe(0.5);
    expect(afterDelete(1.5, 2, 4)).toBeCloseTo(1.25, 12);
    expect(afterDelete(2.5, 2, 4)).toBeCloseTo(1.75, 12);
    expect(afterDelete(3, 2, 4)).toBe(2);
    expect(afterDelete(0.5, 0, 4)).toBe(0); // the first stretch is gone
    expect(afterDelete(2.5, 3, 4)).toBe(2); // the last stretch is gone
  });

  it("converts metres to waypoint positions and back through the core", () => {
    const g = PathGeometry.build(spec.waypoints);
    for (const w of [0, 0.3, 1, 1.75, 2.5, 3]) expect(waypointAtS(g, g.sAtWaypoint(w))).toBeCloseTo(w, 9);
    expect(waypointAtS(g, -1)).toBe(0);
    expect(waypointAtS(g, g.lengthMeters + 1)).toBe(3);
  });

  it("round to what the file holds", () => {
    expect(round5(1 / 3)).toBe(0.33333);
    const r = round5Radians(1);
    expect(round5(r / (Math.PI / 180))).toBe(57.29578);
  });
});

describe("path edits", () => {
  const path = makeEditorPath("Test", spec, 0);

  it("insert a waypoint and renumber what was after it", () => {
    const p = insertWaypoint(path, 1, 0.5, -1, 0);
    expect(p.spec.waypoints).toHaveLength(5);
    expect(p.spec.waypoints[2]).toEqual(makeWaypoint(-1, 0));
    expect(p.links).toEqual([null, null, null, null, null]);
    expect(p.spec.rotationTargets[0]!.waypoint).toBeCloseTo(2, 12); // 1.5 was the middle of 1..2
    expect(p.spec.rotationTargets[1]!.meters).toBe(2.5); // metres are left alone
    expect(p.spec.facePointZones[0]!.fromWaypoint).toBe(1);
    expect(p.spec.facePointZones[0]!.toWaypoint).toBe(3);
    expect(p.spec.events[0]!.waypoint).toBeCloseTo(1.5, 12);
    expect(p.spec.events[1]).toEqual(eventAtMeters("shoot", 3));
  });

  it("delete a waypoint, dropping zones that collapse", () => {
    const p = deleteWaypoint(path, 1);
    expect(p.spec.waypoints).toHaveLength(3);
    expect(p.spec.constraintZones).toHaveLength(1);
    const q = deleteWaypoint(p, 0);
    // the constraint zone 0.25..0.75 sat on the first stretch, which is gone
    expect(q.spec.constraintZones).toHaveLength(0);
    expect(() => deleteWaypoint(q, 0)).toThrow(/two waypoints/);
  });

  it("move and append keep the tangent and grow the links", () => {
    const withTangent = { ...path, spec: { ...path.spec, waypoints: [makeWaypoint(-4, -1, 0.5, 1.2), ...path.spec.waypoints.slice(1)] } };
    const moved = moveWaypoint(withTangent, 0, -5, -2);
    expect(moved.spec.waypoints[0]).toEqual(makeWaypoint(-5, -2, 0.5, 1.2));
    const longer = appendWaypoint(path, 4, 0);
    expect(longer.spec.waypoints).toHaveLength(5);
    expect(longer.links).toHaveLength(5);
  });
});

describe("headings and pins", () => {
  // a swerve path: set headings at both ends, one pass-through point between
  const posed = makeEditorPath(
    "Posed",
    makePathSpec({ waypoints: [makeWaypoint(-4, 0, Number.NaN, 1, 0), makeWaypoint(-2, 1), makeWaypoint(0, 0, Number.NaN, 1, 1.5)] }),
    0,
  );

  it("a new path is two robot poses facing the same way", () => {
    const s = starterSpec(1, 2, Math.PI / 2);
    expect(s.waypoints.map((w) => w.headingRadians)).toEqual([Math.PI / 2, Math.PI / 2]);
    expect(s.waypoints[1]!.xMeters).toBeCloseTo(1, 12);
    expect(s.waypoints[1]!.yMeters).toBeCloseTo(4.5, 12);
    expect(s.headingMode).toBe("hold");
  });

  it("appending moves the end's heading to the new end; the old end passes through", () => {
    const p = appendWaypoint(posed, 2, 1);
    expect(p.spec.waypoints.map((w) => w.headingRadians)).toEqual([0, Number.NaN, Number.NaN, 1.5]);
    expect(p.pinned).toEqual([false, false, false, false]);
    // appending to a two-point path keeps the start's heading
    const two = makeEditorPath("Two", starterSpec(0, 0, 0.5), 0);
    expect(appendWaypoint(two, 5, 5).spec.waypoints.map((w) => w.headingRadians)).toEqual([0.5, Number.NaN, 0.5]);
  });

  it("inserted points pass through, and pins stay with their waypoints", () => {
    const p = insertWaypoint(setPinned(posed, 1, true), 0, 0.5, -3, 0.5);
    expect(Number.isNaN(p.spec.waypoints[1]!.headingRadians)).toBe(true);
    expect(p.pinned).toEqual([false, false, true, false]);
    expect(deleteWaypoint(p, 1).pinned).toEqual([false, true, false]);
  });

  it("the ends always keep a heading", () => {
    expect(() => setHeading(posed, 0, Number.NaN)).toThrow(/always have a heading/);
    expect(() => setHeading(posed, 2, Number.NaN)).toThrow(/always have a heading/);
    expect(setHeading(posed, 1, 0.75).spec.waypoints[1]!.headingRadians).toBe(0.75);
    // deleting an end hands its heading to the new end when that one passes through
    expect(deleteWaypoint(posed, 2).spec.waypoints[1]!.headingRadians).toBe(1.5);
    expect(deleteWaypoint(posed, 0).spec.waypoints[0]!.headingRadians).toBe(0);
  });

  it("moving a waypoint keeps its heading", () => {
    expect(moveWaypoint(posed, 2, 1, 1).spec.waypoints[2]).toEqual(makeWaypoint(1, 1, Number.NaN, 1, 1.5));
  });
});

describe("named points", () => {
  it("move every linked waypoint in every path", () => {
    let project: Project = {
      ...emptyProject(),
      paths: [makeEditorPath("A", spec, 0), makeEditorPath("B", spec, 1)],
    };
    const [withPoint, point] = addNamedPoint(project, "Reef", 1, 1);
    project = linkToPoint(withPoint, project.paths[0]!.id, 3, point.id);
    project = linkToPoint(project, project.paths[1]!.id, 0, point.id);
    expect(project.paths[0]!.spec.waypoints[3]).toMatchObject({ xMeters: 1, yMeters: 1 });
    const moved = moveNamedPoint(project, point.id, 2.5, -0.5);
    expect(moved.paths[0]!.spec.waypoints[3]).toMatchObject({ xMeters: 2.5, yMeters: -0.5 });
    expect(moved.paths[1]!.spec.waypoints[0]).toMatchObject({ xMeters: 2.5, yMeters: -0.5 });
    expect(moved.paths[1]!.spec.waypoints[1]).toEqual(spec.waypoints[1]); // unlinked ones stay
    const removed = removeNamedPoint(moved, point.id);
    expect(removed.paths[0]!.links[3]).toBeNull();
    expect(removed.paths[0]!.spec.waypoints[3]).toMatchObject({ xMeters: 2.5, yMeters: -0.5 });
  });

  it("collect event names across the project for the picker", () => {
    const project = { ...emptyProject(), paths: [makeEditorPath("A", spec, 0), makeEditorPath("B", { ...spec, events: [eventAtMeters("align", 1)] }, 1)] };
    expect(eventNames(project)).toEqual(["align", "intake", "shoot"]);
  });
});
