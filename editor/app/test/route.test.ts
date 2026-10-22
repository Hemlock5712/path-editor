import { describe, expect, it } from "vitest";
import { DEFAULT_ROBOT, makePathSpec, makeWaypoint, type PathSpec } from "../../core/src/index.ts";
import { analyse, CHECKS } from "../src/compute/analysis.ts";
import { optimise } from "../src/compute/optimiser.ts";
import { routeAround } from "../src/compute/route.ts";
import { DEFAULT_FIELD, makeEditorPath } from "../src/model/project.ts";

const input = (spec: PathSpec) => ({
  spec,
  robot: DEFAULT_ROBOT,
  bumperLengthMeters: 0.85,
  bumperWidthMeters: 0.85,
  fieldLengthMeters: 16.54,
  fieldWidthMeters: 8.07,
  obstacles: DEFAULT_FIELD.obstacles,
});
const setup = { field: DEFAULT_FIELD, robot: DEFAULT_ROBOT, bumperLengthMeters: 0.85, bumperWidthMeters: 0.85 };
const planned = (spec: PathSpec) => {
  const a = analyse(input(spec));
  if (!a.ok) throw new Error(a.error);
  return a;
};

// from the blue start area across the blue hub: waypoint 2 sits in it
const throughHub = makePathSpec({
  waypoints: [makeWaypoint(-6.5, -1.5, Number.NaN, 1, 0), makeWaypoint(-4, -1.5), makeWaypoint(-3.65, 0.3), makeWaypoint(-0.5, 0, Number.NaN, 1, 0)],
  maxMetersPerSecond: 4,
  maxMetersPerSecondSquared: 6,
});

describe("route around obstacles", () => {
  it("replaces the pass-through point in the hub with the pathfinder's corners, and the result is clear", () => {
    const r = routeAround(makeEditorPath("P", throughHub, 0), planned(throughHub), setup);
    if (!r.ok) throw new Error(r.reason);
    expect([r.from, r.to, r.removed]).toEqual([1, 3, 1]);
    expect(r.added).toBeGreaterThan(0);
    const w = r.path.spec.waypoints;
    expect(w).toHaveLength(4 - 1 + r.added);
    // the ends and the waypoint before the stretch stay where they were; the new points pass through
    expect(w[0]).toEqual(throughHub.waypoints[0]);
    expect(w[1]).toEqual(throughHub.waypoints[1]);
    expect(w.at(-1)).toEqual(throughHub.waypoints[3]);
    expect(w.slice(2, -1).every((p) => Number.isNaN(p.headingRadians))).toBe(true);
    expect([r.path.links.length, r.path.pinned.length]).toEqual([w.length, w.length]);
    const a = planned(r.path.spec);
    expect(a.minClearanceMeters).toBeGreaterThanOrEqual(CHECKS.bumperMarginMeters);
    // and the curves-only optimiser keeps it clear while it shapes it
    const o = optimise(input(r.path.spec), { maxEvaluations: 300, seed: 1 });
    expect(o.clear).toBe(true);
    expect(o.spec.waypoints.map((p) => [p.xMeters, p.yMeters])).toEqual(w.map((p) => [p.xMeters, p.yMeters]));
  });

  it("routes through a waypoint in the stretch that has a heading, leg by leg, and keeps it", () => {
    const headed = makePathSpec({
      ...throughHub,
      waypoints: throughHub.waypoints.map((p, i) => (i === 2 ? makeWaypoint(-3.65, 1.3, Number.NaN, 1, 0.5) : p)),
    });
    const a = planned(headed);
    expect(a.checks.some((c) => c.kind === "bumper")).toBe(true); // it still clips the hub on the way up
    const r = routeAround(makeEditorPath("P", headed, 0), a, setup);
    if (!r.ok) throw new Error(r.reason);
    expect(r.removed).toBe(0);
    expect(r.path.spec.waypoints.some((p) => p.xMeters === -3.65 && p.yMeters === 1.3 && p.headingRadians === 0.5)).toBe(true);
  });

  it("says when there is nothing to route around", () => {
    const clear = makePathSpec({ waypoints: [makeWaypoint(-6.5, -1.5, Number.NaN, 1, 0), makeWaypoint(-4, -1.5, Number.NaN, 1, 0)], maxMetersPerSecond: 4 });
    const r = routeAround(makeEditorPath("P", clear, 0), planned(clear), setup);
    expect(r).toEqual({ ok: false, reason: "Nothing to route around: the path clears every obstacle." });
  });
});
