import { describe, expect, it } from "vitest";
import {
  DEFAULT_ROBOT,
  eventAtMeters,
  eventAtWaypoint,
  makeConstraintZone,
  makePathSpec,
  makeWaypoint,
  writePath,
  type PathSpec,
} from "../../core/src/index.ts";
import { analyse, type AnalysisInput } from "../src/compute/analysis.ts";
import { fixedWaypoints, optimise } from "../src/compute/optimiser.ts";

function input(spec: PathSpec, obstacles: AnalysisInput["obstacles"] = []): AnalysisInput {
  return {
    spec,
    robot: DEFAULT_ROBOT,
    bumperLengthMeters: 0.85,
    bumperWidthMeters: 0.85,
    fieldLengthMeters: 16.54,
    fieldWidthMeters: 8.07,
    obstacles,
  };
}

// a weave whose interior waypoints sit off the straight line: room to save time inside 0.5 m
const weave = makePathSpec({
  waypoints: [makeWaypoint(-6, 0, 0), makeWaypoint(-4, 0.9), makeWaypoint(-2, -0.9), makeWaypoint(0, 0.9), makeWaypoint(2, 0, 0)],
  maxMetersPerSecond: 4,
  maxMetersPerSecondSquared: 6,
  events: [eventAtWaypoint("intake", 1.5), eventAtMeters("shoot", 6)],
});

const quick = { maxEvaluations: 300, timeBudgetMs: Infinity, seed: 7 };

describe("optimiser", () => {
  it("shortens a weaving path, moving only free waypoints inside the corridor", () => {
    const r = optimise(input(weave), quick);
    expect(r.afterSeconds).toBeLessThan(r.beforeSeconds * 0.97);
    expect(r.free).toEqual([1, 2, 3]);
    expect(r.spec.waypoints[0]).toEqual(weave.waypoints[0]);
    expect(r.spec.waypoints[4]).toEqual(weave.waypoints[4]);
    r.spec.waypoints.forEach((w, i) => {
      const o = weave.waypoints[i]!;
      expect(Math.hypot(w.xMeters - o.xMeters, w.yMeters - o.yMeters)).toBeLessThanOrEqual(0.5 + 1e-5);
    });
    // its reported time is the core's plan of what it returns
    const a = analyse(input(r.spec));
    expect(a.ok && a.totalTimeSeconds).toBe(r.afterSeconds);
  });

  it("is deterministic for a seed and budget", () => {
    const a = optimise(input(weave), quick);
    const b = optimise(input(weave), quick);
    expect(writePath(a.spec)).toBe(writePath(b.spec));
    expect(a.evaluations).toBe(b.evaluations);
    const c = optimise(input(weave), { ...quick, seed: 8 });
    expect(c.afterSeconds).toBeLessThan(c.beforeSeconds);
  });

  it("never makes a path slower, and leaves an already straight one alone", () => {
    const straight = makePathSpec({ waypoints: [makeWaypoint(-3, 0, 0), makeWaypoint(0, 0), makeWaypoint(3, 0, 0)], maxMetersPerSecond: 4 });
    for (const spec of [straight, weave]) {
      const r = optimise(input(spec), { ...quick, maxEvaluations: 120 });
      expect(r.afterSeconds).toBeLessThanOrEqual(r.beforeSeconds);
    }
    const r = optimise(input(straight), { ...quick, maxEvaluations: 120 });
    if (r.afterSeconds === r.beforeSeconds) expect(r.spec).toBe(straight);
  });

  it("keeps the swept bumpers clear of obstacles", () => {
    // a post above the dip at x = -2: the weave clears it, a straightened path would run into it
    const post = { name: "post", polygon: [[-2.3, -0.3], [-1.7, -0.3], [-1.7, 0], [-2.3, 0]] as [number, number][] };
    const base = analyse(input(weave, [post]));
    expect(base.ok && base.minClearanceMeters).toBeGreaterThan(0.05);
    const free = optimise(input(weave), quick);
    const r = optimise(input(weave, [post]), quick);
    const after = analyse(input(r.spec, [post]));
    expect(after.ok && after.minClearanceMeters).toBeGreaterThanOrEqual(0.05 - 1e-9);
    expect(r.afterSeconds).toBeLessThan(r.beforeSeconds);
    // the post costs time: the unconstrained optimum would hit it
    const unconstrained = analyse(input(free.spec, [post]));
    expect(unconstrained.ok && unconstrained.minClearanceMeters).toBeLessThan(0.05);
    expect(r.afterSeconds).toBeGreaterThan(free.afterSeconds);
  });

  it("leaves locked waypoints and zone anchors alone, and keeps events in place", () => {
    const zoned = { ...weave, constraintZones: [makeConstraintZone(2.5, 3, 3, Infinity)] };
    expect(fixedWaypoints(zoned, [false, true])).toEqual([true, true, true, true, true]);
    const r = optimise(input(weave), { ...quick, locked: [false, false, true] });
    expect(r.free).toEqual([1, 3]);
    expect(r.spec.waypoints[2]).toEqual(weave.waypoints[2]);
    // the waypoint-relative event keeps its position; the metres one keeps its share of the length
    expect(r.spec.events[0]).toEqual(weave.events[0]);
    const before = analyse(input(weave));
    const after = analyse(input(r.spec));
    if (before.ok && after.ok) {
      expect(r.spec.events[1]!.meters / after.lengthMeters).toBeCloseTo(6 / before.lengthMeters, 4);
    }
  });
});
