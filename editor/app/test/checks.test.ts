import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import {
  DEFAULT_ROBOT,
  eventAtMeters,
  eventAtWaypoint,
  makeFacePointZone,
  makePathSpec,
  makeRotationTarget,
  makeWaypoint,
  Path,
  readPath,
  type PathSpec,
} from "../../core/src/index.ts";
import { analyse, type Analysis, type AnalysisInput, type Check } from "../src/compute/analysis.ts";
import { bumperCorners, convexDistance, convexHull, convexOverlap } from "../src/compute/polygons.ts";

const ROOT = join(import.meta.dirname, "..", "..", "..");
const AUTO_ROUTE = readPath(readFileSync(join(ROOT, "lib", "src", "test", "resources", "example.path.json"), "utf8"));

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

function ok(spec: PathSpec, obstacles: AnalysisInput["obstacles"] = []): Analysis {
  const a = analyse(input(spec, obstacles));
  if (!a.ok) throw new Error(a.error);
  return a;
}

const kinds = (checks: readonly Check[], kind: Check["kind"]) => checks.filter((c) => c.kind === kind);

const straight = makePathSpec({ waypoints: [makeWaypoint(-3, 0, 0), makeWaypoint(3, 0, 0)], maxMetersPerSecond: 4, maxMetersPerSecondSquared: 6 });

describe("analysis", () => {
  it("reports the core's own numbers", () => {
    const a = ok(AUTO_ROUTE);
    const path = Path.build(AUTO_ROUTE, DEFAULT_ROBOT);
    expect(a.lengthMeters).toBe(path.lengthMeters);
    expect(a.totalTimeSeconds).toBe(path.totalTimeSeconds);
    const i = 300;
    const p = path.sample(a.stations.s[i]!);
    expect(a.stations.v[i]).toBe(p.metersPerSecond);
    expect(a.stations.heading[i]).toBe(p.headingRadians);
    expect(a.stations.kappa[i]).toBe(p.curvature);
    const q = path.sampleAtTime(1.0);
    expect(a.byTime.x[Math.round(1.0 / a.byTime.dt)]).toBe(q.xMeters);
  });

  it("places events at their path position and plan time", () => {
    const a = ok(AUTO_ROUTE);
    const path = Path.build(AUTO_ROUTE, DEFAULT_ROBOT);
    expect(a.events.map((e) => e.name)).toEqual(["intakeDown", "spinUp", "intakeUp", 'score "L4"']);
    expect(a.events[0]!.s).toBe(path.geometry.sAtWaypoint(0.5));
    expect(a.events[1]!.t).toBe(path.profile.timeAt(6.25));
  });

  it("fails cleanly on a spec the core can't build", () => {
    const a = analyse(input({ ...straight, waypoints: [makeWaypoint(0, 0), makeWaypoint(0, 0), makeWaypoint(0, 0)] }));
    expect(a.ok).toBe(false);
  });
});

describe("checks", () => {
  it("a plain straight path has none", () => {
    expect(ok(straight).checks).toEqual([]);
  });

  it("finds the fast face-point blend in the auto route (the plan drops to about 0.7 m/s)", () => {
    // on carpet grip (μ 0.7, no tipping limit): Spike Grip corners hard enough to take most of the crawl out
    const carpet = (spec: PathSpec): Analysis => {
      const a = analyse({ ...input(spec), robot: { ...DEFAULT_ROBOT, frictionCoefficient: 0.7, centerOfMassHeightMeters: 0 } });
      if (!a.ok) throw new Error(a.error);
      return a;
    };
    const before = carpet(AUTO_ROUTE);
    const turns = kinds(before.checks, "heading");
    const degrees = (c: Check) => Number(/turns (\d+)°/.exec(c.message)![1]);
    const slowest = (c: Check) => Number(/slows to (\d+\.\d+) m\/s/.exec(c.message)![1]);
    expect(turns.some((c) => degrees(c) > 100 && slowest(c) < 0.8), turns.map((c) => c.message).join(" | ")).toBe(true);
    // a longer blend spreads the turn: the plan no longer crawls, and the path is faster
    const zone = AUTO_ROUTE.facePointZones[0]!;
    const gentle = carpet({ ...AUTO_ROUTE, facePointZones: [makeFacePointZone(zone.fromWaypoint, zone.toWaypoint, zone.point, 2.0)] });
    expect(kinds(gentle.checks, "heading").every((c) => slowest(c) > 1.0)).toBe(true);
    expect(gentle.totalTimeSeconds).toBeLessThan(before.totalTimeSeconds - 0.5);
  });

  it("notes a hairpin rather than warning about it", () => {
    // down to a point and straight back up: the robot slows there whatever the curve
    const hairpin = makePathSpec({
      waypoints: [makeWaypoint(-4, 3, 0), makeWaypoint(-0.6, 0.4, Math.PI, 0.26), makeWaypoint(-1.5, 3.4, Math.PI / 2)],
      maxMetersPerSecond: 4,
    });
    const bends = kinds(ok(hairpin).checks, "curvature");
    expect(bends).toHaveLength(1);
    expect(bends[0]!.severity).toBe("info");
    expect(bends[0]!.message).toMatch(/^Hairpin at/);
  });

  it("flags a bend too tight for the path's speed", () => {
    // a short tangent handle at the corner makes it nearly sharp
    const corner = makePathSpec({
      waypoints: [makeWaypoint(-3, 0, 0), makeWaypoint(0, 0, Math.PI / 4, 0.2), makeWaypoint(0, 3, Math.PI / 2)],
      maxMetersPerSecond: 4,
    });
    const bends = kinds(ok(corner).checks, "curvature");
    expect(bends).toHaveLength(1);
    expect(bends[0]!.message).toMatch(/radius 0\.0\d m caps the speed at 0\.\d\d m\/s/);
    expect(bends[0]!.severity).toBe("warning"); // a 90° corner is worth rounding
    // the default handle rounds the same corner enough
    const rounder = { ...corner, waypoints: [corner.waypoints[0]!, makeWaypoint(0, 0, Math.PI / 4, 1), corner.waypoints[2]!] };
    expect(kinds(ok(rounder).checks, "curvature")).toEqual([]);
  });

  it("finds the bumpers touching an obstacle, and a near miss", () => {
    const box = (y0: number) => ({ name: "box", polygon: [[-0.3, y0], [0.3, y0], [0.3, y0 + 0.6], [-0.3, y0 + 0.6]] as [number, number][] });
    const hit = kinds(ok(straight, [box(0.2)]).checks, "bumper");
    expect(hit).toHaveLength(1);
    expect(hit[0]!.severity).toBe("error");
    expect(hit[0]!.message).toMatch(/hit box/);
    // bumper half-width 0.425: an obstacle 0.45 m from the centre line clears by 2.5 cm
    const near = kinds(ok(straight, [box(0.45)]).checks, "bumper");
    expect(near).toHaveLength(1);
    expect(near[0]!.severity).toBe("warning");
    expect(near[0]!.message).toMatch(/2\.50 cm from box/);
    expect(kinds(ok(straight, [box(0.6)]).checks, "bumper")).toHaveLength(0);
  });

  it("finds the bumpers leaving the field", () => {
    const edge = makePathSpec({ waypoints: [makeWaypoint(7, 0, 0), makeWaypoint(8, 0, 0)] });
    const walls = kinds(ok(edge).checks, "bumper");
    expect(walls).toHaveLength(1);
    expect(walls[0]!.message).toMatch(/field wall/);
  });

  it("finds events off the path", () => {
    const spec = { ...straight, events: [eventAtMeters("inside", 2), eventAtMeters("after", 6.5), eventAtWaypoint("before", -0.5)] };
    const off = kinds(ok(spec).checks, "event");
    expect(off.map((c) => c.message.split('"')[1])).toEqual(["after", "before"]);
    expect(off.every((c) => c.severity === "error")).toBe(true);
  });

  it("finds a start speed the plan can't hold", () => {
    const turnAtStart = makePathSpec({
      waypoints: [makeWaypoint(0, 0, 0), makeWaypoint(1, 0, 0)],
      rotationTargets: [makeRotationTarget(0, Number.NaN, 0), makeRotationTarget(0.3, Number.NaN, Math.PI)],
      startMetersPerSecond: 3,
    });
    const speed = kinds(ok(turnAtStart).checks, "speed");
    expect(speed).toHaveLength(1);
    expect(speed[0]!.message).toMatch(/can't start at 3\.00 m\/s/);
  });
});

describe("polygons", () => {
  it("hulls, overlaps and distances", () => {
    expect(convexHull([[0, 0], [1, 0], [0.5, 0.2], [1, 1], [0, 1]])).toHaveLength(4);
    const robot = bumperCorners(0, 0, Math.PI / 4, 1, 1);
    const square = [[1, -0.5], [2, -0.5], [2, 0.5], [1, 0.5]] as [number, number][];
    expect(convexOverlap(robot, square)).toBe(false); // the corner reaches 0.707
    expect(convexDistance(robot, square)).toBeCloseTo(1 - Math.SQRT1_2, 9);
    const square2 = square.map(([x, y]) => [x - 0.4, y] as [number, number]);
    expect(convexDistance(robot, square2)).toBe(0);
  });
});

describe("signed clearance", () => {
  it("is the gap when apart and minus the penetration when overlapping", async () => {
    const { signedClearance } = await import("../src/compute/polygons.ts");
    const square = (x: number, y: number, h: number) => [[x - h, y - h], [x + h, y - h], [x + h, y + h], [x - h, y + h]] as [number, number][];
    expect(signedClearance(square(0, 0, 0.5), square(2, 0, 0.5))).toBeCloseTo(1, 12);
    expect(signedClearance(square(0, 0, 0.5), square(0.8, 0, 0.5))).toBeCloseTo(-0.2, 12);
    expect(signedClearance(square(0, 0, 0.5), square(0, 0, 0.5))).toBeCloseTo(-1, 12);
  });
});
