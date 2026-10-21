import { describe, expect, it } from "vitest";
import {
  DEFAULT_ROBOT,
  eventAtMeters,
  eventAtWaypoint,
  makeConstraintZone,
  makePathSpec,
  makeWaypoint,
  readPath,
  writePath,
  type PathSpec,
} from "../../core/src/index.ts";
import { analyse, CHECKS, type AnalysisInput } from "../src/compute/analysis.ts";
import { DEFAULT_FIELD } from "../src/model/project.ts";
import { spawnSync } from "node:child_process";
import { mkdtempSync, readFileSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import {
  blockedMessage,
  evaluationsFor,
  fixedWaypoints,
  optimise,
  whatMoves,
  whyFixed,
  type OptimiseOptions,
} from "../src/compute/optimiser.ts";

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
const moving: Partial<OptimiseOptions> = { ...quick, moveWaypoints: true };
const DEG = Math.PI / 180;

/** Each waypoint's position and heading as the file holds them. */
const poses = (spec: PathSpec) =>
  JSON.parse(writePath(spec)).waypoints.map((w: { x: number; y: number; headingDegrees?: number }) => [w.x, w.y, w.headingDegrees ?? null]);

// straight across the blue hub (2026 field), with one free waypoint over its middle
const throughHub = makePathSpec({
  waypoints: [makeWaypoint(-6.5, 0, 0, 1, 0), makeWaypoint(-3.645, 0.3), makeWaypoint(-0.5, 0, 0, 1, 0)],
  maxMetersPerSecond: 4,
  maxMetersPerSecondSquared: 6,
});

// along the near wall, turning from 0° to 90° on the way: halfway round, the bumpers' corner hits the wall
const wallRun = makePathSpec({
  waypoints: [makeWaypoint(-2.5, -3.45, Number.NaN, 1, 0), makeWaypoint(1.5, -3.45, Number.NaN, 1, 90 * DEG)],
  maxMetersPerSecond: 4,
  maxMetersPerSecondSquared: 6,
});

describe("optimiser", () => {
  it("by default reshapes only the curve: positions and headings stay exactly where they were", () => {
    const r = optimise(input(weave), quick);
    expect(r.afterSeconds).toBeLessThan(r.beforeSeconds * 0.97);
    expect(r.free).toEqual([]);
    expect(r.turned).toEqual([0, 1, 2, 3, 4]);
    expect(poses(r.spec)).toEqual(poses(weave));
    const reshaped = r.spec.waypoints.some(
      (w, i) => w.tangentRadians !== weave.waypoints[i]!.tangentRadians || w.tangentScale !== weave.waypoints[i]!.tangentScale,
    );
    expect(reshaped).toBe(true);
    // its reported time is the core's plan of what it returns
    const a = analyse(input(r.spec));
    expect(a.ok && a.totalTimeSeconds).toBe(r.afterSeconds);
    expect(whatMoves(weave, { moveWaypoints: false, locked: [], segment: null, corridorMeters: 0.5 })).toBe(
      "Reshape the curve at 5 waypoints (tangent directions and scales; the speeds follow). Waypoints and headings stay where they are.",
    );
  });

  it("with moving waypoints on, also moves the free ones inside the corridor", () => {
    const r = optimise(input(weave), moving);
    expect(r.afterSeconds).toBeLessThan(r.beforeSeconds * 0.97);
    expect(r.free).toEqual([1, 2, 3]);
    expect(r.spec.waypoints[0]!.xMeters).toBe(weave.waypoints[0]!.xMeters);
    expect(r.spec.waypoints[4]!.yMeters).toBe(weave.waypoints[4]!.yMeters);
    r.spec.waypoints.forEach((w, i) => {
      const o = weave.waypoints[i]!;
      expect(Math.hypot(w.xMeters - o.xMeters, w.yMeters - o.yMeters)).toBeLessThanOrEqual(0.5 + 1e-5);
    });
    expect(whatMoves(weave, { moveWaypoints: true, locked: [], segment: null, corridorMeters: 0.5 })).toContain(
      "move 3 unlocked waypoints within 0.5 m",
    );
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
    // a post above the dip at x = -2: the weave clears it (holding heading 0), a straightened path would run into it
    const post = { name: "post", polygon: [[-2.1, -0.2], [-1.9, -0.2], [-1.9, 0], [-2.1, 0]] as [number, number][] };
    const base = analyse(input(weave, [post]));
    expect(base.ok && base.minClearanceMeters).toBeGreaterThan(0.05);
    const free = optimise(input(weave), moving);
    const r = optimise(input(weave, [post]), moving);
    const after = analyse(input(r.spec, [post]));
    expect(after.ok && after.minClearanceMeters).toBeGreaterThanOrEqual(0.05 - 1e-9);
    expect(r.afterSeconds).toBeLessThan(r.beforeSeconds);
    // the post costs time: the unconstrained optimum would hit it
    const unconstrained = analyse(input(free.spec, [post]));
    expect(unconstrained.ok && unconstrained.minClearanceMeters).toBeLessThan(0.05);
    expect(r.afterSeconds).toBeGreaterThan(free.afterSeconds);
  });

  it("never moves locked waypoints or zone anchors, in either mode, and keeps events in place", () => {
    const zoned = { ...weave, constraintZones: [makeConstraintZone(2.5, 3, 3, Infinity)] };
    expect(fixedWaypoints(zoned, [false, true])).toEqual([true, true, true, true, true]);
    for (const options of [quick, moving]) {
      const r = optimise(input(weave), { ...options, locked: [false, false, true] });
      expect(r.free).toEqual(options === moving ? [1, 3] : []);
      expect([r.spec.waypoints[2]!.xMeters, r.spec.waypoints[2]!.yMeters]).toEqual([-2, -0.9]);
      // the waypoint-relative event keeps its position; the metres one keeps its share of the length
      expect(r.spec.events[0]).toEqual(weave.events[0]);
      const before = analyse(input(weave));
      const after = analyse(input(r.spec));
      if (before.ok && after.ok) {
        expect(r.spec.events[1]!.meters / after.lengthMeters).toBeCloseTo(6 / before.lengthMeters, 4);
      }
    }
  });

  it("moves a waypoint with a set heading when moving is on, and keeps its heading exactly", () => {
    const posed = {
      ...weave,
      waypoints: weave.waypoints.map((w, i) => (i === 2 ? makeWaypoint(w.xMeters, w.yMeters, Number.NaN, 1, 30 * DEG) : w)),
    };
    expect(whyFixed(posed, [false, false, false, true])).toEqual([
      "the start and end poses stay put",
      null,
      null,
      "locked",
      "the start and end poses stay put",
    ]);
    const r = optimise(input(posed), { ...moving, locked: [false, false, false, true] });
    expect(r.free).toEqual([1, 2]);
    const w = r.spec.waypoints[2]!;
    expect([w.xMeters, w.yMeters]).not.toEqual([-2, -0.9]);
    expect(w.headingRadians).toBe(30 * DEG);
    expect(r.spec.waypoints[3]!.xMeters).toBe(0); // locked
  });

  it("only changes the path inside a segment, the tangents at its ends included", () => {
    expect(fixedWaypoints(weave, [], [0, 2])).toEqual([true, false, true, true, true]);
    const r = optimise(input(weave), { ...moving, segment: [2, 4] });
    expect(r.free).toEqual([3]);
    expect(r.turned).toEqual([2, 3, 4]);
    expect(r.spec.waypoints.slice(0, 2)).toEqual(weave.waypoints.slice(0, 2));
  });

  it("runs headless from Node with the same result for the same budget and seed", () => {
    const dir = mkdtempSync(join(tmpdir(), "optimise-"));
    const repo = join(import.meta.dirname, "..", "..", "..");
    const run = (spec: PathSpec, ...extra: string[]) => {
      const file = join(dir, "in.path.json");
      const out = join(dir, "out.path.json");
      writeFileSync(file, writePath(spec));
      const r = spawnSync(
        process.execPath,
        [join(repo, "tools", "optimise.mjs"), file, "--budget-ms", "400", "--seed", "5", "--out", out, ...extra],
        { encoding: "utf8" },
      );
      return { status: r.status, stderr: r.stderr, out: readFileSync(out, "utf8") };
    };
    const budget = { maxEvaluations: evaluationsFor(0.4), timeBudgetMs: Infinity, seed: 5 };
    // without --field it plans on the editor's default field, the 2026 one with its obstacles: the wall run
    // is cleared by the curve alone, exactly as in the editor
    const a = run(wallRun);
    expect(a.status, a.stderr).toBe(0);
    expect(a.stderr).toContain("clear of obstacles by");
    expect(a.out).toBe(writePath(optimise(input(readPath(writePath(wallRun)), DEFAULT_FIELD.obstacles), budget).spec));
    // --move-waypoints moves them too (on an open field here)
    const open = join(dir, "field.json");
    writeFileSync(open, JSON.stringify({ field: { length_m: 16.54, width_m: 8.07 }, obstacles: [] }));
    const b = run(weave, "--move-waypoints", "--field", open);
    expect(b.status, b.stderr).toBe(0);
    expect(b.out).toBe(writePath(optimise(input(readPath(writePath(weave))), { ...budget, moveWaypoints: true }).spec));
    // a hit the curve can't clear: exit code 3, the path unchanged, and where
    const c = run(throughHub);
    expect(c.status).toBe(3);
    expect(c.stderr).toContain("Can't clear the blue hub by reshaping the curve between waypoints 0 and 1: add a waypoint or move one.");
    expect(c.out).toBe(writePath(throughHub));
  }, 60_000);
});

describe("the optimiser and the 2026 field's obstacles", () => {
  const field = DEFAULT_FIELD.obstacles;
  const bumperChecks = (spec: PathSpec) => {
    const a = analyse(input(spec, field));
    return a.ok ? a.checks.filter((c) => c.kind === "bumper") : null;
  };

  it("clears a wall hit by reshaping the curve alone (the waypoints and headings stay put)", () => {
    expect(bumperChecks(wallRun)!.map((c) => c.message)).toEqual([expect.stringContaining("hit the field wall")]);
    const r = optimise(input(wallRun, field), { ...quick, maxEvaluations: 1500, seed: 1 });
    expect(r.clear).toBe(true);
    expect(r.changed).toBe(true);
    expect(r.clearanceMeters).toBeGreaterThanOrEqual(CHECKS.bumperMarginMeters - 1e-9);
    expect(bumperChecks(r.spec)).toEqual([]);
    expect(poses(r.spec)).toEqual(poses(wallRun));
  }, 30_000);

  it("says what the curve can't clear and where, suggests a waypoint, and changes nothing", () => {
    expect(bumperChecks(throughHub)!.some((c) => c.severity === "error")).toBe(true);
    const r = optimise(input(throughHub, field), { ...quick, maxEvaluations: 600 });
    expect(r.clear).toBe(false);
    expect(r.blockedBy).toBe("blue hub");
    expect(r.spec).toBe(throughHub);
    expect(r.afterSeconds).toBe(r.beforeSeconds);
    expect(blockedMessage(r, { moveWaypoints: false, corridorMeters: 0.5 })).toBe(
      "Can't clear the blue hub by reshaping the curve between waypoints 0 and 1: add a waypoint or move one.",
    );
    // the suggestion is out of the hub, on the side the path was nearer
    const at = r.blockedAt!;
    const fromHub = (x: number, y: number) => Math.max(Math.abs(x + 3.645), Math.abs(y));
    expect(fromHub(at.suggestX, at.suggestY)).toBeGreaterThan(fromHub(at.x, at.y) + 0.5);
  }, 30_000);

  it("with moving waypoints on, pushes a path through the hub out to a clear one", () => {
    const r = optimise(input(throughHub, field), { ...moving, corridorMeters: 1.5, maxEvaluations: 1500 });
    expect(r.clear).toBe(true);
    expect(r.clearanceMeters).toBeGreaterThanOrEqual(CHECKS.bumperMarginMeters - 1e-9);
    expect(bumperChecks(r.spec)).toEqual([]);
    expect(r.spec.waypoints[0]!.xMeters).toBe(-6.5); // the ends' poses stay put
    expect(r.spec.waypoints[2]!.xMeters).toBe(-0.5);
  }, 30_000);

  it("with moving waypoints on and too narrow a corridor, says so", () => {
    const options = { ...quick, moveWaypoints: true, corridorMeters: 0.2, maxEvaluations: 600 };
    const r = optimise(input(throughHub, field), options);
    expect(r.clear).toBe(false);
    expect(r.spec).toBe(throughHub);
    expect(blockedMessage(r, options)).toMatch(
      /^Can't clear the blue hub between waypoints [01] and [12] within 0.2 m: widen the corridor, add a waypoint, or use Route around obstacles\.$/,
    );
  }, 30_000);

  it("never makes a clear path unclear", () => {
    // under the hub, clear by about 27 cm; the faster line would cut the corner into it
    const under = makePathSpec({
      waypoints: [makeWaypoint(-6.5, -1.3, 0, 1, 0), makeWaypoint(-2.2, -1.3), makeWaypoint(-0.5, 0, 0, 1, 0)],
      maxMetersPerSecond: 4,
      maxMetersPerSecondSquared: 6,
    });
    const before = analyse(input(under, field));
    expect(before.ok && before.minClearanceMeters).toBeGreaterThanOrEqual(CHECKS.bumperMarginMeters);
    for (const options of [quick, { ...moving, corridorMeters: 1 }]) {
      const r = optimise(input(under, field), options);
      expect(r.clear).toBe(true);
      const after = analyse(input(r.spec, field));
      expect(after.ok && after.minClearanceMeters).toBeGreaterThanOrEqual(CHECKS.bumperMarginMeters - 1e-9);
      expect(r.afterSeconds).toBeLessThanOrEqual(r.beforeSeconds);
    }
  }, 30_000);
});
