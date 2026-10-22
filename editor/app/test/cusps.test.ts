import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { DEFAULT_ROBOT, makePathSpec, makeWaypoint, PathGeometry, readPath, type PathSpec } from "../../core/src/index.ts";
import { analyse, type AnalysisInput } from "../src/compute/analysis.ts";
import { hasReversal, reversedTangents } from "../src/compute/cusps.ts";
import { optimise } from "../src/compute/optimiser.ts";
import { DEFAULT_FIELD } from "../src/model/project.ts";

// Joe's auto (2026 field): its last tangent, 18.87°, points back against the approach (about 179°)
const joe = readPath(readFileSync(join(import.meta.dirname, "fixtures", "joe_cusp.path.json"), "utf8"));

const input = (spec: PathSpec, obstacles: AnalysisInput["obstacles"] = DEFAULT_FIELD.obstacles): AnalysisInput => ({
  spec,
  robot: DEFAULT_ROBOT,
  bumperLengthMeters: 0.85,
  bumperWidthMeters: 0.85,
  fieldLengthMeters: 16.54,
  fieldWidthMeters: 8.07,
  obstacles,
});

describe("cusps", () => {
  it("finds the one in Joe's path, where it doubles back near the end, and the tangent that makes it", () => {
    const g = PathGeometry.build(joe.waypoints);
    const at = g.cusps;
    expect(at).toHaveLength(1);
    expect(at[0]!).toBeCloseTo(11.67, 1);
    expect(reversedTangents(joe)).toEqual([4]);
  });

  it("is an error check, naming the waypoint to reset", () => {
    const a = analyse(input(joe));
    if (!a.ok) throw new Error(a.error);
    const c = a.checks.find((k) => k.kind === "cusp")!;
    expect(c.severity).toBe("error");
    expect(c.message).toMatch(/^The path reverses direction at s = 11\.6\d m \(a cusp\): the robot will overshoot and come back\.$/);
    expect(c.waypoint).toBe(4);
  });

  it("resetting the waypoint's curve to automatic removes it", () => {
    const fixed = { ...joe, waypoints: joe.waypoints.map((w, i) => (i === 4 ? makeWaypoint(w.xMeters, w.yMeters, Number.NaN, 1, w.headingRadians) : w)) };
    expect(hasReversal(fixed)).toBe(false);
  });

  it("the optimiser repairs Joe's path (no reversal, the waypoints where they were)", () => {
    const r = optimise(input(joe), { maxEvaluations: 600, timeBudgetMs: Infinity, seed: 1 });
    expect(r.changed).toBe(true);
    expect(hasReversal(r.spec)).toBe(false);
    expect(r.spec.waypoints.map((w) => [w.xMeters, w.yMeters, w.headingRadians])).toEqual(joe.waypoints.map((w) => [w.xMeters, w.yMeters, w.headingRadians]));
    const a = analyse(input(r.spec));
    expect(a.ok && a.checks.filter((c) => c.kind === "cusp")).toEqual([]);
  }, 60_000);

  it("the optimiser never produces a reversal, from paths whose tangents are near the limit", () => {
    for (const [seed, end] of [
      [1, 95],
      [2, -95],
      [3, 80],
      [4, 170],
    ] as const) {
      const spec = makePathSpec({
        waypoints: [makeWaypoint(-2, -1, 0, 1, 0), makeWaypoint(0, 0.5), makeWaypoint(2, -1, (end * Math.PI) / 180, 0.5, 0)],
        maxMetersPerSecond: 4,
        maxMetersPerSecondSquared: 6,
      });
      for (const moveWaypoints of [false, true]) {
        const r = optimise(input(spec, []), { maxEvaluations: 300, timeBudgetMs: Infinity, seed, moveWaypoints });
        expect(hasReversal(r.spec)).toBe(false);
      }
    }
  }, 60_000);
});
