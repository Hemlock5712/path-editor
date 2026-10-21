// The pathfinder port against lib's golden plans (fixtures/plan/plans.json, written by PlanFixtureGen): the grown
// obstacles, the graph size, and for each start and goal on the 2026 field the waypoints, the planner's figures
// and the path every 0.25 m, to 1e-9 (relative where values are larger than 1).

import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { wrap } from "../src/angles.ts";
import { fieldFromJson, Pathfinder } from "../src/plan/pathfinder.ts";
import { DEFAULT_ROBOT } from "../src/robot.ts";

const DIR = join(import.meta.dirname, "..", "..", "..", "fixtures");
const TOLERANCE = 1e-9;

interface PlanCase {
  name: string;
  start: [number, number, number];
  velocity: [number, number, number];
  goal: [number, number, number];
  endSpeed: number;
  endTangent?: number;
  ok: boolean;
  waypoints: [number, number][];
  routeLengthMeters: number;
  straightLineMeters: number;
  startSnapMeters: number;
  goalSnapMeters: number;
  refits: number;
  clear: boolean;
  minClearanceMeters: number;
  lengthMeters: number;
  totalTimeSeconds: number;
  samples: number[][];
}

const fx = JSON.parse(readFileSync(join(DIR, "plan", "plans.json"), "utf8")) as {
  field: string;
  grown: [number, number][][];
  nodeCount: number;
  plans: PlanCase[];
};
const field = fieldFromJson(JSON.parse(readFileSync(join(DIR, fx.field), "utf8")));
const planner = new Pathfinder(field, DEFAULT_ROBOT);

function dev(expected: number, actual: number, angle = false): number {
  const d = angle ? Math.abs(wrap(actual - expected)) : Math.abs(actual - expected);
  return d / Math.max(1, Math.abs(expected));
}

describe("pathfinder parity with lib (fixtures/plan)", () => {
  it("grows the obstacles and builds the same graph", () => {
    expect(planner.grown.length).toBe(fx.grown.length);
    let worst = 0;
    planner.grown.forEach((p, i) => {
      expect(p.size).toBe(fx.grown[i]!.length);
      fx.grown[i]!.forEach(([x, y], k) => {
        worst = Math.max(worst, dev(x, p.xs[k]!), dev(y, p.ys[k]!));
      });
    });
    expect(worst).toBeLessThanOrEqual(TOLERANCE);
    expect(planner.nodeCount).toBe(fx.nodeCount);
  });

  for (const c of fx.plans) {
    it(`plans ${c.name}`, () => {
      const r = planner.plan(
        { xMeters: c.start[0], yMeters: c.start[1], headingRadians: c.start[2] },
        { vxMetersPerSecond: c.velocity[0], vyMetersPerSecond: c.velocity[1], omegaRadPerSec: c.velocity[2] },
        { xMeters: c.goal[0], yMeters: c.goal[1], headingRadians: c.goal[2] },
        c.endSpeed,
        c.endTangent ?? Number.NaN,
      );
      expect(r.path !== null).toBe(c.ok);
      expect(r.waypoints.length).toBe(c.waypoints.length);
      expect(r.refits).toBe(c.refits);
      expect(r.clear).toBe(c.clear);
      let worst = 0;
      r.waypoints.forEach(([x, y], i) => {
        worst = Math.max(worst, dev(c.waypoints[i]![0], x), dev(c.waypoints[i]![1], y));
      });
      for (const k of ["routeLengthMeters", "straightLineMeters", "startSnapMeters", "goalSnapMeters", "minClearanceMeters"] as const) {
        worst = Math.max(worst, dev(c[k], r[k]));
      }
      const path = r.path!;
      worst = Math.max(worst, dev(c.lengthMeters, path.lengthMeters), dev(c.totalTimeSeconds, path.totalTimeSeconds));
      for (const [s, x, y, h, v, t] of c.samples as [number, number, number, number, number, number][]) {
        const p = path.sample(Math.min(path.lengthMeters, s));
        worst = Math.max(worst, dev(s, p.s), dev(x, p.xMeters), dev(y, p.yMeters), dev(h, p.headingRadians, true),
          dev(v, p.metersPerSecond), dev(t, p.seconds));
      }
      expect(worst).toBeLessThanOrEqual(TOLERANCE);
    });
  }

  it("keeps every plan clear of the obstacles", () => {
    for (const c of fx.plans) expect(c.minClearanceMeters).toBeGreaterThan(0);
  });
});
