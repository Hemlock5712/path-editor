// The golden vectors: every fixture written by lib's FixtureGen must come out of this port to 1e-9
// (relative where values are larger than 1).

import { readFileSync, readdirSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { wrap } from "../src/angles.ts";
import { Path, type PathSample } from "../src/path.ts";
import { asObject, pathFromJson, robotFromJson } from "../src/pathFile.ts";
import type { JsonValue } from "../src/json.ts";

const FIXTURES = join(import.meta.dirname, "..", "..", "..", "fixtures");
const TOLERANCE = 1e-9;

interface Fixture {
  inputs: { path: JsonValue; robot: JsonValue; gridMeters: number; stepMeters: number };
  lengthMeters: number;
  totalTimeSeconds: number;
  samples: Record<string, number[]>;
}

const COLUMNS: Record<string, (p: PathSample) => number> = {
  s: (p) => p.s,
  x: (p) => p.xMeters,
  y: (p) => p.yMeters,
  tangentX: (p) => p.tangentX,
  tangentY: (p) => p.tangentY,
  curvature: (p) => p.curvature,
  dCurvatureDs: (p) => p.dCurvatureDs,
  heading: (p) => p.headingRadians,
  headingPerMeter: (p) => p.headingRadiansPerMeter,
  velocity: (p) => p.metersPerSecond,
  acceleration: (p) => p.metersPerSecondSquared,
  time: (p) => p.seconds,
};

/** The same stations as ProfileSamples.stations in Java. */
function stations(lengthMeters: number, stepMeters: number): number[] {
  const n = Math.ceil(lengthMeters / stepMeters - 1e-9);
  const s: number[] = [];
  for (let i = 0; i < n; i++) s.push(i * stepMeters);
  s.push(lengthMeters);
  return s;
}

function deviation(expected: number, actual: number, angle: boolean): number {
  const diff = angle ? Math.abs(wrap(actual - expected)) : Math.abs(actual - expected);
  return diff / Math.max(1, Math.abs(expected));
}

const files = readdirSync(FIXTURES).filter((f) => f.endsWith(".json"));

describe("fixtures", () => {
  it("there are fixtures to check", () => {
    expect(files.length).toBeGreaterThanOrEqual(6);
  });

  for (const file of files) {
    it(`reproduces ${file} to 1e-9`, () => {
      const fx = JSON.parse(readFileSync(join(FIXTURES, file), "utf8")) as Fixture;
      const spec = pathFromJson(asObject(fx.inputs.path, "path"));
      const robot = robotFromJson(asObject(fx.inputs.robot, "robot"));
      const path = Path.build(spec, robot, fx.inputs.gridMeters);

      expect(deviation(fx.lengthMeters, path.lengthMeters, false)).toBeLessThanOrEqual(TOLERANCE);
      expect(deviation(fx.totalTimeSeconds, path.totalTimeSeconds, false)).toBeLessThanOrEqual(TOLERANCE);

      const at = stations(path.lengthMeters, fx.inputs.stepMeters);
      expect(at.length).toBe(fx.samples.s!.length);
      let worst = 0;
      let worstWhere = "";
      for (let i = 0; i < at.length; i++) {
        const p = path.sample(at[i]!);
        for (const [name, get] of Object.entries(COLUMNS)) {
          const dev = deviation(fx.samples[name]![i]!, get(p), name === "heading");
          if (dev > worst) {
            worst = dev;
            worstWhere = `${name} at s = ${at[i]}`;
          }
        }
      }
      console.log(`${file}: ${at.length} samples, worst deviation ${worst.toExponential(2)} (${worstWhere || "none"})`);
      expect(worst, worstWhere).toBeLessThanOrEqual(TOLERANCE);
    });
  }
});
