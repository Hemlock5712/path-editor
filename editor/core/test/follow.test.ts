// The follower port against lib's golden closed-loop traces (fixtures/follow, written by FollowFixtureGen):
// every loop's pose, command, progress, cross-track and phase, and the loop each event fired in, to 1e-9
// (relative where values are larger than 1).

import { readFileSync, readdirSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { wrap } from "../src/angles.ts";
import type { FieldFrame } from "../src/frame.ts";
import { PHASES, followerConfig, type FollowerConfig } from "../src/follow.ts";
import type { JsonValue } from "../src/json.ts";
import { javaHypot } from "../src/jmath.ts";
import { Path } from "../src/path.ts";
import { asObject, pathFromJson, robotFromJson } from "../src/pathFile.ts";
import { runFollowerSim, type DriveSettings, type Shove } from "../src/sim.ts";

const DIR = join(import.meta.dirname, "..", "..", "..", "fixtures", "follow");
const TOLERANCE = 1e-9;

interface FollowFixture {
  inputs: {
    path: JsonValue;
    robot: JsonValue;
    follower: Partial<Omit<FollowerConfig, "robot">>;
    drive: DriveSettings;
    frame: FieldFrame;
    shove?: Shove;
    afterSeconds: number;
    maxSeconds: number;
  };
  finishLoop: number;
  events: { name: string; loop: number }[];
  loops: Record<string, number[]>;
}

function deviation(expected: number, actual: number, angle: boolean): number {
  const diff = angle ? Math.abs(wrap(actual - expected)) : Math.abs(actual - expected);
  return diff / Math.max(1, Math.abs(expected));
}

const files = readdirSync(DIR).filter((f) => f.endsWith(".json"));

describe("follower golden traces", () => {
  it("there are traces to check", () => {
    expect(files.length).toBeGreaterThanOrEqual(12);
  });

  for (const file of files) {
    it(`reproduces ${file} loop for loop`, () => {
      const fx = JSON.parse(readFileSync(join(DIR, file), "utf8")) as FollowFixture;
      const robot = robotFromJson(asObject(fx.inputs.robot, "robot"));
      const path = Path.build(pathFromJson(asObject(fx.inputs.path, "path")), robot);
      const cfg = followerConfig(robot, fx.inputs.follower);
      const trace = runFollowerSim(path, cfg, fx.inputs.drive, fx.inputs.frame, fx.inputs.shove ?? null,
        fx.inputs.afterSeconds, fx.inputs.maxSeconds);

      expect(trace.finishLoop).toBe(fx.finishLoop);
      expect(trace.events).toEqual(fx.events);
      expect(trace.t.length).toBe(fx.loops.t!.length);
      let worst = 0;
      let where = "";
      for (const column of ["t", "x", "y", "heading", "vx", "vy", "omega", "s", "crossTrack"] as const) {
        const want = fx.loops[column]!;
        const got = trace[column];
        for (let i = 0; i < want.length; i++) {
          const dev = deviation(want[i]!, got[i]!, column === "heading");
          if (dev > worst) {
            worst = dev;
            where = `${column} at loop ${i}`;
          }
        }
      }
      expect(trace.phase.map((p) => PHASES.indexOf(p))).toEqual(fx.loops.phase);
      console.log(`${file}: ${trace.t.length} loops, finish loop ${trace.finishLoop}, worst deviation ${worst.toExponential(2)} (${where || "none"})`);
      expect(worst, where).toBeLessThanOrEqual(TOLERANCE);
    });
  }
});

describe("javaHypot", () => {
  it("agrees with sqrt(x² + y²) and handles the edges", () => {
    for (const [x, y] of [[3, 4], [1e-200, 1e-200], [1e200, 1e200], [0, -5], [1e300, 1], [0.1, 0.2]] as const) {
      expect(javaHypot(x, y)).toBeCloseTo(Math.hypot(x, y), 12);
      expect(Math.abs(javaHypot(x, y) / Math.hypot(x, y) - 1)).toBeLessThan(1e-15);
    }
    expect(javaHypot(Infinity, Number.NaN)).toBe(Infinity);
    expect(Number.isNaN(javaHypot(Number.NaN, 1))).toBe(true);
    expect(javaHypot(3, 4)).toBe(5);
  });
});
