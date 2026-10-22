import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { allSettled, courseWinner, parseMatched, planSums, PRIMARY, scoreboard, verdict, type MatchedData, type RunStats } from "../src/bench/matchedData.ts";

const SNAPSHOT = parseMatched(readFileSync(join(__dirname, "..", "public", "bench", "matched.json"), "utf8"));

function run(t: number, never = 0, err = 0.1): RunStats {
  return { n: 10, t_5cm: t - 0.1, t_settle: never === 10 ? null : t, t_settle_nan: never, err_end_cm: err, ct_own_rms_cm: 1, wheel_slip_ms: 0, overshoot_cm: 0 };
}

const TINY: MatchedData = {
  schema: 1,
  seeds: 10,
  methods: [
    { key: "ours_opt", label: "Ours" },
    { key: "choreo", label: "Choreo" },
  ],
  courses: { plan: ["a", "b"], disturbed: ["hit_mid_path"] },
  courseLabels: {},
  settleRule: "",
  rules: {
    equalPush: {
      traction: 6.5,
      plans: { a: { choreo: 1.0, ours_opt: 1.1 }, b: { choreo: 2.0, ours_opt: 2.0 } },
      runs: {
        truth: {
          a: { ours_opt: run(1.3), choreo: run(1.2, 3) },
          b: { ours_opt: run(2.3), choreo: run(2.2) },
          hit_mid_path: { ours_opt: run(2.5), choreo: run(2.4, 10, 40) },
        },
      },
    },
  },
};

describe("matched-limits head-to-head", () => {
  it("only counts a course for a method whose every run settled", () => {
    expect(allSettled(TINY.rules.equalPush!.runs.truth!.a!.choreo!)).toBe(false);
    expect(courseWinner(TINY, "equalPush", "truth", "a", ["ours_opt", "choreo"])).toBe("ours_opt");
    expect(courseWinner(TINY, "equalPush", "truth", "b", ["ours_opt", "choreo"])).toBe("choreo");
  });

  it("scores wins, runs that never settle and sums", () => {
    const [ours, choreo] = scoreboard(TINY, "equalPush", "truth", "plan", ["ours_opt", "choreo"]);
    expect(ours).toMatchObject({ wins: 1, never: 0, runs: 20 });
    expect(ours!.settledSum).toBeCloseTo(3.6);
    expect(choreo).toMatchObject({ wins: 1, never: 3, runs: 20 });
    expect(planSums(TINY, "equalPush")).toEqual({ choreo: 3.0, ours_opt: 3.1 });
  });

  it("says Choreo plans faster when it does", () => {
    const v = verdict(TINY, "equalPush", "truth", ["ours_opt", "choreo"]);
    expect(v.join("\n")).toContain("Choreo's solver plans the fastest paths: 3.00 s");
    expect(v.join("\n")).toContain("Runs that never settle, of 20 each: Choreo 3");
  });

  it("reads the committed snapshot", () => {
    expect(Object.keys(SNAPSHOT.rules)).toEqual(["equalPush", "tipOrGrip"]);
    for (const rule of Object.keys(SNAPSHOT.rules)) {
      for (const pose of ["truth", "odom"]) {
        const board = scoreboard(SNAPSHOT, rule, pose, "plan", PRIMARY);
        expect(board.map((s) => s.method)).toEqual(PRIMARY);
        expect(verdict(SNAPSHOT, rule, pose, PRIMARY).length).toBeGreaterThan(2);
      }
    }
  });
});
