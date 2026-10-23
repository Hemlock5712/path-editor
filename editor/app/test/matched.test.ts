import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { allFinished, andList, courseWinners, parseMatched, planTotals, SHOWN, summaries, type MatchedData, type RunStats } from "../src/bench/matchedData.ts";

const SNAPSHOT = parseMatched(readFileSync(join(__dirname, "..", "public", "bench", "matched.json"), "utf8"));

function run(t: number, never = 0, err = 0.1): RunStats {
  return { n: 10, t_5cm: t - 0.1, t_settle: never === 10 ? null : t, t_settle_nan: never, err_end_cm: err, ct_own_rms_cm: 1, wheel_slip_ms: 0, overshoot_cm: 0 };
}

const TINY: MatchedData = {
  schema: 1,
  seeds: 10,
  methods: [],
  courses: { plan: ["a", "b", "c"], disturbed: [] },
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
          c: { ours_opt: run(3.0), choreo: run(2.9, 10, 40) },
        },
      },
    },
  },
};

describe("the bench page's data", () => {
  it("only lets a method win a course when every run finished", () => {
    expect(allFinished(TINY.rules.equalPush!.runs.truth!.a!.choreo!)).toBe(false);
    expect(courseWinners(TINY, "equalPush", "truth", "a")).toEqual(["ours_opt"]);
    expect(courseWinners(TINY, "equalPush", "truth", "b")).toEqual(["choreo"]);
  });

  it("counts a tie at the hundredth for both", () => {
    const tie: MatchedData = { ...TINY, rules: { equalPush: { ...TINY.rules.equalPush!, runs: { truth: { a: { ours_opt: run(2.562), choreo: run(2.558) } } } } } };
    expect(courseWinners(tie, "equalPush", "truth", "a")).toEqual(["ours_opt", "choreo"]);
  });

  it("sums the runs that finish, the wins and the time, and names courses never finished", () => {
    const [ours, choreo] = summaries(TINY, "equalPush", "truth", "plan");
    expect(ours).toMatchObject({ method: "ours_opt", wins: 2, finished: 30, runs: 30, neverFinished: [] });
    expect(ours!.totalTime).toBeCloseTo(6.6);
    expect(choreo).toMatchObject({ method: "choreo", wins: 1, finished: 17, neverFinished: ["c"] });
    expect(choreo!.totalTime).toBeNaN();
    expect(planTotals(TINY, "equalPush")).toEqual({ choreo: 3.0, ours_opt: 3.1 });
  });

  it("joins lists in words", () => {
    expect(andList(["a"])).toBe("a");
    expect(andList(["a", "b", "c"])).toBe("a, b and c");
  });

  it("reads the committed snapshot: every shown method on every course and pose source, at the robot's rule", () => {
    expect(Object.keys(SNAPSHOT.rules)).toEqual(["tipOrGrip"]);
    for (const rule of Object.keys(SNAPSHOT.rules)) {
      for (const pose of ["truth", "odom"]) {
        for (const group of ["plan", "disturbed"] as const) {
          expect(summaries(SNAPSHOT, rule, pose, group).map((s) => s.method)).toEqual(SHOWN);
        }
      }
    }
  });
});
