import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { bestValidByMetric, parseBench, scoreboard, tableFor, validity, type BenchData } from "../src/bench/benchData.ts";

const stat = (mean: number | null, worst = mean, nan = 0) => ({ mean, worst, nan });

/** Three methods on one course: a fast one that never settles, a slow clean one, a quick one that ends off. */
function sample(): BenchData {
  const row = (tDone: number, tSettle: number | null, unsettled: number, errMean: number, errWorst: number) => ({
    n: 10,
    t_done: stat(tDone),
    t_settle: stat(tSettle, tSettle, unsettled),
    err_end_cm: stat(errMean, errWorst),
    ct_rms_cm: stat(1),
  });
  return {
    schema: 1,
    source: "test",
    meta: { seeds: [0], courses: ["c"], poses: ["odom", "truth"], tracePose: "truth", traceSeeds: [0] },
    metrics: [
      { key: "t_done", label: "time to done", unit: "s" },
      { key: "t_settle", label: "time settled", unit: "s" },
      { key: "err_end_cm", label: "final error", unit: "cm" },
    ],
    methods: [
      { key: "bline_readme", label: "BLine README" },
      { key: "ours_v2", label: "Ours" },
      { key: "choreo", label: "Choreo" },
      { key: "pathplanner_tuned", label: "PP" },
    ],
    courses: [{ key: "c", description: "a course", look: "the end" }],
    runs: {
      truth: {
        c: {
          bline_readme: row(1.1, null, 10, 41, 43),
          ours_v2: row(1.5, 1.6, 0, 0.1, 0.2),
          choreo: row(1.3, 1.4, 0, 6, 9),
          pathplanner_tuned: row(1.8, 2.0, 3, 0.5, 1),
        },
      },
      odom: { c: {} },
    },
    geometry: {},
    traces: {},
  };
}

describe("reading the bench fairly", () => {
  it("only counts runs that settled every seed and ended within tolerance", () => {
    const t = tableFor(sample(), "c");
    expect(validity(t, "truth", "c", "bline_readme")).toMatchObject({ valid: false, reason: "never settled" });
    expect(validity(t, "truth", "c", "choreo")).toMatchObject({ valid: false, reason: "9 cm off" });
    expect(validity(t, "truth", "c", "pathplanner_tuned")).toMatchObject({ valid: false, reason: "3/10 never settled", settled: 7 });
    expect(validity(t, "truth", "c", "ours_v2")).toMatchObject({ valid: true, settled: 10, n: 10 });
  });

  it("ranks valid rows by time settled, then reliability and final error; the rest follow unranked", () => {
    const t = tableFor(sample(), "c");
    const rows = scoreboard(t, "truth", "c", ["bline_readme", "choreo", "ours_v2", "pathplanner_tuned"]);
    expect(rows.map((r) => [r.method, r.rank])).toEqual([
      ["ours_v2", 1],
      ["choreo", null],
      ["pathplanner_tuned", null],
      ["bline_readme", null],
    ]);
  });

  it("never makes an invalid run the best in a column", () => {
    const t = tableFor(sample(), "c");
    const best = bestValidByMetric(t, "truth", "c", ["bline_readme", "choreo", "ours_v2", "pathplanner_tuned"]);
    expect(best.t_done).toBe("ours_v2"); // bline_readme (1.1 s) and choreo (1.3 s) are faster but don't count
    expect(best.err_end_cm).toBe("ours_v2");
  });

  it("the committed snapshot: perfect-pose traces, a 'what to look at' line per course, and BLine's README gains not valid", () => {
    const text = readFileSync(join(import.meta.dirname, "..", "public", "bench", "results-lite.json"), "utf8");
    const data = parseBench(text);
    expect(data.meta.tracePose).toBe("truth");
    expect(data.courses.every((c) => (c.look ?? "").length > 10)).toBe(true);
    const t = tableFor(data, "straight_3m");
    expect(validity(t, "truth", "straight_3m", "bline_readme").valid).toBe(false);
    expect(validity(t, "truth", "straight_3m", "ours_v2").valid).toBe(true);
  });
});
