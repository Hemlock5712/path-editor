// The Bench view's state: the loaded export and what is chosen. Nothing here touches the project.

import { parseBench, PRIMARY_METHODS, type BenchData } from "../bench/benchData.ts";
import { createStore } from "./store.ts";

export interface BenchState {
  readonly open: boolean;
  readonly data: BenchData | null;
  readonly fileName: string;
  readonly error: string | null;
  readonly pose: string;
  readonly course: string;
  readonly seed: string;
  readonly shown: readonly string[];
  /** Draw the true traces (else only each method's authored geometry). */
  readonly traces: boolean;
  /** List every method, not just ours and each rival's best-tuned entry. */
  readonly moreMethods: boolean;
  /** The comparison charts: which metric, and all targets or one chart per target. */
  readonly compareMetric: string;
  readonly compareView: "aggregate" | "per";
  readonly loadingSnapshot: boolean;
}

export const bench = createStore<BenchState>({
  open: false,
  data: null,
  fileName: "",
  error: null,
  pose: "",
  course: "",
  seed: "0",
  shown: PRIMARY_METHODS,
  moreMethods: false,
  compareMetric: "t_settle",
  compareView: "aggregate",
  traces: true,
  loadingSnapshot: false,
});

/** The committed snapshot (bench/run.py --export-editor writes it): what the deployed editor opens with. */
export const SNAPSHOT_URL = `${import.meta.env.BASE_URL}bench/results-lite.json`;

/** Fetches the committed snapshot; a file loaded meanwhile wins. */
export async function loadSnapshot(): Promise<void> {
  bench.set({ loadingSnapshot: true });
  try {
    const response = await fetch(SNAPSHOT_URL);
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const text = await response.text();
    if (!bench.get().data) loadBench(text, "results-lite.json (snapshot)");
  } catch (e) {
    if (!bench.get().data) bench.set({ error: `No bench snapshot (${(e as Error).message}): load an export from a file.` });
  } finally {
    bench.set({ loadingSnapshot: false });
  }
}

/** Loads an export; the choices reset to perfect pose (if it has it), its first course and seed, and the main methods. */
export function loadBench(text: string, fileName: string): void {
  try {
    const data = parseBench(text);
    const methods = new Set([...data.methods, ...(data.ppcompare?.methods ?? [])].map((m) => m.key));
    bench.set({
      data,
      fileName,
      error: null,
      pose: data.meta.poses.includes("truth") ? "truth" : (data.meta.poses[0] ?? ""),
      course: data.meta.courses[0] ?? "",
      seed: String(data.meta.traceSeeds[0] ?? 0),
      shown: PRIMARY_METHODS.filter((m) => methods.has(m)),
    });
  } catch (e) {
    bench.set({ error: `${fileName}: ${(e as Error).message}` });
  }
}

export function toggleShown(method: string): void {
  bench.set((s) => ({ shown: s.shown.includes(method) ? s.shown.filter((m) => m !== method) : [...s.shown, method] }));
}
