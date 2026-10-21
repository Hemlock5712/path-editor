// The Bench view's state: the loaded export and what is chosen. Nothing here touches the project.

import { DEFAULT_SHOWN, parseBench, type BenchData } from "../bench/benchData.ts";
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
}

export const bench = createStore<BenchState>({
  open: false,
  data: null,
  fileName: "",
  error: null,
  pose: "",
  course: "",
  seed: "0",
  shown: DEFAULT_SHOWN,
  traces: true,
});

export function toggleBench(): void {
  bench.set((s) => ({ open: !s.open }));
}

/** Loads an export; the choices reset to its first pose source, course and seed. */
export function loadBench(text: string, fileName: string): void {
  try {
    const data = parseBench(text);
    const methods = new Set(data.methods.map((m) => m.key));
    bench.set({
      data,
      fileName,
      error: null,
      pose: data.meta.poses[0] ?? "",
      course: data.meta.courses[0] ?? "",
      seed: String(data.meta.traceSeeds[0] ?? 0),
      shown: DEFAULT_SHOWN.filter((m) => methods.has(m)),
    });
  } catch (e) {
    bench.set({ error: `${fileName}: ${(e as Error).message}` });
  }
}

export function toggleShown(method: string): void {
  bench.set((s) => ({ shown: s.shown.includes(method) ? s.shown.filter((m) => m !== method) : [...s.shown, method] }));
}
