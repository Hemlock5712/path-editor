// The Bench page's state: the matched-limits snapshot. Nothing here touches the project.

import { parseMatched, type MatchedData } from "../bench/matchedData.ts";
import { createStore } from "./store.ts";

export interface BenchState {
  readonly data: MatchedData | null;
  readonly error: string | null;
  readonly loadingSnapshot: boolean;
}

export const bench = createStore<BenchState>({ data: null, error: null, loadingSnapshot: false });

/** The committed snapshot (bench/matched.py export writes it): what the deployed editor shows. */
export const SNAPSHOT_URL = `${import.meta.env.BASE_URL}bench/matched.json`;

/** Fetches the committed snapshot once. */
export async function loadSnapshot(): Promise<void> {
  bench.set({ loadingSnapshot: true });
  try {
    const response = await fetch(SNAPSHOT_URL);
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    bench.set({ data: parseMatched(await response.text()), error: null });
  } catch (e) {
    bench.set({ error: `The bench results didn't load (${(e as Error).message}).` });
  } finally {
    bench.set({ loadingSnapshot: false });
  }
}
