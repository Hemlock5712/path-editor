// The Telemetry page's state: the NT4 connection (one for the app, it outlives the page), the recorded
// runs (the newest few, plus any imported), which one is shown, and the cursor through it.

import { LiveFeed, type LiveSettings } from "../nt4/liveFeed.ts";
import { importRun, type RecordedRun } from "../nt4/recorder.ts";
import { createStore } from "./store.ts";

export interface TelemetryState {
  /** Recorded and imported runs, oldest first. */
  readonly runs: readonly RecordedRun[];
  /** Bumped whenever a run gains samples (the runs are appended to in place). */
  readonly version: number;
  readonly selectedId: string | null;
  /** Keep the newest run selected and the cursor on its latest loop. */
  readonly followLive: boolean;
  /** The cursor: seconds into the selected run. */
  readonly cursor: number;
  readonly axis: "t" | "s";
  readonly playing: boolean;
}

export const telemetry = createStore<TelemetryState>({
  runs: [],
  version: 0,
  selectedId: null,
  followLive: true,
  cursor: 0,
  axis: "t",
  playing: false,
});

let feed: LiveFeed | null = null;
let imported: RecordedRun[] = [];
let pending = false;

function publish(recorded: readonly RecordedRun[]): void {
  const runs = [...recorded, ...imported];
  const s = telemetry.get();
  const newest = recorded.at(-1);
  const follow = s.followLive && newest !== undefined;
  telemetry.set({
    runs,
    version: s.version + 1,
    selectedId: follow ? newest.id : s.selectedId && runs.some((r) => r.id === s.selectedId) ? s.selectedId : (newest?.id ?? null),
    cursor: follow ? (newest.samples.at(-1)?.t ?? 0) : s.cursor,
  });
}

export function connect(settings: LiveSettings): void {
  feed?.stop();
  feed = new LiveFeed(settings, (recorder) => {
    // the recorder changes every loop; the page redraws at most every 100 ms
    if (pending) return;
    pending = true;
    setTimeout(() => {
      pending = false;
      publish(recorder.runs);
    }, 100);
  });
  feed.start();
}

export function disconnect(): void {
  feed?.stop();
  feed = null;
}

export function selectRun(id: string): void {
  const newest = feed?.recorder.runs.at(-1);
  telemetry.set({ selectedId: id, followLive: newest?.id === id && !newest.finished, cursor: 0, playing: false });
}

export function setCursor(t: number): void {
  telemetry.set({ cursor: t, followLive: false });
}

export function followLive(): void {
  telemetry.set({ followLive: true, playing: false });
  if (feed) publish(feed.recorder.runs);
}

export function addImported(text: string): void {
  const run = importRun(text);
  imported = [...imported, run];
  telemetry.set((s) => ({ runs: [...s.runs, run], selectedId: run.id, followLive: false, cursor: 0, version: s.version + 1 }));
}

export function selectedRun(s: TelemetryState = telemetry.get()): RecordedRun | null {
  return s.runs.find((r) => r.id === s.selectedId) ?? null;
}
