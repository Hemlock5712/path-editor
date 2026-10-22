// Connects the NT4 client to the live store: the robot's pose, and the path follower's telemetry as
// PathTelemetry publishes it (WPILib 2027 puts Telemetry under /Telemetry):
//   /Telemetry/PathFollower/{Active, EventsFired, LastEvent}
//   /Telemetry/Path/<active path>/{Phase, Progress, Remaining, CrossTrack, Reference, Goal}
// Updates are gathered and written to the store at most every 50 ms.

import { live, LIVE_OFF, type LiveState } from "../state/live.ts";
import { Recorder } from "./recorder.ts";
import { Nt4Client, NT4_PORT, poseOf, robotAddress, type Nt4Value } from "./client.ts";

export interface LiveSettings {
  /** Team number (5712 → 10.57.12.2) or an address; localhost for a simulation. */
  readonly address: string;
  readonly port: number;
  /** The robot's pose: a struct:Pose2d, or a Field2d-style double[] [x, y, degrees]. */
  readonly poseTopic: string;
}

export const DEFAULT_LIVE: LiveSettings = { address: "localhost", port: NT4_PORT, poseTopic: "/Telemetry/Robot/Pose" };

const FOLLOWER = "/Telemetry/PathFollower/";
const PATHS = "/Telemetry/Path/";
const FLUSH_MS = 50;

export class LiveFeed {
  readonly client: Nt4Client;
  /** Every run of the follower, recorded loop by loop. */
  readonly recorder: Recorder;
  private pending: Partial<LiveState> = {};
  private flushTimer: ReturnType<typeof setTimeout> | null = null;

  constructor(
    private readonly settings: LiveSettings,
    onRecording: (recorder: Recorder) => void = () => undefined,
  ) {
    this.recorder = new Recorder(settings.poseTopic, 10, () => onRecording(this.recorder));
    this.client = new Nt4Client({
      host: robotAddress(settings.address),
      port: settings.port,
      prefixes: [FOLLOWER, PATHS, settings.poseTopic],
      all: true,
      onValue: (v) => {
        this.recorder.onValue(v);
        this.onValue(v);
      },
      onFrameEnd: () => this.recorder.onFrameEnd(),
      onStatus: (status, detail) => {
        if (status === "closed") live.set({ ...LIVE_OFF });
        else live.set({ status, message: detail, ...(status === "connecting" ? { synced: false } : {}) });
      },
      onTimeSync: (_offset, rtt) => this.update({ rttMs: rtt / 1000, synced: true }),
    });
  }

  start(): void {
    live.set({ ...LIVE_OFF, status: "connecting" });
    this.client.connect();
  }

  stop(): void {
    if (this.flushTimer) clearTimeout(this.flushTimer);
    this.flushTimer = null;
    this.pending = {};
    this.client.close();
    live.set(LIVE_OFF);
  }

  private update(change: Partial<LiveState>): void {
    this.pending = { ...this.pending, ...change };
    if (!this.flushTimer) {
      this.flushTimer = setTimeout(() => {
        this.flushTimer = null;
        live.set(this.pending);
        this.pending = {};
      }, FLUSH_MS);
    }
  }

  /** The newest value of a field, pending or already in the store. */
  private current<K extends keyof LiveState>(key: K): LiveState[K] {
    return (key in this.pending ? this.pending[key] : live.get()[key]) as LiveState[K];
  }

  onValue(v: Nt4Value): void {
    const name = v.topic.name;
    const value = v.value;
    if (name === this.settings.poseTopic) {
      const pose = poseOf(v.topic.type, value);
      const serverNow = this.client.serverNowUs();
      if (pose) {
        this.update({
          robot: { ...this.current("robot"), pose },
          poseAgeMs: serverNow === null ? Number.NaN : (serverNow - v.serverTimeUs) / 1000,
        });
      }
      return;
    }
    if (name.startsWith(FOLLOWER)) {
      const key = name.slice(FOLLOWER.length);
      if (key === "Active" && typeof value === "string") this.update({ active: value });
      else if (key === "EventsFired" && Array.isArray(value)) this.update({ eventsFired: value.map(String) });
      return;
    }
    const active = this.current("active");
    if (!active || !name.startsWith(`${PATHS}${active}/`)) return;
    const key = name.slice(`${PATHS}${active}/`.length);
    if (key === "Reference") {
      const reference = poseOf(v.topic.type, value);
      if (reference) this.update({ robot: { ...this.current("robot"), reference } });
    } else if (key === "Phase" && typeof value === "string") this.update({ phase: value });
    else if (key === "Progress" && typeof value === "number") this.update({ progressMeters: value });
    else if (key === "Remaining" && typeof value === "number") this.update({ remainingMeters: value });
    else if (key === "CrossTrack" && typeof value === "number") this.update({ crossTrackMeters: value });
  }
}
