// What the live robot connection (NT4) last reported. The client in ../nt4 writes it; the canvas and
// the live panel read it.

import type { LiveRobot } from "../ui/draw.ts";
import { createStore } from "./store.ts";

export interface LiveState {
  readonly status: "off" | "connecting" | "connected" | "error";
  readonly message: string;
  readonly robot: LiveRobot | null;
  /** The follower's telemetry values by key (Phase, Progress, CrossTrack, ...). */
  readonly telemetry: Readonly<Record<string, string | number | boolean>>;
}

export const live = createStore<LiveState>({ status: "off", message: "", robot: null, telemetry: {} });
