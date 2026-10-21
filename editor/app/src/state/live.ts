// What the live robot connection (NT4) last reported. nt4/liveFeed.ts writes it; the canvas and the
// live panel read it.

import type { LiveRobot } from "../ui/draw.ts";
import { createStore } from "./store.ts";

export interface LiveState {
  readonly status: "off" | "connecting" | "connected" | "error";
  readonly message: string;
  /** The robot's pose and the follower's reference, real field frame. */
  readonly robot: LiveRobot | null;
  /** The running path's name ("" when none), from /Telemetry/PathFollower/Active. */
  readonly active: string;
  readonly phase: string;
  readonly progressMeters: number;
  readonly remainingMeters: number;
  readonly crossTrackMeters: number;
  readonly eventsFired: readonly string[];
  /** The last time sync round trip, ms, and whether the clocks are synced. */
  readonly rttMs: number;
  readonly synced: boolean;
  /** How old the newest pose is, on the server's clock, ms. */
  readonly poseAgeMs: number;
}

export const LIVE_OFF: LiveState = {
  status: "off",
  message: "",
  robot: null,
  active: "",
  phase: "",
  progressMeters: Number.NaN,
  remainingMeters: Number.NaN,
  crossTrackMeters: Number.NaN,
  eventsFired: [],
  rttMs: Number.NaN,
  synced: false,
  poseAgeMs: Number.NaN,
};

export const live = createStore<LiveState>(LIVE_OFF);
