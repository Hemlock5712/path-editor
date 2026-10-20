// A minimal NT4 client: subscribes to the robot's pose and the follower's telemetry and writes them
// to the live store. Read-only (it never publishes), and it ignores the time sync, so values show
// as they arrive. Tested against recorded frames only, not yet against a robot.
//
// WPILib 2027's Telemetry publishes under /Telemetry, so PathTelemetry("Path/ToReef") appears at
// /Telemetry/Path/ToReef/{Phase, Progress, Remaining, CrossTrack, Reference, Goal, TailRebuilds}.

import { live } from "../state/live.ts";
import { decodeAll, type MsgValue } from "./msgpack.ts";

export interface LiveSettings {
  /** Robot address: 10.57.12.2, localhost for simulation, ... */
  readonly host: string;
  /** The robot pose topic: a struct:Pose2d, or a double[] [x, y, degrees] like Field2d's. */
  readonly poseTopic: string;
  /** Where PathTelemetry tables are: the editor shows every value under it. */
  readonly telemetryPrefix: string;
}

export const DEFAULT_LIVE: LiveSettings = {
  host: "localhost",
  poseTopic: "/SmartDashboard/Field/Robot",
  telemetryPrefix: "/Telemetry/Path/",
};

interface Topic {
  readonly name: string;
  readonly type: string;
}

/** A Pose2d struct: Translation2d (x, y) then Rotation2d (radians), little-endian doubles. */
export function decodePose2d(bytes: Uint8Array): { x: number; y: number; heading: number } | null {
  if (bytes.length < 24) return null;
  const v = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  return { x: v.getFloat64(0, true), y: v.getFloat64(8, true), heading: v.getFloat64(16, true) };
}

/** A pose value of either supported type, or null. */
export function poseOf(type: string, value: MsgValue): { x: number; y: number; heading: number } | null {
  if (type === "struct:Pose2d" && value instanceof Uint8Array) return decodePose2d(value);
  if (type === "double[]" && Array.isArray(value) && value.length >= 3) {
    const [x, y, deg] = value as number[];
    return { x: x!, y: y!, heading: (deg! * Math.PI) / 180 };
  }
  return null;
}

export class Nt4Client {
  private socket: WebSocket | null = null;
  private readonly topics = new Map<number, Topic>();
  private closedByUser = false;

  constructor(private readonly settings: LiveSettings) {}

  connect(): void {
    this.closedByUser = false;
    const url = `ws://${this.settings.host}:5810/nt/pathEditor5712`;
    live.set({ status: "connecting", message: url, robot: null, telemetry: {} });
    let socket: WebSocket;
    try {
      socket = new WebSocket(url, ["v4.1.networktables.first.wpi.edu", "networktables.first.wpi.edu"]);
    } catch (e) {
      live.set({ status: "error", message: (e as Error).message });
      return;
    }
    socket.binaryType = "arraybuffer";
    socket.onopen = () => {
      live.set({ status: "connected", message: url });
      const topics = [this.settings.telemetryPrefix, this.settings.poseTopic].filter((t) => t !== "");
      socket.send(JSON.stringify([{ method: "subscribe", params: { topics, subuid: 1, options: { prefix: true } } }]));
    };
    socket.onmessage = (event) => {
      if (typeof event.data === "string") this.onText(event.data);
      else this.onBinary(new Uint8Array(event.data as ArrayBuffer));
    };
    socket.onerror = () => live.set({ status: "error", message: `Couldn't reach ${url}` });
    socket.onclose = () => {
      if (!this.closedByUser && live.get().status !== "error") live.set({ status: "error", message: "Connection closed" });
    };
    this.socket = socket;
  }

  disconnect(): void {
    this.closedByUser = true;
    this.socket?.close();
    this.socket = null;
    this.topics.clear();
    live.set({ status: "off", message: "", robot: null, telemetry: {} });
  }

  /** Announcements: which id is which topic. */
  onText(text: string): void {
    let messages: { method: string; params: { id: number; name: string; type: string } }[];
    try {
      messages = JSON.parse(text);
    } catch {
      return;
    }
    for (const m of messages) {
      if (m.method === "announce") this.topics.set(m.params.id, { name: m.params.name, type: m.params.type });
      else if (m.method === "unannounce") this.topics.delete(m.params.id);
    }
  }

  /** Values: [id, timestamp, type, value] for each update in the frame. */
  onBinary(bytes: Uint8Array): void {
    let values: MsgValue[];
    try {
      values = decodeAll(bytes);
    } catch {
      return;
    }
    const telemetry = { ...live.get().telemetry };
    let robot = live.get().robot;
    let changed = false;
    for (const v of values) {
      if (!Array.isArray(v) || v.length < 4) continue;
      const topic = this.topics.get(Number(v[0]));
      if (!topic) continue;
      const value = v[3]!;
      if (topic.name === this.settings.poseTopic) {
        const pose = poseOf(topic.type, value);
        if (pose) {
          robot = { ...robot, pose };
          changed = true;
        }
      } else if (topic.name.startsWith(this.settings.telemetryPrefix)) {
        const key = topic.name.slice(this.settings.telemetryPrefix.length);
        if (key.endsWith("/Reference")) {
          const reference = poseOf(topic.type, value);
          if (reference) robot = { ...robot, reference };
        } else if (typeof value === "number" || typeof value === "string" || typeof value === "boolean") {
          telemetry[key] = value;
        }
        changed = true;
      }
    }
    if (changed) live.set({ telemetry, robot });
  }
}
