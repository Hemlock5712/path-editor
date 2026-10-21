// A NetworkTables 4 client over WebSocket, following WPILib 2027's ntcore (NetworkClient.cpp,
// net/ClientImpl.cpp, server/ServerClient4.cpp). Read-only: it subscribes and never publishes.
//
//   - Connects to ws://host:5810/nt/<name> with the v4.1 subprotocol (v4.0 as the fallback).
//   - Text frames are JSON arrays of messages: it sends "subscribe" and handles "announce",
//     "unannounce" and "properties".
//   - Binary frames are MessagePack arrays [topic id, server time µs, type, value], back to back.
//   - Time sync: it sends [-1, 0, int, its own time µs]; the server answers [-1, server time, int, that
//     time]. Then server time ≈ answer's time + round trip / 2, so offset = that − local time. Like ntcore
//     it keeps the estimate from the shortest round trip, and pings again every few seconds.
//   - If the connection drops it reconnects every second until closed.

import { decodeAll, encode, type MsgValue } from "./msgpack.ts";

export const NT4_PORT = 5810;
const PROTOCOLS = ["v4.1.networktables.first.wpi.edu", "networktables.first.wpi.edu"];
const PING_INTERVAL_MS = 3000;
const RECONNECT_MS = 1000;
/** NT4 type code for an integer (the time sync ping's value). */
const INT_TYPE = 2;

/**
 * Where a robot is: a team number becomes 10.TE.AM.2 (5712 → 10.57.12.2); anything else (an address,
 * a host name, localhost for a simulation) is used as given.
 */
export function robotAddress(input: string): string {
  const t = input.trim();
  if (/^\d{1,5}$/.test(t)) {
    const n = Number(t);
    return `10.${Math.floor(n / 100)}.${n % 100}.2`;
  }
  return t;
}

export interface Nt4Topic {
  readonly id: number;
  readonly name: string;
  readonly type: string;
  readonly properties: Readonly<Record<string, unknown>>;
}

export interface Nt4Value {
  readonly topic: Nt4Topic;
  /** The value's time on the server's clock, µs. */
  readonly serverTimeUs: number;
  readonly value: MsgValue;
}

export type Nt4Status = "connecting" | "connected" | "closed";

export interface Nt4Options {
  readonly host: string;
  readonly port?: number;
  readonly clientName?: string;
  /** Topic name prefixes to subscribe to. */
  readonly prefixes: readonly string[];
  /** How often the server should send updates, seconds. */
  readonly periodicSeconds?: number;
  readonly onValue?: (v: Nt4Value) => void;
  readonly onStatus?: (status: Nt4Status, detail: string) => void;
  readonly onTimeSync?: (offsetUs: number, rttUs: number) => void;
  /** Local clock, µs (tests can replace it). */
  readonly nowUs?: () => number;
}

interface SubscribeMessage {
  readonly method: "subscribe";
  readonly params: {
    readonly topics: readonly string[];
    readonly subuid: number;
    readonly options: { readonly prefix: boolean; readonly periodic: number; readonly all: boolean };
  };
}

type ServerMessage =
  | { method: "announce"; params: { name: string; id: number; type: string; properties?: Record<string, unknown> } }
  | { method: "unannounce"; params: { name: string; id: number } }
  | { method: "properties"; params: { name: string; update: Record<string, unknown> } };

export class Nt4Client {
  private socket: WebSocket | null = null;
  private readonly topicsById = new Map<number, Nt4Topic>();
  private readonly latestByName = new Map<string, Nt4Value>();
  private offsetUs: number | null = null;
  private bestRttUs = Infinity;
  private lastRttUs = Number.NaN;
  private pingTimer: ReturnType<typeof setInterval> | null = null;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private closed = true;
  private readonly now: () => number;

  constructor(private readonly options: Nt4Options) {
    this.now = options.nowUs ?? (() => Math.round((performance.timeOrigin + performance.now()) * 1000));
  }

  get url(): string {
    return `ws://${this.options.host}:${this.options.port ?? NT4_PORT}/nt/${encodeURIComponent(this.options.clientName ?? "pathEditor5712")}`;
  }

  /** Server time minus local time, µs; null until the first time sync answer. */
  get serverOffsetUs(): number | null {
    return this.offsetUs;
  }

  /** The subprotocol the server chose ("" until connected). */
  get protocol(): string {
    return this.socket?.protocol ?? "";
  }

  /** The last time sync round trip, µs. */
  get roundTripUs(): number {
    return this.lastRttUs;
  }

  /** Now, on the server's clock (µs), or null before the first time sync. */
  serverNowUs(): number | null {
    return this.offsetUs === null ? null : this.now() + this.offsetUs;
  }

  topic(name: string): Nt4Topic | undefined {
    return [...this.topicsById.values()].find((t) => t.name === name);
  }

  latest(name: string): Nt4Value | undefined {
    return this.latestByName.get(name);
  }

  connect(): void {
    this.closed = false;
    this.open();
  }

  close(): void {
    this.closed = true;
    if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
    this.stopPings();
    this.socket?.close();
    this.socket = null;
    this.options.onStatus?.("closed", "");
  }

  private open(): void {
    this.topicsById.clear();
    this.latestByName.clear();
    this.offsetUs = null;
    this.bestRttUs = Infinity;
    this.options.onStatus?.("connecting", this.url);
    let socket: WebSocket;
    try {
      socket = new WebSocket(this.url, PROTOCOLS);
    } catch (e) {
      this.options.onStatus?.("closed", (e as Error).message);
      this.scheduleReconnect();
      return;
    }
    socket.binaryType = "arraybuffer";
    socket.onopen = () => {
      this.options.onStatus?.("connected", `${this.url} (${socket.protocol || "v4.0"})`);
      this.subscribe(socket);
      this.ping();
      this.pingTimer = setInterval(() => this.ping(), PING_INTERVAL_MS);
    };
    socket.onmessage = (event) => {
      if (typeof event.data === "string") this.onText(event.data);
      else this.onBinary(new Uint8Array(event.data as ArrayBuffer));
    };
    socket.onclose = (event) => {
      this.stopPings();
      if (this.socket !== socket) return;
      this.socket = null;
      if (!this.closed) {
        this.options.onStatus?.("connecting", `connection closed${event.reason ? `: ${event.reason}` : ""}; retrying`);
        this.scheduleReconnect();
      }
    };
    this.socket = socket;
  }

  private scheduleReconnect(): void {
    if (this.closed) return;
    this.reconnectTimer = setTimeout(() => this.open(), RECONNECT_MS);
  }

  private stopPings(): void {
    if (this.pingTimer) clearInterval(this.pingTimer);
    this.pingTimer = null;
  }

  private subscribe(socket: WebSocket): void {
    const message: SubscribeMessage = {
      method: "subscribe",
      params: {
        topics: [...this.options.prefixes],
        subuid: 1,
        options: { prefix: true, periodic: this.options.periodicSeconds ?? 0.02, all: false },
      },
    };
    socket.send(JSON.stringify([message]));
  }

  /** A time sync ping: [-1, 0, int, local time µs]. */
  private ping(): void {
    if (this.socket?.readyState === WebSocket.OPEN) this.socket.send(encode([-1, 0, INT_TYPE, this.now()]));
  }

  /** Announcements and property changes (JSON text frames). */
  onText(text: string): void {
    let messages: ServerMessage[];
    try {
      messages = JSON.parse(text) as ServerMessage[];
    } catch {
      return;
    }
    for (const m of messages) {
      if (m.method === "announce") {
        this.topicsById.set(m.params.id, { id: m.params.id, name: m.params.name, type: m.params.type, properties: m.params.properties ?? {} });
      } else if (m.method === "unannounce") {
        this.topicsById.delete(m.params.id);
        this.latestByName.delete(m.params.name);
      } else if (m.method === "properties") {
        for (const [id, t] of this.topicsById) {
          if (t.name === m.params.name) this.topicsById.set(id, { ...t, properties: { ...t.properties, ...m.params.update } });
        }
      }
    }
  }

  /** Values and time sync answers (MessagePack binary frames). */
  onBinary(bytes: Uint8Array): void {
    let frames: MsgValue[];
    try {
      frames = decodeAll(bytes);
    } catch {
      return;
    }
    for (const f of frames) {
      if (!Array.isArray(f) || f.length < 4) continue;
      const id = Number(f[0]);
      const serverTimeUs = Number(f[1]);
      if (id === -1) {
        this.onTimeSync(serverTimeUs, Number(f[3]));
        continue;
      }
      const topic = this.topicsById.get(id);
      if (!topic) continue;
      const v: Nt4Value = { topic, serverTimeUs, value: f[3]! };
      this.latestByName.set(topic.name, v);
      this.options.onValue?.(v);
    }
  }

  private onTimeSync(serverTimeUs: number, sentUs: number): void {
    const rtt = this.now() - sentUs;
    if (!(rtt >= 0)) return;
    this.lastRttUs = rtt;
    // the shortest round trip gives the tightest estimate (as ntcore's client keeps)
    if (rtt <= this.bestRttUs) {
      this.bestRttUs = rtt;
      this.offsetUs = serverTimeUs + rtt / 2 - this.now();
    }
    this.options.onTimeSync?.(this.offsetUs!, rtt);
  }
}

/** A Pose2d struct: Translation2d (x, y) then Rotation2d (radians), little-endian doubles. */
export function decodePose2d(bytes: Uint8Array): { x: number; y: number; heading: number } | null {
  if (bytes.length < 24) return null;
  const v = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  return { x: v.getFloat64(0, true), y: v.getFloat64(8, true), heading: v.getFloat64(16, true) };
}

/** A pose value: a struct:Pose2d, or a double[] [x, y, degrees] as Field2d publishes. */
export function poseOf(type: string, value: MsgValue): { x: number; y: number; heading: number } | null {
  if (type === "struct:Pose2d" && value instanceof Uint8Array) return decodePose2d(value);
  if (type === "double[]" && Array.isArray(value) && value.length >= 3) {
    const [x, y, deg] = value as number[];
    return { x: x!, y: y!, heading: (deg! * Math.PI) / 180 };
  }
  return null;
}
