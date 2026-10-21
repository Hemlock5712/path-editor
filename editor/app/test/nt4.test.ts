// The NT4 client's protocol handling without a server. The same client against WPILib's real ntcore
// server is test/nt4/ntcore.live.ts (npm run test:nt4).

import { afterEach, describe, expect, it, vi } from "vitest";
import { decodePose2d, Nt4Client, poseOf, robotAddress } from "../src/nt4/client.ts";
import { LiveFeed } from "../src/nt4/liveFeed.ts";
import { decodeAll, encode } from "../src/nt4/msgpack.ts";
import { live, LIVE_OFF } from "../src/state/live.ts";

/** A binary frame: [id, time, type, value] arrays back to back (hand-encoded MessagePack). */
function frame(...messages: number[][]): Uint8Array {
  return new Uint8Array(messages.flat());
}

const uint64 = (n: number) => {
  const d = new DataView(new ArrayBuffer(8));
  d.setBigUint64(0, BigInt(n));
  return [0xcf, ...new Uint8Array(d.buffer)];
};
const float64 = (x: number) => {
  const d = new DataView(new ArrayBuffer(8));
  d.setFloat64(0, x);
  return [0xcb, ...new Uint8Array(d.buffer)];
};
const str = (s: string) => [0xa0 | s.length, ...new TextEncoder().encode(s)];
const pose2d = (x: number, y: number, h: number) => {
  const d = new DataView(new ArrayBuffer(24));
  d.setFloat64(0, x, true);
  d.setFloat64(8, y, true);
  d.setFloat64(16, h, true);
  return [0xc4, 24, ...new Uint8Array(d.buffer)];
};

afterEach(() => live.set(LIVE_OFF));

describe("MessagePack", () => {
  it("decodes NT4 value frames and encodes time sync pings", () => {
    const values = decodeAll(frame([0x94, 0x01, ...uint64(5_000_000_000), 0x01, ...float64(3.25)], [0x94, 0x02, 0x00, 0x13, ...pose2d(1.5, -2, 0.5)]));
    expect(values).toHaveLength(2);
    expect(values[0]).toEqual([1, 5_000_000_000, 1, 3.25]);
    expect(decodePose2d((values[1] as unknown[])[3] as Uint8Array)).toEqual({ x: 1.5, y: -2, heading: 0.5 });
    // [-1, 0, int, a time in µs that needs 64 bits]
    const ping = encode([-1, 0, 2, 1_729_000_000_123_456]);
    expect(decodeAll(ping)).toEqual([[-1, 0, 2, 1_729_000_000_123_456]]);
    expect(poseOf("double[]", [1, 2, 90])!.heading).toBeCloseTo(Math.PI / 2, 12);
  });
});

describe("NT4 client", () => {
  it("maps announced topics to values with their server times", () => {
    const seen: string[] = [];
    const client = new Nt4Client({ host: "x", prefixes: ["/"], onValue: (v) => seen.push(`${v.topic.name}=${String(v.value)}@${v.serverTimeUs}`) });
    client.onText(JSON.stringify([{ method: "announce", params: { id: 7, name: "/Telemetry/Path/A/Progress", type: "double" } }]));
    client.onBinary(frame([0x94, 0x07, ...uint64(1234), 0x01, ...float64(1.5)], [0x94, 0x08, 0x00, 0x01, ...float64(9)]));
    expect(seen).toEqual(["/Telemetry/Path/A/Progress=1.5@1234"]); // id 8 was never announced
    expect(client.latest("/Telemetry/Path/A/Progress")?.value).toBe(1.5);
    client.onText(JSON.stringify([{ method: "unannounce", params: { id: 7, name: "/Telemetry/Path/A/Progress" } }]));
    expect(client.latest("/Telemetry/Path/A/Progress")).toBeUndefined();
  });

  it("syncs its clock with the server from the shortest round trip", () => {
    let local = 1_000_000;
    const client = new Nt4Client({ host: "x", prefixes: [], nowUs: () => local });
    // a ping sent at local 1,000,000 comes back 4 ms later; the server stamped it at 50,000,000
    local = 1_004_000;
    client.onBinary(frame([0x94, 0xff, ...uint64(50_000_000), 0x02, ...uint64(1_000_000)]));
    expect(client.roundTripUs).toBe(4000);
    expect(client.serverOffsetUs).toBe(50_000_000 + 2000 - 1_004_000);
    expect(client.serverNowUs()).toBe(50_002_000);
    // a slower answer later doesn't replace the better estimate
    local = 2_020_000;
    client.onBinary(frame([0x94, 0xff, ...uint64(51_010_000), 0x02, ...uint64(2_000_000)]));
    expect(client.roundTripUs).toBe(20_000);
    expect(client.serverOffsetUs).toBe(50_000_000 + 2000 - 1_004_000);
  });

  it("turns a team number into the robot's address", () => {
    expect(robotAddress("5712")).toBe("10.57.12.2");
    expect(robotAddress("254")).toBe("10.2.54.2");
    expect(robotAddress(" localhost ")).toBe("localhost");
    expect(robotAddress("10.57.12.2")).toBe("10.57.12.2");
  });
});

describe("live feed", () => {
  it("follows the active path's telemetry and the robot's pose", () => {
    vi.useFakeTimers();
    const feed = new LiveFeed({ address: "localhost", port: 5810, poseTopic: "/Telemetry/Robot/Pose" });
    const c = feed.client;
    c.onText(
      JSON.stringify([
        { method: "announce", params: { id: 1, name: "/Telemetry/PathFollower/Active", type: "string" } },
        { method: "announce", params: { id: 2, name: "/Telemetry/Path/SCurve/Progress", type: "double" } },
        { method: "announce", params: { id: 3, name: "/Telemetry/Path/Other/Progress", type: "double" } },
        { method: "announce", params: { id: 4, name: "/Telemetry/Robot/Pose", type: "struct:Pose2d" } },
        { method: "announce", params: { id: 5, name: "/Telemetry/PathFollower/EventsFired", type: "string[]" } },
        { method: "announce", params: { id: 6, name: "/Telemetry/Path/SCurve/Reference", type: "struct:Pose2d" } },
      ]),
    );
    c.onBinary(
      frame(
        [0x94, 0x01, 0x00, 0x04, ...str("SCurve")],
        [0x94, 0x02, 0x00, 0x01, ...float64(2.5)],
        [0x94, 0x03, 0x00, 0x01, ...float64(99)],
        [0x94, 0x04, 0x00, 0x13, ...pose2d(1, 2, 0.3)],
        [0x94, 0x05, 0x00, 0x14, 0x92, ...str("intake"), ...str("shoot")],
        [0x94, 0x06, 0x00, 0x13, ...pose2d(1.1, 2, 0.3)],
      ),
    );
    vi.advanceTimersByTime(60);
    const s = live.get();
    expect(s.active).toBe("SCurve");
    expect(s.progressMeters).toBe(2.5); // not the other path's 99
    expect(s.robot?.pose).toEqual({ x: 1, y: 2, heading: 0.3 });
    expect(s.robot?.reference).toEqual({ x: 1.1, y: 2, heading: 0.3 });
    expect(s.eventsFired).toEqual(["intake", "shoot"]);
    vi.useRealTimers();
  });
});
