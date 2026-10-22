// The editor's NT4 client against WPILib's real ntcore server: lib's Nt4TelemetryServer starts ntcore
// (2027 alpha-7 jars and desktop natives) and runs a real PathFollower in FollowPathCommand on the
// simulated drivetrain, in real time, publishing its telemetry, the robot's pose and the server's clock.
// Run with: npm run test:nt4 (it starts the server through gradle).

import { spawn, spawnSync, type ChildProcess } from "node:child_process";
import { join } from "node:path";
import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { Nt4Client, poseOf, type Nt4Value } from "../../src/nt4/client.ts";
import { LiveFeed } from "../../src/nt4/liveFeed.ts";
import { Recorder, summarise } from "../../src/nt4/recorder.ts";
import { live } from "../../src/state/live.ts";

const REPO = join(import.meta.dirname, "..", "..", "..", "..");
const PORT = 5897;
const SERVER_SECONDS = 120;
const windows = process.platform === "win32";

let server: ChildProcess;
let output = "";

function startServer(): Promise<void> {
  const gradlew = join(REPO, "lib", windows ? "gradlew.bat" : "gradlew");
  server = spawn(gradlew, ["-p", "lib", "nt4Server", "--no-daemon", "-q", `-Pnt4Port=${PORT}`, `-Pnt4Seconds=${SERVER_SECONDS}`], {
    cwd: REPO,
    shell: windows,
  });
  return new Promise((resolve, reject) => {
    const onData = (chunk: Buffer) => {
      output += chunk.toString();
      if (output.includes("NT4 READY")) resolve();
    };
    server.stdout!.on("data", onData);
    server.stderr!.on("data", onData);
    server.on("exit", (code) => reject(new Error(`the NT4 server exited (${String(code)}) before it was ready:\n${output}`)));
  });
}

function stopServer(): void {
  if (!server?.pid) return;
  if (windows) spawnSync("taskkill", ["/pid", String(server.pid), "/T", "/F"]);
  else server.kill("SIGTERM");
}

const until = async (what: string, ok: () => boolean, ms = 20_000) => {
  const end = Date.now() + ms;
  while (!ok()) {
    if (Date.now() > end) throw new Error(`timed out waiting for ${what}`);
    await new Promise((r) => setTimeout(r, 50));
  }
};

describe("NT4 client against WPILib's ntcore", () => {
  beforeAll(startServer);
  afterAll(stopServer);

  it("receives the follower's telemetry, with time sync, from a real server", async () => {
    // each value with the server's time when it arrived, as the client reckons it, and the round trip of the
    // time sync answer that reckoning came from (the first answer can be slow: the server is busy sending
    // every topic to a new client; a better one replaces it a few seconds later)
    const received: { v: Nt4Value; serverNowAtArrival: number | null; syncRoundTripUs: number }[] = [];
    const client: Nt4Client = new Nt4Client({
      host: "127.0.0.1",
      port: PORT,
      prefixes: ["/Telemetry/"],
      onValue: (v) => received.push({ v, serverNowAtArrival: client.serverNowUs(), syncRoundTripUs: client.syncRoundTripUs }),
    });
    client.connect();
    try {
      await until("the connection and time sync", () => client.serverOffsetUs !== null);
      expect(client.protocol).toBe("v4.1.networktables.first.wpi.edu");
      // localhost: within a few pings (every 3 s) one comes back fast; the first can wait behind every
      // topic's announcement on a busy PC
      await until("a localhost round trip", () => client.syncRoundTripUs < 50_000, 15_000);

      // the follower runs the S-curve over and over: watch a whole run's events go by
      const fired = () => received.filter((r) => r.v.topic.name === "/Telemetry/PathFollower/EventsFired").map((r) => r.v.value as string[]);
      await until("every event of a run", () => fired().some((e) => e.length === 3), 30_000);
      expect(fired().find((e) => e.length === 3)).toEqual(["intakeDown", "spinUp", "shoot"]);
      expect(client.latest("/Telemetry/PathFollower/Active")?.value).toMatch(/^(SCurve|)$/);

      // progress along the path rises through a run, inside the path's length
      const progress = received.filter((r) => r.v.topic.name === "/Telemetry/Path/SCurve/Progress").map((r) => r.v.value as number);
      let rises = 0;
      for (let i = 1; i < progress.length; i++) if (progress[i]! > progress[i - 1]!) rises++;
      expect(rises).toBeGreaterThan(30);
      const length =
        (client.latest("/Telemetry/Path/SCurve/Progress")!.value as number) + (client.latest("/Telemetry/Path/SCurve/Remaining")!.value as number);
      expect(length).toBeGreaterThan(8); // the S-curve fixture is about 9.3 m
      expect(Math.max(...progress)).toBeLessThanOrEqual(length + 1e-6);
      expect(client.topic("/Telemetry/Path/SCurve/CrossTrack")?.type).toBe("double");

      // the robot's pose and the follower's reference, as structs, close together on the field
      const pose = poseOf("struct:Pose2d", client.latest("/Telemetry/Robot/Pose")!.value)!;
      const reference = poseOf(client.topic("/Telemetry/Path/SCurve/Reference")!.type, client.latest("/Telemetry/Path/SCurve/Reference")!.value)!;
      expect(Math.abs(pose.x)).toBeLessThan(8.3);
      expect(Math.hypot(pose.x - reference.x, pose.y - reference.y)).toBeLessThan(0.5);

      // time sync: the server publishes its own clock (ntcore's, in ns) every loop and flushes it at once.
      // When a value arrives, the client's estimate of the server's time is that clock plus the transport
      // time, less the estimate's error. The error is at most half the round trip of the answer the estimate
      // came from (the server stamped it between the ping leaving and the answer arriving), so the transport
      // time, which can't be negative, leaves each lag no lower than minus that half round trip. 0.05 ms
      // covers the two clocks' readings, each rounded to the µs.
      const clock = received.filter((r) => r.v.topic.name === "/Telemetry/Harness/ServerTimeNanos" && r.serverNowAtArrival !== null);
      expect(clock.length).toBeGreaterThan(50);
      const early = clock
        .map((r) => ({ lag: (r.serverNowAtArrival! - (r.v.value as number) / 1000) / 1000, bound: -r.syncRoundTripUs / 2000 - 0.05 }))
        .filter((x) => x.lag < x.bound);
      expect(early, "lags (ms) earlier than the sync's accuracy allows").toEqual([]);
      const lags = clock.map((r) => (r.serverNowAtArrival! - (r.v.value as number) / 1000) / 1000);
      // the NT timestamp is taken when the value is logged, just after the harness reads the clock into it
      const stamps = clock.map((r) => (r.v.serverTimeUs - (r.v.value as number) / 1000) / 1000).sort((a, b) => a - b);
      lags.sort((a, b) => a - b);
      const median = lags[Math.floor(lags.length / 2)]!;
      console.log(
        `NT4: round trip ${(client.roundTripUs / 1000).toFixed(2)} ms (sync's ${(client.syncRoundTripUs / 1000).toFixed(2)} ms); arrival lag median ${median.toFixed(1)} ms, ` +
          `range ${lags[0]!.toFixed(1)} to ${lags.at(-1)!.toFixed(1)} ms over ${lags.length} values; stamp gap median ${stamps[Math.floor(stamps.length / 2)]!.toFixed(2)}, worst ${stamps.at(-1)!.toFixed(2)} ms`,
      );
      // the value's NT timestamp is on the clock it carries: never before it (µs rounding aside), and as a
      // rule straight after it (the worst is however long the harness's thread was held up, on a busy PC)
      expect(stamps[0]!).toBeGreaterThan(-0.002);
      expect(stamps[Math.floor(stamps.length / 2)]!).toBeLessThan(1);
      const p95 = lags[Math.floor(lags.length * 0.95)]!;
      console.log(`NT4: 95th percentile lag ${p95.toFixed(1)} ms`);
      expect(median).toBeLessThan(15);
      expect(p95).toBeLessThan(40); // the rest is the harness's own scheduling (JIT, GC) on a busy PC
    } finally {
      client.close();
    }
  });

  it("drives the editor's live view", async () => {
    const feed = new LiveFeed({ address: "127.0.0.1", port: PORT, poseTopic: "/Telemetry/Robot/Pose" });
    feed.start();
    try {
      await until("the live view to fill in", () => {
        const s = live.get();
        return s.status === "connected" && s.synced && s.active === "SCurve" && Number.isFinite(s.progressMeters) && s.robot?.pose !== undefined;
      });
      await until("events in the live view", () => live.get().eventsFired.length > 0, 30_000);
      const s = live.get();
      expect(s.rttMs).toBeLessThan(50);
      expect(s.poseAgeMs).toBeLessThan(100);
      expect(s.robot?.reference).toBeDefined();
      expect(["intakeDown", "spinUp", "shoot"]).toEqual(expect.arrayContaining([...s.eventsFired]));
    } finally {
      feed.stop();
    }
  });

  it("records a whole run as the follower logged it (the Telemetry page's recording)", async () => {
    // the harness publishes the follower's own account of each run when it ends: [run, loops, cross-track
    // RMS, cross-track max, final position error, final heading error, seconds]. Runs differ in length (the
    // harness runs in real time, so a slow loop changes the run), so each recorded run is matched to its own
    // account by order, not by its numbers: the first account to arrive after the run started is its own
    // (the next run starts half a second after the last one's account).
    const summaries: { at: number; values: number[] }[] = [];
    const startedAt: number[] = []; // the frame each recorded run started in
    let frames = 0;
    const recorder = new Recorder("/Telemetry/Robot/Pose", 10);
    const client = new Nt4Client({
      host: "127.0.0.1",
      port: PORT,
      prefixes: ["/Telemetry/"],
      all: true,
      onValue: (v) => {
        recorder.onValue(v);
        if (v.topic.name === "/Telemetry/Harness/RunSummary") summaries.push({ at: frames, values: v.value as number[] });
      },
      onFrameEnd: () => {
        recorder.onFrameEnd();
        while (startedAt.length < recorder.runs.length) startedAt.push(frames);
        frames++;
      },
    });
    client.connect();
    try {
      // a run that started after we connected, recorded from its first loop to its finish
      await until("a whole recorded run", () => recorder.runs.some((r) => r.finished && r.samples[0] !== undefined && r.samples[0].t < 0.05), 60_000);
      const run = recorder.runs.find((r) => r.finished && r.samples[0]!.t < 0.05)!;
      const start = startedAt[recorder.runs.indexOf(run)]!;
      const account = () => summaries.find((x) => x.at >= start);
      await until("the harness's account of it", () => account() !== undefined, 10_000);
      const theirs = account()!;
      const [, loops, rms, max, endError, endHeading, seconds] = theirs!.values as [number, number, number, number, number, number, number];
      const ours = summarise(run);
      console.log(
        `recorded run: ${ours.loops} loops (follower ${loops}), cross-track RMS ${(ours.crossTrackRms * 100).toFixed(3)} cm ` +
          `(follower ${(rms * 100).toFixed(3)}), max ${(ours.crossTrackMax * 100).toFixed(3)} cm (${(max * 100).toFixed(3)}), ` +
          `end ${(ours.endError * 100).toFixed(2)} cm (${(endError * 100).toFixed(2)}), ${ours.seconds.toFixed(2)} s (${seconds.toFixed(2)})`,
      );
      expect(Math.abs(ours.loops - loops)).toBeLessThanOrEqual(1);
      expect(Math.abs(ours.crossTrackRms - rms)).toBeLessThan(0.001); // 1 mm
      expect(Math.abs(ours.crossTrackMax - max)).toBeLessThan(0.001);
      expect(Math.abs(ours.endError - endError)).toBeLessThan(0.01); // the pose a loop either side, at the finish
      expect(Math.abs(ours.endHeadingError - endHeading)).toBeLessThan(0.01);
      expect(Math.abs(ours.seconds - seconds)).toBeLessThan(0.1);
      // the plan came with it, and the events fired as planned
      expect(run.plan?.events.map((e) => e.name)).toEqual(["intakeDown", "spinUp", "shoot"]);
      expect(ours.eventsFired).toEqual(["intakeDown", "spinUp", "shoot"]);
      expect(run.plan!.x.length).toBeGreaterThan(100);
      expect(ours.planSeconds).toBeGreaterThan(1);
      // and the samples carry the new loop values
      const mid = run.samples[Math.floor(run.samples.length / 2)]!;
      for (const k of ["alongTrack", "headingError", "speed", "plannedSpeed", "omega", "plannedOmega"] as const) expect(Number.isFinite(mid[k]), k).toBe(true);
      expect(mid.reference).not.toBeNull();
      expect(mid.pose).not.toBeNull();
    } finally {
      client.close();
    }
  });
});
