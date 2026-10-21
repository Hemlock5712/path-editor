// The follower closed-loop on a simulated drivetrain: ports of lib's SimDrivetrain and FollowerSim
// (frc.team5712.path.sim), which write the golden traces in fixtures/follow. The editor's sim preview runs
// this in its worker.

import { wrap } from "./angles.ts";
import { flipPose, flipSpeeds, type FieldFrame, type FieldSpeeds, type Pose } from "./frame.ts";
import { PHASES, PathFollower, type FollowerConfig, type Phase } from "./follow.ts";
import type { Path } from "./path.ts";

/** The drivetrain's numbers: command latency, first-order velocity lag, integration step. */
export interface DriveSettings {
  readonly latencySeconds: number;
  readonly lagSeconds: number;
  readonly substepSeconds: number;
}

/** The end-to-end suite's drivetrain. */
export const DEFAULT_DRIVE: DriveSettings = { latencySeconds: 0.02, lagSeconds: 0.04, substepSeconds: 0.001 };

/** How late the drivetrain responds, all told: what the follower's latencySeconds should be. */
export function responseDelaySeconds(drive: DriveSettings): number {
  return drive.latencySeconds + drive.lagSeconds + 0.5 * 0.02;
}

/** A swerve drive reduced to its chassis: latency, then a first-order lag; no noise, no slip. */
export class SimDrivetrain {
  private readonly latency: number;
  private readonly lag: number;
  private readonly substep: number;
  private readonly kSubstep: number;
  private readonly inFlight: number[][] = [];
  private target: number[];
  private t = 0;
  private x: number;
  private y: number;
  private heading: number;
  private vx: number;
  private vy: number;
  private omega: number;

  constructor(pose: Pose, velocity: FieldSpeeds, drive: DriveSettings) {
    this.latency = drive.latencySeconds;
    this.lag = drive.lagSeconds;
    this.substep = drive.substepSeconds;
    this.kSubstep = 1.0 - Math.exp(-this.substep / this.lag);
    this.x = pose.xMeters;
    this.y = pose.yMeters;
    this.heading = pose.headingRadians;
    this.vx = velocity.vxMetersPerSecond;
    this.vy = velocity.vyMetersPerSecond;
    this.omega = velocity.omegaRadPerSec;
    this.target = [0.0, this.vx, this.vy, this.omega];
  }

  /** Sends a command now; it takes effect one latency later. */
  command(v: FieldSpeeds): void {
    this.inFlight.push([this.t + this.latency, v.vxMetersPerSecond, v.vyMetersPerSecond, v.omegaRadPerSec]);
  }

  /** A hit: adds (dvx, dvy) m/s to the chassis velocity now. */
  shove(dvx: number, dvy: number): void {
    this.vx += dvx;
    this.vy += dvy;
  }

  advanceTo(until: number): void {
    while (this.t < until - 1e-12) {
      while (this.inFlight.length > 0 && this.inFlight[0]![0]! <= this.t + 1e-12) {
        this.target = this.inFlight.shift()!;
      }
      const dt = Math.min(this.substep, until - this.t);
      const k = dt === this.substep ? this.kSubstep : 1.0 - Math.exp(-dt / this.lag);
      const vx0 = this.vx;
      const vy0 = this.vy;
      const w0 = this.omega;
      this.vx += (this.target[1]! - this.vx) * k;
      this.vy += (this.target[2]! - this.vy) * k;
      this.omega += (this.target[3]! - this.omega) * k;
      this.x += 0.5 * (vx0 + this.vx) * dt;
      this.y += 0.5 * (vy0 + this.vy) * dt;
      this.heading += 0.5 * (w0 + this.omega) * dt;
      this.t += dt;
    }
  }

  pose(): Pose {
    return { xMeters: this.x, yMeters: this.y, headingRadians: wrap(this.heading) };
  }

  velocity(): FieldSpeeds {
    return { vxMetersPerSecond: this.vx, vyMetersPerSecond: this.vy, omegaRadPerSec: this.omega };
  }
}

export const LOOP_SECONDS = 0.02;

/** A hit at atSeconds: add (dvx, dvy) m/s, field frame. */
export interface Shove {
  readonly atSeconds: number;
  readonly dvx: number;
  readonly dvy: number;
}

/** Every loop of a run, column by column; the command is the one sent that loop. */
export interface FollowTrace {
  t: number[];
  x: number[];
  y: number[];
  heading: number[];
  vx: number[];
  vy: number[];
  omega: number[];
  s: number[];
  crossTrack: number[];
  phase: Phase[];
  events: { name: string; loop: number }[];
  /** The loop the follower finished in (-1 if it never did). */
  finishLoop: number;
}

/**
 * Runs the path from its start pose (with its start speed along the start tangent) in `frame`, at 50 Hz,
 * until the follower finishes and `afterSeconds` more have passed, or `maxSeconds` (FollowerSim.run).
 */
export function runFollowerSim(
  path: Path,
  config: FollowerConfig,
  drive: DriveSettings,
  frame: FieldFrame,
  shove: Shove | null,
  afterSeconds: number,
  maxSeconds: number,
): FollowTrace {
  const start = path.sample(0.0);
  const v0 = path.spec.startMetersPerSecond;
  const startPose = flipPose(frame, { xMeters: start.xMeters, yMeters: start.yMeters, headingRadians: start.headingRadians });
  const startVel = flipSpeeds(frame, { vxMetersPerSecond: v0 * start.tangentX, vyMetersPerSecond: v0 * start.tangentY, omegaRadPerSec: 0.0 });
  const sim = new SimDrivetrain(startPose, startVel, drive);
  const follower = new PathFollower(path, config);
  const trace: FollowTrace = {
    t: [], x: [], y: [], heading: [], vx: [], vy: [], omega: [], s: [], crossTrack: [], phase: [], events: [], finishLoop: -1,
  };
  let loop = 0;
  follower.onEvent((name) => trace.events.push({ name, loop }));
  let shoved = false;
  let done = false;
  let sent: FieldSpeeds = { vxMetersPerSecond: 0.0, vyMetersPerSecond: 0.0, omegaRadPerSec: 0.0 };
  for (let k = 0; ; k++) {
    loop = k;
    const t = k * LOOP_SECONDS;
    sim.advanceTo(t);
    if (shove !== null && !shoved && t >= shove.atSeconds - 1e-12) {
      sim.shove(shove.dvx, shove.dvy);
      shoved = true;
    }
    const p = sim.pose();
    const v = sim.velocity();
    if (k === 0) follower.start(t, p, v, frame);
    if (!done) {
      sent = follower.step(t, p, v);
      sim.command(sent);
      if (follower.isFinished()) {
        done = true;
        trace.finishLoop = k;
        if (!follower.rollingEnd) {
          sent = { vxMetersPerSecond: 0.0, vyMetersPerSecond: 0.0, omegaRadPerSec: 0.0 };
          sim.command(sent);
        }
      }
    }
    trace.t.push(t);
    trace.x.push(p.xMeters);
    trace.y.push(p.yMeters);
    trace.heading.push(p.headingRadians);
    trace.vx.push(sent.vxMetersPerSecond);
    trace.vy.push(sent.vyMetersPerSecond);
    trace.omega.push(sent.omegaRadPerSec);
    trace.s.push(follower.progressMeters);
    trace.crossTrack.push(follower.crossTrackMeters);
    trace.phase.push(follower.phase);
    if ((done && t >= trace.finishLoop * LOOP_SECONDS + afterSeconds - 1e-9) || t >= maxSeconds - 1e-9) {
      return trace;
    }
  }
}


