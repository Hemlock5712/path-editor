// The path follower: a line-for-line port of lib's PathFollower, OutputLimiter and StoppingLaw
// (frc.team5712.path.follow), for the editor's sim preview. Keep the arithmetic in Java's order: the
// closed-loop golden traces (fixtures/follow) must come out to 1e-9.
//
// Left out, and Java-only: adaptive grip and slip sensing (the preview assumes the floor grips), and the
// moving goal's tail rebuild (it needs a spline with a fixed start curvature). With no slip signal,
// Java's adaptive grip leaves every number unchanged, so the port matches it as is.

import { wrap } from "./angles.ts";
import { flipPose, flipSpeeds, type FieldFrame, type FieldSpeeds, type Pose } from "./frame.ts";
import { javaHypot } from "./jmath.ts";
import { PURE_ROTATION_METERS, eventMeters, type Path, type PathSample } from "./path.ts";
import {
  GRAVITY,
  driveAccelMetersPerSecondSquared,
  spinLeverMeters,
  type RobotConfig,
} from "./robot.ts";

const DEG = Math.PI / 180.0;

// ------------------------------------------------------------------------------ settings

/** Gains and limits (FollowerConfig in Java; the same names and defaults, less the slip settings). */
export interface FollowerConfig {
  readonly robot: RobotConfig;
  readonly latencySeconds: number;
  readonly windowMeters: number;
  readonly reprojectMeters: number;
  readonly crossTrackKp: number;
  readonly crossTrackKd: number;
  readonly headingKp: number;
  readonly terminalRadiusMeters: number;
  readonly terminalTurnRadians: number;
  readonly brakeMetersPerSecondSquared: number;
  readonly easeInPerSecond: number;
  readonly stopDeadbandMeters: number;
  readonly maxOmegaRadPerSec: number;
  readonly maxAlphaRadPerSecSquared: number;
  readonly headingEaseInPerSecond: number;
  readonly headingDeadbandRadians: number;
  readonly limiterFrictionCoefficient: number;
  readonly positionToleranceMeters: number;
  readonly headingToleranceRadians: number;
  readonly settledMetersPerSecond: number;
  readonly debounceSeconds: number;
}

/** The defaults for a robot, with any fields overridden by name (as FollowerConfig.Builder.set). */
export function followerConfig(robot: RobotConfig, options: Partial<Omit<FollowerConfig, "robot">> = {}): FollowerConfig {
  return {
    robot,
    latencySeconds: 0.05,
    windowMeters: 0.5,
    reprojectMeters: 0.5,
    crossTrackKp: 5.0,
    crossTrackKd: 0.0,
    headingKp: 5.0,
    terminalRadiusMeters: 0.3,
    terminalTurnRadians: 2.0 * DEG,
    brakeMetersPerSecondSquared: 6.0,
    easeInPerSecond: 12.0,
    stopDeadbandMeters: 0.003,
    maxOmegaRadPerSec: 540.0 * DEG,
    maxAlphaRadPerSecSquared: 720.0 * DEG,
    headingEaseInPerSecond: 5.0,
    headingDeadbandRadians: 0.3 * DEG,
    limiterFrictionCoefficient: 1.0,
    positionToleranceMeters: 0.02,
    headingToleranceRadians: 1.0 * DEG,
    settledMetersPerSecond: 0.05,
    debounceSeconds: 0.1,
    ...options,
  };
}

// ------------------------------------------------------------------------------ the laws

function copySign(magnitude: number, sign: number): number {
  return sign < 0 || Object.is(sign, -0) ? -Math.abs(magnitude) : Math.abs(magnitude);
}

/** DriveToPose's stopping law (StoppingLaw.java). */
export const StoppingLaw = {
  speed(distance: number, speedNow: number, decel: number, reactionSeconds: number, maxSpeed: number,
    easeIn: number, deadband: number): number {
    if (distance < deadband) return 0.0;
    const braking = Math.max(0.0, distance - speedNow * reactionSeconds);
    return Math.min(maxSpeed, Math.min(Math.sqrt(2.0 * decel * braking), easeIn * distance));
  },
  brake(distance: number, speedNow: number, decel: number, reactionSeconds: number, endSpeed: number): number {
    const braking = Math.max(0.0, distance - speedNow * reactionSeconds);
    return Math.sqrt(endSpeed * endSpeed + 2.0 * decel * braking);
  },
  spin(error: number, spinNow: number, alpha: number, reactionSeconds: number, maxOmega: number,
    easeIn: number, deadband: number): number {
    const angle = Math.abs(error);
    if (angle < deadband) return 0.0;
    const braking = Math.max(0.0, angle - Math.abs(spinNow) * reactionSeconds);
    const rate = Math.min(maxOmega, Math.min(Math.sqrt(2.0 * alpha * braking), easeIn * angle));
    return copySign(rate, error);
  },
};

/** The one limit every output goes through (OutputLimiter.java), with the full grip budget. */
export class OutputLimiter {
  private readonly robot: RobotConfig;
  private readonly grip: number;
  private readonly lever: number;
  private readonly maxAlpha: number;
  vx = 0;
  vy = 0;
  omega = 0;

  constructor(robot: RobotConfig, frictionCoefficient: number, maxAlphaRadPerSecSquared: number) {
    this.robot = robot;
    this.grip = frictionCoefficient * GRAVITY;
    this.lever = spinLeverMeters(robot);
    this.maxAlpha = maxAlphaRadPerSecSquared;
  }

  reset(vx: number, vy: number, omega: number): void {
    this.vx = vx;
    this.vy = vy;
    this.omega = omega;
  }

  limit(wantVx: number, wantVy: number, wantOmega: number, dt: number): void {
    let ax = (wantVx - this.vx) / dt;
    let ay = (wantVy - this.vy) / dt;
    let alpha = (wantOmega - this.omega) / dt;
    alpha = Math.max(-this.maxAlpha, Math.min(this.maxAlpha, alpha));
    const speedingUp = ax * this.vx + ay * this.vy > 0.0;
    if (speedingUp) {
      const wheel = Math.sqrt(this.vx * this.vx + this.vy * this.vy) + Math.abs(this.omega) * this.robot.moduleRadiusMeters;
      const motor = driveAccelMetersPerSecondSquared(this.robot, wheel);
      const a = Math.sqrt(ax * ax + ay * ay);
      if (a > motor) {
        ax *= motor / a;
        ay *= motor / a;
      }
    }
    const total = Math.sqrt(ax * ax + ay * ay) + this.lever * Math.abs(alpha);
    const budget = this.grip; // Java: grip or brakeGrip, both the full budget without slip
    if (total > budget) {
      const k = budget / total;
      ax *= k;
      ay *= k;
      alpha *= k;
    }
    this.vx += ax * dt;
    this.vy += ay * dt;
    this.omega += alpha * dt;
  }

  force(vx: number, vy: number, omega: number): void {
    this.vx = vx;
    this.vy = vy;
    this.omega = omega;
  }

  speed(): number {
    return Math.sqrt(this.vx * this.vx + this.vy * this.vy);
  }
}

// ------------------------------------------------------------------------------ the follower

export type Phase = "IDLE" | "TRACKING" | "TERMINAL" | "FINISHED" | "CANCELLED";

/** The phases in Java's enum order (the golden traces store ordinals). */
export const PHASES: readonly Phase[] = ["IDLE", "TRACKING", "TERMINAL", "FINISHED", "CANCELLED"];

const NOMINAL_LOOP_SECONDS = 0.02;
const MAX_PROGRESS_RATE = 8.0;

/** One controller from path to pose (PathFollower.java; see there for the design). */
export class PathFollower {
  readonly path: Path;
  private readonly cfg: FollowerConfig;
  private readonly limiter: OutputLimiter;
  readonly rollingEnd: boolean;
  private readonly endSpeed: number;
  private readonly baseGoal: Pose; // blue
  private readonly pureRotation: boolean;

  private readonly eventNames: string[] = [];
  private eventBaseS: number[] = [];
  private eventS: number[] = [];
  private eventFired: boolean[] = [];
  private readonly listeners: ((name: string) => void)[] = [];

  /** Metres left of the path to shift the reference by. */
  lateralOffset: () => number = () => 0.0;
  /** Along-track speed scale, 0..1. */
  alongTrackScale: () => number = () => 1.0;

  private frame: FieldFrame = "BLUE";
  private goal: Pose;
  private phaseNow: Phase = "IDLE";
  private s = 0;
  private lastS = 0;
  private lastTime = 0;
  private settledSince = Number.NaN;
  private crossTrack = 0;
  private refX = 0;
  private refY = 0;
  private refHeading = 0;

  constructor(path: Path, config: FollowerConfig) {
    this.path = path;
    this.cfg = config;
    this.limiter = new OutputLimiter(config.robot, config.limiterFrictionCoefficient, config.maxAlphaRadPerSecSquared);
    this.endSpeed = path.spec.endMetersPerSecond;
    this.rollingEnd = this.endSpeed > 0.0;
    const end = path.sample(path.lengthMeters);
    this.baseGoal = { xMeters: end.xMeters, yMeters: end.yMeters, headingRadians: end.headingRadians };
    this.goal = this.baseGoal;
    this.pureRotation = path.lengthMeters <= 1.5 * PURE_ROTATION_METERS;
    for (const e of path.spec.events) this.addEvent(e.name, eventMeters(e, path.geometry));
  }

  /** An event at sMeters along the path (the file's own events are already added). */
  addEvent(name: string, sMeters: number): this {
    this.eventNames.push(name);
    const n = this.eventNames.length;
    const base = this.eventBaseS.slice();
    base.push(Math.max(0.0, Math.min(this.path.lengthMeters, sMeters)));
    for (let i = n - 1; i > 0 && base[i - 1]! > base[i]!; i--) {
      const t = base[i - 1]!;
      base[i - 1] = base[i]!;
      base[i] = t;
      const moved = this.eventNames.splice(i, 1)[0]!;
      this.eventNames.splice(i - 1, 0, moved);
    }
    this.eventBaseS = base;
    this.eventS = base.slice();
    this.eventFired = new Array<boolean>(n).fill(false);
    return this;
  }

  /** Adds a listener called with each event's name as it fires. */
  onEvent(listener: (name: string) => void): this {
    this.listeners.push(listener);
    return this;
  }

  start(timeSeconds: number, measuredPose: Pose, measuredVelocity: FieldSpeeds, frame: FieldFrame,
    handedCommand: FieldSpeeds = measuredVelocity): void {
    this.frame = frame;
    this.goal = this.baseGoal;
    const cmd = flipSpeeds(frame, handedCommand);
    this.limiter.reset(cmd.vxMetersPerSecond, cmd.vyMetersPerSecond, cmd.omegaRadPerSec);
    const p = flipPose(frame, measuredPose);
    this.s = 0.0;
    this.updateProgress(p);
    this.lastS = this.s;
    this.lastTime = timeSeconds;
    this.settledSince = Number.NaN;
    this.phaseNow = "TRACKING";
    this.eventS = this.eventBaseS.slice();
    this.eventFired.fill(false);
  }

  step(timeSeconds: number, measuredPose: Pose, measuredVelocity: FieldSpeeds): FieldSpeeds {
    if (this.phaseNow === "IDLE") throw new Error("step() before start()");
    if (this.phaseNow === "FINISHED" || this.phaseNow === "CANCELLED") {
      return this.rollingEnd && this.phaseNow === "FINISHED"
        ? this.output()
        : { vxMetersPerSecond: 0.0, vyMetersPerSecond: 0.0, omegaRadPerSec: 0.0 };
    }
    const cfg = this.cfg;
    const active = this.path;
    const dt = timeSeconds > this.lastTime
      ? Math.max(0.001, Math.min(0.1, timeSeconds - this.lastTime))
      : NOMINAL_LOOP_SECONDS;
    this.lastTime = timeSeconds;
    const pose = flipPose(this.frame, measuredPose);
    const vel = flipSpeeds(this.frame, measuredVelocity);
    if (!(Number.isFinite(pose.xMeters) && Number.isFinite(pose.yMeters) && Number.isFinite(pose.headingRadians))) {
      this.limiter.limit(0.0, 0.0, 0.0, dt);
      return this.output();
    }

    const brakeDecel = Math.min(cfg.brakeMetersPerSecondSquared, Infinity * GRAVITY); // no slip: Java's slipMu is +∞
    const gripScale = 1.0;
    this.updateProgress(pose);
    const remaining = active.lengthMeters - this.s;
    const goal = this.goal;
    if (this.phaseNow === "TRACKING" && !this.rollingEnd && remaining < cfg.terminalRadiusMeters
      && Math.abs(wrap(goal.headingRadians - active.heading.sample(this.s).radians)) <= cfg.terminalTurnRadians) {
      this.phaseNow = "TERMINAL";
    }

    let wantVx: number;
    let wantVy: number;
    let wantOmega: number;
    const offset = this.lateralOffset();
    const scale = Math.max(0.0, Math.min(1.0, this.alongTrackScale()));
    if (this.phaseNow === "TERMINAL") {
      const end = active.sample(active.lengthMeters);
      const gx = goal.xMeters - end.tangentY * offset;
      const gy = goal.yMeters + end.tangentX * offset;
      const ex = gx - pose.xMeters;
      const ey = gy - pose.yMeters;
      const d = Math.sqrt(ex * ex + ey * ey);
      const speed = scale * StoppingLaw.speed(d, this.limiter.speed(), brakeDecel, cfg.latencySeconds,
        active.spec.maxMetersPerSecond, cfg.easeInPerSecond, cfg.stopDeadbandMeters);
      wantVx = d > 1e-6 ? (ex / d) * speed : 0.0;
      wantVy = d > 1e-6 ? (ey / d) * speed : 0.0;
      const headingError = wrap(goal.headingRadians - pose.headingRadians);
      wantOmega = StoppingLaw.spin(headingError, this.limiter.omega, cfg.maxAlphaRadPerSecSquared,
        cfg.latencySeconds, cfg.maxOmegaRadPerSec, cfg.headingEaseInPerSecond, cfg.headingDeadbandRadians);
      this.crossTrack = 0.0;
      this.refX = gx;
      this.refY = gy;
      this.refHeading = goal.headingRadians;
    } else if (this.pureRotation) {
      const profile = active.profile;
      const ahead = active.sample(profile.sAtTime(profile.timeAt(this.s) + cfg.latencySeconds));
      const headingError = wrap(active.heading.sample(this.s).radians - pose.headingRadians);
      wantOmega = scale * gripScale * ahead.metersPerSecond * ahead.headingRadiansPerMeter + cfg.headingKp * headingError;
      wantOmega = Math.max(-cfg.maxOmegaRadPerSec, Math.min(cfg.maxOmegaRadPerSec, wantOmega));
      wantVx = cfg.crossTrackKp * (goal.xMeters - pose.xMeters);
      wantVy = cfg.crossTrackKp * (goal.yMeters - pose.yMeters);
      this.crossTrack = javaHypot(goal.xMeters - pose.xMeters, goal.yMeters - pose.yMeters);
      this.refX = goal.xMeters;
      this.refY = goal.yMeters;
      this.refHeading = active.heading.sample(this.s).radians;
    } else {
      const profile = active.profile;
      const sAhead = profile.sAtTime(profile.timeAt(this.s) + cfg.latencySeconds);
      const here: PathSample = active.sample(this.s);
      const ahead: PathSample = active.sample(sAhead);
      const brake = StoppingLaw.brake(remaining, this.limiter.speed(), brakeDecel, cfg.latencySeconds, this.endSpeed);
      const speed = scale * Math.min(gripScale * ahead.metersPerSecond, brake);
      const nx = -here.tangentY;
      const ny = here.tangentX;
      this.crossTrack = (pose.xMeters - here.xMeters) * nx + (pose.yMeters - here.yMeters) * ny;
      const drift = vel.vxMetersPerSecond * nx + vel.vyMetersPerSecond * ny;
      const sideways = -cfg.crossTrackKp * (this.crossTrack - offset) - cfg.crossTrackKd * drift;
      wantVx = speed * ahead.tangentX + sideways * nx;
      wantVy = speed * ahead.tangentY + sideways * ny;
      const headingError = wrap(here.headingRadians - pose.headingRadians);
      wantOmega = speed * ahead.headingRadiansPerMeter + cfg.headingKp * headingError;
      wantOmega = Math.max(-cfg.maxOmegaRadPerSec, Math.min(cfg.maxOmegaRadPerSec, wantOmega));
      this.refX = here.xMeters + nx * offset;
      this.refY = here.yMeters + ny * offset;
      this.refHeading = here.headingRadians;
    }

    this.limiter.limit(wantVx, wantVy, wantOmega, dt);

    const rate = Math.max(0.0, Math.min(MAX_PROGRESS_RATE, (this.s - this.lastS) / dt));
    this.lastS = this.s;
    this.fireEventsUpTo(this.s + 0.5 * rate * dt);
    if (this.phaseNow === "TERMINAL" && this.arrived(pose)) this.fireEventsUpTo(active.lengthMeters);
    this.checkFinished(timeSeconds, pose, vel, remaining);
    return this.output();
  }

  private arrived(pose: Pose): boolean {
    return javaHypot(this.refX - pose.xMeters, this.refY - pose.yMeters) < this.cfg.positionToleranceMeters
      && Math.abs(wrap(this.goal.headingRadians - pose.headingRadians)) < this.cfg.headingToleranceRadians;
  }

  /** Interrupted: stops tracking; events that haven't fired never will. */
  cancel(): void {
    if (this.phaseNow === "TRACKING" || this.phaseNow === "TERMINAL") this.phaseNow = "CANCELLED";
  }

  isFinished(): boolean {
    return this.phaseNow === "FINISHED";
  }

  /** The last command, in the robot's frame. */
  endCommand(): FieldSpeeds {
    return this.output();
  }

  private output(): FieldSpeeds {
    return flipSpeeds(this.frame, {
      vxMetersPerSecond: this.limiter.vx,
      vyMetersPerSecond: this.limiter.vy,
      omegaRadPerSec: this.limiter.omega,
    });
  }

  private updateProgress(pose: Pose): void {
    if (this.pureRotation) {
      this.s = this.turnProgress(pose.headingRadians);
      return;
    }
    const geometry = this.path.geometry;
    let near = geometry.project(pose.xMeters, pose.yMeters, this.s, this.cfg.windowMeters);
    if (near.distanceMeters > this.cfg.reprojectMeters) {
      const all = geometry.projectGlobal(pose.xMeters, pose.yMeters);
      if (all.distanceMeters < near.distanceMeters - 0.05) near = all;
    }
    this.s = near.s;
  }

  private turnProgress(heading: number): number {
    const profile = this.path.heading;
    const length = this.path.lengthMeters;
    const steps = 64;
    const s = this.s;
    let prevS = s;
    let prevErr = wrap(profile.sample(s).radians - heading);
    let best = s;
    let bestErr = Math.abs(prevErr);
    for (let i = 1; i <= steps; i++) {
      const si = s + ((length - s) * i) / steps;
      const err = wrap(profile.sample(si).radians - heading);
      if (Math.sign(err) !== Math.sign(prevErr) && prevErr !== 0.0) {
        let lo = prevS;
        let hi = si;
        for (let k = 0; k < 30; k++) {
          const mid = 0.5 * (lo + hi);
          const e = wrap(profile.sample(mid).radians - heading);
          if (Math.sign(e) === Math.sign(prevErr)) {
            lo = mid;
          } else {
            hi = mid;
          }
        }
        return 0.5 * (lo + hi);
      }
      if (Math.abs(err) < bestErr) {
        best = si;
        bestErr = Math.abs(err);
      }
      prevS = si;
      prevErr = err;
    }
    return best;
  }

  private checkFinished(t: number, pose: Pose, vel: FieldSpeeds, remaining: number): void {
    if (this.rollingEnd) {
      const end = this.path.sample(this.path.lengthMeters);
      const past = (pose.xMeters - end.xMeters) * end.tangentX + (pose.yMeters - end.yMeters) * end.tangentY;
      if (remaining <= 1e-6 || past >= 0.0) this.finish();
      return;
    }
    if (this.phaseNow !== "TERMINAL") return;
    const cfg = this.cfg;
    const d = javaHypot(this.refX - pose.xMeters, this.refY - pose.yMeters);
    const yaw = Math.abs(wrap(this.goal.headingRadians - pose.headingRadians));
    const speed = javaHypot(vel.vxMetersPerSecond, vel.vyMetersPerSecond);
    const ok = d < cfg.positionToleranceMeters && yaw < cfg.headingToleranceRadians && speed < cfg.settledMetersPerSecond;
    if (!ok) {
      this.settledSince = Number.NaN;
    } else if (Number.isNaN(this.settledSince)) {
      this.settledSince = t;
    }
    if (ok && t - this.settledSince >= cfg.debounceSeconds - 1e-9) this.finish();
  }

  private finish(): void {
    this.fireEventsUpTo(Infinity);
    this.phaseNow = "FINISHED";
    if (!this.rollingEnd) this.limiter.force(0.0, 0.0, 0.0);
  }

  private fireEventsUpTo(sNow: number): void {
    for (let i = 0; i < this.eventS.length; i++) {
      if (!this.eventFired[i] && this.eventS[i]! <= sNow) {
        this.eventFired[i] = true;
        for (const l of this.listeners) l(this.eventNames[i]!);
      }
    }
  }

  // ------------------------------------------------------------------------------ telemetry

  get phase(): Phase {
    return this.phaseNow;
  }

  /** Progress along the path, metres. */
  get progressMeters(): number {
    return this.s;
  }

  /** Signed distance from the path, metres (positive left; 0 in the terminal phase). */
  get crossTrackMeters(): number {
    return this.crossTrack;
  }

  /** The pose the follower is steering for, in the robot's frame. */
  get reference(): Pose {
    return flipPose(this.frame, { xMeters: this.refX, yMeters: this.refY, headingRadians: this.refHeading });
  }

  /** Every event's name, in firing order, and its metres along the path. */
  get events(): { name: string; sMeters: number }[] {
    return this.eventNames.map((name, i) => ({ name, sMeters: this.eventBaseS[i]! }));
  }
}
