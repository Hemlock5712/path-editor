// The path follower: a line-for-line port of lib's PathFollower, OutputLimiter and StoppingLaw
// (frc.team5712.path.follow), for the editor's sim preview. Keep the arithmetic in Java's order: the
// closed-loop golden traces (fixtures/follow) must come out to 1e-9.
//
// Left out, and Java-only: adaptive grip and slip sensing (the preview assumes the floor grips), and the
// moving goal's tail rebuild. With no slip signal, Java's adaptive grip leaves every number unchanged, so
// the port matches it as is.

import { wrap } from "./angles.ts";
import { flipPose, flipSpeeds, type FieldFrame, type FieldSpeeds, type Pose } from "./frame.ts";
import { javaHypot } from "./jmath.ts";
import { PURE_ROTATION_METERS, Path, eventMeters, type PathSample } from "./path.ts";
import { DEFAULT_GRID_METERS } from "./profile.ts";
import type { PathSpec } from "./spec.ts";
import {
  GRAVITY,
  driveAccelMetersPerSecondSquared,
  spinLeverMeters,
  tractionMetersPerSecondSquared,
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
  /** The limiter's budget as a μ; NaN = the robot's traction limit. */
  readonly limiterFrictionCoefficient: number;
  readonly positionToleranceMeters: number;
  readonly headingToleranceRadians: number;
  readonly settledMetersPerSecond: number;
  readonly debounceSeconds: number;
  /** At start, when already moving (not backwards along the path), re-plan the speeds from the measured speed. */
  readonly replanStartFromMeasuredVelocity: boolean;
  /** Knocked further than this off the path, build a piece back onto it (+∞ = never). */
  readonly rejoinAboveMeters: number;
}

/** How far ahead (along the path) a rejoining piece meets it. */
export const REJOIN_METERS = 1.0;

/** At most one rejoin per this long. */
export const REJOIN_COOLDOWN_SECONDS = 0.5;

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
    limiterFrictionCoefficient: Number.NaN,
    positionToleranceMeters: 0.02,
    headingToleranceRadians: 1.0 * DEG,
    settledMetersPerSecond: 0.05,
    debounceSeconds: 0.1,
    replanStartFromMeasuredVelocity: true,
    rejoinAboveMeters: 0.4,
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

  constructor(robot: RobotConfig, budgetMetersPerSecondSquared: number, maxAlphaRadPerSecSquared: number) {
    this.robot = robot;
    this.grip = budgetMetersPerSecondSquared;
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
      // only the push along the travel is the motors'; turning the velocity is the tread's grip
      const speed = Math.sqrt(this.vx * this.vx + this.vy * this.vy);
      const wheel = speed + Math.abs(this.omega) * this.robot.moduleRadiusMeters;
      const motor = driveAccelMetersPerSecondSquared(this.robot, wheel);
      const ux = this.vx / speed;
      const uy = this.vy / speed;
      const along = ax * ux + ay * uy;
      if (along > motor) {
        ax -= (along - motor) * ux;
        ay -= (along - motor) * uy;
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
/** Progress reads no closer to a cusp than this (its tangent is undefined), and steps past it by as much. */
const CUSP_EDGE_METERS = 1e-3;
const MAX_PROGRESS_RATE = 8.0;
const MIN_TAIL_METERS = 0.15;
const ROLLING_LEAD_SECONDS = 0.3;
const MIN_MOVING_SPEED = 0.1;
const MIN_POSE_DT = 0.005;
const MAX_POSE_SPEED_RATIO = 1.5;

/** The output limiter's budget, m/s²: the robot's traction limit, or the configured μ·g (PathFollower.limiterBudget). */
export function limiterBudget(cfg: FollowerConfig): number {
  return Number.isNaN(cfg.limiterFrictionCoefficient) ? tractionMetersPerSecondSquared(cfg.robot)
    : cfg.limiterFrictionCoefficient * GRAVITY;
}

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
  private startTime = 0;
  private settledSince = Number.NaN;
  private crossTrack = 0;
  private refX = 0;
  private refY = 0;
  private refHeading = 0;
  // The pose at the previous step, and the velocity between the two (PathFollower.java)
  private prevX = Number.NaN;
  private prevY = 0;
  private prevPoseTime = 0;
  private poseVx = 0;
  private poseVy = 0;
  // The path followed: the track (the path, or it re-planned from the measured speed at start), or a piece
  // from where the robot was knocked to back onto the track at headJoinS, replacing the stretch from headStartS
  private active: Path;
  private track: Path;
  private head: Path | null = null;
  private headStartS = 0;
  private headJoinS = 0;
  private rejoins = 0;
  private lastRejoinTime = Number.NEGATIVE_INFINITY;

  constructor(path: Path, config: FollowerConfig) {
    this.path = path;
    this.active = path;
    this.track = path;
    this.cfg = config;
    this.limiter = new OutputLimiter(config.robot, limiterBudget(config), config.maxAlphaRadPerSecSquared);
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
    this.active = this.path;
    this.s = 0.0;
    this.updateProgress(p);
    this.lastS = this.s;
    this.lastTime = timeSeconds;
    this.startTime = timeSeconds;
    this.prevX = Number.NaN;
    this.settledSince = Number.NaN;
    this.phaseNow = "TRACKING";
    this.eventS = this.eventBaseS.slice();
    this.eventFired.fill(false);
    this.head = null;
    this.rejoins = 0;
    this.lastRejoinTime = Number.NEGATIVE_INFINITY;
    this.track = this.path;
    const v = flipSpeeds(frame, measuredVelocity);
    const speed = javaHypot(v.vxMetersPerSecond, v.vyMetersPerSecond);
    const first = this.path.sample(0.0);
    if (this.cfg.replanStartFromMeasuredVelocity && !this.pureRotation && this.path.spec.startMetersPerSecond === 0.0
      && speed > MIN_MOVING_SPEED
      && v.vxMetersPerSecond * first.tangentX + v.vyMetersPerSecond * first.tangentY > 0.0) {
      // a path planned from rest, started while moving: the speeds are re-planned from the measured speed
      // (only the profile is rebuilt; a path that starts moving already plans its start speed)
      this.track = this.path.withStartSpeed(speed, this.cfg.robot);
      this.active = this.track;
    }
  }

  /**
   * Replaces the track up to joinS with a piece from the robot's pose back onto it, if the join is ahead
   * of the robot; returns whether it was built (PathFollower.rejoin).
   */
  private rejoin(pose: Pose, vx: number, vy: number, joinS: number): boolean {
    const track = this.track;
    const length = track.lengthMeters;
    joinS = Math.min(joinS, length);
    const j = track.sample(joinS);
    const v = javaHypot(vx, vy);
    const dx = j.xMeters - pose.xMeters;
    const dy = j.yMeters - pose.yMeters;
    const toEnd = joinS >= length - 1e-9;
    const straightIn = toEnd && !this.rollingEnd; // a rolling end keeps its exit direction
    if (javaHypot(dx, dy) < MIN_TAIL_METERS || (v > MIN_MOVING_SPEED && vx * dx + vy * dy <= 0.0)
      || (!straightIn && j.tangentX * dx + j.tangentY * dy <= 0.0)) {
      return false;
    }
    const chord = Math.atan2(dy, dx);
    const spec = track.spec;
    const startTangent = v > MIN_MOVING_SPEED ? Math.atan2(vy, vx) : chord;
    const endTangent = straightIn ? chord : Math.atan2(j.tangentY, j.tangentX);
    const endK = straightIn ? Number.NaN : j.curvature;
    const pieceSpec: PathSpec = {
      waypoints: [
        { xMeters: pose.xMeters, yMeters: pose.yMeters, tangentRadians: startTangent, tangentScale: 1.0, headingRadians: Number.NaN },
        { xMeters: j.xMeters, yMeters: j.yMeters, tangentRadians: endTangent, tangentScale: 1.0, headingRadians: Number.NaN },
      ],
      rotationTargets: [
        { waypoint: Number.NaN, meters: 0.0, headingRadians: pose.headingRadians },
        { waypoint: 1.0, meters: Number.NaN, headingRadians: j.headingRadians },
      ],
      facePointZones: [],
      constraintZones: [],
      maxMetersPerSecond: spec.maxMetersPerSecond,
      maxMetersPerSecondSquared: spec.maxMetersPerSecondSquared,
      startMetersPerSecond: Math.min(v, spec.maxMetersPerSecond),
      endMetersPerSecond: toEnd ? spec.endMetersPerSecond : j.metersPerSecond,
      events: [],
      headingMode: "hold",
    };
    let piece: Path;
    try {
      piece = Path.build(pieceSpec, this.cfg.robot, DEFAULT_GRID_METERS, Number.NaN, endK);
    } catch {
      return false; // can't be planned
    }
    if (piece.geometry.cusps.length > 0) return false;
    this.headStartS = this.trackProgress();
    this.head = piece;
    this.headJoinS = joinS;
    this.active = piece;
    this.s = 0.0;
    this.rejoins++;
    return true;
  }

  /** Metres left to the end of the path: on a rejoining piece, its rest plus the track after the join. */
  private remainingMeters(sNow: number): number {
    if (this.head !== null) return this.head.lengthMeters - sNow + (this.track.lengthMeters - this.headJoinS);
    return this.active.lengthMeters - sNow;
  }

  /** Progress in the track's metres (events are placed in them). */
  private trackProgress(): number {
    if (this.head !== null) {
      return this.headStartS + (this.headJoinS - this.headStartS) * Math.min(1.0, this.s / this.head.lengthMeters);
    }
    return this.s;
  }

  /** The robot has reached the end of the rejoining piece (or passed its end line, near it). */
  private headDone(pose: Pose): boolean {
    const head = this.head!;
    const length = head.lengthMeters;
    if (this.s >= length - 1e-3) return true;
    const end = head.sample(length);
    const past = (pose.xMeters - end.xMeters) * end.tangentX + (pose.yMeters - end.yMeters) * end.tangentY;
    return past >= 0.0 && length - this.s <= this.cfg.windowMeters;
  }

  /** The velocity from the pose's change since the last step (PathFollower.updatePoseVelocity). */
  private updatePoseVelocity(t: number, pose: Pose, vel: FieldSpeeds): void {
    const dt = t - this.prevPoseTime;
    this.poseVx = vel.vxMetersPerSecond;
    this.poseVy = vel.vyMetersPerSecond;
    if (!Number.isNaN(this.prevX) && dt > MIN_POSE_DT) {
      const vx = (pose.xMeters - this.prevX) / dt;
      const vy = (pose.yMeters - this.prevY) / dt;
      if (javaHypot(vx, vy) <= MAX_POSE_SPEED_RATIO * this.track.spec.maxMetersPerSecond) {
        this.poseVx = vx;
        this.poseVy = vy;
      }
    }
    this.prevX = pose.xMeters;
    this.prevY = pose.yMeters;
    this.prevPoseTime = t;
  }

  step(timeSeconds: number, measuredPose: Pose, measuredVelocity: FieldSpeeds): FieldSpeeds {
    if (this.phaseNow === "IDLE") throw new Error("step() before start()");
    if (this.phaseNow === "FINISHED" || this.phaseNow === "CANCELLED") {
      return this.rollingEnd && this.phaseNow === "FINISHED"
        ? this.output()
        : { vxMetersPerSecond: 0.0, vyMetersPerSecond: 0.0, omegaRadPerSec: 0.0 };
    }
    const cfg = this.cfg;
    const dt = timeSeconds > this.lastTime
      ? Math.max(0.001, Math.min(0.1, timeSeconds - this.lastTime))
      : NOMINAL_LOOP_SECONDS;
    this.lastTime = timeSeconds;
    const pose = flipPose(this.frame, measuredPose);
    const vel = flipSpeeds(this.frame, measuredVelocity);
    this.updatePoseVelocity(timeSeconds, pose, vel);
    if (!(Number.isFinite(pose.xMeters) && Number.isFinite(pose.yMeters) && Number.isFinite(pose.headingRadians))) {
      this.limiter.limit(0.0, 0.0, 0.0, dt);
      return this.output();
    }

    const brakeDecel = Math.min(cfg.brakeMetersPerSecondSquared, Infinity * GRAVITY); // no slip: Java's slipMu is +∞
    const gripScale = 1.0;
    this.updateProgress(pose);
    if (this.head !== null && this.headJoinS < this.track.lengthMeters - 1e-9 && this.headDone(pose)) {
      this.active = this.track; // the piece has met the track: on along it
      this.head = null;
      this.s = this.headJoinS;
      this.updateProgress(pose);
    }
    // Knocked well off: a piece from where the robot is, the way it is really going, back onto the track
    // a little ahead (or further, to the first join ahead of the robot; or to the goal, if that's near).
    // Never past a cusp: the robot must still get to where the path reverses.
    if (this.phaseNow === "TRACKING" && !this.pureRotation && Math.abs(this.crossTrack) > cfg.rejoinAboveMeters
      && timeSeconds - this.lastRejoinTime >= REJOIN_COOLDOWN_SECONDS) {
      this.lastRejoinTime = timeSeconds;
      const from = this.trackProgress();
      const cusp = this.track.geometry.nextCusp(from);
      const length = this.track.lengthMeters;
      const last = Number.isNaN(cusp) ? length : cusp - CUSP_EDGE_METERS;
      let join = from;
      do {
        join += REJOIN_METERS;
        if (join > length - cfg.terminalRadiusMeters) join = length;
        join = Math.min(join, last);
      } while (!this.rejoin(pose, this.poseVx, this.poseVy, join) && join < last);
    }
    const active = this.active;
    const remaining = this.remainingMeters(this.s);
    const goal = this.goal;
    if (this.phaseNow === "TRACKING" && !this.rollingEnd && remaining < cfg.terminalRadiusMeters
      && Math.abs(wrap(goal.headingRadians - active.heading.sample(this.s).radians)) <= cfg.terminalTurnRadians
      && Number.isNaN(active.geometry.nextCusp(this.s))) { // not while the path still has to double back
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
      let lead = cfg.latencySeconds;
      if (this.track.spec.startMetersPerSecond > 0.0) {
        // a rolling start: the lead builds up over ROLLING_LEAD_SECONDS (PathFollower.java)
        lead *= Math.min(1.0, (timeSeconds - this.startTime) / ROLLING_LEAD_SECONDS);
      }
      let sAhead = profile.sAtTime(profile.timeAt(this.s) + lead);
      const cusp = active.geometry.nextCusp(this.s);
      if (sAhead > cusp - CUSP_EDGE_METERS) {
        // the plan stops at a cusp and comes back: read ahead no further than the stop, on this side of it
        sAhead = Math.max(this.s, cusp - CUSP_EDGE_METERS);
      }
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

    const progress = this.trackProgress();
    const rate = Math.max(0.0, Math.min(MAX_PROGRESS_RATE, (progress - this.lastS) / dt));
    this.lastS = progress;
    this.fireEventsUpTo(progress + 0.5 * rate * dt);
    if (this.phaseNow === "TERMINAL" && this.arrived(pose)) this.fireEventsUpTo(this.track.lengthMeters);
    // not done while a piece is still to meet the track
    if (this.head === null || this.headJoinS >= this.track.lengthMeters - 1e-9) {
      this.checkFinished(timeSeconds, pose, vel, remaining);
    }
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
    const geometry = this.active.geometry;
    if (geometry.cusps.length > 0) {
      this.updateProgressAroundCusps(pose);
      return;
    }
    let near = geometry.project(pose.xMeters, pose.yMeters, this.s, this.cfg.windowMeters);
    if (near.distanceMeters > this.cfg.reprojectMeters) {
      const all = geometry.projectGlobal(pose.xMeters, pose.yMeters);
      if (all.distanceMeters < near.distanceMeters - 0.05) near = all;
    }
    this.s = near.s;
  }

  /** Progress on a path that doubles back: stays on the robot's side of the next cusp (PathFollower.java). */
  private updateProgressAroundCusps(pose: Pose): void {
    const geometry = this.active.geometry;
    const next = geometry.nextCusp(this.s);
    const prev = geometry.previousCusp(this.s);
    const lo = Number.isNaN(prev) ? 0.0 : prev + CUSP_EDGE_METERS;
    const hi = Number.isNaN(next) ? geometry.lengthMeters : next - CUSP_EDGE_METERS;
    let near = geometry.projectWithin(pose.xMeters, pose.yMeters,
      Math.max(lo, this.s - this.cfg.windowMeters), Math.min(hi, this.s + this.cfg.windowMeters));
    if (near.distanceMeters > this.cfg.reprojectMeters) {
      const all = geometry.projectGlobalWithin(pose.xMeters, pose.yMeters, lo, hi);
      if (all.distanceMeters < near.distanceMeters - 0.05) near = all;
    }
    this.s = near.s;
    if (!Number.isNaN(next) && next - this.s <= this.cfg.positionToleranceMeters) {
      this.s = Math.min(geometry.lengthMeters, next + CUSP_EDGE_METERS);
    }
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
      const end = this.active.sample(this.active.lengthMeters);
      const past = (pose.xMeters - end.xMeters) * end.tangentX + (pose.yMeters - end.yMeters) * end.tangentY;
      // past the end's line, once near the end (a path can start beyond that line, e.g. an on-the-fly one)
      if (remaining <= 1e-6 || (past >= 0.0 && remaining <= this.cfg.windowMeters)) this.finish();
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

  /** Progress along the path (on a rejoining piece, its share of the stretch it replaces), metres. */
  get progressMeters(): number {
    return this.trackProgress();
  }

  /** Pieces built this run to rejoin the path after the robot was knocked off it. */
  get rejoinCount(): number {
    return this.rejoins;
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
