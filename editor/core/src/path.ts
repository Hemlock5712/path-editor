// A ready-to-follow path: geometry, heading profile and velocity profile. Port of lib's Path.java.

import { PathGeometry } from "./geometry.ts";
import { HeadingProfile } from "./heading.ts";
import { DEFAULT_GRID_METERS, VelocityProfile } from "./profile.ts";
import type { RobotConfig } from "./robot.ts";
import type { PathSpec } from "./spec.ts";

/** Everything the plan says about one distance along the path. */
export interface PathSample {
  readonly s: number;
  readonly xMeters: number;
  readonly yMeters: number;
  readonly tangentX: number;
  readonly tangentY: number;
  readonly curvature: number;
  readonly dCurvatureDs: number;
  readonly headingRadians: number;
  readonly headingRadiansPerMeter: number;
  readonly metersPerSecond: number;
  readonly metersPerSecondSquared: number;
  readonly seconds: number;
}

/** Length of the segment a pure rotation is planned on, metres (as in Java). */
export const PURE_ROTATION_METERS = 0.001;

/**
 * Two waypoints at the same place are a turn on the spot: planned on a 1 mm segment that ends exactly on
 * the point, along the final heading (Path.pureRotation in Java).
 */
export function pureRotation(spec: PathSpec): PathSpec {
  const w = spec.waypoints;
  if (w.length !== 2 || Math.hypot(w[0]!.xMeters - w[1]!.xMeters, w[0]!.yMeters - w[1]!.yMeters) > 1e-9) {
    return spec;
  }
  let end = Number.NaN;
  for (const t of spec.rotationTargets) end = t.headingRadians;
  const dir = Number.isNaN(end) ? 0.0 : end;
  const p = w[1]!;
  return {
    ...spec,
    waypoints: [
      {
        xMeters: p.xMeters - PURE_ROTATION_METERS * Math.cos(dir),
        yMeters: p.yMeters - PURE_ROTATION_METERS * Math.sin(dir),
        tangentRadians: dir,
        tangentScale: 1.0,
      },
      { xMeters: p.xMeters, yMeters: p.yMeters, tangentRadians: dir, tangentScale: 1.0 },
    ],
    startMetersPerSecond: 0.0,
    endMetersPerSecond: 0.0,
  };
}

export class Path {
  readonly spec: PathSpec;
  readonly geometry: PathGeometry;
  readonly heading: HeadingProfile;
  readonly profile: VelocityProfile;

  private constructor(spec: PathSpec, geometry: PathGeometry, heading: HeadingProfile, profile: VelocityProfile) {
    this.spec = spec;
    this.geometry = geometry;
    this.heading = heading;
    this.profile = profile;
  }

  static build(spec: PathSpec, robot: RobotConfig, gridMeters = DEFAULT_GRID_METERS): Path {
    spec = pureRotation(spec);
    const geometry = PathGeometry.build(spec.waypoints);
    const heading = HeadingProfile.build(geometry, spec.rotationTargets, spec.facePointZones);
    const profile = VelocityProfile.build(geometry, heading, spec, robot, gridMeters);
    return new Path(spec, geometry, heading, profile);
  }

  get lengthMeters(): number {
    return this.geometry.lengthMeters;
  }

  get totalTimeSeconds(): number {
    return this.profile.totalTimeSeconds;
  }

  /** The whole plan at s (clamped to the path). */
  sample(s: number): PathSample {
    const g = this.geometry.sample(s);
    const h = this.heading.sampleAt(g);
    const sc = g.s;
    return {
      s: sc,
      xMeters: g.xMeters,
      yMeters: g.yMeters,
      tangentX: g.tangentX,
      tangentY: g.tangentY,
      curvature: g.curvature,
      dCurvatureDs: g.dCurvatureDs,
      headingRadians: h.radians,
      headingRadiansPerMeter: h.radiansPerMeter,
      metersPerSecond: this.profile.velocityAt(sc),
      metersPerSecondSquared: this.profile.accelerationAt(sc),
      seconds: this.profile.timeAt(sc),
    };
  }

  /** The plan at time t seconds after the start. */
  sampleAtTime(t: number): PathSample {
    return this.sample(this.profile.sAtTime(t));
  }
}
