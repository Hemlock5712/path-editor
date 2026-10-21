// Port of lib's PathFileSamples: the same 32-bit LCG and the same draws in the same order, so both
// languages make the same specs. fixtures/files/path_files.json holds the Java writer's text for each.

import {
  eventAtMeters,
  eventAtWaypoint,
  makeConstraintZone,
  makeFacePointZone,
  makePathSpec,
  makeRotationTarget,
  makeWaypoint,
  type ConstraintZone,
  type FacePointZone,
  type PathEvent,
  type PathSpec,
  type RotationTarget,
  type Waypoint,
} from "../src/spec.ts";

export const COUNT = 40;
export const SEED = 5712;

const NAMES = ["intakeDown", "shoot", 'score "L4"', "align tag 18", "back\\slash", "tab\there", "ünïcödé", "x"];

class Lcg {
  private state: number;

  constructor(seed: number) {
    this.state = seed | 0;
  }

  /** Uniform on [0, 1), 24 bits; Java's int overflow is Math.imul plus | 0. */
  next(): number {
    this.state = (Math.imul(this.state, 1664525) + 1013904223) | 0;
    return (this.state >>> 8) / 16777216.0;
  }

  below(n: number): number {
    return Math.floor(this.next() * n);
  }
}

export function allSamples(): PathSpec[] {
  const g = new Lcg(SEED);
  const specs: PathSpec[] = [];
  for (let i = 0; i < COUNT; i++) specs.push(spec(g));
  return specs;
}

function spec(g: Lcg): PathSpec {
  const n = 2 + g.below(5);
  const last = n - 1;
  const waypoints: Waypoint[] = [];
  for (let i = 0; i < n; i++) {
    const x = -8.0 + 16.0 * g.next();
    const y = -4.0 + 8.0 * g.next();
    const tangent = g.next() < 0.4 ? (g.next() * 2.0 - 1.0) * Math.PI : Number.NaN;
    const scale = g.next() < 0.3 ? 0.5 + g.next() : 1.0;
    const heading = g.next() < 0.5 ? (g.next() * 2.0 - 1.0) * Math.PI : Number.NaN;
    waypoints.push(makeWaypoint(x, y, tangent, scale, heading));
  }

  const rotationTargets: RotationTarget[] = [];
  for (let k = g.below(3); k > 0; k--) {
    const byWaypoint = g.next() < 0.5;
    const where = byWaypoint ? g.next() * last : g.next() * 10.0;
    const heading = (g.next() * 2.0 - 1.0) * Math.PI;
    rotationTargets.push(
      byWaypoint ? makeRotationTarget(where, Number.NaN, heading) : makeRotationTarget(Number.NaN, where, heading),
    );
  }

  const facePointZones: FacePointZone[] = [];
  for (let k = g.below(2); k > 0; k--) {
    const from = g.next() * last * 0.5;
    const to = from + 0.1 + g.next() * (last - from - 0.1);
    facePointZones.push(makeFacePointZone(from, to, { x: -8.0 + 16.0 * g.next(), y: -4.0 + 8.0 * g.next() }, g.next() * 2.0));
  }

  const constraintZones: ConstraintZone[] = [];
  for (let k = g.below(2); k > 0; k--) {
    const from = g.next() * last * 0.5;
    const to = from + 0.1 + g.next() * (last - from - 0.1);
    const maxSpeed = g.next() < 0.3 ? Infinity : 0.5 + g.next() * 3.0;
    const maxAccel = g.next() < 0.5 ? Infinity : 1.0 + g.next() * 5.0;
    constraintZones.push(makeConstraintZone(from, to, maxSpeed, maxAccel));
  }

  const events: PathEvent[] = [];
  for (let k = g.below(4); k > 0; k--) {
    const name = NAMES[g.below(NAMES.length)]!;
    events.push(g.next() < 0.5 ? eventAtWaypoint(name, g.next() * last) : eventAtMeters(name, g.next() * 12.0));
  }

  const maxMetersPerSecond = g.next() < 0.5 ? Infinity : 1.0 + g.next() * 4.0;
  const maxMetersPerSecondSquared = g.next() < 0.5 ? Infinity : 1.0 + g.next() * 4.0;
  const startMetersPerSecond = g.next() < 0.3 ? g.next() * 2.0 : 0.0;
  const endMetersPerSecond = g.next() < 0.3 ? g.next() * 2.0 : 0.0;
  const headingMode = g.next() < 0.3 ? "faceTravel" : "hold";
  return makePathSpec({
    waypoints,
    rotationTargets,
    facePointZones,
    constraintZones,
    maxMetersPerSecond,
    maxMetersPerSecondSquared,
    startMetersPerSecond,
    endMetersPerSecond,
    events,
    headingMode,
  });
}
