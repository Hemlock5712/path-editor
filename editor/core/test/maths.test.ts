// Properties of the ported maths, mirroring the Java tests. Exact agreement with Java is the
// fixture test's job; these check the port behaves on paths the fixtures don't cover.

import { describe, expect, it } from "vitest";
import { smoothstep5, wrap } from "../src/angles.ts";
import { flipHeading, flipPoint, flipPose, flipSpeeds, type FieldFrame } from "../src/frame.ts";
import { PathGeometry, tangentRadians } from "../src/geometry.ts";
import { HeadingProfile } from "../src/heading.ts";
import { Path } from "../src/path.ts";
import {
  DEFAULT_ROBOT,
  brakeAccelMetersPerSecondSquared,
  driveAccelMetersPerSecondSquared,
  freeSpeedMetersPerSecond,
} from "../src/robot.ts";
import { QuinticSpline } from "../src/spline.ts";
import {
  makeConstraintZone,
  makeFacePointZone,
  makePathSpec,
  makeRotationTarget,
  makeWaypoint,
  type Waypoint,
} from "../src/spec.ts";

/** A small deterministic generator (mulberry32). */
function rng(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function randomWaypoints(next: () => number): Waypoint[] {
  const n = 2 + Math.floor(next() * 11);
  const w: Waypoint[] = [];
  let x = -6 + next() * 2;
  let y = -3 + next() * 6;
  for (let i = 0; i < n; i++) {
    const tangent = next() < 0.3 ? (next() * 2 - 1) * Math.PI : Number.NaN;
    const scale = next() < 0.2 ? 0.6 + next() * 0.8 : 1.0;
    w.push(makeWaypoint(x, y, tangent, scale));
    const heading = next() * 2 * Math.PI;
    const step = 0.3 + next() * 2.5;
    x += step * Math.cos(heading);
    y += step * Math.sin(heading);
  }
  return w;
}

const S_CURVE = [makeWaypoint(-4, -1, 0), makeWaypoint(-1, 0.5), makeWaypoint(2, -0.5), makeWaypoint(4, 1, 0)];
const HAIRPIN = [
  makeWaypoint(-3, -1, 0),
  makeWaypoint(0, -1),
  makeWaypoint(1, 0),
  makeWaypoint(0, 1),
  makeWaypoint(-3, 1, Math.PI),
];

function curvature(d: Float64Array): number {
  const q = d[2]! * d[2]! + d[3]! * d[3]!;
  return (d[2]! * d[5]! - d[3]! * d[4]!) / (q * Math.sqrt(q));
}

describe("spline", () => {
  it("is curvature-continuous across every knot of random paths", () => {
    const next = rng(5712);
    const d = new Float64Array(8);
    for (let trial = 0; trial < 300; trial++) {
      const spline = QuinticSpline.through(randomWaypoints(next));
      for (let k = 1; k < spline.segmentCount; k++) {
        spline.evaluate(k - 1, 1.0, d);
        const before = curvature(d);
        spline.evaluate(k, 0.0, d);
        expect(Math.abs(before - curvature(d))).toBeLessThan(1e-6);
        expect(Math.abs(spline.knotCurvature(k) - curvature(d))).toBeLessThan(1e-6);
      }
    }
  });

  it("gives a circle's curvature for points on a circle", () => {
    const w: Waypoint[] = [];
    for (let i = 0; i <= 6; i++) w.push(makeWaypoint(2 * Math.cos((i * Math.PI) / 4), 2 * Math.sin((i * Math.PI) / 4)));
    const g = PathGeometry.build(w);
    for (let s = 0; s <= g.lengthMeters; s += 0.05) expect(g.sample(s).curvature).toBeCloseTo(0.5, 3);
  });

  it("rejects repeated waypoints", () => {
    expect(() => QuinticSpline.through([makeWaypoint(1, 1), makeWaypoint(1, 1)])).toThrow(/same place/);
  });
});

describe("geometry", () => {
  const g = PathGeometry.build(HAIRPIN);

  it("keeps arc length monotone and chords no longer than arcs", () => {
    let prev = g.sample(0);
    for (let s = 0.01; s <= g.lengthMeters; s += 0.01) {
      const p = g.sample(s);
      const chord = Math.hypot(p.xMeters - prev.xMeters, p.yMeters - prev.yMeters);
      expect(chord).toBeLessThanOrEqual(0.01 * (1 + 1e-3));
      expect(chord).toBeGreaterThanOrEqual(0.01 * (1 - 1e-3));
      prev = p;
    }
  });

  it("maps waypoints to their distances", () => {
    for (let i = 0; i < g.waypointCount; i++) {
      const p = g.sample(g.waypointS(i));
      expect(p.xMeters).toBeCloseTo(HAIRPIN[i]!.xMeters, 4);
      expect(p.yMeters).toBeCloseTo(HAIRPIN[i]!.yMeters, 4);
    }
    expect(g.sAtWaypoint(1.5)).toBeCloseTo(0.5 * (g.waypointS(1) + g.waypointS(2)), 12);
    expect(g.sAtWaypoint(-1)).toBe(0);
    expect(g.sAtWaypoint(99)).toBe(g.lengthMeters);
  });

  it("projects points on and beside the path", () => {
    for (let s = 0.5; s < g.lengthMeters - 0.5; s += 0.37) {
      const p = g.sample(s);
      const x = p.xMeters - p.tangentY * 0.2;
      const y = p.yMeters + p.tangentX * 0.2;
      const pr = g.project(x, y, s + 0.1, 0.6);
      expect(pr.s).toBeCloseTo(s, 7);
      expect(pr.crossTrackMeters).toBeCloseTo(0.2, 7);
      expect(g.projectGlobal(p.xMeters, p.yMeters).s).toBeCloseTo(s, 7);
    }
    // equally near both legs: the window picks the leg
    expect(g.sample(g.project(-1, 0, 2, 0.75).s).yMeters).toBeLessThan(0);
    expect(g.sample(g.project(-1, 0, g.lengthMeters - 2, 0.75).s).yMeters).toBeGreaterThan(0);
  });
});

describe("heading", () => {
  const g = PathGeometry.build(S_CURVE);

  it("faces the direction of travel with no targets", () => {
    const h = HeadingProfile.build(g, [], []);
    for (let s = 0; s <= g.lengthMeters; s += 0.5) {
      const gs = g.sample(s);
      expect(h.sample(s).radians).toBeCloseTo(tangentRadians(gs), 12);
      expect(h.sample(s).radiansPerMeter).toBeCloseTo(gs.curvature, 12);
    }
  });

  it("blends with minimum jerk and turns the short way", () => {
    const a = g.waypointS(1);
    const b = g.waypointS(2);
    const h = HeadingProfile.build(g, [makeRotationTarget(Number.NaN, a, 0), makeRotationTarget(Number.NaN, b, 1)], []);
    expect(h.sample(a + 0.25 * (b - a)).radians).toBeCloseTo(smoothstep5(0.25), 12);
    expect(h.sample(0).radians).toBe(0);
    const wrapAround = HeadingProfile.build(
      g,
      [makeRotationTarget(0, Number.NaN, (170 * Math.PI) / 180), makeRotationTarget(3, Number.NaN, (-170 * Math.PI) / 180)],
      [],
    );
    expect(Math.abs(wrapAround.sample(0.5 * g.lengthMeters).radians)).toBeCloseTo(Math.PI, 12);
  });

  it("faces the point inside a face-point zone and has consistent derivatives", () => {
    const point = { x: 0, y: 3 };
    const h = HeadingProfile.build(g, [makeRotationTarget(0, Number.NaN, 0)], [makeFacePointZone(1, 2, point, 0.5)]);
    const s = 0.5 * (g.waypointS(1) + g.waypointS(2));
    const p = g.sample(s);
    expect(h.sample(s).radians).toBeCloseTo(Math.atan2(point.y - p.yMeters, point.x - p.xMeters), 12);
    const d = 1e-5;
    for (let t = 0.05; t < g.lengthMeters - 0.05; t += 0.0731) {
      const fd = wrap(h.sample(t + d).radians - h.sample(t - d).radians) / (2 * d);
      const exact = h.sample(t).radiansPerMeter;
      expect(Math.abs(fd - exact)).toBeLessThan(1e-5 + 5e-4 * Math.abs(exact));
    }
  });
});

describe("velocity profile", () => {
  it("is the textbook trapezoid on a straight line", () => {
    const spec = makePathSpec({
      waypoints: [makeWaypoint(-1.5, 0, 0), makeWaypoint(1.5, 0, 0)],
      maxMetersPerSecond: 4,
      maxMetersPerSecondSquared: 6,
    });
    const path = Path.build(spec, DEFAULT_ROBOT);
    expect(path.totalTimeSeconds).toBeCloseTo((2 * 4) / 6 + (3 - (2 * 16) / 12) / 4, 3);
    expect(path.profile.velocityAt(1.5)).toBeCloseTo(4, 12);
  });

  it("keeps the grip budget at every grid point and respects zones and end speeds", () => {
    const spec = makePathSpec({
      waypoints: HAIRPIN,
      constraintZones: [makeConstraintZone(1, 2, 1.5)],
      maxMetersPerSecond: 4,
      maxMetersPerSecondSquared: 6,
      endMetersPerSecond: 0.5,
    });
    const path = Path.build(spec, DEFAULT_ROBOT);
    const p = path.profile;
    const grip = DEFAULT_ROBOT.frictionCoefficient * 9.81;
    for (let i = 0; i + 1 < p.gridPointCount; i++) {
      const s = i * p.gridMeters;
      const v = p.gridVelocity(i);
      const v1 = p.gridVelocity(i + 1);
      const a = (v1 * v1 - v * v) / (2 * p.gridMeters);
      const kappa = Math.abs(path.geometry.sample(s).curvature);
      expect(Math.hypot(a, v * v * kappa)).toBeLessThanOrEqual(grip * (1 + 1e-9));
    }
    for (let s = path.geometry.waypointS(1); s <= path.geometry.waypointS(2); s += 0.01) {
      expect(p.velocityAt(s)).toBeLessThanOrEqual(1.5 + 1e-9);
    }
    expect(p.velocityAt(path.lengthMeters)).toBeCloseTo(0.5, 12);
    for (let s = 0; s <= path.lengthMeters; s += 0.013) expect(p.sAtTime(p.timeAt(s))).toBeCloseTo(s, 9);
    expect(path.sampleAtTime(path.totalTimeSeconds).xMeters).toBeCloseTo(-3, 4);
  });

  it("motor numbers match the Java model", () => {
    const stalled = (4 * (9.362 / 476.1) * 120 * 6.75) / 0.0508 / 60;
    expect(driveAccelMetersPerSecondSquared(DEFAULT_ROBOT, 0)).toBeCloseTo(stalled, 9);
    expect(brakeAccelMetersPerSecondSquared(DEFAULT_ROBOT)).toBeCloseTo(stalled, 9);
    expect(driveAccelMetersPerSecondSquared(DEFAULT_ROBOT, freeSpeedMetersPerSecond(DEFAULT_ROBOT))).toBeCloseTo(0, 9);
  });
});

describe("frames", () => {
  const frames: FieldFrame[] = ["BLUE", "RED_HALF_TURN", "RED_MIRROR"];

  it("are their own inverses", () => {
    for (const f of frames) {
      const pose = { xMeters: 3.2, yMeters: -1.7, headingRadians: 2.5 };
      const back = flipPose(f, flipPose(f, pose));
      expect(back.xMeters).toBe(pose.xMeters);
      expect(wrap(back.headingRadians - pose.headingRadians)).toBeCloseTo(0, 15);
      const v = { vxMetersPerSecond: 1.5, vyMetersPerSecond: -0.4, omegaRadPerSec: 0.9 };
      expect(flipSpeeds(f, flipSpeeds(f, v))).toEqual(v);
    }
    expect(flipPoint("RED_MIRROR", { x: 5, y: 2 })).toEqual({ x: -5, y: 2 });
    expect(flipHeading("RED_MIRROR", Math.PI / 6)).toBeCloseTo((5 * Math.PI) / 6, 15);
  });

  it("the flipped path is the transformed blue path", () => {
    const frame: FieldFrame = "RED_MIRROR";
    const blue = Path.build(makePathSpec({ waypoints: S_CURVE, maxMetersPerSecond: 4 }), DEFAULT_ROBOT);
    const redWaypoints = S_CURVE.map((w) => {
      const p = flipPoint(frame, { x: w.xMeters, y: w.yMeters });
      return makeWaypoint(p.x, p.y, Number.isNaN(w.tangentRadians) ? Number.NaN : flipHeading(frame, w.tangentRadians));
    });
    const red = Path.build(makePathSpec({ waypoints: redWaypoints, maxMetersPerSecond: 4 }), DEFAULT_ROBOT);
    expect(red.totalTimeSeconds).toBeCloseTo(blue.totalTimeSeconds, 9);
    for (let s = 0; s <= blue.lengthMeters; s += 0.1) {
      const b = blue.sample(s);
      const r = red.sample(s);
      expect(r.xMeters).toBeCloseTo(-b.xMeters, 8);
      expect(r.yMeters).toBeCloseTo(b.yMeters, 8);
      expect(r.curvature).toBeCloseTo(-b.curvature, 8);
      expect(r.metersPerSecond).toBeCloseTo(b.metersPerSecond, 8);
    }
  });
});
