// The robot's outline seen from above. Port of lib's core/Footprint.java: a rectangle around the robot's
// centre, given by how far it reaches to the front (along its heading), the back, the left and the right.
// "bumpers" is the bumpers alone; robot.json names the others, each the bumpers plus a mechanism that sticks
// out (an intake, an arm). A path event can switch to one (PathEvent.footprint); the editor checks the outline
// in force along the path, and the pathfinder plans with whichever one it's given.

import { javaHypot } from "./jmath.ts";
import type { JsonValue } from "./json.ts";

export interface Footprint {
  /** What events and PathSpec.startFootprint call it. */
  readonly name: string;
  readonly frontMeters: number;
  readonly backMeters: number;
  readonly leftMeters: number;
  readonly rightMeters: number;
  /** How long the mechanism takes to come back in: after an event switches away from it, the checks keep it this long. */
  readonly retractSeconds: number;
}

/** The bumpers alone: every path starts with them unless it says otherwise. */
export const BUMPERS = "bumpers";

function fail(message: string): never {
  throw new RangeError(message);
}

export function makeFootprint(
  name: string,
  frontMeters: number,
  backMeters: number,
  leftMeters: number,
  rightMeters: number,
  retractSeconds = 0.0,
): Footprint {
  if (name.trim() === "") fail("a footprint needs a name");
  if (
    !(frontMeters > -backMeters) ||
    !(leftMeters > -rightMeters) ||
    !Number.isFinite(frontMeters + backMeters) ||
    !Number.isFinite(leftMeters + rightMeters)
  ) {
    fail(`footprint ${name} must have a positive, finite size`);
  }
  if (!(retractSeconds >= 0) || !Number.isFinite(retractSeconds)) fail(`footprint ${name}: retractSeconds must be >= 0`);
  return { name, frontMeters, backMeters, leftMeters, rightMeters, retractSeconds };
}

/** The bumpers: a length × width rectangle centred on the robot. */
export function bumperFootprint(lengthMeters: number, widthMeters: number): Footprint {
  return makeFootprint(BUMPERS, lengthMeters / 2, lengthMeters / 2, widthMeters / 2, widthMeters / 2, 0.0);
}

/** The bumpers plus a mechanism reaching this much further on each side. */
export function extendedFootprint(
  bumpers: Footprint,
  name: string,
  front: number,
  back: number,
  left: number,
  right: number,
  retractSeconds: number,
): Footprint {
  return makeFootprint(
    name,
    bumpers.frontMeters + front,
    bumpers.backMeters + back,
    bumpers.leftMeters + left,
    bumpers.rightMeters + right,
    retractSeconds,
  );
}

/** How far the outline reaches from the robot's centre in any heading: the far corner. */
export function reachMeters(f: Footprint): number {
  return javaHypot(2 * Math.max(f.frontMeters, f.backMeters), 2 * Math.max(f.leftMeters, f.rightMeters)) / 2;
}

/** The smallest rectangle around both (each side the further of the two), named after the first. */
export function unionFootprint(a: Footprint, b: Footprint): Footprint {
  return {
    name: a.name,
    frontMeters: Math.max(a.frontMeters, b.frontMeters),
    backMeters: Math.max(a.backMeters, b.backMeters),
    leftMeters: Math.max(a.leftMeters, b.leftMeters),
    rightMeters: Math.max(a.rightMeters, b.rightMeters),
    retractSeconds: Math.max(a.retractSeconds, b.retractSeconds),
  };
}

/** A footprint as robot.json holds it: how much further than the bumpers it reaches on each side (absent = 0). */
export interface FootprintExtension {
  readonly name: string;
  readonly frontMeters: number;
  readonly backMeters: number;
  readonly leftMeters: number;
  readonly rightMeters: number;
  readonly retractSeconds: number;
}

/** robot.json's "footprints" list, checked: unique names, none "bumpers", every number >= 0. */
export function extensionsFromJson(list: JsonValue | undefined): FootprintExtension[] {
  if (list === undefined || list === null) return [];
  if (!Array.isArray(list)) fail("'footprints' must be a list");
  const names = new Set([BUMPERS]);
  return list.map((o) => {
    if (o === null || typeof o !== "object" || Array.isArray(o)) fail("footprint must be a JSON object");
    const m = o as { [key: string]: JsonValue };
    if (typeof m.name !== "string") fail("a footprint needs a 'name' string");
    const name = m.name;
    if (names.has(name)) fail(`two footprints are called ${name}${name === BUMPERS ? ' ("bumpers" is the bumpers alone)' : ""}`);
    names.add(name);
    const extension = (key: string) => {
      const v = m[key] ?? 0.0;
      if (typeof v !== "number") fail(`'${key}' must be a number`);
      if (!(v >= 0) || !Number.isFinite(v)) fail(`footprint ${name}: '${key}' must be >= 0`);
      return v;
    };
    return {
      name,
      frontMeters: extension("frontMeters"),
      backMeters: extension("backMeters"),
      leftMeters: extension("leftMeters"),
      rightMeters: extension("rightMeters"),
      retractSeconds: extension("retractSeconds"),
    };
  });
}

/** The robot's footprints: the bumpers first, then each extension added to them (Java's Footprint.fromRobotFile). */
export function robotFootprints(
  bumperLengthMeters: number,
  bumperWidthMeters: number,
  extensions: readonly FootprintExtension[],
): Footprint[] {
  const base = bumperFootprint(bumperLengthMeters, bumperWidthMeters);
  return [
    base,
    ...extensions.map((e) => extendedFootprint(base, e.name, e.frontMeters, e.backMeters, e.leftMeters, e.rightMeters, e.retractSeconds)),
  ];
}
