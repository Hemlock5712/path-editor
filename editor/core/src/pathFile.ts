// Reads and writes *.path.json. Port of lib's PathFile.java and RobotFile.java: 5-decimal numbers,
// keys in a fixed order, degrees in the file and radians everywhere else, unknown keys ignored.

import { fixed5, writeJson, type JsonValue } from "./json.ts";
import { DEFAULT_ROBOT, checkRobot, type Motor, type RobotConfig } from "./robot.ts";
import {
  DEFAULT_BLEND_METERS,
  hasTangent,
  makeConstraintZone,
  makeFacePointZone,
  makePathSpec,
  makeRotationTarget,
  makeWaypoint,
  type PathSpec,
} from "./spec.ts";

/** The schema this code writes and reads. */
export const SCHEMA = 1;

const DEGREES = Math.PI / 180.0;

type JsonObject = { [key: string]: JsonValue };

export function readPath(text: string): PathSpec {
  return pathFromJson(asObject(JSON.parse(text) as JsonValue, "path"));
}

export function writePath(spec: PathSpec): string {
  return writeJson(pathToJson(spec), fixed5);
}

/** The file's JSON tree, keys in file order. */
export function pathToJson(spec: PathSpec): JsonObject {
  const root: JsonObject = { schema: SCHEMA };
  root.waypoints = spec.waypoints.map((w) => {
    const m: JsonObject = { x: w.xMeters, y: w.yMeters };
    if (hasTangent(w)) m.tangentDegrees = w.tangentRadians / DEGREES;
    if (w.tangentScale !== 1.0) m.tangentScale = w.tangentScale;
    return m;
  });
  root.rotationTargets = spec.rotationTargets.map((t) => {
    const m: JsonObject = !Number.isNaN(t.waypoint) ? { waypoint: t.waypoint } : { meters: t.meters };
    m.degrees = t.headingRadians / DEGREES;
    return m;
  });
  root.facePointZones = spec.facePointZones.map((z) => ({
    fromWaypoint: z.fromWaypoint,
    toWaypoint: z.toWaypoint,
    x: z.point.x,
    y: z.point.y,
    blendMeters: z.blendMeters,
  }));
  root.constraintZones = spec.constraintZones.map((z) => {
    const m: JsonObject = { fromWaypoint: z.fromWaypoint, toWaypoint: z.toWaypoint };
    putIfFinite(m, "maxMetersPerSecond", z.maxMetersPerSecond);
    putIfFinite(m, "maxMetersPerSecondSquared", z.maxMetersPerSecondSquared);
    return m;
  });
  putIfFinite(root, "maxMetersPerSecond", spec.maxMetersPerSecond);
  putIfFinite(root, "maxMetersPerSecondSquared", spec.maxMetersPerSecondSquared);
  root.startMetersPerSecond = spec.startMetersPerSecond;
  root.endMetersPerSecond = spec.endMetersPerSecond;
  return root;
}

function putIfFinite(m: JsonObject, key: string, v: number): void {
  if (Number.isFinite(v)) m[key] = v;
}

/** Builds a spec from a parsed file. */
export function pathFromJson(root: JsonObject): PathSpec {
  const schema = num(root, "schema", Number.NaN);
  if (schema !== SCHEMA) {
    throw new RangeError(
      Number.isNaN(schema) ? "not a path file: no schema" : `path file schema ${schema} (this code reads ${SCHEMA})`,
    );
  }
  const waypoints = list(root, "waypoints").map((o) => {
    const m = asObject(o, "waypoint");
    const tangent = num(m, "tangentDegrees", Number.NaN);
    return makeWaypoint(
      required(m, "x"),
      required(m, "y"),
      Number.isNaN(tangent) ? Number.NaN : tangent * DEGREES,
      num(m, "tangentScale", 1.0),
    );
  });
  const rotationTargets = list(root, "rotationTargets").map((o) => {
    const m = asObject(o, "rotation target");
    return makeRotationTarget(num(m, "waypoint", Number.NaN), num(m, "meters", Number.NaN), required(m, "degrees") * DEGREES);
  });
  const facePointZones = list(root, "facePointZones").map((o) => {
    const m = asObject(o, "face-point zone");
    return makeFacePointZone(
      required(m, "fromWaypoint"),
      required(m, "toWaypoint"),
      { x: required(m, "x"), y: required(m, "y") },
      num(m, "blendMeters", DEFAULT_BLEND_METERS),
    );
  });
  const constraintZones = list(root, "constraintZones").map((o) => {
    const m = asObject(o, "constraint zone");
    return makeConstraintZone(
      required(m, "fromWaypoint"),
      required(m, "toWaypoint"),
      num(m, "maxMetersPerSecond", Infinity),
      num(m, "maxMetersPerSecondSquared", Infinity),
    );
  });
  return makePathSpec({
    waypoints,
    rotationTargets,
    facePointZones,
    constraintZones,
    maxMetersPerSecond: num(root, "maxMetersPerSecond", Infinity),
    maxMetersPerSecondSquared: num(root, "maxMetersPerSecondSquared", Infinity),
    startMetersPerSecond: num(root, "startMetersPerSecond", 0.0),
    endMetersPerSecond: num(root, "endMetersPerSecond", 0.0),
  });
}

/** Reads a robot config; every key is optional and defaults to DEFAULT_ROBOT. */
export function robotFromJson(m: JsonObject): RobotConfig {
  const d = DEFAULT_ROBOT;
  let motor: Motor = d.driveMotor;
  if (m.motor !== undefined && m.motor !== null) {
    const mm = asObject(m.motor, "motor");
    motor = {
      nominalVolts: num(mm, "nominalVolts", motor.nominalVolts),
      stallTorqueNewtonMeters: num(mm, "stallTorqueNewtonMeters", motor.stallTorqueNewtonMeters),
      stallCurrentAmps: num(mm, "stallCurrentAmps", motor.stallCurrentAmps),
      freeSpeedRadPerSec: num(mm, "freeSpeedRadPerSec", motor.freeSpeedRadPerSec),
    };
  }
  return checkRobot({
    massKg: num(m, "massKg", d.massKg),
    momentOfInertiaKgM2: num(m, "momentOfInertiaKgM2", d.momentOfInertiaKgM2),
    moduleRadiusMeters: num(m, "moduleRadiusMeters", d.moduleRadiusMeters),
    wheelRadiusMeters: num(m, "wheelRadiusMeters", d.wheelRadiusMeters),
    driveGearRatio: num(m, "driveGearRatio", d.driveGearRatio),
    driveMotor: motor,
    driveMotorCount: Math.trunc(num(m, "driveMotorCount", d.driveMotorCount)),
    statorCurrentLimitAmps: num(m, "statorCurrentLimitAmps", d.statorCurrentLimitAmps),
    frictionCoefficient: num(m, "frictionCoefficient", d.frictionCoefficient),
    maxWheelMetersPerSecond: num(m, "maxWheelMetersPerSecond", d.maxWheelMetersPerSecond),
    maxAngularRadPerSec: num(m, "maxAngularRadPerSec", d.maxAngularRadPerSec),
  });
}

/** A robot config's JSON tree; unset optional limits are left out. */
export function robotToJson(r: RobotConfig): JsonObject {
  const m: JsonObject = {
    massKg: r.massKg,
    momentOfInertiaKgM2: r.momentOfInertiaKgM2,
    moduleRadiusMeters: r.moduleRadiusMeters,
    wheelRadiusMeters: r.wheelRadiusMeters,
    driveGearRatio: r.driveGearRatio,
    driveMotorCount: r.driveMotorCount,
    statorCurrentLimitAmps: r.statorCurrentLimitAmps,
    frictionCoefficient: r.frictionCoefficient,
  };
  putIfFinite(m, "maxWheelMetersPerSecond", r.maxWheelMetersPerSecond);
  putIfFinite(m, "maxAngularRadPerSec", r.maxAngularRadPerSec);
  m.motor = {
    nominalVolts: r.driveMotor.nominalVolts,
    stallTorqueNewtonMeters: r.driveMotor.stallTorqueNewtonMeters,
    stallCurrentAmps: r.driveMotor.stallCurrentAmps,
    freeSpeedRadPerSec: r.driveMotor.freeSpeedRadPerSec,
  };
  return m;
}

// ------------------------------------------------------------------------------ helpers

export function asObject(o: JsonValue | undefined, what: string): JsonObject {
  if (o === null || o === undefined || typeof o !== "object" || Array.isArray(o)) {
    throw new RangeError(`${what} must be a JSON object`);
  }
  return o;
}

function list(m: JsonObject, key: string): JsonValue[] {
  const v = m[key];
  if (v === undefined || v === null) return [];
  if (!Array.isArray(v)) throw new RangeError(`'${key}' must be a list`);
  return v;
}

function num(m: JsonObject, key: string, fallback: number): number {
  const v = m[key];
  if (v === undefined || v === null) return fallback;
  if (typeof v !== "number") throw new RangeError(`'${key}' must be a number`);
  return v;
}

function required(m: JsonObject, key: string): number {
  const v = num(m, key, Number.NaN);
  if (Number.isNaN(v)) throw new RangeError(`missing '${key}'`);
  return v;
}
