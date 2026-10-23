// field.json and robot.json: parsing and writing, keeping keys the editor doesn't edit. No editor state
// here, so the headless tools (tools/optimise.mjs) can load it under Node.

import { fixed5, robotFromJson, robotToJson, writeJson, type JsonValue } from "../../../core/src/index.ts";
import {
  DEFAULT_FIELD,
  DEFAULT_ROBOT_SETTINGS,
  DEFAULT_SHARE,
  topSpeedMetersPerSecond,
  withTopSpeed,
  type FieldDoc,
  type Obstacle,
  type RobotSettings,
} from "../model/project.ts";

type JsonObject = { [key: string]: JsonValue };

// ------------------------------------------------------------------------------ field.json

export function parseField(text: string): FieldDoc {
  const root = JSON.parse(text) as JsonObject;
  const field = (root.field ?? {}) as JsonObject;
  const obstacles = Array.isArray(root.obstacles) ? root.obstacles : [];
  return {
    lengthMeters: typeof field.length_m === "number" ? field.length_m : DEFAULT_FIELD.lengthMeters,
    widthMeters: typeof field.width_m === "number" ? field.width_m : DEFAULT_FIELD.widthMeters,
    obstacles: obstacles.map((o, i): Obstacle => {
      const m = o as JsonObject;
      const polygon = Array.isArray(m.polygon) ? m.polygon : [];
      return {
        name: typeof m.name === "string" ? m.name : `obstacle ${i + 1}`,
        polygon: polygon.map((p) => {
          const xy = p as number[];
          if (!Array.isArray(xy) || typeof xy[0] !== "number" || typeof xy[1] !== "number") {
            throw new RangeError("polygon points must be [x, y]");
          }
          return [xy[0], xy[1]] as const;
        }),
      };
    }),
    extra: root,
    fieldExtra: field,
  };
}

/** The file text: unknown keys kept in their places, then field size and obstacles. */
export function writeField(doc: FieldDoc): string {
  const root: JsonObject = { ...(doc.extra as JsonObject) };
  root.field = { ...(doc.fieldExtra as JsonObject), length_m: doc.lengthMeters, width_m: doc.widthMeters };
  root.obstacles = doc.obstacles.map((o) => ({ name: o.name, polygon: o.polygon.map(([x, y]) => [x, y]) }));
  return writeJson(root, fixed5);
}

// ------------------------------------------------------------------------------ robot.json

const OPTIONAL_LIMITS = ["maxWheelMetersPerSecond", "maxAngularRadPerSec", "maxAngularRadPerSecSquared"];

export function parseRobot(text: string): RobotSettings {
  const root = JSON.parse(text) as JsonObject;
  const num = (key: string, fallback: number) => (typeof root[key] === "number" ? (root[key] as number) : fallback);
  const config = robotFromJson(root);
  const speedAt12VoltsMetersPerSecond = num("speedAt12VoltsMetersPerSecond", Number.NaN);
  // an older file without a share: a wheel-speed cap it set is kept as its share of the top speed
  const top = topSpeedMetersPerSecond({ config, speedAt12VoltsMetersPerSecond });
  const kept = Number.isFinite(config.maxWheelMetersPerSecond) ? Math.min(1, Math.max(0.05, config.maxWheelMetersPerSecond / top)) : DEFAULT_SHARE;
  return withTopSpeed({
    config,
    bumperLengthMeters: num("bumperLengthMeters", DEFAULT_ROBOT_SETTINGS.bumperLengthMeters),
    bumperWidthMeters: num("bumperWidthMeters", DEFAULT_ROBOT_SETTINGS.bumperWidthMeters),
    topSpeedShare: num("topSpeedShare", kept),
    speedAt12VoltsMetersPerSecond,
    extra: root,
  });
}

/** Keys robot.json's writer owns; any others are the team's, kept after them as they were. */
const OWN_KEYS = new Set([
  ...Object.keys(robotToJson(DEFAULT_ROBOT_SETTINGS.config)),
  ...OPTIONAL_LIMITS,
  "tipDistanceMeters",
  "bumperLengthMeters",
  "bumperWidthMeters",
  "topSpeedShare",
  "speedAt12VoltsMetersPerSecond",
  "tractionRule", // the old second traction rule: there is only the one now
]);

/** Always in the same order (RobotConfig's, then the bumpers and the editor's keys, then the team's). */
export function writeRobot(robot: RobotSettings): string {
  const root: JsonObject = { ...robotToJson(robot.config) };
  root.bumperLengthMeters = robot.bumperLengthMeters;
  root.bumperWidthMeters = robot.bumperWidthMeters;
  // editor-only (the robot reads maxWheelMetersPerSecond): what the top speed share is of
  root.topSpeedShare = robot.topSpeedShare;
  if (Number.isFinite(robot.speedAt12VoltsMetersPerSecond)) root.speedAt12VoltsMetersPerSecond = robot.speedAt12VoltsMetersPerSecond;
  for (const [key, value] of Object.entries(robot.extra as JsonObject)) if (!OWN_KEYS.has(key)) root[key] = value;
  return writeJson(root, fixed5);
}
