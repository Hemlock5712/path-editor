// field.json and robot.json: parsing and writing, keeping keys the editor doesn't edit. No editor state
// here, so the headless tools (tools/optimise.mjs) can load it under Node.

import { fixed5, robotFromJson, robotToJson, writeJson, type JsonValue } from "../../../core/src/index.ts";
import { DEFAULT_FIELD, DEFAULT_ROBOT_SETTINGS, type FieldDoc, type Obstacle, type RobotSettings } from "../model/project.ts";

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
  return {
    config: robotFromJson(root),
    bumperLengthMeters: num("bumperLengthMeters", DEFAULT_ROBOT_SETTINGS.bumperLengthMeters),
    bumperWidthMeters: num("bumperWidthMeters", DEFAULT_ROBOT_SETTINGS.bumperWidthMeters),
    extra: root,
  };
}

export function writeRobot(robot: RobotSettings): string {
  const root: JsonObject = { ...(robot.extra as JsonObject) };
  for (const key of OPTIONAL_LIMITS) delete root[key];
  Object.assign(root, robotToJson(robot.config));
  root.bumperLengthMeters = robot.bumperLengthMeters;
  root.bumperWidthMeters = robot.bumperWidthMeters;
  return writeJson(root, fixed5);
}
