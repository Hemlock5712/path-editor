// The editor's document. Path specs are the core's PathSpec as is, so what is saved is exactly
// writePath(spec). Everything else here (named points, links, colours, view) is editor-only and lives
// in the sidecar .pathproject.json, never in the path files.

import {
  DEFAULT_ROBOT,
  makePathSpec,
  makeWaypoint,
  type FieldFrame,
  type PathSpec,
  type RobotConfig,
} from "../../../core/src/index.ts";

/** A field point shared across paths. Moving it moves every waypoint linked to it. */
export interface NamedPoint {
  readonly id: string;
  readonly name: string;
  readonly x: number;
  readonly y: number;
}

export interface EditorPath {
  readonly id: string;
  /** File name without .path.json; also the PathName enum constant's source. */
  readonly name: string;
  readonly spec: PathSpec;
  /** Per waypoint: the NamedPoint id it follows, or null. Same length as spec.waypoints. */
  readonly links: readonly (string | null)[];
  readonly colour: string;
}

/** One obstacle polygon in field.json (metres, origin at the field centre). */
export interface Obstacle {
  readonly name: string;
  readonly polygon: readonly (readonly [number, number])[];
}

/**
 * deploy/paths/field.json in the format the Java FieldMap reads. `extra` keeps keys the editor
 * doesn't edit (description, routes, ...), so saving doesn't drop them.
 */
export interface FieldDoc {
  readonly lengthMeters: number;
  readonly widthMeters: number;
  readonly obstacles: readonly Obstacle[];
  readonly extra: Readonly<Record<string, unknown>>;
  readonly fieldExtra: Readonly<Record<string, unknown>>;
}

/** The robot file: RobotConfig plus the bumper size (read by PlannerConfig.fromRobotFile in Java). */
export interface RobotSettings {
  readonly config: RobotConfig;
  readonly bumperLengthMeters: number;
  readonly bumperWidthMeters: number;
  /** Keys the editor doesn't know, kept on save. */
  readonly extra: Readonly<Record<string, unknown>>;
}

export interface ProjectSettings {
  /** How red is made from blue: a half-turn about the centre or a mirror. */
  readonly redFrame: Exclude<FieldFrame, "BLUE">;
  /** Java package of the generated PathName enum, e.g. frc.robot. Empty = don't generate it. */
  readonly javaPackage: string;
}

export interface Project {
  readonly paths: readonly EditorPath[];
  readonly namedPoints: readonly NamedPoint[];
  readonly field: FieldDoc;
  readonly robot: RobotSettings;
  readonly settings: ProjectSettings;
}

export const DEFAULT_FIELD: FieldDoc = {
  lengthMeters: 16.54,
  widthMeters: 8.07,
  obstacles: [],
  extra: {},
  fieldExtra: {},
};

export const DEFAULT_ROBOT_SETTINGS: RobotSettings = {
  config: DEFAULT_ROBOT,
  bumperLengthMeters: 0.85,
  bumperWidthMeters: 0.85,
  extra: {},
};

export const DEFAULT_SETTINGS: ProjectSettings = { redFrame: "RED_HALF_TURN", javaPackage: "frc.robot" };

export const PATH_COLOURS = ["#38bdf8", "#f97316", "#a3e635", "#e879f9", "#facc15", "#2dd4bf", "#fb7185"];

let counter = 0;

/** A fresh id, unique in this session. */
export function newId(prefix: string): string {
  counter += 1;
  return `${prefix}-${Date.now().toString(36)}-${counter}`;
}

/** A path name that's a valid file name and Java identifier start, unique in the project. */
export function uniquePathName(project: Pick<Project, "paths">, base = "NewPath"): string {
  const taken = new Set(project.paths.map((p) => p.name));
  if (!taken.has(base)) return base;
  for (let i = 2; ; i++) if (!taken.has(`${base}${i}`)) return `${base}${i}`;
}

/** Path names become file names and enum constants: letters, digits, _ and -, starting with a letter. */
export function isValidPathName(name: string): boolean {
  return /^[A-Za-z][A-Za-z0-9_-]*$/.test(name);
}

export function makeEditorPath(name: string, spec: PathSpec, index: number, links?: readonly (string | null)[]): EditorPath {
  return {
    id: newId("path"),
    name,
    spec,
    links: links ?? spec.waypoints.map(() => null),
    colour: PATH_COLOURS[index % PATH_COLOURS.length]!,
  };
}

/** A new path: a short straight run out from the blue side, facing the field. */
export function starterSpec(): PathSpec {
  return makePathSpec({
    waypoints: [makeWaypoint(-6.5, -1.0), makeWaypoint(-4.0, -1.0)],
    maxMetersPerSecond: 4.0,
    maxMetersPerSecondSquared: 6.0,
  });
}

export function emptyProject(): Project {
  return {
    paths: [makeEditorPath("Path1", starterSpec(), 0)],
    namedPoints: [],
    field: DEFAULT_FIELD,
    robot: DEFAULT_ROBOT_SETTINGS,
    settings: DEFAULT_SETTINGS,
  };
}

/** Every event name used anywhere in the project, sorted, for the name picker. */
export function eventNames(project: Pick<Project, "paths">): string[] {
  const names = new Set<string>();
  for (const p of project.paths) for (const e of p.spec.events) names.add(e.name);
  return [...names].sort((a, b) => a.localeCompare(b));
}
