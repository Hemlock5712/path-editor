// The editor's document. Path specs are the core's PathSpec as is, so what is saved is exactly
// writePath(spec). Everything else here (named points, links, colours, view) is editor-only and lives
// in the sidecar .pathproject.json, never in the path files.

import {
  DEFAULT_ROBOT,
  makePathSpec,
  makeWaypoint,
  type FieldFrame,
  type HeadingMode,
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
  /** Per waypoint: pinned, so the optimiser leaves it where it is. Same length as spec.waypoints. */
  readonly pinned: readonly boolean[];
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

/** What a new path starts with. */
export interface PathDefaults {
  readonly maxMetersPerSecond: number;
  readonly maxMetersPerSecondSquared: number;
  readonly headingMode: HeadingMode;
}

/** The simulated drivetrain the Sim preview runs the follower on (editor-only). */
export interface SimPreviewSettings {
  /** From a command to the wheels starting to answer, ms. */
  readonly latencyMs: number;
  /** The drive's first-order lag (time constant), ms. */
  readonly lagMs: number;
  /** The floor's grip in the sim, μ (caps the simulated chassis's acceleration at μ·g); null: no cap. */
  readonly gripMu: number | null;
}

export interface ProjectSettings {
  /** How red is made from blue: a half-turn about the centre or a mirror. */
  readonly redFrame: Exclude<FieldFrame, "BLUE">;
  /** Java package of the generated PathName enum, e.g. frc.robot. Empty = don't generate it. */
  readonly javaPackage: string;
  readonly pathDefaults: PathDefaults;
  readonly simPreview: SimPreviewSettings;
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

export const DEFAULT_PATH_DEFAULTS: PathDefaults = { maxMetersPerSecond: 4.0, maxMetersPerSecondSquared: 6.0, headingMode: "hold" };

export const DEFAULT_SIM_PREVIEW: SimPreviewSettings = { latencyMs: 20, lagMs: 40, gripMu: null };

export const DEFAULT_SETTINGS: ProjectSettings = {
  redFrame: "RED_HALF_TURN",
  javaPackage: "frc.robot",
  pathDefaults: DEFAULT_PATH_DEFAULTS,
  simPreview: DEFAULT_SIM_PREVIEW,
};

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

export function makeEditorPath(
  name: string,
  spec: PathSpec,
  index: number,
  links?: readonly (string | null)[],
  pinned?: readonly boolean[],
): EditorPath {
  return {
    id: newId("path"),
    name,
    spec,
    links: links ?? spec.waypoints.map(() => null),
    pinned: pinned ?? spec.waypoints.map(() => false),
    colour: PATH_COLOURS[index % PATH_COLOURS.length]!,
  };
}

/** How far a new path's first stretch runs, metres. */
const STARTER_LENGTH = 2.5;

/**
 * A new path: a robot pose and a short straight run from it, holding that heading at both ends (the
 * first and last waypoints always have one). By default it starts on the blue side, facing the field.
 */
export function starterSpec(x = -6.5, y = -1.0, headingRadians = 0, defaults: PathDefaults = DEFAULT_PATH_DEFAULTS): PathSpec {
  const ex = Math.round((x + STARTER_LENGTH * Math.cos(headingRadians)) * 1e5) / 1e5;
  const ey = Math.round((y + STARTER_LENGTH * Math.sin(headingRadians)) * 1e5) / 1e5;
  return makePathSpec({
    waypoints: [makeWaypoint(x, y, Number.NaN, 1, headingRadians), makeWaypoint(ex, ey, Number.NaN, 1, headingRadians)],
    maxMetersPerSecond: defaults.maxMetersPerSecond,
    maxMetersPerSecondSquared: defaults.maxMetersPerSecondSquared,
    headingMode: defaults.headingMode,
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
