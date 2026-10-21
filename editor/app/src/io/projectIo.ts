// A project on disk, in a robot project folder:
//
//   src/main/deploy/paths/<name>.path.json   one per path, written by the core's writePath
//   src/main/deploy/paths/field.json         obstacles, in the format the Java FieldMap reads
//   src/main/deploy/paths/robot.json         RobotConfig (RobotFile) plus the bumper size
//   src/main/deploy/paths/.pathproject.json  editor-only: tab order, colours, named points, links, pins, view
//   src/main/java/<package>/PathName.java    the generated enum
//
// toFiles/fromFiles are pure (text in, text out); loadFrom/saveTo move them through ProjectFiles.

import {
  fixed5,
  exact,
  readPath,
  robotFromJson,
  robotToJson,
  writeJson,
  writePath,
  type JsonValue,
} from "../../../core/src/index.ts";
import {
  DEFAULT_FIELD,
  DEFAULT_ROBOT_SETTINGS,
  DEFAULT_SETTINGS,
  makeEditorPath,
  type FieldDoc,
  type NamedPoint,
  type Obstacle,
  type Project,
  type ProjectSettings,
  type RobotSettings,
} from "../model/project.ts";
import { DEFAULT_VIEW, type ViewState } from "../state/editor.ts";
import type { ProjectFiles } from "./files.ts";
import { enumPath, isValidPackage, pathNameEnum } from "./javaEnum.ts";

export const PATHS_DIR = "src/main/deploy/paths";
export const PATH_SUFFIX = ".path.json";
export const FIELD_FILE = `${PATHS_DIR}/field.json`;
export const ROBOT_FILE = `${PATHS_DIR}/robot.json`;
export const SIDECAR_FILE = `${PATHS_DIR}/.pathproject.json`;
export const SIDECAR_SCHEMA = 1;

type JsonObject = { [key: string]: JsonValue };

export interface LoadedProject {
  readonly project: Project;
  readonly view: ViewState;
  /** Files that couldn't be read, with why. The rest of the project still loads. */
  readonly problems: readonly string[];
  /** Whether field.json and robot.json were there (saving only writes them if so, or if edited). */
  readonly hadField: boolean;
  readonly hadRobot: boolean;
}

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

// ------------------------------------------------------------------------------ sidecar

interface Sidecar {
  readonly schema: number;
  readonly paths: readonly {
    readonly name: string;
    readonly colour?: string;
    readonly links?: readonly (string | null)[];
    readonly pinned?: readonly boolean[];
  }[];
  readonly namedPoints: readonly NamedPoint[];
  readonly settings: Partial<ProjectSettings>;
  readonly view: Partial<ViewState>;
}

export function writeSidecar(project: Project, view: ViewState): string {
  const root: JsonObject = {
    schema: SIDECAR_SCHEMA,
    paths: project.paths.map((p) => ({ name: p.name, colour: p.colour, links: [...p.links], pinned: [...p.pinned] })),
    namedPoints: project.namedPoints.map((p) => ({ id: p.id, name: p.name, x: p.x, y: p.y })),
    settings: { redFrame: project.settings.redFrame, javaPackage: project.settings.javaPackage },
    view: {
      alliance: view.alliance,
      zoom: view.zoom,
      centerX: view.centerX,
      centerY: view.centerY,
      footprintSpacing: view.footprintSpacing,
      shapeCurve: view.shapeCurve,
      gridSnap: view.gridSnap,
    },
  };
  return writeJson(root, exact);
}

function parseSidecar(text: string): Sidecar {
  const root = JSON.parse(text) as Partial<Sidecar>;
  if (root.schema !== SIDECAR_SCHEMA) throw new RangeError(`sidecar schema ${String(root.schema)}`);
  return {
    schema: SIDECAR_SCHEMA,
    paths: Array.isArray(root.paths) ? root.paths : [],
    namedPoints: Array.isArray(root.namedPoints) ? root.namedPoints : [],
    settings: root.settings ?? {},
    view: root.view ?? {},
  };
}

// ------------------------------------------------------------------------------ whole project

/** What to write for a project: relative path → text. */
export function toFiles(
  project: Project,
  view: ViewState,
  options: { hadField?: boolean; hadRobot?: boolean } = {},
): Map<string, string> {
  const out = new Map<string, string>();
  for (const p of project.paths) out.set(`${PATHS_DIR}/${p.name}${PATH_SUFFIX}`, writePath(p.spec));
  if (options.hadField || project.field !== DEFAULT_FIELD) out.set(FIELD_FILE, writeField(project.field));
  if (options.hadRobot || project.robot !== DEFAULT_ROBOT_SETTINGS) out.set(ROBOT_FILE, writeRobot(project.robot));
  out.set(SIDECAR_FILE, writeSidecar(project, view));
  const pkg = project.settings.javaPackage;
  if (pkg !== "" && isValidPackage(pkg)) out.set(enumPath(pkg), pathNameEnum(pkg, project.paths.map((p) => p.name)));
  return out;
}

/** Builds a project from file texts (relative path → text). Null if there are no path files. */
export function fromFiles(files: ReadonlyMap<string, string>): LoadedProject | null {
  const problems: string[] = [];
  const pathTexts = [...files.entries()]
    .filter(([p]) => p.startsWith(`${PATHS_DIR}/`) && p.endsWith(PATH_SUFFIX) && !p.slice(PATHS_DIR.length + 1).includes("/"))
    .map(([p, text]) => ({ name: p.slice(PATHS_DIR.length + 1, -PATH_SUFFIX.length), text }))
    .sort((a, b) => a.name.localeCompare(b.name));

  let sidecar: Sidecar | null = null;
  const sidecarText = files.get(SIDECAR_FILE);
  if (sidecarText !== undefined) {
    try {
      sidecar = parseSidecar(sidecarText);
    } catch (e) {
      problems.push(`.pathproject.json: ${(e as Error).message}`);
    }
  }

  // tab order from the sidecar, then any new files alphabetically
  const order = new Map((sidecar?.paths ?? []).map((p, i) => [p.name, i]));
  pathTexts.sort((a, b) => (order.get(a.name) ?? 1e9) - (order.get(b.name) ?? 1e9));

  const paths = [];
  for (const { name, text } of pathTexts) {
    try {
      const spec = readPath(text);
      const meta = sidecar?.paths.find((p) => p.name === name);
      const links = meta?.links?.length === spec.waypoints.length ? meta.links : undefined;
      const pinned = meta?.pinned?.length === spec.waypoints.length ? meta.pinned : undefined;
      const path = makeEditorPath(name, spec, paths.length, links, pinned);
      paths.push(meta?.colour ? { ...path, colour: meta.colour } : path);
    } catch (e) {
      problems.push(`${name}${PATH_SUFFIX}: ${(e as Error).message}`);
    }
  }
  if (paths.length === 0) return null;

  let field = DEFAULT_FIELD;
  const fieldText = files.get(FIELD_FILE);
  if (fieldText !== undefined) {
    try {
      field = parseField(fieldText);
    } catch (e) {
      problems.push(`field.json: ${(e as Error).message}`);
    }
  }
  let robot = DEFAULT_ROBOT_SETTINGS;
  const robotText = files.get(ROBOT_FILE);
  if (robotText !== undefined) {
    try {
      robot = parseRobot(robotText);
    } catch (e) {
      problems.push(`robot.json: ${(e as Error).message}`);
    }
  }

  const pointIds = new Set((sidecar?.namedPoints ?? []).map((p) => p.id));
  const project: Project = {
    paths: paths.map((p) => ({ ...p, links: p.links.map((l) => (l !== null && pointIds.has(l) ? l : null)) })),
    namedPoints: sidecar?.namedPoints ?? [],
    field,
    robot,
    settings: { ...DEFAULT_SETTINGS, ...(sidecar?.settings ?? {}) },
  };
  return {
    project,
    view: { ...DEFAULT_VIEW, ...(sidecar?.view ?? {}) },
    problems,
    hadField: fieldText !== undefined,
    hadRobot: robotText !== undefined,
  };
}

/** Reads every file fromFiles looks at. */
export async function loadFrom(files: ProjectFiles): Promise<LoadedProject | null> {
  const texts = new Map<string, string>();
  for (const name of await files.list(PATHS_DIR)) {
    const text = await files.read(`${PATHS_DIR}/${name}`);
    if (text !== null) texts.set(`${PATHS_DIR}/${name}`, text);
  }
  return fromFiles(texts);
}

export interface SaveResult {
  readonly written: readonly string[];
  readonly removed: readonly string[];
}

/**
 * Writes the project. Path files that this session loaded or saved but the project no longer has
 * (deleted or renamed tabs) are removed; nothing else in the folder is touched.
 */
export async function saveTo(
  files: ProjectFiles,
  project: Project,
  view: ViewState,
  options: { hadField?: boolean; hadRobot?: boolean; knownPathFiles?: Iterable<string> } = {},
): Promise<SaveResult> {
  const out = toFiles(project, view, options);
  const written: string[] = [];
  for (const [path, text] of out) {
    if ((await files.read(path)) === text) continue;
    await files.write(path, text);
    written.push(path);
  }
  const removed: string[] = [];
  for (const path of options.knownPathFiles ?? []) {
    if (!out.has(path) && path.endsWith(PATH_SUFFIX)) {
      await files.remove(path);
      removed.push(path);
    }
  }
  return { written, removed };
}
