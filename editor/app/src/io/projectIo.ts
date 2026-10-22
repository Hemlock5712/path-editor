// A project on disk, in a robot project folder:
//
//   src/main/deploy/paths/<name>.path.json   one per path, written by the core's writePath
//   src/main/deploy/paths/field.json         obstacles, in the format the Java FieldMap reads
//   src/main/deploy/paths/robot.json         RobotConfig (RobotFile) plus the bumper size
//   src/main/deploy/paths/.pathproject.json  editor-only: tab order, colours, named points, links, pins, view
//   src/main/java/<package>/PathEditorNames.java    the generated names (paths, events, named points)
//
// toFiles/fromFiles are pure (text in, text out); loadFrom/saveTo move them through ProjectFiles.

import {
  exact,
  readPath,
  writeJson,
  writePath,
  type JsonValue,
} from "../../../core/src/index.ts";
import {
  DEFAULT_FIELD,
  DEFAULT_ROBOT_SETTINGS,
  DEFAULT_SETTINGS,
  isDrawn,
  makeEditorPath,
  type NamedPoint,
  type Project,
  type ProjectSettings,
} from "../model/project.ts";
import type { Transform } from "../model/mirror.ts";
import { DEFAULT_VIEW, type ViewState } from "../state/editor.ts";
import type { ProjectFiles } from "./files.ts";
import { isValidPackage, javaPath, LEGACY_ENUM, namesFile } from "./javaEnum.ts";
import { parseField, parseRobot, writeField, writeRobot } from "./robotField.ts";

export { parseField, parseRobot, writeField, writeRobot };

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

/** The sim preview's drivetrain from the sidecar, each value checked (older files had none). */
function simPreviewOf(stored: unknown): ProjectSettings["simPreview"] {
  const saved = stored as Partial<Record<string, unknown>> | undefined;
  const d = DEFAULT_SETTINGS.simPreview;
  const num = (v: unknown, fallback: number) => (typeof v === "number" && Number.isFinite(v) && v >= 0 ? v : fallback);
  return {
    latencyMs: num(saved?.latencyMs, d.latencyMs),
    lagMs: num(saved?.lagMs, d.lagMs),
    gripMu: typeof saved?.gripMu === "number" && saved.gripMu > 0 ? saved.gripMu : null,
  };
}

// ------------------------------------------------------------------------------ sidecar

interface Sidecar {
  readonly schema: number;
  readonly paths: readonly {
    readonly name: string;
    readonly colour?: string;
    readonly links?: readonly (string | null)[];
    readonly pinned?: readonly boolean[];
    /** A linked copy: the source path's name and the transform. */
    readonly mirrorOf?: { readonly name: string; readonly transform: Transform } | null;
  }[];
  readonly namedPoints: readonly NamedPoint[];
  readonly settings: Partial<ProjectSettings>;
  readonly view: Partial<ViewState>;
}

export function writeSidecar(project: Project, view: ViewState): string {
  const root: JsonObject = {
    schema: SIDECAR_SCHEMA,
    paths: project.paths.filter((p) => isDrawn(p.spec)).map((p) => {
      const meta: JsonObject = { name: p.name, colour: p.colour, links: [...p.links], pinned: [...p.pinned] };
      const source = p.mirrorOf ? project.paths.find((q) => q.id === p.mirrorOf!.id) : undefined;
      if (source) meta.mirrorOf = { name: source.name, transform: p.mirrorOf!.transform };
      return meta;
    }),
    namedPoints: project.namedPoints.map((p) => {
      const meta: JsonObject = { id: p.id, name: p.name, x: p.x, y: p.y };
      if (p.heading !== undefined && !Number.isNaN(p.heading)) meta.heading = p.heading;
      if (p.twin) meta.twin = { id: p.twin.id, transform: p.twin.transform };
      return meta;
    }),
    settings: {
      redFrame: project.settings.redFrame,
      javaPackage: project.settings.javaPackage,
      pathDefaults: { ...project.settings.pathDefaults },
      simPreview: { latencyMs: project.settings.simPreview.latencyMs, lagMs: project.settings.simPreview.lagMs, gripMu: project.settings.simPreview.gripMu },
    },
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
  // a path still being drawn (no end yet) isn't a path file until it has one
  const paths = project.paths.filter((p) => isDrawn(p.spec));
  for (const p of paths) out.set(`${PATHS_DIR}/${p.name}${PATH_SUFFIX}`, writePath(p.spec));
  // always: the robot's FieldMap must load the obstacles the editor planned around (the 2026 field's by
  // default; a project's own field.json stays as it is)
  out.set(FIELD_FILE, writeField(project.field));
  if (options.hadRobot || project.robot !== DEFAULT_ROBOT_SETTINGS) out.set(ROBOT_FILE, writeRobot(project.robot));
  out.set(SIDECAR_FILE, writeSidecar(project, view));
  const pkg = project.settings.javaPackage;
  if (pkg !== "" && isValidPackage(pkg)) {
    out.set(
      javaPath(pkg),
      namesFile(pkg, {
        paths: paths.map((p) => p.name),
        events: paths.flatMap((p) => p.spec.events.map((e) => e.name)),
        points: project.namedPoints.map((p) => p.name),
      }),
    );
  }
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
  // linked copies name their source; ids are made fresh on every load
  const idOf = new Map(paths.map((p) => [p.name, p.id]));
  const mirrorOf = (name: string) => {
    const m = sidecar?.paths.find((q) => q.name === name)?.mirrorOf;
    const id = m ? idOf.get(m.name) : undefined;
    return m && id && id !== idOf.get(name) ? { id, transform: m.transform } : null;
  };
  const project: Project = {
    paths: paths.map((p) => ({ ...p, links: p.links.map((l) => (l !== null && pointIds.has(l) ? l : null)), mirrorOf: mirrorOf(p.name) })),
    namedPoints: (sidecar?.namedPoints ?? []).map((p) => ({
      ...p,
      heading: typeof p.heading === "number" ? p.heading : Number.NaN,
      twin: p.twin && pointIds.has(p.twin.id) ? p.twin : null,
    })),
    field,
    robot,
    settings: {
      ...DEFAULT_SETTINGS,
      ...(sidecar?.settings ?? {}),
      pathDefaults: { ...DEFAULT_SETTINGS.pathDefaults, ...(sidecar?.settings?.pathDefaults ?? {}) },
      simPreview: simPreviewOf(sidecar?.settings?.simPreview),
    },
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
  // the enum the editor generated before PathEditorNames replaced it (only if the editor wrote it)
  const pkg = project.settings.javaPackage;
  if (pkg !== "" && isValidPackage(pkg)) {
    const legacy = javaPath(pkg, LEGACY_ENUM);
    const text = await files.read(legacy).catch(() => undefined);
    if (text?.startsWith("// Generated by the 5712 path editor")) {
      await files.remove(legacy);
      removed.push(legacy);
    }
  }
  return { written, removed };
}
