// The commands that change a Project. Each replaces one part (a path, the field, the robot, the
// named points, the settings) or adds or removes a path, so undoing one edit never touches another.

import type { EditorPath, FieldDoc, NamedPoint, Project, ProjectSettings, RobotSettings } from "../model/project.ts";
import { composite, type Command } from "./history.ts";

export type ProjectCommand = Command<Project>;

function setPath(doc: Project, path: EditorPath): Project {
  return { ...doc, paths: doc.paths.map((p) => (p.id === path.id ? path : p)) };
}

export function replacePath(label: string, before: EditorPath, after: EditorPath): ProjectCommand {
  return { label, apply: (doc) => setPath(doc, after), revert: (doc) => setPath(doc, before) };
}

export function insertPath(label: string, path: EditorPath, index: number): ProjectCommand {
  return {
    label,
    apply: (doc) => {
      const paths = doc.paths.slice();
      paths.splice(Math.min(index, paths.length), 0, path);
      return { ...doc, paths };
    },
    revert: (doc) => ({ ...doc, paths: doc.paths.filter((p) => p.id !== path.id) }),
  };
}

export function deletePath(label: string, path: EditorPath, index: number): ProjectCommand {
  const insert = insertPath(label, path, index);
  return { label, apply: insert.revert, revert: insert.apply };
}

function replacePart<K extends "field" | "robot" | "namedPoints" | "settings">(
  key: K,
  label: string,
  before: Project[K],
  after: Project[K],
): ProjectCommand {
  return { label, apply: (doc) => ({ ...doc, [key]: after }), revert: (doc) => ({ ...doc, [key]: before }) };
}

export const replaceField = (label: string, before: FieldDoc, after: FieldDoc) => replacePart("field", label, before, after);
export const replaceRobot = (label: string, before: RobotSettings, after: RobotSettings) =>
  replacePart("robot", label, before, after);
export const replaceNamedPoints = (label: string, before: readonly NamedPoint[], after: readonly NamedPoint[]) =>
  replacePart("namedPoints", label, before, after);
export const replaceSettings = (label: string, before: ProjectSettings, after: ProjectSettings) =>
  replacePart("settings", label, before, after);

/** Replaces the whole path list (reordering, or anything diff() can't express per path). */
function replacePaths(label: string, before: readonly EditorPath[], after: readonly EditorPath[]): ProjectCommand {
  return { label, apply: (doc) => ({ ...doc, paths: after }), revert: (doc) => ({ ...doc, paths: before }) };
}

/**
 * The commands that turn `before` into `after`, one per changed part: how a drag (previewed live)
 * becomes a single undo step when it ends. Null when nothing changed.
 */
export function diff(label: string, before: Project, after: Project): ProjectCommand | null {
  const commands: ProjectCommand[] = [];
  const sameIds =
    before.paths.length === after.paths.length && before.paths.every((p, i) => p.id === after.paths[i]!.id);
  if (sameIds) {
    after.paths.forEach((p, i) => {
      if (p !== before.paths[i]) commands.push(replacePath(label, before.paths[i]!, p));
    });
  } else {
    commands.push(replacePaths(label, before.paths, after.paths));
  }
  if (before.field !== after.field) commands.push(replaceField(label, before.field, after.field));
  if (before.robot !== after.robot) commands.push(replaceRobot(label, before.robot, after.robot));
  if (before.namedPoints !== after.namedPoints) {
    commands.push(replaceNamedPoints(label, before.namedPoints, after.namedPoints));
  }
  if (before.settings !== after.settings) commands.push(replaceSettings(label, before.settings, after.settings));
  if (commands.length === 0) return null;
  return commands.length === 1 ? commands[0]! : composite(label, commands);
}
