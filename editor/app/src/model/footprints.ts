// Project edits for the robot's footprints (robot.json's "footprints"): add, change, rename and delete. A rename
// carries every event and path start that used the old name with it, so nothing is left pointing at nothing.

import { BUMPERS, makePathEvent, type FootprintExtension } from "../../../core/src/index.ts";
import { withSpec } from "./edits.ts";
import type { Project } from "./project.ts";

/** A name no footprint has yet: intakeOut, then intakeOut2, ... */
export function uniqueFootprintName(project: Project, base = "intakeOut"): string {
  const taken = new Set([BUMPERS, ...project.robot.footprints.map((f) => f.name)]);
  if (!taken.has(base)) return base;
  for (let k = 2; ; k++) if (!taken.has(`${base}${k}`)) return `${base}${k}`;
}

/**
 * A new footprint's reach past the front bumper: a 0.3 m extension limit, which the rules measure from the frame
 * perimeter, less a typical bumper's depth (about 8.5 cm), since a footprint is measured from the bumpers.
 */
export const DEFAULT_EXTENSION_METERS = 0.22;

/** A new footprint reaching DEFAULT_EXTENSION_METERS past the front bumper (an over-the-bumper intake). */
export function addFootprint(project: Project): Project {
  const f: FootprintExtension = { name: uniqueFootprintName(project), frontMeters: DEFAULT_EXTENSION_METERS, backMeters: 0, leftMeters: 0, rightMeters: 0, retractSeconds: 0 };
  return { ...project, robot: { ...project.robot, footprints: [...project.robot.footprints, f] } };
}

/** Footprint i's sizes changed (not its name: renameFootprint). Negative or non-finite numbers are refused. */
export function updateFootprint(project: Project, i: number, change: Partial<Omit<FootprintExtension, "name">>): Project {
  for (const v of Object.values(change)) if (!(v >= 0) || !Number.isFinite(v)) throw new RangeError("footprint sizes and times must be >= 0");
  return { ...project, robot: { ...project.robot, footprints: project.robot.footprints.map((f, k) => (k === i ? { ...f, ...change } : f)) } };
}

/** Why a footprint can't be called this (null = it can). */
export function footprintNameProblem(project: Project, i: number, name: string): string | null {
  if (name.trim() === "") return "A footprint needs a name";
  if (name === BUMPERS) return '"bumpers" is the bumpers alone';
  if (project.robot.footprints.some((f, k) => k !== i && f.name === name)) return "Another footprint has this name";
  return null;
}

/** Renames footprint i, and every event and path start that used its old name. */
export function renameFootprint(project: Project, i: number, name: string): Project {
  const problem = footprintNameProblem(project, i, name);
  if (problem) throw new RangeError(problem);
  const old = project.robot.footprints[i]!.name;
  return {
    ...project,
    robot: { ...project.robot, footprints: project.robot.footprints.map((f, k) => (k === i ? { ...f, name } : f)) },
    paths: project.paths.map((p) => {
      const uses = p.spec.startFootprint === old || p.spec.events.some((e) => e.footprint === old);
      if (!uses) return p;
      return withSpec(p, {
        startFootprint: p.spec.startFootprint === old ? name : p.spec.startFootprint,
        events: p.spec.events.map((e) => (e.footprint === old ? makePathEvent(e.name, e.waypoint, e.meters, name) : e)),
      });
    }),
  };
}

/** How many events and path starts use footprint name, across the project. */
export function footprintUses(project: Project, name: string): number {
  let n = 0;
  for (const p of project.paths) {
    if (p.spec.startFootprint === name) n++;
    for (const e of p.spec.events) if (e.footprint === name) n++;
  }
  return n;
}

/** Deletes footprint i. Events that switched to it are left as they are: the checks list them, with fixes. */
export function deleteFootprint(project: Project, i: number): Project {
  return { ...project, robot: { ...project.robot, footprints: project.robot.footprints.filter((_, k) => k !== i) } };
}
