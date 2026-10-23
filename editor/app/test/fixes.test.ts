import { beforeEach, describe, expect, it } from "vitest";
import { makeConstraintZone, makeFacePointZone, makePathEvent, makePathSpec, makeRotationTarget, makeWaypoint } from "../../core/src/index.ts";
import type { Check } from "../src/compute/analysis.ts";
import { emptyProject, makeEditorPath } from "../src/model/project.ts";
import { activePath, editor, loadProject } from "../src/state/editor.ts";
import { fixesFor, splitZone } from "../src/state/fixes.ts";

const spec = makePathSpec({
  waypoints: [makeWaypoint(-6, -2, Number.NaN, 1, 0), makeWaypoint(-3, -2), makeWaypoint(-1, -1, Number.NaN, 1, 0)],
  events: [makePathEvent("late", 5, Number.NaN)],
  rotationTargets: [makeRotationTarget(Number.NaN, 99, 1)],
  facePointZones: [makeFacePointZone(1, 3, { x: 0, y: 0 }, 0.5)],
  constraintZones: [makeConstraintZone(-1, 1.5, 2, Infinity)],
  maxMetersPerSecond: 4,
});

const check = (c: Partial<Check> & Pick<Check, "kind">): Check => ({ severity: "warning", message: "", sFrom: 1, sTo: 2, ...c });

beforeEach(() => loadProject({ ...emptyProject(), paths: [makeEditorPath("P", spec, 0)] }));

describe("fix-its", () => {
  it("offer Route around and Optimise for a hit", () => {
    expect(fixesFor(check({ kind: "bumper", severity: "error" })).map((f) => f.label)).toEqual(["Route around", "Optimise"]);
  });

  it("slow the robot for a tight bend with a constraint zone at its cornering speed", () => {
    const fixes = fixesFor(check({ kind: "curvature", capMetersPerSecond: 1.234, sFrom: 2, sTo: 3 }));
    expect(fixes.map((f) => f.label)).toEqual(["Slow to 1.2 m/s here", "Optimise"]);
    fixes[0]!.run();
    const z = activePath().spec.constraintZones.at(-1)!;
    expect(z.maxMetersPerSecond).toBe(1.2);
    expect(z.toWaypoint).toBeGreaterThan(z.fromWaypoint);
  });

  it("offer nothing for a hairpin note", () => {
    expect(fixesFor(check({ kind: "curvature", severity: "info", capMetersPerSecond: 0.7, sFrom: 2, sTo: 3 }))).toEqual([]);
  });

  it("lengthen a face-point zone's blend for a turn too sharp for its distance", () => {
    const fixes = fixesFor(check({ kind: "heading", sFrom: 3, sTo: 4 }));
    expect(fixes[0]!.label).toMatch(/^Lengthen the blend to [\d.]+ m$/);
    fixes[0]!.run();
    expect(activePath().spec.facePointZones[0]!.blendMeters).toBeGreaterThan(0.5);
  });

  it("snap events and headings back onto the path, and clip zones to it", () => {
    fixesFor(check({ kind: "event", severity: "error", ref: { kind: "event", index: 0 } }))[0]!.run();
    expect(activePath().spec.events[0]!.waypoint).toBe(2);
    fixesFor(check({ kind: "position", ref: { kind: "rotation", index: 0 } }))[0]!.run();
    expect(activePath().spec.rotationTargets[0]!.meters).toBeLessThan(10);
    fixesFor(check({ kind: "position", ref: { kind: "face", index: 0 } }))[0]!.run();
    expect(activePath().spec.facePointZones[0]!.toWaypoint).toBe(2);
    fixesFor(check({ kind: "position", ref: { kind: "constraint", index: 0 } }))[0]!.run();
    expect(activePath().spec.constraintZones[0]!.fromWaypoint).toBe(0);
  });

  it("set a start or end speed the plan can reach", () => {
    const [f] = fixesFor(check({ kind: "speed", end: "end", capMetersPerSecond: 1.237 }));
    expect(f!.label).toBe("End at 1.23 m/s");
    f!.run();
    expect(activePath().spec.endMetersPerSecond).toBe(1.23);
  });

  it("split a zone at its middle", () => {
    splitZone("face", 0);
    expect(activePath().spec.facePointZones.map((z) => [z.fromWaypoint, z.toWaypoint])).toEqual([
      [1, 2],
      [2, 3],
    ]);
    expect(editor.get().canUndo).toBe(true);
  });
});
