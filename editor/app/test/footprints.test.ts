import { describe, expect, it } from "vitest";
import {
  BUMPERS,
  DEFAULT_ROBOT,
  eventAtMeters,
  makePathEvent,
  makePathSpec,
  makeWaypoint,
  Path,
  robotFootprints,
  type FootprintExtension,
  type PathEvent,
} from "../../core/src/index.ts";
import { analyse, type Analysis, type AnalysisInput } from "../src/compute/analysis.ts";
import { FootprintTimeline, outlineAtS } from "../src/compute/footprints.ts";
import { hullsOf, shortfall } from "../src/compute/optimiser.ts";
import { routeAround } from "../src/compute/route.ts";
import { parseRobot, writeRobot } from "../src/io/robotField.ts";
import { addFootprint, footprintNameProblem, footprintUses, renameFootprint } from "../src/model/footprints.ts";
import { DEFAULT_FIELD, emptyProject, makeEditorPath } from "../src/model/project.ts";

// an intake out the robot's left side: 0.6 m past the bumpers, 1 s to come back in
const INTAKE: FootprintExtension = { name: "intakeOut", frontMeters: 0, backMeters: 0, leftMeters: 0.6, rightMeters: 0, retractSeconds: 1 };
// a pillar just left of a straight run along y = 0 (the robot faces +x, so its left is +y): 0.475 m from the
// bumpers' side, inside the intake's reach
const PILLAR = { name: "pillar", polygon: [[-0.3, 0.9], [0.3, 0.9], [0.3, 1.5], [-0.3, 1.5]] as const };

function spec(events: PathEvent[], startFootprint = BUMPERS) {
  return makePathSpec({
    waypoints: [makeWaypoint(-3, 0, 0, 1, 0), makeWaypoint(3, 0, 0, 1, 0)],
    maxMetersPerSecond: 4,
    maxMetersPerSecondSquared: 6,
    events,
    startFootprint,
  });
}

function input(events: PathEvent[], footprints: FootprintExtension[] = [INTAKE], startFootprint = BUMPERS): AnalysisInput {
  return {
    spec: spec(events, startFootprint),
    robot: DEFAULT_ROBOT,
    bumperLengthMeters: 0.85,
    bumperWidthMeters: 0.85,
    footprints,
    fieldLengthMeters: 16.54,
    fieldWidthMeters: 8.07,
    obstacles: [PILLAR],
  };
}

function ok(i: AnalysisInput): Analysis {
  const a = analyse(i);
  if (!a.ok) throw new Error(a.error);
  return a;
}

const hits = (a: Analysis) => a.checks.filter((c) => c.kind === "bumper" && c.severity === "error");
const deploy = (m: number) => makePathEvent("deployIntake", Number.NaN, m, "intakeOut");
const stow = (m: number) => makePathEvent("stowIntake", Number.NaN, m, BUMPERS);

describe("footprint timeline", () => {
  const path = Path.build(spec([deploy(1), eventAtMeters("shoot", 1.5), stow(4)]), DEFAULT_ROBOT);
  const t = new FootprintTimeline(path.spec, robotFootprints(0.85, 0.85, [INTAKE]), path);
  const at = (s: number) => t.at(s, path.sample(s).seconds);

  it("starts with the bumpers, grows at the event that deploys, and ignores events that name no footprint", () => {
    expect(t.bumpersOnly).toBe(false);
    expect(at(0.5).footprint.name).toBe(BUMPERS);
    expect(at(1.0).footprint).toMatchObject({ name: "intakeOut", leftMeters: 0.425 + 0.6, rightMeters: 0.425 });
    expect(at(2.0).footprint.name).toBe("intakeOut");
  });

  it("keeps a stowed mechanism for its retract time after the stow event, then drops it", () => {
    const stowT = path.sample(4).seconds;
    const s = (seconds: number) => path.sampleAtTime(seconds).s;
    const retracting = t.at(4.05, path.sample(4.05).seconds);
    expect(retracting.footprint.name).toBe(BUMPERS);
    expect(retracting.retracting).toEqual(["intakeOut"]);
    expect(retracting.footprint.leftMeters).toBeCloseTo(1.025, 12);
    if (stowT + 1 < path.totalTimeSeconds) {
      const after = t.at(s(stowT + 1.01), stowT + 1.01);
      expect(after.retracting).toEqual([]);
      expect(after.footprint.leftMeters).toBe(0.425);
    }
  });

  it("is the bumpers alone for a path with no footprint events", () => {
    const plain = Path.build(spec([eventAtMeters("shoot", 1)]), DEFAULT_ROBOT);
    expect(new FootprintTimeline(plain.spec, robotFootprints(0.85, 0.85, [INTAKE]), plain).bumpersOnly).toBe(true);
  });
});

describe("checks with a mechanism out", () => {
  it("the bumpers alone pass the pillar", () => {
    expect(hits(ok(input([])))).toEqual([]);
    expect(ok(input([])).outlines).toHaveLength(1);
  });

  it("an intake deployed before the pillar hits it, and the check says the intake did it", () => {
    const a = ok(input([deploy(1.5)]));
    const h = hits(a);
    expect(h).toHaveLength(1);
    expect(h[0]!.message).toMatch(/^With intakeOut, the robot hits pillar from 2\.\d\d to 3\.\d\d m\.$/);
    expect(outlineAtS(a.outlines, 3).footprint.name).toBe("intakeOut");
    expect(outlineAtS(a.outlines, 1).footprint.name).toBe(BUMPERS);
  });

  it("an intake deployed after the pillar, or stowed in time, is clear", () => {
    expect(hits(ok(input([deploy(4)])))).toEqual([]);
    expect(hits(ok(input([deploy(0.5), stow(1.5)], [{ ...INTAKE, retractSeconds: 0 }])))).toEqual([]);
  });

  it("stowed just before the pillar, it still hits while retracting", () => {
    const h = hits(ok(input([deploy(0.5), stow(2)])));
    expect(h).toHaveLength(1);
    expect(h[0]!.message).toMatch(/^With intakeOut still retracting, the robot hits pillar/);
  });

  it("a path that starts with the intake out is checked with it from the start", () => {
    expect(hits(ok(input([], [INTAKE], "intakeOut")))).toHaveLength(1);
    expect(hits(ok(input([stow(1)], [{ ...INTAKE, retractSeconds: 0 }], "intakeOut")))).toEqual([]);
  });

  it("flags a footprint the robot doesn't have, and checks with the bumpers there", () => {
    const a = ok(input([makePathEvent("deployArm", Number.NaN, 1, "armUp")], [INTAKE], "intake"));
    const f = a.checks.filter((c) => c.kind === "footprint");
    expect(f.map((c) => c.ref?.index)).toEqual([undefined, 0]);
    expect(f[1]!.message).toContain('Event "deployArm" switches to the footprint "armUp"');
    expect(f[1]!.message).toContain('"bumpers", "intakeOut"');
    expect(hits(a)).toEqual([]);
  });

  it("the optimiser's clearance measure sees the intake too", () => {
    const i = input([deploy(1.5)]);
    const hulls = hullsOf(i.obstacles);
    const out = shortfall(Path.build(i.spec, DEFAULT_ROBOT), i, hulls, 0.05);
    const plain = shortfall(Path.build(input([]).spec, DEFAULT_ROBOT), i, hulls, 0.05);
    expect(out.least).toBeLessThan(0);
    expect(out.worst).toBe("pillar");
    expect(plain.least).toBeGreaterThan(0.4);
  });

  it("Route around plans with the intake that's out over the stretch", () => {
    const i = input([deploy(0.5)]);
    const a = ok(i);
    const r = routeAround(makeEditorPath("P", i.spec, 0), a, { field: { ...DEFAULT_FIELD, obstacles: i.obstacles }, robot: DEFAULT_ROBOT, bumperLengthMeters: 0.85, bumperWidthMeters: 0.85 });
    if (!r.ok) throw new Error(r.reason);
    expect(r.added).toBeGreaterThan(0);
    // the route's corners keep the intake (1.025 m to the left) off the pillar, not just the bumpers
    const after = ok({ ...i, spec: r.path.spec });
    expect(after.checks.filter((c) => c.kind === "bumper" && c.message.includes("pillar") && c.severity === "error")).toEqual([]);
  });
});

describe("robot.json footprints", () => {
  it("round-trip, writing only the sides that reach past the bumpers, as the Java reads them", () => {
    const robot = parseRobot('{ "footprints": [ { "name": "intakeOut", "frontMeters": 0.3, "retractSeconds": 0.4 }, { "name": "arm", "backMeters": 0.1 } ] }');
    expect(robot.footprints).toEqual([
      { name: "intakeOut", frontMeters: 0.3, backMeters: 0, leftMeters: 0, rightMeters: 0, retractSeconds: 0.4 },
      { name: "arm", frontMeters: 0, backMeters: 0.1, leftMeters: 0, rightMeters: 0, retractSeconds: 0 },
    ]);
    const out = JSON.parse(writeRobot(robot));
    expect(out.footprints).toEqual([{ name: "intakeOut", frontMeters: 0.3, retractSeconds: 0.4 }, { name: "arm", backMeters: 0.1 }]);
    expect(writeRobot(parseRobot(writeRobot(robot)))).toBe(writeRobot(robot));
    expect(JSON.parse(writeRobot(parseRobot("{}"))).footprints).toBeUndefined();
  });

  it("refuses what the robot library would refuse at boot", () => {
    for (const bad of ['[{ "name": "bumpers" }]', '[{ "name": "a" }, { "name": "a" }]', '[{ "name": "a", "frontMeters": -0.1 }]', '[{ "frontMeters": 0.1 }]']) {
      expect(() => parseRobot(`{ "footprints": ${bad} }`), bad).toThrow(RangeError);
    }
  });
});

describe("footprint edits", () => {
  it("a rename carries every event and path start that used the old name", () => {
    let p = addFootprint(emptyProject());
    expect(p.robot.footprints.map((f) => f.name)).toEqual(["intakeOut"]);
    p = {
      ...p,
      paths: [
        makeEditorPath("A", spec([deploy(1), stow(2), eventAtMeters("shoot", 3)]), 0),
        makeEditorPath("B", spec([], "intakeOut"), 1),
        makeEditorPath("C", spec([]), 2),
      ],
    };
    expect(footprintUses(p, "intakeOut")).toBe(2);
    const r = renameFootprint(p, 0, "groundIntake");
    expect(r.robot.footprints[0]!.name).toBe("groundIntake");
    expect(r.paths[0]!.spec.events.map((e) => e.footprint)).toEqual(["groundIntake", BUMPERS, undefined]);
    expect(r.paths[1]!.spec.startFootprint).toBe("groundIntake");
    expect(r.paths[2]).toBe(p.paths[2]);
    expect(footprintNameProblem(r, 0, BUMPERS)).toMatch(/bumpers alone/);
    expect(footprintNameProblem(addFootprint(r), 1, "groundIntake")).toMatch(/Another/);
  });
});
