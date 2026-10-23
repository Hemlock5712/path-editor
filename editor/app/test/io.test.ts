import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { DEFAULT_ROBOT, eventAtMeters, readPath, writePath } from "../../core/src/index.ts";
import { MemoryFiles } from "../src/io/files.ts";
import { constantName, javaPath, namesFile } from "../src/io/javaEnum.ts";
import {
  FIELD_FILE,
  fromFiles,
  loadFrom,
  parseField,
  parseRobot,
  PATH_SUFFIX,
  PATHS_DIR,
  ROBOT_FILE,
  saveTo,
  SIDECAR_FILE,
  toFiles,
  writeField,
  writeRobot,
} from "../src/io/projectIo.ts";
import { crc32, zip } from "../src/io/zip.ts";
import { addNamedPoint, linkToPoint } from "../src/model/namedPoints.ts";
import { withRobotLimits, withSpec } from "../src/model/edits.ts";
import { DEFAULT_ROBOT_SETTINGS, emptyProject, hasOldPathLimits, makeEditorPath, starterSpec, type Project } from "../src/model/project.ts";
import { DEFAULT_VIEW } from "../src/state/editor.ts";

/** A new project with a short drawn path (a new one in the app starts empty). */
const starterProject = () => ({ ...emptyProject(), paths: [makeEditorPath("Path1", starterSpec(), 0)] });

const ROOT = join(import.meta.dirname, "..", "..", "..");
const read = (...p: string[]) => readFileSync(join(ROOT, ...p), "utf8").replace(/\r\n/g, "\n");
const EXAMPLE = read("lib", "src", "test", "resources", "example.path.json");
const BENCH_FIELD = read("bench", "fields", "obstacles_v1.json");

function sampleProject(): Project {
  const base = starterProject();
  const auto = makeEditorPath("AutoRoute", readPath(EXAMPLE), 1);
  let project: Project = { ...base, paths: [base.paths[0]!, auto], field: parseField(BENCH_FIELD) };
  const [withPoint, point] = addNamedPoint(project, "Start", -5.2, -2.5);
  project = linkToPoint(withPoint, auto.id, 0, point.id);
  return { ...project, robot: { ...DEFAULT_ROBOT_SETTINGS, bumperLengthMeters: 0.9 } };
}

describe("project files", () => {
  it("writes each path with the core writer, byte for byte the Java format", () => {
    const files = toFiles(sampleProject(), DEFAULT_VIEW);
    expect(files.get(`${PATHS_DIR}/AutoRoute.path.json`)).toBe(EXAMPLE);
    expect(files.get(`${PATHS_DIR}/Path1.path.json`)).toBe(writePath(starterProject().paths[0]!.spec));
  });

  it("leaves out a path still being drawn (no end yet) until it has one", () => {
    const files = toFiles(emptyProject(), DEFAULT_VIEW);
    expect([...files.keys()].some((k) => k.endsWith(PATH_SUFFIX))).toBe(false);
    expect(JSON.parse(files.get(SIDECAR_FILE)!).paths).toEqual([]);
  });

  it("reads back what it writes, and writes it again identically", () => {
    const files = toFiles(sampleProject(), DEFAULT_VIEW);
    const loaded = fromFiles(files)!;
    expect(loaded.problems).toEqual([]);
    expect(loaded.project.paths.map((p) => p.name)).toEqual(["Path1", "AutoRoute"]);
    expect(loaded.project.paths[1]!.links[0]).toBe(loaded.project.namedPoints[0]!.id);
    expect(loaded.project.robot.bumperLengthMeters).toBe(0.9);
    const again = toFiles(loaded.project, loaded.view, { hadField: loaded.hadField, hadRobot: loaded.hadRobot });
    expect([...again.keys()].sort()).toEqual([...files.keys()].sort());
    for (const [path, text] of files) expect(again.get(path), path).toBe(text);
  });

  it("always writes field.json (the robot plans around what the editor did), and robot.json once it exists or is edited", () => {
    const files = toFiles(starterProject(), DEFAULT_VIEW);
    // a new project's field is the 2026 field, obstacles and all, in the format the Java FieldMap reads
    const field = parseField(files.get(FIELD_FILE)!);
    expect(field.obstacles.map((o) => o.name)).toEqual(expect.arrayContaining(["blue hub", "red hub", "blue tower", "red tower"]));
    expect(field.obstacles.find((o) => o.name === "blue hub")!.polygon).toEqual([
      [-4.2419, -0.5969],
      [-3.0481, -0.5969],
      [-3.0481, 0.5969],
      [-4.2419, 0.5969],
    ]);
    expect(files.has(ROBOT_FILE)).toBe(false);
    expect(files.has(SIDECAR_FILE)).toBe(true);
    expect(files.has(javaPath("frc.robot"))).toBe(true);
  });

  it("keeps pins and the drawing view settings in the sidecar", () => {
    const project = sampleProject();
    const auto = project.paths[1]!;
    const pinned = { ...project, paths: [project.paths[0]!, { ...auto, pinned: auto.pinned.map((_, i) => i === 1) }] };
    const loaded = fromFiles(toFiles(pinned, { ...DEFAULT_VIEW, shapeCurve: true, gridSnap: true }))!;
    expect(loaded.project.paths[1]!.pinned).toEqual(auto.pinned.map((_, i) => i === 1));
    expect(loaded.project.paths[0]!.pinned.every((p) => !p)).toBe(true);
    expect(loaded.view.shapeCurve).toBe(true);
    expect(loaded.view.gridSnap).toBe(true);
  });

  it("new paths plan to the robot's limits; the defaults save blank and read back blank", () => {
    const project = starterProject();
    expect(project.paths[0]!.spec.maxMetersPerSecond).toBe(Infinity);
    expect(project.paths[0]!.spec.maxMetersPerSecondSquared).toBe(Infinity);
    const files = toFiles(project, DEFAULT_VIEW);
    expect(files.get(`${PATHS_DIR}/Path1${PATH_SUFFIX}`)).not.toContain("maxMetersPerSecond");
    const loaded = fromFiles(files)!.project.settings.pathDefaults;
    expect(loaded.maxMetersPerSecond).toBe(Infinity);
    expect(loaded.maxMetersPerSecondSquared).toBe(Infinity);
  });

  it("clears the old fixed 4 m/s and 6 m/s² from the defaults and the paths that still have them, and keeps chosen ones", () => {
    const base = sampleProject();
    const old = { maxMetersPerSecond: 4, maxMetersPerSecondSquared: 6 };
    const project: Project = {
      ...base,
      paths: [withSpec(base.paths[0]!, old), withSpec(base.paths[1]!, { maxMetersPerSecond: 3, maxMetersPerSecondSquared: 6 })],
      settings: { ...base.settings, pathDefaults: { ...base.settings.pathDefaults, ...old } },
    };
    expect(hasOldPathLimits(project.settings.pathDefaults)).toBe(true);
    const fixed = withRobotLimits(project);
    expect(fixed.paths[0]!.spec.maxMetersPerSecondSquared).toBe(Infinity);
    expect(fixed.paths[0]!.spec.maxMetersPerSecond).toBe(Infinity);
    expect(fixed.paths[1]!.spec.maxMetersPerSecond).toBe(3);
    expect(fixed.paths[1]!.spec.maxMetersPerSecondSquared).toBe(6);
    expect(hasOldPathLimits(fixed.settings.pathDefaults)).toBe(false);
  });

  it("keeps the tab order from the sidecar and adds new files after", () => {
    const files = toFiles(sampleProject(), DEFAULT_VIEW);
    files.set(`${PATHS_DIR}/Aaa.path.json`, writePath(starterProject().paths[0]!.spec));
    expect(fromFiles(files)!.project.paths.map((p) => p.name)).toEqual(["Path1", "AutoRoute", "Aaa"]);
  });

  it("skips a bad path file and says why", () => {
    const files = toFiles(sampleProject(), DEFAULT_VIEW);
    files.set(`${PATHS_DIR}/Broken.path.json`, '{ "schema": 7 }');
    const loaded = fromFiles(files)!;
    expect(loaded.project.paths).toHaveLength(2);
    expect(loaded.problems[0]).toMatch(/Broken\.path\.json: path file schema 7/);
  });

  it("reads schema 1 files", () => {
    const v1 = read("lib", "src", "test", "resources", "example.v1.path.json");
    const loaded = fromFiles(new Map([[`${PATHS_DIR}/Old.path.json`, v1]]))!;
    expect(loaded.project.paths[0]!.spec.events).toEqual([]);
  });
});

describe("field.json", () => {
  it("reads the format the Java FieldMap reads, and keeps keys it doesn't edit", () => {
    const field = parseField(BENCH_FIELD);
    expect(field.lengthMeters).toBe(16.54);
    expect(field.obstacles.length).toBeGreaterThan(3);
    const text = writeField(field);
    const tree = JSON.parse(text);
    expect(Object.keys(tree)).toEqual(Object.keys(JSON.parse(BENCH_FIELD)));
    expect(tree.route).toEqual(JSON.parse(BENCH_FIELD).route);
    expect(tree.field.origin).toBe("centre");
    expect(tree.obstacles[0].polygon[0]).toEqual(field.obstacles[0]!.polygon[0]);
    expect(writeField(parseField(text))).toBe(text);
  });
});

describe("robot.json", () => {
  it("round-trips RobotConfig plus bumpers and keeps unknown keys", () => {
    const text = '{ "massKg": 52, "maxAngularRadPerSec": 9, "teamNote": "comp bot", "bumperWidthMeters": 0.8 }';
    const robot = parseRobot(text);
    expect(robot.config.massKg).toBe(52);
    expect(robot.config.maxAngularRadPerSec).toBe(9);
    expect(robot.bumperWidthMeters).toBe(0.8);
    expect(robot.bumperLengthMeters).toBe(0.85);
    const cleared = { ...robot, config: { ...robot.config, maxAngularRadPerSec: Infinity } };
    const out = JSON.parse(writeRobot(cleared));
    expect(out.teamNote).toBe("comp bot");
    expect(out.maxAngularRadPerSec).toBeUndefined();
    expect(out.motor.stallTorqueNewtonMeters).toBe(DEFAULT_ROBOT.driveMotor.stallTorqueNewtonMeters);
    expect(writeRobot(parseRobot(writeRobot(cleared)))).toBe(writeRobot(cleared));
  });
});

describe("saving to a folder", () => {
  it("writes changed files only and removes the files of deleted or renamed paths", async () => {
    const files = new MemoryFiles("robot");
    const project = sampleProject();
    const first = await saveTo(files, project, DEFAULT_VIEW);
    expect(first.written).toContain(`${PATHS_DIR}/AutoRoute.path.json`);
    const loaded = (await loadFrom(files))!;
    const second = await saveTo(files, loaded.project, loaded.view, { hadField: true, hadRobot: true });
    expect(second.written).toEqual([]);

    const renamed = { ...loaded.project, paths: [loaded.project.paths[0]!, { ...loaded.project.paths[1]!, name: "Renamed" }] };
    const known = [`${PATHS_DIR}/Path1.path.json`, `${PATHS_DIR}/AutoRoute.path.json`];
    const third = await saveTo(files, renamed, loaded.view, { hadField: true, hadRobot: true, knownPathFiles: known });
    expect(third.removed).toEqual([`${PATHS_DIR}/AutoRoute.path.json`]);
    expect(await files.list(PATHS_DIR)).toContain("Renamed.path.json");
    expect(await files.read(`${PATHS_DIR}/AutoRoute.path.json`)).toBeNull();
    expect(await files.read(javaPath("frc.robot"))).toContain('RENAMED("Renamed")');
  });

  it("an event added in the editor survives the round trip", async () => {
    const files = new MemoryFiles("robot");
    const project = starterProject();
    const path = project.paths[0]!;
    const withEvent = { ...project, paths: [{ ...path, spec: { ...path.spec, events: [eventAtMeters("intakeDown", 1.23457)] } }] };
    await saveTo(files, withEvent, DEFAULT_VIEW);
    const text = (await files.read(`${PATHS_DIR}/Path1.path.json`))!;
    expect(text).toContain('{ "name": "intakeDown", "atMeters": 1.23457 }');
    const loaded = (await loadFrom(files))!;
    expect(loaded.project.paths[0]!.spec.events).toEqual([eventAtMeters("intakeDown", 1.23457)]);
  });
});

describe("the generated names (PathEditorNames)", () => {
  it("makes Java constants from names", () => {
    expect(constantName("ToReef")).toBe("TO_REEF");
    expect(constantName("intake-2")).toBe("INTAKE_2");
    expect(constantName("HTTPLink")).toBe("HTTP_LINK");
    expect(constantName("leftL4Score")).toBe("LEFT_L4_SCORE");
    expect(constantName("Score L (mirror)")).toBe("SCORE_L_MIRROR");
    expect(constantName("2nd")).toBe("N_2ND");
    expect(javaPath("frc.robot.auto")).toBe("src/main/java/frc/robot/auto/PathEditorNames.java");
  });

  it("writes one class of three sorted enums, each constant carrying its exact name, clashes numbered", () => {
    const java = namesFile("frc.robot", { paths: ["b-path", "A", "b_path"], events: ["shoot", "intakeDown", "shoot"], points: ['Score "L"'] });
    expect(java).toContain("package frc.robot;");
    expect(java).toContain("public final class PathEditorNames {");
    expect(java).toContain('  public enum Paths implements PathId {\n    A("A"),\n    B_PATH("b-path"),\n    B_PATH_2("b_path");');
    expect(java).toContain('  public enum Events implements EventId {\n    INTAKE_DOWN("intakeDown"),\n    SHOOT("shoot");');
    expect(java).toContain('    SCORE_L("Score \\"L\\"");');
    expect(java).toContain("public String pointName()");
    expect(namesFile("frc.robot", { paths: [], events: [], points: [] })).toContain("public enum Events implements EventId {\n;");
  });

  it("are written on save, from every path's events and the named points, and the old PathName.java goes", async () => {
    const files = new MemoryFiles("robot");
    await files.write("src/main/java/frc/robot/PathName.java", "// Generated by the 5712 path editor from src/main/deploy/paths. Don't edit.\n");
    await files.write("src/main/java/frc/robot/Robot.java", "class Robot {}\n");
    const project = starterProject();
    const path = project.paths[0]!;
    const [withPoint] = addNamedPoint({ ...project, paths: [{ ...path, spec: { ...path.spec, events: [eventAtMeters("intakeDown", 1)] } }] }, "Score L", 1, 2);
    const r = await saveTo(files, withPoint, DEFAULT_VIEW);
    expect(r.removed).toEqual(["src/main/java/frc/robot/PathName.java"]);
    const java = (await files.read(javaPath("frc.robot")))!;
    expect(java).toContain('PATH1("Path1")');
    expect(java).toContain('INTAKE_DOWN("intakeDown")');
    expect(java).toContain('SCORE_L("Score L")');
    expect(await files.read("src/main/java/frc/robot/Robot.java")).toBe("class Robot {}\n");
  });
});

describe("zip", () => {
  it("computes the standard CRC-32 and writes a readable archive", () => {
    expect(crc32(new TextEncoder().encode("123456789"))).toBe(0xcbf43926);
    const bytes = zip(new Map([["a/b.txt", "hello"]]));
    const view = new DataView(bytes.buffer);
    expect(view.getUint32(0, true)).toBe(0x04034b50);
    expect(view.getUint32(bytes.length - 22, true)).toBe(0x06054b50);
    expect(new TextDecoder().decode(bytes.slice(30 + 7, 30 + 12))).toBe("hello");
  });
});
