import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { DEFAULT_ROBOT, eventAtMeters, readPath, writePath } from "../../core/src/index.ts";
import { MemoryFiles } from "../src/io/files.ts";
import { constantName, enumPath, pathNameEnum } from "../src/io/javaEnum.ts";
import {
  FIELD_FILE,
  fromFiles,
  loadFrom,
  parseField,
  parseRobot,
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
import { DEFAULT_ROBOT_SETTINGS, emptyProject, makeEditorPath, type Project } from "../src/model/project.ts";
import { DEFAULT_VIEW } from "../src/state/editor.ts";

const ROOT = join(import.meta.dirname, "..", "..", "..");
const read = (...p: string[]) => readFileSync(join(ROOT, ...p), "utf8").replace(/\r\n/g, "\n");
const EXAMPLE = read("lib", "src", "test", "resources", "example.path.json");
const BENCH_FIELD = read("bench", "fields", "obstacles_v1.json");

function sampleProject(): Project {
  const base = emptyProject();
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
    expect(files.get(`${PATHS_DIR}/Path1.path.json`)).toBe(writePath(emptyProject().paths[0]!.spec));
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

  it("leaves field.json and robot.json alone until they exist or are edited", () => {
    const files = toFiles(emptyProject(), DEFAULT_VIEW);
    expect(files.has(FIELD_FILE)).toBe(false);
    expect(files.has(ROBOT_FILE)).toBe(false);
    expect(files.has(SIDECAR_FILE)).toBe(true);
    expect(files.has(enumPath("frc.robot"))).toBe(true);
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

  it("keeps the tab order from the sidecar and adds new files after", () => {
    const files = toFiles(sampleProject(), DEFAULT_VIEW);
    files.set(`${PATHS_DIR}/Aaa.path.json`, writePath(emptyProject().paths[0]!.spec));
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
    expect(await files.read(enumPath("frc.robot"))).toContain('RENAMED("Renamed")');
  });

  it("an event added in the editor survives the round trip", async () => {
    const files = new MemoryFiles("robot");
    const project = emptyProject();
    const path = project.paths[0]!;
    const withEvent = { ...project, paths: [{ ...path, spec: { ...path.spec, events: [eventAtMeters("intakeDown", 1.23457)] } }] };
    await saveTo(files, withEvent, DEFAULT_VIEW);
    const text = (await files.read(`${PATHS_DIR}/Path1.path.json`))!;
    expect(text).toContain('{ "name": "intakeDown", "atMeters": 1.23457 }');
    const loaded = (await loadFrom(files))!;
    expect(loaded.project.paths[0]!.spec.events).toEqual([eventAtMeters("intakeDown", 1.23457)]);
  });
});

describe("PathName enum", () => {
  it("makes Java constants from file names", () => {
    expect(constantName("ToReef")).toBe("TO_REEF");
    expect(constantName("intake-2")).toBe("INTAKE_2");
    expect(constantName("HTTPLink")).toBe("HTTP_LINK");
    expect(constantName("leftL4Score")).toBe("LEFT_L4_SCORE");
    expect(enumPath("frc.robot.auto")).toBe("src/main/java/frc/robot/auto/PathName.java");
  });

  it("writes a sorted enum, separating constants that would clash", () => {
    const java = pathNameEnum("frc.robot", ["b-path", "A", "b_path"]);
    expect(java).toContain("package frc.robot;");
    expect(java).toContain('  A("A"),\n  B_PATH("b-path"),\n  B_PATH_2("b_path");');
    expect(pathNameEnum("frc.robot", [])).toContain("public enum PathName {\n;");
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
