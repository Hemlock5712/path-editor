import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { exact, fixed5, writeJson } from "../src/json.ts";
import { eventMeters, Path } from "../src/path.ts";
import { pathToJson, readPath, robotFromJson, robotToJson, writePath } from "../src/pathFile.ts";
import { DEFAULT_ROBOT } from "../src/robot.ts";
import { eventAtMeters, eventAtWaypoint, makePathSpec, makeWaypoint } from "../src/spec.ts";
import { allSamples, COUNT, SEED } from "./pathFileSamples.ts";

const ROOT = join(import.meta.dirname, "..", "..", "..");
const RESOURCES = join(ROOT, "lib", "src", "test", "resources");
const read = (file: string) => readFileSync(file, "utf8").replace(/\r\n/g, "\n");
const example = () => read(join(RESOURCES, "example.path.json"));
const exampleV1 = () => read(join(RESOURCES, "example.v1.path.json"));

describe("fixed5", () => {
  it("rounds and trims like the Java writer", () => {
    const cases: [number, string][] = [
      [0, "0"],
      [-0, "0"],
      [4e-6, "0"],
      [6e-6, "0.00001"],
      [-1e-5, "-0.00001"],
      [1.5, "1.5"],
      [-2.000001, "-2"],
      [Math.PI, "3.14159"],
      [123456.789012, "123456.78901"],
      [0.1 + 0.2 - 0.2, "0.1"],
    ];
    for (const [x, text] of cases) expect(fixed5(x)).toBe(text);
    expect(() => fixed5(Number.NaN)).toThrow();
  });

  it("exact round-trips", () => {
    for (const x of [0.1, 1 / 3, -2.5e-12, 1e300, 42, Math.PI]) expect(Number(exact(x))).toBe(x);
    expect(exact(-0)).toBe("0");
  });
});

describe("writeJson", () => {
  it("puts plain containers inline and spreads the rest", () => {
    const text = writeJson({ n: 3, list: [{ x: 1.25, name: "a\nb" }, []], nums: [1, 2], empty: {} }, fixed5);
    expect(text).toBe(
      '{\n  "n": 3,\n  "list": [\n    { "x": 1.25, "name": "a\\nb" },\n    []\n  ],\n  "nums": [1, 2],\n  "empty": {}\n}\n',
    );
  });
});

describe("path files", () => {
  it("writes the Java-written example back character for character", () => {
    expect(writePath(readPath(example()))).toBe(example());
  });

  it("reads the example's contents", () => {
    const spec = readPath(example());
    expect(spec.waypoints).toHaveLength(10);
    expect(spec.waypoints[9]!.tangentRadians).toBeCloseTo(Math.PI, 15);
    expect(Number.isNaN(spec.waypoints[1]!.tangentRadians)).toBe(true);
    expect(spec.rotationTargets[3]!.meters).toBe(15);
    expect(spec.facePointZones[0]!.point).toEqual({ x: 4, y: 0 });
    expect(spec.constraintZones[0]!.maxMetersPerSecondSquared).toBe(Infinity);
    expect(spec.endMetersPerSecond).toBe(0.5);
    expect(Path.build(spec, DEFAULT_ROBOT).totalTimeSeconds).toBeGreaterThan(5);
  });

  it("defaults optional keys, ignores unknown ones and leaves them out when writing", () => {
    const spec = readPath(
      '{ "schema": 1, "waypoints": [ { "x": 0, "y": 0 }, { "x": 2, "y": 1, "tangentScale": 1.3 } ], "editorOnly": 1 }',
    );
    expect(spec.maxMetersPerSecond).toBe(Infinity);
    expect(spec.waypoints[1]!.tangentScale).toBe(1.3);
    const written = writePath(spec);
    expect(written).not.toContain("maxMetersPerSecond");
    expect(written).not.toContain("editorOnly");
    expect(written).toContain('"tangentScale": 1.3');
  });

  it("writes five decimals", () => {
    const spec = makePathSpec({ waypoints: [makeWaypoint(1 / 3, -2 / 3), makeWaypoint(1e-7, 123.456789012)] });
    const written = writePath(spec);
    expect(written).toContain('{ "x": 0.33333, "y": -0.66667 }');
    expect(written).toContain('{ "x": 0, "y": 123.45679 }');
    expect(Object.keys(pathToJson(spec))).toEqual([
      "schema",
      "waypoints",
      "rotationTargets",
      "facePointZones",
      "constraintZones",
      "events",
      "startMetersPerSecond",
      "endMetersPerSecond",
    ]);
  });

  it("rejects a wrong schema and missing fields", () => {
    expect(() => readPath('{ "waypoints": [] }')).toThrow(/schema/);
    expect(() => readPath('{ "schema": 3, "waypoints": [] }')).toThrow(/schema 3/);
    expect(() => readPath('{ "schema": 1.5, "waypoints": [] }')).toThrow(/schema 1.5/);
    expect(() => readPath('{ "schema": 1, "waypoints": [ { "x": 0 }, { "x": 1, "y": 0 } ] }')).toThrow(/'y'/);
    expect(() => readPath('{ "schema": 1, "waypoints": [ { "x": "0", "y": 0 } ] }')).toThrow(/number/);
    expect(() => readPath("[1, 2]")).toThrow(/object/);
  });

  it("round-trips a robot config", () => {
    const r = { ...DEFAULT_ROBOT, maxWheelMetersPerSecond: 4.2, frictionCoefficient: 0.9 };
    expect(robotFromJson(JSON.parse(writeJson(robotToJson(r), exact)))).toEqual(r);
    expect(robotFromJson({})).toEqual(DEFAULT_ROBOT);
    expect(robotFromJson({ massKg: 50, motor: { stallCurrentAmps: 400 } }).driveMotor.stallTorqueNewtonMeters).toBe(9.362);
  });
});

describe("events (schema 2)", () => {
  it("still reads schema 1 files, and writes the current schema", () => {
    const v1 = readPath(exampleV1());
    const v2 = readPath(example());
    expect(v1.events).toEqual([]);
    expect(v1.waypoints).toEqual(v2.waypoints);
    const written = writePath(v1);
    expect(written.startsWith('{\n  "schema": 2,')).toBe(true);
    expect(written).toContain('"events": []');
  });

  it("reads events in file order, with positions along the path", () => {
    const spec = readPath(example());
    expect(spec.events).toHaveLength(4);
    expect(spec.events[0]).toEqual(eventAtWaypoint("intakeDown", 0.5));
    expect(spec.events[1]).toEqual(eventAtMeters("spinUp", 6.25));
    expect(spec.events[3]!.name).toBe('score "L4"');
    const path = Path.build(spec, DEFAULT_ROBOT);
    expect(eventMeters(spec.events[0]!, path.geometry)).toBe(path.geometry.sAtWaypoint(0.5));
    expect(eventMeters(spec.events[1]!, path.geometry)).toBe(6.25);
  });

  it("rejects bad events like the Java reader", () => {
    const head = '{ "schema": 2, "waypoints": [ { "x": 0, "y": 0 }, { "x": 1, "y": 0 } ], "events": ';
    expect(() => readPath(head + '[ { "atMeters": 1 } ] }')).toThrow(/name/);
    expect(() => readPath(head + '[ { "name": 3, "atMeters": 1 } ] }')).toThrow(/name/);
    expect(() => readPath(head + '[ { "name": " ", "atMeters": 1 } ] }')).toThrow(/name/);
    expect(() => readPath(head + '[ { "name": "a" } ] }')).toThrow(/exactly one/);
    expect(() => readPath(head + '[ { "name": "a", "atMeters": 1, "atWaypoint": 0 } ] }')).toThrow(/exactly one/);
    expect(() => readPath(head + '{ "name": "a" } }')).toThrow(/list/);
  });

  // The Java <-> TS round trip: Java wrote these files from PathFileSamples; the TS port builds the
  // same specs from the same generator and must print the same characters, and reading any of them
  // and writing it again must give the file back.
  const fixture = JSON.parse(read(join(ROOT, "fixtures", "files", "path_files.json"))) as {
    seed: number;
    files: { text: string }[];
  };

  it("has the Java-written sample files", () => {
    expect(fixture.seed).toBe(SEED);
    expect(fixture.files).toHaveLength(COUNT);
  });

  it("prints the same text as Java for the same specs", () => {
    const specs = allSamples();
    let events = 0;
    specs.forEach((spec, i) => {
      expect(writePath(spec), `sample ${i}`).toBe(fixture.files[i]!.text);
      events += spec.events.length;
    });
    expect(events).toBeGreaterThan(20);
  });

  it("reads each Java-written file and writes it back byte for byte", () => {
    for (const { text } of fixture.files) expect(writePath(readPath(text))).toBe(text);
  });
});
