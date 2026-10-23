// The robot's traction settings: the limit the Settings page shows, robot.json's keys, and the Sim
// preview floor's cap.

import { describe, expect, it } from "vitest";
import { DEFAULT_ROBOT, GRAVITY, tractionMetersPerSecondSquared } from "../../core/src/index.ts";
import { parseRobot, writeRobot } from "../src/io/robotField.ts";
import { DEFAULT_ROBOT_SETTINGS } from "../src/model/project.ts";
import { hasOldDefaultMu, simFloorCapMetersPerSecondSquared, tractionHeadline, tractionWorking, wheelSlipOnsetOnFloor } from "../src/model/traction.ts";

describe("traction", () => {
  it("says what limits the default robot: Spike Grip μ 1.5 with the centre of mass 0.2 m up, 90 % of grip or tipping as AccelerationLimiter", () => {
    expect(DEFAULT_ROBOT.frictionCoefficient).toBe(1.5);
    expect(DEFAULT_ROBOT.centerOfMassHeightMeters).toBe(0.2);
    expect(tractionHeadline(DEFAULT_ROBOT)).toBe("Traction limit 12.8 m/s², tipping limits first");
    expect(tractionWorking(DEFAULT_ROBOT)).toBe("12.8 m/s² = 90 % of the lower of grip μ·g = 14.7 m/s² and tipping g × tip 0.29 m ÷ height 0.20 m = 14.2 m/s².");
  });

  it("follows the numbers: a lower centre of mass or a slicker tread makes grip the limit", () => {
    const low = { ...DEFAULT_ROBOT, centerOfMassHeightMeters: 0 };
    expect(tractionHeadline(low)).toBe("Traction limit 13.2 m/s², grip limits first");
    expect(tractionWorking(low)).toContain("no tipping limit");
    const carpet = { ...DEFAULT_ROBOT, frictionCoefficient: 0.7 };
    expect(tractionHeadline(carpet)).toBe(`Traction limit ${(0.9 * 0.7 * GRAVITY).toFixed(1)} m/s², grip limits first`);
    // a wider base than the square default pushes tipping further off
    expect(tractionMetersPerSecondSquared({ ...DEFAULT_ROBOT, tipDistanceMeters: 0.4 })).toBeCloseTo(0.9 * 1.5 * GRAVITY, 9);
    // a softer share
    expect(tractionWorking({ ...DEFAULT_ROBOT, tractionShare: 0.8 })).toMatch(/^11\.4 m\/s² = 80 % of/);
  });

  it("spots the old 0.7 planning default", () => {
    expect(hasOldDefaultMu(DEFAULT_ROBOT)).toBe(false);
    expect(hasOldDefaultMu({ ...DEFAULT_ROBOT, frictionCoefficient: 0.7 })).toBe(true);
    expect(hasOldDefaultMu({ ...DEFAULT_ROBOT, frictionCoefficient: 0.75 })).toBe(false);
  });

  it("reads an old robot.json with the new defaults and keeps its μ; writes the tip distance only when set", () => {
    const old = parseRobot(JSON.stringify({ massKg: 55, moduleRadiusMeters: 0.41, frictionCoefficient: 0.7, bumperLengthMeters: 0.9, tractionRule: "equalPush" }));
    expect(old.config.frictionCoefficient).toBe(0.7);
    expect(old.config.centerOfMassHeightMeters).toBe(0.2);
    expect(old.config.tractionShare).toBe(0.9);
    expect(Number.isNaN(old.config.tipDistanceMeters)).toBe(true);
    const text = writeRobot(DEFAULT_ROBOT_SETTINGS);
    expect(JSON.parse(text)).toMatchObject({ frictionCoefficient: 1.5, centerOfMassHeightMeters: 0.2, tractionShare: 0.9 });
    expect(text).not.toContain("tipDistanceMeters");
    // an older file's traction rule is dropped on save: there is only the one
    expect(writeRobot(old)).not.toContain("tractionRule");
    const set = { ...DEFAULT_ROBOT_SETTINGS, config: { ...DEFAULT_ROBOT, tipDistanceMeters: 0.33 } };
    expect(parseRobot(writeRobot(set)).config.tipDistanceMeters).toBe(0.33);
  });

  it("caps the Sim preview floor at what the whole robot gets, and knows where the light wheels slip", () => {
    const tip = (GRAVITY * 0.29) / 0.2;
    expect(simFloorCapMetersPerSecondSquared(1.5, DEFAULT_ROBOT)).toBeCloseTo(tip, 9);
    expect(simFloorCapMetersPerSecondSquared(0.5, DEFAULT_ROBOT)).toBeCloseTo(0.5 * GRAVITY, 12);
    // μg / (1 + μh/tip): about half of either limit for Spike Grip at 0.2 m
    const slip = wheelSlipOnsetOnFloor(1.5, DEFAULT_ROBOT);
    expect(slip).toBeCloseTo((1.5 * GRAVITY) / (1 + (1.5 * 0.2) / 0.29), 9);
    expect(slip).toBeLessThan(Math.min(1.5 * GRAVITY, tip));
    expect(wheelSlipOnsetOnFloor(1.5, { ...DEFAULT_ROBOT, centerOfMassHeightMeters: 0 })).toBeCloseTo(1.5 * GRAVITY, 12);
  });
});
