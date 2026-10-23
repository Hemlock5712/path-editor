// Importing the robot's code (TunerConstants.java, AccelerationLimiter.java), and the top speed share it feeds.

import { readFileSync } from "node:fs";
import { join } from "node:path";
import { describe, expect, it } from "vitest";
import { tractionMetersPerSecondSquared, wheelSpeedCapMetersPerSecond } from "../../core/src/index.ts";
import { parseRobot, writeRobot } from "../src/io/robotField.ts";
import { parseAccelerationLimiter, parseRobotCode, parseTunerConstants } from "../src/io/robotCode.ts";
import { DEFAULT_ROBOT_SETTINGS, topSpeedMetersPerSecond, withTopSpeed } from "../src/model/project.ts";
import { importSummary, slipCurrentWarning, withRobotCode } from "../src/model/robotImport.ts";

const TUNER = readFileSync(join(import.meta.dirname, "fixtures", "TunerConstants.java"), "utf8");
const LIMITER = readFileSync(join(import.meta.dirname, "fixtures", "AccelerationLimiter.java"), "utf8");

describe("TunerConstants import", () => {
  it("reads the gearing, the wheel, the module positions and the speed at 12 V", () => {
    const t = parseTunerConstants(TUNER);
    expect(t.driveGearRatio).toBeCloseTo(7.363636363636365, 12);
    expect(t.wheelRadiusMeters).toBeCloseTo(2.167 * 0.0254, 12);
    // modules at (±10, ±10) in
    expect(t.moduleRadiusMeters).toBeCloseTo(Math.hypot(0.254, 0.254), 12);
    expect(t.tipDistanceMeters).toBeCloseTo(0.254, 12);
    expect(t.speedAt12VoltsMetersPerSecond).toBe(4.54);
    expect(t.slipCurrentAmps).toBe(120);
    expect(t.found).toContain("kFrontLeftXPos");
  });

  it("takes other units, arithmetic and comments, and refuses a file with none of the constants", () => {
    const t = parseTunerConstants(`
      // private static final double kDriveGearRatio = 99;
      private static final double kDriveGearRatio = (50.0 / 14) * (17.0 / 27) * (45.0 / 15);
      private static final Distance kWheelRadius = Meters.of(0.05);
      public static final LinearVelocity kSpeedAt12Volts = FeetPerSecond.of(15.5);
      private static final Distance kFrontLeftXPos = Inches.of(22.75 / 2);
      private static final Distance kFrontLeftYPos = Inches.of(-10.375);
    `);
    expect(t.driveGearRatio).toBeCloseTo((50 / 14) * (17 / 27) * (45 / 15), 12);
    expect(t.wheelRadiusMeters).toBe(0.05);
    expect(t.speedAt12VoltsMetersPerSecond).toBeCloseTo(15.5 * 0.3048, 12);
    expect(t.tipDistanceMeters).toBeCloseTo(10.375 * 0.0254, 12);
    expect(() => parseTunerConstants("public class Robot {}")).toThrow(/no TunerConstants/);
  });
});

describe("AccelerationLimiter import", () => {
  it("reads what paths plan with: μ, the mass, the centre of mass, the current, the turning cap, the share and the motor", () => {
    const l = parseAccelerationLimiter(LIMITER);
    expect(l).toMatchObject({ frictionCoefficient: 1.5, massKg: 60, centerOfMassHeightMeters: 0.2, statorCurrentLimitAmps: 150, maxAngularRadPerSecSquared: 12, tractionShare: 0.9 });
    expect(l.driveMotor).toEqual({ nominalVolts: 12, stallTorqueNewtonMeters: 9.362, stallCurrentAmps: 476.1, freeSpeedRadPerSec: (5785 * 2 * Math.PI) / 60 });
    // the one button tells the two files apart
    expect(parseRobotCode(LIMITER).kind).toBe("limiter");
    expect(parseRobotCode(TUNER).kind).toBe("tuner");
    expect(() => parseRobotCode("public class Robot {}")).toThrow(/no TunerConstants/);
  });

  it("applies each file's own numbers and leaves the rest", () => {
    const soft = { ...DEFAULT_ROBOT_SETTINGS, config: { ...DEFAULT_ROBOT_SETTINGS.config, frictionCoefficient: 0.9, massKg: 50, tractionShare: 0.8 } };
    const tuned = withRobotCode(soft, parseTunerConstants(TUNER));
    expect(tuned.config.driveGearRatio).toBeCloseTo(7.3636, 4);
    expect(tuned.config.massKg).toBe(50); // TunerConstants has no mass
    expect(tuned.config.statorCurrentLimitAmps).toBe(150); // nor the planning current: kSlipCurrent isn't applied
    expect(tuned.config.maxWheelMetersPerSecond).toBeCloseTo(0.9 * 4.54, 5);
    const both = withRobotCode(tuned, parseAccelerationLimiter(LIMITER));
    expect(both.config).toMatchObject({ frictionCoefficient: 1.5, massKg: 60, tractionShare: 0.9, statorCurrentLimitAmps: 150, maxAngularRadPerSecSquared: 12 });
    expect(both.config.driveGearRatio).toBe(tuned.config.driveGearRatio); // the limiter has no gearing
    // Joe's robot, both files: AccelerationLimiter's own 11.2 m/s²
    expect(tractionMetersPerSecondSquared(both.config)).toBeCloseTo(11.21, 2);
    expect(importSummary("AccelerationLimiter.java", parseAccelerationLimiter(LIMITER))).toBe(
      "AccelerationLimiter.java: mass 60 kg, μ 1.5, centre of mass 0.2 m, motor current 150 A, turning 12 rad/s², acceleration 90 %, drive motor 9.362 N·m stall.",
    );
  });

  it("warns when kSlipCurrent caps the motors below the current paths plan with", () => {
    const t = parseTunerConstants(TUNER); // 120 A
    expect(slipCurrentWarning(DEFAULT_ROBOT_SETTINGS, t)).toMatch(/^kSlipCurrent is 120 A, below the 150 A paths plan with/);
    expect(slipCurrentWarning(DEFAULT_ROBOT_SETTINGS, { ...t, slipCurrentAmps: 240 })).toBeNull();
    expect(slipCurrentWarning(DEFAULT_ROBOT_SETTINGS, parseAccelerationLimiter(LIMITER))).toBeNull();
  });
});

describe("a new project's robot", () => {
  it("plans with AccelerationLimiter's MODEL_CURRENT and ANGULAR_ACCEL, which TunerConstants doesn't have", () => {
    expect(DEFAULT_ROBOT_SETTINGS.config.statorCurrentLimitAmps).toBe(150);
    expect(DEFAULT_ROBOT_SETTINGS.config.maxAngularRadPerSecSquared).toBe(12);
    const t = parseTunerConstants(TUNER);
    expect(Object.keys(t)).not.toContain("statorCurrentLimitAmps");
    expect(Object.keys(t)).not.toContain("maxAngularRadPerSecSquared");
    // and they survive robot.json
    const back = parseRobot(writeRobot(DEFAULT_ROBOT_SETTINGS)).config;
    expect([back.statorCurrentLimitAmps, back.maxAngularRadPerSecSquared]).toEqual([150, 12]);
  });
});

describe("how hard paths drive", () => {
  it("plans to 90 % of the top speed and of the traction limit by default", () => {
    const r = DEFAULT_ROBOT_SETTINGS;
    expect(r.topSpeedShare).toBe(0.9);
    expect(r.config.tractionShare).toBe(0.9);
    const free = (r.config.driveMotor.freeSpeedRadPerSec / r.config.driveGearRatio) * r.config.wheelRadiusMeters;
    expect(wheelSpeedCapMetersPerSecond(r.config)).toBeCloseTo(0.9 * free, 4);
  });

  it("takes the share of the speed at 12 V once there is one, and of whatever share is set", () => {
    const r = withTopSpeed({ ...DEFAULT_ROBOT_SETTINGS, speedAt12VoltsMetersPerSecond: 4.54, topSpeedShare: 0.8 });
    expect(topSpeedMetersPerSecond(r)).toBe(4.54);
    expect(r.config.maxWheelMetersPerSecond).toBeCloseTo(0.8 * 4.54, 5);
    const softer = { ...r.config, tractionShare: 0.75 };
    expect(tractionMetersPerSecondSquared(softer) / tractionMetersPerSecondSquared(r.config)).toBeCloseTo(0.75 / 0.9, 12);
  });

  it("keeps both shares and the speed at 12 V in robot.json; an older file's own wheel cap becomes its share", () => {
    const r = withTopSpeed({ ...DEFAULT_ROBOT_SETTINGS, speedAt12VoltsMetersPerSecond: 4.54, topSpeedShare: 0.85, config: { ...DEFAULT_ROBOT_SETTINGS.config, tractionShare: 0.8 } });
    const text = writeRobot(r);
    expect(JSON.parse(text)).toMatchObject({ topSpeedShare: 0.85, speedAt12VoltsMetersPerSecond: 4.54, tractionShare: 0.8 });
    const back = parseRobot(text);
    expect(back.topSpeedShare).toBe(0.85);
    expect(back.config.tractionShare).toBe(0.8);
    expect(back.config.maxWheelMetersPerSecond).toBeCloseTo(0.85 * 4.54, 5);
    // no share in the file: 90 %, unless it capped the wheels itself
    expect(parseRobot("{}").topSpeedShare).toBe(0.9);
    const free = topSpeedMetersPerSecond(DEFAULT_ROBOT_SETTINGS);
    expect(parseRobot(JSON.stringify({ maxWheelMetersPerSecond: 0.5 * free })).topSpeedShare).toBeCloseTo(0.5, 9);
  });
});
