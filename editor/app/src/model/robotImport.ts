// Applying an import of the robot's code (io/robotCode.ts) to the robot settings, and saying what it did.

import type { LimiterImport, RobotCodeImport, TunerImport } from "../io/robotCode.ts";
import { withTopSpeed, type RobotSettings } from "./project.ts";

/** The robot with an import's numbers in; anything the file didn't have is left as it was. */
export function withRobotCode(robot: RobotSettings, imp: RobotCodeImport): RobotSettings {
  const c = robot.config;
  if (imp.kind === "tuner") {
    return withTopSpeed({
      ...robot,
      speedAt12VoltsMetersPerSecond: imp.speedAt12VoltsMetersPerSecond ?? robot.speedAt12VoltsMetersPerSecond,
      config: {
        ...c,
        driveGearRatio: imp.driveGearRatio ?? c.driveGearRatio,
        wheelRadiusMeters: imp.wheelRadiusMeters ?? c.wheelRadiusMeters,
        moduleRadiusMeters: imp.moduleRadiusMeters ?? c.moduleRadiusMeters,
        tipDistanceMeters: imp.tipDistanceMeters ?? c.tipDistanceMeters,
      },
    });
  }
  return withTopSpeed({
    ...robot,
    config: {
      ...c,
      frictionCoefficient: imp.frictionCoefficient ?? c.frictionCoefficient,
      massKg: imp.massKg ?? c.massKg,
      centerOfMassHeightMeters: imp.centerOfMassHeightMeters ?? c.centerOfMassHeightMeters,
      statorCurrentLimitAmps: imp.statorCurrentLimitAmps ?? c.statorCurrentLimitAmps,
      maxAngularRadPerSecSquared: imp.maxAngularRadPerSecSquared ?? c.maxAngularRadPerSecSquared,
      tractionShare: imp.tractionShare ?? c.tractionShare,
      driveMotor: imp.driveMotor ?? c.driveMotor,
    },
  });
}

const f = (v: number, digits: number) => String(Number(v.toFixed(digits)));

/** "TunerConstants.java: gear ratio 7.364:1, …" for one import. */
export function importSummary(file: string, imp: RobotCodeImport): string {
  const parts = imp.kind === "tuner" ? tunerParts(imp) : limiterParts(imp);
  return `${file}: ${parts.join(", ")}.`;
}

function tunerParts(t: TunerImport): string[] {
  return [
    t.driveGearRatio !== undefined && `gear ratio ${f(t.driveGearRatio, 3)}:1`,
    t.wheelRadiusMeters !== undefined && `wheel radius ${f(t.wheelRadiusMeters, 4)} m`,
    t.moduleRadiusMeters !== undefined && `module radius ${f(t.moduleRadiusMeters, 3)} m`,
    t.tipDistanceMeters !== undefined && `tip distance ${f(t.tipDistanceMeters, 3)} m`,
    t.speedAt12VoltsMetersPerSecond !== undefined && `speed at 12 V ${f(t.speedAt12VoltsMetersPerSecond, 2)} m/s`,
  ].filter((p): p is string => typeof p === "string");
}

function limiterParts(l: LimiterImport): string[] {
  return [
    l.massKg !== undefined && `mass ${f(l.massKg, 2)} kg`,
    l.frictionCoefficient !== undefined && `μ ${f(l.frictionCoefficient, 3)}`,
    l.centerOfMassHeightMeters !== undefined && `centre of mass ${f(l.centerOfMassHeightMeters, 3)} m`,
    l.statorCurrentLimitAmps !== undefined && `motor current ${f(l.statorCurrentLimitAmps, 1)} A`,
    l.maxAngularRadPerSecSquared !== undefined && `turning ${f(l.maxAngularRadPerSecSquared, 2)} rad/s²`,
    l.tractionShare !== undefined && `acceleration ${f(l.tractionShare * 100, 1)} %`,
    l.driveMotor !== undefined && `drive motor ${f(l.driveMotor.stallTorqueNewtonMeters, 3)} N·m stall`,
  ].filter((p): p is string => typeof p === "string");
}

/**
 * kSlipCurrent below the current paths plan with: the motors are capped under what the plan counts on, so
 * the robot falls behind it when it speeds up. kSlipCurrent is a ceiling (Swerve School chapter 5 sets 240 A).
 */
export function slipCurrentWarning(robot: RobotSettings, imp: RobotCodeImport): string | null {
  if (imp.kind !== "tuner" || imp.slipCurrentAmps === undefined) return null;
  const plan = robot.config.statorCurrentLimitAmps;
  if (imp.slipCurrentAmps >= plan) return null;
  return (
    `kSlipCurrent is ${f(imp.slipCurrentAmps, 1)} A, below the ${f(plan, 1)} A paths plan with (MODEL_CURRENT): the motors can't push ` +
    `as hard as the plan counts on. kSlipCurrent is a ceiling, not the plan; raise it in TunerConstants (240 A in Swerve School).`
  );
}
