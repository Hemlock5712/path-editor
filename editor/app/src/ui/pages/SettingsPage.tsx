// Settings: the things that rarely change, out of the editor's way. Robot (robot.json), defaults for new
// paths, the field and its obstacles (field.json), the project (red flip, the generated names), the Sim
// preview's drivetrain, and the editor's own preferences.

import { useEffect, useState } from "react";
import { Bot, Cpu, FolderCog, Hexagon, Map as MapIcon, Route, SlidersHorizontal, Trash2 } from "lucide-react";
import {
  checkRobot,
  gripMetersPerSecondSquared,
  KRAKEN_X60_FOC,
  tipDistanceInUseMeters,
  tipMetersPerSecondSquared,
  wheelSpeedCapMetersPerSecond,
  type Motor,
  type RobotConfig,
} from "../../../../core/src/index.ts";
import { isValidPackage } from "../../io/javaEnum.ts";
import { PATHS_DIR } from "../../io/projectIo.ts";
import { withRobotLimits } from "../../model/edits.ts";
import { parseTunerConstants, type TunerImport } from "../../io/tunerConstants.ts";
import {
  DEFAULT_ROBOT_SETTINGS,
  DEFAULT_SIM_PREVIEW,
  hasOldPathLimits,
  topSpeedMetersPerSecond,
  withTopSpeed,
  type PathDefaults,
  type RobotSettings,
  type SimPreviewSettings,
} from "../../model/project.ts";
import {
  wheelSlipOnsetOnFloor,
  hasOldDefaultMu,
  simFloorCapMetersPerSecondSquared,
  SPIKE_GRIP_MU,
  tractionWorking,
  tractionHeadline,
} from "../../model/traction.ts";
import { editProject, select, setTool, setView } from "../../state/editor.ts";
import { session } from "../../state/session.ts";
import { useStore } from "../../state/store.ts";
import { setPage, ui, type SettingsTab } from "../../state/ui.ts";
import { Button, formatNumber, NumberField, Segmented, SelectField, Switch, TextField } from "../controls.tsx";
import { useEditor } from "../panels/common.tsx";
import { showQuickStart } from "../QuickStart.tsx";

const TABS: readonly { tab: SettingsTab; label: string; icon: typeof Bot }[] = [
  { tab: "robot", label: "Robot", icon: Bot },
  { tab: "defaults", label: "Path defaults", icon: Route },
  { tab: "field", label: "Field", icon: MapIcon },
  { tab: "project", label: "Project", icon: FolderCog },
  { tab: "sim", label: "Sim preview", icon: Cpu },
  { tab: "editor", label: "Editor", icon: SlidersHorizontal },
];

const MOTORS: { readonly value: string; readonly label: string; readonly motor: Motor }[] = [
  { value: "krakenX60Foc", label: "Kraken X60 (FOC)", motor: KRAKEN_X60_FOC },
  {
    value: "krakenX60",
    label: "Kraken X60",
    motor: { nominalVolts: 12, stallTorqueNewtonMeters: 7.09, stallCurrentAmps: 366, freeSpeedRadPerSec: (6000 * 2 * Math.PI) / 60 },
  },
  {
    value: "falcon500Foc",
    label: "Falcon 500 (FOC)",
    motor: { nominalVolts: 12, stallTorqueNewtonMeters: 5.84, stallCurrentAmps: 304, freeSpeedRadPerSec: (6080 * 2 * Math.PI) / 60 },
  },
  {
    value: "neoVortex",
    label: "NEO Vortex",
    motor: { nominalVolts: 12, stallTorqueNewtonMeters: 3.6, stallCurrentAmps: 211, freeSpeedRadPerSec: (6784 * 2 * Math.PI) / 60 },
  },
];

function motorKey(m: Motor): string {
  const close = (a: number, b: number) => Math.abs(a - b) < 1e-3 * Math.max(1, Math.abs(b));
  const found = MOTORS.find(
    (x) => close(x.motor.stallTorqueNewtonMeters, m.stallTorqueNewtonMeters) && close(x.motor.stallCurrentAmps, m.stallCurrentAmps) && close(x.motor.freeSpeedRadPerSec, m.freeSpeedRadPerSec),
  );
  return found?.value ?? "custom";
}

/** The same checks the core's robot reader does, so a bad number is refused before it's planned. */
function checkedRobot(r: RobotConfig): RobotConfig {
  const positive = [r.massKg, r.momentOfInertiaKgM2, r.moduleRadiusMeters, r.wheelRadiusMeters, r.driveGearRatio, r.statorCurrentLimitAmps, r.frictionCoefficient];
  if (positive.some((v) => !(v > 0)) || r.driveMotorCount < 1) throw new RangeError("robot numbers must be positive");
  return checkRobot(r);
}

function RobotSettingsTab() {
  const robot = useEditor((s) => s.project.robot);
  const c = robot.config;
  const set = (label: string, change: Partial<RobotConfig>, extra: Partial<RobotSettings> = {}) =>
    editProject(label, (p) => ({ ...p, robot: withTopSpeed({ ...p.robot, ...extra, config: checkedRobot({ ...p.robot.config, ...change }) }) }));
  const setBumpers = (change: Partial<RobotSettings>) => editProject("Bumper size", (p) => ({ ...p, robot: { ...p.robot, ...change } }));
  const rpm = (radPerSec: number) => (radPerSec * 60) / (2 * Math.PI);
  const [imported, setImported] = useState<TunerImport | string | null>(null);
  const importTuner = async (file: File) => {
    let t: TunerImport;
    try {
      t = parseTunerConstants(await file.text());
    } catch (e) {
      setImported(`${file.name}: ${e instanceof Error ? e.message : String(e)}`);
      return;
    }
    set(
      "Import TunerConstants",
      {
        ...(t.driveGearRatio !== undefined && { driveGearRatio: t.driveGearRatio }),
        ...(t.wheelRadiusMeters !== undefined && { wheelRadiusMeters: t.wheelRadiusMeters }),
        ...(t.moduleRadiusMeters !== undefined && { moduleRadiusMeters: t.moduleRadiusMeters }),
        ...(t.tipDistanceMeters !== undefined && { tipDistanceMeters: t.tipDistanceMeters }),
      },
      t.speedAt12VoltsMetersPerSecond !== undefined ? { speedAt12VoltsMetersPerSecond: t.speedAt12VoltsMetersPerSecond } : {},
    );
    setImported(t);
  };
  return (
    <section className="settings-section" aria-label="Robot">
      <h3>Robot</h3>
      <p className="hint">
        Saved to <code>deploy/paths/robot.json</code>, which the path follower reads on the robot. Each number under{" "}
        <strong>Match your robot code</strong> should equal the constant named under it, in <code>AccelerationLimiter.java</code> or{" "}
        <code>TunerConstants</code>: then the paths plan to the same limits teleop drives to.
      </p>
      <div className="settings-group" aria-label="Import">
        <h4>Start from your robot code</h4>
        <p className="hint">
          Import the drivetrain's <code>TunerConstants.java</code> (CTRE Tuner X): it fills in the gear ratio, the wheel radius, the module
          positions and the speed at 12 V. Then measure the mass, μ and the centre of mass height, below.
        </p>
        <label className="btn import-btn">
          Import TunerConstants.java
          <input
            type="file"
            accept=".java"
            hidden
            data-testid="import-tuner"
            onChange={(e) => {
              const f = e.target.files?.[0];
              e.target.value = "";
              if (f) void importTuner(f);
            }}
          />
        </label>
        {typeof imported === "string" && (
          <p className="check error" role="alert" data-testid="tuner-import-error">
            {imported}
          </p>
        )}
        {imported && typeof imported !== "string" && (
          <p className="stats" data-testid="tuner-imported">
            Imported {imported.found.join(", ")}:{" "}
            {[
              imported.driveGearRatio !== undefined && `gear ratio ${formatNumber(imported.driveGearRatio, 3)}:1`,
              imported.wheelRadiusMeters !== undefined && `wheel radius ${formatNumber(imported.wheelRadiusMeters, 4)} m`,
              imported.moduleRadiusMeters !== undefined && `module radius ${formatNumber(imported.moduleRadiusMeters, 3)} m`,
              imported.tipDistanceMeters !== undefined && `tip distance ${formatNumber(imported.tipDistanceMeters, 3)} m`,
              imported.speedAt12VoltsMetersPerSecond !== undefined && `speed at 12 V ${formatNumber(imported.speedAt12VoltsMetersPerSecond, 2)} m/s`,
            ]
              .filter(Boolean)
              .join(", ")}
            .
          </p>
        )}
      </div>
      <MarginsGroup robot={robot} set={set} />
      <LimitsSummary config={c} />
      <div className="settings-group" aria-label="Match your robot code">
        <h4>Match your robot code</h4>
        <div className="grid3">
          <NumberField label="Mass" unit="kg" min={1} value={c.massKg} code="ROBOT_MASS" onCommit={(v) => set("Robot mass", { massKg: v })} />
          <NumberField
            label="Tread μ"
            min={0.05}
            step={0.05}
            value={c.frictionCoefficient}
            code="MU"
            title="The tread's friction on carpet: about 1.5 for SDS MK5 Spike Grip"
            testId="tread-mu"
            onCommit={(v) => set("Tread μ", { frictionCoefficient: v })}
          />
          <NumberField
            label="Centre of mass height"
            unit="m"
            min={0}
            step={0.01}
            value={c.centerOfMassHeightMeters}
            code="COM_HEIGHT"
            title="Above the floor. 0 = no tipping limit"
            testId="com-height"
            onCommit={(v) => set("Centre of mass height", { centerOfMassHeightMeters: v })}
          />
          <NumberField
            label="Tip distance"
            unit="m"
            blank="auto (square base)"
            blankValue={Number.NaN}
            min={0.01}
            step={0.01}
            value={c.tipDistanceMeters}
            code="TIP_DISTANCE"
            title="Centre to the line the robot tips over: the nearer of the modules' x and y. Blank = module radius ÷ √2"
            testId="tip-distance"
            onCommit={(v) => set("Tip distance", { tipDistanceMeters: v })}
          />
          <NumberField
            label="Module radius"
            unit="m"
            min={0.01}
            value={c.moduleRadiusMeters}
            code="BASE_RADIUS"
            title="Centre to each swerve module: the hypot of TunerConstants' module X and Y"
            onCommit={(v) => set("Module radius", { moduleRadiusMeters: v })}
          />
          <NumberField
            label="Wheel radius"
            unit="m"
            min={0.001}
            step={0.001}
            value={c.wheelRadiusMeters}
            code="WheelRadius"
            title="TunerConstants' WheelRadius"
            onCommit={(v) => set("Wheel radius", { wheelRadiusMeters: v })}
          />
          <NumberField
            label="Gear ratio"
            unit=":1"
            min={0.1}
            value={c.driveGearRatio}
            code="DriveMotorGearRatio"
            title="TunerConstants' DriveMotorGearRatio"
            onCommit={(v) => set("Gear ratio", { driveGearRatio: v })}
          />
          <NumberField
            label="Motor current"
            unit="A"
            min={1}
            value={c.statorCurrentLimitAmps}
            code="MODEL_CURRENT"
            title="Amps per drive motor the plan counts on when it works out how hard the motors can push"
            onCommit={(v) => set("Current limit", { statorCurrentLimitAmps: v })}
          />
          <NumberField
            label="Max spin accel"
            unit="rad/s²"
            blank="none"
            blankValue={Infinity}
            min={0.1}
            value={c.maxAngularRadPerSecSquared}
            code="ANGULAR_ACCEL"
            title="The turning cap: AccelerationLimiter uses 12"
            onCommit={(v) => set("Max spin acceleration", { maxAngularRadPerSecSquared: v })}
          />
          <div className="field">
            <SelectField
              label="Drive motor"
              value={motorKey(c.driveMotor)}
              options={[...MOTORS.map((m) => ({ value: m.value, label: m.label })), { value: "custom", label: "Custom (from robot.json)" }]}
              onChange={(v) => {
                const m = MOTORS.find((x) => x.value === v);
                if (m) set("Drive motor", { driveMotor: m.motor });
              }}
            />
            <code className="field-code">DRIVE_MOTOR</code>
          </div>
        </div>
        <p className="stats">
          Motor: {formatNumber(c.driveMotor.stallTorqueNewtonMeters, 3)} N·m stall, {formatNumber(c.driveMotor.stallCurrentAmps, 1)} A,{" "}
          {formatNumber(rpm(c.driveMotor.freeSpeedRadPerSec), 0)} rpm free
        </p>
        {hasOldDefaultMu(c) && (
          <div className="settings-note" role="note" data-testid="old-mu-note">
            <span>μ 0.7 is the old planning default. μ is now the tread's own (about 1.5 for Spike Grip), and the plan takes 0.9 of the limit it makes.</span>
            <Button onClick={() => set("Tread μ", { frictionCoefficient: SPIKE_GRIP_MU })}>Use 1.5</Button>
          </div>
        )}
        <div className="hint" data-testid="traction-calibrate">
          <strong>μ and the centre of mass height are estimates: measure them, then change them here and in the robot code.</strong>
          <ul className="calibrate">
            <li>
              <strong>μ</strong>: brake the wheels and pull the robot slowly across carpet with a luggage scale. μ = the peak pull ÷ the
              robot's weight (both in kg).
            </li>
            <li>
              <strong>Centre of mass height</strong>: lift the front until the robot balances on its back wheels. h = the back wheels'
              distance from the centre ({formatNumber(tipDistanceInUseMeters(c), 2)} m on this base) ÷ tan(the tilt).
            </li>
          </ul>
        </div>
      </div>
      <div className="settings-group" aria-label="Bumpers">
        <h4>Bumpers</h4>
        <p className="hint">For the wall and obstacle checks and the pathfinder. Not in the limiter.</p>
        <div className="grid3">
          <NumberField label="Bumper length" unit="m" min={0.1} value={robot.bumperLengthMeters} title="Front to back, with bumpers" onCommit={(v) => setBumpers({ bumperLengthMeters: v })} />
          <NumberField label="Bumper width" unit="m" min={0.1} value={robot.bumperWidthMeters} onCommit={(v) => setBumpers({ bumperWidthMeters: v })} />
        </div>
      </div>
      <details className="settings-group settings-advanced" aria-label="Advanced" data-testid="robot-advanced">
        <summary>
          <h4>Advanced: path follower only</h4>
        </summary>
        <p className="hint">Limits the path follower can use that AccelerationLimiter doesn't have. The defaults are fine for most robots.</p>
        <NumberField
          label="Speed at 12 V"
          unit="m/s"
          blank="motor free speed"
          blankValue={Number.NaN}
          min={0.1}
          value={robot.speedAt12VoltsMetersPerSecond}
          code="kSpeedAt12Volts"
          title="What the top speed share is of. Blank: the motor's free speed through the gearing"
          testId="speed-at-12v"
          onCommit={(v) => set("Speed at 12 V", {}, { speedAt12VoltsMetersPerSecond: v })}
        />
        <div className="grid3">
          <NumberField
            label="Moment of inertia"
            unit="kg·m²"
            min={0.01}
            value={c.momentOfInertiaKgM2}
            title="How hard the robot is to spin; the sims use it"
            onCommit={(v) => set("Moment of inertia", { momentOfInertiaKgM2: v })}
          />
          <NumberField label="Drive motors" min={1} decimals={0} value={c.driveMotorCount} onCommit={(v) => set("Drive motors", { driveMotorCount: Math.round(v) })} />
          <NumberField label="Max spin" unit="rad/s" blank="none" blankValue={Infinity} min={0.1} value={c.maxAngularRadPerSec} onCommit={(v) => set("Max spin", { maxAngularRadPerSec: v })} />
        </div>
      </details>
      <div>
        <Button onClick={() => editProject("Reset robot", (p) => ({ ...p, robot: { ...DEFAULT_ROBOT_SETTINGS, extra: p.robot.extra } }))}>
          Reset the robot to the defaults
        </Button>
      </div>
    </section>
  );
}

/** One share of a limit, 50 to 100 %, as a slider with the number it makes. */
function ShareSlider({
  label,
  value,
  onCommit,
  working,
  code,
  testId,
}: {
  label: string;
  value: number;
  onCommit: (share: number) => void;
  working: string;
  code?: string;
  testId: string;
}) {
  const [shown, setShown] = useState(Math.round(value * 100));
  useEffect(() => setShown(Math.round(value * 100)), [value]);
  const commit = () => {
    if (shown / 100 !== value) onCommit(shown / 100);
  };
  return (
    <div className="share-slider">
      <label className="field-label" htmlFor={testId}>
        {label} <strong>{shown} %</strong>
      </label>
      <input
        id={testId}
        data-testid={testId}
        type="range"
        min={50}
        max={100}
        step={1}
        value={shown}
        onChange={(e) => setShown(Number(e.target.value))}
        onPointerUp={commit}
        onKeyUp={commit}
        onBlur={commit}
      />
      <span className="stats">
        {working}
        {code && <code className="field-code">{code}</code>}
      </span>
    </div>
  );
}

/** How much of the robot's top speed and traction paths use: 90 % each by default, the rest for feedback. */
function MarginsGroup({ robot, set }: { robot: RobotSettings; set: (label: string, change: Partial<RobotConfig>, extra?: Partial<RobotSettings>) => void }) {
  const c = robot.config;
  const top = topSpeedMetersPerSecond(robot);
  const of = robot.speedAt12VoltsMetersPerSecond > 0 ? "the speed at 12 V" : "the motor's free speed";
  const limit = Math.min(gripMetersPerSecondSquared(c), tipMetersPerSecondSquared(c));
  const what = tipMetersPerSecondSquared(c) < gripMetersPerSecondSquared(c) ? "tipping" : "grip";
  return (
    <div className="settings-group" aria-label="Margins">
      <h4>How hard paths drive</h4>
      <p className="hint">
        The share of the robot's top speed and grip that paths plan to. The rest is room for the follower to catch up. 90 % is a good start;
        AccelerationLimiter uses 0.9 for acceleration.
      </p>
      <div className="grid2">
        <ShareSlider
          label="Top speed"
          value={robot.topSpeedShare}
          testId="top-speed-share"
          working={`of ${of}, ${formatNumber(top, 2)} m/s = ${formatNumber(robot.topSpeedShare * top, 2)} m/s`}
          onCommit={(share) => set("Top speed share", {}, { topSpeedShare: share })}
        />
        <ShareSlider
          label="Acceleration"
          value={c.tractionShare}
          testId="traction-share"
          working={`of ${what}, ${formatNumber(limit, 1)} m/s² = ${formatNumber(c.tractionShare * limit, 1)} m/s²`}
          code="the 0.9 in TRACTION_ACCEL"
          onCommit={(share) => set("Acceleration share", { tractionShare: share })}
        />
      </div>
    </div>
  );
}

/** The limits the numbers above make: what AccelerationLimiter works out from the same constants. */
function LimitsSummary({ config: c }: { config: RobotConfig }) {
  const spin = c.maxAngularRadPerSecSquared;
  return (
    <div className="settings-group limits-summary" aria-label="Limits">
      <h4>What the robot plans to</h4>
      <ul>
        <li data-testid="traction-limit">
          <strong>{tractionHeadline(c)}</strong>: speeding up, braking and cornering <code>TRACTION_ACCEL</code>
          <br />
          <span className="stats" data-testid="traction-working">{tractionWorking(c)}</span>
        </li>
        <li>
          <strong>Top wheel speed {formatNumber(wheelSpeedCapMetersPerSecond(c), 2)} m/s</strong>
        </li>
        <li>
          <strong>Turning acceleration {Number.isFinite(spin) ? `${formatNumber(spin, 1)} rad/s²` : "not capped"}</strong> <code>ANGULAR_ACCEL</code>
        </li>
      </ul>
      <p className="hint">Every path plans to these unless its own Top speed or Max accel is set lower.</p>
      <OldPathLimitsNote />
    </div>
  );
}

/** Paths (or the defaults) still on the old fixed 4 m/s and 6 m/s², which hold them under the robot's limits. */
function OldPathLimitsNote() {
  const project = useEditor((s) => s.project);
  const paths = project.paths.filter((p) => hasOldPathLimits(p.spec)).length;
  const defaults = hasOldPathLimits(project.settings.pathDefaults);
  if (paths === 0 && !defaults) return null;
  const which = [paths > 0 ? `${paths} ${paths === 1 ? "path" : "paths"}` : "", defaults ? "the path defaults" : ""].filter(Boolean).join(" and ");
  return (
    <div className="settings-note" role="note" data-testid="old-path-limits-note">
      <span>
        {which[0]!.toUpperCase() + which.slice(1)} still {paths === 1 && !defaults ? "has" : "have"} the old fixed 4 m/s and 6 m/s², which hold
        them under the robot's limits. Blank, they plan to the robot's own.
      </span>
      <Button testId="use-robot-limits" onClick={() => editProject("Use the robot's limits", withRobotLimits)}>
        Use the robot's limits
      </Button>
    </div>
  );
}

function DefaultsTab() {
  const d = useEditor((s) => s.project.settings.pathDefaults);
  const set = (change: Partial<PathDefaults>) => editProject("Path defaults", (p) => ({ ...p, settings: { ...p.settings, pathDefaults: { ...p.settings.pathDefaults, ...change } } }));
  return (
    <section className="settings-section" aria-label="Path defaults">
      <h3>Path defaults</h3>
      <p className="hint">
        What a new path starts with. Leave the limits blank and paths plan to the robot's own (Settings, Robot); set one only to make paths
        slower than the robot can go. Each path keeps its own copy (change it in the path's Global constraints).
      </p>
      <OldPathLimitsNote />
      <div className="settings-group">
        <div className="grid3">
          <NumberField
            label="Top speed"
            unit="m/s"
            blank="robot's limit"
            blankValue={Infinity}
            min={0.1}
            value={d.maxMetersPerSecond}
            testId="default-top-speed"
            onCommit={(v) => set({ maxMetersPerSecond: v })}
          />
          <NumberField
            label="Max accel"
            unit="m/s²"
            blank="robot's limit"
            blankValue={Infinity}
            min={0.1}
            value={d.maxMetersPerSecondSquared}
            testId="default-max-accel"
            onCommit={(v) => set({ maxMetersPerSecondSquared: v })}
          />
          <SelectField
            label="Heading between set headings"
            value={d.headingMode}
            options={[
              { value: "hold", label: "Hold and blend (swerve)" },
              { value: "faceTravel", label: "Face the direction of travel" },
            ]}
            onChange={(headingMode) => set({ headingMode })}
          />
        </div>
      </div>
    </section>
  );
}

function FieldTab() {
  const field = useEditor((s) => s.project.field);
  return (
    <section className="settings-section" aria-label="Field">
      <h3>Field</h3>
      <p className="hint">
        <code>deploy/paths/field.json</code>, as the Java <code>FieldMap</code> reads it. The origin is the field's centre (WPILib 2027). The
        image is the 2026 field.
      </p>
      <div className="settings-group">
        <h4>Size</h4>
        <div className="grid3">
          <NumberField
            label="Field length"
            unit="m"
            min={1}
            value={field.lengthMeters}
            onCommit={(v) => editProject("Field size", (p) => ({ ...p, field: { ...p.field, lengthMeters: v } }))}
          />
          <NumberField
            label="Field width"
            unit="m"
            min={1}
            value={field.widthMeters}
            onCommit={(v) => editProject("Field size", (p) => ({ ...p, field: { ...p.field, widthMeters: v } }))}
          />
        </div>
      </div>
      <div className="settings-group">
        <h4>Obstacles</h4>
        <p className="hint">The bumper checks and the optimiser keep clear of these. Each is used as its convex hull.</p>
        <ul className="item-list" aria-label="Obstacles">
          {field.obstacles.map((o, i) => (
            <li key={i} className="row" style={{ flexWrap: "nowrap" }}>
              <button
                type="button"
                className="item"
                onClick={() => {
                  setPage("editor");
                  select({ kind: "obstacle", index: i });
                }}
                data-tip="Show it on the field"
              >
                <Hexagon size={14} />
                <span className="main">{o.name}</span>
                <span className="tag">{o.polygon.length} corners</span>
              </button>
              <button
                type="button"
                className="icon-btn"
                aria-label={`Delete obstacle ${o.name}`}
                data-tip="Delete"
                onClick={() => editProject("Delete obstacle", (p) => ({ ...p, field: { ...p.field, obstacles: p.field.obstacles.filter((_, k) => k !== i) } }), null)}
              >
                <Trash2 size={14} />
              </button>
            </li>
          ))}
          {field.obstacles.length === 0 && <li className="hint">No obstacles yet.</li>}
        </ul>
        <div>
          <Button
            onClick={() => {
              setPage("editor");
              setTool("obstacle");
            }}
          >
            <Hexagon size={14} />
            Draw an obstacle on the field
          </Button>
        </div>
      </div>
    </section>
  );
}

function ProjectTab() {
  const settings = useEditor((s) => s.project.settings);
  const files = useStore(session, (s) => s.files);
  return (
    <section className="settings-section" aria-label="Project">
      <h3>Project</h3>
      <div className="settings-group">
        <h4>Robot project</h4>
        <p className="stats">
          {files ? (
            <>
              Folder <strong>{files.label}</strong>; paths in <code>{PATHS_DIR}</code>.
            </>
          ) : (
            <>Not saved to a robot project yet. Open the robot project's root folder (the one with build.gradle); paths go in <code>{PATHS_DIR}</code>.</>
          )}
        </p>
      </div>
      <div className="settings-group">
        <h4>Red alliance</h4>
        <Segmented
          label="Red alliance"
          value={settings.redFrame}
          options={[
            { value: "RED_HALF_TURN", label: "Half-turn about the centre" },
            { value: "RED_MIRROR", label: "Mirror across the centre line" },
          ]}
          onChange={(redFrame) => editProject("Red frame", (p) => ({ ...p, settings: { ...p.settings, redFrame } }))}
        />
        <p className="hint">How the robot flips blue paths for red (the FieldFrame you pass the follow command). The red view uses the same flip.</p>
      </div>
      <div className="settings-group">
        <h4>Generated names</h4>
        <div className="grid3">
          <TextField
            label="Java package"
            value={settings.javaPackage || "-"}
            validate={(v) => (v === "-" || isValidPackage(v) ? null : "A Java package, like frc.robot")}
            onCommit={(v) => editProject("Names package", (p) => ({ ...p, settings: { ...p.settings, javaPackage: v === "-" ? "" : v } }))}
          />
        </div>
        <p className="hint">
          Saving writes <code>PathEditorNames.java</code> there: the path files, the event names and the named points as constants
          (<code>Paths.TO_REEF</code>, <code>Events.SHOOT</code>, <code>Points.SCORE_L</code>), which every name-taking method in lib
          accepts, so a typo is a compile error. <code>PathLibrary.loadAll()</code> checks them against deploy at boot. It replaces the old{" "}
          <code>PathName.java</code>. Enter <code>-</code> to stop generating it.
        </p>
      </div>
    </section>
  );
}

function SimTab() {
  const sim = useEditor((s) => s.project.settings.simPreview);
  const robot = useEditor((s) => s.project.robot.config);
  const set = (change: Partial<SimPreviewSettings>) => editProject("Sim preview", (p) => ({ ...p, settings: { ...p.settings, simPreview: { ...p.settings.simPreview, ...change } } }));
  return (
    <section className="settings-section" aria-label="Sim preview">
      <h3>Sim preview drivetrain</h3>
      <p className="hint">
        The simulated drivetrain the Sim preview runs the robot's follower on (switch it on in the playback bar), so it shows what the robot
        would do, not just the plan. Saved in the project's editor file, not robot.json: it describes the sim, not the robot.
      </p>
      <div className="settings-group">
        <div className="grid3">
          <NumberField label="Command latency" unit="ms" min={0} value={sim.latencyMs} title="From a command being sent to the modules acting on it" onCommit={(v) => set({ latencyMs: v })} />
          <NumberField label="Velocity lag" unit="ms" min={1} value={sim.lagMs} title="Time constant of the chassis velocity's first-order lag" onCommit={(v) => set({ lagMs: v })} />
          <NumberField
            label="Floor tread μ"
            blank="no cap"
            blankValue={Number.NaN}
            min={0.05}
            step={0.05}
            value={sim.gripMu ?? Number.NaN}
            title="The tread's μ on the simulated floor (1.5 for Spike Grip); blank for no cap (as in the end-to-end suite)"
            testId="sim-floor-mu"
            onCommit={(v) => set({ gripMu: Number.isFinite(v) ? v : null })}
          />
        </div>
        <p className="stats" data-testid="sim-floor-cap">
          {sim.gripMu === null ? (
            <>No floor cap: the chassis gets whatever the follower asks for.</>
          ) : (
            <>
              <strong>
                Caps the chassis at {formatNumber(simFloorCapMetersPerSecondSquared(sim.gripMu, robot), 1)} m/s²
              </strong>
              : the lower of μ·g and tipping, with the Robot tab's centre of mass and tip distance, and no margin. The sim moves the chassis
              as one body, so it doesn't show wheel slip: with all four wheels pushing equally, the light wheels start slipping at{" "}
              {formatNumber(wheelSlipOnsetOnFloor(sim.gripMu, robot), 1)} m/s².
            </>
          )}
        </p>
        <div>
          <Button variant="ghost" onClick={() => editProject("Sim preview", (p) => ({ ...p, settings: { ...p.settings, simPreview: DEFAULT_SIM_PREVIEW } }))}>
            Sim defaults (the end-to-end suite's: 20 ms, 40 ms, no floor cap)
          </Button>
        </div>
      </div>
    </section>
  );
}

function EditorTab() {
  const view = useEditor((s) => s.view);
  return (
    <section className="settings-section" aria-label="Editor">
      <h3>Editor</h3>
      <div className="settings-group">
        <h4>Snapping</h4>
        <Switch label="Snap points to a 25 cm grid (Shift snaps to 5 cm)" checked={view.gridSnap} onChange={(on) => setView({ gridSnap: on })} />
        <p className="hint">Headings snap to 15° (hold Alt to turn freely). Waypoints dragged near a named point snap to it and follow it.</p>
      </div>
      <div className="settings-group">
        <h4>Field</h4>
        <SelectField
          label="Robot footprints along the path"
          value={String(view.footprintSpacing)}
          options={[
            { value: "0", label: "At waypoints only" },
            { value: "0.5", label: "Every 0.5 m" },
            { value: "1", label: "Every 1 m" },
            { value: "2", label: "Every 2 m" },
          ]}
          onChange={(v) => setView({ footprintSpacing: Number(v) })}
        />
      </div>
      <div className="settings-group">
        <h4>Help</h4>
        <div className="row">
          <Button
            onClick={() => {
              setPage("editor");
              showQuickStart();
            }}
          >
            Show the quick start
          </Button>
          <Button variant="ghost" onClick={() => ui.set({ helpOpen: true })}>
            Keyboard shortcuts
          </Button>
        </div>
      </div>
    </section>
  );
}

export function SettingsPage() {
  const tab = useStore(ui, (s) => s.settingsTab);
  return (
    <div className="settings" data-testid="settings-page">
      <nav className="settings-nav" role="tablist" aria-label="Settings" aria-orientation="vertical">
        <h2>Settings</h2>
        {TABS.map(({ tab: t, label, icon: Icon }) => (
          <button key={t} type="button" role="tab" className="settings-tab" aria-selected={tab === t} onClick={() => ui.set({ settingsTab: t })} data-testid={`settings-${t}`}>
            <Icon size={16} />
            {label}
          </button>
        ))}
      </nav>
      <div className="settings-body" role="tabpanel">
        {tab === "robot" && <RobotSettingsTab />}
        {tab === "defaults" && <DefaultsTab />}
        {tab === "field" && <FieldTab />}
        {tab === "project" && <ProjectTab />}
        {tab === "sim" && <SimTab />}
        {tab === "editor" && <EditorTab />}
      </div>
    </div>
  );
}
