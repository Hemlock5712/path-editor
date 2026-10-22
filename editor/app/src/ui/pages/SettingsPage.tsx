// Settings: the things that rarely change, out of the editor's way. Robot (robot.json), defaults for new
// paths, the field and its obstacles (field.json), the project (red flip, the generated names), the Sim
// preview's drivetrain, and the editor's own preferences.

import { Bot, Cpu, FolderCog, Hexagon, Map as MapIcon, Route, SlidersHorizontal, Trash2 } from "lucide-react";
import { checkRobot, KRAKEN_X60_FOC, tipDistanceInUseMeters, type Motor, type RobotConfig } from "../../../../core/src/index.ts";
import { isValidPackage } from "../../io/javaEnum.ts";
import { PATHS_DIR } from "../../io/projectIo.ts";
import { DEFAULT_ROBOT_SETTINGS, DEFAULT_SIM_PREVIEW, type PathDefaults, type RobotSettings, type SimPreviewSettings } from "../../model/project.ts";
import {
  equalPushSlipMetersPerSecondSquared,
  hasOldDefaultMu,
  simFloorCapMetersPerSecondSquared,
  SPIKE_GRIP_MU,
  equalPushWorking,
  tipOrGripWorking,
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
    editProject(label, (p) => ({ ...p, robot: { ...p.robot, ...extra, config: checkedRobot({ ...p.robot.config, ...change }) } }));
  const setBumpers = (change: Partial<RobotSettings>) => editProject("Bumper size", (p) => ({ ...p, robot: { ...p.robot, ...change } }));
  const rpm = (radPerSec: number) => (radPerSec * 60) / (2 * Math.PI);
  return (
    <section className="settings-section" aria-label="Robot">
      <h3>Robot</h3>
      <p className="hint">
        <code>deploy/paths/robot.json</code>: what the plan and the robot's <code>RobotFile</code> use. Blank limits mean none.
      </p>
      <div className="settings-group">
        <h4>Body</h4>
        <div className="grid3">
          <NumberField label="Mass" unit="kg" min={1} value={c.massKg} onCommit={(v) => set("Robot mass", { massKg: v })} />
          <NumberField label="Moment of inertia" unit="kg·m²" min={0.01} value={c.momentOfInertiaKgM2} onCommit={(v) => set("Moment of inertia", { momentOfInertiaKgM2: v })} />
          <NumberField label="Module radius" unit="m" min={0.01} value={c.moduleRadiusMeters} title="Centre to each swerve module" onCommit={(v) => set("Module radius", { moduleRadiusMeters: v })} />
          <NumberField label="Bumper length" unit="m" min={0.1} value={robot.bumperLengthMeters} title="Front to back, with bumpers" onCommit={(v) => setBumpers({ bumperLengthMeters: v })} />
          <NumberField label="Bumper width" unit="m" min={0.1} value={robot.bumperWidthMeters} onCommit={(v) => setBumpers({ bumperWidthMeters: v })} />
        </div>
      </div>
      <div className="settings-group">
        <h4>Drive</h4>
        <div className="grid3">
          <SelectField
            label="Drive motor"
            value={motorKey(c.driveMotor)}
            options={[...MOTORS.map((m) => ({ value: m.value, label: m.label })), { value: "custom", label: "Custom (from robot.json)" }]}
            onChange={(v) => {
              const m = MOTORS.find((x) => x.value === v);
              if (m) set("Drive motor", { driveMotor: m.motor });
            }}
          />
          <NumberField label="Drive motors" min={1} decimals={0} value={c.driveMotorCount} onCommit={(v) => set("Drive motors", { driveMotorCount: Math.round(v) })} />
          <NumberField label="Gear ratio" unit=":1" min={0.1} value={c.driveGearRatio} onCommit={(v) => set("Gear ratio", { driveGearRatio: v })} />
          <NumberField label="Wheel radius" unit="m" min={0.001} step={0.001} value={c.wheelRadiusMeters} onCommit={(v) => set("Wheel radius", { wheelRadiusMeters: v })} />
          <NumberField label="Stator limit" unit="A" min={1} value={c.statorCurrentLimitAmps} onCommit={(v) => set("Current limit", { statorCurrentLimitAmps: v })} />
        </div>
        <p className="stats">
          Motor: {formatNumber(c.driveMotor.stallTorqueNewtonMeters, 3)} N·m stall, {formatNumber(c.driveMotor.stallCurrentAmps, 1)} A,{" "}
          {formatNumber(rpm(c.driveMotor.freeSpeedRadPerSec), 0)} rpm free
        </p>
      </div>
      <TractionGroup config={c} set={set} />
      <div className="settings-group">
        <h4>Limits</h4>
        <div className="grid3">
          <NumberField
            label="Max wheel speed"
            unit="m/s"
            blank="motor free speed"
            blankValue={Number.NaN}
            min={0.1}
            value={c.maxWheelMetersPerSecond}
            onCommit={(v) => set("Max wheel speed", { maxWheelMetersPerSecond: v })}
          />
          <NumberField label="Max spin" unit="rad/s" blank="none" blankValue={Infinity} min={0.1} value={c.maxAngularRadPerSec} onCommit={(v) => set("Max spin", { maxAngularRadPerSec: v })} />
          <NumberField
            label="Max spin accel"
            unit="rad/s²"
            blank="none"
            blankValue={Infinity}
            min={0.1}
            value={c.maxAngularRadPerSecSquared}
            onCommit={(v) => set("Max spin acceleration", { maxAngularRadPerSecSquared: v })}
          />
        </div>
      </div>
      <div>
        <Button onClick={() => editProject("Reset robot", (p) => ({ ...p, robot: { ...DEFAULT_ROBOT_SETTINGS, extra: p.robot.extra } }))}>
          Reset the robot to the defaults
        </Button>
      </div>
    </section>
  );
}

/**
 * Tread μ, centre of mass height, tip distance and the traction rule, and the limit they make: the plan
 * and the follower accelerate and corner at no more than it.
 */
function TractionGroup({ config: c, set }: { config: RobotConfig; set: (label: string, change: Partial<RobotConfig>) => void }) {
  return (
    <div className="settings-group" aria-label="Traction">
      <h4>Traction</h4>
      <div className="grid3">
        <NumberField
          label="Tread μ (1.5 for Spike Grip)"
          min={0.05}
          step={0.05}
          value={c.frictionCoefficient}
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
          title="Centre to the line the robot tips over: the nearer of the modules' x and y. Blank = module radius ÷ √2"
          testId="tip-distance"
          onCommit={(v) => set("Tip distance", { tipDistanceMeters: v })}
        />
      </div>
      <Segmented
        label="Traction rule"
        value={c.tractionRule}
        options={[
          { value: "tipOrGrip", label: "Tip or grip", tip: "0.9 × the lower of μ·g and g·tip/h: the tread slides or the robot tips, whichever first" },
          { value: "equalPush", label: "Equal push", tip: "0.9 × μ·g ÷ (1 + μ·h/tip): every wheel pushes the same, so the unloaded ones slip first" },
        ]}
        onChange={(tractionRule) => set("Traction rule", { tractionRule })}
      />
      <div className="stats" data-testid="traction-limit">
        <strong>{tractionHeadline(c)}</strong>
        <ul className="traction-rules">
          <li className={c.tractionRule === "tipOrGrip" ? "in-use" : undefined}>{tipOrGripWorking(c)}</li>
          <li className={c.tractionRule === "equalPush" ? "in-use" : undefined}>{equalPushWorking(c)}</li>
        </ul>
      </div>
      {hasOldDefaultMu(c) && (
        <div className="settings-note" role="note" data-testid="old-mu-note">
          <span>μ 0.7 is the old planning default. μ is now the tread's own (about 1.5 for Spike Grip), and the plan takes 0.9 of the limit it makes.</span>
          <Button onClick={() => set("Tread μ", { frictionCoefficient: SPIKE_GRIP_MU })}>Use 1.5</Button>
        </div>
      )}
      <div className="hint" data-testid="traction-calibrate">
        <strong>Estimates: calibrate on your robot.</strong>
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
  );
}

function DefaultsTab() {
  const d = useEditor((s) => s.project.settings.pathDefaults);
  const set = (change: Partial<PathDefaults>) => editProject("Path defaults", (p) => ({ ...p, settings: { ...p.settings, pathDefaults: { ...p.settings.pathDefaults, ...change } } }));
  return (
    <section className="settings-section" aria-label="Path defaults">
      <h3>Path defaults</h3>
      <p className="hint">What a new path starts with. Each path keeps its own copy (change it in the path's Global constraints).</p>
      <div className="settings-group">
        <div className="grid3">
          <NumberField label="Top speed" unit="m/s" min={0.1} value={d.maxMetersPerSecond} onCommit={(v) => set({ maxMetersPerSecond: v })} />
          <NumberField label="Max accel" unit="m/s²" min={0.1} value={d.maxMetersPerSecondSquared} onCommit={(v) => set({ maxMetersPerSecondSquared: v })} />
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
              {formatNumber(equalPushSlipMetersPerSecondSquared(sim.gripMu, robot), 1)} m/s².
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
