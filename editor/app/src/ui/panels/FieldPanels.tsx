// Project-wide things: named points (shared across paths), the field's obstacles, the robot, and the
// project settings.

import { KRAKEN_X60_FOC, type Motor, type RobotConfig } from "../../../../core/src/index.ts";
import { addNamedPoint, linkCount, linkToPoint, moveNamedPoint, removeNamedPoint, renameNamedPoint, uniquePointName } from "../../model/namedPoints.ts";
import { DEFAULT_ROBOT_SETTINGS, type RobotSettings } from "../../model/project.ts";
import { editProject, select } from "../../state/editor.ts";
import { isValidPackage } from "../../io/javaEnum.ts";
import { IconButton, NumberField, Section, SelectField, TextField, formatNumber } from "../controls.tsx";
import { useActivePath, useEditor } from "./common.tsx";

export function PointsPanel() {
  const project = useEditor((s) => s.project);
  const selection = useEditor((s) => s.selection);
  const path = useActivePath();
  const selectedWaypoint = selection?.kind === "waypoint" ? selection.index : -1;
  return (
    <Section title="Named points" count={project.namedPoints.length}>
      <p className="hint">Points shared across paths. Moving one moves every waypoint linked to it, in every path.</p>
      {project.namedPoints.map((p) => {
        const sel = selection?.kind === "point" && selection.id === p.id;
        return (
          <fieldset key={p.id} className={sel ? "card selected" : "card"} onFocus={() => !sel && select({ kind: "point", id: p.id })}>
            <legend>{p.name}</legend>
            <TextField
              label="Name"
              value={p.name}
              validate={(v) => (project.namedPoints.some((q) => q.id !== p.id && q.name === v) ? "Another point has that name" : null)}
              onCommit={(name) => editProject("Rename point", (pr) => renameNamedPoint(pr, p.id, name))}
            />
            <div className="grid2">
              <NumberField label="x" unit="m" value={p.x} onCommit={(v) => editProject("Move point", (pr) => moveNamedPoint(pr, p.id, v, p.y))} />
              <NumberField label="y" unit="m" value={p.y} onCommit={(v) => editProject("Move point", (pr) => moveNamedPoint(pr, p.id, p.x, v))} />
            </div>
            <p className="stats">{linkedText(linkCount(project, p.id))}</p>
            <div className="row">
              {selectedWaypoint >= 0 && path.links[selectedWaypoint] !== p.id && (
                <IconButton
                  label={`Link waypoint ${selectedWaypoint} to ${p.name}`}
                  onClick={() => editProject("Link waypoint", (pr) => linkToPoint(pr, path.id, selectedWaypoint, p.id))}
                >
                  Link waypoint {selectedWaypoint}
                </IconButton>
              )}
              <IconButton label={`Delete point ${p.name}`} onClick={() => editProject("Delete point", (pr) => removeNamedPoint(pr, p.id), null)}>
                Delete
              </IconButton>
            </div>
          </fieldset>
        );
      })}
      <IconButton
        label={selectedWaypoint >= 0 ? `New point at waypoint ${selectedWaypoint}` : "New point at the field centre"}
        onClick={() => {
          const w = path.spec.waypoints[selectedWaypoint];
          editProject("Add point", (pr) => {
            const [withPoint, point] = addNamedPoint(pr, uniquePointName(pr), w?.xMeters ?? 0, w?.yMeters ?? 0);
            return selectedWaypoint >= 0 ? linkToPoint(withPoint, path.id, selectedWaypoint, point.id) : withPoint;
          });
        }}
      >
        {selectedWaypoint >= 0 ? `+ Point from waypoint ${selectedWaypoint}` : "+ Point"}
      </IconButton>
    </Section>
  );
}

function linkedText(n: number): string {
  return n === 0 ? "No waypoints linked" : n === 1 ? "1 waypoint linked" : `${n} waypoints linked`;
}

export function ObstaclesPanel() {
  const field = useEditor((s) => s.project.field);
  const selection = useEditor((s) => s.selection);
  const sel = selection?.kind === "obstacle" ? selection.index : -1;
  return (
    <Section title="Field and obstacles" count={field.obstacles.length}>
      <p className="hint">
        <code>deploy/paths/field.json</code>, as the Java <code>FieldMap</code> reads it. The O tool draws a new obstacle:
        click its corners, then double-click or press Enter. Each is used as its convex hull.
      </p>
      <div className="grid2">
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
      <ul className="item-list" aria-label="Obstacles">
        {field.obstacles.map((o, i) => (
          <li key={i}>
            <button type="button" className={i === sel ? "item selected" : "item"} aria-pressed={i === sel} onClick={() => select({ kind: "obstacle", index: i })}>
              {o.name} <span className="tag">{o.polygon.length} corners</span>
            </button>
          </li>
        ))}
      </ul>
      {sel >= 0 && field.obstacles[sel] && (
        <div className="detail">
          <TextField
            label="Name"
            value={field.obstacles[sel]!.name}
            onCommit={(name) =>
              editProject("Rename obstacle", (p) => ({
                ...p,
                field: { ...p.field, obstacles: p.field.obstacles.map((o, i) => (i === sel ? { ...o, name } : o)) },
              }))
            }
          />
          <IconButton
            label="Delete obstacle"
            onClick={() =>
              editProject("Delete obstacle", (p) => ({ ...p, field: { ...p.field, obstacles: p.field.obstacles.filter((_, i) => i !== sel) } }), null)
            }
          >
            Delete obstacle
          </IconButton>
        </div>
      )}
    </Section>
  );
}

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

export function RobotPanel() {
  const robot = useEditor((s) => s.project.robot);
  const c = robot.config;
  const set = (label: string, change: Partial<RobotConfig>, extra: Partial<RobotSettings> = {}) =>
    editProject(label, (p) => ({ ...p, robot: { ...p.robot, ...extra, config: checkedRobot({ ...p.robot.config, ...change }) } }));
  const setBumpers = (change: Partial<RobotSettings>) => editProject("Bumper size", (p) => ({ ...p, robot: { ...p.robot, ...change } }));
  const rpm = (radPerSec: number) => (radPerSec * 60) / (2 * Math.PI);
  return (
    <Section title="Robot">
      <p className="hint">
        <code>deploy/paths/robot.json</code>: what the plan and the robot's <code>RobotFile</code> use. Blank limits mean none.
      </p>
      <div className="grid2">
        <NumberField label="Mass" unit="kg" min={1} value={c.massKg} onCommit={(v) => set("Robot mass", { massKg: v })} />
        <NumberField label="Moment of inertia" unit="kg·m²" min={0.01} value={c.momentOfInertiaKgM2} onCommit={(v) => set("Moment of inertia", { momentOfInertiaKgM2: v })} />
        <NumberField label="Module radius" unit="m" min={0.01} value={c.moduleRadiusMeters} title="Centre to each swerve module" onCommit={(v) => set("Module radius", { moduleRadiusMeters: v })} />
        <NumberField label="Wheel radius" unit="m" min={0.001} value={c.wheelRadiusMeters} onCommit={(v) => set("Wheel radius", { wheelRadiusMeters: v })} />
        <NumberField label="Gear ratio" unit=":1" min={0.1} value={c.driveGearRatio} onCommit={(v) => set("Gear ratio", { driveGearRatio: v })} />
        <NumberField label="Drive motors" min={1} decimals={0} value={c.driveMotorCount} onCommit={(v) => set("Drive motors", { driveMotorCount: Math.round(v) })} />
        <NumberField label="Stator limit" unit="A" min={1} value={c.statorCurrentLimitAmps} onCommit={(v) => set("Current limit", { statorCurrentLimitAmps: v })} />
        <NumberField label="Grip μ" min={0.05} value={c.frictionCoefficient} title="Measured traction; around 0.7 for carpet" onCommit={(v) => set("Grip", { frictionCoefficient: v })} />
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
        <NumberField label="Bumper length" unit="m" min={0.1} value={robot.bumperLengthMeters} title="Front to back, with bumpers" onCommit={(v) => setBumpers({ bumperLengthMeters: v })} />
        <NumberField label="Bumper width" unit="m" min={0.1} value={robot.bumperWidthMeters} onCommit={(v) => setBumpers({ bumperWidthMeters: v })} />
      </div>
      <SelectField
        label="Drive motor"
        value={motorKey(c.driveMotor)}
        options={[...MOTORS.map((m) => ({ value: m.value, label: m.label })), { value: "custom", label: "Custom (from robot.json)" }]}
        onChange={(v) => {
          const m = MOTORS.find((x) => x.value === v);
          if (m) set("Drive motor", { driveMotor: m.motor });
        }}
      />
      <p className="stats">
        Motor: {formatNumber(c.driveMotor.stallTorqueNewtonMeters, 3)} N·m stall, {formatNumber(c.driveMotor.stallCurrentAmps, 1)} A,{" "}
        {formatNumber(rpm(c.driveMotor.freeSpeedRadPerSec), 0)} rpm free
      </p>
      <IconButton label="Reset the robot to the defaults" onClick={() => editProject("Reset robot", (p) => ({ ...p, robot: { ...DEFAULT_ROBOT_SETTINGS, extra: p.robot.extra } }))}>
        Reset to defaults
      </IconButton>
    </Section>
  );
}

/** The same checks the core's robot reader does, so a bad number is refused before it's planned. */
function checkedRobot(r: RobotConfig): RobotConfig {
  const positive = [r.massKg, r.momentOfInertiaKgM2, r.moduleRadiusMeters, r.wheelRadiusMeters, r.driveGearRatio, r.statorCurrentLimitAmps, r.frictionCoefficient];
  if (positive.some((v) => !(v > 0)) || r.driveMotorCount < 1) throw new RangeError("robot numbers must be positive");
  return r;
}

export function ProjectPanel() {
  const settings = useEditor((s) => s.project.settings);
  return (
    <Section title="Project">
      <SelectField
        label="Red alliance"
        value={settings.redFrame}
        options={[
          { value: "RED_HALF_TURN", label: "Half-turn about the centre" },
          { value: "RED_MIRROR", label: "Mirror across the centre line" },
        ]}
        onChange={(redFrame) => editProject("Red frame", (p) => ({ ...p, settings: { ...p.settings, redFrame } }))}
      />
      <p className="hint">How the robot flips blue paths for red (the FieldFrame you pass the follow command). The red view uses the same flip.</p>
      <TextField
        label="PathName package"
        value={settings.javaPackage || "-"}
        validate={(v) => (v === "-" || isValidPackage(v) ? null : "A Java package, like frc.robot")}
        onCommit={(v) => editProject("Enum package", (p) => ({ ...p, settings: { ...p.settings, javaPackage: v === "-" ? "" : v } }))}
      />
      <p className="hint">
        Saving writes <code>PathName.java</code> there. Enter <code>-</code> to stop generating it.
      </p>
    </Section>
  );
}
