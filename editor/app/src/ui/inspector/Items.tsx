// The inspector's view of one selected item: a waypoint, a heading along the path, an event, a zone,
// an obstacle or a named point. Only that item's properties, with Delete at the bottom.

import { CornerDownRight, Lock, MapPin, Trash2 } from "lucide-react";
import { DEFAULT_BLEND_METERS, makeConstraintZone, makeFacePointZone, makePathEvent, makeRotationTarget } from "../../../../core/src/index.ts";
import {
  deleteWaypoint,
  addWaypointAfter,
  moveWaypoint,
  removeConstraintZone,
  removeEvent,
  removeFacePointZone,
  removeRotationTarget,
  setHeading,
  setPinned,
  setTangent,
  updateConstraintZone,
  updateEvent,
  updateFacePointZone,
  updateRotationTarget,
} from "../../model/edits.ts";
import { addNamedPoint, linkCount, linkToPoint, moveNamedPoint, removeNamedPoint, renameNamedPoint, uniquePointName } from "../../model/namedPoints.ts";
import { eventNames } from "../../model/project.ts";
import { round5, round5Radians } from "../../model/positions.ts";
import { newPath } from "../../state/actions.ts";
import { geometryOf, playback } from "../../state/analysis.ts";
import { editPath, editProject, select, setView, type Selection } from "../../state/editor.ts";
import { Button, formatNumber, IconButton, NumberField, SelectField, Switch, TextField } from "../controls.tsx";
import { waypointPose } from "../draw.ts";
import { DEGREES, PositionField, toDegrees, useActivePath, useAnalysis, useEditor } from "../panels/common.tsx";


function DeleteButton({ label, onClick, disabled }: { label: string; onClick: () => void; disabled?: boolean }) {
  return (
    <Button variant="danger" onClick={onClick} disabled={disabled} label={label} className="block">
      <Trash2 size={14} />
      {label}
    </Button>
  );
}

export function WaypointInspector({ index }: { index: number }) {
  const path = useActivePath();
  const points = useEditor((s) => s.project.namedPoints);
  const shapeCurve = useEditor((s) => s.view.shapeCurve);
  const analysisResult = useAnalysis();
  const w = path.spec.waypoints[index];
  if (!w) return null;
  const n = path.spec.waypoints.length;
  const link = path.links[index];
  const isEnd = index === 0 || index === n - 1;
  const passThrough = Number.isNaN(w.headingRadians);
  /** The plan's heading here, in 15° steps: where a newly set heading starts. */
  const plannedHeading = () => {
    const h = waypointPose({ active: path, analysis: analysisResult }, index).heading;
    return round5Radians(Math.round(h / (Math.PI / 12)) * (Math.PI / 12));
  };
  // a linked waypoint follows its point, so moving it moves the point (and every waypoint on it)
  const moveTo = (x: number, y: number) =>
    link ? editProject("Move point", (p) => moveNamedPoint(p, link, x, y)) : editPath("Move waypoint", (p) => moveWaypoint(p, index, x, y));
  // the keyboard's double-click: halfway to the next waypoint, or 1 m on past the end
  const insertAfter = () => {
    const g = geometryOf(path.spec);
    let at: [number, number];
    if (g && index < n - 1) {
      const mid = g.sample(g.sAtWaypoint(index + 0.5));
      at = [mid.xMeters, mid.yMeters];
    } else {
      const end = g?.sample(g.lengthMeters);
      const h = Number.isNaN(w.headingRadians) ? 0 : w.headingRadians;
      const [dx, dy] = end ? [end.tangentX, end.tangentY] : [Math.cos(h), Math.sin(h)];
      at = [w.xMeters + dx, w.yMeters + dy];
    }
    editPath("Add waypoint", (p) => addWaypointAfter(p, index, round5(at[0]), round5(at[1])), { kind: "waypoint", index: index + 1 });
  };
  return (
    <div className="inspector-body" role="group" aria-label={`Waypoint ${index}`}>
      <div className="grid2">
        <NumberField label="x" unit="m" value={w.xMeters} onCommit={(v) => moveTo(round5(v), w.yMeters)} />
        <NumberField label="y" unit="m" value={w.yMeters} onCommit={(v) => moveTo(w.xMeters, round5(v))} />
        <NumberField
          label="Heading"
          unit="°"
          value={toDegrees(w.headingRadians)}
          blank={isEnd ? "" : "pass-through"}
          blankValue={isEnd ? undefined : Number.NaN}
          step={15}
          testId="waypoint-heading"
          title="Where the robot faces here (a swerve's heading is its own). Blank: pass through"
          onCommit={(v) => editPath("Set heading", (p) => setHeading(p, index, Number.isNaN(v) ? v : round5Radians(v * DEGREES)))}
        />
        <div className="field" style={{ justifyContent: "flex-end" }}>
          <Switch
            label="Pass-through"
            checked={passThrough}
            disabled={isEnd}
            testId="waypoint-pass-through"
            title={isEnd ? "The first and last waypoints always have a heading" : "No heading here: the robot blends between the set ones"}
            onChange={(on) => editPath(on ? "Pass through" : "Set heading", (p) => setHeading(p, index, on ? Number.NaN : plannedHeading()))}
          />
        </div>
      </div>
      <Switch
        label={
          <>
            <Lock size={12} style={{ verticalAlign: -1 }} /> Locked (the optimiser never moves it)
          </>
        }
        checked={path.pinned[index] === true}
        testId="waypoint-lock"
        onChange={(on) => editPath(on ? "Lock" : "Unlock", (p) => setPinned(p, index, on))}
      />
      <div className="position">
        <SelectField
          label="Named point"
          value={link ?? ""}
          options={[{ value: "", label: "none" }, ...points.map((p) => ({ value: p.id, label: p.name }))]}
          onChange={(id) => editProject("Link waypoint", (p) => linkToPoint(p, path.id, index, id || null))}
        />
        {!link && (
          <IconButton
            label={`New point at waypoint ${index}`}
            onClick={() => {
              let id = "";
              editProject("Add point", (pr) => {
                const [withPoint, point] = addNamedPoint(pr, uniquePointName(pr), w.xMeters, w.yMeters);
                id = point.id;
                return linkToPoint(withPoint, path.id, index, point.id);
              });
              if (id) select({ kind: "point", id });
            }}
          >
            <MapPin size={15} />
          </IconButton>
        )}
      </div>
      <Switch
        label="Shape the curve by hand"
        checked={shapeCurve}
        testId="shape-curve"
        title="Show this waypoint's tangent handle; otherwise travel direction is automatic (and the optimiser shapes it)"
        onChange={(on) => setView({ shapeCurve: on })}
      />
      {shapeCurve && (
        <div className="grid2">
          <NumberField
            label="Travel direction"
            unit="°"
            value={toDegrees(w.tangentRadians)}
            blank="auto"
            blankValue={Number.NaN}
            title="Direction of travel through this waypoint; blank = automatic"
            onCommit={(v) => editPath("Change tangent", (p) => setTangent(p, index, Number.isNaN(v) ? v : round5Radians(v * DEGREES), w.tangentScale))}
          />
          <NumberField
            label="Tangent scale"
            value={w.tangentScale}
            min={0.01}
            title="Above 1 holds the direction longer"
            onCommit={(v) => editPath("Change tangent", (p) => setTangent(p, index, w.tangentRadians, v))}
          />
          <Button
            className="span2"
            variant="ghost"
            disabled={Number.isNaN(w.tangentRadians) && w.tangentScale === 1}
            onClick={() => editPath("Automatic tangent", (p) => setTangent(p, index, Number.NaN, 1))}
          >
            Reset to auto
          </Button>
        </div>
      )}
      <div className="row">
        <Button onClick={insertAfter} label={`Add a waypoint after waypoint ${index}`} testId="add-after">
          <CornerDownRight size={14} />
          Add after
        </Button>
      </div>
      <DeleteButton label="Delete waypoint" disabled={n <= 2} onClick={() => editPath("Delete waypoint", (p) => deleteWaypoint(p, index), null)} />
    </div>
  );
}

export function RotationInspector({ index }: { index: number }) {
  const path = useActivePath();
  const t = path.spec.rotationTargets[index];
  if (!t) return null;
  return (
    <div className="inspector-body" role="group" aria-label={`Heading ${index + 1}`}>
      <p className="hint">The robot turns to this heading by here, blending from the one before.</p>
      <PositionField
        label="At"
        waypoint={t.waypoint}
        meters={t.meters}
        onCommit={(w, m) => editPath("Move rotation target", (p) => updateRotationTarget(p, index, makeRotationTarget(w, m, t.headingRadians)))}
      />
      <NumberField
        label="Heading"
        unit="°"
        step={15}
        value={toDegrees(t.headingRadians)}
        onCommit={(v) => editPath("Turn rotation target", (p) => updateRotationTarget(p, index, makeRotationTarget(t.waypoint, t.meters, round5Radians(v * DEGREES))))}
      />
      <DeleteButton label={`Delete heading ${index + 1}`} onClick={() => editPath("Delete heading", (p) => removeRotationTarget(p, index), null)} />
    </div>
  );
}

export function EventInspector({ index }: { index: number }) {
  const path = useActivePath();
  const project = useEditor((s) => s.project);
  const a = useAnalysis();
  const e = path.spec.events[index];
  if (!e) return null;
  const placed = a?.events[index];
  return (
    <div className="inspector-body" role="group" aria-label={`Event ${e.name}`} data-testid={`event-${index}`}>
      <TextField
        label="Name"
        value={e.name}
        options={eventNames(project)}
        testId={`event-name-${index}`}
        autoFocus={e.name === "event"}
        onCommit={(name) => editPath("Rename event", (p) => updateEvent(p, index, makePathEvent(name, e.waypoint, e.meters)))}
      />
      <PositionField
        label="At"
        waypoint={e.waypoint}
        meters={e.meters}
        testId={`event-at-${index}`}
        onCommit={(w, m) => editPath("Move event", (p) => updateEvent(p, index, makePathEvent(e.name, w, m)))}
      />
      {placed && (
        <p className="stats">
          {formatNumber(placed.s, 2)} m along, fires at{" "}
          <button type="button" className="link" onClick={() => playback.set({ t: placed.t, playing: false })}>
            {formatNumber(placed.t, 2)} s
          </button>
        </p>
      )}
      <p className="hint">
        Robot code binds each name once with <code>PathLibrary.bindEvent</code>.
      </p>
      <DeleteButton label={`Delete event ${e.name}`} onClick={() => editPath("Delete event", (p) => removeEvent(p, index), null)} />
    </div>
  );
}

export function FaceZoneInspector({ index }: { index: number }) {
  const path = useActivePath();
  const z = path.spec.facePointZones[index];
  if (!z) return null;
  const update = (label: string, changes: Partial<typeof z>) =>
    editPath(label, (p) => {
      const n = { ...z, ...changes };
      return updateFacePointZone(p, index, makeFacePointZone(n.fromWaypoint, n.toWaypoint, n.point, n.blendMeters));
    });
  return (
    <div className="inspector-body" role="group" aria-label={`Face-point zone ${index + 1}`}>
      <p className="hint">Between two waypoint positions the robot faces a field point, easing in and out over the blend.</p>
      <div className="grid2">
        <NumberField label="From" unit="wp" value={z.fromWaypoint} onCommit={(v) => update("Change zone", { fromWaypoint: v })} />
        <NumberField label="To" unit="wp" value={z.toWaypoint} onCommit={(v) => update("Change zone", { toWaypoint: v })} />
        <NumberField label="Point x" unit="m" value={z.point.x} onCommit={(v) => update("Move face point", { point: { x: v, y: z.point.y } })} />
        <NumberField label="Point y" unit="m" value={z.point.y} onCommit={(v) => update("Move face point", { point: { x: z.point.x, y: v } })} />
        <NumberField
          label="Blend"
          unit="m"
          value={z.blendMeters}
          min={0}
          title="Distance to ease in and out; short blends make fast turns that slow the plan"
          onCommit={(v) => update("Change blend", { blendMeters: v })}
        />
      </div>
      <DeleteButton label={`Delete face-point zone ${index + 1}`} onClick={() => editPath("Delete zone", (p) => removeFacePointZone(p, index), null)} />
    </div>
  );
}

export function ConstraintZoneInspector({ index }: { index: number }) {
  const path = useActivePath();
  const z = path.spec.constraintZones[index];
  if (!z) return null;
  const update = (changes: Partial<typeof z>) =>
    editPath("Change constraint zone", (p) => {
      const n = { ...z, ...changes };
      return updateConstraintZone(p, index, makeConstraintZone(n.fromWaypoint, n.toWaypoint, n.maxMetersPerSecond, n.maxMetersPerSecondSquared));
    });
  return (
    <div className="inspector-body" role="group" aria-label={`Constraint zone ${index + 1}`}>
      <div className="grid2">
        <NumberField label="From" unit="wp" value={z.fromWaypoint} onCommit={(v) => update({ fromWaypoint: v })} />
        <NumberField label="To" unit="wp" value={z.toWaypoint} onCommit={(v) => update({ toWaypoint: v })} />
        <NumberField label="Top speed" unit="m/s" blank="none" blankValue={Infinity} min={0.01} value={z.maxMetersPerSecond} onCommit={(v) => update({ maxMetersPerSecond: v })} />
        <NumberField
          label="Max accel"
          unit="m/s²"
          blank="none"
          blankValue={Infinity}
          min={0.01}
          value={z.maxMetersPerSecondSquared}
          onCommit={(v) => update({ maxMetersPerSecondSquared: v })}
        />
      </div>
      <p className="hint">Waypoint positions run 0 to {path.spec.waypoints.length - 1}.</p>
      <DeleteButton label={`Delete constraint zone ${index + 1}`} onClick={() => editPath("Delete zone", (p) => removeConstraintZone(p, index), null)} />
    </div>
  );
}

export function ObstacleInspector({ index }: { index: number }) {
  const o = useEditor((s) => s.project.field.obstacles[index]);
  if (!o) return null;
  return (
    <div className="inspector-body" role="group" aria-label={`Obstacle ${o.name}`}>
      <TextField
        label="Name"
        value={o.name}
        onCommit={(name) =>
          editProject("Rename obstacle", (p) => ({ ...p, field: { ...p.field, obstacles: p.field.obstacles.map((x, i) => (i === index ? { ...x, name } : x)) } }))
        }
      />
      <p className="stats">{o.polygon.length} corners. Drag them on the field; it is used as its convex hull.</p>
      <DeleteButton
        label="Delete obstacle"
        onClick={() => editProject("Delete obstacle", (p) => ({ ...p, field: { ...p.field, obstacles: p.field.obstacles.filter((_, i) => i !== index) } }), null)}
      />
    </div>
  );
}

export function PointInspector({ id }: { id: string }) {
  const project = useEditor((s) => s.project);
  const p = project.namedPoints.find((q) => q.id === id);
  if (!p) return null;
  const n = linkCount(project, p.id);
  return (
    <div className="inspector-body" role="group" aria-label={`Point ${p.name}`}>
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
      <p className="stats">{n === 0 ? "No waypoints linked" : n === 1 ? "1 waypoint linked" : `${n} waypoints linked`}. Moving it moves every one, in every path.</p>
      <Button onClick={() => newPath({ x: p.x, y: p.y, heading: 0, pointId: p.id })} label={`Start a new path at ${p.name}`}>
        Start a path here
      </Button>
      <DeleteButton label={`Delete point ${p.name}`} onClick={() => editProject("Delete point", (pr) => removeNamedPoint(pr, p.id), null)} />
    </div>
  );
}

/** The inspector title for a selection. */
export function selectionTitle(sel: Selection, names: { events: readonly { name: string }[]; obstacles: readonly { name: string }[]; points: readonly { id: string; name: string }[] }): string {
  switch (sel.kind) {
    case "waypoint":
      return `Waypoint ${sel.index}`;
    case "rotation":
      return `Heading ${sel.index + 1}`;
    case "event":
      return `Event: ${names.events[sel.index]?.name ?? ""}`;
    case "face":
      return `Point-towards zone ${sel.index + 1}`;
    case "constraint":
      return `Constraint zone ${sel.index + 1}`;
    case "obstacle":
      return `Obstacle: ${names.obstacles[sel.index]?.name ?? ""}`;
    case "point":
      return `Point: ${names.points.find((p) => p.id === sel.id)?.name ?? ""}`;
  }
}

export const NEW_FACE_ZONE = (last: number) => makeFacePointZone(Math.max(0, last - 1), last, { x: 0, y: 0 }, Math.max(1.0, DEFAULT_BLEND_METERS));
