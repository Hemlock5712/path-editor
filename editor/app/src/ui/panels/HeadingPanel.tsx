// Headings along the path (rotation targets), face-point zones and constraint zones of the active path.

import {
  makeConstraintZone,
  makeFacePointZone,
  makeRotationTarget,
  DEFAULT_BLEND_METERS,
} from "../../../../core/src/index.ts";
import {
  addConstraintZone,
  addFacePointZone,
  addRotationTarget,
  removeConstraintZone,
  removeFacePointZone,
  removeRotationTarget,
  updateConstraintZone,
  updateFacePointZone,
  updateRotationTarget,
} from "../../model/edits.ts";
import { round5Radians } from "../../model/positions.ts";
import { editPath, select } from "../../state/editor.ts";
import { IconButton, NumberField, Section, formatNumber } from "../controls.tsx";
import { DEGREES, PositionField, toDegrees, useActivePath, useEditor } from "./common.tsx";

export function RotationPanel() {
  const path = useActivePath();
  const selection = useEditor((s) => s.selection);
  const targets = path.spec.rotationTargets;
  const last = path.spec.waypoints.length - 1;
  return (
    <Section title="Headings along the path" count={targets.length}>
      <p className="hint">
        Headings between waypoints (set a waypoint's own heading in its row). The Rotation tool (R) adds one where you click the path; drag
        its robot along the path, or its knob to turn it.
      </p>
      {targets.map((t, i) => {
        const sel = selection?.kind === "rotation" && selection.index === i;
        return (
          <fieldset key={i} className={sel ? "card selected" : "card"} onFocus={() => !sel && select({ kind: "rotation", index: i })}>
            <legend>Heading {i + 1}</legend>
            <PositionField
              label="At"
              waypoint={t.waypoint}
              meters={t.meters}
              onCommit={(w, m) => editPath("Move rotation target", (p) => updateRotationTarget(p, i, makeRotationTarget(w, m, t.headingRadians)))}
            />
            <NumberField
              label="Heading"
              unit="°"
              value={toDegrees(t.headingRadians)}
              onCommit={(v) =>
                editPath("Turn rotation target", (p) => updateRotationTarget(p, i, makeRotationTarget(t.waypoint, t.meters, round5Radians(v * DEGREES))))
              }
            />
            <IconButton label={`Delete heading ${i + 1}`} onClick={() => editPath("Delete heading", (p) => removeRotationTarget(p, i), null)}>
              Delete
            </IconButton>
          </fieldset>
        );
      })}
      <IconButton
        label="Add a heading at the end of the path"
        onClick={() =>
          editPath("Add rotation target", (p) => addRotationTarget(p, makeRotationTarget(last, Number.NaN, 0)), {
            kind: "rotation",
            index: targets.length,
          })
        }
      >
        + Heading at the end
      </IconButton>
    </Section>
  );
}

export function ZonesPanel() {
  const path = useActivePath();
  const selection = useEditor((s) => s.selection);
  const spec = path.spec;
  const last = spec.waypoints.length - 1;
  return (
    <Section title="Zones" count={spec.facePointZones.length + spec.constraintZones.length}>
      <h3 className="subhead">Face a point</h3>
      <p className="hint">Between two waypoint positions the robot faces a field point, easing in and out over the blend.</p>
      {spec.facePointZones.map((z, i) => {
        const sel = selection?.kind === "face" && selection.index === i;
        const update = (label: string, changes: Partial<typeof z>) =>
          editPath(label, (p) => {
            const n = { ...z, ...changes };
            return updateFacePointZone(p, i, makeFacePointZone(n.fromWaypoint, n.toWaypoint, n.point, n.blendMeters));
          });
        return (
          <fieldset key={i} className={sel ? "card selected" : "card"} onFocus={() => !sel && select({ kind: "face", index: i })}>
            <legend>Face-point zone {i + 1}</legend>
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
            <IconButton label={`Delete face-point zone ${i + 1}`} onClick={() => editPath("Delete zone", (p) => removeFacePointZone(p, i), null)}>
              Delete
            </IconButton>
          </fieldset>
        );
      })}
      <IconButton
        label="Add face-point zone"
        onClick={() =>
          editPath(
            "Add face-point zone",
            (p) => addFacePointZone(p, makeFacePointZone(Math.max(0, last - 1), last, { x: 0, y: 0 }, Math.max(1.0, DEFAULT_BLEND_METERS))),
            { kind: "face", index: spec.facePointZones.length },
          )
        }
      >
        + Face-point zone
      </IconButton>

      <h3 className="subhead">Constraint zones</h3>
      {spec.constraintZones.map((z, i) => {
        const sel = selection?.kind === "constraint" && selection.index === i;
        const update = (changes: Partial<typeof z>) =>
          editPath("Change constraint zone", (p) => {
            const n = { ...z, ...changes };
            return updateConstraintZone(p, i, makeConstraintZone(n.fromWaypoint, n.toWaypoint, n.maxMetersPerSecond, n.maxMetersPerSecondSquared));
          });
        return (
          <fieldset key={i} className={sel ? "card selected" : "card"} onFocus={() => !sel && select({ kind: "constraint", index: i })}>
            <legend>Constraint zone {i + 1}</legend>
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
            <IconButton label={`Delete constraint zone ${i + 1}`} onClick={() => editPath("Delete zone", (p) => removeConstraintZone(p, i), null)}>
              Delete
            </IconButton>
          </fieldset>
        );
      })}
      <IconButton
        label="Add constraint zone"
        onClick={() =>
          editPath("Add constraint zone", (p) => addConstraintZone(p, makeConstraintZone(0, Math.min(1, last), 1.5, Infinity)), {
            kind: "constraint",
            index: spec.constraintZones.length,
          })
        }
      >
        + Constraint zone
      </IconButton>
      <p className="hint">Waypoint positions run 0 to {formatNumber(last)}.</p>
    </Section>
  );
}
