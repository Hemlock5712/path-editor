// The right-hand inspector. With something selected it shows only that item; otherwise the path's
// overview: Optimise path first, then collapsible cards in PathPlanner's order (waypoints open).

import { ArrowLeft, ChevronLeft, ChevronRight, Link2, Lock, Plus, Trash2 } from "lucide-react";
import { tractionMetersPerSecondSquared, wheelSpeedCapMetersPerSecond } from "../../../../core/src/index.ts";
import { withSpec } from "../../model/edits.ts";
import { deleteItem, unlinkActivePath } from "../../state/actions.ts";
import { TRANSFORM_LABELS } from "../../model/mirror.ts";
import { editPath, select } from "../../state/editor.ts";
import { startFlow } from "../../state/flow.ts";
import { Button, Card, formatNumber, IconButton, NumberField, SelectField } from "../controls.tsx";
import { OptimiseControls, OptimiserSettings } from "../panels/OptimisePanel.tsx";
import { toDegrees, useActivePath, useEditor } from "../panels/common.tsx";
import { Pane } from "../shell.tsx";
import { SimCard } from "../../sim/SimPanel.tsx";
import {
  ConstraintZoneInspector,
  EventInspector,
  FaceZoneInspector,
  ObstacleInspector,
  PointInspector,
  RotationInspector,
  selectionTitle,
  WaypointInspector,
} from "./Items.tsx";

function WaypointsCard() {
  const path = useActivePath();
  const points = useEditor((s) => s.project.namedPoints);
  const n = path.spec.waypoints.length;
  return (
    <Card title="Waypoints" open count={n} cardKey="waypoints">
      <ol className="item-list" aria-label="Waypoints">
        {path.spec.waypoints.map((w, i) => {
          const set = !Number.isNaN(w.headingRadians);
          return (
            <li key={i}>
              <button type="button" className="item" onClick={() => select({ kind: "waypoint", index: i })}>
                <span className="idx">{i}</span>
                <span className="main">
                  ({formatNumber(w.xMeters, 2)}, {formatNumber(w.yMeters, 2)})
                </span>
                {path.pinned[i] && <Lock size={12} aria-label="locked" />}
                {path.links[i] && <span className="tag accent">{points.find((q) => q.id === path.links[i])?.name}</span>}
                <span className={set ? "heading-tag set" : "heading-tag"}>{set ? `${formatNumber(toDegrees(w.headingRadians), 1)}°` : "pass"}</span>
              </button>
            </li>
          );
        })}
      </ol>
      <p className="hint" data-testid="add-waypoint-hint">
        {n === 0
          ? "Double-click the field to place the start, then the end."
          : n === 1
            ? "Double-click the field to place the end."
            : "Select a waypoint, then double-click the field to add one after it."}
      </p>
    </Card>
  );
}

function ConstraintsCard() {
  const spec = useActivePath().spec;
  const robot = useEditor((s) => s.project.robot.config);
  return (
    <Card title="Global constraints" cardKey="constraints">
      <div className="grid2">
        <NumberField
          label="Top speed"
          unit="m/s"
          value={spec.maxMetersPerSecond}
          blank={`robot's (${formatNumber(wheelSpeedCapMetersPerSecond(robot), 2)})`}
          blankValue={Infinity}
          min={0.01}
          onCommit={(v) => editPath("Top speed", (p) => withSpec(p, { maxMetersPerSecond: v }))}
        />
        <NumberField
          label="Max accel"
          unit="m/s²"
          value={spec.maxMetersPerSecondSquared}
          blank={`robot's (${formatNumber(tractionMetersPerSecondSquared(robot), 1)})`}
          blankValue={Infinity}
          min={0.01}
          onCommit={(v) => editPath("Max acceleration", (p) => withSpec(p, { maxMetersPerSecondSquared: v }))}
        />
      </div>
      <p className="hint">Blank: the robot's own limits, from Settings, Robot. Set one only to make this path slower.</p>
    </Card>
  );
}

function EndStateCard() {
  const spec = useActivePath().spec;
  return (
    <Card title="Start and end state" cardKey="end-state">
      <div className="grid2">
        <NumberField
          label="Start speed"
          unit="m/s"
          value={spec.startMetersPerSecond}
          min={0}
          title="Above 0 when the path before hands its speed on"
          onCommit={(v) => editPath("Start speed", (p) => withSpec(p, { startMetersPerSecond: v }))}
        />
        <NumberField
          label="End speed"
          unit="m/s"
          value={spec.endMetersPerSecond}
          min={0}
          title="Above 0 is a rolling end: the robot hands its speed to the next path"
          onCommit={(v) => editPath("End speed", (p) => withSpec(p, { endMetersPerSecond: v }))}
        />
      </div>
      <p className="hint">The end pose is the last waypoint's (its heading is set in its row).</p>
    </Card>
  );
}

function RotationCard() {
  const path = useActivePath();
  const spec = path.spec;
  return (
    <Card title="Rotation targets" count={spec.rotationTargets.length} cardKey="rotation">
      <SelectField
        label="Heading between set headings"
        value={spec.headingMode}
        options={[
          { value: "hold", label: "Hold, and blend between them (swerve)" },
          { value: "faceTravel", label: "Face the direction of travel" },
        ]}
        onChange={(headingMode) => editPath("Heading mode", (p) => withSpec(p, { headingMode }))}
        testId="heading-mode"
      />
      <ul className="item-list" aria-label="Rotation targets">
        {spec.rotationTargets.map((t, i) => (
          <li key={i}>
            <button type="button" className="item" onClick={() => select({ kind: "rotation", index: i })}>
              <span className="idx">{i + 1}</span>
              <span className="main">{Number.isNaN(t.waypoint) ? `${formatNumber(t.meters, 2)} m` : `waypoint ${formatNumber(t.waypoint, 2)}`}</span>
              <span className="heading-tag set">{formatNumber(toDegrees(t.headingRadians), 1)}°</span>
            </button>
            <RowDelete label={`Delete heading ${i + 1}`} onClick={() => deleteItem({ kind: "rotation", index: i })} />
          </li>
        ))}
      </ul>
      <Button variant="ghost" onClick={() => startFlow("rotation")} label="Add a heading along the path" title="Click the path, then turn the robot there|R" testId="add-rotation">
        <Plus size={14} />
        Heading
      </Button>
      <p className="hint">Waypoints carry their own headings; these turn the robot partway along.</p>
    </Card>
  );
}

function EventsCard() {
  const events = useActivePath().spec.events;
  return (
    <Card title="Event markers" count={events.length} cardKey="events">
      <ul className="item-list" aria-label="Event markers">
        {events.map((e, i) => (
          <li key={i}>
            <button type="button" className="item" onClick={() => select({ kind: "event", index: i })} data-testid={`event-item-${i}`}>
              <span className="idx">{i + 1}</span>
              <span className="main">{e.name}</span>
              <span className="heading-tag">{Number.isNaN(e.waypoint) ? `${formatNumber(e.meters, 2)} m` : `wp ${formatNumber(e.waypoint, 2)}`}</span>
            </button>
            <RowDelete label={`Delete event ${e.name}`} onClick={() => deleteItem({ kind: "event", index: i })} />
          </li>
        ))}
      </ul>
      <Button variant="ghost" onClick={() => startFlow("event")} label="Add an event" title="Click the path, then name it|M" testId="add-event">
        <Plus size={14} />
        Event
      </Button>
    </Card>
  );
}

function ConstraintZonesCard() {
  const spec = useActivePath().spec;
  return (
    <Card title="Constraint zones" count={spec.constraintZones.length} cardKey="constraint-zones">
      <ul className="item-list" aria-label="Constraint zones">
        {spec.constraintZones.map((z, i) => (
          <li key={i}>
            <button type="button" className="item" onClick={() => select({ kind: "constraint", index: i })}>
              <span className="idx">{i + 1}</span>
              <span className="main">
                wp {formatNumber(z.fromWaypoint, 2)} to {formatNumber(z.toWaypoint, 2)}
              </span>
              <span className="heading-tag">{Number.isFinite(z.maxMetersPerSecond) ? `${formatNumber(z.maxMetersPerSecond, 2)} m/s` : ""}</span>
            </button>
            <RowDelete label={`Delete constraint zone ${i + 1}`} onClick={() => deleteItem({ kind: "constraint", index: i })} />
          </li>
        ))}
      </ul>
      <Button variant="ghost" label="Add a constraint zone" onClick={() => startFlow("constraint")} title="Click its start and end on the path, then set its speed limit|Z" testId="add-constraint">
        <Plus size={14} />
        Constraint zone
      </Button>
    </Card>
  );
}

function FaceZonesCard() {
  const spec = useActivePath().spec;
  return (
    <Card title="Point-towards zones" count={spec.facePointZones.length} cardKey="face-zones">
      <ul className="item-list" aria-label="Point-towards zones">
        {spec.facePointZones.map((z, i) => (
          <li key={i}>
            <button type="button" className="item" onClick={() => select({ kind: "face", index: i })}>
              <span className="idx">{i + 1}</span>
              <span className="main">
                wp {formatNumber(z.fromWaypoint, 2)} to {formatNumber(z.toWaypoint, 2)}
              </span>
              <span className="heading-tag">
                ({formatNumber(z.point.x, 1)}, {formatNumber(z.point.y, 1)})
              </span>
            </button>
            <RowDelete label={`Delete point-towards zone ${i + 1}`} onClick={() => deleteItem({ kind: "face", index: i })} />
          </li>
        ))}
      </ul>
      <Button variant="ghost" label="Add a point-towards zone" onClick={() => startFlow("face")} title="Click its start and end on the path, then the point to face|T" testId="add-face">
        <Plus size={14} />
        Point-towards zone
      </Button>
    </Card>
  );
}

function PointsCard() {
  const points = useEditor((s) => s.project.namedPoints);
  return (
    <Card title="Named points" count={points.length} cardKey="points">
      <ul className="item-list" aria-label="Named points">
        {points.map((p) => (
          <li key={p.id}>
            <button type="button" className="item" onClick={() => select({ kind: "point", id: p.id })}>
              <span className="main">{p.name}</span>
              <span className="heading-tag">
                ({formatNumber(p.x, 2)}, {formatNumber(p.y, 2)})
              </span>
            </button>
            <RowDelete label={`Delete point ${p.name}`} onClick={() => deleteItem({ kind: "point", id: p.id })} />
          </li>
        ))}
      </ul>
      <Button variant="ghost" label="Add a named point" onClick={() => startFlow("point")} title="Click the spot, then name it|N" testId="add-point">
        <Plus size={14} />
        Point
      </Button>
      <p className="hint">Field points shared across paths: moving one moves every waypoint linked to it. Link a waypoint from its row.</p>
    </Card>
  );
}

/** The trash icon at the end of an item's row. */
function RowDelete({ label, onClick }: { label: string; onClick: () => void }) {
  return (
    <IconButton label={label} onClick={onClick} className="row-delete">
      <Trash2 size={13} />
    </IconButton>
  );
}

/** On a linked copy: what it follows, and Unlink. */
function LinkedNote() {
  const path = useActivePath();
  const paths = useEditor((s) => s.project.paths);
  const source = path.mirrorOf ? paths.find((p) => p.id === path.mirrorOf!.id) : undefined;
  if (!source) return null;
  return (
    <p className="link-badge banner" data-testid="linked-note">
      <Link2 size={12} />
      A linked copy of {source.name} ({TRANSFORM_LABELS[path.mirrorOf!.transform].toLowerCase()}): it follows {source.name}, so edit that one.
      <button type="button" className="link" onClick={() => unlinkActivePath(path.id)}>
        Unlink to edit it on its own
      </button>
    </p>
  );
}

function Overview() {
  return (
    <>
      <div className="inspector-top">
        <LinkedNote />
        <OptimiseControls />
      </div>
      <WaypointsCard />
      <ConstraintsCard />
      <EndStateCard />
      <RotationCard />
      <EventsCard />
      <ConstraintZonesCard />
      <FaceZonesCard />
      <PointsCard />
      <SimCard />
      <Card title="Optimiser settings" cardKey="optimiser">
        <OptimiserSettings />
      </Card>
    </>
  );
}

export function Inspector() {
  const sel = useEditor((s) => s.selection);
  const path = useActivePath();
  const obstacles = useEditor((s) => s.project.field.obstacles);
  const points = useEditor((s) => s.project.namedPoints);
  if (!sel) {
    return (
      <Pane title="Path">
        <Overview />
      </Pane>
    );
  }
  const n = sel.kind === "waypoint" ? path.spec.waypoints.length : 0;
  return (
    <>
      <div className="inspector-title">
        <IconButton label="Back to the path overview" shortcut="Esc" onClick={() => select(null)} testId="inspector-back">
          <ArrowLeft size={16} />
        </IconButton>
        <h2>{selectionTitle(sel, { events: path.spec.events, obstacles, points })}</h2>
        {sel.kind === "waypoint" && (
          <>
            <IconButton label="Previous waypoint" disabled={sel.index <= 0} onClick={() => select({ kind: "waypoint", index: sel.index - 1 })}>
              <ChevronLeft size={16} />
            </IconButton>
            <IconButton label="Next waypoint" disabled={sel.index >= n - 1} onClick={() => select({ kind: "waypoint", index: sel.index + 1 })} testId="next-waypoint">
              <ChevronRight size={16} />
            </IconButton>
          </>
        )}
      </div>
      <div className="pane-body">
        {sel.kind === "waypoint" && <WaypointInspector index={sel.index} />}
        {sel.kind === "rotation" && <RotationInspector index={sel.index} />}
        {sel.kind === "event" && <EventInspector index={sel.index} />}
        {sel.kind === "face" && <FaceZoneInspector index={sel.index} />}
        {sel.kind === "constraint" && <ConstraintZoneInspector index={sel.index} />}
        {sel.kind === "obstacle" && <ObstacleInspector index={sel.index} />}
        {sel.kind === "point" && <PointInspector id={sel.id} />}
      </div>
    </>
  );
}

