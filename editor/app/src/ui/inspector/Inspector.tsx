// The right-hand inspector. With something selected it shows only that item; otherwise the path's
// overview: Optimise path first, then collapsible cards in PathPlanner's order (waypoints open).

import { ArrowLeft, ChevronLeft, ChevronRight, Lock, Plus } from "lucide-react";
import { makeConstraintZone, makeRotationTarget } from "../../../../core/src/index.ts";
import { addConstraintZone, addFacePointZone, addRotationTarget, appendWaypoint, withSpec } from "../../model/edits.ts";
import { addNamedPoint, uniquePointName } from "../../model/namedPoints.ts";
import { round5 } from "../../model/positions.ts";
import { geometryOf } from "../../state/analysis.ts";
import { editPath, editProject, select, setTool } from "../../state/editor.ts";
import { Button, Card, formatNumber, IconButton, NumberField, SelectField } from "../controls.tsx";
import { OptimiseControls, OptimiserSettings } from "../panels/OptimisePanel.tsx";
import { toDegrees, useActivePath, useEditor } from "../panels/common.tsx";
import { Pane } from "../shell.tsx";
import {
  ConstraintZoneInspector,
  EventInspector,
  FaceZoneInspector,
  NEW_FACE_ZONE,
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
  // the same as the W tool, from the keyboard: a point 1 m on along the end's direction
  const appendPastEnd = () => {
    const g = geometryOf(path.spec);
    const end = g?.sample(g.lengthMeters);
    const last = path.spec.waypoints.at(-1)!;
    const [dx, dy] = end ? [end.tangentX, end.tangentY] : [1, 0];
    editPath("Add waypoint", (p) => appendWaypoint(p, round5(last.xMeters + dx), round5(last.yMeters + dy)), { kind: "waypoint", index: n });
  };
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
                {path.pinned[i] && <Lock size={12} aria-label="pinned" />}
                {path.links[i] && <span className="tag accent">{points.find((q) => q.id === path.links[i])?.name}</span>}
                <span className={set ? "heading-tag set" : "heading-tag"}>{set ? `${formatNumber(toDegrees(w.headingRadians), 1)}°` : "pass"}</span>
              </button>
            </li>
          );
        })}
      </ol>
      <div className="add-row">
        <Button onClick={appendPastEnd} label="Add a waypoint 1 m past the end" variant="ghost">
          <Plus size={14} />
          Waypoint at the end
        </Button>
        <Button onClick={() => setTool("waypoint")} variant="ghost" title="Draw: click the field to add points|W">
          Draw…
        </Button>
      </div>
    </Card>
  );
}

function ConstraintsCard() {
  const spec = useActivePath().spec;
  return (
    <Card title="Global constraints" cardKey="constraints">
      <div className="grid2">
        <NumberField
          label="Top speed"
          unit="m/s"
          value={spec.maxMetersPerSecond}
          blank="robot"
          blankValue={Infinity}
          min={0.01}
          onCommit={(v) => editPath("Top speed", (p) => withSpec(p, { maxMetersPerSecond: v }))}
        />
        <NumberField
          label="Max accel"
          unit="m/s²"
          value={spec.maxMetersPerSecondSquared}
          blank="grip"
          blankValue={Infinity}
          min={0.01}
          onCommit={(v) => editPath("Max acceleration", (p) => withSpec(p, { maxMetersPerSecondSquared: v }))}
        />
      </div>
      <p className="hint">Blank: the robot's own limits (motor speed, grip). Spin limits are the robot's, in Settings.</p>
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
  const last = spec.waypoints.length - 1;
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
          </li>
        ))}
      </ul>
      <div className="add-row">
        <Button
          variant="ghost"
          label="Add a heading at the end of the path"
          onClick={() =>
            editPath("Add rotation target", (p) => addRotationTarget(p, makeRotationTarget(last, Number.NaN, 0)), {
              kind: "rotation",
              index: spec.rotationTargets.length,
            })
          }
        >
          <Plus size={14} />
          Heading
        </Button>
        <Button variant="ghost" onClick={() => setTool("rotation")} title="Click the path to add a heading there|R">
          Place on the path…
        </Button>
      </div>
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
          </li>
        ))}
      </ul>
      <Button variant="ghost" onClick={() => setTool("event")} label="Place an event on the path" title="Then click the path|E">
        <Plus size={14} />
        Event
      </Button>
    </Card>
  );
}

function ConstraintZonesCard() {
  const spec = useActivePath().spec;
  const last = spec.waypoints.length - 1;
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
          </li>
        ))}
      </ul>
      <Button
        variant="ghost"
        label="Add constraint zone"
        onClick={() =>
          editPath("Add constraint zone", (p) => addConstraintZone(p, makeConstraintZone(0, Math.min(1, last), 1.5, Infinity)), {
            kind: "constraint",
            index: spec.constraintZones.length,
          })
        }
      >
        <Plus size={14} />
        Constraint zone
      </Button>
    </Card>
  );
}

function FaceZonesCard() {
  const spec = useActivePath().spec;
  const last = spec.waypoints.length - 1;
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
          </li>
        ))}
      </ul>
      <Button
        variant="ghost"
        label="Add face-point zone"
        onClick={() => editPath("Add face-point zone", (p) => addFacePointZone(p, NEW_FACE_ZONE(last)), { kind: "face", index: spec.facePointZones.length })}
      >
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
          </li>
        ))}
      </ul>
      <Button
        variant="ghost"
        label="New point at the field centre"
        onClick={() => {
          let id = "";
          editProject("Add point", (pr) => {
            const [withPoint, point] = addNamedPoint(pr, uniquePointName(pr), 0, 0);
            id = point.id;
            return withPoint;
          });
          if (id) select({ kind: "point", id });
        }}
      >
        <Plus size={14} />
        Point
      </Button>
      <p className="hint">Field points shared across paths: moving one moves every waypoint linked to it. Link a waypoint from its row.</p>
    </Card>
  );
}

function Overview() {
  return (
    <>
      <div className="inspector-top">
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

