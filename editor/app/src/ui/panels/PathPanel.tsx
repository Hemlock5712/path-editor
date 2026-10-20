// The active path's settings, its waypoints, and the selected waypoint's details.

import { round5, round5Radians } from "../../model/positions.ts";
import { appendWaypoint, deleteWaypoint, insertWaypoint, moveWaypoint, setTangent, withSpec } from "../../model/edits.ts";
import { geometryOf } from "../../state/analysis.ts";
import { isValidPathName } from "../../model/project.ts";
import { linkToPoint, moveNamedPoint } from "../../model/namedPoints.ts";
import { editPath, editProject, select } from "../../state/editor.ts";
import { IconButton, NumberField, Section, TextField, formatNumber } from "../controls.tsx";
import { DEGREES, toDegrees, useActivePath, useAnalysis, useEditor } from "./common.tsx";

export function PathPanel() {
  const path = useActivePath();
  const names = useEditor((s) => s.project.paths);
  const a = useAnalysis();
  const spec = path.spec;
  return (
    <Section title="Path" open>
      <TextField
        label="Name"
        value={path.name}
        testId="path-name"
        validate={(v) =>
          !isValidPathName(v)
            ? "Letters, digits, _ and -, starting with a letter"
            : names.some((p) => p.id !== path.id && p.name === v)
              ? "Another path has that name"
              : null
        }
        onCommit={(name) => editPath("Rename path", (p) => ({ ...p, name }))}
      />
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
        <NumberField
          label="Start speed"
          unit="m/s"
          value={spec.startMetersPerSecond}
          min={0}
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
      <p className="stats" data-testid="path-stats">
        {a ? (
          <>
            {formatNumber(a.lengthMeters, 2)} m, <strong>{formatNumber(a.totalTimeSeconds, 2)} s</strong>, top{" "}
            {formatNumber(Math.max(...a.stations.v), 2)} m/s
          </>
        ) : (
          "computing…"
        )}
      </p>
    </Section>
  );
}

export function WaypointsPanel() {
  const path = useActivePath();
  const selection = useEditor((s) => s.selection);
  const points = useEditor((s) => s.project.namedPoints);
  const selected = selection?.kind === "waypoint" ? selection.index : -1;
  const w = path.spec.waypoints[selected];
  const link = path.links[selected];
  // a linked waypoint follows its point, so moving it moves the point (and every waypoint on it)
  const moveTo = (x: number, y: number) =>
    link
      ? editProject("Move point", (p) => moveNamedPoint(p, link, x, y))
      : editPath("Move waypoint", (p) => moveWaypoint(p, selected, x, y));
  // keyboard routes to what the W tool does on the canvas
  const appendPastEnd = () => {
    const g = geometryOf(path.spec);
    const end = g?.sample(g.lengthMeters);
    const last = path.spec.waypoints.at(-1)!;
    const [dx, dy] = end ? [end.tangentX, end.tangentY] : [1, 0];
    const n = path.spec.waypoints.length;
    editPath("Add waypoint", (p) => appendWaypoint(p, round5(last.xMeters + dx), round5(last.yMeters + dy)), { kind: "waypoint", index: n });
  };
  const insertAfter = (k: number) => {
    const g = geometryOf(path.spec);
    if (!g) return;
    const mid = g.sample(g.sAtWaypoint(k + 0.5));
    editPath("Insert waypoint", (p) => insertWaypoint(p, k, 0.5, round5(mid.xMeters), round5(mid.yMeters)), { kind: "waypoint", index: k + 1 });
  };
  return (
    <Section title="Waypoints" open count={path.spec.waypoints.length}>
      <ol className="item-list" aria-label="Waypoints">
        {path.spec.waypoints.map((p, i) => (
          <li key={i}>
            <button
              type="button"
              className={i === selected ? "item selected" : "item"}
              aria-pressed={i === selected}
              onClick={() => select({ kind: "waypoint", index: i })}
            >
              <span className="idx">{i}</span>({formatNumber(p.xMeters, 2)}, {formatNumber(p.yMeters, 2)})
              {path.links[i] && <span className="tag">{points.find((q) => q.id === path.links[i])?.name}</span>}
            </button>
          </li>
        ))}
      </ol>
      <IconButton label="Add a waypoint 1 m past the end" onClick={appendPastEnd}>
        + Waypoint at the end
      </IconButton>
      {w && (
        <div className="detail" aria-label={`Waypoint ${selected}`}>
          <div className="grid2">
            <NumberField label="x" unit="m" value={w.xMeters} onCommit={(v) => moveTo(round5(v), w.yMeters)} />
            <NumberField label="y" unit="m" value={w.yMeters} onCommit={(v) => moveTo(w.xMeters, round5(v))} />
            <NumberField
              label="Tangent"
              unit="°"
              value={toDegrees(w.tangentRadians)}
              blank="auto"
              blankValue={Number.NaN}
              title="Direction of travel through this waypoint; blank = automatic"
              onCommit={(v) => editPath("Change tangent", (p) => setTangent(p, selected, Number.isNaN(v) ? v : round5Radians(v * DEGREES), w.tangentScale))}
            />
            <NumberField
              label="Tangent scale"
              value={w.tangentScale}
              min={0.01}
              title="Above 1 holds the direction longer"
              onCommit={(v) => editPath("Change tangent", (p) => setTangent(p, selected, w.tangentRadians, v))}
            />
          </div>
          <label className="field">
            <span className="field-label">Named point</span>
            <span className="field-input">
              <select
                value={path.links[selected] ?? ""}
                onChange={(e) => editProject("Link waypoint", (p) => linkToPoint(p, path.id, selected, e.target.value || null))}
              >
                <option value="">none</option>
                {points.map((p) => (
                  <option key={p.id} value={p.id}>
                    {p.name}
                  </option>
                ))}
              </select>
            </span>
          </label>
          <div className="row">
            {selected < path.spec.waypoints.length - 1 && (
              <IconButton label={`Insert a waypoint after waypoint ${selected}`} onClick={() => insertAfter(selected)}>
                Insert after
              </IconButton>
            )}
            <IconButton
              label="Delete waypoint"
              disabled={path.spec.waypoints.length <= 2}
              onClick={() => editPath("Delete waypoint", (p) => deleteWaypoint(p, selected), null)}
            >
              Delete waypoint
            </IconButton>
          </div>
        </div>
      )}
    </Section>
  );
}
