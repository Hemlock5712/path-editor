// The optimiser, the main way to shape a path: the Optimise path button with its progress and result
// (in the Path panel), and its settings: whether it may move waypoints, and which are locked.

import { CHECKS } from "../../compute/analysis.ts";
import { blockedMessage, whatMoves, whyFixed } from "../../compute/optimiser.ts";
import { setPinned } from "../../model/edits.ts";
import { isDrawn } from "../../model/project.ts";
import { editPath } from "../../state/editor.ts";
import { addSuggestedWaypoint, applyOptimise, cancelOptimise, discardOptimise, optimiser, startOptimise, startRouteAround } from "../../state/optimiser.ts";
import { useStore } from "../../state/store.ts";
import { Lock, LockOpen, Plus, Route, WandSparkles } from "lucide-react";
import { formatNumber, NumberField, Switch } from "../controls.tsx";
import { useActivePath, useAnalysis } from "./common.tsx";

const STOPPED = {
  evaluations: "Used its budget",
  time: "Stopped by the clock",
  converged: "Converged",
  cancelled: "Stopped",
};

const cm = (meters: number) => `${formatNumber(Math.max(0, 100 * meters), 0)} cm`;

/** How close the path's swept bumpers come to the obstacles and the wall (the optimiser's hard rule). */
function Clearance() {
  const a = useAnalysis();
  if (!a) return null;
  const c = a.minClearanceMeters;
  const margin = CHECKS.bumperMarginMeters;
  const text = !Number.isFinite(c)
    ? "No obstacles on this field."
    : c >= margin
      ? `Clears obstacles by ${cm(c)}.`
      : c > 0
        ? `Only ${cm(c)} from an obstacle (it needs ${cm(margin)}): Optimise reshapes the path clear if it can.`
        : "The bumpers hit an obstacle (red on the path): Optimise reshapes the path clear if it can.";
  return (
    <p className={`hint clearance${c < margin ? " bad" : ""}`} data-testid="optimise-clearance">
      {text}
    </p>
  );
}

/** Optimise path, its progress, and Apply / Discard (the field shows the new shape dashed meanwhile). */
export function OptimiseControls() {
  const path = useActivePath();
  const o = useStore(optimiser, (s) => s);
  const r = forResult(o);
  const running = o.status === "running";
  const forThisPath = o.pathId === path.id;
  const saved = o.beforeSeconds - o.bestSeconds;
  const locked = path.spec.waypoints.map((_, i) => path.pinned[i] === true || path.links[i] !== null);
  const options = { moveWaypoints: o.moveWaypoints, corridorMeters: o.corridorMeters, locked, segment: o.segment };
  // the curve can always be reshaped once there is one: never greyed out for a path with an end
  const drawn = isDrawn(path.spec);
  const blocked = forThisPath && r && !r.clear ? r : null;
  const a = useAnalysis();
  const hits = drawn && a !== null && a.checks.some((c) => c.kind === "bumper");
  // a route is worth applying once it's clear, faster or not; a plain run only when it changed something
  const applicable = o.result !== null && (o.routed ? o.result.clear : o.result.changed);
  return (
    <div className="optimise">
      <div className="row">
        {running && forThisPath ? (
          <button type="button" className="btn big block" onClick={cancelOptimise} data-tip="Stop and keep the best so far" data-testid="optimise-cancel">
            Stop
          </button>
        ) : (
          <button
            type="button"
            className="btn primary big block"
            onClick={() => startOptimise()}
            disabled={!drawn}
            data-tip={
              drawn
                ? `${whatMoves(path.spec, options)} It keeps the bumpers ${cm(CHECKS.bumperMarginMeters)} clear of obstacles and the wall (${formatNumber(o.budgetSeconds, 1)} s budget).`
                : "Draw the path first: double-click the field for its start and end"
            }
            data-testid="optimise-path"
          >
            <WandSparkles size={16} />
            Optimise path
          </button>
        )}
      </div>
      <Clearance />
      <div className="row">
        <span
          className="block"
          data-tip={
            hits
              ? "Plan a way round the first obstacle the path hits with the robot's own pathfinder, put its corners in as pass-through points, then reshape the curve. Shown dashed until you Apply."
              : "Nothing to route around: the path clears every obstacle by 5 cm"
          }
        >
          <button type="button" className="btn block" disabled={!hits || running} onClick={startRouteAround} data-testid="route-around">
            <Route size={16} />
            Route around obstacles
          </button>
        </span>
      </div>
      {forThisPath && o.status !== "idle" && (
        <div role="status" className="stats" data-testid="optimise-status">
          {o.routed && (
            <span className="route-note" data-testid="route-note">
              Route around adds {o.routed.added} waypoint{o.routed.added === 1 ? "" : "s"}
              {o.routed.removed > 0 ? ` (replacing ${o.routed.removed} pass-through point${o.routed.removed === 1 ? "" : "s"})` : ""}.{" "}
            </span>
          )}
          {o.status === "error" && <span className="field-error">{o.error}</span>}
          {(running || o.status === "done") && Number.isFinite(o.bestSeconds) && !blocked && (
            <>
              {formatNumber(o.beforeSeconds, 2)} s → <strong>{formatNumber(o.bestSeconds, 2)} s</strong>
              {saved > 1e-9 ? ` (${formatNumber((100 * saved) / o.beforeSeconds, 1)} % faster)` : ""}
              {saved < -1e-9 ? ` (${formatNumber((-100 * saved) / o.beforeSeconds, 1)} % slower, to get clear)` : ""}
              {running ? `, ${o.evaluations} plans tried…` : `. ${STOPPED[o.result?.stoppedBy ?? "converged"]}.`}
              {r && r.changed && ` Clears obstacles by ${cm(r.clearanceMeters)}.`}
            </>
          )}
          {blocked && (
            <span className="field-error" role="alert">
              {blockedMessage(blocked, options)} It's marked on the field.
            </span>
          )}
        </div>
      )}
      {forThisPath && o.status === "done" && o.result && (
        <div className="row">
          {applicable ? (
            <>
              <button
                type="button"
                className="btn primary"
                onClick={applyOptimise}
                data-tip={o.routed ? "Use the dashed route, its new points and all (one undo step)" : "Use the dashed path (one undo step)"}
                data-testid="optimise-apply"
              >
                Apply
              </button>
              <button type="button" className="btn ghost" onClick={discardOptimise} data-tip="Keep the path as it was" data-testid="optimise-discard">
                Discard
              </button>
            </>
          ) : blocked?.blockedAt ? (
            <>
              <button
                type="button"
                className="btn"
                onClick={addSuggestedWaypoint}
                data-tip={`A pass-through point at (${formatNumber(blocked.blockedAt.suggestX, 2)}, ${formatNumber(blocked.blockedAt.suggestY, 2)}), after waypoint ${blocked.blockedAt.fromWaypoint}; then Optimise again`}
                data-testid="optimise-add-point"
              >
                <Plus size={14} />
                Add a waypoint here
              </button>
              <button type="button" className="btn ghost" onClick={discardOptimise} data-testid="optimise-discard">
                OK
              </button>
            </>
          ) : (
            <>
              <span className="hint">{o.result.clear ? "No faster curve found." : "The path is unchanged."}</span>
              <button type="button" className="btn ghost" onClick={discardOptimise} data-testid="optimise-discard">
                OK
              </button>
            </>
          )}
        </div>
      )}
    </div>
  );
}

function forResult(o: ReturnType<typeof optimiser.get>) {
  return o.status === "done" ? o.result : null;
}

export function OptimiserSettings() {
  const path = useActivePath();
  const o = useStore(optimiser, (s) => s);
  const running = o.status === "running";
  const linked = path.spec.waypoints.map((_, i) => path.links[i] !== null);
  const why = whyFixed(path.spec, linked, o.segment);
  const n = path.spec.waypoints.length;
  const setSegment = (from: number, to: number) =>
    optimiser.set({ segment: Number.isNaN(from) || Number.isNaN(to) ? null : [Math.round(from), Math.round(to)] });
  return (
    <>
      <p className="hint">
        Optimise path reshapes the curve to shorten the planned time: the tangent's direction and scale at every waypoint, the ends included.
        The speeds follow the curve (it slows where it bends harder), and that is also how it clears a wall or an obstacle. Waypoints and
        headings stay exactly where they are. It keeps the bumpers at least {cm(CHECKS.bumperMarginMeters)} clear of obstacles and the wall: a
        path that hits one is reshaped clear first, even if that makes it slower, and if it can't be, it says where and changes nothing. The
        budget sets how many plans it tries, so the same budget and seed give the same path every time.
      </p>
      <Switch
        label="Also move unlocked waypoints"
        checked={o.moveWaypoints}
        onChange={(on) => optimiser.set({ moveWaypoints: on })}
        testId="optimise-move-waypoints"
        title="Let it move the interior waypoints that aren't locked, within the corridor, keeping their headings. The start, the end, locked points, named points and zone anchors never move."
      />
      <div className="grid2">
        {o.moveWaypoints && (
          <NumberField label="Corridor" unit="m" min={0.01} value={o.corridorMeters} onCommit={(v) => optimiser.set({ corridorMeters: v })} />
        )}
        <NumberField label="Budget" unit="s" min={0.2} value={o.budgetSeconds} onCommit={(v) => optimiser.set({ budgetSeconds: v })} />
        <NumberField label="Seed" decimals={0} value={o.seed} onCommit={(v) => optimiser.set({ seed: Math.round(v) })} />
      </div>
      <Switch label="Tangent scales too" checked={o.optimiseScales} onChange={(on) => optimiser.set({ optimiseScales: on })} />
      <div className="grid2" data-tip="Only change the path between these two waypoints; blank = the whole path">
        <NumberField
          label="Only between waypoint"
          decimals={0}
          blank="start"
          blankValue={Number.NaN}
          value={o.segment ? o.segment[0] : Number.NaN}
          onCommit={(v) => setSegment(v, o.segment ? o.segment[1] : n - 1)}
        />
        <NumberField
          label="and waypoint"
          decimals={0}
          blank="end"
          blankValue={Number.NaN}
          value={o.segment ? o.segment[1] : Number.NaN}
          onCommit={(v) => setSegment(o.segment ? o.segment[0] : 0, v)}
        />
      </div>
      <div className="field">
        <span className="field-label">Locked waypoints</span>
        <div className="locks" role="group" aria-label="Locked waypoints">
          {path.spec.waypoints.map((_, i) => {
            const isLocked = path.pinned[i] === true;
            // (linked waypoints come back as locked: say why)
            const reason = why[i] === "locked" ? "linked to a named point" : why[i]!;
            return (
              <button
                key={i}
                type="button"
                className="pin-chip"
                aria-pressed={isLocked || reason !== null}
                aria-label={`Lock waypoint ${i}`}
                disabled={reason !== null || running}
                data-tip={reason ?? (isLocked ? "Locked: the optimiser never moves it" : "Lock it so the optimiser never moves it")}
                onClick={() => editPath(isLocked ? "Unlock" : "Lock", (p) => setPinned(p, i, !isLocked))}
              >
                {isLocked || reason !== null ? <Lock size={11} /> : <LockOpen size={11} />}
                {i}
              </button>
            );
          })}
        </div>
      </div>
    </>
  );
}
