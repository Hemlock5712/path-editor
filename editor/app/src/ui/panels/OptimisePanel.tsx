// The optimiser, the main way to shape a path: the Optimise path button with its progress and result
// (in the Path panel), and its settings with which waypoints stay put (a section of their own).

import { whyFixed } from "../../compute/optimiser.ts";
import { setPinned } from "../../model/edits.ts";
import { editPath } from "../../state/editor.ts";
import { applyOptimise, cancelOptimise, discardOptimise, optimiser, startOptimise } from "../../state/optimiser.ts";
import { useStore } from "../../state/store.ts";
import { formatNumber, IconButton, NumberField, Section } from "../controls.tsx";
import { useActivePath } from "./common.tsx";

const STOPPED = {
  evaluations: "used its budget",
  time: "stopped by the clock",
  converged: "converged",
  cancelled: "stopped",
};

/** Optimise path, its progress, and Apply / Discard (the field shows the new shape dashed meanwhile). */
export function OptimiseControls() {
  const path = useActivePath();
  const o = useStore(optimiser, (s) => s);
  const running = o.status === "running";
  const forThisPath = o.pathId === path.id;
  const saved = o.beforeSeconds - o.bestSeconds;
  const locked = path.spec.waypoints.map((_, i) => path.pinned[i] === true || path.links[i] !== null);
  const free = whyFixed(path.spec, locked, o.segment).filter((w) => w === null).length;
  return (
    <div className="optimise">
      <div className="row">
        {running && forThisPath ? (
          <IconButton label="Stop the optimiser and keep the best so far" onClick={cancelOptimise} testId="optimise-cancel">
            Stop
          </IconButton>
        ) : (
          <button
            type="button"
            className="primary"
            onClick={startOptimise}
            disabled={free === 0 || (forThisPath && o.status === "done")}
            title={
              free === 0
                ? "Nothing to move: only pass-through points that aren't pinned move"
                : `Move the ${free} pass-through point${free === 1 ? "" : "s"} and their tangents to shorten the planned time (${formatNumber(o.budgetSeconds, 1)} s budget)`
            }
            data-testid="optimise-path"
          >
            Optimise path
          </button>
        )}
      </div>
      {forThisPath && o.status !== "idle" && (
        <div role="status" className="stats" data-testid="optimise-status">
          {o.status === "error" && <span className="field-error">{o.error}</span>}
          {(running || o.status === "done") && Number.isFinite(o.bestSeconds) && (
            <>
              {formatNumber(o.beforeSeconds, 2)} s → <strong>{formatNumber(o.bestSeconds, 2)} s</strong>
              {saved > 1e-9 ? ` (${formatNumber((100 * saved) / o.beforeSeconds, 1)} % faster)` : ""}
              {running ? `, ${o.evaluations} plans tried…` : `. ${STOPPED[o.result?.stoppedBy ?? "converged"]}.`}
            </>
          )}
        </div>
      )}
      {forThisPath && o.status === "done" && o.result && (
        <div className="row">
          {o.result.afterSeconds < o.result.beforeSeconds ? (
            <>
              <button type="button" className="primary" onClick={applyOptimise} title="Use the dashed path (one undo step)" data-testid="optimise-apply">
                Apply
              </button>
              <IconButton label="Keep the path as it was" onClick={discardOptimise} testId="optimise-discard">
                Discard
              </IconButton>
            </>
          ) : (
            <>
              <span className="hint">No faster path within the corridor.</span>
              <IconButton label="Close the optimiser result" onClick={discardOptimise} testId="optimise-discard">
                OK
              </IconButton>
            </>
          )}
        </div>
      )}
    </div>
  );
}

export function OptimisePanel() {
  const path = useActivePath();
  const o = useStore(optimiser, (s) => s);
  const running = o.status === "running";
  const linked = path.spec.waypoints.map((_, i) => path.links[i] !== null);
  const why = whyFixed(path.spec, linked, o.segment);
  const n = path.spec.waypoints.length;
  const setSegment = (from: number, to: number) =>
    optimiser.set({ segment: Number.isNaN(from) || Number.isNaN(to) ? null : [Math.round(from), Math.round(to)] });
  return (
    <Section title="Optimiser settings">
      <p className="hint">
        Optimise path moves the pass-through points (within the corridor) and their tangent scales to shorten the planned time, keeping the
        bumpers clear of obstacles and the heading checks as they are. The start, the end, points with a set heading and pinned points stay
        put. The budget sets how many plans it tries, so the same budget and seed give the same path every time.
      </p>
      <div className="grid2">
        <NumberField label="Corridor" unit="m" min={0.01} value={o.corridorMeters} onCommit={(v) => optimiser.set({ corridorMeters: v })} />
        <NumberField label="Budget" unit="s" min={0.2} value={o.budgetSeconds} onCommit={(v) => optimiser.set({ budgetSeconds: v })} />
        <NumberField label="Seed" decimals={0} value={o.seed} onCommit={(v) => optimiser.set({ seed: Math.round(v) })} />
      </div>
      <label className="row">
        <input type="checkbox" checked={o.optimiseScales} onChange={() => optimiser.set({ optimiseScales: !o.optimiseScales })} /> Tangent scales too
      </label>
      <div className="grid2" title="Only move the waypoints between these two; blank = the whole path">
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
      <fieldset className="card">
        <legend>Pinned waypoints</legend>
        <div className="locks">
          {path.spec.waypoints.map((_, i) => {
            const pinned = path.pinned[i] === true;
            // (linked waypoints come back as locked: say why)
            const reason = why[i] === "pinned" ? "linked to a named point" : why[i]!;
            return (
              <label key={i} title={reason ?? (pinned ? "pinned: the optimiser leaves it" : "pin it to keep it where it is")}>
                <input
                  type="checkbox"
                  checked={pinned || reason !== null}
                  disabled={reason !== null || running}
                  onChange={(e) => editPath(e.target.checked ? "Pin" : "Unpin", (p) => setPinned(p, i, e.target.checked))}
                />{" "}
                {i}
              </label>
            );
          })}
        </div>
      </fieldset>
    </Section>
  );
}
