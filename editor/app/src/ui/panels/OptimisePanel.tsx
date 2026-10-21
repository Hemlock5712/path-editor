// The waypoint optimiser: options, which waypoints may move, progress, and the result to apply.

import { fixedWaypoints } from "../../compute/optimiser.ts";
import { applyOptimise, cancelOptimise, discardOptimise, optimiser, setLock, startOptimise } from "../../state/optimiser.ts";
import { useStore } from "../../state/store.ts";
import { formatNumber, IconButton, NumberField, Section } from "../controls.tsx";
import { useActivePath } from "./common.tsx";

const STOPPED = {
  evaluations: "used its evaluation budget",
  time: "used its time budget",
  converged: "converged",
  cancelled: "cancelled",
};

export function OptimisePanel() {
  const path = useActivePath();
  const o = useStore(optimiser, (s) => s);
  const locks = o.locks[path.id] ?? [];
  // what the optimiser keeps fixed whatever the user says: the ends and zone anchors, and named points
  const forced = fixedWaypoints(path.spec, []);
  const reason = (i: number) =>
    i === 0 || i === path.spec.waypoints.length - 1
      ? "the path's ends stay put"
      : path.links[i] !== null
        ? "linked to a named point"
        : forced[i]
          ? "a zone starts or ends here"
          : null;
  const running = o.status === "running";
  const forThisPath = o.pathId === path.id;
  const saved = o.beforeSeconds - o.bestSeconds;
  return (
    <Section title="Optimise waypoints">
      <p className="hint">
        Moves the free waypoints (within the corridor) and their tangent scales to shorten the planned time, keeping the bumpers clear of
        obstacles and the heading checks as they are. Events and zones keep their places.
      </p>
      <div className="grid2">
        <NumberField label="Corridor" unit="m" min={0.01} value={o.corridorMeters} onCommit={(v) => optimiser.set({ corridorMeters: v })} />
        <NumberField label="Time budget" unit="s" min={0.5} value={o.timeBudgetSeconds} onCommit={(v) => optimiser.set({ timeBudgetSeconds: v })} />
        <NumberField label="Seed" decimals={0} value={o.seed} onCommit={(v) => optimiser.set({ seed: Math.round(v) })} />
      </div>
      <label className="row">
        <input type="checkbox" checked={o.optimiseScales} onChange={() => optimiser.set({ optimiseScales: !o.optimiseScales })} /> Tangent scales too
      </label>
      <fieldset className="card">
        <legend>Lock waypoints</legend>
        <div className="locks">
          {path.spec.waypoints.map((_, i) => {
            const why = reason(i);
            return (
              <label key={i} title={why ?? "keep this waypoint where it is"}>
                <input type="checkbox" checked={why !== null || locks[i] === true} disabled={why !== null || running} onChange={(e) => setLock(path.id, i, e.target.checked)} />{" "}
                {i}
              </label>
            );
          })}
        </div>
      </fieldset>
      <div className="row">
        {running ? (
          <IconButton label="Stop the optimiser and keep the best so far" onClick={cancelOptimise} testId="optimise-cancel">
            Cancel
          </IconButton>
        ) : (
          <IconButton label="Optimise this path's waypoints" onClick={startOptimise} testId="optimise-start">
            Optimise
          </IconButton>
        )}
      </div>
      {forThisPath && o.status !== "idle" && (
        <div role="status" className="stats" data-testid="optimise-status">
          {o.status === "error" && <span className="field-error">{o.error}</span>}
          {(running || o.status === "done") && Number.isFinite(o.bestSeconds) && (
            <>
              {running ? `Running: ${o.evaluations} plans tried. ` : `Done: ${STOPPED[o.result?.stoppedBy ?? "converged"]}, ${o.evaluations} plans tried. `}
              {formatNumber(o.beforeSeconds, 3)} s → <strong>{formatNumber(o.bestSeconds, 3)} s</strong>
              {saved > 1e-9 ? ` (${formatNumber(saved, 3)} s, ${formatNumber((100 * saved) / o.beforeSeconds, 1)} % faster)` : ""}
            </>
          )}
        </div>
      )}
      {forThisPath && o.status === "done" && o.result && (
        <div className="row">
          {o.result.afterSeconds < o.result.beforeSeconds ? (
            <>
              <IconButton label="Apply the optimised waypoints (one undo step)" onClick={applyOptimise} testId="optimise-apply">
                Apply
              </IconButton>
              <IconButton label="Discard the optimised waypoints" onClick={discardOptimise}>
                Discard
              </IconButton>
            </>
          ) : (
            <>
              <span className="hint">No faster path within the corridor.</span>
              <IconButton label="Close the optimiser result" onClick={discardOptimise}>
                OK
              </IconButton>
            </>
          )}
        </div>
      )}
    </Section>
  );
}
