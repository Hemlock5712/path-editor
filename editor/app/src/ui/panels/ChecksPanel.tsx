// The live checks on the active path. Clicking one moves the playback cursor to it.

import { playback } from "../../state/analysis.ts";
import { Section } from "../controls.tsx";
import { atS } from "../draw.ts";
import { useAnalysis } from "./common.tsx";
import { useStore } from "../../state/store.ts";
import { analysis } from "../../state/analysis.ts";

const KIND_LABELS = {
  curvature: "Bend",
  heading: "Turn",
  bumper: "Bumpers",
  event: "Event",
  position: "Placement",
  speed: "Speed",
};

export function ChecksPanel() {
  const a = useAnalysis();
  const failure = useStore(analysis, (s) => (s.result && !s.result.ok ? s.result.error : null));
  const checks = a?.checks ?? [];
  const errors = checks.filter((c) => c.severity === "error").length;
  return (
    <Section title="Checks" open count={checks.length}>
      {failure && (
        <p className="check error" role="alert">
          The path can't be planned: {failure}
        </p>
      )}
      {a && checks.length === 0 && <p className="hint">No problems found.</p>}
      <ul className="checks" aria-label="Checks" data-testid="checks">
        {checks.map((c, i) => (
          <li key={i}>
            <button
              type="button"
              className={`check ${c.severity}`}
              onClick={() => a && playback.set({ t: atS(a.stations, a.stations.t, c.sFrom), playing: false })}
            >
              <span className="check-kind">{KIND_LABELS[c.kind]}</span> {c.message}
            </button>
          </li>
        ))}
      </ul>
      {errors > 0 && <p className="hint">{errors === 1 ? "1 error" : `${errors} errors`}: the robot would run this path, but it won't do what the plan shows.</p>}
    </Section>
  );
}
