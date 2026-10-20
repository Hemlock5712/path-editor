// Named events on the active path. Names come from the project's existing ones (so a typo doesn't
// make a second event the robot code never binds), and the times are the plan's, from the core.

import { makePathEvent } from "../../../../core/src/index.ts";
import { removeEvent, updateEvent } from "../../model/edits.ts";
import { eventNames } from "../../model/project.ts";
import { playback } from "../../state/analysis.ts";
import { editPath, select, setTool } from "../../state/editor.ts";
import { IconButton, Section, TextField, formatNumber } from "../controls.tsx";
import { PositionField, useActivePath, useAnalysis, useEditor } from "./common.tsx";

export function EventsPanel() {
  const path = useActivePath();
  const project = useEditor((s) => s.project);
  const selection = useEditor((s) => s.selection);
  const a = useAnalysis();
  const names = eventNames(project);
  const events = path.spec.events;
  return (
    <Section title="Events" open count={events.length}>
      <p className="hint">
        The E tool places an event where you click on the path. Robot code binds each name once with{" "}
        <code>PathLibrary.bindEvent</code>.
      </p>
      {events.map((e, i) => {
        const sel = selection?.kind === "event" && selection.index === i;
        const placed = a?.events[i];
        return (
          <fieldset key={i} className={sel ? "card selected" : "card"} data-testid={`event-${i}`} onFocus={() => !sel && select({ kind: "event", index: i })}>
            <legend>{e.name}</legend>
            <TextField
              label="Name"
              value={e.name}
              options={names}
              testId={`event-name-${i}`}
              autoFocus={sel && e.name === "event"}
              onCommit={(name) => editPath("Rename event", (p) => updateEvent(p, i, makePathEvent(name, e.waypoint, e.meters)))}
            />
            <PositionField
              label="At"
              waypoint={e.waypoint}
              meters={e.meters}
              testId={`event-at-${i}`}
              onCommit={(w, m) => editPath("Move event", (p) => updateEvent(p, i, makePathEvent(e.name, w, m)))}
            />
            {placed && (
              <p className="stats">
                {formatNumber(placed.s, 2)} m along, fires at{" "}
                <button type="button" className="link" onClick={() => playback.set({ t: placed.t, playing: false })}>
                  {formatNumber(placed.t, 2)} s
                </button>
              </p>
            )}
            <IconButton label={`Delete event ${e.name}`} onClick={() => editPath("Delete event", (p) => removeEvent(p, i), null)}>
              Delete
            </IconButton>
          </fieldset>
        );
      })}
      <IconButton label="Place an event on the path" onClick={() => setTool("event")}>
        + Event (E, then click the path)
      </IconButton>
    </Section>
  );
}
