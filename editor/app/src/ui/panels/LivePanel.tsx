// The live robot over NT4: the connect box, and what the path follower is doing now. The robot (pink),
// the follower's reference and its progress along the path are drawn on the field.

import { useEffect, useRef, useState } from "react";
import { robotAddress } from "../../nt4/client.ts";
import { DEFAULT_LIVE, LiveFeed, type LiveSettings } from "../../nt4/liveFeed.ts";
import { editor, setActivePath } from "../../state/editor.ts";
import { live } from "../../state/live.ts";
import { useStore } from "../../state/store.ts";
import { formatNumber, IconButton, NumberField, Section, TextField } from "../controls.tsx";

const STORAGE_KEY = "pathEditor.live2";

function loadSettings(): LiveSettings {
  try {
    return { ...DEFAULT_LIVE, ...(JSON.parse(localStorage.getItem(STORAGE_KEY) ?? "{}") as Partial<LiveSettings>) };
  } catch {
    return DEFAULT_LIVE;
  }
}

const metres = (x: number) => (Number.isFinite(x) ? `${formatNumber(x, 3)} m` : "-");

export function LivePanel() {
  const [settings, setSettings] = useState(loadSettings);
  const s = useStore(live, (l) => l);
  const paths = useStore(editor, (e) => e.project.paths);
  const feed = useRef<LiveFeed | null>(null);

  useEffect(() => () => feed.current?.stop(), []);

  const update = (change: Partial<LiveSettings>) => {
    const next = { ...settings, ...change };
    setSettings(next);
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(next));
    } catch {
      // the settings just aren't remembered
    }
  };

  const running = s.status === "connected" || s.status === "connecting";
  const tab = paths.find((p) => p.name === s.active);
  return (
    <Section title="Live robot (NT4)">
      <p className="hint">
        Shows the robot (pink), the follower's reference and its progress over the path. Read-only: it never publishes.
      </p>
      <TextField
        label="Team number or address"
        value={settings.address}
        testId="live-address"
        onCommit={(address) => update({ address })}
      />
      <p className="hint">Connects to {robotAddress(settings.address)}. Use localhost for a simulation.</p>
      <div className="grid2">
        <NumberField label="Port" decimals={0} min={1} value={settings.port} onCommit={(port) => update({ port: Math.round(port) })} />
      </div>
      <TextField label="Robot pose topic" value={settings.poseTopic} onCommit={(poseTopic) => update({ poseTopic })} />
      <div className="row">
        {running ? (
          <IconButton label="Disconnect from the robot" onClick={() => feed.current?.stop()} testId="live-disconnect">
            Disconnect
          </IconButton>
        ) : (
          <IconButton
            label="Connect to the robot"
            testId="live-connect"
            onClick={() => {
              feed.current?.stop();
              feed.current = new LiveFeed(settings);
              feed.current.start();
            }}
          >
            Connect
          </IconButton>
        )}
        <span className={`live-status ${s.status}`} role="status" data-testid="live-status">
          {s.status}
          {s.status === "connected" && (s.synced ? `, clocks synced (round trip ${formatNumber(s.rttMs, 1)} ms)` : ", syncing clocks")}
        </span>
      </div>
      {s.message && <p className="hint">{s.message}</p>}
      {s.status === "connected" && (
        <table className="telemetry" data-testid="live-telemetry">
          <tbody>
            <tr>
              <th scope="row">Active path</th>
              <td>
                {s.active || "none"}
                {tab && editor.get().activePathId !== tab.id && (
                  <>
                    {" "}
                    <button type="button" className="link" onClick={() => setActivePath(tab.id)}>
                      show it
                    </button>
                  </>
                )}
              </td>
            </tr>
            <tr>
              <th scope="row">Phase</th>
              <td>{s.phase || "-"}</td>
            </tr>
            <tr>
              <th scope="row">Progress s</th>
              <td>{metres(s.progressMeters)}</td>
            </tr>
            <tr>
              <th scope="row">Remaining</th>
              <td>{metres(s.remainingMeters)}</td>
            </tr>
            <tr>
              <th scope="row">Cross-track</th>
              <td>{metres(s.crossTrackMeters)}</td>
            </tr>
            <tr>
              <th scope="row">Target (reference)</th>
              <td>
                {s.robot?.reference
                  ? `(${formatNumber(s.robot.reference.x, 2)}, ${formatNumber(s.robot.reference.y, 2)}) ${formatNumber((s.robot.reference.heading * 180) / Math.PI, 1)}°`
                  : "-"}
              </td>
            </tr>
            <tr>
              <th scope="row">Robot pose</th>
              <td>
                {s.robot?.pose
                  ? `(${formatNumber(s.robot.pose.x, 2)}, ${formatNumber(s.robot.pose.y, 2)}), ${formatNumber(s.poseAgeMs, 0)} ms old`
                  : "-"}
              </td>
            </tr>
            <tr>
              <th scope="row">Events fired</th>
              <td data-testid="live-events">{s.eventsFired.length > 0 ? s.eventsFired.join(", ") : "-"}</td>
            </tr>
          </tbody>
        </table>
      )}
    </Section>
  );
}
