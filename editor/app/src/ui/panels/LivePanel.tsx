// The live robot over NT4: connection settings and the follower's telemetry values.

import { useEffect, useRef, useState } from "react";
import { DEFAULT_LIVE, Nt4Client, type LiveSettings } from "../../nt4/client.ts";
import { live } from "../../state/live.ts";
import { useStore } from "../../state/store.ts";
import { IconButton, Section, TextField } from "../controls.tsx";

const STORAGE_KEY = "pathEditor.live";

function loadSettings(): LiveSettings {
  try {
    return { ...DEFAULT_LIVE, ...(JSON.parse(localStorage.getItem(STORAGE_KEY) ?? "{}") as Partial<LiveSettings>) };
  } catch {
    return DEFAULT_LIVE;
  }
}

export function LivePanel() {
  const [settings, setSettings] = useState(loadSettings);
  const status = useStore(live, (s) => s.status);
  const message = useStore(live, (s) => s.message);
  const telemetry = useStore(live, (s) => s.telemetry);
  const client = useRef<Nt4Client | null>(null);

  useEffect(() => () => client.current?.disconnect(), []);

  const update = (change: Partial<LiveSettings>) => {
    const next = { ...settings, ...change };
    setSettings(next);
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(next));
    } catch {
      // settings just won't be remembered
    }
  };

  const connected = status === "connected" || status === "connecting";
  const keys = Object.keys(telemetry).sort();
  return (
    <Section title="Live robot (NT4)">
      <p className="hint">
        Shows the robot (pink) and the follower's reference over the path. Read-only. Not yet tried on a robot.
      </p>
      <TextField label="Robot address" value={settings.host} onCommit={(host) => update({ host })} />
      <TextField label="Pose topic" value={settings.poseTopic} onCommit={(poseTopic) => update({ poseTopic })} />
      <TextField label="Telemetry prefix" value={settings.telemetryPrefix} onCommit={(telemetryPrefix) => update({ telemetryPrefix })} />
      <div className="row">
        {connected ? (
          <IconButton label="Disconnect from the robot" onClick={() => client.current?.disconnect()}>
            Disconnect
          </IconButton>
        ) : (
          <IconButton
            label="Connect to the robot"
            onClick={() => {
              client.current?.disconnect();
              client.current = new Nt4Client(settings);
              client.current.connect();
            }}
          >
            Connect
          </IconButton>
        )}
        <span className={`live-status ${status}`} role="status">
          {status}
          {message && `: ${message}`}
        </span>
      </div>
      {keys.length > 0 && (
        <table className="telemetry">
          <tbody>
            {keys.map((k) => (
              <tr key={k}>
                <th scope="row">{k}</th>
                <td>{typeof telemetry[k] === "number" ? (telemetry[k] as number).toFixed(3) : String(telemetry[k])}</td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
    </Section>
  );
}
