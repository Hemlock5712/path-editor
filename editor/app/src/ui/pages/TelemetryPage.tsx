// Telemetry: the NT4 connection to the robot (or a simulation) and what the path follower is doing now.
// The connection stays up while you edit; the editor's field shows the live robot when the overlay is on.

import { Plug, Radio, Unplug } from "lucide-react";
import { useEffect, useState } from "react";
import { robotAddress } from "../../nt4/client.ts";
import { DEFAULT_LIVE, LiveFeed, type LiveSettings } from "../../nt4/liveFeed.ts";
import { localAccess, type LocalAccess } from "../../nt4/localAccess.ts";
import { editor, setActivePath } from "../../state/editor.ts";
import { live } from "../../state/live.ts";
import { useStore } from "../../state/store.ts";
import { setPage, ui } from "../../state/ui.ts";
import { Button, formatNumber, NumberField, Switch, TextField } from "../controls.tsx";

const STORAGE_KEY = "pathEditor.live2";

function loadSettings(): LiveSettings {
  try {
    return { ...DEFAULT_LIVE, ...(JSON.parse(localStorage.getItem(STORAGE_KEY) ?? "{}") as Partial<LiveSettings>) };
  } catch {
    return DEFAULT_LIVE;
  }
}

/** One connection for the whole app: it outlives the page. */
let feed: LiveFeed | null = null;

function connect(settings: LiveSettings): void {
  feed?.stop();
  feed = new LiveFeed(settings);
  feed.start();
}

function disconnect(): void {
  feed?.stop();
  feed = null;
}

const metres = (x: number) => (Number.isFinite(x) ? `${formatNumber(x, 3)} m` : "-");

function Tile({ label, value, testId }: { label: string; value: string; testId?: string }) {
  return (
    <div className="tile">
      <div className="label">{label}</div>
      <div className="value" data-testid={testId}>
        {value}
      </div>
    </div>
  );
}

export function TelemetryPage() {
  const [settings, setSettings] = useState(loadSettings);
  const s = useStore(live, (l) => l);
  const overlay = useStore(ui, (u) => u.liveOverlay);
  const paths = useStore(editor, (e) => e.project.paths);

  const update = (change: Partial<LiveSettings>) => {
    const next = { ...settings, ...change };
    setSettings(next);
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(next));
    } catch {
      // the settings just aren't remembered
    }
  };

  // from the https site, Local Network Access stands between the page and the robot: say what to do
  const [access, setAccess] = useState<LocalAccess>({ state: "fine", message: "" });
  const host = robotAddress(settings.address);
  useEffect(() => {
    let current = true;
    void localAccess(host).then((a) => current && setAccess(a));
    return () => {
      current = false;
    };
  }, [host, s.status]);

  const running = s.status === "connected" || s.status === "connecting";
  const tab = paths.find((p) => p.name === s.active);
  const ref = s.robot?.reference;
  const pose = s.robot?.pose;
  return (
    <div className="telemetry-page">
      <aside className="telemetry-side" aria-label="Connection">
        <h2>Telemetry</h2>
        <p className="hint">The robot (or a simulation) over NT4. Read-only: it never publishes.</p>
        <TextField label="Team number or address" value={settings.address} testId="live-address" onCommit={(address) => update({ address })} />
        <p className="hint">Connects to {host}. Use localhost for a simulation.</p>
        {access.state !== "fine" && (
          <p className={`check ${access.state === "ask" ? "warning" : "error"}`} data-testid="live-https">
            {access.message}
          </p>
        )}
        <div className="grid2">
          <NumberField label="Port" decimals={0} min={1} value={settings.port} onCommit={(port) => update({ port: Math.round(port) })} />
        </div>
        <TextField label="Robot pose topic" value={settings.poseTopic} onCommit={(poseTopic) => update({ poseTopic })} />
        <div className="row">
          {running ? (
            <Button onClick={disconnect} testId="live-disconnect">
              <Unplug size={14} />
              Disconnect
            </Button>
          ) : (
            <Button variant="primary" onClick={() => connect(settings)} testId="live-connect">
              <Plug size={14} />
              Connect
            </Button>
          )}
          <span className={`live-status ${s.status}`} role="status" data-testid="live-status">
            {s.status}
            {s.status === "connected" && (s.synced ? `, clocks synced (${formatNumber(s.rttMs, 1)} ms)` : ", syncing clocks")}
          </span>
        </div>
        {s.message && <p className="hint">{s.message}</p>}
        <Switch label="Show the live robot on the editor's field" checked={overlay} onChange={(on) => ui.set({ liveOverlay: on })} testId="live-overlay-switch" />
      </aside>
      <main className="telemetry-main" aria-label="Live data">
        {s.status !== "connected" ? (
          <div className="empty-state">
            <Radio size={40} />
            <h3>Not connected</h3>
            <p>Connect to the robot or a simulation to see the follower: the active path, its progress, cross-track error, and the events it fires.</p>
          </div>
        ) : (
          <>
            <div className="tiles" data-testid="live-telemetry">
              <Tile label="Active path" value={s.active || "none"} />
              <Tile label="Phase" value={s.phase || "-"} />
              <Tile label="Progress" value={metres(s.progressMeters)} />
              <Tile label="Remaining" value={metres(s.remainingMeters)} />
              <Tile label="Cross-track" value={metres(s.crossTrackMeters)} />
              <Tile label="Pose age" value={Number.isFinite(s.poseAgeMs) ? `${formatNumber(s.poseAgeMs, 0)} ms` : "-"} />
            </div>
            <table className="telemetry">
              <tbody>
                <tr>
                  <th scope="row">Target (reference)</th>
                  <td>{ref ? `(${formatNumber(ref.x, 2)}, ${formatNumber(ref.y, 2)}) ${formatNumber((ref.heading * 180) / Math.PI, 1)}°` : "-"}</td>
                </tr>
                <tr>
                  <th scope="row">Robot pose</th>
                  <td>{pose ? `(${formatNumber(pose.x, 2)}, ${formatNumber(pose.y, 2)}) ${formatNumber((pose.heading * 180) / Math.PI, 1)}°` : "-"}</td>
                </tr>
                <tr>
                  <th scope="row">Events fired</th>
                  <td data-testid="live-events">{s.eventsFired.length > 0 ? s.eventsFired.join(", ") : "-"}</td>
                </tr>
              </tbody>
            </table>
            {tab && (
              <div>
                <Button
                  onClick={() => {
                    setActivePath(tab.id);
                    ui.set({ liveOverlay: true });
                    setPage("editor");
                  }}
                >
                  Show {tab.name} in the editor with the live robot
                </Button>
              </div>
            )}
          </>
        )}
      </main>
    </div>
  );
}
