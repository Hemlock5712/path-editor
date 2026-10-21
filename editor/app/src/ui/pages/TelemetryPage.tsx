// Telemetry: how well the robot tracks its path. The NT4 connection (it stays up on the other pages),
// every run recorded loop by loop, and for the chosen run: the field (plan, where it should be, where it
// is, the trail coloured by error, events), a summary, and graphs over time or over the path, all on one
// cursor that follows the live run or scrubs and replays a recorded one.

import { Download, Pause, Play, Plug, Radio, Unplug, Upload } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import { robotAddress } from "../../nt4/client.ts";
import { DEFAULT_LIVE, type LiveSettings } from "../../nt4/liveFeed.ts";
import { localAccess, type LocalAccess } from "../../nt4/localAccess.ts";
import { exportRun, summarise, type RecordedRun } from "../../nt4/recorder.ts";
import { live } from "../../state/live.ts";
import { useStore } from "../../state/store.ts";
import { addImported, connect, disconnect, followLive, selectedRun, selectRun, setCursor, telemetry } from "../../state/telemetry.ts";
import { toast, ui } from "../../state/ui.ts";
import { Button, formatNumber, IconButton, NumberField, Segmented, Slider, Switch, TextField } from "../controls.tsx";
import { TrackingCharts } from "../telemetry/TrackingCharts.tsx";
import { TrackingField } from "../telemetry/TrackingField.tsx";

const STORAGE_KEY = "pathEditor.live2";

function loadSettings(): LiveSettings {
  try {
    return { ...DEFAULT_LIVE, ...(JSON.parse(localStorage.getItem(STORAGE_KEY) ?? "{}") as Partial<LiveSettings>) };
  } catch {
    return DEFAULT_LIVE;
  }
}

const cm = (m: number, d = 1) => (Number.isFinite(m) ? `${formatNumber(m * 100, d)} cm` : "-");

function Connection() {
  const [settings, setSettings] = useState(loadSettings);
  const s = useStore(live, (l) => l);
  const overlay = useStore(ui, (u) => u.liveOverlay);
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
  return (
    <>
      <h2>Telemetry</h2>
      <p className="hint">The robot (or a simulation) over NT4. Read-only: it never publishes. Every run of the path follower is recorded.</p>
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
    </>
  );
}

function download(name: string, text: string): void {
  const url = URL.createObjectURL(new Blob([text], { type: "application/json" }));
  const a = document.createElement("a");
  a.href = url;
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function RunList() {
  const runs = useStore(telemetry, (s) => s.runs);
  useStore(telemetry, (s) => s.version);
  const selectedId = useStore(telemetry, (s) => s.selectedId);
  const file = useRef<HTMLInputElement>(null);
  const selected = runs.find((r) => r.id === selectedId) ?? null;
  return (
    <div className="field">
      <span className="field-label">Runs (the last 10, newest first)</span>
      <ul className="item-list" aria-label="Recorded runs" data-testid="run-list">
        {[...runs].reverse().map((r) => {
          const sum = summarise(r);
          return (
            <li key={r.id}>
              <button type="button" className="item" aria-pressed={r.id === selectedId} onClick={() => selectRun(r.id)} data-testid="run-item">
                <span className={`run-dot ${r.finished ? "done" : "live"}`} aria-hidden="true" />
                <span className="main">
                  {r.path} <span className="heading-tag">{new Date(r.startedAt).toLocaleTimeString()}</span>
                </span>
                <span className="heading-tag">{Number.isFinite(sum.seconds) ? `${formatNumber(sum.seconds, 1)} s` : ""}</span>
              </button>
            </li>
          );
        })}
        {runs.length === 0 && <li className="hint">None yet: runs appear here as the robot follows paths.</li>}
      </ul>
      <div className="row">
        <Button variant="ghost" disabled={!selected} onClick={() => selected && download(`${selected.path}-${new Date(selected.startedAt).toISOString().slice(0, 19).replace(/[:T]/g, "-")}.run.json`, exportRun(selected))} testId="run-export">
          <Download size={14} />
          Export
        </Button>
        <Button variant="ghost" onClick={() => file.current?.click()} testId="run-import-button">
          <Upload size={14} />
          Import
        </Button>
        <input
          ref={file}
          type="file"
          accept=".json,application/json"
          className="sr-only"
          aria-label="Import a recorded run"
          data-testid="run-import"
          onChange={async (e) => {
            const f = e.target.files?.[0];
            e.target.value = "";
            if (!f) return;
            try {
              addImported(await f.text());
            } catch (err) {
              toast(`${f.name}: ${(err as Error).message}`, "error");
            }
          }}
        />
      </div>
    </div>
  );
}

function Tile({ label, value, sub, testId }: { label: string; value: string; sub?: string; testId?: string }) {
  return (
    <div className="tile">
      <div className="label">{label}</div>
      <div className="value" data-testid={testId}>
        {value}
      </div>
      {sub && <div className="hint">{sub}</div>}
    </div>
  );
}

function Summary({ run }: { run: RecordedRun }) {
  useStore(telemetry, (s) => s.version);
  const sum = summarise(run);
  const missed = sum.eventsExpected.filter((e) => !sum.eventsFired.includes(e));
  const late = sum.seconds - sum.planSeconds;
  return (
    <div className="tiles" data-testid="run-summary">
      <Tile label="Cross-track RMS" value={cm(sum.crossTrackRms, 2)} sub={`max ${cm(sum.crossTrackMax, 1)}`} testId="summary-ct-rms" />
      <Tile label="Final position" value={run.finished ? cm(sum.endError, 1) : "running…"} sub={run.finished ? `heading ${formatNumber((sum.endHeadingError * 180) / Math.PI, 2)}°` : undefined} />
      <Tile
        label="Time against plan"
        value={Number.isFinite(sum.seconds) ? `${formatNumber(sum.seconds, 2)} s` : "-"}
        sub={
          !Number.isFinite(sum.planSeconds)
            ? "no plan received"
            : run.finished
              ? `plan ${formatNumber(sum.planSeconds, 2)} s (${late >= 0 ? "+" : ""}${formatNumber(late, 2)} s, with settling)`
              : `plan ${formatNumber(sum.planSeconds, 2)} s; running`
        }
      />
      <Tile
        label="Events"
        value={`${sum.eventsFired.length}/${sum.eventsExpected.length || sum.eventsFired.length}`}
        sub={missed.length ? `not fired: ${missed.join(", ")}` : sum.eventsFired.join(", ") || "none on this path"}
        testId="summary-events"
      />
    </div>
  );
}

/** Replay: moves the cursor through the run at real speed. */
function useReplay(run: RecordedRun | null): void {
  const playing = useStore(telemetry, (s) => s.playing);
  useEffect(() => {
    if (!playing || !run) return;
    let frame = 0;
    let last = performance.now();
    const end = run.samples.at(-1)?.t ?? 0;
    const tick = (now: number) => {
      const t = telemetry.get().cursor + (now - last) / 1000;
      last = now;
      if (t >= end) {
        telemetry.set({ cursor: end, playing: false });
        return;
      }
      telemetry.set({ cursor: t });
      frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frame);
  }, [playing, run]);
}

export function TelemetryPage() {
  const status = useStore(live, (l) => l.status);
  const s = useStore(telemetry, (x) => x);
  const run = selectedRun(s);
  useReplay(run);
  const end = run?.samples.at(-1)?.t ?? 0;
  return (
    <div className="telemetry-page">
      <aside className="telemetry-side" aria-label="Connection and runs">
        <Connection />
        <RunList />
      </aside>
      <main className="telemetry-main" aria-label="Tracking">
        {!run ? (
          <div className="empty-state">
            <Radio size={40} />
            <h3>{status === "connected" ? "Waiting for a run" : "Not connected"}</h3>
            <p>
              {status === "connected"
                ? "Each time the robot follows a path it is recorded here: the path, where it should be against where it is, and graphs of how well it tracked."
                : "Connect to the robot or a simulation, or import a recorded run, to see how well it tracks its paths."}
            </p>
          </div>
        ) : (
          <>
            <div className="tracking-head">
              <h2>
                {run.path} <span className="tag">{run.finished ? "recorded" : "live"}</span>
              </h2>
              <Segmented
                label="Graphs against"
                value={s.axis}
                options={[
                  { value: "t", label: "Time" },
                  { value: "s", label: "Distance along" },
                ]}
                onChange={(axis) => telemetry.set({ axis })}
              />
            </div>
            <div className="tracking-stage">
              <TrackingField />
              <div className="playback-bar" role="group" aria-label="Replay">
                <IconButton label={s.playing ? "Pause" : "Replay"} onClick={() => telemetry.set({ playing: !s.playing, cursor: s.playing || s.cursor < end ? s.cursor : 0, followLive: false })} testId="replay">
                  {s.playing ? <Pause size={16} /> : <Play size={16} />}
                </IconButton>
                <Slider label="Run time" value={Math.min(s.cursor, end)} min={0} max={end || 1} step={0.02} valueText={`${formatNumber(s.cursor, 2)} s`} onChange={setCursor} testId="run-scrub" />
                <span className="time">
                  {formatNumber(Math.min(s.cursor, end), 2)} / <strong>{formatNumber(end, 2)} s</strong>
                </span>
                <Button variant={s.followLive ? "primary" : "ghost"} onClick={followLive} className="small" title="Follow the newest run as it happens">
                  Live
                </Button>
              </div>
              <div className="tracking-legend" aria-hidden="true">
                <span>
                  <i className="lg-ghost" /> where it should be
                </span>
                <span>
                  <i className="lg-robot" /> where it is
                </span>
                <span>
                  <i className="lg-trail" /> trail: 0 → 10 cm off
                </span>
              </div>
            </div>
            <Summary run={run} />
            <TrackingCharts run={run} />
          </>
        )}
      </main>
    </div>
  );
}
