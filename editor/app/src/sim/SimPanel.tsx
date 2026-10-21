// The sim preview's controls and readouts: a bar under the playback (on/off and the headline numbers),
// a sidebar section (the shove, cross-track over time, where each event fired) and the drivetrain's
// numbers in the Robot panel.

import { useId, type MouseEvent } from "react";
import { playback } from "../state/analysis.ts";
import { useStore } from "../state/store.ts";
import { formatNumber, NumberField, Section } from "../ui/controls.tsx";
import { SIM_COLOURS } from "./draw.ts";
import { OVERRUN_SECONDS, type SimPreview } from "./preview.ts";
import { DEFAULT_SIM_SETTINGS, sim, simSettings, type SimSettings } from "./state.ts";
import "./sim.css";

const set = (change: Partial<SimSettings>) => simSettings.set(change);

function signed(seconds: number): string {
  return `${seconds >= 0 ? "+" : "−"}${formatNumber(Math.abs(seconds), 2)} s`;
}

function cm(meters: number): string {
  return `${formatNumber(meters * 100, 1)} cm`;
}

function useSimResult() {
  const enabled = useStore(simSettings, (s) => s.enabled);
  const result = useStore(sim, (s) => s.result);
  return { enabled, result };
}

/** Under the playback: switch the preview on, and its headline numbers. */
export function SimBar() {
  const { enabled, result } = useSimResult();
  return (
    <div className="sim-bar" role="group" aria-label="Sim preview">
      <label className="sim-toggle">
        <input type="checkbox" checked={enabled} data-testid="sim-toggle" onChange={(e) => set({ enabled: e.target.checked })} />
        Sim preview
      </label>
      {enabled && !result && <span className="sim-muted">simulating…</span>}
      {enabled && result && !result.ok && <span className="sim-error">Can't simulate: {result.error}</span>}
      {enabled && result?.ok && <Headline p={result} />}
      {!enabled && <span className="sim-muted">Runs the follower on a simulated drivetrain (Robot panel) to show how the robot tracks the plan.</span>}
    </div>
  );
}

function Headline({ p }: { p: SimPreview }) {
  const finished = Number.isFinite(p.finishSeconds);
  return (
    <span className="sim-numbers">
      <span data-testid="sim-time" title="When the follower finished, against the plan's total time">
        {finished ? (
          <>
            Sim <strong>{formatNumber(p.finishSeconds, 2)} s</strong> vs plan {formatNumber(p.planSeconds, 2)} s ({signed(p.gapSeconds)})
          </>
        ) : (
          <span className="sim-error">Didn't finish within {formatNumber(p.planSeconds + OVERRUN_SECONDS, 1)} s</span>
        )}
      </span>
      {Number.isFinite(p.settleSeconds) && (
        <span title="Time in the terminal phase: from the hand-over to the goal pose until finished">settle {formatNumber(p.settleSeconds, 2)} s</span>
      )}
      <span title="The furthest the robot got from the path, sideways">
        cross-track max <strong>{cm(p.maxCrossTrackMeters)}</strong>
      </span>
    </span>
  );
}

const W = 300;
const H = 96;
const PAD = { left: 30, right: 6, top: 8, bottom: 14 };

/** Cross-track error over time; click to move the playback there. */
function CrossTrackChart({ p }: { p: SimPreview }) {
  const titleId = useId();
  const t = useStore(playback, (s) => s.t);
  const tEnd = p.t.at(-1) || 1;
  const yMax = Math.max(0.01, p.maxCrossTrackMeters) * 1.1;
  const px = (x: number) => PAD.left + (x / tEnd) * (W - PAD.left - PAD.right);
  const py = (y: number) => PAD.top + (0.5 - y / (2 * yMax)) * (H - PAD.top - PAD.bottom);
  const points = p.t.map((x, i) => `${px(x).toFixed(1)},${py(p.crossTrack[i]!).toFixed(1)}`).join(" ");
  const seek = (e: MouseEvent<SVGSVGElement>) => {
    const r = e.currentTarget.getBoundingClientRect();
    const x = ((e.clientX - r.left) / r.width) * W;
    playback.set({ t: Math.max(0, Math.min(tEnd, ((x - PAD.left) / (W - PAD.left - PAD.right)) * tEnd)), playing: false });
  };
  const vline = (x: number, cls: string) =>
    Number.isFinite(x) && <line key={cls} className={cls} x1={px(x)} x2={px(x)} y1={PAD.top} y2={H - PAD.bottom} />;
  return (
    <svg className="sim-chart" viewBox={`0 0 ${W} ${H}`} role="img" aria-labelledby={titleId} onClick={seek}>
      <title id={titleId}>Cross-track error over time; dashed grey: the plan's end, orange: finished, pink: the shove</title>
      <rect className="chart-frame" x={PAD.left} y={PAD.top} width={W - PAD.left - PAD.right} height={H - PAD.top - PAD.bottom} />
      <line className="chart-zero" x1={PAD.left} x2={W - PAD.right} y1={py(0)} y2={py(0)} />
      {vline(p.planSeconds, "sim-plan-end")}
      {vline(p.finishSeconds, "sim-finish")}
      {p.shove && vline(p.shove.atSeconds, "sim-shove")}
      {vline(Math.min(t, tEnd), "sim-cursor")}
      <polyline points={points} fill="none" stroke={SIM_COLOURS.trace} strokeWidth={1.5} />
      <text className="chart-axis" x={PAD.left - 3} y={py(yMax) + 6} textAnchor="end">
        {formatNumber(yMax * 100, 1)}
      </text>
      <text className="chart-axis" x={PAD.left - 3} y={py(-yMax)} textAnchor="end">
        −{formatNumber(yMax * 100, 1)}
      </text>
      <text className="chart-axis" x={PAD.left - 3} y={H - 3} textAnchor="end">
        cm
      </text>
      <text className="chart-axis" x={W - PAD.right} y={H - 3} textAnchor="end">
        {formatNumber(tEnd, 1)} s
      </text>
    </svg>
  );
}

/** The sidebar section: the shove, cross-track error and the events. */
export function SimPanel() {
  const { enabled, result } = useSimResult();
  const s = useStore(simSettings, (x) => x);
  return (
    <Section title="Sim preview" open={enabled}>
      <p className="hint">
        The robot's follower, ported, on a simulated drivetrain (latency and lag are in the Robot panel). It assumes the
        floor grips: slip sensing and adaptive grip run only on the robot.
      </p>
      <label className="sim-toggle">
        <input type="checkbox" checked={s.shoveOn} data-testid="sim-shove" onChange={(e) => set({ shoveOn: e.target.checked })} />
        Shove the robot
      </label>
      {s.shoveOn && (
        <div className="grid2">
          <NumberField label="At" unit="s" min={0} value={s.shoveAtSeconds} onCommit={(v) => set({ shoveAtSeconds: v })} />
          <span />
          <NumberField label="Push x" unit="m/s" value={s.shoveDvx} title="Velocity added along the field's x (blue frame)" onCommit={(v) => set({ shoveDvx: v })} />
          <NumberField label="Push y" unit="m/s" value={s.shoveDvy} title="Velocity added along the field's y (blue frame)" onCommit={(v) => set({ shoveDvy: v })} />
        </div>
      )}
      {!enabled && <p className="hint">Switch on Sim preview under the playback to run it.</p>}
      {enabled && result?.ok && <SimDetails p={result} />}
    </Section>
  );
}

function SimDetails({ p }: { p: SimPreview }) {
  return (
    <>
      <CrossTrackChart p={p} />
      <p className="stats">
        Cross-track RMS {cm(p.rmsCrossTrackMeters)}, max <strong>{cm(p.maxCrossTrackMeters)}</strong>. At the end:{" "}
        {cm(p.endErrorMeters)} and {formatNumber((Math.abs(p.endHeadingErrorRadians) * 180) / Math.PI, 1)}° off the goal.
      </p>
      {p.events.length > 0 && (
        <table className="telemetry sim-events" data-testid="sim-events">
          <thead>
            <tr>
              <th scope="col">Event</th>
              <th scope="col" title="When the plan reaches it">
                Plan
              </th>
              <th scope="col" title="When it fired in the sim">
                Sim
              </th>
              <th scope="col" title="How far the robot was from the authored point when it fired">
                Off by
              </th>
            </tr>
          </thead>
          <tbody>
            {p.events.map((e, i) => (
              <tr key={i}>
                <th scope="row">{e.name}</th>
                <td>{formatNumber(e.planT, 2)} s</td>
                <td>{e.loop >= 0 ? `${formatNumber(e.t, 2)} s` : "didn't fire"}</td>
                <td>{e.loop >= 0 ? cm(e.offsetMeters) : ""}</td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
    </>
  );
}

/** The simulated drivetrain's numbers, for the Robot panel. Kept in this browser, not in robot.json. */
export function SimDriveFields() {
  const s = useStore(simSettings, (x) => x);
  return (
    <>
      <p className="hint">Sim preview drivetrain: the editor's own, not saved in robot.json.</p>
      <div className="grid2">
        <NumberField
          label="Command latency"
          unit="s"
          min={0}
          value={s.latencySeconds}
          title="From a command being sent to the modules acting on it"
          onCommit={(v) => set({ latencySeconds: v })}
        />
        <NumberField label="Velocity lag" unit="s" min={0.001} value={s.lagSeconds} title="Time constant of the chassis velocity's first-order lag" onCommit={(v) => set({ lagSeconds: v })} />
        <NumberField
          label="Floor grip μ"
          blank="no cap"
          blankValue={Infinity}
          min={0.05}
          value={s.gripMu}
          title="Caps the simulated chassis's acceleration at μ·g; blank for none"
          onCommit={(v) => set({ gripMu: v })}
        />
        <button
          type="button"
          onClick={() => set({ latencySeconds: DEFAULT_SIM_SETTINGS.latencySeconds, lagSeconds: DEFAULT_SIM_SETTINGS.lagSeconds, gripMu: DEFAULT_SIM_SETTINGS.gripMu })}
        >
          Sim defaults
        </button>
      </div>
    </>
  );
}
