// The first-run quick start: how to draw a swerve path in six lines. Shown over the field until it is
// dismissed (remembered in this browser), and always at the top of the help dialog.

import { createStore, useStore } from "../state/store.ts";

export const QUICK_START: readonly string[] = [
  "Draw (W): click to add points; click the path to insert one. Enter when done.",
  "Every point is a robot pose: drag the knob on its ring to turn it (15° steps; Alt turns freely).",
  "Points without a heading are pass-through: the robot blends between the set ones.",
  "Drag the path to insert a point; right-click a point to set its heading, pin it or delete it.",
  "Heading (R): click the path to turn the robot partway along it.",
  "Optimise path shapes the curve for speed; pin the points that must not move.",
];

const SEEN_KEY = "pathEditor.quickStartSeen";

function seen(): boolean {
  try {
    return localStorage.getItem(SEEN_KEY) === "1";
  } catch {
    return false;
  }
}

/** The quick-start list, for the overlay and the help dialog. */
export function QuickStartList() {
  return (
    <ol className="quick-start-list">
      {QUICK_START.map((line) => (
        <li key={line}>{line}</li>
      ))}
    </ol>
  );
}

const overlay = createStore({ open: !seen() });

/** Shows the quick start over the field again (Settings, Editor). */
export function showQuickStart(): void {
  overlay.set({ open: true });
}

/** The first-run card over the field; "Got it" hides it for good (Settings or ? show it again). */
export function QuickStartOverlay() {
  const open = useStore(overlay, (s) => s.open);
  if (!open) return null;
  const dismiss = () => {
    try {
      localStorage.setItem(SEEN_KEY, "1");
    } catch {
      // private mode: it shows again next time
    }
    overlay.set({ open: false });
  };
  return (
    <div className="quick-start" role="dialog" aria-label="Quick start" data-testid="quick-start">
      <h2>Quick start</h2>
      <QuickStartList />
      <button type="button" className="btn primary" onClick={dismiss} data-testid="quick-start-dismiss">
        Got it
      </button>
    </div>
  );
}
