// The first-run quick start: how to draw a swerve path in six lines. Shown over the field until it is
// dismissed (remembered in this browser), and always at the top of the help dialog.

import { createStore, useStore } from "../state/store.ts";

export const QUICK_START: readonly string[] = [
  "Double-click the field to place the start, then the end. Select a point and double-click to add one after it.",
  "Every point is a robot pose: drag the knob on its ring to turn it (15° steps; Alt turns freely).",
  "Points without a heading are pass-through: the robot blends between the set ones.",
  "Drag the path to insert a point; right-click a point to set its heading, lock it or delete it.",
  "Add in a few clicks: R heading, M event marker, Z constraint zone, T point-towards zone, N named point (Esc cancels). Q and E turn the selected point 5°; L measures lengths.",
  "Optimise path reshapes the curves and speeds and leaves your points where they are; lock a point to keep it put even when it may move them.",
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
