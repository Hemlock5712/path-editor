// Hooks and the position editor the panels share.

import type { PathGeometry } from "../../../../core/src/index.ts";
import type { EditorPath } from "../../model/project.ts";
import { round5, waypointAtS } from "../../model/positions.ts";
import { analysis, geometryOf } from "../../state/analysis.ts";
import { activePath, editor, type EditorState } from "../../state/editor.ts";
import { useStore } from "../../state/store.ts";
import { NumberField, SelectField } from "../controls.tsx";

export function useEditor<T>(selector: (s: EditorState) => T): T {
  return useStore(editor, selector);
}

export function useActivePath(): EditorPath {
  return useEditor(activePath);
}

export function useAnalysis() {
  const result = useStore(analysis, (a) => a.result);
  return result && result.ok ? result : null;
}

export function useGeometry(): PathGeometry | null {
  return geometryOf(useActivePath().spec);
}

/**
 * A place on the path: waypoint-relative (1.5 = halfway from waypoint 1 to 2, which stays put when
 * the path changes) or metres along it. Switching converts at the path's current shape.
 */
export function PositionField({
  label,
  waypoint,
  meters,
  onCommit,
  testId,
}: {
  label: string;
  waypoint: number;
  meters: number;
  onCommit: (waypoint: number, meters: number) => void;
  testId?: string;
}) {
  const g = useGeometry();
  const byWaypoint = !Number.isNaN(waypoint);
  const switchMode = (toWaypoint: boolean) => {
    if (toWaypoint === byWaypoint || !g) return;
    if (toWaypoint) onCommit(round5(waypointAtS(g, meters)), Number.NaN);
    else onCommit(Number.NaN, round5(g.sAtWaypoint(waypoint)));
  };
  return (
    <div className="position">
      <NumberField
        label={label}
        value={byWaypoint ? waypoint : meters}
        unit={byWaypoint ? "wp" : "m"}
        testId={testId}
        title={byWaypoint ? "Waypoint-relative: 1.5 is halfway from waypoint 1 to 2" : "Metres along the path"}
        onCommit={(v) => (byWaypoint ? onCommit(v, Number.NaN) : onCommit(Number.NaN, v))}
      />
      <SelectField
        label={`${label} measured by`}
        inline
        value={byWaypoint ? "wp" : "m"}
        options={[
          { value: "wp", label: "by waypoint" },
          { value: "m", label: "by metres" },
        ]}
        onChange={(v) => switchMode(v === "wp")}
      />
    </div>
  );
}

export const DEGREES = Math.PI / 180;

/** Radians to degrees for display (the file's unit). */
export function toDegrees(radians: number): number {
  return radians / DEGREES;
}
